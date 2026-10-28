// Curve parameters are convenient for formulas but bad for layout: equal steps of parameter
// are not equal steps along the curve. Arc length converts between the two
// (Engine/Math/Curves/ArcLength.swift).
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include "../Linear/MathProblem.h"
#include "../Numerics/Integration.h"
#include "../Linear/Scalar.h"
#include "Curve.h"

namespace editor {

/// Settings for the arc-length functions.
///
/// Use when: tuning how accurately curve lengths are measured
/// Tags: arc length settings, integration steps, length accuracy, curve length
struct ArcLength {
    /// How many Gauss–Legendre pieces are used per unit of curve parameter. A cubic Bézier
    /// (range 0…1) is integrated in 8 pieces of 5 samples each; a Catmull–Rom spline in 8
    /// per point-to-point piece. The pieces sit on a fixed grid of multiples of 1/8, so
    /// every whole parameter is a piece border. The speed of a cubic is smooth, so this is
    /// far more than enough: the error is dominated by Float rounding, about 1e-6 of the length, unless the curve has a
    /// near-cusp (speed close to zero), where the speed has a sharp dip and accuracy drops.
    static constexpr int piecesPerUnitParameter = 8;

    /// How close `parameter(atDistance:)` must get before it stops, as a fraction of the
    /// length being searched. Relative, because Float cannot resolve 1e-6 world units on a
    /// curve 100 m long.
    static constexpr float relativeDistanceTolerance = 1e-6f;

    /// Safety limit for the search. Bisection alone halves the interval each step, so 60
    /// steps shrink any interval below Float resolution.
    static constexpr int maximumIterations = 60;
};

namespace curve_detail {

/// Swift's `Float.ulp` for a finite number: the gap between it and the next float of larger
/// magnitude (for the largest float, the gap below it, as Swift reports).
inline float unitInTheLastPlace(float value) {
    float magnitude = std::fabs(value);
    float next = std::nextafter(magnitude, std::numeric_limits<float>::infinity());
    if (std::isinf(next)) return magnitude - std::nextafter(magnitude, 0.0f);
    return next - magnitude;
}

} // namespace curve_detail

/// The length of the curve between two parameters, measured along the curve.
///
/// How: length is the integral of speed, `∫ |P′(parameter)| d parameter` from `start` to
/// `end`, evaluated with 5-point Gauss–Legendre quadrature on small pieces
/// (`ArcLength.piecesPerUnitParameter` per unit of parameter), cut at multiples of
/// 1/`piecesPerUnitParameter`.
///
///     ●━━━━━━━━●━━━━━━━●        the sum of the lengths of tiny
///     start    ← length →  end  straight steps, in the limit
///
/// Conventions: world units. If `end` ≤ `start` the result is 0. Accuracy: see
/// `ArcLength.piecesPerUnitParameter`. Parameters must be finite (Swift traps there; here a
/// `MathProblem` is thrown, because the piece index cannot be computed).
///
/// Source: Guenter & Parent (1990), "Computing the arc length of parametric curves";
/// Gauss–Legendre: Abramowitz & Stegun 25.4.29.
///
/// Use when: measuring how long a road, rope or river stretch is; placing a marker a
///   known distance along a path
/// Tags: arc length, curve length, path length, length between points, polyline
///   length, spline length
/// Node: inside curveLength, resampleCurve, curveFrames
template <typename Derived, CurvePoint PointType>
float Curve<Derived, PointType>::length(float start, float end) const {
    if (!(end > start)) return 0;
    if (!std::isfinite(start) || !std::isfinite(end)) throw MathProblem("Arc length needs finite parameters.");
    float cellsPerUnit = static_cast<float>(ArcLength::piecesPerUnitParameter);
    std::int64_t firstCell = static_cast<std::int64_t>(std::floor(start * cellsPerUnit));
    std::int64_t lastCell = static_cast<std::int64_t>(std::ceil(end * cellsPerUnit));
    float total = 0;
    for (std::int64_t cell = firstCell; cell < lastCell; cell += 1) {
        // Cell borders sit on a fixed grid, so they always fall on whole parameters,
        // where a spline may change speed abruptly. Integrating across such a jump would
        // lose accuracy.
        float cellStart = largerOf(start, static_cast<float>(cell) / cellsPerUnit);
        float cellEnd = smallerOf(end, static_cast<float>(cell + 1) / cellsPerUnit);
        total += GaussLegendre::integral(
            [this](float parameter) { return derived().firstDerivative(parameter).length(); },
            cellStart, cellEnd);
    }
    return total;
}

/// The length of the whole curve, from the start of `parameterRange` to its end.
///
/// Use when: sizing a texture along a road; counting how many planks fit along a path
/// Tags: total length, curve length, path length, perimeter, spline length
/// Node: curveLength
template <typename Derived, CurvePoint PointType>
float Curve<Derived, PointType>::totalLength() const {
    return length(derived().parameterRange().lowerBound, derived().parameterRange().upperBound);
}

/// The parameter reached after walking `distance` along the curve from the start.
/// This is the inverse of `length(from:to:)`.
///
/// How: find the parameter whose length from the start equals `distance`, by solving
/// `length(start → parameter) − distance = 0` with Newton's method, for which the slope
/// is the speed `|P′(parameter)|`:
///
///     parameter ← parameter − (length(start → parameter) − distance) / speed
///
/// Newton converges in a few steps on a smooth curve, but can jump out of range where
/// the speed is tiny. So the search keeps a bracket known to contain the answer
/// (length is increasing, so a negative error moves the lower end up, a positive one
/// the upper end down) and falls back to bisection whenever a Newton step leaves it.
///
/// Conventions: `distance` is clamped to 0…`totalLength`, so you get the start or end
/// parameter outside that. Accuracy: within `ArcLength.relativeDistanceTolerance` of the
/// length, plus the accuracy of `length`; together about 1e-5 of the length.
/// Parameters must be finite (NaN or infinity traps when the piece index is computed).
///
/// Source: Guenter & Parent (1990) use the same inversion idea with a lookup table; we
/// solve directly because the curves here are cheap and the answer needs no table.
///
/// Use when: walking an object along a road at constant speed; placing a mark a set
///   distance down a path
/// Tags: distance along curve, arc length parameterization, constant speed, walk
///   along path, inverse arc length
/// Node: inside resampleCurve, curveFrames
template <typename Derived, CurvePoint PointType>
float Curve<Derived, PointType>::parameterAtDistance(float distance) const {
    return parameterAtDistance(distance, derived().parameterRange().lowerBound);
}

/// The parameter reached after walking `distance` along the curve from `startParameter`.
/// Same method and conventions as `parameter(atDistance:)`, with the walk starting
/// part-way along; the answer is clamped to `startParameter…parameterRange.upperBound`.
/// A start at or past the end of the range is returned unchanged.
///
/// Use when: moving an object on from where it is by a set distance; laying out
///   repeated pieces one after another on a path
/// Tags: distance along curve, advance along path, step along spline, parameter from
///   distance, arc length
/// Node: inside resampleCurve, curveFrames
template <typename Derived, CurvePoint PointType>
float Curve<Derived, PointType>::parameterAtDistance(float distance, float startParameter) const {
    float upperBound = derived().parameterRange().upperBound;
    if (!(distance > 0 && startParameter < upperBound)) return startParameter;
    float remainingLength = length(startParameter, upperBound);
    if (!(distance < remainingLength)) return upperBound;

    float lowerBracket = startParameter;
    float upperBracket = upperBound;
    // The guess assumes constant speed, which is exact for straight, evenly paced curves.
    float parameter = linearBlend(startParameter, upperBound, distance / remainingLength);

    float distanceTolerance = ArcLength::relativeDistanceTolerance * remainingLength;
    // The bracket cannot shrink below the gap between neighbouring Floats at its top end.
    float parameterResolution = curve_detail::unitInTheLastPlace(
        largerOf(std::fabs(startParameter), std::fabs(upperBound)));
    for (int iteration = 0; iteration < ArcLength::maximumIterations; iteration += 1) {
        float error = length(startParameter, parameter) - distance;
        if (isNearlyZero(error, distanceTolerance)) break;
        if (error < 0) {
            lowerBracket = parameter;
        } else {
            upperBracket = parameter;
        }

        float speed = derived().firstDerivative(parameter).length();
        float newtonGuess = parameter - error / speed;
        bool isInsideBracket = newtonGuess > lowerBracket && newtonGuess < upperBracket;
        parameter = (speed > 0 && isInsideBracket) ? newtonGuess : linearBlend(lowerBracket, upperBracket, 0.5f);
        if (upperBracket - lowerBracket <= parameterResolution) break;
    }
    return parameter;
}

/// `count` parameters whose points are evenly spaced along the curve, from its start to
/// its end inclusive. This is what a sweep needs: samples at equal distances, so a road
/// surface or a row of fence posts is not bunched up where the parameter moves slowly.
///
/// How: with `step = totalLength / (count − 1)`, sample `i` is the parameter reached
/// after walking one `step` from sample `i − 1` (see `parameter(atDistance:startingAt:)`).
/// Walking from the previous sample, not from the start, keeps each search short. The
/// first and last parameters are the exact range ends.
///
/// Conventions: needs `count` ≥ 2. Spacing is equal along the curve, so the straight-line
/// gaps between samples are slightly shorter on bends. Each sample can be off by about
/// 1e-5 world units (see `parameter(atDistance:)`).
///
/// Use when: placing fence posts, lamp posts or road dashes at equal gaps; resampling
///   a spline
/// Tags: even spacing, equidistant points, resample curve, arc length spacing,
///   uniform sampling
/// Node: inside resampleCurve, curveFrames
template <typename Derived, CurvePoint PointType>
std::vector<float> Curve<Derived, PointType>::evenlySpacedParameters(int count) const {
    if (!(count >= 2)) throw MathProblem("Evenly spaced samples need at least a start and an end.");
    float step = totalLength() / static_cast<float>(count - 1);
    std::vector<float> parameters = {derived().parameterRange().lowerBound};
    for (int sample = 1; sample < count - 1; sample += 1) {
        parameters.push_back(parameterAtDistance(step, parameters.back()));
    }
    parameters.push_back(derived().parameterRange().upperBound);
    return parameters;
}

/// The points of the curve at `evenlySpacedParameters(count:)`.
///
/// Use when: turning a spline into evenly spaced points for a road mesh or a polyline
/// Tags: resample, equidistant points, even samples, curve to polyline, evenly
///   spaced, sample curve
/// Node: none (shorthand for evenlySpacedParameters then point(at:), which resampleCurve does to read radii too)
template <typename Derived, CurvePoint PointType>
std::vector<PointType> Curve<Derived, PointType>::evenlySpacedPoints(int count) const {
    std::vector<PointType> points;
    for (float parameter : evenlySpacedParameters(count)) points.push_back(derived().point(parameter));
    return points;
}

} // namespace editor
