// Reading a flat cubic at a parameter and cutting it by length: every level of the de Casteljau
// construction, a lookup table, the unit tangent and normal, the osculating circle, and splitting
// at shares of arc length. Ported from graphics_cpp's (bezier.cpp: casteljau, lut,
// tangent, normal, osculating; geometry.cpp: cutAt, splitEven), after Pomax, "A Primer on Bézier
// Curves" (https://pomax.github.io/bezierinfo).
#include <algorithm>
#include <cmath>
#include <limits>

#include "../Linear/MathProblem.h"
#include "CubicBezier.h"

namespace editor {

/// The de Casteljau construction at `parameter`: the four control points, the three points
/// between them, the two between those, and the point on the curve, in that order (Bezier.js's
/// "hull").
///
/// How: each level blends neighbours by the parameter, Qᵢ = Pᵢ + (Pᵢ₊₁ − Pᵢ)·t.
///
///     index: 0 1 2 3 | 4 5 6 | 7 8 | 9
///            P0…P3     level 1   level 2  point on the curve
///
/// Conventions: a parameter outside 0…1 extrapolates.
///
/// Source: de Casteljau (1959); Pomax, "A Primer on Bézier Curves".
///
/// Use when: drawing the construction lines of a curve; reading the handles of the two halves
///   of a split (indices 0, 4, 7, 9 and 9, 8, 6, 3)
/// Tags: de Casteljau, hull, construction, subdivision points, Bezier levels
/// Node: planned (casteljauPoints)
template <CurvePoint PointType>
std::array<Vector2, 10> CubicBezier<PointType>::deCasteljauPoints(float parameter) const
    requires std::same_as<PointType, Vector2>
{
    Vector2 firstOfLevelOne = Vector2::linearBlend(start, startHandle, parameter);
    Vector2 secondOfLevelOne = Vector2::linearBlend(startHandle, endHandle, parameter);
    Vector2 thirdOfLevelOne = Vector2::linearBlend(endHandle, end, parameter);
    Vector2 firstOfLevelTwo = Vector2::linearBlend(firstOfLevelOne, secondOfLevelOne, parameter);
    Vector2 secondOfLevelTwo = Vector2::linearBlend(secondOfLevelOne, thirdOfLevelOne, parameter);
    return {start, startHandle, endHandle, end, firstOfLevelOne, secondOfLevelOne, thirdOfLevelOne,
            firstOfLevelTwo, secondOfLevelTwo, Vector2::linearBlend(firstOfLevelTwo, secondOfLevelTwo, parameter)};
}

/// `steps + 1` points along the curve at even steps of its parameter: a lookup table.
///
/// How: point(i / steps) for i = 0…steps. Even steps of the parameter are not even steps of
/// length; use `evenlySpacedPoints` for that.
///
/// Conventions: `steps` is at least 1 or the call throws a `MathProblem`.
///
/// Use when: drawing a polyline of a curve; finding the nearest sample to a point before refining
/// Tags: lookup table, LUT, sample curve, polyline, points along a Bezier
/// Node: planned (sampleCurve)
template <CurvePoint PointType>
std::vector<Vector2> CubicBezier<PointType>::lookupTable(int steps) const
    requires std::same_as<PointType, Vector2>
{
    if (!(steps >= 1)) throw MathProblem("A lookup table needs at least one step.");
    std::vector<Vector2> table;
    for (int index = 0; index <= steps; index += 1) {
        table.push_back(point(static_cast<float>(index) / static_cast<float>(steps)));
    }
    return table;
}

/// The unit direction the curve runs in at `parameter`.
///
/// How: the first derivative, normalised. Where it vanishes (a handle on its node, only at an
/// end) the direction to the next control point is used: P2 − P0 near the start (parameter < ½),
/// P3 − P1 near the end, and P3 − P0 if that is zero too. All zero gives (0, 0).
///
/// Conventions: y up; the direction follows increasing parameter.
///
/// Use when: placing an arrow or a car along a curve; the direction a path leaves a point
/// Tags: tangent, unit tangent, direction at t, curve heading
/// Node: planned (curveFrameAt)
template <CurvePoint PointType>
Vector2 CubicBezier<PointType>::tangentDirection(float parameter) const
    requires std::same_as<PointType, Vector2>
{
    constexpr float vanishing = 1e-12f;
    Vector2 velocity = firstDerivative(parameter);
    if (velocity.length() > vanishing) return velocity.normalized();
    Vector2 nextControl = parameter < 0.5f ? endHandle - start : end - startHandle;
    if (nextControl.length() > vanishing) return nextControl.normalized();
    return (end - start).normalized();
}

/// The unit normal at `parameter`: the tangent turned a quarter turn counter-clockwise, so it
/// points to the left of the direction of travel (y up).
///
/// Use when: offsetting a point sideways from a curve; the side a curve bends to
/// Tags: normal, unit normal, perpendicular to curve, left normal
/// Node: planned (curveFrameAt)
template <CurvePoint PointType>
Vector2 CubicBezier<PointType>::unitNormal(float parameter) const
    requires std::same_as<PointType, Vector2>
{
    return tangentDirection(parameter).perpendicular();
}

/// The circle that fits the curve best at `parameter`: touching it there with the same tangent
/// and curvature.
///
/// How: with signed curvature κ (`signedCurvature`), the radius is 1/|κ| and the centre is the
/// curve point plus the unit normal times 1/κ, which puts it on the side the curve bends to.
///
///        ╭───●───╮    the circle touches the curve at ●,
///     ●──╯   │   ╰──● sharing its tangent and bend
///            ○ centre
///
/// Conventions: where |κ| < 1e-12 the curve runs straight: no centre, infinite radius, 0
/// curvature.
///
/// Source: do Carmo, *Differential Geometry of Curves and Surfaces* (1976), evolute and
/// osculating circle.
///
/// Use when: rounding a corner to match a curve's bend; reading how tightly a curve turns
/// Tags: osculating circle, circle of curvature, centre of curvature, radius of curvature
/// Node: planned (osculatingCircle)
template <CurvePoint PointType>
OsculatingCircle CubicBezier<PointType>::osculatingCircle(float parameter) const
    requires std::same_as<PointType, Vector2>
{
    float curvature = signedCurvature(parameter);
    if (std::fabs(curvature) < 1e-12f) return OsculatingCircle{std::nullopt, std::numeric_limits<float>::infinity(), 0};
    return OsculatingCircle{point(parameter) + unitNormal(parameter) / curvature, std::fabs(1 / curvature), curvature};
}

/// The curve cut at shares of its length (ascending, 0…1 each): one piece per gap, so n shares
/// give n + 1 pieces.
///
/// How: the control points are scaled by `metric` (what a unit across and up are worth), the
/// scaled curve's total length found, and each share times that total turned into a parameter by
/// `parameterAtDistance`; the pieces are `portion`s of the original between neighbouring
/// parameters. So the lengths are shares of length in the metric, while the pieces are in the
/// curve's own units.
///
/// Conventions: a share outside 0…1 or out of order throws a `MathProblem`; no shares gives the
/// whole curve as one piece.
///
/// Use when: dashing a curve at set lengths; cutting a road at its quarter points
/// Tags: cut curve, split at length, split at distance, dash, arc length cut
/// Node: planned (splitCurveAtLength)
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::cutAtLengthShares(
    const std::vector<float>& shares, Vector2 metric) const
    requires std::same_as<PointType, Vector2>
{
    CubicBezier<Vector2> inMetric(start * metric, startHandle * metric, endHandle * metric, end * metric);
    float totalLength = inMetric.totalLength();
    std::vector<float> cuts{0};
    float previousShare = 0;
    for (float share : shares) {
        if (!(share >= previousShare && share <= 1)) {
            throw MathProblem("Shares of length must ascend and lie in 0…1.");
        }
        previousShare = share;
        // Keep the parameters from running backwards through rounding.
        cuts.push_back(std::max(cuts.back(), inMetric.parameterAtDistance(share * totalLength)));
    }
    cuts.push_back(1);
    std::vector<CubicBezier<Vector2>> pieces;
    for (size_t index = 1; index < cuts.size(); index += 1) pieces.push_back(portion(cuts[index - 1], cuts[index]));
    return pieces;
}

/// The curve cut into `count` pieces of equal length.
///
/// How: `evenlySpacedParameters(count + 1)` gives the parameters at equal steps of arc length;
/// the pieces are the `portion`s between neighbours.
///
/// Conventions: `count` is at least 1 or the call throws a `MathProblem`.
///
/// Use when: splitting a road into equal segments; placing equally spaced joints
/// Tags: split even, equal length pieces, divide curve, subdivide by length
/// Node: planned (splitCurveEvenly)
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::splitIntoEqualLengths(int count) const
    requires std::same_as<PointType, Vector2>
{
    if (!(count >= 1)) throw MathProblem("A curve is cut into at least one piece.");
    std::vector<float> parameters = this->evenlySpacedParameters(count + 1);
    std::vector<CubicBezier<Vector2>> pieces;
    for (int index = 0; index < count; index += 1) pieces.push_back(portion(parameters[index], parameters[index + 1]));
    return pieces;
}

/// The curve cut into `count` pieces at even steps of its parameter (graphics_cpp's `splitEven`):
/// pieces of equal parameter span, not of equal length.
///
/// How: repeatedly split the rest at 1 / (pieces left), which lands the cuts at i / count.
///
/// Conventions: `count` is at least 1 or the call throws a `MathProblem`.
///
/// Use when: subdividing a curve cheaply, where equal length does not matter
/// Tags: split even parameter, subdivide curve, uniform parameter split
/// Node: planned (splitCurveEvenly)
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::splitAtEvenParameters(int count) const
    requires std::same_as<PointType, Vector2>
{
    if (!(count >= 1)) throw MathProblem("A curve is cut into at least one piece.");
    std::vector<CubicBezier<Vector2>> pieces;
    CubicBezier<Vector2> rest = *this;
    for (int remaining = count; remaining > 1; remaining -= 1) {
        CubicBezierSplit<Vector2> halves = rest.split(1.0f / static_cast<float>(remaining));
        pieces.push_back(halves.first);
        rest = halves.second;
    }
    pieces.push_back(rest);
    return pieces;
}

template class CubicBezier<Vector2>;

} // namespace editor
