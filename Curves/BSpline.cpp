// B-splines and NURBS: de Boor's algorithm, basis functions, knot insertion, Bézier pieces
// (Engine/Math/Curves/BSpline.swift).
#include "BSpline.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../Linear/NumberText.h"
#include "../Linear/Angle.h"
#include "../Linear/Scalar.h"

namespace editor {

/// A spline, or an empty optional (Swift's `nil`) when `problem(...)` finds something wrong with
/// the inputs.
///
/// Use when: building a B-spline or NURBS from control points and knots
/// Tags: create spline, make NURBS, B-spline init
template <CurvePoint PointType>
std::optional<BSpline<PointType>> BSpline<PointType>::make(
    int degree, const std::vector<Point>& controlPoints, const std::vector<float>& knots,
    const std::optional<std::vector<float>>& weights) {
    if (problem(degree, controlPoints, knots, weights).has_value()) return std::nullopt;
    BSpline spline;
    spline.degree = degree;
    spline.controlPoints = controlPoints;
    spline.knots = knots;
    spline.weights = weights;
    return spline;
}

/// What is wrong with these spline inputs, in a readable sentence, or empty when they are
/// fine.
///
/// Checks, in order: degree in 1...`highestDegree`; at least `degree + 1` and at most
/// `mostControlPoints` control points, all finite; exactly `n + degree + 1` knots, finite,
/// with a finite difference `last − first`, and non-decreasing; a domain of positive length
/// (`knots[degree] < knots[n]`); no knot repeated more than `degree` times inside the domain,
/// or more than `degree + 1` times at either end of the domain (more would cut the curve into
/// disconnected pieces); weights, if given, one per control point, each positive and finite;
/// and every weighted step `(Pᵢ − Pⱼ)·wᵢ` to the `degree` points before it finite, since the
/// evaluation works with exactly those numbers (see `weightedStepsAreFinite`).
///
/// Use when: telling the user why a spline cannot be built; validating imported NURBS data
/// Tags: validate spline, knot vector check, NURBS validation, invalid knots, error message
/// Node: none (validation)
template <CurvePoint PointType>
std::optional<std::string> BSpline<PointType>::problem(
    int degree, const std::vector<Point>& controlPoints, const std::vector<float>& knots,
    const std::optional<std::vector<float>>& weights) {
    if (!(degree >= 1 && degree <= highestDegree())) {
        return "The degree must be between 1 and " + std::to_string(highestDegree()) + ", not " + std::to_string(degree) + ".";
    }
    int count = static_cast<int>(controlPoints.size());
    if (!(count >= degree + 1)) {
        return "A spline of degree " + std::to_string(degree) + " needs at least " + std::to_string(degree + 1)
            + " control points, not " + std::to_string(count) + ".";
    }
    if (!(count <= mostControlPoints())) {
        return "A spline may have at most " + std::to_string(mostControlPoints()) + " control points, not "
            + std::to_string(count) + ".";
    }
    for (const Point& controlPoint : controlPoints) {
        if (!controlPoint.isFinite()) return std::string("Every control point must have finite coordinates.");
    }
    if (!(static_cast<int>(knots.size()) == count + degree + 1)) {
        return "The knot vector needs control points + degree + 1 = " + std::to_string(count + degree + 1)
            + " knots, not " + std::to_string(knots.size()) + ".";
    }
    for (float knot : knots) {
        if (!std::isfinite(knot)) return std::string("Every knot must be finite.");
    }
    // Every ratio in de Boor's algorithm divides by a difference of two knots; the widest is
    // last − first, which can overflow even when both knots are finite (−3e38 to 3e38).
    if (!std::isfinite(knots[knots.size() - 1] - knots[0])) {
        return std::string("The knots span too wide a range: the last minus the first must be finite.");
    }
    for (size_t index = 1; index < knots.size(); index += 1) {
        if (knots[index] < knots[index - 1]) {
            return "The knots must not decrease, but knot " + std::to_string(index) + " is smaller than knot "
                + std::to_string(index - 1) + ".";
        }
    }
    if (!(knots[degree] < knots[count])) {
        return "The domain is empty: knot " + std::to_string(degree) + " must be smaller than knot " + std::to_string(count) + ".";
    }
    size_t runStart = 0;
    while (runStart < knots.size()) {
        size_t runEnd = runStart;
        while (runEnd + 1 < knots.size() && knots[runEnd + 1] == knots[runStart]) runEnd += 1;
        int multiplicity = static_cast<int>(runEnd - runStart + 1);
        bool isDomainEnd = knots[runStart] == knots[degree] || knots[runStart] == knots[count];
        if (multiplicity > (isDomainEnd ? degree + 1 : degree)) {
            return "The knot " + numberText(knots[runStart]) + " is repeated " + std::to_string(multiplicity)
                + " times; the most allowed is " + std::to_string(isDomainEnd ? degree + 1 : degree) + ".";
        }
        runStart = runEnd + 1;
    }
    if (weights.has_value()) {
        if (!(static_cast<int>(weights->size()) == count)) {
            return "There must be one weight per control point (" + std::to_string(count) + "), not "
                + std::to_string(weights->size()) + ".";
        }
        for (float weight : *weights) {
            if (!(weight > 0 && std::isfinite(weight))) return std::string("Every weight must be positive and finite.");
        }
    }
    if (!weightedStepsAreFinite(degree, controlPoints, weights)) {
        return std::string("A control point is too far from its neighbours for its weight: (point − neighbour) × weight overflows.");
    }
    return std::nullopt;
}

/// Whether every control point, minus each of the `degree` points before it and multiplied by
/// its own weight (1 when there are no weights), has finite coordinates.
///
/// Why: evaluation and knot insertion work relative to the first control point of a span (for
/// precision far from the origin, see `point`), on the homogeneous numbers
/// `(Pᵢ − P_first)·wᵢ`, and `P_first` is at most `degree` places before `Pᵢ`. With
/// P = (1e20, 1e20) and w = 1e30 that product is beyond Float and the point became NaN. Blends
/// of finite numbers with ratios in 0...1 stay finite, so this check is enough for every
/// evaluation inside the domain. Cost: `n·degree` subtractions.
///
/// Use when: validating imported NURBS data with extreme weights or coordinates
/// Tags: weighted point overflow, homogeneous coordinates, NURBS validation, finite check
/// Node: none (validation)
template <CurvePoint PointType>
bool BSpline<PointType>::weightedStepsAreFinite(
    int degree, const std::vector<Point>& controlPoints, const std::optional<std::vector<float>>& weights) {
    for (int index = 0; index < static_cast<int>(controlPoints.size()); index += 1) {
        float weight = weights.has_value() ? (*weights)[index] : 1;
        for (int before = std::max(0, index - degree); before < index; before += 1) {
            if (!((controlPoints[index] - controlPoints[before]) * weight).isFinite()) return false;
        }
    }
    return true;
}

/// The knot vector of a **clamped uniform** spline: `degree + 1` zeros, then the whole numbers
/// `1 … n − degree − 1`, then `degree + 1` copies of `n − degree`. Its domain is `0 … n − degree`,
/// one unit per piece, and it passes through the first and last control point. Empty (Swift's
/// `nil`) if the degree or count is out of range.
///
/// Example: degree 3, 4 points: `[0,0,0,0,1,1,1,1]`; degree 3, 6 points: `[0,0,0,0,1,2,3,3,3,3]`.
///
/// Use when: making an ordinary spline through a list of control points without choosing knots
/// Tags: uniform knots, clamped knot vector, open uniform, default knots
/// Node: inside curveBSpline, profileBSpline
template <CurvePoint PointType>
std::optional<std::vector<float>> BSpline<PointType>::clampedUniformKnots(int degree, int controlPointCount) {
    if (!(degree >= 1 && degree <= highestDegree() && controlPointCount >= degree + 1
          && controlPointCount <= mostControlPoints())) {
        return std::nullopt;
    }
    std::vector<float> result(static_cast<size_t>(degree + 1), 0.0f);
    for (int whole = 1; whole < controlPointCount - degree; whole += 1) result.push_back(static_cast<float>(whole));
    float end = static_cast<float>(controlPointCount - degree);
    result.insert(result.end(), static_cast<size_t>(degree + 1), end);
    return result;
}

/// The parameter values the curve is defined on: `knots[degree]` to `knots[n]`.
///
/// Use when: sampling the whole spline from start to end
/// Tags: spline domain, parameter range, knot span, t range
/// Node: none (protocol plumbing)
template <CurvePoint PointType>
ParameterRange BSpline<PointType>::parameterRange() const {
    return ParameterRange{knots[degree], knots[controlPoints.size()]};
}

/// Whether the curve begins exactly at the first control point: the first `degree + 1`
/// knots are equal.
///
/// Use when: knowing whether a spline touches its first control point
/// Tags: clamped start, open knot vector, interpolates first point
/// Node: none (validation)
template <CurvePoint PointType>
bool BSpline<PointType>::isClampedAtStart() const {
    return knots[0] == knots[degree];
}

/// Whether the curve ends exactly at the last control point: the last `degree + 1` knots
/// are equal.
///
/// Use when: knowing whether a spline touches its last control point
/// Tags: clamped end, open knot vector, interpolates last point
/// Node: none (validation)
template <CurvePoint PointType>
bool BSpline<PointType>::isClampedAtEnd() const {
    return knots[controlPoints.size()] == knots[controlPoints.size() + degree];
}

// MARK: - Spans and basis functions

/// The index `k` of the knot span that holds `parameter`: `knots[k] ≤ parameter < knots[k+1]`,
/// with `degree ≤ k ≤ n − 1`, by binary search. The span is never empty (knots[k] < knots[k+1]).
/// (Swift's `span(containing:)`.)
///
/// Outside the domain the first or last non-empty span is returned (so evaluation continues
/// that piece's polynomial); at the end of the domain it is the last non-empty span.
/// NaN gives the first non-empty span. Below the domain the search alone stops at `degree`,
/// which is empty when the domain's first knot is repeated (knots `[0,0,1,1,2,3,3,3]`,
/// degree 2: span 2 is `[1, 1]`), so it then steps forward to the first non-empty span; the
/// domain has positive length, so one exists.
///
/// Source: Piegl & Tiller, algorithm A2.1 (FindSpan).
///
/// Use when: finding which polynomial piece of the spline a parameter falls in
/// Tags: find span, knot span search, binary search knots, FindSpan
/// Node: none (solver)
template <CurvePoint PointType>
int BSpline<PointType>::span(float parameter) const {
    int count = static_cast<int>(controlPoints.size());
    if (parameter >= knots[count]) {
        int last = count - 1;
        while (last > degree && knots[last] == knots[last + 1]) last -= 1;
        return last;
    }
    int low = degree;
    int high = count - 1;
    // Largest k in low...high with knots[k] ≤ parameter (the first span if parameter is below).
    while (low < high) {
        int middle = (low + high + 1) / 2;
        if (knots[middle] <= parameter) {
            low = middle;
        } else {
            high = middle - 1;
        }
    }
    // Only below the domain (or for NaN) can `low` be an empty span; inside it the largest
    // k with knots[k] ≤ parameter always has knots[k + 1] > parameter.
    while (knots[low] == knots[low + 1]) low += 1;
    return low;
}

/// The `degree + 1` basis functions that are not zero at `parameter`, and the index of the
/// control point the first belongs to: `values[j]` multiplies `controlPoints[firstIndex + j]`.
/// Inside `parameterRange` they are never negative and **add up to 1** (partition of unity).
/// Outside it they still add up to 1 (they continue the end piece's polynomials), but some
/// can be negative.
///
/// How: the triangular form of the Cox–de Boor recurrence. Starting from `N = [1]` of degree 0,
/// each degree adds one function from the previous ones with the two ratios of the recurrence,
/// so only the non-zero functions are ever computed.
///
/// Source: Cox (1972); de Boor (1972); Piegl & Tiller, algorithm A2.2 (BasisFuns).
///
/// Use when: seeing how much each control point pulls the curve at a parameter; building a
///   least-squares spline fit
/// Tags: basis functions, B-spline basis, Cox-de Boor, partition of unity, blending functions
/// Node: none (solver)
template <CurvePoint PointType>
BasisFunctionValues BSpline<PointType>::basisFunctions(float parameter) const {
    int containingSpan = span(parameter);
    std::vector<float> values(static_cast<size_t>(degree + 1), 0.0f);
    std::vector<float> left(static_cast<size_t>(degree + 1), 0.0f);
    std::vector<float> right(static_cast<size_t>(degree + 1), 0.0f);
    values[0] = 1;
    for (int level = 1; level <= degree; level += 1) {
        left[level] = parameter - knots[containingSpan + 1 - level];
        right[level] = knots[containingSpan + level] - parameter;
        float saved = 0;
        for (int index = 0; index < level; index += 1) {
            float share = values[index] / (right[index + 1] + left[level - index]);
            values[index] = saved + right[index + 1] * share;
            saved = left[level - index] * share;
        }
        values[level] = saved;
    }
    return BasisFunctionValues{containingSpan - degree, values};
}

// MARK: - de Boor's algorithm

/// Runs `rounds` rounds of de Boor's algorithm on the `degree + 1` control points that matter
/// for `parameter` (homogeneous: each point, measured from the span's first control point,
/// multiplied by its weight, and the weights carried along). After round r the entry `j ≥ r` is
/// a point of "level r"; after `degree` rounds entry `degree` is the answer, relative to
/// `origin`. The work area holds the span, the first control point of the span (every weighted
/// point is `(P − origin)·w`), the weighted points and the weights.
template <CurvePoint PointType>
typename BSpline<PointType>::DeBoorState BSpline<PointType>::deBoor(float parameter, int rounds) const {
    int containingSpan = span(parameter);
    Point origin = controlPoints[containingSpan - degree];
    std::vector<Point> weightedPoints;
    std::vector<float> pointWeights;
    weightedPoints.reserve(static_cast<size_t>(degree + 1));
    pointWeights.reserve(static_cast<size_t>(degree + 1));
    for (int offset = 0; offset <= degree; offset += 1) {
        int index = containingSpan - degree + offset;
        float weight = weights.has_value() ? (*weights)[index] : 1;
        weightedPoints.push_back((controlPoints[index] - origin) * weight);
        pointWeights.push_back(weight);
    }
    for (int round = 1; round <= rounds; round += 1) {
        for (int offset = degree; offset >= round; offset -= 1) {
            int index = offset + containingSpan - degree;
            float alpha = (parameter - knots[index]) / (knots[index + degree + 1 - round] - knots[index]);
            weightedPoints[offset] = Point::linearBlend(weightedPoints[offset - 1], weightedPoints[offset], alpha);
            pointWeights[offset] = linearBlend(pointWeights[offset - 1], pointWeights[offset], alpha);
        }
    }
    return DeBoorState{containingSpan, origin, weightedPoints, pointWeights};
}

/// The point of the curve at `parameter`, by de Boor's algorithm.
///
/// How: take the `p + 1` control points of the span that holds `parameter`, `dⱼ = P(k−p+j)`.
/// For round r = 1 … p, replace `dⱼ` (for j = p down to r) by the blend of its left neighbour
/// and itself with
///
///     αⱼ,ᵣ = (u − knots[i]) / (knots[i+p+1−r] − knots[i]),    i = j + k − p
///
/// The last entry `dₚ` is the point. It is de Casteljau's algorithm with knot-dependent ratios;
/// every ratio lies in 0...1 inside the span, so it is a stable chain of convex blends. For a
/// rational spline the points are first multiplied by their weights, the weights are blended
/// with the same ratios, and the point is divided by the blended weight at the end.
///
/// The points are measured from the span's first control point and it is added back at the
/// end: `C = P_first + Σ Nᵢwᵢ(Pᵢ − P_first) / Σ Nᵢwᵢ`, the same curve since the basis adds up
/// to 1. Far from the origin, the weighted sums of large coordinates would otherwise round at
/// the size of the coordinates times the weights: a unit NURBS circle around (1e6, 1e6) was
/// off by 0.126, now by the rounding of the final addition.
///
/// For a clamped end, the end of the domain returns the first or last control point itself,
/// exactly; blends alone would miss it by a rounding step.
///
/// Conventions: see the type's comment for outside the domain and NaN.
///
/// Source: de Boor (1972); Piegl & Tiller A4.1 for the rational case.
///
/// Use when: sampling a spline to draw it or to place objects along it
/// Tags: evaluate B-spline, de Boor algorithm, NURBS point, spline position, sample spline
/// Node: inside curveBSpline
template <CurvePoint PointType>
PointType BSpline<PointType>::point(float parameter) const {
    if (!std::isfinite(parameter)) return controlPoints[0] * std::numeric_limits<float>::quiet_NaN();
    if (parameter == parameterRange().lowerBound && isClampedAtStart()) return controlPoints[0];
    if (parameter == parameterRange().upperBound && isClampedAtEnd()) return controlPoints[controlPoints.size() - 1];
    DeBoorState state = deBoor(parameter, degree);
    return state.origin + state.weightedPoints[degree] * (1 / state.pointWeights[degree]);
}

/// The velocity at `parameter`: the derivative of the position with respect to the parameter.
///
/// How: after `p − 1` rounds of de Boor's algorithm the two remaining entries `a = d(p−1)`
/// and `b = d(p)` are the ends of a straight segment that is traversed at constant rate over
/// the span, so
///
///     C′(u) = p · (b − a) / (knots[k+1] − knots[k])
///
/// (for a cubic Bézier this is `3·(e − d)` of de Casteljau's second level, as in
/// `CubicBezier.firstDerivative`). For a rational spline `A` is the weighted curve and `W`
/// its weight: `C′ = (A′ − W′·C) / W`, the quotient rule. Measured from the span's first
/// control point as in `point`: moving the curve does not change its velocity, so `C` here
/// is the point relative to it.
///
/// Source: de Boor (1972) (the derivative of a B-spline is a B-spline of lower degree);
/// Piegl & Tiller, ch. 3 and 4 for the rational quotient rule.
///
/// Use when: finding the direction a spline heads in; building a frame along it
/// Tags: B-spline tangent, NURBS derivative, spline velocity, curve direction, hodograph
/// Node: none (a Curve requirement; graph curves are drawn as points, so nothing reads it)
template <CurvePoint PointType>
PointType BSpline<PointType>::firstDerivative(float parameter) const {
    if (!std::isfinite(parameter)) return controlPoints[0] * std::numeric_limits<float>::quiet_NaN();
    DeBoorState state = deBoor(parameter, degree - 1);
    float spanLength = knots[state.span + 1] - knots[state.span];
    float rate = static_cast<float>(degree) / spanLength;
    Point weightedStep = (state.weightedPoints[degree] - state.weightedPoints[degree - 1]) * rate;
    if (!weights.has_value()) return weightedStep;
    float weightStep = (state.pointWeights[degree] - state.pointWeights[degree - 1]) * rate;
    float alpha = (parameter - knots[state.span]) / spanLength;
    Point weightedPoint = Point::linearBlend(state.weightedPoints[degree - 1], state.weightedPoints[degree], alpha);
    float weight = linearBlend(state.pointWeights[degree - 1], state.pointWeights[degree], alpha);
    Point relativePoint = weightedPoint * (1 / weight);
    return (weightedStep - relativePoint * weightStep) * (1 / weight);
}

// MARK: - Knot insertion

/// The same curve with `knot` added to the knot vector and one more control point: the shape
/// does not change, but there is one more point to edit. Empty (Swift's `nil`) if `knot` is not
/// strictly inside the domain, is not finite, or already appears `degree` times, or if the spline
/// already has `mostControlPoints` control points (the result would have one too many).
///
/// How (Boehm): with `k` the span of `knot` and `s` its multiplicity so far, the new control
/// points are
///
///     Qᵢ = Pᵢ                              for i ≤ k − p
///     Qᵢ = (1 − αᵢ)·Pᵢ₋₁ + αᵢ·Pᵢ           for k − p + 1 ≤ i ≤ k − s,   αᵢ = (u − knots[i]) / (knots[i+p] − knots[i])
///     Qᵢ = Pᵢ₋₁                            for i ≥ k − s + 1
///
/// Only p − s of the old points change (their neighbours are blended); the rest are copied.
/// A rational spline does the blends on weighted points and weights, then divides. Each blend
/// is measured from its first point, as in `point`:
///
///     wQ = (1 − α)·wᵢ₋₁ + α·wᵢ,    Qᵢ = Pᵢ₋₁ + (α·wᵢ / wQ)·(Pᵢ − Pᵢ₋₁)
///
/// which is `((1 − α)·wᵢ₋₁·Pᵢ₋₁ + α·wᵢ·Pᵢ) / wQ` rearranged (with every weight 1 it is the
/// plain blend).
///
/// Source: Boehm, "Inserting new knots into B-spline curves", *Computer-Aided Design* 12
/// (1980); Piegl & Tiller, algorithm A5.1.
///
/// Use when: adding an editable point to a spline without changing it; splitting a spline into
///   Bézier pieces
/// Tags: knot insertion, Boehm, refine spline, add control point, subdivide B-spline
/// Node: none (cubicBezierPieces gets the same pieces by blossoming)
template <CurvePoint PointType>
std::optional<BSpline<PointType>> BSpline<PointType>::insertingKnot(float knot) const {
    int count = static_cast<int>(controlPoints.size());
    if (!(std::isfinite(knot) && knot > parameterRange().lowerBound && knot < parameterRange().upperBound)) {
        return std::nullopt;
    }
    int multiplicity = 0;
    for (float existing : knots) {
        if (existing == knot) multiplicity += 1;
    }
    if (!(multiplicity < degree)) return std::nullopt;
    int containingSpan = span(knot);

    std::vector<Point> newPoints;
    std::vector<float> newWeights;
    newPoints.reserve(static_cast<size_t>(count + 1));
    newWeights.reserve(static_cast<size_t>(count + 1));
    for (int index = 0; index <= count; index += 1) {
        if (index <= containingSpan - degree) {
            newPoints.push_back(controlPoints[index]);
            newWeights.push_back(weights.has_value() ? (*weights)[index] : 1);
        } else if (index <= containingSpan - multiplicity) {
            float alpha = (knot - knots[index]) / (knots[index + degree] - knots[index]);
            float beforeWeight = weights.has_value() ? (*weights)[index - 1] : 1;
            float afterWeight = weights.has_value() ? (*weights)[index] : 1;
            float weight = linearBlend(beforeWeight, afterWeight, alpha);
            Point step = controlPoints[index] - controlPoints[index - 1];
            newPoints.push_back(controlPoints[index - 1] + step * (alpha * afterWeight / weight));
            newWeights.push_back(weight);
        } else {
            newPoints.push_back(controlPoints[index - 1]);
            newWeights.push_back(weights.has_value() ? (*weights)[index - 1] : 1);
        }
    }
    std::vector<float> newKnots = knots;
    newKnots.insert(newKnots.begin() + (containingSpan + 1), knot);
    return make(degree, newPoints, newKnots, weights.has_value() ? std::optional<std::vector<float>>(newWeights) : std::nullopt);
}

// MARK: - Bézier pieces

/// The curve as cubic Bézier pieces, one per knot span, joined exactly: the form a `Path2D`,
/// a flattener or an exporter takes. Empty (Swift's `nil`) unless the spline is a non-rational
/// **cubic**, clamped at both ends.
///
/// How: on the span `[a, b]` the spline is one cubic polynomial, and its Bézier control points
/// are its **blossom** (polar form) at `(a,a,a)`, `(a,a,b)`, `(a,b,b)` and `(b,b,b)`. The blossom
/// is de Boor's algorithm with a different parameter in every round: round r uses the r-th
/// argument. This is the same result as inserting each interior knot until it has multiplicity 3
/// with `insertingKnot` and reading the control points in groups of three (a test compares them),
/// but it costs a constant amount per span instead of a pass over all the points per insertion.
/// To make the join exact, each piece starts at the stored end of the one before; the first piece
/// starts at the first control point and the last ends at the last one. The blossom is measured
/// from the span's first control point, as in `point`.
///
/// Source: Ramshaw, "Blossoming: A connect-the-dots approach to splines", DEC SRC Research
/// Report 19 (1987), for blossoms; Boehm (1980) for the Bézier points as the result of
/// inserting each knot to full multiplicity; Piegl & Tiller, algorithm A5.6 (DecomposeCurve),
/// for extracting the pieces of a whole curve.
///
/// Use when: converting a B-spline to Béziers to draw, flatten, offset or export it
/// Tags: B-spline to Bezier, Bezier extraction, decompose spline, spline to cubic pieces,
///   knot insertion, blossom
/// Node: inside profileBSpline
template <CurvePoint PointType>
std::optional<std::vector<CubicBezier<PointType>>> BSpline<PointType>::cubicBezierPieces() const {
    if (!(degree == 3 && !weights.has_value() && isClampedAtStart() && isClampedAtEnd())) return std::nullopt;
    std::vector<CubicBezier<Point>> pieces;
    for (int containingSpan = degree; containingSpan < static_cast<int>(controlPoints.size()); containingSpan += 1) {
        if (!(knots[containingSpan] < knots[containingSpan + 1])) continue;
        float a = knots[containingSpan];
        float b = knots[containingSpan + 1];
        Point start = pieces.empty() ? controlPoints[0] : pieces.back().end;
        Point end = containingSpan == lastNonEmptySpan() ? controlPoints[controlPoints.size() - 1]
                                                         : blossom(containingSpan, {b, b, b});
        pieces.push_back(CubicBezier<Point>(
            start,
            blossom(containingSpan, {a, a, b}),
            blossom(containingSpan, {a, b, b}),
            end));
    }
    return pieces;
}

/// The last span with a positive length.
template <CurvePoint PointType>
int BSpline<PointType>::lastNonEmptySpan() const {
    return span(parameterRange().upperBound);
}

/// The blossom of the span's polynomial at the given arguments: de Boor's algorithm where round
/// r blends with `arguments[r − 1]`. Equal arguments give the curve point.
template <CurvePoint PointType>
PointType BSpline<PointType>::blossom(int containingSpan, const std::array<float, 3>& arguments) const {
    Point origin = controlPoints[containingSpan - degree];
    std::vector<Point> points;
    for (int offset = 0; offset <= degree; offset += 1) {
        points.push_back(controlPoints[containingSpan - degree + offset] - origin);
    }
    for (int round = 1; round <= degree; round += 1) {
        for (int offset = degree; offset >= round; offset -= 1) {
            int index = offset + containingSpan - degree;
            float alpha = (arguments[round - 1] - knots[index]) / (knots[index + degree + 1 - round] - knots[index]);
            points[offset] = Point::linearBlend(points[offset - 1], points[offset], alpha);
        }
    }
    return origin + points[degree];
}

// MARK: - 2D conveniences

/// The spline as a `Path2D` of cubic pieces, for a clamped non-rational cubic (see
/// `cubicBezierPieces`), or empty (Swift's `nil`).
///
/// Use when: drawing, flattening or sweeping a B-spline profile with the path tools
/// Tags: spline to path, B-spline Path2D, NURBS to path, flatten spline
/// Node: inside profileBSpline
template <CurvePoint PointType>
std::optional<Path2D> BSpline<PointType>::path() const
    requires std::same_as<PointType, Vector2>
{
    std::optional<std::vector<CubicBezier<Vector2>>> pieces = cubicBezierPieces();
    if (!pieces || pieces->empty()) return std::nullopt;
    Path2D result(pieces->front().start);
    for (const CubicBezier<Vector2>& piece : *pieces) {
        result.addCubic(piece.startHandle, piece.endHandle, piece.end);
    }
    return result;
}

/// The **NURBS circle** around `centre`: a closed degree-2 rational spline with 9 control
/// points, four 90° arcs, whose points are exactly on the circle.
///
/// How: the control points are the corners and edge midpoints of the square around the circle,
/// starting at `(r, 0)` and going counterclockwise; the corner points have weight cos 45° = √2/2
/// and the others weight 1. The knots are `[0,0,0, ¼,¼, ½,½, ¾,¾, 1,1,1]`, so each quarter turn is
/// one rational quadratic Bézier piece (see `RationalQuadraticBezier.circularArc`). The
/// parameter is not the angle. The seven-point form (three 120° arcs, weight ½) also exists; this
/// nine-point form is used because every piece is a quarter turn.
///
/// Empty (Swift's `nil`) unless `radius` is positive and finite and `centre` is finite.
///
/// Source: Piegl & Tiller, *The NURBS Book* (1997), ch. 7 (conics and circles).
///
/// Use when: an exact circle for CAD export or for a revolve profile; a circle that scales and
///   moves by moving control points
/// Tags: NURBS circle, exact circle, conic spline, rational circle, 9-point circle
/// Node: none (profileCircle draws circles with Mesh.circleProfile)
template <CurvePoint PointType>
std::optional<BSpline<PointType>> BSpline<PointType>::circle(Vector2 centre, float radius)
    requires std::same_as<PointType, Vector2>
{
    if (!(radius > 0 && std::isfinite(radius) && std::isfinite(centre.x) && std::isfinite(centre.y))) return std::nullopt;
    std::vector<Vector2> corners = {
        Vector2(1, 0), Vector2(1, 1), Vector2(0, 1), Vector2(-1, 1), Vector2(-1, 0),
        Vector2(-1, -1), Vector2(0, -1), Vector2(1, -1), Vector2(1, 0),
    };
    float cornerWeight = Angle::degrees(45).cosine();
    std::vector<Vector2> circlePoints;
    for (Vector2 corner : corners) circlePoints.push_back(centre + corner * radius);
    return make(
        2, circlePoints, {0, 0, 0, 0.25f, 0.25f, 0.5f, 0.5f, 0.75f, 0.75f, 1, 1, 1},
        std::vector<float>{1, cornerWeight, 1, cornerWeight, 1, cornerWeight, 1, cornerWeight, 1});
}

template class BSpline<Vector2>;
template class BSpline<Vector3>;

} // namespace editor
