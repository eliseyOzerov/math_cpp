// Evaluating Béziers of any order (see BezierCurveN.h).
#include "BezierCurveN.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "../Linear/MathProblem.h"

namespace editor {

BezierCurveN::BezierCurveN(std::vector<Vector2> controlPoints) : controlPoints(std::move(controlPoints)) {
    if (this->controlPoints.empty()) throw MathProblem("A Bézier curve needs at least one control point.");
}

/// The curve with a cubic's four control points.
///
/// Use when: raising a cubic to a quartic, or taking its derivative as a quadratic
/// Tags: cubic to order n, control points of cubic
/// Node: planned (bezierCurve)
BezierCurveN BezierCurveN::fromCubic(const CubicBezier<Vector2>& cubic) {
    return BezierCurveN({cubic.start, cubic.startHandle, cubic.endHandle, cubic.end});
}

/// The point at `parameter` on the curve.
///
/// How: de Casteljau. Blend each neighbouring pair of control points by `parameter`
/// (`a + (b − a)·parameter`), then blend the results the same way, until one point is left.
/// Stable at every order (all weights stay between 0 and 1 for parameters in 0…1).
///
///     p0 ●──○──● p1 ●──○──● p2     ○ = first blends; the next row blends those; ...
///
/// Conventions: y-up plane; a parameter outside 0…1 extrapolates. One control point is a
/// constant curve.
///
/// Source: de Casteljau (1959); Farin, *Curves and Surfaces for CAGD*, ch. 4.
///
/// Use when: a point on a quadratic, quartic or higher curve; the end of a hodograph
/// Tags: Bezier at t, de Casteljau, bezierAt, evaluate Bezier, degree n
/// Node: planned (bezierPoint)
Vector2 BezierCurveN::point(float parameter) const { return deCasteljauLevels(parameter).back().front(); }

/// Every level of de Casteljau's construction at `parameter`: row 0 is the control points,
/// each next row the blends of neighbours, down to the one point on the curve (the last row).
/// The first points of the rows are the control points of the left part, the last points of
/// the rows (reversed) those of the right part.
///
/// Use when: drawing the construction in a lesson or a proof; cutting the curve
/// Tags: casteljau levels, construction lines, de Casteljau triangle, casteljauN
/// Node: planned (bezierPoint)
std::vector<std::vector<Vector2>> BezierCurveN::deCasteljauLevels(float parameter) const {
    std::vector<std::vector<Vector2>> levels{controlPoints};
    while (levels.back().size() > 1) {
        const std::vector<Vector2>& row = levels.back();
        std::vector<Vector2> next;
        for (size_t index = 0; index + 1 < row.size(); index += 1) {
            next.push_back(Vector2::linearBlend(row[index], row[index + 1], parameter));
        }
        levels.push_back(std::move(next));
    }
    return levels;
}

/// The curve cut at `parameter` into two of the same order that together trace it.
///
/// How: the de Casteljau rows (`deCasteljauLevels`): the first point of every row is the
/// first part's control points; the last point of every row, read from the bottom up, is the
/// second part's.
///
/// Conventions: both parts share the point at `parameter`.
///
/// Use when: cutting a quartic; tests of splitting; finding the pieces to the left and right of a
///   crossing
/// Tags: split Bezier, subdivide, cut curve, splitN
/// Node: planned (splitBezier)
BezierCurveNSplit BezierCurveN::split(float parameter) const {
    std::vector<std::vector<Vector2>> levels = deCasteljauLevels(parameter);
    std::vector<Vector2> before;
    std::vector<Vector2> after;
    for (const std::vector<Vector2>& row : levels) {
        before.push_back(row.front());
        after.push_back(row.back());
    }
    std::reverse(after.begin(), after.end());
    return BezierCurveNSplit{BezierCurveN(before), BezierCurveN(after)};
}

/// The derivative curve (hodograph): a Bézier of one order less whose control points are
/// `order · (pᵢ₊₁ − pᵢ)`. Its point at a parameter is the velocity of this curve there.
///
/// Conventions: a curve of order 0 has no derivative: throws a `MathProblem`.
///
/// Source: Farin, *Curves and Surfaces for CAGD*, §4.2.
///
/// Use when: the speed or tangent of a high-order curve; finding where it turns (the hodograph
///   crosses an axis)
/// Tags: derivative Bezier, hodograph, velocity curve, derivativeN
/// Node: planned (bezierDerivative)
BezierCurveN BezierCurveN::derivative() const {
    if (order() < 1) throw MathProblem("A Bézier curve of order 0 has no derivative.");
    float scale = static_cast<float>(order());
    std::vector<Vector2> differences;
    for (size_t index = 0; index + 1 < controlPoints.size(); index += 1) {
        differences.push_back((controlPoints[index + 1] - controlPoints[index]) * scale);
    }
    return BezierCurveN(differences);
}

/// The same curve with one more control point (degree elevation): with k the old number of
/// control points, qᵢ = (i/k)·pᵢ₋₁ + (1 − i/k)·pᵢ for 0 < i < k, and the first and last points stay.
///
/// Why: the Bernstein basis of order n is a combination of the basis of order n + 1, so the
/// curve is unchanged; only its handles move.
///
/// Conventions: the curve traces exactly the same points.
///
/// Source: Farin, *Curves and Surfaces for CAGD*, §5.1 (degree elevation).
///
/// Use when: giving a line or quadratic the four points of a cubic (a TrueType contour read back
///   to cubics); adding a handle to pull
/// Tags: degree elevation, raise order, elevate Bezier, quadratic to cubic
/// Node: planned (raiseBezier)
BezierCurveN BezierCurveN::raised() const {
    size_t count = controlPoints.size();
    std::vector<Vector2> raisedPoints;
    for (size_t index = 0; index <= count; index += 1) {
        if (index == 0) {
            raisedPoints.push_back(controlPoints.front());
        } else if (index == count) {
            raisedPoints.push_back(controlPoints.back());
        } else {
            raisedPoints.push_back(Vector2::linearBlend(
                controlPoints[index], controlPoints[index - 1], static_cast<float>(index) / static_cast<float>(count)));
        }
    }
    return BezierCurveN(raisedPoints);
}

/// The point at `parameter` on a rational Bézier: each control point pulls by its weight.
///
/// How: `Σ wᵢ·Bᵢ(t)·pᵢ / Σ wᵢ·Bᵢ(t)` with the Bernstein weights `Bᵢ` (`bernsteinWeights`). All weights
/// 1 gives the plain curve; a weight above 1 draws the curve towards that point; a missing weight
/// counts as 1. Weights `[1, cos(θ/2), 1]` on three points make a circular arc of angle θ.
///
/// Conventions: weights should be positive for the curve to stay inside the hull of its points.
///
/// Source: Farin, *Curves and Surfaces for CAGD*, ch. 13 (rational Bézier curves).
///
/// Use when: true conics (a circle's quarter, an ellipse's arc); a curve with a "tension" per handle
/// Tags: rational Bezier, weights, conic, NURBS piece, rationalAt
/// Node: planned (rationalBezier)
Vector2 BezierCurveN::rationalPoint(const std::vector<float>& weights, float parameter) const {
    std::vector<float> blend = bernsteinWeights(order(), parameter);
    Vector2 weightedSum;
    float weightSum = 0;
    for (size_t index = 0; index < controlPoints.size(); index += 1) {
        float weight = blend[index] * (index < weights.size() ? weights[index] : 1.0f);
        weightedSum += controlPoints[index] * weight;
        weightSum += weight;
    }
    return weightedSum / weightSum;
}

/// The Bernstein weights of an order-`order` Bézier at `parameter`: `order + 1` numbers
/// `C(n, i)·(1 − t)ⁿ⁻ⁱ·tⁱ` that add up to 1; the curve is the sum of control points times these.
///
/// How: Pascal's triangle by repeated blending: start with `[1]`, and for each order each weight
/// becomes `(1 − t)` times itself plus `t` times its left neighbour.
///
/// Conventions: order 0 is `[1]`. Throws a `MathProblem` for a negative order.
///
/// Source: Bernstein (1912); Farin, *Curves and Surfaces for CAGD*, §4.1.
///
/// Use when: computing a curve as a weighted sum; building a least-squares fit matrix
/// Tags: Bernstein polynomials, Bernstein basis, blending weights, binomial weights
/// Node: planned (bezierPoint)
std::vector<float> BezierCurveN::bernsteinWeights(int order, float parameter) {
    if (order < 0) throw MathProblem("The order of a Bézier curve cannot be negative, but got " + std::to_string(order) + ".");
    std::vector<float> weights{1.0f};
    for (int level = 1; level <= order; level += 1) {
        weights.push_back(0);
        for (int index = level; index >= 0; index -= 1) {
            float stay = index < level ? weights[index] * (1 - parameter) : 0.0f;
            float step = index > 0 ? weights[index - 1] * parameter : 0.0f;
            weights[index] = stay + step;
        }
    }
    return weights;
}

} // namespace editor
