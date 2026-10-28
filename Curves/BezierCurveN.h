// Béziers of any order and rational Béziers: evaluating, splitting and differentiating them,
// raising and lowering their order, fitting one of a given order to points, and bringing any of
// them back to cubics (graphics_cpp curves/bezn). Control points are Vector2; the order is the
// number of control points minus one, so a line is order 1, a quadratic order 2, a cubic order 3.
#pragma once

#include <optional>
#include <vector>

#include "../Linear/Vector2.h"
#include "CubicBezier.h"

namespace editor {

class BezierCurveN;

/// The two curves a `BezierCurveN` is cut into (like `CubicBezierSplit`, for any order).
///
/// Use when: using both parts of a cut high-order curve
/// Tags: Bezier split, de Casteljau split, halves
/// Node: planned (splitBezier)
struct BezierCurveNSplit;

/// A Bézier curve of any order: the curve from the first control point to the last, pulled
/// towards the ones between.
///
/// Use when: a quadratic (TrueType's curves), a quartic or higher drawn by a script, or the
///   hodograph (derivative curve) of a cubic; a curve that must pass a chosen number of handles
/// Tags: Bezier curve, degree n Bezier, order n, control points, Bernstein, de Casteljau
/// Node: planned (bezierCurve)
class BezierCurveN {
public:
    /// The control points; at least one.
    std::vector<Vector2> controlPoints;

    /// The curve with these control points. Throws a `MathProblem` when there are none.
    explicit BezierCurveN(std::vector<Vector2> controlPoints);

    bool operator==(const BezierCurveN&) const = default;

    /// The order: the number of control points minus one.
    int order() const { return static_cast<int>(controlPoints.size()) - 1; }

    /// The curve as a cubic when it has exactly four control points.
    static BezierCurveN fromCubic(const CubicBezier<Vector2>& cubic);

    // MARK: Evaluating (BezierCurveN.cpp)

    Vector2 point(float parameter) const;
    std::vector<std::vector<Vector2>> deCasteljauLevels(float parameter) const;
    BezierCurveNSplit split(float parameter) const;
    BezierCurveN derivative() const;
    BezierCurveN raised() const;
    BezierCurveN lowered() const;
    Vector2 rationalPoint(const std::vector<float>& weights, float parameter) const;
    static std::vector<float> bernsteinWeights(int order, float parameter);

    // MARK: Fitting and conversion (BezierCurveN+Fitting.cpp)

    static BezierCurveN fitted(const std::vector<Vector2>& points, int order = 3);
    std::vector<CubicBezier<Vector2>> asCubics(
        const std::optional<std::vector<float>>& weights = std::nullopt, float tolerance = 1e-3f, int samples = 64) const;

    /// The number of rounds of `fitted`: 5. Each solves the control points, then moves every
    /// point's parameter to its nearest place on the curve so far.
    static constexpr int fittingRounds = 5;
};

struct BezierCurveNSplit {
    /// The part from the original start up to the cut.
    BezierCurveN first;
    /// The part from the cut to the original end.
    BezierCurveN second;
};

} // namespace editor
