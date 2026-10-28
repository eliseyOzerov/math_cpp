// A B-spline is a curve made of polynomial pieces that join smoothly, steered by control points.
// A Bézier curve is the special case of one piece; a B-spline lets a long smooth curve be edited
// locally, because each control point only affects a few neighbouring pieces. Giving every
// control point a weight makes it a NURBS, which can draw circles and other conics exactly
// (Engine/Math/Curves/BSpline.swift).
//
// Swift's failable initializer `init?` is the static function `make`, which returns an empty
// `std::optional` where Swift returns `nil`.
#pragma once

#include <array>
#include <concepts>
#include <optional>
#include <string>
#include <vector>

#include "Path2D.h"
#include "../Linear/Vector2.h"
#include "../Linear/Vector3.h"
#include "CubicBezier.h"
#include "Curve.h"

namespace editor {

/// The basis functions that are not zero at a parameter, and the control point the first belongs
/// to (the result of `BSpline::basisFunctions`).
///
/// Use when: seeing how much each control point pulls the curve at a parameter
/// Tags: basis functions, partition of unity, blending functions
/// Node: none (solver)
struct BasisFunctionValues {
    int firstIndex = 0;
    std::vector<float> values;
};

/// A **B-spline** curve of a given degree through control points, optionally **rational**
/// (a NURBS) when every control point has a weight. `Point` is `Vector2` or `Vector3`: the same
/// algorithm serves a profile and a path in space, since it is built only from straight blends.
///
///          P1 ●─ ─ ─ ─ ─ ● P2          degree 3, 4 control points, knots [0,0,0,0,1,1,1,1]
///            ╱   ╭─────╮   ╲           (clamped): a single cubic Bézier curve.
///        P0 ●───╯       ╰───● P3
///
/// **Parts.** `degree` p (1 line pieces, 2 quadratic, 3 cubic); `n = controlPoints.count`; a
/// non-decreasing `knots` vector of `n + p + 1` numbers that says where the pieces join; and
/// optional positive `weights`. The curve lives on `parameterRange = knots[p]...knots[n]`. Between
/// two different neighbouring knots it is one polynomial of degree p; a knot repeated m times
/// makes the join continuous in the derivatives up to order `p − m`.
///
/// **Formula** (Cox 1972, de Boor 1972): `C(u) = Σ Nᵢ,ₚ(u)·Pᵢ`, with the basis functions
///
///     Nᵢ,₀(u) = 1 if knots[i] ≤ u < knots[i+1], else 0
///     Nᵢ,ₚ(u) = (u − knotsᵢ)/(knotsᵢ₊ₚ − knotsᵢ)·Nᵢ,ₚ₋₁(u) + (knotsᵢ₊ₚ₊₁ − u)/(knotsᵢ₊ₚ₊₁ − knotsᵢ₊₁)·Nᵢ₊₁,ₚ₋₁(u)
///
/// For a rational spline the same sum is taken over `wᵢ·Pᵢ` and over `wᵢ`, and the first is divided
/// by the second: `C(u) = Σ Nᵢ wᵢ Pᵢ / Σ Nᵢ wᵢ` (homogeneous coordinates). The code evaluates it
/// with **de Boor's algorithm** (`point`), not with the sum.
///
/// **Clamped** means the first p + 1 knots are equal and so are the last p + 1; then the curve
/// starts exactly at the first control point and ends exactly at the last (`clampedUniformKnots`
/// makes such a vector).
///
/// Conventions: positions in the curve's units; the parameter has the units of the knots and is not
/// distance along the curve. Outside `parameterRange` the polynomial of the first or last piece
/// simply continues. A NaN parameter gives a point of NaN. Build one with `make`, which
/// refuses inputs for which `problem(...)` gives a message, so a constructed spline is always
/// valid.
///
/// Source: Cox, "The numerical evaluation of B-splines", *J. Inst. Math. Appl.* 10 (1972); de
/// Boor, "On calculating with B-splines", *J. Approx. Theory* 6 (1972); Piegl & Tiller, *The NURBS
/// Book*, 2nd ed. (1997) (FindSpan is A2.1, the rational point A4.1).
///
/// Use when: a long smooth curve that is edited by moving control points; a NURBS profile or
///   circle; exporting to CAD formats that use NURBS
/// Tags: B-spline, NURBS, rational spline, de Boor, knot vector, control points, weights, spline
///   curve, clamped spline
/// Node: curveBSpline, profileBSpline
template <CurvePoint PointType>
class BSpline : public Curve<BSpline<PointType>, PointType> {
public:
    using Point = PointType;

    /// The degree p of the polynomial pieces.
    int degree = 0;
    /// The control points `P₀ … Pₙ₋₁`.
    std::vector<Point> controlPoints;
    /// The knot vector: `controlPoints.size() + degree + 1` non-decreasing numbers.
    std::vector<float> knots;
    /// A positive weight per control point, or empty (Swift's `nil`) for an ordinary
    /// (non-rational) spline.
    std::optional<std::vector<float>> weights;

    bool operator==(const BSpline&) const = default;

    /// The highest degree accepted: 16. It bounds the work area of one evaluation.
    ///
    /// Use when: knowing the largest degree a spline may have
    /// Tags: maximum degree, degree limit, spline order cap
    /// Node: none (validation)
    static int highestDegree() { return 16; }

    /// The most control points accepted: 1,000,000.
    ///
    /// Use when: budgeting memory for a spline; knowing why a huge one is refused
    /// Tags: maximum control points, size limit, spline cap
    /// Node: none (validation)
    static int mostControlPoints() { return 1'000'000; }

    static std::optional<BSpline> make(
        int degree, const std::vector<Point>& controlPoints, const std::vector<float>& knots,
        const std::optional<std::vector<float>>& weights = std::nullopt);

    static std::optional<std::string> problem(
        int degree, const std::vector<Point>& controlPoints, const std::vector<float>& knots,
        const std::optional<std::vector<float>>& weights);

    static bool weightedStepsAreFinite(
        int degree, const std::vector<Point>& controlPoints, const std::optional<std::vector<float>>& weights);

    static std::optional<std::vector<float>> clampedUniformKnots(int degree, int controlPointCount);

    // MARK: Periodic (closed) splines

    static std::optional<BSpline> periodic(
        int degree, const std::vector<Point>& controlPoints, const std::optional<std::vector<float>>& weights = std::nullopt);

    ParameterRange parameterRange() const;
    bool isClampedAtStart() const;
    bool isClampedAtEnd() const;

    // MARK: Spans and basis functions

    int span(float parameter) const;
    BasisFunctionValues basisFunctions(float parameter) const;

    // MARK: de Boor's algorithm

    Point point(float parameter) const;
    Point firstDerivative(float parameter) const;

    // MARK: Knot insertion

    std::optional<BSpline> insertingKnot(float knot) const;

    // MARK: Bézier pieces

    std::optional<std::vector<CubicBezier<Point>>> cubicBezierPieces() const;
    std::optional<std::vector<CubicBezier<Point>>> unclampedCubicBezierPieces() const;

    // MARK: 2D conveniences

    std::optional<Path2D> path() const requires std::same_as<PointType, Vector2>;

    static std::optional<BSpline> circle(Vector2 centre, float radius)
        requires std::same_as<PointType, Vector2>;

    static std::optional<Path2D> closedPath(const std::vector<Vector2>& controlPoints)
        requires std::same_as<PointType, Vector2>;

private:
    BSpline() = default;

    struct DeBoorState {
        int span;
        Point origin;
        std::vector<Point> weightedPoints;
        std::vector<float> pointWeights;
    };

    DeBoorState deBoor(float parameter, int rounds) const;
    int lastNonEmptySpan() const;
    Point blossom(int span, const std::array<float, 3>& arguments) const;
};

} // namespace editor
