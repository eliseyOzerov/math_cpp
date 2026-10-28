// A cubic Bézier can only approximate a circle. Give a quadratic Bézier a weight on its middle
// point and it can draw any conic section exactly: circle and ellipse arcs, parabolas and
// hyperbolas. It is the building block of the NURBS circle
// (Engine/Math/Curves/RationalQuadraticBezier.swift).
//
// Swift's failable initializer `init?` is the static function `make`, which returns an empty
// `std::optional` where Swift returns `nil`.
#pragma once

#include <concepts>
#include <optional>

#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"
#include "../Linear/Vector3.h"
#include "Curve.h"

namespace editor {

/// The kind of conic section a `RationalQuadraticBezier` is an arc of.
///
/// Use when: telling whether a weighted quadratic is an ellipse, parabola or hyperbola arc
/// Tags: conic type, ellipse, parabola, hyperbola, conic section kind
enum class ConicType {
    /// A weight below 1: a closed-curve arc (a circle arc at `cos(θ/2)`).
    ellipse,
    /// A weight of exactly 1: the ordinary quadratic Bézier curve. A circle arc of less than
    /// about 0.03° also lands here, because its weight cos(θ/2) rounds to exactly 1 in Float.
    parabola,
    /// A weight above 1.
    hyperbola,
};

/// A **rational quadratic Bézier curve**: three control points and a weight on the middle one.
/// The end points have weight 1. `Point` is `Vector2` or `Vector3`.
///
///                  control ●  (weight w)
///                         ╱ ╲
///                        ╱   ╲          the curve leaves `start` toward `control`
///                 start ●─ ╭──╮─● end    and arrives at `end` from `control`;
///                          ╰──╯          a larger w pulls it closer to `control`
///
/// Formula, with `u = 1 − t`:
///
///     C(t) = (u²·P₀ + 2w·u·t·P₁ + t²·P₂) / (u² + 2w·u·t + t²)
///
/// This is the ordinary quadratic Bézier curve of homogeneous points `(P₀, 1)`, `(w·P₁, w)`,
/// `(P₂, 1)`, divided by its last coordinate. With `w = 1` the denominator is `(u + t)² = 1` and the
/// formula is the usual quadratic Bézier curve. The code runs de Casteljau's algorithm on the
/// homogeneous points and divides at the end.
///
/// Which conic it is follows from the weight (`conicType`): w < 1 an ellipse arc, w = 1 a
/// parabola, w > 1 a hyperbola. If the three points are collinear it is a straight piece
/// whatever the weight. For the circle arc, `circularArc` gives the control point and the weight.
///
/// Conventions: parameter 0...1; outside it the same rational function continues, which can pass
/// through infinity where the denominator is zero (it cannot inside 0...1, since all three terms
/// are positive for w > 0). The weight must be positive and finite (`make` enforces it).
///
/// Source: Farin, *Curves and Surfaces for CAGD* (2002), on rational Bézier curves and conics;
/// Piegl & Tiller, *The NURBS Book* (1997), ch. 7 (conics and circles).
///
/// Use when: an exact circle or ellipse arc; a conic section as a Bézier piece; the pieces of a
///   NURBS circle
/// Tags: rational Bezier, conic Bezier, weighted quadratic, exact arc, ellipse, conic section,
///   NURBS piece, rational quadratic
/// Node: curveConic
template <CurvePoint PointType>
class RationalQuadraticBezier : public Curve<RationalQuadraticBezier<PointType>, PointType> {
public:
    using Point = PointType;

    Point start{};
    Point control{};
    Point end{};
    /// The weight of `control`; the end points have weight 1.
    float controlWeight = 1;

    bool operator==(const RationalQuadraticBezier&) const = default;

    static std::optional<RationalQuadraticBezier> make(Point start, Point control, Point end, float controlWeight);

    /// The parameter values of the curve: 0 at `start` to 1 at `end`.
    ///
    /// Use when: sampling the whole piece
    /// Tags: conic parameter range, 0 to 1
    /// Node: none (protocol plumbing)
    ParameterRange parameterRange() const { return ParameterRange{0, 1}; }

    ConicType conicType() const;
    Point point(float parameter) const;
    Point firstDerivative(float parameter) const;

    // MARK: 2D conveniences

    static std::optional<RationalQuadraticBezier> circularArc(Vector2 centre, float radius, Angle startAngle, Angle sweep)
        requires std::same_as<PointType, Vector2>;

private:
    RationalQuadraticBezier() = default;
    Point relativePoint(float parameter) const;
};

} // namespace editor
