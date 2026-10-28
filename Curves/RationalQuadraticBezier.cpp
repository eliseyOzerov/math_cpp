// Rational quadratic Béziers: exact conics (Engine/Math/Curves/RationalQuadraticBezier.swift).
#include "RationalQuadraticBezier.h"

#include <cmath>

#include "../Linear/Angle.h"
#include "../Linear/Scalar.h"

namespace editor {

namespace {
// Swift's `Float.pi`: rounded toward zero, one step below the nearest float (Angle keeps its own private).
} // namespace

/// A curve, or an empty optional (Swift's `nil`) if a point is not finite, the weight is not
/// positive and finite, or the numbers the evaluation works with, `(control − start)·w` and
/// `end − start`, overflow.
template <CurvePoint PointType>
std::optional<RationalQuadraticBezier<PointType>> RationalQuadraticBezier<PointType>::make(
    Point start, Point control, Point end, float controlWeight) {
    if (!(controlWeight > 0 && std::isfinite(controlWeight)
          && start.isFinite() && control.isFinite() && end.isFinite()
          && ((control - start) * controlWeight).isFinite() && (end - start).isFinite())) {
        return std::nullopt;
    }
    RationalQuadraticBezier curve;
    curve.start = start;
    curve.control = control;
    curve.end = end;
    curve.controlWeight = controlWeight;
    return curve;
}

/// The kind of conic: by the weight alone, `ellipse` below 1, `parabola` at exactly 1,
/// `hyperbola` above 1.
///
/// With the end weights equal to 1, the conic's type depends only on how `controlWeight`
/// compares with 1: closed arc below, parabola at, open arc above. Three collinear points
/// give a straight piece whatever the weight; this property does not look at the points. The
/// comparison with 1 is exact, so a circle arc from `circularArc` with a sweep below about
/// 0.03° is a `.parabola`: its weight cos(θ/2) is within half a Float step of 1 when
/// θ/2 < about 2.4e-4 radians, and rounds to exactly 1. Such an arc is still on its circle to
/// within Float rounding, since a parabola and a circle that short differ by about r·θ⁴/1000.
///
/// Source: Farin, *Curves and Surfaces for CAGD* (2002), on rational quadratic conics;
/// Piegl & Tiller, *The NURBS Book* (1997), ch. 7.
///
/// Use when: classifying a conic arc; deciding how to export a weighted quadratic
/// Tags: conic classification, ellipse parabola hyperbola, shoulder weight, conic type
/// Node: curveConic
template <CurvePoint PointType>
ConicType RationalQuadraticBezier<PointType>::conicType() const {
    if (controlWeight < 1) return ConicType::ellipse;
    if (controlWeight == 1) return ConicType::parabola;
    return ConicType::hyperbola;
}

/// The point at `parameter`, by de Casteljau's algorithm on the homogeneous points and a
/// division by the blended weight.
///
/// How: blend `P₀` with `w·P₁` and `w·P₁` with `P₂` (and the weights 1, w, 1 the same way),
/// blend the two results, and divide the point by the weight. At parameter 0 and 1 the stored
/// end points are returned unchanged. The points are measured from `start` and it is added back
/// at the end (`C = P₀ + Σ bᵢwᵢ(Pᵢ − P₀) / Σ bᵢwᵢ`, the same since the weighted Bernstein terms
/// divided by their sum add up to 1), so an arc far from the origin is as accurate as one near
/// it, up to the rounding of that last addition.
///
/// Use when: sampling the arc to draw it or to place objects along it
/// Tags: rational Bezier point, conic point, evaluate conic, NURBS piece point
/// Node: inside curveConic
template <CurvePoint PointType>
PointType RationalQuadraticBezier<PointType>::point(float parameter) const {
    if (parameter == 0) return start;
    if (parameter == 1) return end;
    return start + relativePoint(parameter);
}

/// The point at `parameter` minus `start`, by de Casteljau on `(0, 1)`, `((P₁ − P₀)·w, w)`,
/// `(P₂ − P₀, 1)` and a division by the blended weight.
template <CurvePoint PointType>
PointType RationalQuadraticBezier<PointType>::relativePoint(float parameter) const {
    Point origin = start * 0.0f;
    Point weightedControl = (control - start) * controlWeight;
    Point firstBlend = Point::linearBlend(origin, weightedControl, parameter);
    Point secondBlend = Point::linearBlend(weightedControl, end - start, parameter);
    float firstWeight = linearBlend(1, controlWeight, parameter);
    float secondWeight = linearBlend(controlWeight, 1, parameter);
    Point weightedPoint = Point::linearBlend(firstBlend, secondBlend, parameter);
    float weight = linearBlend(firstWeight, secondWeight, parameter);
    return weightedPoint * (1 / weight);
}

/// The velocity at `parameter`.
///
/// How: the quotient rule on `C = A / W`, where `A` is the quadratic Bézier curve of
/// `(P₀, w·P₁, P₂)` and `W` that of `(1, w, 1)`: `C′ = (A′ − W′·C) / W`, with
/// `A′ = 2·linearBlend(w·P₁ − P₀, P₂ − w·P₁, t)` and `W′ = 2·linearBlend(w − 1, 1 − w, t)`, the
/// derivatives of quadratic Bézier curves from their steps. At 0 it is `2w·(P₁ − P₀)`, at 1 it is
/// `2w·(P₂ − P₁)`. Measured from `start`, as in `point(at:)`: moving the curve does not change
/// its velocity.
///
/// Use when: the direction of a conic arc; the normal of an exact circle arc
/// Tags: conic tangent, rational Bezier derivative, arc direction, velocity
/// Node: none (a Curve requirement; graph curves are drawn as points, so nothing reads it)
template <CurvePoint PointType>
PointType RationalQuadraticBezier<PointType>::firstDerivative(float parameter) const {
    Point weightedControl = (control - start) * controlWeight;
    Point weightedStep = Point::linearBlend(weightedControl, (end - start) - weightedControl, parameter) * 2;
    float weightStep = linearBlend(controlWeight - 1, 1 - controlWeight, parameter) * 2;
    float weight = linearBlend(linearBlend(1, controlWeight, parameter), linearBlend(controlWeight, 1, parameter), parameter);
    return (weightedStep - relativePoint(parameter) * weightStep) * (1 / weight);
}

/// The circle arc around `centre` with `radius`, from `startAngle` turning by `sweep`
/// (counterclockwise for a positive sweep), as **one exact** rational quadratic piece. Empty
/// (Swift's `nil`) when the sweep is zero, not finite, or 180° or more, or the radius or centre
/// is not finite and positive / finite. A sweep below about 0.03° gets a weight of exactly 1 and
/// so reports `conicType() == ConicType::parabola` (see `conicType`); its points are still on the
/// circle.
///
/// How: for an arc of angle θ with the chord between the end points, the two tangent lines at
/// the ends meet at the control point, at distance `r / cos(θ/2)` from the centre on the middle
/// ray; the weight that puts the curve exactly on the circle is `w = cos(θ/2)`. Check at the
/// middle, with h = θ/2, the end points at `r·(cos h, ∓ sin h)` and `P₁ = (r/cos h, 0)`: the
/// numerator of the formula at t = ½ has x = ¼·2r·cos h + ½·cos h·r/cos h = r(cos h + 1)/2 and the
/// denominator is ¼ + ½·cos h + ¼ = (1 + cos h)/2, so the point is `(r, 0)`, on the circle. A full
/// circle takes four such pieces (see `BSpline.circle`).
///
/// ```
///            control ●
///                   ╱ ╲        weight cos(θ/2) < 1: an ellipse arc, here a circle
///           start ●╱   ╲● end
///                 ╲_____╱
/// ```
///
/// Accuracy: the points are on the circle up to Float rounding of the formula (about 1e-7·r
/// for a 90° arc); near 180° the control point is far away (`r / cos(θ/2)`), and the weight small, so
/// the error grows; use pieces of at most 90° for best accuracy.
///
/// Source: Piegl & Tiller, *The NURBS Book* (1997), ch. 7; Farin (2002).
///
/// Use when: an exact arc; a circle that stays a circle under scaling and rotation
/// Tags: exact circle arc, rational arc, conic arc, circle as NURBS, weight cos half angle
/// Node: none (profileArc draws arcs with cubic pieces)
template <CurvePoint PointType>
std::optional<RationalQuadraticBezier<PointType>> RationalQuadraticBezier<PointType>::circularArc(
    Vector2 centre, float radius, Angle startAngle, Angle sweep)
    requires std::same_as<PointType, Vector2>
{
    if (!(radius > 0 && std::isfinite(radius) && std::isfinite(centre.x) && std::isfinite(centre.y)
          && std::isfinite(sweep.radians) && sweep.radians != 0 && std::fabs(sweep.radians) < Angle::pi)) {
        return std::nullopt;
    }
    float weight = (sweep / 2).cosine();
    Angle middle = startAngle + sweep / 2;
    Angle endAngle = startAngle + sweep;
    return make(
        centre + Vector2(startAngle.cosine(), startAngle.sine()) * radius,
        centre + Vector2(middle.cosine(), middle.sine()) * (radius / weight),
        centre + Vector2(endAngle.cosine(), endAngle.sine()) * radius,
        weight);
}

template class RationalQuadraticBezier<Vector2>;
template class RationalQuadraticBezier<Vector3>;

} // namespace editor
