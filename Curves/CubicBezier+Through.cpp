// Curves made to pass through a point: a quadratic or a cubic through three points, and molding
// (dragging a point on an existing cubic). All use the projection identity of Pomax, "A Primer
// on Bézier Curves" (https://pomax.github.io/bezierinfo), sections "Curve fitting" and "Molding
// a curve"; ported from graphics_cpp's bezier.cpp (quadThrough, cubicThrough, mold).
#include <algorithm>
#include <cmath>

#include "CubicBezier.h"

namespace editor {

namespace {

/// Where a curve's point B at parameter t sits against the point C on its chord and the
/// de Casteljau point A above it (the Primer's ratio and u functions).
struct ProjectionIdentity {
    Vector2 apex;
    Vector2 chordPoint;
};

/// For a curve of `degree` 2 or 3 from `start` to `end` through `through` at `parameter`:
/// u(t) = (1−t)ⁿ / (tⁿ + (1−t)ⁿ), ratio(t) = |(tⁿ + (1−t)ⁿ − 1) / (tⁿ + (1−t)ⁿ)|,
/// C = u·start + (1−u)·end, and A = B + (B − C)/ratio.
ProjectionIdentity projectionIdentity(int degree, Vector2 start, Vector2 through, Vector2 end, float parameter) {
    float parameterPower = std::pow(parameter, static_cast<float>(degree));
    float complementPower = std::pow(1 - parameter, static_cast<float>(degree));
    float startShare = complementPower / (parameterPower + complementPower);
    float ratio = std::fabs((parameterPower + complementPower - 1) / (parameterPower + complementPower));
    Vector2 chordPoint = Vector2::linearBlend(end, start, startShare);
    return ProjectionIdentity{through + (through - chordPoint) / ratio, chordPoint};
}

/// The share along the curve a point is likely at, guessed from its distances to the ends:
/// |start→through| / (|start→through| + |through→end|), ½ if both are zero.
float guessedParameter(Vector2 start, Vector2 through, Vector2 end) {
    float before = start.distance(through);
    float after = through.distance(end);
    return before + after > 0 ? before / (before + after) : 0.5f;
}

/// Keeps a parameter away from 0 and 1, where the identity divides by zero.
float awayFromEnds(float parameter) { return std::fmin(1 - 1e-3f, std::fmax(1e-3f, parameter)); }

/// The cubic from `start` to `end` whose de Casteljau points at `parameter` are `apex` (the
/// middle of the second level) and `before` and `after` (either side of the curve's point): the
/// construction run backwards.
CubicBezier<Vector2> fromStruts(
    Vector2 start, Vector2 end, float parameter, Vector2 apex, Vector2 before, Vector2 after) {
    Vector2 firstLevelTwo = apex + (before - apex) / (1 - parameter);
    Vector2 secondLevelTwo = apex + (after - apex) / parameter;
    return CubicBezier<Vector2>(
        start, start + (firstLevelTwo - start) / parameter, end + (secondLevelTwo - end) / (1 - parameter), end);
}

} // namespace

/// The quadratic Bézier from `start` to `end` through `through`, as a cubic.
///
/// How: with the parameter t at `through` (guessed from distances unless given, kept in
/// 0.001…0.999), the single control point of the quadratic is A = B + (B − C)/ratio from the
/// projection identity with degree 2. The cubic raises the degree exactly: handles at
/// two thirds of the way from each end to A.
///
/// Source: Pomax, "A Primer on Bézier Curves", fitting a quadratic through three points.
///
/// Use when: drawing a smooth arc through a clicked point; a simple bend between two nodes
/// Tags: quadratic through three points, curve through point, fit three points, degree elevation
/// Node: planned (curveThroughPoint)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::quadraticThrough(
    Vector2 start, Vector2 through, Vector2 end, std::optional<float> parameter)
    requires std::same_as<PointType, Vector2>
{
    float at = awayFromEnds(parameter.value_or(guessedParameter(start, through, end)));
    Vector2 control = projectionIdentity(2, start, through, end, at).apex;
    return CubicBezier<Vector2>(
        start, Vector2::linearBlend(start, control, 2.0f / 3), Vector2::linearBlend(end, control, 2.0f / 3), end);
}

/// The cubic Bézier from `start` to `end` through `through`, shaped by `options`.
///
/// How: with t the parameter at `through` (default guessed from distances, kept in 0.001…0.999)
/// and A from the projection identity with degree 3, the de Casteljau points at t either side of
/// the curve point lie along the direction `options.direction` (default: start to end), `l` behind
/// it and `l·(1 − t)/t` ahead, where `l` is `options.strutLength` (default t·(|start→through| +
/// |through→end|)/4, so the struts together are a quarter of the path through the point). The
/// construction is run backwards to get the handles (Bezier.js's cubicFromPoints).
///
/// Conventions: if start equals end and no direction is given the struts are zero-length and
/// the result is degenerate.
///
/// Source: Pomax, "A Primer on Bézier Curves"; Bezier.js cubicFromPoints.
///
/// Use when: dragging a curve through a point with control over the direction there
/// Tags: cubic through three points, curve fitting, cubic from points, strut length, pass through
/// Node: planned (curveThroughPoint)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::cubicThrough(
    Vector2 start, Vector2 through, Vector2 end, const ThroughOptions& options)
    requires std::same_as<PointType, Vector2>
{
    float at = awayFromEnds(options.parameter.value_or(guessedParameter(start, through, end)));
    Vector2 apex = projectionIdentity(3, start, through, end, at).apex;
    float strut = options.strutLength.value_or(at * (start.distance(through) + through.distance(end)) / 4);
    Vector2 direction = options.direction.value_or(end - start).normalized();
    return fromStruts(start, end, at, apex, through - direction * strut, through + direction * (strut * (1 - at) / at));
}

/// The curve with its point at `parameter` dragged to `destination`: ends kept, handles moved so
/// the curve passes through the new place there.
///
/// How: the de Casteljau points at t give the point B, and the two level-two points around it;
/// the move d = destination − B is added to those, the new apex comes from the projection identity
/// (A = destination + (destination − C)/ratio, with C from the original ends), and the cubic is
/// rebuilt from the struts (Primer's molding). It is affine, so it works in any units.
///
/// Conventions: the parameter is kept in 0.001…0.999; the curve's ends never move.
///
/// Source: Pomax, "A Primer on Bézier Curves", "Moulding a curve".
///
/// Use when: dragging a curve by a point on it, as in a vector editor; reshaping a road in place
/// Tags: mold curve, drag point on curve, reshape bezier, direct manipulation
/// Node: planned (moldCurve)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::molded(float parameter, Vector2 destination) const
    requires std::same_as<PointType, Vector2>
{
    float at = awayFromEnds(parameter);
    std::array<Vector2, 10> levels = deCasteljauPoints(at);
    Vector2 movement = destination - levels[9];
    Vector2 chordPoint = projectionIdentity(3, start, levels[9], end, at).chordPoint;
    float parameterPower = std::pow(at, 3.0f);
    float complementPower = std::pow(1 - at, 3.0f);
    float ratio = std::fabs((parameterPower + complementPower - 1) / (parameterPower + complementPower));
    Vector2 apex = destination + (destination - chordPoint) / ratio;
    return fromStruts(start, end, at, apex, levels[7] + movement, levels[8] + movement);
}

template class CubicBezier<Vector2>;

} // namespace editor
