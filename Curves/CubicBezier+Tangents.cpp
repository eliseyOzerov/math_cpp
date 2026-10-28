// Directions at the ends of a flat cubic. Offsets, corner fillets and crossings all start from
// these: an offset moves the ends along the normals there, a corner is where the outgoing
// direction turns away from the incoming one (`Angle.signedTurn`)
// (Engine/Math/Curves/CubicBezier+Tangents.swift).
//
// Conventions: y up, counterclockwise turns positive, as everywhere in `Engine/Math`.
#include "../Linear/MathProblem.h"
#include "CubicBezier.h"

namespace editor {

/// The direction the curve leaves its start in, as a unit vector; (0, 0) when the curve is
/// a single point (or not finite).
///
/// How: the tangent at t = 0 is `3(P1 − P0)`. When the handle sits on the start the
/// tangent is zero, but the curve still leaves along `P2 − P0` (the next derivative), and
/// when that is zero too along `P3 − P0`. The first of these that is not zero is used.
///
/// Use when: the direction of a path at the start of a piece (corners, joins, offsets);
///   the tangent of a curve whose handle was dragged onto its end point
/// Tags: start tangent, initial direction, tangent at t=0, outgoing direction
/// Node: inside roundCorners, offsetProfile, fitProfile
template <CurvePoint PointType>
Vector2 CubicBezier<PointType>::startDirection() const
    requires std::same_as<PointType, Vector2>
{
    for (Vector2 target : {startHandle, endHandle, end}) {
        if (target == start) continue;
        if (std::optional<Vector2> direction = (target - start).unitDirection()) return *direction;
    }
    return Vector2();
}

/// The direction the curve arrives at its end in, as a unit vector; (0, 0) when the curve
/// is a single point (or not finite).
///
/// How: the mirror of `startDirection`: `P3 − P2`, else `P3 − P1`, else `P3 − P0`, the
/// first that is not zero.
///
/// Use when: the direction of a path at the end of a piece (corners, joins, offsets)
/// Tags: end tangent, final direction, tangent at t=1, incoming direction
/// Node: inside roundCorners, offsetProfile, fitProfile
template <CurvePoint PointType>
Vector2 CubicBezier<PointType>::endDirection() const
    requires std::same_as<PointType, Vector2>
{
    for (Vector2 source : {endHandle, startHandle, start}) {
        if (source == end) continue;
        if (std::optional<Vector2> direction = (end - source).unitDirection()) return *direction;
    }
    return Vector2();
}

/// The same curve moved by `offset`: every control point plus `offset`.
///
/// Use when: moving a curve near the origin before measuring it, and back afterwards
/// Tags: translate curve, move Bezier, shift control points
/// Node: none (2D helper)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::translated(Vector2 offset) const
    requires std::same_as<PointType, Vector2>
{
    return CubicBezier<Vector2>(start + offset, startHandle + offset, endHandle + offset, end + offset);
}

/// The part of the curve between parameters `lower` and `upper` (0 ≤ lower ≤ upper ≤ 1), as
/// its own cubic running from `point(at: lower)` to `point(at: upper)`.
///
/// How: split at `upper` and keep the first part, then split that at `lower / upper` and
/// keep the second part (de Casteljau both times). A bound of exactly 0 or 1 keeps the
/// stored end point, because no split happens there.
///
/// Use when: trimming a path piece back from a corner; the stretch of a curve between two
///   crossings
/// Tags: subcurve, trim curve, segment of Bezier, portion between parameters, blossom
/// Node: inside roundCorners
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::portion(float lower, float upper) const
    requires std::same_as<PointType, Vector2>
{
    if (!(0 <= lower && lower <= upper && upper <= 1)) throw MathProblem("A portion needs 0 ≤ lower ≤ upper ≤ 1.");
    CubicBezier<Vector2> head = upper < 1 ? split(upper).first : *this;
    if (!(lower > 0)) return head;
    if (!(lower < upper)) {
        Vector2 point = head.end;
        return CubicBezier<Vector2>(point, point, point, point);
    }
    return head.split(lower / upper).second;
}

template class CubicBezier<Vector2>;

} // namespace editor
