// Offsets whose distance is a function along the curve, as Bezier.js's `scale` and `offset` make
// them (graphics_cpp curves/offset): a simple piece is moved out along its normals with its
// handles placed where the moved end tangents meet the old handle rays, and a whole curve is cut
// into simple pieces first. `CubicBezier::offset` in CubicBezier+Offset.cpp is the
// tolerance-driven version with a distance that is linear in the parameter.
//
// Sign: like `offset`, a positive distance is to the RIGHT of the direction of travel.
#include <cmath>
#include <optional>

#include "../Geometry/Line2.h"
#include "../Linear/Scalar.h"
#include "CubicBezier.h"

namespace editor {

namespace {

/// Where the line from `origin` along `direction` meets the line from `otherOrigin` along
/// `otherDirection`, as a point; nothing when they are parallel (relative to the directions'
/// lengths, so the test holds at any scale).
std::optional<Vector2> meetingPoint(Vector2 origin, Vector2 direction, Vector2 otherOrigin, Vector2 otherDirection) {
    float flat = 1e-6f * direction.length() * otherDirection.length();
    std::optional<LineMeeting> meeting = Line2{origin, direction}.meets(Line2{otherOrigin, otherDirection}, flat);
    if (!meeting) return std::nullopt;
    return origin + direction * meeting->alongFirst;
}

/// The unit normal to the right of the direction of travel (the side of a positive distance).
Vector2 rightUnitNormal(const CubicBezier<Vector2>& curve, float parameter) { return -curve.unitNormal(parameter); }

} // namespace

/// The point at `parameter` moved sideways by `distance`: `point + distance · rightNormal`.
///
/// Conventions: y-up plane; positive is to the right of the direction of travel, negative to
/// the left, as in `offset`. At a cusp the direction falls back as in `tangentDirection`.
///
/// Use when: placing a rivet, a lane marking or a label beside a curve
/// Tags: offset point, point beside curve, offset at, normal offset, lane position
/// Node: planned (offsetPointOnCurve)
template <CurvePoint PointType>
Vector2 CubicBezier<PointType>::pointOffsetBy(float parameter, float distance) const
    requires std::same_as<PointType, Vector2>
{
    return point(parameter) + rightUnitNormal(*this, parameter) * distance;
}

/// A piece moved out by the same `distance` along its normals: its ends exactly `distance` from the
/// ends of the piece, and its handles where the old handle rays meet the moved end tangents.
///
/// How: the normals at the two ends are lines that meet in a point O (the middle of the
/// piece's bend). Each handle is a point on the ray from O through the old handle; the moved
/// handle is that ray's meeting with the tangent line through the moved end:
///
///          O  ← where the end normals meet
///         ╱ ╲
///      h1'   h2'    the handles slide out along the rays from O
///      e0     e3    the ends move out along their normals
///
/// Scaling about O is exact for a circular arc, and a good fit for any simple piece. A
/// straight piece, or one whose end normals are parallel (an S bend's two ends facing the same
/// way, a straight cubic), has no O: the ends and handles are moved along their own normals.
/// Where a handle ray and tangent are parallel the handle is moved along the end normal.
///
/// Conventions: y-up plane; positive is to the right of travel. Meant for a simple piece
/// (`isSimpleForOffset`): a longer or twistier curve gets a poor fit, so cut it first
/// (`offsetByReach` does).
///
/// Source: graphics_cpp curves/offset `scale` (after Pomax, *A Primer on Bézier Curves*,
/// "Curve offsetting").
///
/// Use when: moving a gentle curve sideways in one step, such as a road's centre line to its kerb
/// Tags: scale curve, offset one piece, Bezier.js scale, move along normals, parallel curve
/// Node: planned (offsetCurvePiece)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::scaledOut(float distance) const
    requires std::same_as<PointType, Vector2>
{
    Vector2 startNormal = rightUnitNormal(*this, 0);
    Vector2 endNormal = rightUnitNormal(*this, 1);
    Vector2 movedStart = start + startNormal * distance;
    Vector2 movedEnd = end + endNormal * distance;
    std::optional<Vector2> meeting;
    if (!isStraight()) meeting = meetingPoint(start, startNormal, end, endNormal);
    CubicBezier<Vector2> alongNormals(movedStart, startHandle + startNormal * distance, endHandle + endNormal * distance, movedEnd);
    if (!meeting) return alongNormals;
    std::optional<Vector2> newStartHandle = meetingPoint(movedStart, tangentDirection(0), *meeting, startHandle - *meeting);
    std::optional<Vector2> newEndHandle = meetingPoint(movedEnd, tangentDirection(1), *meeting, endHandle - *meeting);
    if (!newStartHandle || !newEndHandle) return alongNormals;
    return CubicBezier<Vector2>(movedStart, *newStartHandle, *newEndHandle, movedEnd);
}

/// A piece moved out by a distance that changes along it: `reach(0)` at its start to `reach(1)`
/// at its end, with the handles moved by `reach(⅓)` and `reach(⅔)`.
///
/// How: the ends move exactly (`end + reach · rightNormal`). Each handle moves away from the
/// meeting point O of the end normals (see `scaledOut(distance)`) along its ray from O, by the
/// reach at its third, outwards when the end normal points away from O and inwards when it
/// points towards it. A straight piece or parallel normals move each handle along its end normal.
///
/// Conventions: y-up plane; positive is to the right of travel; `reach` takes a parameter of
/// this piece, 0…1.
///
/// Source: graphics_cpp curves/offset `scale` with a graduated distance.
///
/// Use when: a piece of a road that widens, or one edge of a tapering stroke
/// Tags: graduated offset, scale with function, variable width, tapered offset
/// Node: planned (offsetCurvePiece)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::scaledOut(const OffsetReach& reach) const
    requires std::same_as<PointType, Vector2>
{
    Vector2 startNormal = rightUnitNormal(*this, 0);
    Vector2 endNormal = rightUnitNormal(*this, 1);
    float startReach = reach(0);
    float endReach = reach(1);
    float startHandleReach = reach(1.0f / 3);
    float endHandleReach = reach(2.0f / 3);
    Vector2 movedStart = start + startNormal * startReach;
    Vector2 movedEnd = end + endNormal * endReach;
    std::optional<Vector2> meeting;
    if (!isStraight()) meeting = meetingPoint(start, startNormal, end, endNormal);
    if (!meeting) {
        return CubicBezier<Vector2>(movedStart, startHandle + startNormal * startHandleReach,
                                    endHandle + endNormal * endHandleReach, movedEnd);
    }
    auto movedHandle = [&](Vector2 handle, Vector2 normal, float handleReach) {
        Vector2 outward = handle - *meeting;
        if (outward.length() < 1e-6f) return handle + normal * handleReach;
        Vector2 direction = outward.normalized();
        return handle + direction * (direction.dot(normal) >= 0 ? handleReach : -handleReach);
    };
    return CubicBezier<Vector2>(movedStart, movedHandle(startHandle, startNormal, startHandleReach),
                                movedHandle(endHandle, endNormal, endHandleReach), movedEnd);
}

/// The curve moved sideways by `reach(t)` at parameter `t` of the whole curve, as cubics.
///
/// How: the curve is cut into simple pieces (`simplePieceParameters`: at its inflections, then
/// halved until each bends gently), and each piece is moved with `scaledOut(reach)`, its reach
/// re-read over the parameters the piece covers on the whole curve: piece `[a, b]` calls
/// `reach(a + (b − a)·u)`. The pieces join end to end only where the reach is continuous; the
/// end of one piece is `reach(b)` out from the same point the next starts `reach(b)` out from.
///
/// Conventions: y-up plane; positive is to the right of travel. No tolerance: the fit is the
/// handle-scaling of `scaledOut`, as in Bezier.js; use `offset(start, end, tolerance)` when the
/// distance is linear and a measured error matters.
///
/// Source: graphics_cpp curves/offset `offset` (Bezier.js `offset`).
///
/// Use when: the edge of a stroke whose width swells along it (a brush stroke, a river bank)
/// Tags: variable offset, graduated offset, offset by function, tapered stroke edge, Bezier.js offset
/// Node: planned (offsetProfile)
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::offsetByReach(const OffsetReach& reach) const
    requires std::same_as<PointType, Vector2>
{
    std::vector<CubicBezier<Vector2>> moved;
    for (ParameterRange range : simplePieceParameters()) {
        float lower = range.lowerBound;
        float width = range.upperBound - range.lowerBound;
        OffsetReach overPiece = [&reach, lower, width](float parameter) { return reach(lower + width * parameter); };
        moved.push_back(portion(range.lowerBound, range.upperBound).scaledOut(overPiece));
    }
    return moved;
}

template class CubicBezier<Vector2>;

} // namespace editor
