// An infinite line (Engine/Math/Primitives/Line.swift).
#pragma once

#include "../Linear/Vector3.h"

namespace editor {

/// A point on the line and the direction it runs in, forwards and backwards without end.
/// Where a `Segment` stops at its end points and a `Ray` at its origin, a `Line` has no ends:
/// it is the axis of a cone, the carrier of an edge, the thing a point is measured against.
///
/// Use when: measuring how far a point is from an axis; projecting a point onto the line through two others
/// Tags: infinite line, axis, point and direction, carrier line, line
/// Node: projectOntoLine
struct Line {
    /// A point the line passes through.
    Vector3 origin;
    /// The direction the line runs in, always length 1, so distances along it are in world units.
    Vector3 direction;

    /// Builds a line. The direction is normalized here, so callers can pass any non-zero finite
    /// vector, from the smallest float (1e-45) to the largest (3e38).
    ///
    /// How: the direction is normalized by `unitDirection`, which divides by the largest
    /// component first: squaring the components directly would underflow to 0 below about 1e-19
    /// and overflow to infinity above about 1.8e19. A zero-length direction has no heading, and
    /// one with an infinite or NaN component has none either, so both throw a `GeometryProblem`
    /// (Swift traps).
    Line(Vector3 origin, Vector3 direction);

    bool operator==(const Line&) const = default;

    /// How far along the line, from `origin` and in the direction of `direction`, the point
    /// nearest to `point` lies. Negative values are behind `origin`.
    ///
    /// How: (point − origin) · direction. With a unit direction, the dot product is the length
    /// of the shadow of `point − origin` on the line.
    ///
    /// Use when: finding where along an axis a point projects; splitting a point's offset into a part along an axis and a part across it
    /// Tags: project onto line, scalar projection, coordinate along axis, parameter on line, signed distance along
    /// Node: projectOntoLine
    float distanceAlong(Vector3 point) const;

    /// The point on the line nearest to `point`: the foot of the perpendicular.
    ///
    /// How: origin + direction × distanceAlong(point). The vector between the point and its foot
    /// is at right angles to `direction`. Unlike `Segment::closestPoint`, the result is not
    /// clamped: the line has no ends.
    ///
    /// Use when: snapping a point onto an axis; mirroring a point across a line
    /// Tags: closest point on line, project onto line, foot of perpendicular, nearest point, snap to axis
    /// Node: projectOntoLine
    Vector3 closestPoint(Vector3 point) const;

    /// The shortest distance from `point` to the line: the length of the part of
    /// `point − origin` that is *not* along the line.
    ///
    /// How: |(point − origin) − direction × ((point − origin) · direction)|. The bracket subtracts
    /// the shadow on the line and keeps the perpendicular remainder. Written as a subtraction of
    /// vectors, not as a square root of a difference of squares, so it stays accurate for a point
    /// very close to the line.
    ///
    /// Conventions: always zero or positive; zero for a point on the line. The error is relative
    /// to the distance from `point` to `origin`, not to the distance from the world's origin, so
    /// a line a kilometre out is measured as well as one through (0, 0, 0).
    ///
    /// Use when: measuring how far a point is from an axis, such as the radial part of a cone's distance; testing whether a pick is close to a long edge
    /// Tags: distance point to line, distance to axis, perpendicular distance, radial distance, point line distance
    /// Node: projectOntoLine
    float distance(Vector3 point) const;
};

} // namespace editor
