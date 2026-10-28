// A straight piece of line in the plane (Engine/Math/Primitives/Segment2.swift).
#pragma once

#include "../Linear/Vector2.h"

namespace editor {

/// The straight line between `start` and `end` in the plane, end points included: the 2D twin of
/// `Segment`.
///
/// Use when: measuring how far a point is from a side of a flat outline
/// Tags: 2D segment, line segment, polygon side, edge 2D, two points
/// Node: none (2D segment for profile outlines)
struct Segment2 {
    Vector2 start;
    Vector2 end;

    bool operator==(const Segment2&) const = default;

    /// The point on the segment nearest to `point`.
    ///
    /// Source: Ericson, *Real-Time Collision Detection* (2005) §5.1.2, as in `Segment`.
    ///
    /// How: with direction = end − start, the foot of the perpendicular from `point` is at
    /// t = (point − start)·direction / direction·direction, clamped into 0…1 to stay on the
    /// segment, and the nearest point is start + direction × t.
    ///
    /// Edge case: if start == end the segment is a single point and `start` is returned.
    ///
    /// Use when: snapping a point onto a side of a 2D outline
    /// Tags: closest point on segment 2D, nearest point, project onto segment, foot of perpendicular
    /// Node: none (2D form of closestPoint, for profile outlines)
    Vector2 closestPoint(Vector2 point) const;

    /// The distance from `point` to the nearest point of the segment (see `closestPoint`).
    ///
    /// Use when: the unsigned distance from a point to a side of an outline; the sides of a 2D signed distance
    /// Tags: distance to segment 2D, point segment distance, sdSegment, udSegment
    /// Node: inside extrudedOutlineField
    float distance(Vector2 point) const;
};

} // namespace editor
