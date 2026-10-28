// A straight piece of line in the plane (Engine/Math/Primitives/Segment2.swift).
#pragma once

#include <optional>
#include <vector>

#include "../Linear/Vector2.h"

namespace editor {

/// The nearest point of a segment to some point, and how far along the segment it lies.
struct SegmentFoot {
    /// The point on the segment.
    Vector2 point;
    /// 0 at the segment's start, 1 at its end.
    float parameter = 0;
};

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

    /// The point on the segment nearest to `point`, with how far along the segment it lies.
    ///
    /// How: parameter = (point − start)·direction / direction·direction held in 0…1, nearest =
    /// start + direction × parameter (see `closestPoint`). parameter 0 is `start`, 1 is `end`.
    /// Edge case: if start == end the parameter is 0.
    ///
    /// Use when: snapping a point onto a side and knowing where on the side it landed
    /// Tags: nearest point with parameter, project onto segment, foot of perpendicular, t value
    /// Node: planned (closestPoint)
    SegmentFoot closestFoot(Vector2 point) const;

    /// Whether `point` lies within `tolerance` of the line through the segment, and at most
    /// `overshoot` (default `tolerance`) beyond either end.
    ///
    /// How: with direction = end − start and length L, the sideways distance is
    /// |(point − start) × direction| / L, and the distances along the line are
    /// (point − start)·direction / L (at least −overshoot) and (point − end)·direction / L (at most
    /// overshoot). A segment shorter than 1e-9 has no line and nothing is near it.
    ///
    ///        overshoot  ┌─────────── tolerance band ───────────┐  overshoot
    ///               ····start ───────────────────────────── end····
    ///
    /// Source: graphics_cpp curves/geometry `nearSegment`.
    ///
    /// Use when: deciding whether a click is on a side of an outline, with a margin
    /// Tags: near segment, hit test, point on line within tolerance, pick segment, snap
    /// Node: none (hit test)
    bool isNear(Vector2 point, float tolerance, std::optional<float> overshoot = std::nullopt) const;

    /// `segments` with each segment's start moved onto the end of the one before it, round the
    /// loop: the first starts where the last ends. Closes the small gaps that rounding leaves in a
    /// closed outline. An empty list stays empty.
    ///
    /// Source: graphics_cpp curves/geometry `stitch`.
    ///
    /// Use when: sides of a closed outline were computed separately and must meet exactly
    /// Tags: stitch, close gaps, snap ends together, join segments, weld loop
    /// Node: planned (closeOutline)
    static std::vector<Segment2> stitchedLoop(const std::vector<Segment2>& segments);
};

} // namespace editor
