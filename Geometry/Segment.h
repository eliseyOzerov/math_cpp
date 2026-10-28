// A straight piece of line in 3D (Engine/Math/Primitives/Segment.swift).
#pragma once

#include "../Linear/Vector3.h"

namespace editor {

/// The straight line between `start` and `end`, end points included. Road centrelines, mesh
/// edges and triangle sides are segments.
///
/// Use when: describing a road edge, a bone or a line between two points
/// Tags: line segment, edge, bone, two points, segment
/// Node: capsuleField, taperedCapsuleField
struct Segment {
    Vector3 start;
    Vector3 end;

    bool operator==(const Segment&) const = default;

    /// The point on the segment nearest to `point`.
    ///
    /// Source: Ericson, *Real-Time Collision Detection* (2005) §5.1.2.
    ///
    /// How: write every point of the segment as start + direction × t, where direction = end −
    /// start and t runs from 0 (at `start`) to 1 (at `end`). The distance to `point` is smallest
    /// where the vector from the segment to `point` is at right angles to `direction`, which gives
    /// t = (point − start)·direction / direction·direction. That t is for the infinite line, so it
    /// is clamped into 0…1 to stay on the segment.
    ///
    /// Edge case: if start == end the segment is a single point, direction·direction is 0, and
    /// `start` is returned.
    ///
    /// Use when: picking the object under the cursor; snapping a point onto a road's centre line
    /// Tags: closest point on segment, nearest point, project onto line, distance to segment, snapping
    /// Node: inside closestPoint
    Vector3 closestPoint(Vector3 point) const;
};

} // namespace editor
