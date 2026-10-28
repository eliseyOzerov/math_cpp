// A half-plane of the plane: one side of a line (graphics_cpp curves/fit `Plane`).
#pragma once

#include <vector>

#include "../Linear/Vector2.h"

namespace editor {

/// One side of a line in the plane: the points whose signed distance from the line is ≥ 0.
///
/// Use when: cutting a stroke's end square; keeping the part of a shape on one side of a line
/// Tags: half plane, clip plane, line side, clipping, cut by line
/// Node: planned (clipByLine)
struct HalfPlane2 {
    /// A point on the boundary line.
    Vector2 point;
    /// The direction the kept side lies in, perpendicular to the line. Need not be unit length;
    /// distances scale with it.
    Vector2 normal;

    bool operator==(const HalfPlane2&) const = default;

    /// How far `position` is on the kept side of the line, in units of `normal`'s length:
    /// `(position − point) · normal`. Negative on the cut side, 0 on the line.
    ///
    /// Use when: asking which side of a cut a point is on
    /// Tags: signed distance to line, plane side, half plane test
    /// Node: planned (clipByLine)
    float signedDistance(Vector2 position) const { return (position - point).dot(normal); }

    /// Whether `position` is kept: on the line or on the side `normal` points to.
    ///
    /// Use when: filtering points by a cutting line
    /// Tags: inside half plane, point side, keep side
    /// Node: planned (clipByLine)
    bool contains(Vector2 position) const { return signedDistance(position) >= 0; }

    /// The part of the polygon `corners` that lies in the half-plane: Sutherland–Hodgman clipping
    /// against one edge.
    ///
    /// How: for each side from `a` to `b`, with signed distances `da`, `db`: keep `a` when
    /// `da ≥ 0`, and when the side crosses the line (one of `da`, `db` negative, the other not) add
    /// the crossing `a + (b − a)·da/(da − db)`.
    ///
    ///     kept │ cut         ●───●          a corner outside is replaced by the
    ///     ●────┼───●   →     │   │          two points where its sides cross the line
    ///          line
    ///
    /// Conventions: y-up plane; the order of corners is kept (a counterclockwise polygon stays
    /// counterclockwise). Right for convex polygons; a concave one may come back with
    /// zero-width bridges along the line. The result can have fewer than 3 corners, or none.
    ///
    /// Source: Sutherland & Hodgman, "Reentrant polygon clipping", CACM 17(1), 1974;
    /// graphics_cpp curves/fit `clipPolygon`.
    ///
    /// Use when: trimming a convex stamp at a stroke's end; cutting a convex shape in two
    /// Tags: polygon clipping, Sutherland-Hodgman, clip by plane, cut polygon, trim
    /// Node: planned (clipByLine)
    std::vector<Vector2> clipped(const std::vector<Vector2>& corners) const;
};

} // namespace editor
