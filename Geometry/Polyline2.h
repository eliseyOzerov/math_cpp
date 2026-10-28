// Measures along a chain of points in the plane: running lengths, a point at a distance, and
// the total length (graphics_cpp curves/geometry `runLengths`, `pointAlong`, `hullLength`).
#pragma once

#include <vector>

#include "../Linear/Vector2.h"

namespace editor {

/// A chain of points joined by straight lines, in order. Not closed: the last point does not
/// join the first.
///
/// Use when: measuring a flattened curve; walking along it at an even pace; the control polygon
///   of a Bezier piece (4 points)
/// Tags: polyline, polygonal chain, linestring, point list, path length, control polygon
/// Node: planned (polylineLength)
struct Polyline2 {
    std::vector<Vector2> points;

    bool operator==(const Polyline2&) const = default;

    /// The running lengths: 0 at the first point, then the distance travelled along the chain
    /// to each next point (a non-decreasing list, one number per point). Empty for no points.
    ///
    /// How: run[0] = 0 and run[i] = run[i − 1] + |points[i] − points[i − 1]|.
    ///
    /// Use when: preparing `pointAlong`; spacing samples by length
    /// Tags: running length, cumulative distance, arc length table, chord length parameterization
    /// Node: planned (polylineLength)
    std::vector<float> runLengths() const;

    /// The point `distance` along the chain, given its `runLengths`; the ends hold for a distance
    /// below 0 or past the total.
    ///
    /// How: `distance` is held in 0…total, the first piece whose running length reaches it is
    /// found, and the point is blended between that piece's ends by (distance − run before) /
    /// (piece length). A piece of zero length gives its end point. One point is returned as is.
    /// Throws a `MathProblem` for no points, or for `runLengths` of another size than the points.
    ///
    ///     points[0] ●────●──── ✕ ───●        distance counted from points[0]
    ///
    /// Use when: placing marks at even distances along a flattened path
    /// Tags: point along polyline, sample by arc length, walk along path, interpolate along
    /// Node: planned (pointAlongPath)
    Vector2 pointAlong(const std::vector<float>& runLengths, float distance) const;

    /// The total length of the chain: the sum of the distances between neighbouring points
    /// (0 for fewer than two points). For the four control points of a cubic Bezier this is its
    /// hull length: never less than the curve's arc length, and quick to find.
    ///
    /// Use when: a cheap upper bound on a curve's length; the length of a flattened path
    /// Tags: hull length, control polygon length, polyline length, perimeter, path length
    /// Node: planned (polylineLength)
    float length() const;
};

} // namespace editor
