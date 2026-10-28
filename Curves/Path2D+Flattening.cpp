// Turning a path into straight lines, for drawing, filling or measuring it
// (Engine/Math/Paths/Path2D+Flattening.swift).
#include <cmath>

#include "ArcLength.h"
#include "../Linear/Scalar.h"
#include "Path2D.h"

namespace editor {

/// The points of a polyline that follows the path to within `tolerance`.
///
/// How: each straight piece contributes its end point; each curve contributes the points
/// of `CubicBezier::flattened` (equal parameter steps, their number from the
/// interpolation error bound `0.75·M/n²`, so every point of the curve is within
/// `tolerance` of the polyline).
///
/// Conventions: the first point is `start`. For a closed path the last point is `start`
/// again, so the polyline can be drawn as a loop without special handling. When the
/// path already ends at its start up to rounding (a full circle of arcs ends 1.7e-7 away
/// for radius 1), that last point is replaced by `start` rather than followed by a
/// near-duplicate. `tolerance` is in the path's units and must be positive.
///
/// Use when: drawing a path; filling it as a polygon; measuring distance to it
/// Tags: flatten path, polyline, tessellate path, path to points, discretize outline,
///   curve subdivision
/// Node: inside roundCorners, chamferCorners
std::vector<Vector2> Path2D::flattened(float tolerance) const {
    std::vector<Vector2> points{start};
    for (const Edge& edge : edges()) {
        if (std::optional<CubicBezier<Vector2>> curve = edge.cubic()) {
            std::vector<Vector2> curvePoints = curve->flattened(tolerance);
            points.insert(points.end(), curvePoints.begin() + 1, curvePoints.end());
        } else {
            points.push_back(edge.segment.end());
        }
    }
    if (closed && !points.empty() && points.size() > 1) {
        Vector2 last = points.back();
        if (isSamePointUpToRounding(last, start)) {
            points[points.size() - 1] = start;
        } else {
            points.push_back(start);
        }
    }
    return points;
}

/// Whether two points differ by no more than the rounding of their coordinates: their
/// distance is at most `4·ulp` of their largest coordinate magnitude.
///
/// How: a point computed as a sum such as `centre + radius·direction` is known only to
/// about an ulp of its coordinates per operation; four ulps covers the few operations
/// of an arc's end point. Exactly equal points always count as the same.
///
/// Use when: deciding whether a path's last point is its start again
/// Tags: nearly equal points, ulp comparison, rounding duplicate, coincident points
bool Path2D::isSamePointUpToRounding(Vector2 first, Vector2 second) {
    float size = largerOf(
        largerOf(std::fabs(first.x), std::fabs(first.y)), largerOf(std::fabs(second.x), std::fabs(second.y)));
    return first.distance(second) <= 4 * curve_detail::unitInTheLastPlace(size);
}

} // namespace editor
