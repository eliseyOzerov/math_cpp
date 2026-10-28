// Small polyline helpers the booleans' callers need: scanline crossings for a fill, the narrowest
// place of an outline and the gap between two outlines. Ported from graphics_cpp curves/boolean.cpp
// (`edges`, `scanline`, `narrowest`, `polyGap`).
#include <limits>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "Boolean2D.h"

namespace editor {

/// The edges of closed polylines that a horizontal line can cross: every side except the level
/// ones, the last point joined back to the first.
///
/// Use when: preparing the edges for `scanlineCrossings` of a flattened shape
/// Tags: polygon edges, scanline edges, edge table, level edges dropped
/// Node: none (rasteriser helper)
std::vector<Segment2> scanlineEdges(const std::vector<std::vector<Vector2>>& closedPolylines) {
    std::vector<Segment2> edges;
    for (const std::vector<Vector2>& polyline : closedPolylines) {
        for (size_t index = 0; index < polyline.size(); index += 1) {
            Vector2 from = polyline[index];
            Vector2 to = polyline[(index + 1) % polyline.size()];
            if (from.y != to.y) edges.push_back(Segment2{from, to});
        }
    }
    return edges;
}

/// Where the horizontal line at `height` crosses `edges`, each as its x and its winding (+1 going
/// up, −1 going down), in the order of the edges (not sorted).
///
/// How: an edge counts from its lower end up to but not including its upper one (`(from.y ≤ height)
/// ≠ (to.y ≤ height)`), so a vertex is crossed once; x = from.x + (height − from.y)·Δx/Δy.
///
/// Use when: filling a shape row by row; measuring the width of a shape at one height
/// Tags: scanline, scan conversion, horizontal ray crossings, even-odd spans, row fill
/// Node: none (rasteriser helper)
std::vector<ScanCrossing> scanlineCrossings(const std::vector<Segment2>& edges, float height) {
    std::vector<ScanCrossing> crossings;
    for (const Segment2& edge : edges) {
        if ((edge.start.y <= height) == (edge.end.y <= height)) continue;
        float x = edge.start.x + (height - edge.start.y) * (edge.end.x - edge.start.x) / (edge.end.y - edge.start.y);
        crossings.push_back(ScanCrossing{x, edge.end.y > edge.start.y ? 1 : -1});
    }
    return crossings;
}

/// The narrowest a closed polyline comes across itself: the closest two of its points that lie at
/// least `minimumRunApart` along it either way round (so neighbours do not count), and the distance
/// between them. Nothing where no two points are that far apart along it.
///
/// How: the run length between two points is the shorter of the way along and the way back; every
/// pair is tried, O(n²), so flatten with a coarse step first.
///
/// Use when: finding where an outline pinches (an ink trap, a thin neck); checking a stroke's width
/// Tags: narrowest, pinch point, neck, thinnest part, medial distance, closest self distance
/// Node: planned (narrowestPoint)
std::optional<ClosestPair> narrowestPoint(const std::vector<Vector2>& closedPolyline, float minimumRunApart) {
    size_t count = closedPolyline.size();
    std::vector<float> runs{0};
    for (size_t index = 1; index <= count; index += 1) {
        runs.push_back(runs[index - 1] + closedPolyline[index - 1].distance(closedPolyline[index % count]));
    }
    float total = runs[count];
    std::optional<ClosestPair> best;
    for (size_t first = 0; first < count; first += 1) {
        for (size_t second = first + 1; second < count; second += 1) {
            float along = runs[second] - runs[first];
            if (smallerOf(along, total - along) < minimumRunApart) continue;
            float distance = closedPolyline[first].distance(closedPolyline[second]);
            if (!best || distance < best->distance) best = ClosestPair{distance, closedPolyline[first], closedPolyline[second]};
        }
    }
    return best;
}

/// The closest two points of two polylines (vertex to vertex, not along the sides) and the
/// distance. Throws a `MathProblem` if either has no points.
///
/// Use when: the gap between two outlines flattened finely; spacing letters or parts
/// Tags: polyline gap, closest points, distance between shapes, clearance
/// Node: planned (narrowestPoint)
ClosestPair closestPointsBetween(const std::vector<Vector2>& first, const std::vector<Vector2>& second) {
    if (first.empty() || second.empty()) throw MathProblem("The gap between two polylines needs a point in each.");
    ClosestPair best{std::numeric_limits<float>::infinity(), first[0], second[0]};
    for (Vector2 firstPoint : first) {
        for (Vector2 secondPoint : second) {
            float distance = firstPoint.distance(secondPoint);
            if (distance < best.distance) best = ClosestPair{distance, firstPoint, secondPoint};
        }
    }
    return best;
}

} // namespace editor
