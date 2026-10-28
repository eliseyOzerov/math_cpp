// A path crossed by a line or a circle and read as circular arcs, all answered in progress so the
// results go straight to `MeasuredPath2D::slice` and `cut`. Ported from graphics_cpp's analysis.cpp
// (`lineCrossings`, `circleCrossings`, `arcs`).
#include <algorithm>

#include "CurveCircleCrossings.h"
#include "MeasuredPath2D.h"
#include "Path2D.h"

namespace editor {

namespace {

CubicBezier<Vector2> curveInMetric(const CubicBezier<Vector2>& curve, Vector2 metric) {
    return CubicBezier<Vector2>(curve.start * metric, curve.startHandle * metric, curve.endHandle * metric, curve.end * metric);
}

} // namespace

/// Where the path crosses the line from `lineStart` to `lineEnd`, in order along the path.
///
/// How: `CubicBezier::crossings(lineStart, lineEnd, withinSegment)` on each piece (the closing
/// line of a closed path included), each piece's hits sorted by parameter and turned into
/// progress.
///
///     lineStart ●─────┬──────┬────── ● lineEnd      hits at the ┬ marks, in order of
///                 ╭───╯      ╰───╮                  progress along the path
///
/// Conventions: y-up plane; with `withinSegment` true (the default) only hits between the two
/// points count, otherwise the line is infinite and `lineParameter` runs outside 0…1. The line and
/// the points are in the path's own coordinates; `metric` only sets what progress means.
///
/// Source: graphics_cpp curves/analysis.cpp `lineCrossings`.
///
/// Use when: cutting a profile at a guide line; finding the places a stroke meets a boundary
/// Tags: line intersection, ray cast, cut path with line, scanline crossing
/// Node: planned (pathLineCrossings)
std::vector<PathLineCrossing> Path2D::lineCrossings(
    Vector2 lineStart, Vector2 lineEnd, bool withinSegment, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    std::vector<PathLineCrossing> crossings;
    for (size_t index = 0; index < measured.pieces().size(); index += 1) {
        std::vector<CurveCrossing> hits = measured.pieces()[index].cubic.crossings(lineStart, lineEnd, withinSegment);
        std::sort(hits.begin(), hits.end(), [](const CurveCrossing& first, const CurveCrossing& second) {
            return first.firstParameter < second.firstParameter;
        });
        for (const CurveCrossing& hit : hits) {
            crossings.push_back(PathLineCrossing{
                measured.progress(static_cast<int>(index), hit.firstParameter), hit.secondParameter, hit.point});
        }
    }
    return crossings;
}

/// Where the path crosses the circle round `centre` of `radius`, in order along the path.
///
/// How: `circleCrossings` of each piece scaled by `metric` (so the circle is round on the page)
/// against the scaled centre, the hits scaled back and turned into progress.
///
/// Conventions: y-up plane; `radius` is in the metric's units; `centre` and the returned points
/// are in the path's own coordinates.
///
/// Source: graphics_cpp curves/analysis.cpp `circleCrossings`.
///
/// Use when: trimming a stroke where it enters a round cap; finding the places a path is a given
///   distance from a point
/// Tags: circle intersection, path circle crossing, trim at radius, brush circle
/// Node: planned (pathCircleCrossings)
std::vector<PathCircleCrossing> Path2D::circleCrossings(Vector2 centre, float radius, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    std::vector<PathCircleCrossing> crossings;
    for (size_t index = 0; index < measured.pieces().size(); index += 1) {
        CubicBezier<Vector2> scaled = curveInMetric(measured.pieces()[index].cubic, metric);
        for (const CircleCrossing& hit : editor::circleCrossings(scaled, centre * metric, radius)) {
            crossings.push_back(PathCircleCrossing{measured.progress(static_cast<int>(index), hit.parameter), hit.point / metric});
        }
    }
    return crossings;
}

/// The path as circular arcs, each within `threshold` of the piece it replaces, with the span of
/// the path it covers.
///
/// How: `CubicBezier::circularArcs(threshold)` on each piece scaled by `metric`; the centres and
/// end points are scaled back to the path's coordinates and the parameters turned into progress.
///
/// Conventions: y-up plane; `threshold` and the radii are in the metric's units. Straight stretches
/// come back with no centre and infinite radius.
///
/// Source: graphics_cpp curves/analysis.cpp `arcs`.
///
/// Use when: turning an outline into arcs for a machine or a font format that has only lines and
///   arcs
/// Tags: arc fit, biarc, circular arcs of a path, G-code arcs, arcs and lines
/// Node: planned (pathArcs)
std::vector<PathArc> Path2D::circularArcs(float threshold, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    std::vector<PathArc> arcs;
    for (size_t index = 0; index < measured.pieces().size(); index += 1) {
        int pieceIndex = static_cast<int>(index);
        for (CircularArcPiece arc : curveInMetric(measured.pieces()[index].cubic, metric).circularArcs(threshold)) {
            PathArc found;
            found.startProgress = measured.progress(pieceIndex, arc.startParameter);
            found.endProgress = measured.progress(pieceIndex, arc.endParameter);
            if (arc.centre) arc.centre = *arc.centre / metric;
            arc.from = arc.from / metric;
            arc.to = arc.to / metric;
            found.arc = arc;
            arcs.push_back(found);
        }
    }
    return arcs;
}

} // namespace editor
