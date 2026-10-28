// Building a path and reading its pieces (Engine/Math/Paths/Path2D.swift).
#include <cmath>

#include "ArcLength.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "Path2D.h"

namespace editor {

namespace {

/// The message when a piece is added after `close()`.
const char* const addingToClosedPathMessage = "A closed path cannot grow: add every piece before calling close().";

/// The message when a node mark names a node the path does not have.
const char* const noSuchNodeMessage = "A node kind can only be marked on a node of the path: 0 is the start, then each segment's end.";

} // namespace

// MARK: - Node kinds

std::optional<NodeKind> Path2D::nodeKind(int nodeIndex) const {
    if (nodeIndex < 0 || nodeIndex >= static_cast<int>(nodeKindList.size())) return std::nullopt;
    return nodeKindList[static_cast<size_t>(nodeIndex)];
}

void Path2D::setNodeKind(int nodeIndex, std::optional<NodeKind> kind) {
    if (nodeIndex < 0 || nodeIndex > static_cast<int>(segmentList.size())) throw MathProblem(noSuchNodeMessage);
    if (nodeIndex >= static_cast<int>(nodeKindList.size())) {
        if (!kind) return;
        nodeKindList.resize(static_cast<size_t>(nodeIndex) + 1);
    }
    nodeKindList[static_cast<size_t>(nodeIndex)] = kind;
}

// MARK: - Building

/// Draws a straight line from the current point to `point`.
///
/// The path must still be open: adding a piece after `close()` stops the program, since
/// the closing line would no longer be the last piece.
///
/// Use when: adding a flat side to a profile
/// Tags: line to, lineTo, add segment, straight edge
/// Node: inside roundCorners, chamferCorners
void Path2D::addLine(Vector2 to) {
    if (closed) throw MathProblem(addingToClosedPathMessage);
    segmentList.push_back(Segment::line(to));
}

/// Draws a cubic Bézier curve from the current point, pulled toward `startHandle` and
/// `endHandle`, ending at `end`. The path must still be open (see `addLine`).
///
/// Use when: adding a smooth bend to a profile
/// Tags: curve to, bezierCurveTo, cubicTo, add curve, smooth segment
/// Node: inside roundCorners, chamferCorners
void Path2D::addCubic(Vector2 startHandle, Vector2 endHandle, Vector2 end) {
    if (closed) throw MathProblem(addingToClosedPathMessage);
    segmentList.push_back(Segment::cubic(startHandle, endHandle, end));
}

/// Draws a circle arc of `radius` around `centre`, from `startAngle` turning by `sweep`
/// (counterclockwise for a positive sweep), stored as cubics of at most 90° each (see
/// `CubicBezier::circularArc`).
///
/// If the current point is not already at the arc's first point (further than
/// `arcJoinTolerance` away), a straight line is drawn to it first, as
/// the canvas `arc` call does; otherwise the arc continues the path and its first point
/// is taken to be the current point. A sweep or radius for which `circularArc` gives no
/// pieces adds nothing. The path must still be open (see `addLine`).
///
/// Use when: adding a rounded corner or a circular side to a profile; drawing a circle
/// Tags: arc, arcTo, circle arc, add arc, round corner, canvas arc
/// Node: inside roundCorners, chamferCorners
void Path2D::addArc(Vector2 centre, float radius, Angle startAngle, Angle sweep) {
    if (closed) throw MathProblem(addingToClosedPathMessage);
    std::vector<CubicBezier<Vector2>> arcs = CubicBezier<Vector2>::circularArc(centre, radius, startAngle, sweep);
    if (arcs.empty()) return;
    const CubicBezier<Vector2>& firstArc = arcs.front();
    if (currentPoint().distance(firstArc.start) > arcJoinTolerance(centre, radius)) {
        addLine(firstArc.start);
    }
    for (const CubicBezier<Vector2>& arc : arcs) {
        addCubic(arc.startHandle, arc.endHandle, arc.end);
    }
}

/// How far an arc around `centre` with `radius` may start from the current point and
/// still count as continuing the path.
///
/// How: the larger of `1e-5·radius` and `4·ulp(|centre| + radius)`, with |centre| its
/// largest coordinate. The first is the arc's own scale; the second is the rounding of
/// its points, which are computed as `centre + radius·direction` and so are only known
/// to an ulp of their size. Far from the origin with a small radius the second wins: at
/// centre x = 1e5 an ulp is 0.0078, far more than 1e-5 of a radius of 0.01, so the end
/// of one arc and the start of the next computed separately would otherwise be joined
/// by a needless tiny line.
///
/// Use when: deciding whether an arc continues the path or needs a line to its start
/// Tags: arc join, continuity tolerance, snap to current point, rounding slack, ulp
/// Node: none (solver)
float Path2D::arcJoinTolerance(Vector2 centre, float radius) {
    float size = largerOf(std::fabs(centre.x), std::fabs(centre.y)) + radius;
    return largerOf(1e-5f * radius, 4 * curve_detail::unitInTheLastPlace(size));
}

/// Joins the last point back to the start with a straight line, making the path closed.
/// Calling it again does nothing more. After it no piece can be added.
///
/// Use when: finishing a profile or an outline that should be filled
/// Tags: close path, closePath, close shape, loop
/// Node: inside roundCorners, chamferCorners
void Path2D::close() { closed = true; }

// MARK: - Pieces for the measurements

/// The piece as a cubic Bézier, or `nil` for a straight line.
///
/// Use when: analysing a curved piece (extrema, bounds, curvature)
/// Tags: edge curve, segment as Bezier
/// Node: inside roundCorners, chamferCorners
std::optional<CubicBezier<Vector2>> Path2D::Edge::cubic() const {
    if (segment.kind != Segment::Kind::cubic) return std::nullopt;
    return CubicBezier<Vector2>(start, segment.startHandle, segment.endHandle, segment.endPoint);
}

/// The pieces as edges (each knowing its start), without the closing line.
///
/// Use when: walking a path piece by piece
/// Tags: path edges, iterate segments
/// Node: inside roundCorners, chamferCorners
std::vector<Path2D::Edge> Path2D::edges() const {
    std::vector<Edge> result;
    Vector2 pieceStart = start;
    for (const Segment& segment : segmentList) {
        result.push_back(Edge{pieceStart, segment});
        pieceStart = segment.end();
    }
    return result;
}

/// The pieces as edges plus, when the pen is not back at the start, the straight line
/// that closes the outline. This is the outline that a fill uses, whether or not
/// `isClosed` is set.
///
/// Use when: measuring the region a path encloses (area, winding)
/// Tags: closed outline, fill edges, implicit closing line
/// Node: inside roundCorners, chamferCorners
std::vector<Path2D::Edge> Path2D::edgesWithClosingLine() const {
    std::vector<Edge> result = edges();
    if (!(currentPoint() == start)) result.push_back(Edge{currentPoint(), Segment::line(start)});
    return result;
}

} // namespace editor
