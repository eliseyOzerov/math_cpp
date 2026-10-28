// Running a path backwards, starting a closed path elsewhere, and moving every point
// (graphics_cpp Curves2D path.cpp `reverse`, `startAt`, `map`).
#include "../Linear/MathProblem.h"
#include "Path2D.h"
#include "Shape2D+Pieces.h"

namespace editor {

namespace {

/// The path on `pieces`, closed if asked, with node marks `kinds` (node 0 first).
Path2D pathOnPieces(
    Vector2 start, const std::vector<OutlinePiece>& pieces, bool closed, const std::vector<std::optional<NodeKind>>& kinds) {
    Path2D result(start);
    for (const OutlinePiece& piece : pieces) {
        if (piece.isLine) result.addLine(piece.end());
        else result.addCubic(piece.cubic.startHandle, piece.cubic.endHandle, piece.end());
    }
    if (closed) result.close();
    for (size_t node = 0; node < kinds.size(); node += 1) result.setNodeKind(static_cast<int>(node), kinds[node]);
    return result;
}

} // namespace

/// The path run backwards: the same pieces in the opposite order, each reversed, closed if the
/// path was.
///
/// How: pieces (P0, P1, …) become (…, P1ʳ, P0ʳ) where Pʳ swaps a piece's ends and its two handles.
/// A closed path whose pen is away from the start keeps its implicit closing line, which becomes
/// the first piece. Node marks follow their nodes: node k of n pieces goes to node n − k.
///
/// Conventions: y-up plane; the signed area changes sign. A path with no pieces is unchanged.
///
/// Source: graphics_cpp curves/path `reverse`.
///
/// Use when: turning a hole round to the other direction; making outlines run the same way
/// Tags: reverse path, reverse contour, flip direction, winding order
/// Node: planned (reversePath)
Path2D Path2D::reversed() const {
    std::vector<Edge> pieces = edges();
    bool addsClosingLine = closed && !(currentPoint() == start);
    if (addsClosingLine) pieces.push_back(Edge{currentPoint(), Segment::line(start)});
    Path2D result(pieces.empty() ? start : pieces.back().segment.end());
    for (auto edge = pieces.rbegin(); edge != pieces.rend(); ++edge) {
        if (std::optional<CubicBezier<Vector2>> cubic = edge->cubic()) {
            result.addCubic(cubic->endHandle, cubic->startHandle, cubic->start);
        } else {
            result.addLine(edge->start);
        }
    }
    if (closed) result.close();
    int nodeCount = static_cast<int>(pieces.size());
    for (int node = 0; node <= nodeCount; node += 1) {
        // The closing line, if added, is a piece of its own: its end is node 0 again.
        if (std::optional<NodeKind> marked = nodeKind(node)) result.setNodeKind(nodeCount - node, marked);
    }
    return result;
}

/// A closed path started at its node `nodeIndex`: the same outline, drawn from there round to the
/// same place.
///
/// How: the pieces from piece `nodeIndex` on, then those before it; the start is the old node
/// `nodeIndex`. Marks follow their nodes. An implicit closing line becomes a piece of its own.
///
/// Conventions: only a closed path has a loop to start elsewhere; an open path or a node the
/// path does not have throws a `MathProblem`. Node `piece count` is the start again, so it gives
/// the path as it is.
///
/// Use when: lining up the starts of two outlines before morphing; moving a seam
/// Tags: start at, rotate path start, change start point, seam, cyclic shift
/// Node: planned (startPathAt)
Path2D Path2D::startingAtNode(int nodeIndex) const {
    if (!closed) throw MathProblem("Only a closed path can start at another node.");
    std::vector<Edge> pieces = edgesWithClosingLine();
    int count = static_cast<int>(pieces.size());
    if (count == 0 || nodeIndex < 0 || nodeIndex > count) throw MathProblem("The path has no such node to start at.");
    int first = nodeIndex % count;
    std::vector<OutlinePiece> ordered;
    std::vector<std::optional<NodeKind>> kinds;
    for (int step = 0; step < count; step += 1) {
        const Edge& edge = pieces[static_cast<size_t>((first + step) % count)];
        if (std::optional<CubicBezier<Vector2>> cubic = edge.cubic()) ordered.push_back(OutlinePiece::curve(*cubic));
        else ordered.push_back(OutlinePiece::line(edge.start, edge.segment.end()));
        kinds.push_back(nodeKind((first + step) % count));
    }
    kinds.push_back(kinds.front());
    return pathOnPieces(pieces[static_cast<size_t>(first)].start, ordered, true, kinds);
}

/// The path with every point sent through `pointMap`: the start, both handles and the end of
/// every piece. A line stays a line between the mapped ends. Node marks and closure are kept.
///
/// How: curves are mapped by their control points, which carries them exactly only for affine
/// maps (`AffineTransform2::applied`); for others use `Warp2D`, which also adds pieces.
///
/// Use when: moving, turning or scaling a whole outline
/// Tags: map path, transform path, apply transform, move all points
/// Node: planned (transformPath)
Path2D Path2D::mapped(const std::function<Vector2(Vector2)>& pointMap) const {
    Path2D result(pointMap(start));
    for (const Segment& segment : segmentList) {
        if (segment.isLine()) result.addLine(pointMap(segment.end()));
        else result.addCubic(pointMap(segment.startHandle), pointMap(segment.endHandle), pointMap(segment.end()));
    }
    if (closed) result.close();
    for (int node = 0; node <= static_cast<int>(segmentList.size()); node += 1) result.setNodeKind(node, nodeKind(node));
    return result;
}

} // namespace editor
