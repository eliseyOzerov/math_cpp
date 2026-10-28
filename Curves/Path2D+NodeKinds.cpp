// Node kinds read off the handles, and edits that keep them (graphics_cpp path.cpp
// `kind`, `kinds`, `withKind`, `handle`, `aligned`).
#include <algorithm>
#include <cmath>

#include "../Linear/MathProblem.h"
#include "Path2D.h"
#include "Shape2D+Pieces.h"

namespace editor {

namespace {

/// How far the directions of a node's two handles may differ, as the sine of the angle between
/// them, and still count as one line: a millionth, the tolerance graphics_cpp used.
constexpr float collinearTolerance = 1e-6f;

/// A handle shorter than this has no direction of its own.
constexpr float shortestHandle = 1e-6f;

/// The pieces holding node `nodeIndex`'s handles: the piece arriving and the piece leaving; -1
/// where there is none. A closed path whose pen ends on its start wraps round (node 0's arriving
/// piece is the last one); an implicit closing line has no handles, so it never counts.
struct HandleOwners {
    int arriving;
    int leaving;
};

HandleOwners handleOwners(int pieceCount, bool wrapsRound, int nodeIndex) {
    int arriving = nodeIndex > 0 ? nodeIndex - 1 : (wrapsRound ? pieceCount - 1 : -1);
    int leaving = nodeIndex < pieceCount ? nodeIndex : -1;
    // The last node of a path that wraps round is the start again: its handles are node 0's.
    if (nodeIndex == pieceCount && wrapsRound) return {arriving, -1};
    return {arriving, leaving};
}

/// The pieces a node kind is judged on: the path as drawn, each piece a cubic.
struct NodeFrame {
    std::vector<OutlinePiece> pieces;
    bool wrapsRound = false;
};

NodeFrame frameOf(const Path2D& path) {
    NodeFrame frame;
    frame.pieces = drawnPieces(path);
    frame.wrapsRound = path.isClosed() && !frame.pieces.empty() && path.currentPoint() == path.start;
    return frame;
}

Vector2 inMetricUnits(Vector2 point, Vector2 metric) { return point * metric; }

/// The direction of the first of `candidates` that is not on `node`, from `node` towards it;
/// nothing if all of them are.
std::optional<Vector2> directionFrom(Vector2 node, const std::array<Vector2, 3>& candidates) {
    for (Vector2 candidate : candidates) {
        Vector2 offset = candidate - node;
        if (offset.length() > shortestHandle) return offset;
    }
    return std::nullopt;
}

/// The path rebuilt on `pieces` with node marks `kinds` (node 0 first).
Path2D rebuilt(const std::vector<OutlinePiece>& pieces, const Path2D& original, const std::vector<std::optional<PathNodeKind>>& kinds) {
    Path2D result(original.start);
    for (const OutlinePiece& piece : pieces) {
        if (piece.isLine) result.addLine(piece.end());
        else result.addCubic(piece.cubic.startHandle, piece.cubic.endHandle, piece.end());
    }
    if (original.isClosed()) result.close();
    for (size_t node = 0; node < kinds.size(); node += 1) result.setNodeKind(static_cast<int>(node), kinds[node]);
    return result;
}

std::vector<std::optional<PathNodeKind>> markedKinds(const Path2D& path) {
    std::vector<std::optional<PathNodeKind>> kinds;
    for (int node = 0; node <= static_cast<int>(path.segments().size()); node += 1) kinds.push_back(path.nodeKind(node));
    return kinds;
}

/// The pieces with node `nodeIndex`'s handles lined up to `kind`, led by `lead`, judged in
/// `metric`. Corners, open ends and nodes with a handle sitting on them are left alone.
///
/// How: with N the node, in = N − arriving handle and out = leaving handle − N (in metric units)
/// and their unit vectors, the shared direction is the lead's, or normalized(in + out) for both
/// (left alone when they cancel). Smooth keeps each handle's own length; symmetric gives both the
/// lead's length (the average for both). The handles are placed at N − direction·inLength and
/// N + direction·outLength, then taken back out of metric units.
std::vector<OutlinePiece> alignedPieces(
    std::vector<OutlinePiece> pieces, bool wrapsRound, int nodeIndex, PathNodeKind kind, HandleLead lead, Vector2 metric) {
    HandleOwners owners = handleOwners(static_cast<int>(pieces.size()), wrapsRound, nodeIndex);
    if (kind == PathNodeKind::corner || owners.arriving < 0 || owners.leaving < 0) return pieces;
    OutlinePiece& arriving = pieces[static_cast<size_t>(owners.arriving)];
    OutlinePiece& leaving = pieces[static_cast<size_t>(owners.leaving)];
    Vector2 node = inMetricUnits(leaving.cubic.start, metric);
    Vector2 incoming = node - inMetricUnits(arriving.cubic.endHandle, metric);
    Vector2 outgoing = inMetricUnits(leaving.cubic.startHandle, metric) - node;
    float incomingLength = incoming.length();
    float outgoingLength = outgoing.length();
    if (incomingLength < shortestHandle || outgoingLength < shortestHandle) return pieces;
    Vector2 incomingDirection = incoming / incomingLength;
    Vector2 outgoingDirection = outgoing / outgoingLength;
    Vector2 both = incomingDirection + outgoingDirection;
    if (lead == HandleLead::both && both.length() < 1e-5f) return pieces;
    Vector2 direction = lead == HandleLead::incoming ? incomingDirection
        : lead == HandleLead::outgoing               ? outgoingDirection
                                                     : both.normalized();
    if (kind == PathNodeKind::symmetric) {
        float shared = lead == HandleLead::incoming ? incomingLength
            : lead == HandleLead::outgoing          ? outgoingLength
                                                    : (incomingLength + outgoingLength) / 2;
        incomingLength = shared;
        outgoingLength = shared;
    }
    arriving = OutlinePiece::curve(CubicBezier<Vector2>(
        arriving.cubic.start, arriving.cubic.startHandle, (node - direction * incomingLength) / metric, arriving.cubic.end));
    leaving = OutlinePiece::curve(CubicBezier<Vector2>(
        leaving.cubic.start, (node + direction * outgoingLength) / metric, leaving.cubic.endHandle, leaving.cubic.end));
    return pieces;
}

} // namespace

/// The kind of node `nodeIndex`: the one marked on it, or else the one its handles show.
///
/// How: with in = node − arriving handle and out = leaving handle − node, in metric units (a line
/// counts as a cubic with handles a third along it; a handle lying on the node takes the
/// direction to the next control point instead):
///
///     corner     no handle on one side (an open path's ends), or the directions differ
///                (|in × out| > 1e-6 as unit vectors), or they point against each other
///     smooth     the directions agree
///     symmetric  they agree and the handles are as long as each other (within 1e-6 of the longer)
///
/// Conventions: node 0 is `start`, then the end of each piece. `metric` is what a unit across and
/// a unit up are worth; (1, 1) for true coordinates. Throws a `MathProblem` for a node the path
/// does not have.
///
/// Use when: deciding whether dragging one handle should turn the other; labelling nodes in an editor
/// Tags: node type, smooth or corner, symmetric node, cusp, tangent continuity, handle check
/// Node: planned (setNodeKind)
PathNodeKind Path2D::effectiveNodeKind(int nodeIndex, Vector2 metric) const {
    if (nodeIndex < 0 || nodeIndex > static_cast<int>(segmentList.size())) {
        throw MathProblem("A node kind can only be read on a node of the path: 0 is the start, then each segment's end.");
    }
    if (std::optional<PathNodeKind> marked = this->nodeKind(nodeIndex)) return *marked;
    NodeFrame frame = frameOf(*this);
    HandleOwners owners = handleOwners(static_cast<int>(frame.pieces.size()), frame.wrapsRound, nodeIndex);
    if (owners.arriving < 0 || owners.leaving < 0) return PathNodeKind::corner;
    const CubicBezier<Vector2>& arriving = frame.pieces[static_cast<size_t>(owners.arriving)].cubic;
    const CubicBezier<Vector2>& leaving = frame.pieces[static_cast<size_t>(owners.leaving)].cubic;
    Vector2 node = inMetricUnits(leaving.start, metric);
    auto scaled = [&](Vector2 point) { return inMetricUnits(point, metric); };
    std::optional<Vector2> incoming = directionFrom(node, {scaled(arriving.endHandle), scaled(arriving.startHandle), scaled(arriving.start)});
    std::optional<Vector2> outgoing = directionFrom(node, {scaled(leaving.startHandle), scaled(leaving.endHandle), scaled(leaving.end)});
    if (!incoming || !outgoing) return PathNodeKind::corner;
    Vector2 incomingUnit = incoming->normalized();
    Vector2 outgoingUnit = outgoing->normalized();
    // `incoming` points from the node back along the arriving piece, so a smooth node has it
    // opposite to `outgoing`: the cross product vanishes and the dot product is negative.
    if (std::fabs(incomingUnit.cross(outgoingUnit)) > collinearTolerance || incomingUnit.dot(outgoingUnit) > 0) return PathNodeKind::corner;
    float incomingLength = (node - scaled(arriving.endHandle)).length();
    float outgoingLength = (scaled(leaving.startHandle) - node).length();
    bool isEven = incomingLength > shortestHandle
        && std::fabs(incomingLength - outgoingLength) <= 1e-6f * std::max(incomingLength, outgoingLength);
    return isEven ? PathNodeKind::symmetric : PathNodeKind::smooth;
}

/// The kind of every node, in order.
///
/// Use when: showing all node types of a path at once
/// Tags: node kinds, node types list
/// Node: planned (setNodeKind)
std::vector<PathNodeKind> Path2D::effectiveNodeKinds(Vector2 metric) const {
    std::vector<PathNodeKind> kinds;
    for (int node = 0; node <= static_cast<int>(segmentList.size()); node += 1) kinds.push_back(effectiveNodeKind(node, metric));
    return kinds;
}

/// The path with the nodes in `nodeIndices` marked `kind` and their handles lined up to it.
///
/// How: for each node, the kind is marked (`setNodeKind`) and the handles are moved as
/// `alignedPieces` describes: smooth keeps the two handle lengths, symmetric makes them equal
/// (their average), corner changes nothing but the mark. A line next to an aligned node becomes
/// a cubic. Nodes without handles on both sides keep their handles and get only the mark.
///
/// Conventions: indices as in `nodeKind`; one the path does not have throws a `MathProblem`.
///
/// Use when: turning a corner node smooth in a pen tool; making an outline's nodes symmetric
/// Tags: make smooth, make symmetric, align handles, node type, convert node
/// Node: planned (setNodeKind)
Path2D Path2D::withNodeKind(PathNodeKind kind, const std::vector<int>& nodeIndices, Vector2 metric) const {
    NodeFrame frame = frameOf(*this);
    std::vector<std::optional<PathNodeKind>> kinds = markedKinds(*this);
    for (int nodeIndex : nodeIndices) {
        if (nodeIndex < 0 || nodeIndex >= static_cast<int>(kinds.size())) {
            throw MathProblem("A node kind can only be marked on a node of the path: 0 is the start, then each segment's end.");
        }
        frame.pieces = alignedPieces(frame.pieces, frame.wrapsRound, nodeIndex, kind, HandleLead::both, metric);
        kinds[static_cast<size_t>(nodeIndex)] = kind;
    }
    return rebuilt(frame.pieces, *this, kinds);
}

/// The path with every node marked `kind` (see the version with a list of nodes).
///
/// Use when: smoothing a whole hand-drawn outline in one step
/// Tags: all nodes smooth, set all node kinds
/// Node: planned (setNodeKind)
Path2D Path2D::withNodeKind(PathNodeKind kind, Vector2 metric) const {
    std::vector<int> every;
    for (int node = 0; node <= static_cast<int>(segmentList.size()); node += 1) every.push_back(node);
    return withNodeKind(kind, every, metric);
}

/// The path with node `nodeIndex`'s handle on `side` moved to `to`, and the other handle following
/// as the node's kind says (`effectiveNodeKind`): it turns to stay on one line through the node
/// (smooth) and takes the same length (symmetric), or stays (corner). The kind is marked on the
/// node, so the next edit keeps it.
///
/// Conventions: throws a `MathProblem` when the node has no handle on that side (an open path's
/// ends, or a node next to nothing).
///
/// Use when: dragging a Bézier handle in an editor
/// Tags: move handle, drag handle, control point drag, smooth handle follow
/// Node: planned (moveHandle)
Path2D Path2D::withHandleMoved(int nodeIndex, HandleSide side, Vector2 to, Vector2 metric) const {
    PathNodeKind kind = effectiveNodeKind(nodeIndex, metric);
    NodeFrame frame = frameOf(*this);
    HandleOwners owners = handleOwners(static_cast<int>(frame.pieces.size()), frame.wrapsRound, nodeIndex);
    int ownerIndex = side == HandleSide::incoming ? owners.arriving : owners.leaving;
    if (ownerIndex < 0) throw MathProblem("This node has no handle on that side.");
    CubicBezier<Vector2> moved = frame.pieces[static_cast<size_t>(ownerIndex)].cubic;
    if (side == HandleSide::incoming) moved.endHandle = to;
    else moved.startHandle = to;
    frame.pieces[static_cast<size_t>(ownerIndex)] = OutlinePiece::curve(moved);
    frame.pieces = alignedPieces(
        frame.pieces, frame.wrapsRound, nodeIndex, kind, side == HandleSide::incoming ? HandleLead::incoming : HandleLead::outgoing, metric);
    std::vector<std::optional<PathNodeKind>> kinds = markedKinds(*this);
    kinds[static_cast<size_t>(nodeIndex)] = kind;
    return rebuilt(frame.pieces, *this, kinds);
}

} // namespace editor
