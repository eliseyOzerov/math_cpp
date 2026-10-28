// Placing a path by pairs of points (graphics_cpp attach.cpp: `attach`, `target`, `place`).
#include <algorithm>
#include <cmath>
#include <string>

#include "../Linear/MathProblem.h"
#include "PathAttach.h"
#include "Shape2D+Pieces.h"

namespace editor {

namespace {

/// Two points nearer than this (in the metric) decide no direction.
constexpr float shortestSpan = 1e-6f;

/// Two progresses nearer than this are the same place when blending moves between pairs.
constexpr float shortestProgressGap = 1e-12f;

/// The progress along the whole of `measured` of `progress` within `pieces`, or `progress` itself
/// when no pieces are named.
float wholeProgress(const MeasuredPath2D& measured, const std::optional<PieceRange>& pieces, float progress) {
    return pieces ? measured.progressWithin(pieces->first, pieces->last, progress) : progress;
}

/// The point of `on` at `progress` (within `pieces`, if named), measured in `metric`.
Vector2 pointOnPath(const Path2D& on, float progress, const std::optional<PieceRange>& pieces, Vector2 metric) {
    MeasuredPath2D measured(on, metric);
    return measured.pointAt(wholeProgress(measured, pieces, progress));
}

/// The progress pair `index` of `count` takes on the attached path when none is given: the start for
/// one pair, otherwise evenly from the start to the end.
float defaultProgress(int index, int count) {
    return count == 1 ? 0.0f : static_cast<float>(index) / static_cast<float>(count - 1);
}

/// `path` placed so its point at `fromFirst` lands on `targetFirst`, and, if `second` is given (its
/// progress and target), turned and stretched along the line between the two targets with its depth
/// across that line kept.
///
/// How: in the metric, with `u` the vector between the path's two points and `v` between the
/// targets, a point is split into `along` (its component on `u`, scaled by |v|/|u|) and `across`
/// (its component on the left normal of `u`) and rebuilt on `v`. Two points that coincide, or two
/// targets that do, give the shift alone.
Path2D attachedToPoints(
    const Path2D& path, float fromFirst, Vector2 targetFirst, const std::optional<std::pair<float, Vector2>>& second,
    Vector2 metric) {
    MeasuredPath2D measured(path, metric);
    Vector2 pathFirst = measured.pointAt(fromFirst) * metric;
    Vector2 goalFirst = targetFirst * metric;
    auto shift = [&](Vector2 point) { return (point * metric - pathFirst + goalFirst) / metric; };
    if (!second) return path.mapped(shift);
    Vector2 pathSecond = measured.pointAt(second->first) * metric;
    Vector2 goalSecond = second->second * metric;
    Vector2 pathSpan = pathSecond - pathFirst;
    Vector2 goalSpan = goalSecond - goalFirst;
    if (pathSpan.length() < shortestSpan || goalSpan.length() < shortestSpan) return path.mapped(shift);
    Vector2 pathDirection = pathSpan.normalized();
    Vector2 goalDirection = goalSpan.normalized();
    float stretch = goalSpan.length() / pathSpan.length();
    return path.mapped([&](Vector2 point) {
        Vector2 offset = point * metric - pathFirst;
        float along = offset.dot(pathDirection) * stretch;
        float across = pathDirection.cross(offset);
        Vector2 goalNormal(-goalDirection.y, goalDirection.x);
        return (goalFirst + goalDirection * along + goalNormal * across) / metric;
    });
}

/// `path` bent node by node so its points at the progresses `from` land on `destinations` (given in
/// the metric): a node is cut in at each, moved the rest of the way with its handles, and every
/// other node moved by a blend of the neighbouring pairs' moves by progress.
///
/// How: each pair's move is the vector from where its point is to its destination. A node at
/// progress s moves by the move of the first pair at or after s (the first pair's move before it,
/// the last pair's after the last), or by the blend between the pairs either side of it. A node's
/// handles move with it.
Path2D bentOnto(const Path2D& path, const std::vector<float>& from, const std::vector<Vector2>& destinations, Vector2 metric) {
    Path2D cut = MeasuredPath2D(path, metric).cut(from);
    MeasuredPath2D measuredCut(cut, metric);
    struct Move {
        float progress;
        Vector2 shift;
    };
    std::vector<Move> moves;
    for (size_t index = 0; index < from.size(); index += 1) {
        moves.push_back(Move{from[index], destinations[index] - measuredCut.pointAt(from[index]) * metric});
    }
    std::stable_sort(moves.begin(), moves.end(), [](const Move& first, const Move& second) { return first.progress < second.progress; });
    auto shiftAt = [&](float progress) {
        int later = -1;
        for (int index = 0; index < static_cast<int>(moves.size()); index += 1) {
            if (moves[static_cast<size_t>(index)].progress >= progress) {
                later = index;
                break;
            }
        }
        if (later == 0) return moves.front().shift;
        if (later < 0) return moves.back().shift;
        const Move& before = moves[static_cast<size_t>(later - 1)];
        const Move& after = moves[static_cast<size_t>(later)];
        float share = after.progress - before.progress < shortestProgressGap ? 1.0f : (progress - before.progress) / (after.progress - before.progress);
        return Vector2::linearBlend(before.shift, after.shift, share);
    };
    const std::vector<OutlinePiece>& pieces = measuredCut.pieces();
    int pieceCount = static_cast<int>(pieces.size());
    std::vector<Vector2> shifts;
    for (int node = 0; node < pieceCount + 1; node += 1) shifts.push_back(shiftAt(measuredCut.nodeProgress(node)));
    auto moved = [&](Vector2 point, int node) { return (point * metric + shifts[static_cast<size_t>(node)]) / metric; };
    std::vector<OutlinePiece> bent;
    for (int index = 0; index < pieceCount; index += 1) {
        const OutlinePiece& piece = pieces[static_cast<size_t>(index)];
        int arrival = cut.isClosed() ? (index + 1) % pieceCount : index + 1;
        if (piece.isLine && shifts[static_cast<size_t>(index)] == shifts[static_cast<size_t>(arrival)]) {
            bent.push_back(OutlinePiece::line(moved(piece.start(), index), moved(piece.end(), arrival)));
        } else {
            bent.push_back(OutlinePiece::curve(CubicBezier<Vector2>(
                moved(piece.cubic.start, index), moved(piece.cubic.startHandle, index), moved(piece.cubic.endHandle, arrival),
                moved(piece.cubic.end, arrival))));
        }
    }
    Path2D result = pathOfPieces(bent, cut.isClosed());
    if (result.nodeCount() == cut.nodeCount()) {
        for (int node = 0; node < cut.nodeCount(); node += 1) result.setNodeKind(node, cut.nodeKind(node));
    }
    return result;
}

} // namespace

/// The path placed by one or two mounts: shifted so its point at the first mount's `from` lands on
/// the mount's place; with a second mount, turned and stretched along the line between the two
/// places, its depth across that line kept.
///
/// How: the first mount's `from` defaults to 0 (the path's start), the second's to 1 (its end).
/// One mount is a shift. Two make a similarity along the line between the two places: the path's
/// points are measured along and across the line between its two points, the along part scaled by
/// the ratio of the two lengths, and rebuilt on the line between the places. Two mounts that land
/// on one point, or two points of the path that coincide, place it by the first alone.
///
///     path:  a ●───────● b         placed:        ● B
///                                              ╱
///                                         A ●╱
///
/// Conventions: y-up plane; lengths and angles in `metric`; the same metric measures the path and
/// the paths it is mounted on. Affine, so the pieces move exactly.
///
/// Source: graphics_cpp curves/attach.cpp `attach`.
///
/// Use when: pinning a serif or a branch to another stroke; placing a part between two anchors
/// Tags: attach path, pin to path, mount, snap to curve, shift and stretch
/// Node: planned (attachPath)
Path2D PathAttachment::attached(
    const Path2D& path, const AttachMount& first, const std::optional<AttachMount>& second, Vector2 metric) {
    Vector2 firstPlace = pointOnPath(first.on, first.progress, first.pieces, metric);
    std::optional<std::pair<float, Vector2>> secondPlace;
    if (second) secondPlace = std::pair<float, Vector2>(second->from.value_or(1), pointOnPath(second->on, second->progress, second->pieces, metric));
    return attachedToPoints(path, first.from.value_or(0), firstPlace, secondPlace, metric);
}

/// The point a target names.
///
/// How: a fixed point is itself; a place on a path is the point at its progress (within its
/// pieces, if named); a crossing is the one `which` picks (the first by default) of the places
/// where the two paths cross, in order along the first.
///
/// Conventions: throws a `MathProblem` where the paths do not cross ("the paths don't cross at all",
/// or "…first time" where fewer cross than asked), so a script fails where it is wrong.
///
/// Source: graphics_cpp curves/attach.cpp `target`.
///
/// Use when: reading where an anchor on the drawing is before placing something on it
/// Tags: target point, anchor, crossing anchor, resolve target
/// Node: planned (placePath)
Vector2 PathAttachment::targetPoint(const AttachTarget& target, Vector2 metric) {
    if (const TargetPoint* fixed = std::get_if<TargetPoint>(&target)) return fixed->point;
    if (const TargetOnPath* on = std::get_if<TargetOnPath>(&target)) return pointOnPath(on->on, on->progress, on->pieces, metric);
    const TargetCrossing& crossing = std::get<TargetCrossing>(target);
    std::vector<PathCrossing2> hits = MeasuredPath2D(crossing.crossing, metric).crossings(MeasuredPath2D(crossing.with, metric));
    CrossingChoice which = crossing.which.value_or(CrossingChoice::first());
    const PathCrossing2* chosen = nullptr;
    if (which.kind == CrossingChoice::Kind::indexed) {
        if (which.index >= 0 && which.index < static_cast<int>(hits.size())) chosen = &hits[static_cast<size_t>(which.index)];
    } else if (!hits.empty()) {
        chosen = which.kind == CrossingChoice::Kind::first ? &hits.front() : &hits.back();
    }
    if (!chosen) {
        std::string name = which.kind == CrossingChoice::Kind::indexed ? std::to_string(which.index)
            : which.kind == CrossingChoice::Kind::first                ? "first"
                                                                       : "last";
        throw MathProblem(std::string("the paths don't cross ") + (hits.empty() ? "at all" : name + " time"));
    }
    return chosen->point;
}

/// The path placed by pairs of points, and then turned by an alignment.
///
/// How: pair `k` of `n` sends the path's point at progress `from` (default: `k/(n−1)`, or 0 for one
/// pair) to its target. One or two pairs act as `attached`. Three give the one affine map through
/// them (`affineThroughTriangle`). Four or more give the affine map nearest to all of them
/// (`affineLeastSquares`) and then bend the path node by node onto the targets. Pairs that lie on a
/// line decide no affine map: the path is placed by the outer two pairs and then bent. An alignment
/// then turns the path about its first target (or the point at its `from`) so the direction there
/// runs along the other path's tangent (or normal) plus `extraAngle`.
///
/// Conventions: y-up plane; lengths and angles in `metric`. Throws a `MathProblem` for an alignment
/// with more than one pair ("an alignment goes with one pair at most: two already decide the turn")
/// and where a crossing target has no crossing. No pairs leave the path as it is (before the
/// alignment). A pair's progress is the path's own, before it is bent.
///
/// Source: graphics_cpp curves/attach.cpp `place`.
///
/// Use when: fitting a part onto a drawing by anchors: a handle to a door, a leaf to a stem
/// Tags: place path, snap by pairs, affine fit, attach with bend, register shapes
/// Node: planned (placePath)
Path2D PathAttachment::placed(
    const Path2D& path, const std::vector<AttachPair>& pairs, const std::optional<AttachAlign>& align, Vector2 metric) {
    if (align && pairs.size() > 1) throw MathProblem("an alignment goes with one pair at most: two already decide the turn");
    int count = static_cast<int>(pairs.size());
    std::vector<float> from;
    std::vector<Vector2> goals;
    for (int index = 0; index < count; index += 1) {
        from.push_back(pairs[static_cast<size_t>(index)].from.value_or(defaultProgress(index, count)));
        goals.push_back(targetPoint(pairs[static_cast<size_t>(index)].target, metric));
    }
    Path2D placedPath = path;
    if (count == 1 || count == 2) {
        std::optional<std::pair<float, Vector2>> second;
        if (count == 2) second = std::pair<float, Vector2>(from[1], goals[1]);
        placedPath = attachedToPoints(path, from[0], goals[0], second, metric);
    } else if (count >= 3) {
        MeasuredPath2D measured(path, metric);
        std::vector<Vector2> sources;
        std::vector<Vector2> destinations;
        for (int index = 0; index < count; index += 1) {
            sources.push_back(measured.pointAt(from[static_cast<size_t>(index)]) * metric);
            destinations.push_back(goals[static_cast<size_t>(index)] * metric);
        }
        std::optional<AffineTransform2> map =
            count == 3 ? affineThroughTriangle(sources, destinations) : affineLeastSquares(sources, destinations);
        if (map) {
            placedPath = path.mapped([&](Vector2 point) { return map->applied(point * metric) / metric; });
        } else {
            // Points on a line decide no map (nothing says how far across to stretch): place by the
            // outer two, then bend.
            placedPath = placed(
                path, {AttachPair{TargetPoint{goals.front()}, from.front()}, AttachPair{TargetPoint{goals.back()}, from.back()}},
                std::nullopt, metric);
        }
        if (!map || count > 3) placedPath = bentOnto(placedPath, from, destinations, metric);
    }
    if (!align) return placedPath;
    float alignFrom = align->from.value_or(from.empty() ? 0.0f : from.front());
    MeasuredPath2D measuredPlaced(placedPath, metric);
    Vector2 pivot = (goals.empty() ? measuredPlaced.pointAt(alignFrom) : goals.front()) * metric;
    MeasuredPath2D measuredOn(align->on, metric);
    Vector2 wanted = measuredOn.directionAt(wholeProgress(measuredOn, align->pieces, align->progress));
    if (align->to == AlignTo::normal) wanted = Vector2(-wanted.y, wanted.x);
    Angle turn = Angle::signedTurn(measuredPlaced.directionAt(alignFrom), wanted) + align->extraAngle;
    return placedPath.mapped([&](Vector2 point) { return (point * metric).rotated(turn, pivot) / metric; });
}

} // namespace editor
