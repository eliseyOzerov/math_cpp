// The path form of `CubicBezier::fitted`: the pieces chained into one outline
// (Engine/Math/Paths/Path2D+Fitting.swift).
#include "Path2D.h"

namespace editor {

/// A path of cubic Bézier curves through the first and last of `points` that keeps every
/// point within `tolerance` of the path, or `nil` when the fit gives no pieces (see
/// `CubicBezier.fitted(to:tolerance:startDirection:endDirection:closed:)` for the algorithm, the
/// tolerance and the edge cases).
///
/// The pieces join exactly at shared points. The path is closed, with a smooth join at the
/// start, exactly when the fit treats the input as closed (see `closed` there): by default
/// four or more points with the last exactly the first. So `[A, B, A]` gives an open path
/// that goes to `B` and back, with a cusp at `B`, unless `closed` is `true`.
///
/// Use when: turning a freehand stroke or a dense polyline into an editable path with few
///   nodes
/// Tags: fit path to points, simplify polyline, stroke to path, smooth stroke, freehand
///   path, Schneider
/// Node: fitProfile
std::optional<Path2D> Path2D::fitted(
    const std::vector<Vector2>& points, float tolerance, std::optional<Vector2> startDirection,
    std::optional<Vector2> endDirection, std::optional<bool> closed) {
    std::vector<CubicBezier<Vector2>> pieces =
        CubicBezier<Vector2>::fitted(points, tolerance, startDirection, endDirection, closed);
    if (pieces.empty()) return std::nullopt;
    const CubicBezier<Vector2>& firstPiece = pieces.front();
    const CubicBezier<Vector2>& lastPiece = pieces.back();
    Path2D path(firstPiece.start);
    for (const CubicBezier<Vector2>& piece : pieces) {
        path.addCubic(piece.startHandle, piece.endHandle, piece.end);
    }
    bool fittedAsLoop = CubicBezier<Vector2>::fitsAsClosedLoop(CubicBezier<Vector2>::withoutDuplicates(points), closed);
    if (fittedAsLoop && pieces.size() >= 2 && lastPiece.end == firstPiece.start) path.close();
    return path;
}

} // namespace editor
