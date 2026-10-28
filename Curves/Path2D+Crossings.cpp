// Where a path crosses itself: two of its pieces meeting somewhere other than the corner they
// share, or a single cubic piece looping over itself (Engine/Math/Paths/Path2D+Crossings.swift).
// `PathSelfCrossing` is declared in Path2D.h.
#include <algorithm>
#include <tuple>

#include "Path2D.h"

namespace editor {

/// Every point where the path crosses itself, ordered by first piece and parameter; nil
/// when two of its pieces overlap along a stretch (see `CubicBezier::crossings(other)`),
/// so the crossings cannot be listed.
///
/// How: the pieces are `cornerPieces` (the edges, and the closing line of a closed path,
/// without pieces of zero length). Every pair of pieces is crossed with
/// `CubicBezier::crossings(other)` (a line taken as the cubic with its handles at a third
/// and two thirds, which runs along it at an even pace, so its parameter is the line's),
/// and every cubic's own loop is found with `selfCrossingParameters`.
///
/// Neighbouring pieces always meet at the corner they share (and, on a closed path, the
/// last meets the first at the start, even when there are only two pieces, which then
/// share both corners); that meeting is not a crossing and is left out. A single cubic
/// whose start and end are the same point (a teardrop) meets itself there, which is its
/// closing corner, not a loop: `selfCrossingParameters` leaves the pair (0, 1) out.
/// They can still cross elsewhere, as a curve that doubles back over the line before it.
///
/// Limits: as `CubicBezier::crossings(other)`, crossings closer than 1/256 of the two
/// pieces' size merge into one. A
/// corner where the path passes through the same point twice (a figure eight drawn
/// through a vertex) is reported once for each pair of pieces that meet there, except
/// pairs that are neighbours.
///
/// Use when: checking an outline is simple before extruding it; finding the knots of a
///   hand-drawn path; preparing to cut a path into loops
/// Tags: self intersection, self crossing, path crossings, simple polygon test, knots
/// Node: profileCrossings
std::optional<std::vector<PathSelfCrossing>> Path2D::selfCrossings() const {
    std::vector<Edge> pieces = cornerPieces();
    std::vector<CubicBezier<Vector2>> curves;
    for (const Edge& piece : pieces) {
        std::optional<CubicBezier<Vector2>> curve = piece.cubic();
        curves.push_back(curve ? *curve : evenlyPacedCubic(piece.start, piece.segment.end()));
    }
    std::vector<PathSelfCrossing> found;
    int count = static_cast<int>(curves.size());
    for (int firstIndex = 0; firstIndex < count; firstIndex += 1) {
        if (pieces[firstIndex].cubic().has_value()) {
            if (std::optional<SelfCrossingParameters> loop = curves[firstIndex].selfCrossingParameters()) {
                found.push_back(PathSelfCrossing{
                    firstIndex, loop->first, firstIndex, loop->second, curves[firstIndex].point(loop->first)});
            }
        }
        for (int secondIndex = firstIndex + 1; secondIndex < count; secondIndex += 1) {
            bool followsFirst = secondIndex == firstIndex + 1;
            bool wrapsToFirst = closed && firstIndex == 0 && secondIndex == count - 1;
            std::optional<std::vector<CurveCrossing>> crossings = curves[firstIndex].crossings(curves[secondIndex]);
            if (!crossings) return std::nullopt;
            for (const CurveCrossing& crossing : *crossings) {
                bool isSharedCorner =
                    (followsFirst && crossing.firstParameter == 1 && crossing.secondParameter == 0)
                    || (wrapsToFirst && crossing.firstParameter == 0 && crossing.secondParameter == 1);
                if (isSharedCorner) continue;
                found.push_back(PathSelfCrossing{
                    firstIndex, crossing.firstParameter, secondIndex, crossing.secondParameter, crossing.point});
            }
        }
    }
    std::stable_sort(found.begin(), found.end(), [](const PathSelfCrossing& left, const PathSelfCrossing& right) {
        return std::tie(left.firstPieceIndex, left.firstParameter, left.secondPieceIndex)
            < std::tie(right.firstPieceIndex, right.firstParameter, right.secondPieceIndex);
    });
    return found;
}

/// The straight line from `start` to `end` as a cubic whose handles are at a third and two
/// thirds of the way, so `point(at: t)` is `start + t·(end − start)` exactly in exact
/// arithmetic: the parameter of the cubic is the parameter of the line.
///
/// Use when: treating a line piece of a path like its curved pieces (crossings, offsets)
/// Tags: line as Bezier, degree elevation, straight cubic, linear Bezier
/// Node: none (solver)
CubicBezier<Vector2> Path2D::evenlyPacedCubic(Vector2 start, Vector2 end) {
    return CubicBezier<Vector2>(
        start, Vector2::linearBlend(start, end, 1.0f / 3), Vector2::linearBlend(start, end, 2.0f / 3), end);
}

} // namespace editor
