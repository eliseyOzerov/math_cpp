// Taking a stretch of a measured path and adding nodes at shares of its length.
#include <algorithm>
#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/NumberText.h"
#include "MeasuredPath2D.h"

namespace editor {

namespace {

/// A stretch of a piece shorter than this share of its parameter is not worth a piece.
constexpr float shortestStretch = 1e-6f;

/// A cut this near a node, or to another cut, adds nothing.
constexpr float sameParameterDistance = 1e-6f;

} // namespace

/// The stretch of the path from `startProgress` to `endProgress`, as an open path.
///
/// How: both ends are located; the pieces between are kept whole, the first and last cut to the
/// part inside (`OutlinePiece::portion`, exact for lines and `CubicBezier::portion` for curves);
/// parts shorter than 1e-6 of a parameter are dropped. Node marks are not carried: the nodes are
/// read afresh from the new handles.
///
/// Conventions: progress is a share of length, held in 0…1. `endProgress` must be above
/// `startProgress`, or a `MathProblem` is thrown. The slice of a closed path is open.
///
/// Use when: taking the part of an outline between two marks; trimming a stroke
/// Tags: slice path, sub path, trim path, section of curve, cut out part
/// Node: planned (slicePath)
Path2D MeasuredPath2D::slice(float startProgress, float endProgress) const {
    if (!(endProgress > startProgress)) {
        throw MathProblem("There is nothing between " + numberText(startProgress) + " and " + numberText(endProgress) + ".");
    }
    PathPoint2 first = locate(startProgress);
    PathPoint2 last = locate(endProgress);
    std::vector<OutlinePiece> stretch;
    for (int index = first.pieceIndex; index <= last.pieceIndex; index += 1) {
        float lower = index == first.pieceIndex ? first.parameter : 0;
        float upper = index == last.pieceIndex ? last.parameter : 1;
        if (upper - lower > shortestStretch) stretch.push_back(pieceList[static_cast<size_t>(index)].portion(lower, upper));
    }
    if (stretch.empty()) throw MathProblem("There is nothing between " + numberText(startProgress) + " and " + numberText(endProgress) + ".");
    return pathOfPieces(stretch, false);
}

/// The same path with nodes added at the shares of length in `progresses`; a share that falls on
/// a node, or on one already added, is skipped.
///
/// How: each share is located; the parameters landing strictly inside a piece (more than 1e-6 from
/// both ends) are sorted per piece and the piece is cut there. The shape is unchanged. Node marks
/// stay with their nodes and the new nodes are unmarked. A closed path comes back closed, with
/// its closing line as an explicit piece.
///
/// Use when: adding handles to move at set places; preparing an outline for a warp that needs nodes
/// Tags: cut path, add nodes, insert points, subdivide at lengths, split at progress
/// Node: planned (cutPath)
Path2D MeasuredPath2D::cut(const std::vector<float>& progresses) const {
    if (pieceList.empty()) return pathValue;
    std::vector<std::vector<float>> cutsByPiece(pieceList.size());
    for (float share : progresses) {
        PathPoint2 place = locate(share);
        std::vector<float>& cuts = cutsByPiece[static_cast<size_t>(place.pieceIndex)];
        bool isInside = place.parameter > sameParameterDistance && place.parameter < 1 - sameParameterDistance;
        bool isKnown = std::any_of(cuts.begin(), cuts.end(), [&](float old) { return std::fabs(old - place.parameter) < sameParameterDistance; });
        if (isInside && !isKnown) cuts.push_back(place.parameter);
    }
    bool hasCuts = std::any_of(cutsByPiece.begin(), cutsByPiece.end(), [](const std::vector<float>& cuts) { return !cuts.empty(); });
    if (!hasCuts) return pathValue;

    std::vector<OutlinePiece> cutPieces;
    std::vector<std::optional<NodeKind>> kinds{pathValue.nodeKind(0)};
    for (size_t index = 0; index < pieceList.size(); index += 1) {
        std::vector<float> boundaries{0};
        std::sort(cutsByPiece[index].begin(), cutsByPiece[index].end());
        boundaries.insert(boundaries.end(), cutsByPiece[index].begin(), cutsByPiece[index].end());
        boundaries.push_back(1);
        for (size_t step = 1; step < boundaries.size(); step += 1) {
            cutPieces.push_back(pieceList[index].portion(boundaries[step - 1], boundaries[step]));
            kinds.push_back(step + 1 < boundaries.size() ? std::nullopt : pathValue.nodeKind(static_cast<int>(index) + 1));
        }
    }
    // A closing line the path only implied is now a piece of its own, ending on the start again.
    if (pathValue.isClosed() && pieceList.size() > pathValue.segments().size()) kinds.back() = pathValue.nodeKind(0);
    Path2D result = pathOfPieces(cutPieces, pathValue.isClosed());
    for (size_t node = 0; node < kinds.size(); node += 1) result.setNodeKind(static_cast<int>(node), kinds[node]);
    return result;
}

/// The path with a node every 1/`count` of its length: `count` − 1 cuts at 1/count, 2/count, ….
///
/// Conventions: `count` below 1 throws a `MathProblem`; 1 returns the path unchanged.
///
/// Use when: splitting an outline into equal-length pieces before warping or dashing it
/// Tags: divide path, equal length pieces, subdivide evenly, split into n
/// Node: planned (cutPath)
Path2D MeasuredPath2D::divided(int count) const {
    if (count < 1) throw MathProblem("A path can only be divided into one or more parts.");
    std::vector<float> shares;
    for (int step = 1; step < count; step += 1) shares.push_back(static_cast<float>(step) / static_cast<float>(count));
    return cut(shares);
}

} // namespace editor
