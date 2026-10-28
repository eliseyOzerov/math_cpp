// Choosing the nodes of a path an edit will act on: some nodes by index, the cubics between
// nodes, or the stretch between two progresses. Ported from graphics_cpp's selection.cpp
// (`nodeAt`, `cubicAt`, `section`), without the Selection object they fed: the result is the
// path and a weight per node.
#include <set>

#include "../Linear/MathProblem.h"
#include "MeasuredPath2D.h"
#include "Path2D.h"

namespace editor {

namespace {

/// Index `index` counted from the end when below 0, and checked to be one of `count`.
int wrappedIndex(int index, int count, const char* what) {
    int wrapped = index < 0 ? count + index : index;
    if (wrapped < 0 || wrapped >= count) throw MathProblem(std::string("There is no such ") + what + " in the path.");
    return wrapped;
}

/// The same path with weight 1 on `picked` and 0 on every other node.
PickedNodes weighted(const Path2D& path, const std::set<int>& picked) {
    std::vector<float> weights(static_cast<size_t>(path.nodeCount()), 0.0f);
    for (int node : picked) weights[static_cast<size_t>(node)] = 1;
    return PickedNodes{path, weights};
}

} // namespace

/// The number of nodes: the start and the end of each piece, where a closed path's last piece ends
/// on its start again (its closing line counts as a piece).
///
/// Use when: looping over the nodes; checking an index
/// Tags: node count, number of points, anchor count
/// Node: planned (selectNodes)
int Path2D::nodeCount() const {
    return closed ? static_cast<int>(edgesWithClosingLine().size()) : static_cast<int>(segmentList.size()) + 1;
}

/// The path with the nodes at `nodeIndices` picked whole.
///
/// How: weight 1 on those nodes, 0 on the rest. A negative index counts from the last node.
///
/// Conventions: throws a `MathProblem` for an index outside the path.
///
/// Source: graphics_cpp curves/selection.cpp `nodeAt`.
///
/// Use when: moving or rounding chosen nodes
/// Tags: select nodes, pick points, node selection
/// Node: planned (selectNodes)
PickedNodes Path2D::pickingNodes(const std::vector<int>& nodeIndices) const {
    std::set<int> picked;
    for (int index : nodeIndices) picked.insert(wrappedIndex(index, nodeCount(), "node"));
    return weighted(*this, picked);
}

/// The path with both ends of each of the pieces at `pieceIndices` picked whole: picking a piece
/// picks the two nodes it runs between.
///
/// How: node `i` and node `i + 1` for piece `i` (node 0 again for a closed path's last piece). A
/// negative index counts from the last piece.
///
/// Conventions: throws a `MathProblem` for a piece outside the path.
///
/// Source: graphics_cpp curves/selection.cpp `cubicAt`.
///
/// Use when: moving one curve of an outline with both its nodes
/// Tags: select cubic, pick segment, select curve piece
/// Node: planned (selectNodes)
PickedNodes Path2D::pickingCubics(const std::vector<int>& pieceIndices) const {
    int pieceCount = closed ? nodeCount() : static_cast<int>(segmentList.size());
    std::set<int> picked;
    for (int index : pieceIndices) {
        int piece = wrappedIndex(index, pieceCount, "piece");
        picked.insert(piece);
        picked.insert(closed ? (piece + 1) % pieceCount : piece + 1);
    }
    return weighted(*this, picked);
}

/// The path cut at two progresses, with the nodes between them (the cut ones included) picked whole.
///
/// How: `MeasuredPath2D::cut` at both progresses, then weight 1 on every node whose own progress
/// lies in [lower − 1e-6, upper + 1e-6], where lower and upper are the two progresses in order.
///
/// Conventions: progress is a share of length in `metric`; the progresses may be given in either
/// order. The returned path has the two cut nodes added.
///
/// Source: graphics_cpp curves/selection.cpp `section`.
///
/// Use when: editing the stretch of a stroke between two places
/// Tags: select section, pick stretch, range selection, soft selection span
/// Node: planned (selectNodes)
PickedNodes Path2D::pickingSection(float fromProgress, float toProgress, Vector2 metric) const {
    constexpr float margin = 1e-6f;
    Path2D cut = MeasuredPath2D(*this, metric).cut({fromProgress, toProgress});
    MeasuredPath2D measuredCut(cut, metric);
    float lower = fromProgress <= toProgress ? fromProgress : toProgress;
    float upper = fromProgress <= toProgress ? toProgress : fromProgress;
    std::set<int> picked;
    for (int node = 0; node < cut.nodeCount(); node += 1) {
        float progress = measuredCut.nodeProgress(node);
        if (progress >= lower - margin && progress <= upper + margin) picked.insert(node);
    }
    return weighted(cut, picked);
}

} // namespace editor
