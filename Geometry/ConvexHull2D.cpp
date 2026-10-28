// Andrew's monotone chain and the merge of two convex polygons (see ConvexHull2D.h).
#include "ConvexHull2D.h"

#include <algorithm>

namespace editor {

namespace {

/// Whether `first` comes before `second` in x order, then y.
bool comesBefore(Vector2 first, Vector2 second) {
    return first.x < second.x || (first.x == second.x && first.y < second.y);
}

/// The cross product of (a − origin) and (b − origin): positive for a left turn.
float turnAt(Vector2 origin, Vector2 a, Vector2 b) { return (a - origin).cross(b - origin); }

/// The two sorted chains of a convex polygon between its leftmost and rightmost points, and the
/// merge of two sorted lists.
std::vector<Vector2> mergedInOrder(const std::vector<Vector2>& first, const std::vector<Vector2>& second) {
    std::vector<Vector2> result;
    size_t firstIndex = 0;
    size_t secondIndex = 0;
    while (firstIndex < first.size() || secondIndex < second.size()) {
        bool takeFirst = secondIndex >= second.size()
            || (firstIndex < first.size() && !comesBefore(second[secondIndex], first[firstIndex]));
        result.push_back(takeFirst ? first[firstIndex++] : second[secondIndex++]);
    }
    return result;
}

} // namespace

std::vector<Vector2> ConvexHull2D::of(const std::vector<Vector2>& points, bool isSorted) {
    std::vector<Vector2> sorted = points;
    if (!isSorted) std::stable_sort(sorted.begin(), sorted.end(), comesBefore);
    if (sorted.size() < 3) return sorted;
    std::vector<Vector2> lower;
    std::vector<Vector2> upper;
    for (Vector2 point : sorted) {
        while (lower.size() > 1 && turnAt(lower[lower.size() - 2], lower.back(), point) <= 0) lower.pop_back();
        lower.push_back(point);
    }
    for (size_t index = sorted.size(); index-- > 0;) {
        Vector2 point = sorted[index];
        while (upper.size() > 1 && turnAt(upper[upper.size() - 2], upper.back(), point) <= 0) upper.pop_back();
        upper.push_back(point);
    }
    std::vector<Vector2> hull(lower.begin(), lower.end() - 1);
    hull.insert(hull.end(), upper.begin(), upper.end() - 1);
    return hull;
}

std::vector<Vector2> ConvexHull2D::merged(const std::vector<Vector2>& first, const std::vector<Vector2>& second) {
    std::vector<std::vector<Vector2>> chains;
    for (const std::vector<Vector2>* polygon : {&first, &second}) {
        const std::vector<Vector2>& corners = *polygon;
        long count = static_cast<long>(corners.size());
        if (count == 0) continue;
        long leftmost = 0;
        long rightmost = 0;
        for (long index = 1; index < count; index += 1) {
            if (comesBefore(corners[index], corners[leftmost])) leftmost = index;
            if (comesBefore(corners[rightmost], corners[index])) rightmost = index;
        }
        // Counterclockwise from the leftmost corner the polygon runs along its lower chain to the
        // rightmost corner; clockwise from it, along the upper chain.
        std::vector<Vector2> forward{corners[leftmost]};
        std::vector<Vector2> backward{corners[leftmost]};
        for (long index = (leftmost + 1) % count; index != rightmost; index = (index + 1) % count) forward.push_back(corners[index]);
        for (long index = (leftmost - 1 + count) % count; index != rightmost; index = (index - 1 + count) % count) backward.push_back(corners[index]);
        if (rightmost != leftmost) {
            forward.push_back(corners[rightmost]);
            backward.push_back(corners[rightmost]);
        }
        chains.push_back(forward);
        // The backward chain without its two shared end points.
        chains.push_back(backward.size() >= 2 ? std::vector<Vector2>(backward.begin() + 1, backward.end() - 1) : std::vector<Vector2>{});
    }
    while (chains.size() < 4) chains.emplace_back();
    std::vector<Vector2> all = mergedInOrder(mergedInOrder(chains[0], chains[1]), mergedInOrder(chains[2], chains[3]));
    for (size_t index = 1; index < all.size(); index += 1) {
        if (comesBefore(all[index], all[index - 1])) return of(all);
    }
    return of(all, true);
}

} // namespace editor
