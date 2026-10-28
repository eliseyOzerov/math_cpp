// The convex hull of points in the plane and of two convex polygons (graphics_cpp curves/fit
// `hull` and `mergeHulls`): the smallest convex polygon that holds them.
#pragma once

#include <vector>

#include "../Linear/Vector2.h"

namespace editor {

/// Convex hulls of 2D points.
///
/// Use when: joining a brush's stamps into one outline; the smallest convex wrapper of a shape;
///   a quick overlap test shape
/// Tags: convex hull, monotone chain, Andrew, hull of points, wrap points
/// Node: planned (convexHull)
struct ConvexHull2D {
    /// The corners of the convex hull of `points`, counterclockwise from the leftmost point.
    ///
    /// How: Andrew's monotone chain. Sort by x, then y. Walk left to right building the lower
    /// chain: while the last two chain points and the new one do not make a left turn (cross
    /// product <= 0), drop the last. Walk right to left for the upper chain the same way. Join
    /// the chains without their repeated ends.
    ///
    ///     upper chain ╭──●──╮
    ///               ●╯       ╰●        the hull keeps only the points that turn left
    ///     lower chain ╰──●──╯
    ///
    /// Conventions: y-up plane; counterclockwise. Points on a hull side (collinear) are dropped.
    /// Fewer than 3 points come back as given (sorted). `isSorted` says the points are already
    /// in x order (then y), which skips the sort.
    ///
    /// Source: Andrew, "Another efficient algorithm for convex hulls in two dimensions" (1979).
    ///
    /// Use when: wrapping the stamps of a nib; the convex outline of a scatter of points
    /// Tags: convex hull, monotone chain, Andrew's algorithm, wrap, outline of points
    /// Node: planned (convexHull)
    static std::vector<Vector2> of(const std::vector<Vector2>& points, bool isSorted = false);

    /// The hull of two convex polygons (counterclockwise), such as two neighbouring stamps.
    ///
    /// How: each polygon's two chains between its leftmost and rightmost points already run in x
    /// order, so the four chains are merged (as in merge sort) instead of sorting all the
    /// points, and the hull of the merged list is taken. Rounding can leave a chain a hair out
    /// of order; then the points are sorted after all, so the answer is the same either way.
    ///
    /// Conventions: both polygons convex and counterclockwise; the result is `of(first + second)`.
    ///
    /// Use when: sweeping a nib along a stroke, one pair of stamps at a time
    /// Tags: merge hulls, hull of two convex polygons, sweep, stamp join
    /// Node: planned (convexHull)
    static std::vector<Vector2> merged(const std::vector<Vector2>& first, const std::vector<Vector2>& second);
};

} // namespace editor
