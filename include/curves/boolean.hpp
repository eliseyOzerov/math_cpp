// Boolean operations on closed outlines of lines and cubics: union, intersection, difference, or any rule over the
// winding numbers of several groups of contours. Overlapping strokes want one outline per shape with no overlaps, and
// cut-outs are differences and intersections of the same kind.
//
// One algorithm for every operation, in three steps: every piece is cut where it crosses another (cut), a fragment is
// kept where the rule holds on one side of it and not on the other (keep), and the kept fragments are chained end to
// start into loops (chain). Coincident edges are cut at each other's ends so their fragments match and the second
// copy is dropped. Also here: what the booleans' callers need of contours (their area, orientation as TrueType fills
// them, and polylines to fill).
//
// Known limits: curves that run along each other (not lines) are split only at each other's ends, so a partly shared
// curve may leave a sliver; a loop that does not close (a gap left by a missed crossing) makes the whole operation
// return its input unchanged rather than a broken outline.
//
// Ported from fountain-ts/src/curves/boolean.ts. Decisions:
// - Input groups are `std::span<const std::vector<Contour>>`; the rule is a std::function over the groups' windings.
// - TS null and undefined results are std::optional.
// - `union` is a C++ keyword, so it is `unite`.
#pragma once

#include "curves/geometry.hpp"

#include <functional>

namespace curves {

/** A crossing of two cubics: t on the first, u on the second. */
using Crossing = std::array<double, 2>;
/** An end two cubics share: [a's end, b's end], each 0 or 1. */
using SharedEnd = std::array<int, 2>;

/** Where two cubics cross: pairs of parameters [t on a, u on b]. Found by halving both while their control boxes
 *  overlap, down to `tol` units, and merging the hits that land on the same spot. Ends the two share (`shared`) are
 *  not crossings: the halving near them is cut short. Returns nothing where the curves run along each other (more
 *  hits than two cubics can cross in). */
std::optional<std::vector<Crossing>> crossings(const Cubic& a, const Cubic& b, double tol = 1e-3,
                                               std::span<const SharedEnd> shared = {});

/** The winding number of `cs` round `p`: crossings of a ray to +x, each counted +1 where the outline runs up across
 *  it. The ray sits a hair above `p`, so it passes through no node of an outline in round numbers. */
int winding(std::span<const Contour> cs, Pt p);

/** Whether a point is inside, from the winding number of each group round it. */
using Rule = std::function<bool(const std::vector<int>&)>;

/** The outline where `inside` holds, from the winding numbers of each group of contours round a point: union is
 *  "any group nonzero", or "group 0 nonzero" over one group to remove a group's own overlaps. Outer contours come out
 *  clockwise and holes counter-clockwise (y up). Returns the contours of all groups unchanged where the loops do not
 *  close (tryBoolean says so with nothing). Points closer than `eps` units count as one. */
std::vector<Contour> boolean(std::span<const std::vector<Contour>> input, const Rule& inside, double eps = 0.25);

/** boolean, or nothing where its loops do not close. */
std::optional<std::vector<Contour>> tryBoolean(std::span<const std::vector<Contour>> input, const Rule& inside,
                                               double eps = 0.25);

/** A boolean's outline with where each piece came from: `from[k][j]` is the group (operand) that piece j of contour
 *  k was cut from, so a corner can be told by the two operands meeting there. */
struct Traced {
  std::vector<Contour> contours;
  std::vector<std::vector<int>> from;
};

/** tryBoolean, keeping each piece's group (see Traced). */
std::optional<Traced> traceBoolean(std::span<const std::vector<Contour>> input, const Rule& inside, double eps = 0.25);

/** The contours with their overlaps removed: one outline where they fill under the nonzero rule. */
std::vector<Contour> unite(std::span<const Contour> cs);

/** The signed area of a contour from its nodes and handles: positive counter-clockwise (y up). Known issue: the
 *  handles only approximate a cubic's area. */
double contourArea(const Contour& c);

/** A contour run backwards. */
Contour reverseContour(const Contour& c);

/** One stroke's contours turned as TrueType fills them: the biggest clockwise, the rest counter-clockwise, so
 *  overlapping strokes add up instead of cancelling. */
std::vector<Contour> orient(std::span<const Contour> cs);

/** Contours as closed polylines, each cubic cut into lines about `step` units long (at least `min`). */
std::vector<std::vector<Pt>> flatten(std::span<const Contour> cs, double step = 2, double min = 8);

/** An edge of a polyline. */
using Edge = std::array<Pt, 2>;

/** The edges of closed polylines that a horizontal line can cross (level ones dropped). */
std::vector<Edge> edges(std::span<const std::vector<Pt>> polys);

/** Where the horizontal line at `y` crosses `edges`, each as its x and its winding (1 going up, −1 down), unsorted.
 *  An edge counts from its lower end up to but not including its upper one, so a vertex is crossed once. */
std::vector<std::array<double, 2>> scanline(std::span<const Edge> es, double y);

/** Two points and the distance between them. */
struct Gap {
  double d;
  Pt a, b;
};

/** The narrowest a closed polyline comes across itself: the closest two of its points that lie at least `apart`
 *  along it (so neighbours don't count), and the distance. O(n²) on the points: flatten with a coarse step. */
std::optional<Gap> narrowest(std::span<const Pt> poly, double apart);

/** The closest two polylines come, and where. */
Gap polyGap(std::span<const Pt> p, std::span<const Pt> q);

}  // namespace curves
