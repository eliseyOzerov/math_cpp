// Approximations and polygons: one cubic fitted through points (how the ink's rails become curves), cubics through
// many points to a tolerance (fitCubics), cubics as quadratic splines (how outlines become TrueType contours), and the
// convex hulls and clips that join a nib's stamps into ink.
//
// Ported from fountain-ts/src/curves/fit.ts.
//
// Decisions:
// - The hulls and clips are polygon operations, not fitting, but they live beside the fitting as in the TS.
// - fitCubic's { c, err } is a struct (CubicFit); fitCubics' option object is a struct with the TS default.
// - hull's comparator `a[0] - b[0] || a[1] - b[1]` is a stable sort in that order; `sorted` keeps the input as given.
#pragma once

#include "curves/bezier.hpp"
#include "curves/geometry.hpp"

#include <vector>

namespace curves {

/** A fitted cubic and the furthest any point lies from it. */
struct CubicFit {
  Cubic c;
  double err;
};
/** One cubic through points from the first to the last, leaving along `tA` and arriving along −`tB` (both unit), by
 *  least squares on the handle lengths with Newton steps on each point's parameter. `err` is the furthest any point
 *  lies from the curve. */
CubicFit fitCubic(const std::vector<Pt>& points, Pt tA, Pt tB);

/** Options of fitCubics: points that turn by more than `corner` degrees are cut and the corner kept. */
struct FitOptions {
  double corner = 60;
};
/** Cubics through many points to within `tol` (Schneider's method): one cubic fitted to the run, split at the point
 *  it misses most, each half fitted in turn and the two joined smoothly there (along the line through the split
 *  point's neighbours). Where the points turn by more than `corner` degrees the run is cut and the corner kept. */
std::vector<Cubic> fitCubics(const std::vector<Pt>& points, double tol, FitOptions o = {});

/** The controls of a quadratic spline that follows cubic `c` within `tol`, as cu2qu's cubic_approx_spline: one
 *  piece, two, … sixteen, until it fits. */
std::vector<Pt> cubicToQuads(const Cubic& c, double tol);

/** The convex hull of `points`, counter-clockwise (monotone chain). `sorted`: they are already in x order (then y). */
std::vector<Pt> hull(const std::vector<Pt>& points, bool sorted = false);
/** The hull of two convex polygons (two neighbouring stamps): each polygon's two chains between its leftmost and
 *  rightmost points already run in x order, so the four are merged instead of sorted. Rounding can leave a chain a
 *  hair out of order; then the points are sorted after all. */
std::vector<Pt> mergeHulls(const std::vector<Pt>& A, const std::vector<Pt>& B);

/** A plane: the side where (p − at)·n ≥ 0 is kept. */
struct Plane {
  Pt at, n;
};
/** A convex polygon trimmed to the kept side of a plane (an end of a stroke cut square). */
std::vector<Pt> clipPolygon(const std::vector<Pt>& poly, const Plane& plane);

}  // namespace curves
