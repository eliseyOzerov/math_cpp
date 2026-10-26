// One cubic, analysed exactly: what Pomax's Bezier.js does with a single curve and the Primer on Bézier Curves
// explains (https://pomax.github.io/bezierinfo). Roots of its coordinates in closed form, its extrema and exact
// bounds, inflections, the curve aligned to a line and its tight box, its canonical form (does it loop, has it a
// cusp, how many inflections), the de Casteljau points at a parameter, lookup tables, tangents, normals and the
// osculating circle, the nearest point, a curve through three points, and molding (dragging a point on the curve).
//
// Ported from fountain-ts/src/curves/bezier.ts; its header lists the use-cases of each function.
//
// Decisions (as in the TS):
// - Plain functions on Cubic, no class: they compose with geometry.hpp as they are.
// - Exact where the maths allows (roots of degree ≤ 3, extrema, inflections), so nodes at extrema land exactly;
//   numeric only where it must be (projection, refined by Newton in geometry's `nearest`).
// - The canonical form maps the first three points to (0, 0), (0, 1), (1, 1) and reads the class off where the
//   fourth lands, as in the Primer; the region boundaries are its formulas.
#pragma once

#include "curves/geometry.hpp"

#include <string_view>

namespace curves {

/** The real roots of a·t³ + b·t² + c·t + d, any order; a cubic by Cardano (the trigonometric form for three roots),
 *  each polished by two Newton steps. A leading coefficient tiny next to the rest drops the degree. */
std::vector<double> cubicRoots(double a, double b, double c, double d);
/** Parameters in 0…1 (a hair outside clamped in), sorted, near-duplicates merged. */
std::vector<double> inUnit(std::span<const double> ts, double eps = 1e-9);
/** The parameters in 0…1 where a one-dimensional cubic Bézier with control values p0…p3 equals `v`. */
std::vector<double> coordRoots(double p0, double p1, double p2, double p3, double v = 0);
/** The parameters where coordinate `coord` of `c` equals `v`, exactly (geometry's `roots` samples). */
inline std::vector<double> cubicCoordRoots(const Cubic& c, int coord, double v) {
  return coordRoots(c[0][coord], c[1][coord], c[2][coord], c[3][coord], v);
}

/** Where a cubic turns back in x and in y, strictly inside 0…1, and all of them sorted as `values`. */
struct Extrema {
  std::vector<double> x, y, values;
};
Extrema extrema(const Cubic& c);
/** The exact bounds of `c`: its ends and its extrema. */
Box bbox(const Cubic& c);
/** `c` moved and turned so the line from `a` to `b` lies along the x axis from the origin. */
Cubic align(const Cubic& c, Pt a, Pt b);
/** `c` aligned to its own chord. */
inline Cubic align(const Cubic& c) { return align(c, c[0], c[3]); }
/** The parameters strictly inside 0…1 where `c` changes the way it bends (B′ × B″ = 0), in order. */
std::vector<double> inflections(const Cubic& c);
/** The smallest box around `c` lined up with its chord: four corners, counter-clockwise from the start's side. */
std::array<Pt, 4> tightBox(const Cubic& c);

/** What kind of curve a cubic is (see canonical). */
enum class CubicKind { line, arch, singleInflection, doubleInflection, loop, cusp };
/** The kind's name as the TS spells it ('single inflection'…). */
std::string_view name(CubicKind k);
/** A cubic's canonical form: its kind, and where the map taking its first three points to (0, 0), (0, 1), (1, 1)
 *  puts its fourth (NaN, NaN when the first three lie on a line). */
struct Canonical {
  CubicKind kind;
  Pt at;
};
Canonical canonical(const Cubic& c, double eps = 1e-6);

/** The de Casteljau construction at t: the four control points, the three between them, the two between those and
 *  the point on the curve, in that order (Bezier.js's hull). */
std::array<Pt, 10> casteljau(const Cubic& c, double t);
/** `steps + 1` points along `c` at even steps of its parameter: a lookup table. */
std::vector<Pt> lut(const Cubic& c, int steps = 100);
/** The unit direction `c` runs at t; where a handle lies on its node, the direction to the next control point. */
Pt tangent(const Cubic& c, double t);
/** The unit normal at t: the tangent turned a quarter counter-clockwise. */
inline Pt normal(const Cubic& c, double t) { Pt d = tangent(c, t); return {-d.y, d.x}; }

/** The circle that fits a cubic best at a point: its centre (on the side it bends to; none where it runs straight),
 *  radius (infinite where straight) and signed curvature. */
struct Osculating {
  std::optional<Pt> center;
  double r;
  double k;
};
Osculating osculating(const Cubic& c, double t);

/** The point of a cubic nearest a point: its parameter, the point and the distance. */
struct Projection {
  double t;
  Pt p;
  double d;
};
Projection project(const Cubic& c, Pt p);

/** The quadratic from `S` to `E` through `B` (at parameter t, by default by distance), as a cubic. */
Cubic quadThrough(Pt S, Pt B, Pt E, std::optional<double> t = {});
/** cubicThrough's options: the parameter at B, the strut length behind B, and the direction at B. */
struct ThroughOptions {
  std::optional<double> t;
  std::optional<double> d1;
  std::optional<Pt> dir;
};
/** The cubic from `S` to `E` through `B` at parameter t (by default by distance), running there along `dir` (by
 *  default parallel to S→E) with its de Casteljau struts `d1` behind B and `d1·(1 − t)/t` ahead, as Bezier.js's
 *  cubicFromPoints, except the default struts together are a quarter of S→B→E (Bezier.js's loop on tall arches). */
Cubic cubicThrough(Pt S, Pt B, Pt E, const ThroughOptions& o = {});
/** `c` molded: its point at t dragged to `to`, its ends kept and its handles moved so the curve passes through it,
 *  the struts at t carried along unturned (the Primer's molding). Affine, so it works in any units. */
Cubic mold(const Cubic& c, double t, Pt to);

}  // namespace curves
