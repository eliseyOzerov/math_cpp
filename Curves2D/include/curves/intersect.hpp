// Where curves meet lines, circles, other curves and themselves.
//
// Use-cases, by function:
// - lineCrossings: where a stroke crosses any slanted line (a bar's angle, a cut at the italic slope, the line
//   through two named points), exactly; segment: false takes the whole line.
// - circleCrossings: where a curve comes within a radius of a point: a dot's clearance from a stem, where a serif of
//   a given reach lands on a bowl, a brush's reach in a sculpting panel.
// - curveCrossings: two cubics, pairs [t, u]; it is boolean's `crossings`, under this name too so every intersection
//   is found from one header.
// - selfCrossings: where a centerline crosses itself (a loop, a figure-eight), to cut it into pieces, to find where a
//   wobble or an offset turned a stroke over on itself, or to warn before inking it.
//
// Decisions: lines are exact (the cubic aligned to the line, then Cardano). Circles are a degree-6 equation, found by
// sampling the squared distance and bisecting each change of sign; a curve that only grazes the circle between
// samples is missed. Self-crossings cut every segment where it turns in x or y, as Bezier.js does, and then halve
// pairs of pieces with the shared ends left out.
//
// Ported from fountain-ts/src/curves/intersect.ts.
#pragma once

#include "curves/boolean.hpp"
#include "curves/geometry.hpp"

namespace curves {

/** A crossing of a curve and a line: `t` on the curve, `s` along the line (0 at a, 1 at b), and the point. */
struct LineCrossing {
  double t, s;
  Pt p;
};
/** Where `c` crosses the line from `a` to `b`, in order along the curve. Only between a and b unless `segment` is
 *  false. */
std::vector<LineCrossing> lineCrossings(const Cubic& c, Pt a, Pt b, bool segment = true);

/** A crossing of a curve and a circle: its parameter and point. */
struct CircleCrossing {
  double t;
  Pt p;
};
/** Where `c` crosses the circle round `o` of radius `r`: parameters in order with their points. */
std::vector<CircleCrossing> circleCrossings(const Cubic& c, Pt o, double r, int samples = 64);

/** Two cubics' crossings, pairs [t, u]: boolean's `crossings`. */
inline std::optional<std::vector<Crossing>> curveCrossings(const Cubic& a, const Cubic& b, double tol = 1e-3,
                                                           std::span<const SharedEnd> shared = {}) {
  return crossings(a, b, tol, shared);
}

/** A place on a path: segment `i`, parameter `t`. */
struct SegAt {
  int i;
  double t;
};
/** A self-crossing: the two places on the path, the earlier first, and the point. */
struct SelfCrossing {
  SegAt a, b;
  Pt p;
};
/** Where segments `segs` (one path, closed or not) cross themselves: each crossing once, as the two places
 *  [segment, t] on it, the earlier first. Ends where one segment meets the next are not crossings. Throws on no
 *  segments, as the TS's bounds of nothing does. */
std::vector<SelfCrossing> selfCrossings(std::span<const Cubic> segs, bool closed = false);

}  // namespace curves
