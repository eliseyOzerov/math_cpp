// Offsets and outlines of cubics, as Bezier.js makes them: a curve reduced to simple pieces, each scaled out along
// its normals, and the two sides joined into an outline; and a curve as circular arcs.
//
// Ported from fountain-ts/src/curves/offset.ts; its header lists the use-cases of each function.
//
// Decisions:
// - Reach (a number, or a function of t) is a std::variant<double, std::function<double(double)>>: scale branches on
//   which it holds, as the TS does on typeof. A plain double converts to it implicitly.
// - Arc's optional center is a std::optional<Pt>; reduce's pieces are a struct Reduced {c, t0, t1}.
// - outline's optional d3, d4 are std::optional<double>; d2 defaults to d1 when left out.
// - Positive distances are on the left (the normal of bezier.hpp).
#pragma once

#include "curves/bezier.hpp"
#include "curves/geometry.hpp"

#include <functional>
#include <optional>
#include <variant>
#include <vector>

namespace curves {

/** Distance along a curve: a number, or a function of the parameter t (graduated). */
using Reach = std::variant<double, std::function<double(double)>>;

/** Whether an offset can follow `c` as one cubic: its handles on one side of the chord, and its normals turning by
 *  less than 60°. */
bool simple(const Cubic& c);

/** A piece of a reduced curve with the parameters it spans on the curve. */
struct Reduced {
  Cubic c;
  double t0, t1;
};
/** `c` cut into simple pieces: first where it turns in x or y or changes its bend, then each run grown in steps of a
 *  hundredth while it stays simple. Each piece with the parameters it spans on `c`. */
std::vector<Reduced> reduce(const Cubic& c);

/** A simple piece moved out by `d` along its normals: the ends exactly (at d(0) and d(1)), the handles kept along
 *  the ends' tangents where the end normals meet (or, graduated, moved by d(⅓) and d(⅔) away from that point). A
 *  line, or a piece whose end normals are parallel, is moved along its normals. */
Cubic scale(const Cubic& c, const Reach& d);

/** The curve at distance `d` (on its left; negative on its right), as cubics: reduced, then each piece scaled. A
 *  graduated d is a function of t along the whole curve. */
std::vector<Cubic> offset(const Cubic& c, const Reach& d);

/** The point at distance `d` off `c` at `t`, along its normal. */
Pt offsetAt(const Cubic& c, double t, double d);

/** A closed outline round `c`: `d1` to its left and `d2` (default d1) to its right, tapering to `d3` and `d4` at its
 *  end when given (linearly by length). The left side forwards, a line across the end, the right side backwards and a
 *  line across the start. */
Contour outline(const Cubic& c, double d1, std::optional<double> d2 = {}, std::optional<double> d3 = {},
                std::optional<double> d4 = {});

/** The outline as one closed shape per reduced piece (each its left side, an end, its right side and a start). */
std::vector<Contour> outlineShapes(const Cubic& c, double d1, std::optional<double> d2 = {},
                                   std::optional<double> d3 = {}, std::optional<double> d4 = {});

/** A circular arc of a curve: its center and radius (no center and an infinite radius where the curve is straight),
 *  the angles it runs from and to (radians, counterclockwise from +x), which way, its ends and the span of t. */
struct Arc {
  std::optional<Pt> center;
  double r;
  double start, end;
  bool ccw;
  Pt from, to;
  double t0, t1;
};

/** `c` as circular arcs that stay within `threshold` of it: from each arc's end, the longest next arc that fits
 *  (searched by halving). Bezier.js's `arcs`. */
std::vector<Arc> arcs(const Cubic& c, double threshold = 0.5);

}  // namespace curves
