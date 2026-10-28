// Splines: curves through points (Catmull-Rom) and curves pulled by points (B-splines, plain and rational), both as
// the engine's cubics so a path can be made of either.
//
// Use-cases, by function:
// - catmullRom, catmullRomPath: a stroke that must pass through given points, such as a hand's samples, the places a
//   panel lets the user drag, a spine through a few named heights, or a wobble's keys. Centripetal (alpha 0.5, the
//   default) never loops or cusps between close points; alpha 0 is the uniform spline, alpha 1 the chordal one.
//   Tension below 1 pulls the curve tighter to its polyline, above 1 rounds it out.
// - bspline, bsplinePath: a smooth stroke steered by a control polygon that it doesn't touch (except at clamped
//   ends), so moving one point changes only the curve near it. Good for long, even curves (a swash, an S's spine, a
//   tail) and for curves with a corner made on purpose by repeating a knot.
// - weights (rational B-splines, NURBS): a point that pulls harder or softer; exact circles and ellipses from a
//   square control polygon with corner weights √½ at degree 2.
// - deBoor: a single point on a B-spline without building the cubics, to measure or to place a dot.
//
// Ported from fountain-ts/src/curves/spline.ts (its header has the decisions: Catmull-Rom in the metric, B-splines
// blossomed into Béziers, high-degree and rational spans fitted).
//
// Decisions:
// - The option objects are structs with the TS defaults; bsplinePath's extra `metric` is a field of BSplineOptions.
#pragma once

#include "curves/geometry.hpp"
#include "curves/path.hpp"

#include <optional>
#include <vector>

namespace curves {

struct CatmullRomOptions {
  /** 0 uniform, 0.5 centripetal (no loops, the default), 1 chordal. */
  double alpha = 0.5;
  /** How far the handles reach: 1 the spline's own, 0 the polyline. */
  double tension = 1;
  /** Through the first point again at the end. */
  bool closed = false;
  /** What a unit across and a unit up are worth, as in Path. */
  Pt metric{1, 1};
};

/** Cubics through every point in turn (a Catmull-Rom spline, Barry and Goldman's form for any alpha). An open
 *  spline's ends take a mirrored neighbour, so they leave towards the next point. */
std::vector<Cubic> catmullRom(const std::vector<Pt>& pts, const CatmullRomOptions& o = {});

/** A Catmull-Rom spline through the points, as a path measured with the options' metric. */
Path catmullRomPath(const std::vector<Pt>& pts, const CatmullRomOptions& o = {});

struct BSplineOptions {
  /** 1 a polyline, 2 quadratic, 3 cubic (the default); at most one less than the points. */
  int degree = 3;
  /** Non-decreasing, points + degree + 1 of them; by default clamped and even (the curve starts and ends on the
   *  first and last points). A knot repeated `degree` times puts a corner on the control point there. */
  std::optional<std::vector<double>> knots;
  /** One per point (rational, NURBS); all 1 by default. */
  std::optional<std::vector<double>> weights;
  /** Periodic: the control polygon wraps and the curve closes smoothly (knots are then ignored). */
  bool closed = false;
  /** How closely a span of degree above 3 or with weights is fitted with cubics. */
  double tol = 1e-3;
  /** bsplinePath's metric. */
  Pt metric{1, 1};
};

/** The point at `u` on a B-spline (u runs over the knots' span, 0…1 rescaled). */
Pt deBoor(const std::vector<Pt>& pts, double u, const BSplineOptions& o = {});

/** A B-spline as cubics: each span between distinct knots becomes one Bézier of the spline's degree, exactly (by
 *  blossoming), and then a cubic (raised from a lower degree, or fitted from a higher or rational one). */
std::vector<Cubic> bspline(const std::vector<Pt>& pts, const BSplineOptions& o = {});

/** A B-spline as a path (closed when the options say so). */
Path bsplinePath(const std::vector<Pt>& pts, const BSplineOptions& o = {});

}  // namespace curves
