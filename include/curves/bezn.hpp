// Béziers of any order and rational Béziers: evaluating, splitting and differentiating them, raising and lowering
// their order, fitting one of a given order to points, and bringing any of them back to cubics.
//
// Use-cases, by function:
// - bezierAt, casteljauN: a point on a quadratic (TrueType's curves), a quartic or higher drawn by a script; the
//   de Casteljau levels show how a curve is built, for a panel that teaches or a construction drawn in a proof.
// - splitN, derivativeN: cut a high-order curve, or take its hodograph (the derivative is a Bézier one order lower),
//   to find where it turns or how fast it runs.
// - raise: give a quadratic or a line the cubic's four points exactly (a TrueType contour read back to cubics), or
//   add a control point so a shape gets one more handle to pull.
// - lower: the closest curve with one control point fewer (least squares), when a quartic from a fit or a raise
//   should come back to a cubic and an approximation is fine.
// - fitBezier: one curve of a chosen order through many points, when a single smooth piece matters more than
//   matching every point (a stem's gentle swell, a profile through measured widths).
// - rationalAt, toCubics with weights: true conics (a circle's quarter, an ellipse's arc, a parabola) and curves
//   pulled towards a control point by its weight; a weight is a "tension" per handle that plain cubics lack.
// - toCubics: anything here into the engine's cubics, exactly when it can be and to a tolerance otherwise.
//
// Ported from fountain-ts/src/curves/bezn.ts.
//
// Decisions:
// - Control points are a std::vector<Pt> (any length >= 2), so a cubic here is the same four points as a Cubic
//   elsewhere. A rational Bezier is a Rational (points and weights); the TS functions take the weights as a separate
//   argument, and so do the ones here, with Rational overloads for convenience.
// - toCubics' option object is a struct with the TS defaults; `weights` is optional.
// - A TS array of arrays (solve's matrix) is a std::vector<std::vector<double>>.
// - Lowering and fitting solve small normal equations with Gaussian elimination: orders stay under ten, where that
//   is exact enough and needs no linear algebra package. Rational curves beyond cubics are sampled and fitted with
//   fitCubics, which keeps one path for every case.
// - Empty point lists are outside the contract (the TS returns undefined entries); they are not handled.
#pragma once

#include "curves/fit.hpp"
#include "curves/geometry.hpp"

#include <array>
#include <optional>
#include <vector>

namespace curves {

/** A rational Bezier: control points and a weight for each (a missing weight is 1). */
struct Rational {
  std::vector<Pt> pts;
  std::vector<double> weights;
};

/** The point at `t` on a Bezier of any order (de Casteljau). */
Pt bezierAt(const std::vector<Pt>& pts, double t);

/** Every level of de Casteljau's construction at `t`: the control points, then each row of lerps down to the point
 *  on the curve (levels[n][0]). The rows' first and last points are the two halves' control points. */
std::vector<std::vector<Pt>> casteljauN(const std::vector<Pt>& pts, double t);

/** A Bezier of any order cut at `t` into two of the same order. */
std::array<std::vector<Pt>, 2> splitN(const std::vector<Pt>& pts, double t);

/** The derivative (hodograph): a Bezier one order lower whose points are n(p[i+1] − p[i]). */
std::vector<Pt> derivativeN(const std::vector<Pt>& pts);

/** The same curve with one more control point (degree elevation): q[i] = (i/k)·p[i−1] + (1 − i/k)·p[i], k = n + 1. */
std::vector<Pt> raise(const std::vector<Pt>& pts);

/** The curve with one control point fewer that is closest to this one (degree reduction by least squares on the
 *  elevation matrix M: solve MᵀM r = Mᵀp). Exact when the curve was raised from one of the lower order. */
std::vector<Pt> lower(const std::vector<Pt>& pts);

/** One Bezier with `order + 1` control points fitted to points by least squares, its ends on the first and last
 *  points; each point starts at its share of the polyline's length and is moved to its nearest place on the curve
 *  between rounds. */
std::vector<Pt> fitBezier(const std::vector<Pt>& points, int order = 3);

/** The point at `t` on a rational Bezier: each control point pulls by its weight (all 1 is the plain curve). */
Pt rationalAt(const std::vector<Pt>& pts, const std::vector<double>& weights, double t);
inline Pt rationalAt(const Rational& r, double t) { return rationalAt(r.pts, r.weights, t); }

/** Options of toCubics: `weights` make the curve rational; `tol` and `samples` are for what has to be fitted. */
struct ToCubicsOptions {
  std::optional<std::vector<double>> weights;
  double tol = 1e-3;
  int samples = 64;
};
/** Any Bezier as the engine's cubics: a line or quadratic raised exactly, a cubic as it is, and anything higher or
 *  rational sampled and fitted to `tol`. A rational quadratic is a conic: weights [1, cos(θ/2), 1] give a circular
 *  arc of angle θ. */
std::vector<Cubic> toCubics(const std::vector<Pt>& pts, const ToCubicsOptions& o = {});
inline std::vector<Cubic> toCubics(const Rational& r, ToCubicsOptions o = {}) {
  o.weights = r.weights;
  return toCubics(r.pts, o);
}

/** The Bernstein weights of order n at t. */
std::vector<double> bernstein(int n, double t);

/** Gaussian elimination with partial pivoting; small, dense systems only. */
std::vector<double> solve(const std::vector<std::vector<double>>& A, const std::vector<double>& b);

}  // namespace curves
