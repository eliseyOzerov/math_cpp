#include "curves/intersect.hpp"

#include "curves/bezier.hpp"

#include <algorithm>
#include <cmath>

#pragma STDC FP_CONTRACT OFF

namespace curves {

std::vector<LineCrossing> lineCrossings(const Cubic& c, Pt a, Pt b, bool segment) {
  Pt d = b - a;
  double l2 = dot(d, d);
  if (l2 < 1e-24) return {};
  std::vector<double> ts = cubicCoordRoots(align(c, a, b), 1, 0);
  std::stable_sort(ts.begin(), ts.end(), [](double x, double y) { return x < y; });
  std::vector<LineCrossing> out;
  for (double t : ts) {
    Pt p = bez(c, t);
    double s = dot(p - a, d) / l2;
    if (!segment || (s >= -1e-9 && s <= 1 + 1e-9)) out.push_back({t, s, p});
  }
  return out;
}

std::vector<CircleCrossing> circleCrossings(const Cubic& c, Pt o, double r, int samples) {
  auto f = [&](double t) { Pt p = bez(c, t); return std::pow(p.x - o.x, 2) + std::pow(p.y - o.y, 2) - r * r; };
  std::vector<CircleCrossing> out;
  double t0 = 0, f0 = f(0);
  if (std::abs(f0) < 1e-12) out.push_back({0, c[0]});
  for (int i = 1; i <= samples; i++) {
    double t1 = double(i) / samples, f1 = f(t1);
    if (std::abs(f1) < 1e-12) out.push_back({t1, bez(c, t1)});
    else if (f0 * f1 < 0 && std::abs(f0) >= 1e-12) {
      double lo = t0, hi = t1, flo = f0;
      for (int k = 0; k < 50; k++) {
        double m = (lo + hi) / 2, fm = f(m);
        if (flo * fm <= 0) hi = m; else { lo = m; flo = fm; }
      }
      double t = (lo + hi) / 2;
      out.push_back({t, bez(c, t)});
    }
    t0 = t1; f0 = f1;
  }
  return out;
}

namespace {

/** A crossing [t, u] of a and b made exact by Newton's steps on a(t) − b(u) = 0, kept as found if a step would leave
 *  the curves or the two run parallel there. */
Crossing polish(const Cubic& a, const Cubic& b, Crossing tu) {
  double t = tu[0], u = tu[1];
  for (int k = 0; k < 4; k++) {
    Pt r = bez(a, t) - bez(b, u), da = bezD(a, t), db = bezD(b, u);
    double det = cross(db, da);
    if (std::abs(det) < 1e-14) break;
    // Solve da·Δt − db·Δu = −r by Cramer's rule.
    double dt = cross(r, db) / det, du = -cross(da, r) / det;
    double t2 = t + dt, u2 = u + du;
    if (t2 < 0 || t2 > 1 || u2 < 0 || u2 > 1) break;
    t = t2; u = u2;
  }
  return {t, u};
}

}  // namespace

std::vector<SelfCrossing> selfCrossings(std::span<const Cubic> segs, bool closed) {
  // Pieces that rise or fall all along in x and in y, each remembering its segment and parameters.
  struct Part { Cubic c; int i; double t0, t1; };
  std::vector<Part> pieces;
  for (size_t i = 0; i < segs.size(); i++) {
    const Cubic& c = segs[i];
    std::vector<double> ts{0};
    for (double t : extrema(c).values) if (t > 1e-6 && t < 1 - 1e-6) ts.push_back(t);
    ts.push_back(1);
    for (size_t k = 1; k < ts.size(); k++)
      if (ts[k] - ts[k - 1] > 1e-9) pieces.push_back({trim(c, ts[k - 1], ts[k]), int(i), ts[k - 1], ts[k]});
  }
  std::vector<Pt> ps;
  for (auto& c : segs) ps.insert(ps.end(), c.begin(), c.end());
  Box all = bounds(ps);
  double tol = 1e-6 * std::max(1e-9, curves::hypot(all[1].x - all[0].x, all[1].y - all[0].y));
  size_t n = pieces.size();
  std::vector<SelfCrossing> out;
  for (size_t x = 0; x < n; x++) for (size_t y = x + 1; y < n; y++) {
    // Neighbours along the path share an end: the end of x and the start of y, or round a closed path's seam.
    std::vector<SharedEnd> shared;
    if (y == x + 1) shared.push_back({1, 0});
    if (closed && x == 0 && y == n - 1) shared.push_back({0, 1});
    const Part &P = pieces[x], &Q = pieces[y];
    for (const Crossing& hit : crossings(P.c, Q.c, tol, shared).value_or(std::vector<Crossing>{})) {
      auto [t, u] = polish(P.c, Q.c, hit);
      Pt p = bez(P.c, t);
      if (std::any_of(out.begin(), out.end(), [&](const SelfCrossing& o) {
            return curves::hypot(o.p.x - p.x, o.p.y - p.y) < 100 * tol;
          }))
        continue;
      out.push_back({{P.i, P.t0 + (P.t1 - P.t0) * t}, {Q.i, Q.t0 + (Q.t1 - Q.t0) * u}, p});
    }
  }
  return out;
}

}  // namespace curves
