#include "curves/spline.hpp"

#include "curves/bezn.hpp"
#include "js.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#pragma STDC FP_CONTRACT OFF

namespace curves {

std::vector<Cubic> catmullRom(const std::vector<Pt>& pts, const CatmullRomOptions& o) {
  const double alpha = o.alpha, tension = o.tension;
  const bool closed = o.closed;
  const Pt metric = o.metric;
  std::vector<Pt> M, P;
  for (Pt p : pts) M.push_back(toMetric(p, metric));
  for (size_t i = 0; i < M.size(); i++)
    if (i == 0 || dist(M[i], M[i - 1]) > 1e-12) P.push_back(M[i]);
  if (closed && P.size() > 2 && dist(P[0], P.back()) < 1e-12) P.pop_back();
  const int n = int(P.size());
  if (n < 2) return {};
  auto at = [&](int i) -> Pt {
    if (closed) return P[((i % n) + n) % n];
    if (i < 0) return lerp(P[0], P[1], -1);
    if (i >= n) return lerp(P[n - 1], P[n - 2], -1);
    return P[i];
  };
  std::vector<Cubic> out;
  for (int i = 0; i < (closed ? n : n - 1); i++) {
    Pt p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
    auto gap = [&](Pt a, Pt b) { return js::max(std::pow(dist(a, b), alpha), 1e-9); };
    double t1 = gap(p0, p1), t2 = t1 + gap(p1, p2), t3 = t2 + gap(p2, p3);
    auto d = [](Pt a, Pt b, double k) -> Pt { return {(b[0] - a[0]) / k, (b[1] - a[1]) / k}; };
    auto tan = [&](Pt a, Pt b, Pt c, double ta, double tb, double tc) -> Pt {
      Pt x = d(a, b, tb - ta), y = d(a, c, tc - ta), z = d(b, c, tc - tb);
      return {x[0] - y[0] + z[0], x[1] - y[1] + z[1]};
    };
    Pt m1 = tan(p0, p1, p2, 0, t1, t2), m2 = tan(p1, p2, p3, t1, t2, t3);
    double k = (t2 - t1) / 3 * tension;
    out.push_back({p1, Pt{p1[0] + m1[0] * k, p1[1] + m1[1] * k}, Pt{p2[0] - m2[0] * k, p2[1] - m2[1] * k}, p2});
  }
  for (Cubic& c : out)
    for (Pt& p : c) p = fromMetric(p, metric);
  return out;
}

Path catmullRomPath(const std::vector<Pt>& pts, const CatmullRomOptions& o) {
  return Path::of(catmullRom(pts, o), o.closed, o.metric);
}

namespace {

/** The knots, points and weights a spline is evaluated with: a closed one's polygon wrapped `degree` points on and
 *  its knots even, an open one's clamped unless given. */
struct Setup {
  int p;
  std::vector<Pt> P;
  std::vector<double> W, U;
  int lo, hi;
};
Setup setup(const std::vector<Pt>& pts, const BSplineOptions& o) {
  const bool closed = o.closed;
  int p = std::max(1, std::min(o.degree, int(pts.size()) - 1));
  std::vector<Pt> P = pts;
  if (closed) P.insert(P.end(), pts.begin(), pts.begin() + p);
  std::vector<double> W;
  if (closed && o.weights) { W = *o.weights; W.insert(W.end(), o.weights->begin(), o.weights->begin() + p); }
  else if (o.weights) W = *o.weights;
  else W.assign(P.size(), 1);
  const int n = int(P.size());
  std::vector<double> U;
  if (closed || !o.knots) {
    for (int i = 0; i < n + p + 1; i++) U.push_back(closed ? i : std::min(std::max(i - p, 0), n - p));
  } else {
    if (int(o.knots->size()) != n + p + 1)
      throw std::invalid_argument("a spline of " + std::to_string(n) + " points and degree " + std::to_string(p) + " takes " +
                                  std::to_string(n + p + 1) + " knots, not " + std::to_string(o.knots->size()));
    U = *o.knots;
  }
  return {p, P, W, U, p, n};
}

using Hom = std::array<double, 3>;

/** The blossom of span k at parameters us (de Boor's steps, each at its own parameter), in homogeneous form. */
Hom blossom(const std::vector<Pt>& P, const std::vector<double>& W, const std::vector<double>& U, int p, int k,
            const std::vector<double>& us) {
  std::vector<Hom> d;
  for (int i = 0; i <= p; i++) {
    int j = k - p + i;
    double w = W[j];
    d.push_back({P[j][0] * w, P[j][1] * w, w});
  }
  for (int r = 1; r <= p; r++) {
    for (int i = p; i >= r; i--) {
      double a0 = U[k - p + i], a1 = U[k + i + 1 - r], a = a1 - a0 < 1e-14 ? 0 : (us[r - 1] - a0) / (a1 - a0);
      Hom e;
      for (int c = 0; c < 3; c++) e[c] = (1 - a) * d[i - 1][c] + a * d[i][c];
      d[i] = e;
    }
  }
  return d[p];
}

}  // namespace

Pt deBoor(const std::vector<Pt>& pts, double u, const BSplineOptions& o) {
  auto [p, P, W, U, lo, hi] = setup(pts, o);
  double v = U[lo] + (U[hi] - U[lo]) * js::min(js::max(u, 0), 1);
  int k = lo;
  while (k < hi - 1 && v >= U[k + 1]) k++;
  auto [x, y, w] = blossom(P, W, U, p, k, std::vector<double>(p, v));
  return {x / w, y / w};
}

std::vector<Cubic> bspline(const std::vector<Pt>& pts, const BSplineOptions& o) {
  if (pts.size() < 2) return {};
  auto [p, P, W, U, lo, hi] = setup(pts, o);
  std::vector<Cubic> out;
  for (int k = lo; k < hi; k++) {
    double a = U[k], b = U[k + 1];
    if (b - a < 1e-12) continue;
    std::vector<Pt> ctrl;
    std::vector<double> ws;
    for (int j = 0; j <= p; j++) {
      std::vector<double> us(p - j, a);
      us.insert(us.end(), j, b);
      auto [x, y, w] = blossom(P, W, U, p, k, us);
      ctrl.push_back({x / w, y / w});
      ws.push_back(w);
    }
    if (p == 1) out.push_back(lineCubic(ctrl[0], ctrl[1]));
    else {
      ToCubicsOptions t;
      t.weights = ws;
      t.tol = o.tol;
      for (auto& c : toCubics(ctrl, t)) out.push_back(c);
    }
  }
  return out;
}

Path bsplinePath(const std::vector<Pt>& pts, const BSplineOptions& o) { return Path::of(bspline(pts, o), o.closed, o.metric); }

}  // namespace curves
