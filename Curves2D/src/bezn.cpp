// See include/curves/bezn.hpp. Ported from fountain-ts/src/curves/bezn.ts line for line.
#include "curves/bezn.hpp"

#include "js.hpp"

#include <cmath>

// No fused multiply-adds: JS rounds every operation.
#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

/** Rounds of fitBezier: each solves the control points, then moves every point's parameter to its nearest place on
 *  the curve so far. Chord-length shares alone put an arch's handles 17% of its width off; five rounds bring that
 *  to 3% and the curve to within a few thousandths of the points. */
constexpr int FIT_ROUNDS = 5;

/** The parameter near `t0` where a Bezier of any order comes nearest `p`: Newton's steps on (B − p)·B′. */
double nearestT(const std::vector<Pt>& pts, Pt p, double t0) {
  std::vector<Pt> d1 = derivativeN(pts);
  std::vector<Pt> d2 = d1.size() > 1 ? derivativeN(d1) : std::vector<Pt>{{0, 0}};
  double t = t0;
  for (int k = 0; k < 4; k++) {
    Pt q = bezierAt(pts, t), v = bezierAt(d1, t), w = bezierAt(d2, t);
    Pt r{q[0] - p[0], q[1] - p[1]};
    double f = r[0] * v[0] + r[1] * v[1];
    double df = v[0] * v[0] + v[1] * v[1] + r[0] * w[0] + r[1] * w[1];
    if (std::abs(df) < 1e-14) break;
    t = js::min(1, js::max(0, t - f / df));
  }
  return t;
}

/** Points r minimising |A r − p|² for both coordinates, by the normal equations. */
std::vector<Pt> leastSquares(const std::vector<std::vector<double>>& A, const std::vector<Pt>& p) {
  size_t m = A[0].size();
  std::vector<std::vector<double>> N(m, std::vector<double>(m));
  for (size_t i = 0; i < m; i++)
    for (size_t j = 0; j < m; j++) {
      double s = 0;
      for (auto& r : A) s = s + r[i] * r[j];
      N[i][j] = s;
    }
  std::vector<double> bx(m), by(m);
  for (size_t i = 0; i < m; i++) {
    double sx = 0, sy = 0;
    for (size_t k = 0; k < A.size(); k++) sx = sx + A[k][i] * p[k][0];
    for (size_t k = 0; k < A.size(); k++) sy = sy + A[k][i] * p[k][1];
    bx[i] = sx;
    by[i] = sy;
  }
  std::vector<double> x = solve(N, bx), y = solve(N, by);
  std::vector<Pt> out;
  for (size_t i = 0; i < x.size(); i++) out.push_back({x[i], y[i]});
  return out;
}

}  // namespace

Pt bezierAt(const std::vector<Pt>& pts, double t) {
  std::vector<Pt> q = pts;
  while (q.size() > 1) {
    std::vector<Pt> next;
    for (size_t i = 0; i + 1 < q.size(); i++) next.push_back(lerp(q[i], q[i + 1], t));
    q = std::move(next);
  }
  return q[0];
}

std::vector<std::vector<Pt>> casteljauN(const std::vector<Pt>& pts, double t) {
  std::vector<std::vector<Pt>> levels{pts};
  while (levels.back().size() > 1) {
    const std::vector<Pt> q = levels.back();
    std::vector<Pt> next;
    for (size_t i = 0; i + 1 < q.size(); i++) next.push_back(lerp(q[i], q[i + 1], t));
    levels.push_back(std::move(next));
  }
  return levels;
}

std::array<std::vector<Pt>, 2> splitN(const std::vector<Pt>& pts, double t) {
  auto levels = casteljauN(pts, t);
  std::vector<Pt> a, b;
  for (auto& l : levels) { a.push_back(l.front()); b.push_back(l.back()); }
  std::reverse(b.begin(), b.end());
  return {a, b};
}

std::vector<Pt> derivativeN(const std::vector<Pt>& pts) {
  double n = double(pts.size()) - 1;
  std::vector<Pt> out;
  for (size_t i = 0; i + 1 < pts.size(); i++) {
    Pt p = pts[i + 1];
    out.push_back({n * (p[0] - pts[i][0]), n * (p[1] - pts[i][1])});
  }
  return out;
}

std::vector<Pt> raise(const std::vector<Pt>& pts) {
  size_t k = pts.size();
  std::vector<Pt> out;
  for (size_t i = 0; i < k + 1; i++)
    out.push_back(i == 0 ? pts[0] : i == k ? pts[k - 1] : lerp(pts[i], pts[i - 1], double(i) / double(k)));
  return out;
}

std::vector<Pt> lower(const std::vector<Pt>& pts) {
  int k = int(pts.size()) - 1;
  if (k < 2) return pts;
  std::vector<std::vector<double>> M(k + 1, std::vector<double>(k, 0.0));
  M[0][0] = 1;
  M[k][k - 1] = 1;
  for (int i = 1; i < k; i++) {
    M[i][i] = 1 - double(i) / k;
    M[i][i - 1] = double(i) / k;
  }
  return leastSquares(M, pts);
}

std::vector<Pt> fitBezier(const std::vector<Pt>& points, int order) {
  Pt a = points[0], b = points[points.size() - 1];
  if (int(points.size()) <= order + 1) {
    // Fewer points than unknowns: take the line through the ends at the order asked for.
    std::vector<Pt> out;
    for (int i = 0; i < order + 1; i++) out.push_back(lerp(a, b, double(i) / order));
    return out;
  }
  std::vector<double> run{0};
  for (size_t i = 1; i < points.size(); i++) run.push_back(run[i - 1] + dist(points[i - 1], points[i]));
  double total = js::orElse(run.back(), 1);
  std::vector<double> ts;
  for (double s : run) ts.push_back(s / total);
  std::vector<Pt> ctrl;
  for (int round = 0; round < FIT_ROUNDS; round++) {
    std::vector<std::vector<double>> B;
    for (double t : ts) B.push_back(bernstein(order, t));
    // Pin the ends: move their share to the right-hand side and solve for the inner points only.
    std::vector<std::vector<double>> rows;
    for (auto& r : B) rows.push_back(order >= 1 ? std::vector<double>(r.begin() + 1, r.begin() + order) : std::vector<double>{});
    std::vector<Pt> rhs;
    for (size_t j = 0; j < points.size(); j++) {
      Pt p = points[j];
      rhs.push_back({p[0] - B[j][0] * a[0] - B[j][order] * b[0], p[1] - B[j][0] * a[1] - B[j][order] * b[1]});
    }
    ctrl.clear();
    ctrl.push_back(a);
    if (order > 1) for (Pt q : leastSquares(rows, rhs)) ctrl.push_back(q);
    ctrl.push_back(b);
    std::vector<double> next;
    for (size_t j = 0; j < points.size(); j++)
      next.push_back(j == 0 ? 0 : j == points.size() - 1 ? 1 : nearestT(ctrl, points[j], ts[j]));
    ts = std::move(next);
  }
  return ctrl;
}

Pt rationalAt(const std::vector<Pt>& pts, const std::vector<double>& weights, double t) {
  int n = int(pts.size()) - 1;
  std::vector<double> b = bernstein(n, t);
  double x = 0, y = 0, w = 0;
  for (size_t i = 0; i < pts.size(); i++) {
    Pt p = pts[i];
    double k = b[i] * (i < weights.size() ? weights[i] : 1);
    x += k * p[0];
    y += k * p[1];
    w += k;
  }
  return {x / w, y / w};
}

std::vector<Cubic> toCubics(const std::vector<Pt>& pts, const ToCubicsOptions& o) {
  const std::optional<std::vector<double>>& weights = o.weights;
  bool plain = true;
  if (weights)
    for (double w : *weights)
      if (!(std::abs(w - (*weights)[0]) < 1e-12)) { plain = false; break; }
  if (plain && pts.size() <= 4) {
    std::vector<Pt> q = pts;
    while (q.size() < 4) q = raise(q);
    return {Cubic{q[0], q[1], q[2], q[3]}};
  }
  std::vector<Pt> ps;
  for (int i = 0; i < o.samples + 1; i++)
    ps.push_back(plain ? bezierAt(pts, double(i) / o.samples) : rationalAt(pts, *weights, double(i) / o.samples));
  return fitCubics(ps, o.tol, FitOptions{180});
}

std::vector<double> bernstein(int n, double t) {
  std::vector<double> b{1};
  for (int k = 1; k <= n; k++) {
    b.push_back(0);
    // Pascal's triangle, each row blended by (1 − t) and t.
    for (int i = k; i >= 0; i--) b[i] = (i < k ? b[i] * (1 - t) : 0) + (i > 0 ? b[i - 1] * t : 0);
  }
  return b;
}

std::vector<double> solve(const std::vector<std::vector<double>>& A, const std::vector<double>& b) {
  size_t n = b.size();
  std::vector<std::vector<double>> M;
  for (size_t i = 0; i < A.size(); i++) {
    M.push_back(A[i]);
    M[i].push_back(b[i]);
  }
  for (size_t c = 0; c < n; c++) {
    size_t piv = c;
    for (size_t r = c + 1; r < n; r++)
      if (std::abs(M[r][c]) > std::abs(M[piv][c])) piv = r;
    std::swap(M[c], M[piv]);
    double d = M[c][c];
    if (std::abs(d) < 1e-14) continue;
    for (size_t r = 0; r < n; r++) {
      if (r == c) continue;
      double f = M[r][c] / d;
      if (js::truthy(f)) for (size_t k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  std::vector<double> out;
  for (size_t i = 0; i < M.size(); i++) out.push_back(std::abs(M[i][i]) < 1e-14 ? 0 : M[i][n] / M[i][i]);
  return out;
}

}  // namespace curves
