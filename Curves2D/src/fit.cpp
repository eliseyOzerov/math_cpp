// See include/curves/fit.hpp. Ported from fountain-ts/src/curves/fit.ts line for line.
#include "curves/fit.hpp"

#include "js.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>

// No fused multiply-adds: JS rounds every operation.
#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

constexpr int ROUNDS = 8;
constexpr double MIN_HANDLE = 1e-3;
constexpr int MAX_DEPTH = 12;

bool before(Pt a, Pt b) { return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]); }

/** Whether the cubic of differences [p0…p3] stays within `tol` of zero, halving it until its controls do. */
bool fitsInside(Pt p0, Pt p1, Pt p2, Pt p3, double tol) {
  if (len(p1) <= tol && len(p2) <= tol) return true;
  Pt mid = (p0 + (p1 + p2) * 3 + p3) * 0.125;
  if (len(mid) > tol) return false;
  Pt d3 = ((p3 + p2) - (p1 + p0)) * 0.125;
  return fitsInside(p0, (p0 + p1) * 0.5, mid - d3, mid, tol) && fitsInside(mid, mid + d3, (p2 + p3) * 0.5, p3, tol);
}

/** Where a cubic's end tangents cross, ahead of both ends: the control of the quadratic with the same tangents. */
std::optional<Pt> apex(const Cubic& c) {
  Pt d = norm(c[1] - c[0]);
  auto m = meet(c[0], d, c[3], norm(c[2] - c[3]));
  if (m && (*m)[0] > 0 && (*m)[1] > 0) return c[0] + d * (*m)[0];
  return std::nullopt;
}

}  // namespace

CubicFit fitCubic(const std::vector<Pt>& points, Pt tA, Pt tB) {
  std::vector<Pt> P;
  for (Pt q : points) if (P.empty() || dist(q, P.back()) > 1e-9) P.push_back(q);
  if (points.empty()) return {{Pt{}, Pt{}, Pt{}, Pt{}}, 0};
  Pt last = points.back();
  if (P.size() < 2) return {{P[0], P[0], last, last}, 0};
  P.back() = last;
  Pt A = P[0], B = last;
  double chord = dist(A, B);
  if (P.size() == 2) return {{A, A + tA * (chord / 3), B + tB * (chord / 3), B}, 0};
  std::vector<double> run = runLengths(P);
  std::vector<double> u;
  for (double x : run) u.push_back(x / run.back());
  std::optional<CubicFit> best;
  for (int it = 0; it < ROUNDS; it++) {
    double c00 = 0, c01 = 0, c11 = 0, x0 = 0, x1 = 0;
    for (size_t i = 0; i < P.size(); i++) {
      Pt p = P[i];
      double t = u[i], v = 1 - t, b0 = v * v * v, b1 = 3 * v * v * t, b2 = 3 * v * t * t, b3 = t * t * t;
      Pt a1 = tA * b1, a2 = tB * b2;
      Pt rest{p[0] - A[0] * (b0 + b1) - B[0] * (b2 + b3), p[1] - A[1] * (b0 + b1) - B[1] * (b2 + b3)};
      c00 += dot(a1, a1); c01 += dot(a1, a2); c11 += dot(a2, a2); x0 += dot(rest, a1); x1 += dot(rest, a2);
    }
    double det = c00 * c11 - c01 * c01;
    double la = js::truthy(det) ? (x0 * c11 - x1 * c01) / det : 0, lb = js::truthy(det) ? (c00 * x1 - c01 * x0) / det : 0;
    if (!(la > chord * MIN_HANDLE) || !(lb > chord * MIN_HANDLE)) la = lb = chord / 3;
    la = js::min(la, chord); lb = js::min(lb, chord);
    Cubic c{A, A + tA * la, B + tB * lb, B};
    // How far the points are from the curve, after one Newton step each towards their nearest place on it.
    for (size_t i = 0; i < u.size(); i++) {
      double t = u[i];
      Pt d = bez(c, t) - P[i], d1 = bezD(c, t), d2 = bezDD(c, t);
      double den = dot(d1, d1) + dot(d, d2);
      u[i] = den > 1e-12 ? clamp(t - dot(d, d1) / den, 0, 1) : t;
    }
    std::vector<double> errs;
    for (size_t i = 0; i < P.size(); i++) errs.push_back(dist(bez(c, u[i]), P[i]));
    double err = js::maxOf(errs);
    if (!best || err < best->err) best = CubicFit{c, err};
  }
  return *best;
}

std::vector<Cubic> fitCubics(const std::vector<Pt>& points, double tol, FitOptions o) {
  double corner = o.corner;
  std::vector<Pt> P;
  for (Pt q : points) if (P.empty() || dist(q, P.back()) > 1e-9) P.push_back(q);
  if (P.size() < 2) return {};
  std::vector<size_t> cuts{0};
  for (size_t i = 1; i + 1 < P.size(); i++) {
    Pt a = norm(P[i] - P[i - 1]), b = norm(P[i + 1] - P[i]);
    if (std::acos(clamp(dot(a, b), -1, 1)) > corner * PI / 180) cuts.push_back(i);
  }
  cuts.push_back(P.size() - 1);
  std::function<std::vector<Cubic>(const std::vector<Pt>&, Pt, Pt, int)> run =
    [&](const std::vector<Pt>& Q, Pt tA, Pt tB, int depth) -> std::vector<Cubic> {
    CubicFit f = fitCubic(Q, tA, tB);
    if (f.err <= tol || Q.size() <= 3 || depth >= MAX_DEPTH) return {f.c};
    size_t worst = 1;
    double most = -1;
    for (size_t i = 1; i + 1 < Q.size(); i++) {
      Cubic cs[1] = {f.c};
      double d = nearest(cs, Q[i], 16).d;
      if (d > most) { most = d; worst = i; }
    }
    Pt m = norm(Q[worst + 1] - Q[worst - 1]);
    std::vector<Pt> first(Q.begin(), Q.begin() + worst + 1), second(Q.begin() + worst, Q.end());
    std::vector<Cubic> out = run(first, tA, m * -1, depth + 1);
    std::vector<Cubic> rest = run(second, m, tB, depth + 1);
    out.insert(out.end(), rest.begin(), rest.end());
    return out;
  };
  std::vector<Cubic> result;
  for (size_t k = 0; k + 1 < cuts.size(); k++) {
    size_t e = cuts[k + 1];
    std::vector<Pt> Q(P.begin() + cuts[k], P.begin() + e + 1);
    std::vector<Cubic> r = run(Q, norm(Q[1] - Q[0]), norm(Q[Q.size() - 2] - Q[Q.size() - 1]), 0);
    result.insert(result.end(), r.begin(), r.end());
  }
  return result;
}

std::vector<Pt> cubicToQuads(const Cubic& c, double tol) {
  Pt zero{0, 0};
  auto control = [](double t, const Cubic& p) {
    return lerp(p[0] + (p[1] - p[0]) * 1.5, p[3] + (p[2] - p[3]) * 1.5, t);
  };
  for (int n = 1; n <= 16; n++) {
    if (n == 1) {
      auto q = apex(c);
      if (q && fitsInside(zero, lerp(c[0], *q, 2.0 / 3) - c[1], lerp(c[3], *q, 2.0 / 3) - c[2], zero, tol)) return {*q};
      continue;
    }
    std::vector<Cubic> pieces = splitEven(c, n);
    Cubic next = pieces[0];
    Pt nextQ = control(0, next), q2 = c[0], d1 = zero;
    bool ok = true;
    std::vector<Pt> qs{nextQ};
    for (int i = 1; i <= n && ok; i++) {
      Pt c1 = next[1], c2 = next[2], c3 = next[3], q0 = q2, q1 = nextQ;
      if (i < n) {
        next = pieces[i]; nextQ = control(double(i) / (n - 1), next); qs.push_back(nextQ); q2 = (q1 + nextQ) * 0.5;
      } else q2 = c3;
      Pt d0 = d1;
      d1 = q2 - c3;
      ok = len(d1) <= tol && fitsInside(d0, lerp(q0, q1, 2.0 / 3) - c1, lerp(q2, q1, 2.0 / 3) - c2, d1, tol);
    }
    if (ok) return qs;
  }
  // Too twisty for sixteen pieces: one control per equal piece, at its tangents' meeting.
  std::vector<Pt> out;
  for (const Cubic& p : splitEven(c, 16)) {
    auto a = apex(p);
    out.push_back(a ? *a : (p[1] + p[2]) * 0.5);
  }
  return out;
}

std::vector<Pt> hull(const std::vector<Pt>& points, bool sorted) {
  std::vector<Pt> p = points;
  if (!sorted) std::stable_sort(p.begin(), p.end(), [](Pt a, Pt b) {
    double d = a[0] - b[0];
    return (js::truthy(d) ? d : a[1] - b[1]) < 0;
  });
  if (p.size() < 3) return p;
  auto turns = [](Pt o, Pt a, Pt b) { return cross(a - o, b - o); };
  std::vector<Pt> lo, up;
  for (Pt q : p) {
    while (lo.size() > 1 && turns(lo[lo.size() - 2], lo.back(), q) <= 0) lo.pop_back();
    lo.push_back(q);
  }
  for (size_t i = p.size(); i-- > 0;) {
    Pt q = p[i];
    while (up.size() > 1 && turns(up[up.size() - 2], up.back(), q) <= 0) up.pop_back();
    up.push_back(q);
  }
  std::vector<Pt> out(lo.begin(), lo.end() - 1);
  out.insert(out.end(), up.begin(), up.end() - 1);
  return out;
}

std::vector<Pt> mergeHulls(const std::vector<Pt>& A, const std::vector<Pt>& B) {
  std::vector<std::vector<Pt>> chains;
  for (const std::vector<Pt>* Pp : {&A, &B}) {
    const std::vector<Pt>& P = *Pp;
    long n = long(P.size());
    long lo = 0, hi = 0;
    for (long i = 1; i < n; i++) { if (before(P[i], P[lo])) lo = i; if (before(P[hi], P[i])) hi = i; }
    std::vector<Pt> up{P[lo]}, down{P[lo]};
    for (long i = (lo + 1) % n; i != hi; i = (i + 1) % n) up.push_back(P[i]);
    for (long i = (lo - 1 + n) % n; i != hi; i = (i - 1 + n) % n) down.push_back(P[i]);
    if (hi != lo) { up.push_back(P[hi]); down.push_back(P[hi]); }
    chains.push_back(up);
    // down.slice(1, -1)
    chains.push_back(down.size() >= 2 ? std::vector<Pt>(down.begin() + 1, down.end() - 1) : std::vector<Pt>{});
  }
  auto merge = [](const std::vector<Pt>& a, const std::vector<Pt>& b) {
    std::vector<Pt> out;
    size_t i = 0, j = 0;
    while (i < a.size() || j < b.size())
      out.push_back(j >= b.size() || (i < a.size() && !before(b[j], a[i])) ? a[i++] : b[j++]);
    return out;
  };
  std::vector<Pt> all = merge(merge(chains[0], chains[1]), merge(chains[2], chains[3]));
  for (size_t i = 1; i < all.size(); i++) if (before(all[i], all[i - 1])) return hull(all);
  return hull(all, true);
}

std::vector<Pt> clipPolygon(const std::vector<Pt>& poly, const Plane& plane) {
  Pt at = plane.at, n = plane.n;
  auto side = [&](Pt p) { return dot(p - at, n); };
  std::vector<Pt> out;
  for (size_t i = 0; i < poly.size(); i++) {
    Pt a = poly[i], b = poly[(i + 1) % poly.size()];
    double sa = side(a), sb = side(b);
    if (sa >= 0) out.push_back(a);
    if ((sa >= 0) != (sb >= 0)) out.push_back(lerp(a, b, sa / (sa - sb)));
  }
  return out;
}

}  // namespace curves
