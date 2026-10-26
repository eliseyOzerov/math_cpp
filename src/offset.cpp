// See include/curves/offset.hpp. Ported from fountain-ts/src/curves/offset.ts line for line.
#include "curves/offset.hpp"

#include "js.hpp"

#pragma STDC FP_CONTRACT OFF

#include <limits>
#include <set>

namespace curves {

namespace {

/** The largest turn of the normals across a simple piece. */
const double SIMPLE_TURN = PI / 3;
/** reduce's step through each piece. */
const double STEP = 0.01;

/** Where the lines through p along d and through q along e meet, if they aren't parallel. */
std::optional<Pt> lli(Pt p, Pt d, Pt q, Pt e) {
  double den = cross(d, e);
  if (std::abs(den) < 1e-12 * curves::hypot(d.x, d.y) * curves::hypot(e.x, e.y)) return std::nullopt;
  return p + d * (cross(q - p, e) / den);
}

/** Distances along the curve by its length: from `a` at its start to `b` at its end, for a piece spanning
 *  lengths s0…s1 of a total. */
std::function<double(double)> graded(double a, double b, double s0, double s1, double total) {
  return [=](double u) { return a + (b - a) * (s0 + (s1 - s0) * u) / total; };
}

struct Measured {
  Cubic c;
  double t0, t1, s0, s1;
};

/** The reduced pieces with the lengths where each starts and ends along the curve. */
std::vector<Measured> measured(const Cubic& c) {
  auto pieces = reduce(c);
  double s = 0;
  std::vector<Measured> out;
  for (auto& p : pieces) {
    double l = arcLength(p.c);
    out.push_back({p.c, p.t0, p.t1, s, s + l});
    s += l;
  }
  return out;
}

/** The circle through three points, if they aren't on a line. */
struct Circle {
  Pt center;
  double r;
};
std::optional<Circle> circleThrough(Pt a, Pt b, Pt c) {
  double d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
  double scaleOf = js::max(js::max(dist(a, b), dist(b, c)), dist(a, c));
  if (std::abs(d) < 1e-9 * scaleOf * scaleOf) return std::nullopt;
  auto s = [](Pt p) { return p[0] * p[0] + p[1] * p[1]; };
  Pt center{(s(a) * (b[1] - c[1]) + s(b) * (c[1] - a[1]) + s(c) * (a[1] - b[1])) / d,
            (s(a) * (c[0] - b[0]) + s(b) * (a[0] - c[0]) + s(c) * (b[0] - a[0])) / d};
  return Circle{center, dist(center, a)};
}

struct Fit {
  Arc arc;
  double err;
};

/** The arc through c(t0), c(mid) and c(t1), and how far c strays from it between (the largest gap at five places). */
Fit fit(const Cubic& c, double t0, double t1) {
  Pt a = bez(c, t0), m = bez(c, (t0 + t1) / 2), b = bez(c, t1);
  auto circ = circleThrough(a, m, b);
  std::vector<Pt> samples;
  for (double k : {0.125, 0.25, 0.375, 0.625, 0.75, 0.875}) samples.push_back(bez(c, t0 + (t1 - t0) * k));
  if (!circ) {
    Pt ch = b - a;
    double l = js::orElse(curves::hypot(ch.x, ch.y), 1);
    std::vector<double> gaps;
    for (Pt p : samples) gaps.push_back(std::abs(cross(ch, p - a)) / l);
    double err = js::maxOf(gaps);
    return {{std::nullopt, std::numeric_limits<double>::infinity(), 0, 0, true, a, b, t0, t1}, err};
  }
  Pt center = circ->center;
  double r = circ->r;
  auto ang = [&](Pt p) { return std::atan2(p[1] - center[1], p[0] - center[0]); };
  bool ccw = cross(m - a, b - m) > 0;
  std::vector<double> gaps;
  for (Pt p : samples) gaps.push_back(std::abs(dist(p, center) - r));
  double err = js::maxOf(gaps);
  return {{center, r, ang(a), ang(b), ccw, a, b, t0, t1}, err};
}

Cubic reversed(Cubic q) { return {q[3], q[2], q[1], q[0]}; }

}  // namespace

bool simple(const Cubic& c) {
  Pt ch = c[3] - c[0];
  double a1 = cross(ch, c[1] - c[0]), a2 = cross(ch, c[2] - c[0]);
  if ((a1 > 1e-12 && a2 < -1e-12) || (a1 < -1e-12 && a2 > 1e-12)) return false;
  return std::acos(js::max(-1, js::min(1, dot(normal(c, 0), normal(c, 1))))) < SIMPLE_TURN;
}

std::vector<Reduced> reduce(const Cubic& c) {
  if (isLine(c)) return {{c, 0, 1}};
  std::vector<double> all{0};
  for (double t : extrema(c).values) all.push_back(t);
  for (double t : inflections(c)) all.push_back(t);
  all.push_back(1);
  // new Set: the first of equal values stays (NaN equals itself in a Set)
  std::vector<double> cuts;
  for (double t : all) {
    bool seen = false;
    for (double u : cuts) if (u == t || (std::isnan(u) && std::isnan(t))) { seen = true; break; }
    if (!seen && t >= 0 && t <= 1) cuts.push_back(t);
  }
  std::stable_sort(cuts.begin(), cuts.end(), [](double a, double b) { return a < b; });
  std::vector<Reduced> out;
  for (size_t k = 1; k < cuts.size(); k++) {
    double a = cuts[k - 1], b = cuts[k];
    if (b - a < 1e-9) continue;
    double t1 = a;
    while (t1 < b - 1e-9) {
      double t2 = b;
      if (!simple(trim(c, t1, t2))) {
        t2 = t1 + STEP;
        while (t2 + STEP < b && simple(trim(c, t1, t2 + STEP))) t2 += STEP;
        t2 = js::min(t2, b);
      }
      out.push_back({trim(c, t1, t2), t1, t2});
      t1 = t2;
    }
  }
  return out;
}

Cubic scale(const Cubic& c, const Reach& d) {
  const double* dn = std::get_if<double>(&d);
  std::function<double(double)> at = dn ? std::function<double(double)>([v = *dn](double) { return v; }) : std::get<1>(d);
  double r0 = at(0), r1 = at(1);
  Pt n0 = normal(c, 0), n1 = normal(c, 1);
  std::optional<Pt> o = isLine(c) ? std::nullopt : lli(c[0], n0, c[3], n1);
  Pt e0 = c[0] + n0 * r0, e3 = c[3] + n1 * r1;
  if (!o) return {e0, c[1] + n0 * (dn ? *dn : at(1.0 / 3)), c[2] + n1 * (dn ? *dn : at(2.0 / 3)), e3};
  if (dn) {
    auto h1 = lli(e0, tangent(c, 0), *o, c[1] - *o), h2 = lli(e3, tangent(c, 1), *o, c[2] - *o);
    if (h1 && h2) return {e0, *h1, *h2, e3};
    return {e0, c[1] + n0 * *dn, c[2] + n1 * *dn, e3};
  }
  // Graduated: each handle away from o along the line through it, as far as the distance there, outwards when the
  // normal points away from o.
  Pt op = *o;
  auto hand = [&](Pt h, Pt n, double r) -> Pt {
    Pt v = h - op;
    if (curves::hypot(v.x, v.y) < 1e-12) return h + n * r;
    Pt u = norm(v);
    return h + u * (dot(u, n) >= 0 ? r : -r);
  };
  return {e0, hand(c[1], n0, at(1.0 / 3)), hand(c[2], n1, at(2.0 / 3)), e3};
}

std::vector<Cubic> offset(const Cubic& c, const Reach& d) {
  std::vector<Cubic> out;
  for (auto& p : reduce(c)) {
    if (const double* dn = std::get_if<double>(&d)) out.push_back(scale(p.c, *dn));
    else {
      auto f = std::get<1>(d);
      double t0 = p.t0, t1 = p.t1;
      out.push_back(scale(p.c, Reach(std::function<double(double)>([f, t0, t1](double u) { return f(t0 + (t1 - t0) * u); }))));
    }
  }
  return out;
}

Pt offsetAt(const Cubic& c, double t, double d) { return bez(c, t) + normal(c, t) * d; }

Contour outline(const Cubic& c, double d1, std::optional<double> d2o, std::optional<double> d3, std::optional<double> d4) {
  double d2 = d2o.value_or(d1);
  auto ps = measured(c);
  double total = js::orElse(ps.empty() ? std::numeric_limits<double>::quiet_NaN() : ps.back().s1, 1);
  double l3 = d3.value_or(d1), l4 = d4.value_or(d2);
  std::vector<Cubic> left, right;
  for (auto& p : ps) left.push_back(scale(p.c, Reach(graded(d1, l3, p.s0, p.s1, total))));
  for (auto& p : ps) right.push_back(scale(p.c, Reach(graded(-d2, -l4, p.s0, p.s1, total))));
  std::reverse(right.begin(), right.end());
  for (auto& q : right) q = reversed(q);
  Contour out;
  for (auto& q : left) out.push_back(Piece::of(q));
  out.push_back(Piece::of(left.back()[3], right.front()[0]));
  for (auto& q : right) out.push_back(Piece::of(q));
  out.push_back(Piece::of(right.back()[3], left.front()[0]));
  return out;
}

std::vector<Contour> outlineShapes(const Cubic& c, double d1, std::optional<double> d2o, std::optional<double> d3,
                                   std::optional<double> d4) {
  double d2 = d2o.value_or(d1);
  auto ps = measured(c);
  double total = js::orElse(ps.empty() ? std::numeric_limits<double>::quiet_NaN() : ps.back().s1, 1);
  double l3 = d3.value_or(d1), l4 = d4.value_or(d2);
  std::vector<Contour> out;
  for (auto& p : ps) {
    Cubic f = scale(p.c, Reach(graded(d1, l3, p.s0, p.s1, total)));
    Cubic b = reversed(scale(p.c, Reach(graded(-d2, -l4, p.s0, p.s1, total))));
    out.push_back({Piece::of(f), Piece::of(f[3], b[0]), Piece::of(b), Piece::of(b[3], f[0])});
  }
  return out;
}

std::vector<Arc> arcs(const Cubic& c, double threshold) {
  Box size = bbox(c);
  double floor = 1e-4 * js::max(1e-12, dist(size[0], size[1]));
  std::vector<Arc> out;
  double t0 = 0;
  while (t0 < 1 - 1e-9 && out.size() < 1000) {
    Fit good = fit(c, t0, 1);
    if (good.err > threshold) {
      double lo = t0, hi = 1;
      good = fit(c, t0, js::min(1, t0 + 1e-3));
      for (int k = 0; k < 30 && hi - lo > 1e-6; k++) {
        double m = (lo + hi) / 2;
        Fit f = fit(c, t0, m);
        if (f.err <= js::max(threshold, floor)) { lo = m; good = f; } else hi = m;
      }
    }
    out.push_back(good.arc);
    t0 = good.arc.t1;
  }
  return out;
}

}  // namespace curves
