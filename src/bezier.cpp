// See include/curves/bezier.hpp. Ported from fountain-ts/src/curves/bezier.ts line for line.
#include "curves/bezier.hpp"

// No fused multiply-adds: JS rounds every operation, and the golden values (and near-double roots) depend on it.
#pragma STDC FP_CONTRACT OFF

namespace curves {

std::vector<double> cubicRoots(double a, double b, double c, double d) {
  double big = std::max({std::abs(a), std::abs(b), std::abs(c), std::abs(d)}), tiny = 1e-12 * (big != 0 ? big : 1);
  std::vector<double> out;
  if (std::abs(a) < tiny) {
    if (std::abs(b) < tiny) { if (!(std::abs(c) < tiny)) out = {-d / c}; }
    else {
      double D = c * c - 4 * b * d;
      if (!(D < -tiny * tiny)) { double s = std::sqrt(std::max(D, 0.0)); out = {(-c + s) / (2 * b), (-c - s) / (2 * b)}; }
    }
  } else {
    double A = b / a, B = c / a, C = d / a, off = -A / 3;
    double p = B - A * A / 3, q = 2 * A * A * A / 27 - A * B / 3 + C, disc = q * q / 4 + p * p * p / 27;
    if (std::abs(disc) < 1e-14) { double u = std::cbrt(-q / 2); out = {2 * u + off, -u + off}; }
    else if (disc > 0) { double s = std::sqrt(disc); out = {std::cbrt(-q / 2 + s) + std::cbrt(-q / 2 - s) + off}; }
    else {
      double r = std::sqrt(-p * p * p / 27), phi = std::acos(clamp(-q / (2 * r), -1, 1)), m = 2 * std::cbrt(r);
      for (int k = 0; k < 3; k++) out.push_back(m * std::cos((phi + 2 * PI * k) / 3) + off);
    }
  }
  auto f = [&](double t) { return ((a * t + b) * t + c) * t + d; };
  auto df = [&](double t) { return (3 * a * t + 2 * b) * t + c; };
  for (double& t : out)
    for (int k = 0; k < 2; k++) { double s = df(t); if (std::abs(s) > 1e-14) t -= f(t) / s; }
  return out;
}

std::vector<double> inUnit(std::span<const double> ts, double eps) {
  std::vector<double> in, out;
  for (double t : ts) if (t >= -eps && t <= 1 + eps) in.push_back(clamp(t, 0, 1));
  std::sort(in.begin(), in.end());
  for (double t : in) if (out.empty() || t - out.back() > eps) out.push_back(t);
  return out;
}

std::vector<double> coordRoots(double p0, double p1, double p2, double p3, double v) {
  double q0 = p0 - v, q1 = p1 - v, q2 = p2 - v, q3 = p3 - v;
  return inUnit(cubicRoots(-q0 + 3 * q1 - 3 * q2 + q3, 3 * q0 - 6 * q1 + 3 * q2, -3 * q0 + 3 * q1, q0));
}

Extrema extrema(const Cubic& c) {
  auto one = [&](int k) {
    double d0 = 3 * (c[1][k] - c[0][k]), d1 = 3 * (c[2][k] - c[1][k]), d2 = 3 * (c[3][k] - c[2][k]);
    std::vector<double> out;
    for (double t : inUnit(cubicRoots(0, d0 - 2 * d1 + d2, 2 * (d1 - d0), d0)))
      if (t > 1e-9 && t < 1 - 1e-9) out.push_back(t);
    return out;
  };
  Extrema e{one(0), one(1), {}};
  std::vector<double> all = e.x;
  all.insert(all.end(), e.y.begin(), e.y.end());
  e.values = inUnit(all);
  return e;
}

Box bbox(const Cubic& c) {
  std::vector<Pt> ps{c[0], c[3]};
  for (double t : extrema(c).values) ps.push_back(bez(c, t));
  return bounds(ps);
}

Cubic align(const Cubic& c, Pt a, Pt b) {
  double ang = std::atan2(b.y - a.y, b.x - a.x), cs = std::cos(-ang), sn = std::sin(-ang);
  Cubic out;
  for (int i = 0; i < 4; i++) { Pt d = c[i] - a; out[i] = {d.x * cs - d.y * sn, d.x * sn + d.y * cs}; }
  return out;
}

std::vector<double> inflections(const Cubic& c) {
  Cubic p = align(c);
  double a = p[2].x * p[1].y, b = p[3].x * p[1].y, cc = p[1].x * p[2].y, d = p[3].x * p[2].y;
  double x = 18 * (-3 * a + 2 * b + 3 * cc - d), y = 18 * (3 * a - b - 3 * cc), z = 18 * (cc - a);
  std::vector<double> out;
  for (double t : inUnit(cubicRoots(0, x, y, z))) if (t > 1e-9 && t < 1 - 1e-9) out.push_back(t);
  return out;
}

std::array<Pt, 4> tightBox(const Cubic& c) {
  Box b = bbox(align(c));
  Pt lo = b[0], hi = b[1];
  double ang = std::atan2(c[3].y - c[0].y, c[3].x - c[0].x), cs = std::cos(ang), sn = std::sin(ang);
  auto back = [&](Pt p) { return c[0] + Pt{p.x * cs - p.y * sn, p.x * sn + p.y * cs}; };
  return {back(lo), back({hi.x, lo.y}), back(hi), back({lo.x, hi.y})};
}

std::string_view name(CubicKind k) {
  switch (k) {
    case CubicKind::line: return "line";
    case CubicKind::arch: return "arch";
    case CubicKind::singleInflection: return "single inflection";
    case CubicKind::doubleInflection: return "double inflection";
    case CubicKind::loop: return "loop";
    case CubicKind::cusp: return "cusp";
  }
  return "";
}

static double dot2(Pt v) { return v.x * v.x + v.y * v.y; }
/** JS's `x || 1`: 1 where x is 0 (or NaN). */
static double orOne(double x) { return x != 0 && !std::isnan(x) ? x : 1; }

Canonical canonical(const Cubic& c, double eps) {
  Pt u = c[1] - c[0], v = c[2] - c[0], w = c[3] - c[0];
  double det = cross(u, v);
  if (std::abs(det) < eps * orOne(dot2(u) + dot2(v))) {
    bool straight = std::abs(cross(c[3] - c[0], c[2] - c[1])) < eps * orOne(dot2(w)) && std::abs(cross(w, u)) < eps * orOne(dot2(w));
    return {straight ? CubicKind::line : CubicKind::arch, {NAN, NAN}};
  }
  // Solve w = s·u + r·v, then (0, 1)·s + (1, 1)·r is where the map takes the fourth point.
  double s = cross(w, v) / det, r = cross(u, w) / det, x = r, y = s + r;
  double cusp = (-x * x + 2 * x + 3) / 4;
  if (std::abs(y - cusp) < eps && x <= 1) return {CubicKind::cusp, {x, y}};
  if (y > 1) return {CubicKind::singleInflection, {x, y}};
  if (x > 1) return {CubicKind::arch, {x, y}};
  if (y > cusp) return {CubicKind::doubleInflection, {x, y}};
  double edge = x <= 0 ? (-x * x + 3 * x) / 3 : (std::sqrt(3 * (4 * x - x * x)) - x) / 2;
  return {y >= edge ? CubicKind::loop : CubicKind::arch, {x, y}};
}

std::array<Pt, 10> casteljau(const Cubic& c, double t) {
  Pt a = lerp(c[0], c[1], t), b = lerp(c[1], c[2], t), d = lerp(c[2], c[3], t), e = lerp(a, b, t), f = lerp(b, d, t);
  return {c[0], c[1], c[2], c[3], a, b, d, e, f, lerp(e, f, t)};
}

std::vector<Pt> lut(const Cubic& c, int steps) {
  std::vector<Pt> out;
  for (int i = 0; i <= steps; i++) out.push_back(bez(c, double(i) / steps));
  return out;
}

Pt tangent(const Cubic& c, double t) {
  Pt d = bezD(c, t);
  if (curves::hypot(d.x, d.y) > 1e-12) return norm(d);
  Pt e = t < 0.5 ? c[2] - c[0] : c[3] - c[1];
  return curves::hypot(e.x, e.y) > 1e-12 ? norm(e) : norm(c[3] - c[0]);
}

Osculating osculating(const Cubic& c, double t) {
  double k = curvature(c, t);
  if (std::abs(k) < 1e-12) return {std::nullopt, INFINITY, 0};
  return {bez(c, t) + normal(c, t) * (1 / k), std::abs(1 / k), k};
}

Projection project(const Cubic& c, Pt p) {
  Nearest n = nearest(std::span<const Cubic>(&c, 1), p, 64);
  return {n.t, n.p, n.d};
}

namespace {
/** Where a curve's point at t sits against the de Casteljau point A above it and the point C on its chord, by the
 *  projection identity (the Primer's u(t) and ratio(t)), for a quadratic (n = 2) or a cubic (n = 3). */
struct ABC {
  Pt A, C;
};
ABC abc(int n, Pt S, Pt B, Pt E, double t) {
  double tn = std::pow(t, n), un = std::pow(1 - t, n), u = un / (tn + un), ratio = std::abs((tn + un - 1) / (tn + un));
  Pt C = lerp(E, S, u);
  return {B + (B - C) * (1 / ratio), C};
}

/** The share along the curve a point B sits at, guessed from its distances to the ends (as the Primer does). */
double share(Pt S, Pt B, Pt E) {
  double a = dist(S, B), b = dist(B, E);
  return a + b > 0 ? a / (a + b) : 0.5;
}

/** The cubic from S to E whose de Casteljau points at t are A (on the middle of the hull) and e1, e2 (either side
 *  of the curve's point): the construction run backwards. */
Cubic fromStruts(Pt S, Pt E, double t, Pt A, Pt e1, Pt e2) {
  Pt v1 = A + (e1 - A) * (1 / (1 - t)), v2 = A + (e2 - A) * (1 / t);
  return {S, S + (v1 - S) * (1 / t), E + (v2 - E) * (1 / (1 - t)), E};
}
}  // namespace

Cubic quadThrough(Pt S, Pt B, Pt E, std::optional<double> t) {
  Pt A = abc(2, S, B, E, clamp(t.value_or(share(S, B, E)), 1e-3, 1 - 1e-3)).A;
  return {S, lerp(S, A, 2.0 / 3), lerp(E, A, 2.0 / 3), E};
}

Cubic cubicThrough(Pt S, Pt B, Pt E, const ThroughOptions& o) {
  double t = clamp(o.t.value_or(share(S, B, E)), 1e-3, 1 - 1e-3);
  Pt A = abc(3, S, B, E, t).A;
  double l = o.d1.value_or(t * (dist(S, B) + dist(B, E)) / 4);
  Pt u = norm(o.dir.value_or(E - S));
  return fromStruts(S, E, t, A, B - u * l, B + u * (l * (1 - t) / t));
}

Cubic mold(const Cubic& c, double t, Pt to) {
  t = clamp(t, 1e-3, 1 - 1e-3);
  auto h = casteljau(c, t);
  Pt B = h[9], d = to - B, C = abc(3, c[0], B, c[3], t).C;
  double t3 = std::pow(t, 3), u3 = std::pow(1 - t, 3);
  Pt A = to + (to - C) * (1 / std::abs((t3 + u3 - 1) / (t3 + u3)));
  return fromStruts(c[0], c[3], t, A, h[7] + d, h[8] + d);
}

}  // namespace curves
