// See include/curves/nib.hpp. Ported from fountain-ts/src/ink/nib.ts line for line.
#include "curves/nib.hpp"

#include "js.hpp"
#include "jsmath.hpp"

// No fused multiply-adds: JS rounds every operation, and the golden values depend on it.
#pragma STDC FP_CONTRACT OFF

#include <cmath>

namespace curves {

const double CORNER = rad(20);

const Nib NIB{};

Nib nibOf(const NibInput& nib) {
  if (std::holds_alternative<std::monostate>(nib)) return NIB;
  if (auto n = std::get_if<double>(&nib)) { Nib r = NIB; r.w = *n; r.h = *n; return r; }
  if (auto p = std::get_if<StemPen>(&nib)) return stemNib(*p);
  return std::get<Nib>(nib);
}

Nib stemNib(const StemPen& pen) {
  double c = clamp(pen.contrast, 0, 0.99), th = rad(pen.stress);
  double cs = js::cos(th), sn = js::sin(th), k = 1 - c;
  double w = pen.weight / std::sqrt(cs * cs + k * k * (sn * sn));
  Nib r = NIB;
  r.w = w; r.h = w * (1 - c); r.angle = pen.stress;
  return r;
}

StemPen stemPenOf(const NibInput& nib) {
  if (auto p = std::get_if<StemPen>(&nib)) return *p;
  Nib n = nibOf(nib);
  return {js::round(stemWidth(n) * 100) / 100, js::round((1 - n.h / n.w) * 100) / 100, n.angle};
}

// The nib's shape and its stamp.

const std::array<Pt, 4> CORNERS = {Pt{1, 1}, Pt{-1, 1}, Pt{-1, -1}, Pt{1, -1}};

std::array<double, 4> radii(const Radius& v) {
  if (auto a = std::get_if<std::vector<double>>(&v)) {
    std::array<double, 4> r{};
    for (size_t i = 0; i < 4; i++) r[i] = i < a->size() ? (*a)[i] : a->empty() ? double(NAN) : (*a)[0];
    return r;
  }
  double x = std::get<double>(v);
  return {x, x, x, x};
}

Radius uniform(const Radius& v) {
  auto c = radii(v);
  bool same = true;
  for (double x : c) if (!(std::abs(x - c[0]) < 1e-9)) same = false;
  if (same) return c[0];
  return std::vector<double>(c.begin(), c.end());
}

Shape mixShape(const Shape& a, const Shape& b, double u) {
  auto lerpR = [u](const Radius& p, const Radius& q) -> Radius {
    if (std::holds_alternative<std::vector<double>>(p) || std::holds_alternative<std::vector<double>>(q)) {
      auto P = radii(p), Q = radii(q);
      std::vector<double> m(4);
      for (size_t i = 0; i < 4; i++) m[i] = P[i] + (Q[i] - P[i]) * u;
      return uniform(m);
    }
    double x = std::get<double>(p), y = std::get<double>(q);
    return x + (y - x) * u;
  };
  return {a.w + (b.w - a.w) * u, a.h + (b.h - a.h) * u, lerpR(a.rx, b.rx), lerpR(a.ry, b.ry)};
}

std::array<CornerArc, 4> cornerArcs(const Shape& shape) {
  double a = shape.w / 2, b = shape.h / 2;
  auto RX = radii(shape.rx), RY = radii(shape.ry);
  std::array<CornerArc, 4> out;
  for (size_t k = 0; k < 4; k++) {
    double sx = CORNERS[k][0], sy = CORNERS[k][1];
    double ca = clamp(RX[k], 0, 1) * a, cb = clamp(RY[k], 0, 1) * b;
    out[k] = {Pt{sx, sy}, Pt{sx * (a - ca), sy * (b - cb)}, Pt{ca, cb}};
  }
  return out;
}

std::vector<Pt> stampPolygon(const Shape& shape, double rot, double perCorner) {
  double a = shape.w / 2, b = shape.h / 2;
  std::vector<Pt> out;
  auto arcs = cornerArcs(shape);
  for (size_t k = 0; k < 4; k++) {
    const CornerArc& arc = arcs[k];
    double sx = arc.s[0], sy = arc.s[1], ca = arc.r[0], cb = arc.r[1];
    if (ca < 1e-6 || cb < 1e-6) { out.push_back({sx * a, sy * b}); continue; }
    for (double i = 0; i <= perCorner; i++) {
      double th = rad(90 * double(k) + 90 * i / perCorner);
      out.push_back({arc.o[0] + ca * js::cos(th), arc.o[1] + cb * js::sin(th)});
    }
  }
  for (Pt& p : out) p = js::rotate(p, rot);
  return out;
}

double nibWidth(const NibInput& nib, double angle, double scale) {
  Nib N = nibOf(nib);
  double rot = rad(N.angle) + clamp(N.follow, 0, 1) * (halfTurn(angle) - PI / 2);
  Pt n = {-js::sin(angle), js::cos(angle)};
  auto poly = stampPolygon({N.w * scale, N.h * scale, N.rx, N.ry}, rot, N.perCorner);
  std::vector<double> d;
  for (Pt p : poly) d.push_back(dot(p, n));
  return js::maxOf(d) - js::minOf(d);
}

double stemWidth(const NibInput& nib, double scale) { return nibWidth(nib, PI / 2, scale); }

std::optional<Keys<Shape>> shapesOf(const std::optional<ShapesInput>& v, const Nib& nib) {
  if (!v) return std::nullopt;
  if (auto k = std::get_if<Keys<Shape>>(&*v)) return *k;
  const auto& ps = std::get<std::vector<PartialShape>>(*v);
  if (ps.empty()) return Keys<Shape>{};
  std::vector<Shape> ks;
  for (const PartialShape& p : ps)
    ks.push_back({p.w.value_or(1), p.h.value_or(1), p.rx.value_or(nib.rx), p.ry.value_or(nib.ry)});
  return spread(ks);
}

// Weight compensation.

namespace {

/** The lean off the nearest axis, in degrees, from which a line counts fully as diagonal: below it the factor eases
 *  in, so a nearly upright or level stroke isn't thinned. */
constexpr double DIAGONAL = 15;

/** How far a line leaning `d` degrees off the nearest axis counts as diagonal: 0 upright or level, 1 from DIAGONAL,
 *  eased. */
double diagonalness(double d) {
  double m = js::mod(js::mod(d, 90) + 90, 90), u = js::min(1, js::min(m, 90 - m) / DIAGONAL);
  return smoothstep(u);
}

}  // namespace

double segmentWeight(const Cubic& c, const Compensation& comp) {
  if (!isLine(c)) return comp.round;
  return 1 + (comp.diagonal - 1) * diagonalness(deg(js::atan2(c[3][1] - c[0][1], c[3][0] - c[0][0])));
}

std::optional<Keys<double>> weightCurve(const std::vector<Cubic>& segs, bool closed, const Compensation& comp, double ramp) {
  std::vector<double> f;
  for (const Cubic& c : segs) f.push_back(segmentWeight(c, comp));
  bool flat = true;
  for (double v : f) if (!(std::abs(v - 1) < 1e-12)) flat = false;
  if (flat) return std::nullopt;
  std::vector<double> L;
  for (const Cubic& c : segs) L.push_back(arcLength(c));
  double sum = L[0];
  for (size_t i = 1; i < L.size(); i++) sum += L[i];
  double T = js::orElse(sum, 1);
  size_t n = segs.size();
  Keys<double> out;
  auto half = [&](size_t i) { return js::min(ramp / 2, L[i] / 2); };
  auto push = [&](const Keys<double>& ks) { out.insert(out.end(), ks.begin(), ks.end()); };
  bool wraps = closed && std::abs(f[n - 1] - f[0]) > 1e-12;
  if (wraps) { double r = js::min(half(n - 1), half(0)); push(easeKeys(0, (f[n - 1] + f[0]) / 2, r / T, f[0])); }
  else out.push_back({0, f[0]});
  double s = 0;
  for (size_t i = 0; i + 1 < n; i++) {
    s += L[i];
    if (std::abs(f[i] - f[i + 1]) < 1e-12) continue;
    double r = js::min(half(i), half(i + 1));
    push(easeKeys((s - r) / T, f[i], (s + r) / T, f[i + 1]));
  }
  if (wraps) { double r = js::min(half(n - 1), half(0)); push(easeKeys(1 - r / T, f[n - 1], 1, (f[n - 1] + f[0]) / 2)); }
  else out.push_back({1, f[n - 1]});
  return out;
}

}  // namespace curves
