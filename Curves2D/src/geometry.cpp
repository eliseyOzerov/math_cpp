// See include/curves/geometry.hpp. Ported from fountain-ts/src/curves/geometry.ts line for line.
#include "curves/geometry.hpp"

// No fused multiply-adds: JS rounds every operation, and the golden values (and near-double roots) depend on it.
#pragma STDC FP_CONTRACT OFF

#include <stdexcept>

namespace curves {

Pt mean(std::span<const Pt> ps) {
  double sx = 0, sy = 0;
  for (Pt p : ps) { sx += p.x; sy += p.y; }
  return {sx / double(ps.size()), sy / double(ps.size())};
}

double halfTurn(double a) {
  while (a > PI / 2) a -= PI;
  while (a <= -PI / 2) a += PI;
  return a;
}

Pt norm(Pt v) {
  double l = curves::hypot(v.x, v.y);
  return l > 1e-12 ? Pt{v.x / l, v.y / l} : Pt{0, 0};
}

Pt bez(const Cubic& c, double t) {
  double u = 1 - t, a = u * u * u, b = 3 * u * u * t, d = 3 * u * t * t, e = t * t * t;
  return {a * c[0].x + b * c[1].x + d * c[2].x + e * c[3].x, a * c[0].y + b * c[1].y + d * c[2].y + e * c[3].y};
}

Pt bezD(const Cubic& c, double t) {
  double u = 1 - t;
  Pt out;
  for (int i = 0; i < 2; i++)
    out[i] = 3 * u * u * (c[1][i] - c[0][i]) + 6 * u * t * (c[2][i] - c[1][i]) + 3 * t * t * (c[3][i] - c[2][i]);
  return out;
}

Pt bezDD(const Cubic& c, double t) {
  double u = 1 - t;
  Pt out;
  for (int i = 0; i < 2; i++) out[i] = 6 * u * (c[2][i] - 2 * c[1][i] + c[0][i]) + 6 * t * (c[3][i] - 2 * c[2][i] + c[1][i]);
  return out;
}

double curvature(const Cubic& c, double t) {
  Pt d = bezD(c, t);
  double l = curves::hypot(d.x, d.y);
  return l > 1e-12 ? cross(d, bezDD(c, t)) / std::pow(l, 3) : 0;
}

bool nearSegment(Pt p, Pt a, Pt b, double tol, std::optional<double> over) {
  double o = over.value_or(tol);
  Pt d = b - a;
  double l = len(d);
  return l > 1e-9 && std::abs(cross(p - a, d)) / l <= tol && dot(p - a, d) >= -o * l && dot(p - b, d) <= o * l;
}

std::optional<std::array<double, 2>> meet(Pt p, Pt d, Pt q, Pt e, double flat, std::optional<double> inside) {
  double den = cross(d, e);
  if (std::abs(den) < flat) return std::nullopt;
  Pt w = q - p;
  double s = cross(w, e) / den, r = cross(w, d) / den;
  if (inside) {
    double in = *inside;
    if (!(s > in && s < 1 - in && r > in && r < 1 - in)) return std::nullopt;
  }
  return std::array<double, 2>{s, r};
}

bool isLine(const Cubic& c) {
  Pt d = c[3] - c[0];
  double l2 = dot(d, d);
  if (!(l2 > 1e-18)) return false;
  for (Pt h : {c[1], c[2]})
    if (!(std::abs(cross(h - c[0], d)) <= 1e-9 * l2)) return false;
  return true;
}

Piece reverse(const Piece& q) {
  return q.line ? Piece::of(q.p[1], q.p[0]) : Piece::of(reverse(q.p));
}

std::array<Pt, 2> endTangents(const Piece& q) {
  if (q.line) { Pt d = norm(q.p[1] - q.p[0]); return {d, d}; }
  auto pick = [](std::initializer_list<Pt> vs) {
    for (Pt v : vs) { Pt n = norm(v); if (n.x != 0 || n.y != 0) return n; }
    return Pt{0, 0};
  };
  const auto& c = q.p;
  return {pick({c[1] - c[0], c[2] - c[0], c[3] - c[0]}), pick({c[3] - c[2], c[3] - c[1], c[3] - c[0]})};
}

std::array<Cubic, 2> split(const Cubic& c, double t) {
  Pt ab = lerp(c[0], c[1], t), bc = lerp(c[1], c[2], t), cd = lerp(c[2], c[3], t);
  Pt abc = lerp(ab, bc, t), bcd = lerp(bc, cd, t), m = lerp(abc, bcd, t);
  return {Cubic{c[0], ab, abc, m}, Cubic{m, bcd, cd, c[3]}};
}

Cubic trim(const Cubic& c, double t0, double t1) {
  Cubic head = t1 < 1 ? split(c, t1)[0] : c;
  return t0 > 0 ? split(head, t0 / t1)[1] : head;
}

std::vector<double> roots(const Cubic& c, int coord, double v, int steps) {
  auto f = [&](double t) { return bez(c, t)[coord] - v; };
  std::vector<double> out;
  double t0 = 0, f0 = f(0);
  if (std::abs(f0) < 1e-9) out.push_back(0);
  for (int i = 1; i <= steps; i++) {
    double t1 = double(i) / steps, f1 = f(t1);
    if (std::abs(f1) < 1e-9) out.push_back(t1);
    else if (f0 * f1 < 0 && std::abs(f0) >= 1e-9) {
      double a = t0, b = t1, fa = f0;
      for (int k = 0; k < 60; k++) {
        double m = (a + b) / 2, fm = f(m);
        if (fa * fm <= 0) b = m; else { a = m; fa = fm; }
      }
      out.push_back((a + b) / 2);
    }
    t0 = t1; f0 = f1;
  }
  return out;
}

Nearest nearest(std::span<const Cubic> segs, Pt p, int steps) {
  Nearest best{0, 0, segs[0][0], INFINITY};
  for (int i = 0; i < int(segs.size()); i++)
    for (int k = 0; k <= steps; k++) {
      double t = double(k) / steps;
      Pt q = bez(segs[i], t);
      double d = dist(q, p);
      if (d < best.d) best = {i, t, q, d};
    }
  const Cubic& c = segs[best.i];
  double t = best.t;
  // f(t) = (B(t) − p)·B'(t), zero at the nearest point; its derivative is taken numerically.
  auto f = [&](double u) { return dot(bez(c, u) - p, bezD(c, u)); };
  for (int k = 0; k < 8; k++) {
    double h = 1e-6, df = (f(t + h) - f(t - h)) / (2 * h);
    if (std::abs(df) < 1e-12) break;
    double u = std::min(1.0, std::max(0.0, t - f(t) / df));
    if (dist(bez(c, u), p) > dist(bez(c, t), p) + 1e-12) break;
    t = u;
  }
  Pt q = bez(c, t);
  double d = dist(q, p);
  return d <= best.d ? Nearest{best.i, t, q, d} : best;
}

// 24-point Legendre–Gauss abscissae and weights on [−1, 1], symmetric pairs.
static constexpr double X[] = {0.0640568928626056, 0.1911188674736163, 0.3150426796961634, 0.4337935076260451,
                               0.5454214713888396, 0.6480936519369755, 0.7401241915785544, 0.8200019859739029,
                               0.8864155270044011, 0.9382745520027328, 0.9747285559713095, 0.9951872199970213};
static constexpr double W[] = {0.1279381953467522, 0.1258374563468283, 0.1216704729278034, 0.1155056680537256,
                               0.1074442701159656, 0.0976186521041139, 0.0861901615319533, 0.0733464814110803,
                               0.0592985849154368, 0.0442774388174198, 0.0285313886289337, 0.0123412297999872};

double arcLength(const Cubic& c, double t) {
  double h = t / 2, s = 0;
  auto speed = [&](double u) { Pt d = bezD(c, u); return curves::hypot(d.x, d.y); };
  for (int i = 0; i < 12; i++) s += W[i] * (speed(h * (1 - X[i])) + speed(h * (1 + X[i])));
  return h * s;
}

double tAtLength(const Cubic& c, double s, std::optional<double> total) {
  double tot = total ? *total : arcLength(c);
  if (s <= 0) return 0;
  if (s >= tot) return 1;
  double lo = 0, hi = 1;
  for (int k = 0; k < 50; k++) { double m = (lo + hi) / 2; if (arcLength(c, m) < s) lo = m; else hi = m; }
  return (lo + hi) / 2;
}

double pieceLength(const Piece& q) { return q.line ? dist(q.p[0], q.p[1]) : arcLength(q.p); }

Box box(std::span<const Piece> qs, int samples) {
  std::vector<Pt> ps;
  for (const Piece& q : qs) {
    if (q.line) { ps.push_back(q.p[0]); ps.push_back(q.p[1]); }
    else for (int i = 0; i <= samples; i++) ps.push_back(bez(q.p, double(i) / samples));
  }
  return bounds(ps);
}

Contour& stitch(Contour& c) {
  size_t n = c.size();
  for (size_t i = 0; i < n; i++) c[i].p[0] = c[(i + n - 1) % n].last();
  return c;
}

Cubic arcCubic(Pt a, Pt ta, Pt b, Pt tb) {
  Pt u = norm(ta), v = norm(tb);
  double th = std::abs(turn(u, v)), chord = dist(a, b);
  // A circle's arc turning θ has handles (4/3)·tan(θ/4)·r and a chord 2r·sin(θ/2).
  double k = th < 1e-6 ? chord / 3 : chord * (4.0 / 3) * std::tan(th / 4) / (2 * std::sin(th / 2));
  return {a, a + u * k, b - v * k, b};
}

std::vector<Cubic> arcCubics(Pt a, Pt b, double al) {
  Pt u = norm(b - a);
  double chord = dist(a, b);
  double R = chord / (2 * std::sin(std::abs(al)));
  int n = std::max(1, int(std::ceil(std::abs(2 * al) / (PI / 2) - 1e-9)));
  double beta = -2 * al / n, h = (4.0 / 3) * std::tan(std::abs(beta) / 4) * R, step = 2 * R * std::sin(std::abs(beta) / 2);
  std::vector<Cubic> out;
  Pt p = a, t = rotate(u, al);
  for (int i = 0; i < n; i++) {
    Pt c = rotate(t, beta / 2), t1 = rotate(t, beta);
    Pt q = i == n - 1 ? b : p + c * step;
    out.push_back({p, p + t * h, q - t1 * h, q});
    p = q; t = t1;
  }
  return out;
}

std::vector<Cubic> cutAt(const Cubic& c, std::span<const double> shares, Pt metric) {
  Cubic ci{toMetric(c[0], metric), toMetric(c[1], metric), toMetric(c[2], metric), toMetric(c[3], metric)};
  double total = arcLength(ci);
  std::vector<double> ts{0};
  for (double s : shares) ts.push_back(tAtLength(ci, s * total, total));
  ts.push_back(1);
  std::vector<Cubic> out;
  for (size_t i = 1; i < ts.size(); i++) out.push_back(trim(c, ts[i - 1], ts[i]));
  return out;
}

std::vector<Cubic> splitEven(const Cubic& c, int n) {
  std::vector<Cubic> out;
  Cubic rest = c;
  for (int i = n; i > 1; i--) { auto [h, t] = split(rest, 1.0 / i); out.push_back(h); rest = t; }
  out.push_back(rest);
  return out;
}

std::vector<double> runLengths(std::span<const Pt> ps) {
  std::vector<double> out{0};
  for (size_t i = 1; i < ps.size(); i++) out.push_back(out[i - 1] + dist(ps[i], ps[i - 1]));
  return out;
}

Pt pointAlong(std::span<const Pt> ps, std::span<const double> run, double s) {
  s = clamp(s, 0, run.back());
  int found = -1;
  for (int i = 0; i < int(run.size()); i++) if (run[i] >= s) { found = i; break; }
  int k = std::max(1, found);
  return lerp(ps[k - 1], ps[k], run[k] > run[k - 1] ? (s - run[k - 1]) / (run[k] - run[k - 1]) : 1);
}

double area(std::span<const Pt> ps) {
  double s = 0;
  for (size_t k = 0; k < ps.size(); k++) s += cross(ps[k], ps[(k + 1) % ps.size()]);
  return s / 2;
}

Box bounds(std::span<const Pt> ps) {
  if (ps.empty()) throw std::invalid_argument("nothing is selected");
  // Math.min and Math.max: NaN anywhere makes that coordinate NaN.
  auto lo = [](double a, double b) { return std::isnan(a) || std::isnan(b) ? NAN : std::min(a, b); };
  auto hi = [](double a, double b) { return std::isnan(a) || std::isnan(b) ? NAN : std::max(a, b); };
  Box b{ps[0], ps[0]};
  for (Pt p : ps) {
    b[0].x = lo(b[0].x, p.x); b[0].y = lo(b[0].y, p.y);
    b[1].x = hi(b[1].x, p.x); b[1].y = hi(b[1].y, p.y);
  }
  return b;
}

Affine then(const Affine& m, const Affine& n) {
  return {
      n[0] * m[0] + n[2] * m[1], n[1] * m[0] + n[3] * m[1],
      n[0] * m[2] + n[2] * m[3], n[1] * m[2] + n[3] * m[3],
      n[0] * m[4] + n[2] * m[5] + n[4], n[1] * m[4] + n[3] * m[5] + n[5],
  };
}

/** The linear map `m` about `pivot` instead of the origin. */
static Affine about(const Affine& m, Pt pivot) {
  return then(then({1, 0, 0, 1, -pivot.x, -pivot.y}, m), {1, 0, 0, 1, pivot.x, pivot.y});
}

Affine rotation(double d, Pt pivot) {
  double c = std::cos(rad(d)), s = std::sin(rad(d));
  return about({c, s, -s, c, 0, 0}, pivot);
}

Affine shear(double d, double axis, Pt pivot) {
  Pt u{std::cos(rad(axis)), std::sin(rad(axis))}, v{-u.y, u.x};
  double k = -std::tan(rad(d));
  // p + k·(p·v)·u: the identity plus k·u·vᵀ.
  return about({1 + k * u.x * v.x, k * u.y * v.x, k * u.x * v.y, 1 + k * u.y * v.y, 0, 0}, pivot);
}

Affine inMetric(const Affine& m, Pt metric) {
  double sx = metric.x, sy = metric.y;
  return {m[0], m[1] * sx / sy, m[2] * sy / sx, m[3], m[4] / sx, m[5] / sy};
}

Affine scaling(double kx, double ky, Pt pivot) { return about({kx, 0, 0, ky, 0, 0}, pivot); }

Affine partly(const Affine& m, double w) {
  Affine out;
  for (int i = 0; i < 6; i++) out[i] = identity[i] + (m[i] - identity[i]) * w;
  return out;
}

Affine shift(double d, double axis) { return {1, 0, 0, 1, d * std::cos(rad(axis)), d * std::sin(rad(axis))}; }

}  // namespace curves
