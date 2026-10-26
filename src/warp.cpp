// Ported from fountain-ts/src/curves/warp.ts (see include/curves/warp.hpp).
#include "curves/warp.hpp"

#include "js.hpp"

#pragma STDC FP_CONTRACT OFF

#include <cmath>
#include <cstdint>
#include <limits>

namespace curves {

namespace {

/** JavaScript's ToUint32 of a number. */
uint32_t toUint32(double x) {
  if (!std::isfinite(x)) return 0;
  double m = std::fmod(std::trunc(x), 4294967296.0);
  if (m < 0) m += 4294967296.0;
  return uint32_t(m);
}

/** `X & 255` for a floored double. */
int low8(double x) {
  if (!std::isfinite(x)) return 0;
  return int(toUint32(x) & 255u);
}

/** One cubic warped as one: its ends mapped, its handles carried by the derivative at their ends. */
Cubic warpOne(const Cubic& c, const Warp& f, double eps) {
  auto end = [&](Pt p, Pt h) -> std::array<Pt, 2> {
    std::array<Pt, 2> j = jacobian(f, p, eps);
    Pt jx = j[0], jy = j[1], q = f(p), v = {h[0] - p[0], h[1] - p[1]};
    return {q, Pt{q[0] + jx[0] * v[0] + jy[0] * v[1], q[1] + jx[1] * v[0] + jy[1] * v[1]}};
  };
  std::array<Pt, 2> a = end(c[0], c[1]), b = end(c[3], c[2]);
  return {a[0], a[1], b[1], b[0]};
}

}  // namespace

std::array<Pt, 2> jacobian(const Warp& f, Pt p, double eps) {
  Pt a = f({p[0] + eps, p[1]}), b = f({p[0] - eps, p[1]}), c = f({p[0], p[1] + eps}), d = f({p[0], p[1] - eps});
  return {Pt{(a[0] - b[0]) / (2 * eps), (a[1] - b[1]) / (2 * eps)}, Pt{(c[0] - d[0]) / (2 * eps), (c[1] - d[1]) / (2 * eps)}};
}

std::vector<Pt> meshGrid(Pt lo, Pt hi) {
  std::vector<Pt> out;
  for (double v : {0.0, 0.5, 1.0})
    for (double u : {0.0, 0.5, 1.0}) out.push_back({lo[0] + (hi[0] - lo[0]) * u, lo[1] + (hi[1] - lo[1]) * v});
  return out;
}

Warp mesh(Pt lo, Pt hi, std::vector<Pt> grid) {
  auto w = [](double u) -> std::array<double, 3> { return {2 * (u - 0.5) * (u - 1), -4 * u * (u - 1), 2 * u * (u - 0.5)}; };
  return [=](Pt p) -> Pt {
    std::array<double, 3> a = w((p[0] - lo[0]) / js::orElse(hi[0] - lo[0], 1)), b = w((p[1] - lo[1]) / js::orElse(hi[1] - lo[1], 1));
    double x = 0, y = 0;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) {
        double k = b[r] * a[c];
        Pt g = grid[r * 3 + c];
        x += k * g[0];
        y += k * g[1];
      }
    return {x, y};
  };
}

std::vector<Cubic> warpCubic(const Cubic& c, const Warp& f, const WarpOptions& o) {
  double eps = o.eps;
  double pieces = o.pieces ? *o.pieces : (o.step && js::truthy(*o.step) ? std::ceil(hullLength(c) / *o.step) : 1);
  int n = int(js::max(1, js::round(pieces)));
  std::vector<Cubic> out;
  for (const Cubic& q : splitEven(c, n)) out.push_back(warpOne(q, f, eps));
  return out;
}

std::vector<Cubic> warpCubics(const std::vector<Cubic>& cs, const Warp& f, const WarpOptions& o) {
  std::vector<Cubic> out;
  for (const Cubic& c : cs)
    for (const Cubic& q : warpCubic(c, f, o)) out.push_back(q);
  return out;
}

std::vector<Contour> warpContours(const std::vector<Contour>& cs, const Warp& f, const WarpOptions& o) {
  std::vector<Contour> out;
  for (const Contour& c : cs) {
    Contour r;
    for (const Piece& q : c)
      for (const Cubic& w : warpCubic(asCubic(q), f, o)) r.push_back(Piece::of(w));
    out.push_back(std::move(r));
  }
  return out;
}

Warp spine(std::variant<double, Wave> dx, double y0, double y1) {
  double h = js::orElse(y1 - y0, 1);
  Wave f;
  if (std::holds_alternative<double>(dx)) {
    double d = std::get<double>(dx);
    f = [d](double u) { return 4 * d * u * (1 - u); };
  } else {
    f = std::get<Wave>(dx);
  }
  return [=](Pt p) -> Pt { return {p[0] + f((p[1] - y0) / h), p[1]}; };
}

Warp pinned(Warp f, std::vector<double> ys, double reach) {
  return [=](Pt p) -> Pt {
    Pt q = f(p);
    double d = std::numeric_limits<double>::infinity();
    for (double y : ys) d = js::min(d, std::abs(p[1] - y));
    return {q[0], p[1] + (q[1] - p[1]) * (reach > 0 ? smoothstep(d / reach) : 1)};
  };
}

double wavelength(const Wobble& w) { return 1000 / js::max(1e-6, w.frequency); }

Warp wobble(const Wobble& w) {
  double amplitude = w.amplitude, octaves = w.octaves, seed = w.seed, size = wavelength(w);
  auto nx = noise2(seed * 2 + 1), ny = noise2(seed * 2 + 2);
  int n = int(js::max(1, js::round(octaves)));
  double total = 0;
  for (int o = 0; o < n; o++) total += std::pow(0.5, o);
  return [=](Pt p) -> Pt {
    double dx = 0, dy = 0;
    for (int o = 0; o < n; o++) {
      double k = std::pow(2, o) / size, s = std::pow(0.5, o);
      dx += s * nx(p[0] * k, p[1] * k);
      dy += s * ny(p[0] * k, p[1] * k);
    }
    return {p[0] + amplitude * dx / total, p[1] + amplitude * dy / total};
  };
}

double wobbleStep(const Wobble& w) {
  return wavelength(w) / 2 / std::pow(2, js::max(1, js::round(w.octaves)) - 1);
}

Wave wobbleWave(const Wobble& w, double length, bool closed) {
  Warp f = wobble(w);
  double r = length / (2 * PI);
  return [=](double u) {
    Pt p = closed ? Pt{r * std::cos(2 * PI * u), r * std::sin(2 * PI * u)} : Pt{u * length, 0};
    return 1 + f(p)[0] - p[0];
  };
}

std::function<double(double, double)> noise2(double seed) {
  // A seeded shuffle of 0…255 (mulberry32), doubled so lookups need no wrap.
  uint32_t s = toUint32(seed * 0x9e3779b1);
  auto rnd = [&s]() -> double {
    s = s + 0x6d2b79f5u;
    uint32_t t = s;
    t = (t ^ (t >> 15)) * (t | 1);
    t ^= t + (t ^ (t >> 7)) * (t | 61);
    return double((t ^ (t >> 14))) / 4294967296.0;
  };
  std::vector<int> p(256);
  for (int i = 0; i < 256; i++) p[i] = i;
  for (int i = 255; i > 0; i--) {
    int j = int(std::floor(rnd() * (i + 1)));
    std::swap(p[i], p[j]);
  }
  std::vector<int> P = p;
  P.insert(P.end(), p.begin(), p.end());
  std::vector<Pt> G;
  for (int i = 0; i < 8; i++) G.push_back({std::cos(i * PI / 4), std::sin(i * PI / 4)});
  auto fade = [](double t) { return t * t * t * (t * (t * 6 - 15) + 10); };
  auto grad = [G](int h, double x, double y) { Pt g = G[h & 7]; return g[0] * x + g[1] * y; };
  return [P, fade, grad](double x, double y) -> double {
    double X = std::floor(x), Y = std::floor(y), fx = x - X, fy = y - Y;
    int xi = low8(X), yi = low8(Y);
    double u = fade(fx), v = fade(fy);
    int aa = P[P[xi] + yi], ab = P[P[xi] + yi + 1], ba = P[P[xi + 1] + yi], bb = P[P[xi + 1] + yi + 1];
    double x1 = grad(aa, fx, fy) + u * (grad(ba, fx - 1, fy) - grad(aa, fx, fy));
    double x2 = grad(ab, fx, fy - 1) + u * (grad(bb, fx - 1, fy - 1) - grad(ab, fx, fy - 1));
    // Gradient noise peaks near ±0.7 in 2D; scaled so the field reaches about ±1.
    return 1.4 * (x1 + v * (x2 - x1));
  };
}

}  // namespace curves
