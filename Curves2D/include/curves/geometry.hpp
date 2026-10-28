// Points, vectors and Bezier pieces as small value types, so curve code reads like the math: vector math, a single
// cubic (evaluated, split, trimmed), where it crosses a level and its nearest point, its arc length, circular arcs as
// cubics, and affine maps of the plane. Everything includes this file, so no helper is written twice.
//
// Ported from fountain-ts/src/curves/geometry.ts, which stays the reference until the port is complete: the same
// names, the same formulas in the same order, so the golden tests (tests/golden) match to rounding.
//
// Decisions:
// - Pt is a struct with operators, Cubic a std::array of four, Affine a std::array of six (x' = a·x + c·y + e,
//   y' = b·x + d·y + f), so they are trivially copyable and cross the C API (curves.h) as they are.
// - A Piece is a line or a cubic in one struct (`line` set and only p[0], p[1] used), not a variant, for the same
//   reason.
// - TS defaults that depend on another argument (tAtLength's total) become std::optional.
// - No exceptions except where the TS throws (bounds of nothing); the C API catches them.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <span>
#include <utility>
#include <vector>

// Inline code in these headers compiles with the caller's flags, and clang fuses a·b + c into one rounding by default
// (-ffp-contract=on). The functions below that multiply and add turn that off, so they round as JS does wherever they
// are called from. (GCC leaves contraction off in ISO C++ modes; MSVC only contracts under /fp:contract.)
#if defined(__clang__)
#define CURVES_NO_FMA _Pragma("STDC FP_CONTRACT OFF")
#else
#define CURVES_NO_FMA
#endif

namespace curves {

/** A point or vector. */
struct Pt {
  double x = 0, y = 0;
  constexpr double operator[](int i) const { return i == 0 ? x : y; }
  constexpr double& operator[](int i) { return i == 0 ? x : y; }
  friend constexpr bool operator==(const Pt&, const Pt&) = default;
};

constexpr Pt operator+(Pt a, Pt b) { return {a.x + b.x, a.y + b.y}; }
constexpr Pt operator-(Pt a, Pt b) { return {a.x - b.x, a.y - b.y}; }
constexpr Pt operator-(Pt v) { return {-v.x, -v.y}; }
constexpr Pt operator*(Pt a, double k) { return {a.x * k, a.y * k}; }
constexpr Pt operator*(double k, Pt a) { return {a.x * k, a.y * k}; }

/** A cubic Bezier: start, two handles, end. */
using Cubic = std::array<Pt, 4>;

/** An outline piece: a cubic, or a line from p[0] to p[1] when `line` is set. */
struct Piece {
  std::array<Pt, 4> p{};
  bool line = false;
  static constexpr Piece of(Pt a, Pt b) { return {{a, b, {}, {}}, true}; }
  static constexpr Piece of(const Cubic& c) { return {c, false}; }
  constexpr Pt first() const { return p[0]; }
  constexpr Pt last() const { return line ? p[1] : p[3]; }
};

/** A closed outline: pieces end to end, the last ending where the first starts. */
using Contour = std::vector<Piece>;

/** Bounds: lowest x and y, then highest. */
using Box = std::array<Pt, 2>;

inline constexpr double PI = 3.14159265358979323846;

constexpr double dot(Pt a, Pt b) { CURVES_NO_FMA return a.x * b.x + a.y * b.y; }
constexpr double cross(Pt a, Pt b) { CURVES_NO_FMA return a.x * b.y - a.y * b.x; }
/** Math.hypot as V8 works it out (scaled by the larger, a compensated sum, then the root), so lengths round as in
 *  the TS on every platform: libm's hypot differs in the last bit from one C library to another. */
inline double hypot(double a, double b) {
  CURVES_NO_FMA
  a = std::abs(a), b = std::abs(b);
  if (a == INFINITY || b == INFINITY) return INFINITY;
  if (std::isnan(a) || std::isnan(b)) return NAN;
  double max = a > b ? a : b;
  if (max == 0) return 0;
  double sum = 0, compensation = 0;
  for (double v : {a, b}) {
    double n = v / max, summand = n * n - compensation, preliminary = sum + summand;
    compensation = (preliminary - sum) - summand;
    sum = preliminary;
  }
  return std::sqrt(sum) * max;
}
inline double dist(Pt a, Pt b) { return hypot(a.x - b.x, a.y - b.y); }
inline double len(Pt v) { return hypot(v.x, v.y); }
constexpr Pt lerp(Pt a, Pt b, double t) { CURVES_NO_FMA return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}; }
constexpr double rad(double deg) { return deg * PI / 180; }
constexpr double deg(double r) { return r * 180 / PI; }
/** The unit vector at angle `a` (radians) from the x axis. */
inline Pt fromAngle(double a) { return {std::cos(a), std::sin(a)}; }
/** The mean of points. */
Pt mean(std::span<const Pt> ps);
/** v held between a and b. */
constexpr double clamp(double v, double a, double b) { return std::max(a, std::min(b, v)); }

/** An angle (radians) folded into (−π/2, π/2]: a direction without caring which way along it runs. */
double halfTurn(double a);
/** The signed angle from direction `a` to `b`, in radians: positive counter-clockwise. */
inline double turn(Pt a, Pt b) { return std::atan2(cross(a, b), dot(a, b)); }
/** `v` turned by `t` radians counter-clockwise. */
inline Pt rotate(Pt v, double t) { CURVES_NO_FMA return {v.x * std::cos(t) - v.y * std::sin(t), v.x * std::sin(t) + v.y * std::cos(t)}; }
/** A unit vector along v, or (0, 0) for a zero one. */
Pt norm(Pt v);

/** A cubic's point at t. */
Pt bez(const Cubic& c, double t);
/** A cubic's first derivative at t. */
Pt bezD(const Cubic& c, double t);
/** A cubic's second derivative at t. */
Pt bezDD(const Cubic& c, double t);
/** A cubic's signed curvature at t: one over its radius there, positive where it turns counter-clockwise; 0 where it
 *  has no direction (a handle on its node). */
double curvature(const Cubic& c, double t);

/** Whether `p` lies within `tol` of the line through `a` and `b`, and at most `over` (default `tol`) beyond either end. */
bool nearSegment(Pt p, Pt a, Pt b, double tol, std::optional<double> over = {});

/** Where the line from `p` along `d` meets the line from `q` along `e`, as how far along each: {s, r} with
 *  p + s·d = q + r·e; nothing when they run parallel (|d × e| below `flat`). With `inside`, the segments p…p + d and
 *  q…q + e: nothing unless both lie strictly inside, more than `inside` from either end. */
std::optional<std::array<double, 2>> meet(Pt p, Pt d, Pt q, Pt e, double flat = 1e-9, std::optional<double> inside = {});

/** Whether `c` is straight: its handles on the line between its ends (to a share of its length). */
bool isLine(const Cubic& c);
/** A straight line as a cubic, its handles at the thirds. */
constexpr Cubic lineCubic(Pt a, Pt b) { return {a, lerp(a, b, 1.0 / 3), lerp(a, b, 2.0 / 3), b}; }
/** A piece as a cubic: a line gets its handles at the thirds. */
constexpr Cubic asCubic(const Piece& q) { return q.line ? lineCubic(q.p[0], q.p[1]) : q.p; }
/** A piece run backwards. */
Piece reverse(const Piece& q);
/** A cubic run backwards. */
constexpr Cubic reverse(const Cubic& c) { return {c[3], c[2], c[1], c[0]}; }
/** The direction a piece leaves its start by and arrives at its end by, skipping handles that sit on their node. */
std::array<Pt, 2> endTangents(const Piece& q);

/** A cubic cut in two at t, by de Casteljau. */
std::array<Cubic, 2> split(const Cubic& c, double t);
/** The part of a cubic from t0 to t1 (t0 < t1). */
Cubic trim(const Cubic& c, double t0, double t1);

/** The parameters t in [0, 1] where coordinate `coord` (0 for x, 1 for y) of `c` equals `v`, in increasing order:
 *  sign changes bracketed over `steps` even steps and bisected (bezier.hpp's cubicCoordRoots is exact). */
std::vector<double> roots(const Cubic& c, int coord, double v, int steps = 32);

/** The point of a set of cubics nearest a point: the segment `i`, its parameter `t`, the point and its distance. */
struct Nearest {
  int i = 0;
  double t = 0;
  Pt p;
  double d = 0;
};
/** The point of the cubics `segs` nearest `p`: each sampled at `steps` even steps and the best sample refined by
 *  Newton's method on the squared distance, falling back to the sample where a step would go further. */
Nearest nearest(std::span<const Cubic> segs, Pt p, int steps = 32);

/** The length of `c` from 0 to `t`, by 24-point Legendre–Gauss quadrature as in Bezier.js. */
double arcLength(const Cubic& c, double t = 1);
/** The parameter at which `c` has run `s` of its length (clamped to 0…1); `total` is its length if known. */
double tAtLength(const Cubic& c, double s, std::optional<double> total = {});
/** A piece's length: a line's straight, a cubic's along it. */
double pieceLength(const Piece& q);
/** The bounds of pieces, a cubic's from `samples` + 1 points along it (its control points would overshoot). */
Box box(std::span<const Piece> qs, int samples = 16);
/** `c` with each piece started exactly where the one before ends, round the loop (in place, and returned). */
Contour& stitch(Contour& c);

/** The handle length, as a fraction of the radius, of a cubic quarter of a circle. */
inline constexpr double KAPPA = 0.5523;
/** A cubic from `a` (heading along `ta`) to `b` (heading along `tb`) that bends like a circular arc. */
Cubic arcCubic(Pt a, Pt ta, Pt b, Pt tb);
/** The circular arc from `a` to `b` that leaves its chord `al` radians counter-clockwise of it (and so turns by
 *  −2·al), as cubics of at most a quarter turn each. */
std::vector<Cubic> arcCubics(Pt a, Pt b, double al);
/** `c` cut at shares `shares` (ascending, inside 0…1) of its length, measured in units of `metric`. */
std::vector<Cubic> cutAt(const Cubic& c, std::span<const double> shares, Pt metric = {1, 1});
/** `c` cut into `n` pieces at even steps of its parameter. */
std::vector<Cubic> splitEven(const Cubic& c, int n);

/** The running lengths along a polyline: 0 at its first point, then the distance travelled to each. */
std::vector<double> runLengths(std::span<const Pt> ps);
/** The point `s` along a polyline whose running lengths are `run` (runLengths), clamped to its ends. */
Pt pointAlong(std::span<const Pt> ps, std::span<const double> run, double s);
/** The length of `c`'s control polygon: at least its arc length, and quick. */
inline double hullLength(const Cubic& c) { return dist(c[0], c[1]) + dist(c[1], c[2]) + dist(c[2], c[3]); }
/** The signed area of the polygon through `ps`: positive counter-clockwise (y up). */
double area(std::span<const Pt> ps);
/** The bounds of `ps`; throws std::invalid_argument on none. */
Box bounds(std::span<const Pt> ps);
/** Whether bounds `a` and `b` overlap, or come within `e` of each other. */
constexpr bool overlaps(const Box& a, const Box& b, double e = 0) {
  return a[0].x <= b[1].x + e && b[0].x <= a[1].x + e && a[0].y <= b[1].y + e && b[0].y <= a[1].y + e;
}
/** The point a share `at` of the way across and up bounds `b`: (0, 0) its bottom left, (1, 1) its top right. */
constexpr Pt within(const Box& b, Pt at) { CURVES_NO_FMA return {b[0].x + (b[1].x - b[0].x) * at.x, b[0].y + (b[1].y - b[0].y) * at.y}; }
/** The middle of the bounds of `ps`. */
inline Pt center(std::span<const Pt> ps) { return within(bounds(ps), {0.5, 0.5}); }

// Affine maps of the plane: rotation, shear along an axis and a shift along an axis, each about a pivot. Angles are
// in degrees, counter-clockwise from the x axis.

/** x' = a·x + c·y + e, y' = b·x + d·y + f, stored {a, b, c, d, e, f}. */
using Affine = std::array<double, 6>;

/** A point in the units of `metric` (what a unit across and a unit up are worth), and back. */
constexpr Pt toMetric(Pt p, Pt metric) { return {p.x * metric.x, p.y * metric.y}; }
constexpr Pt fromMetric(Pt p, Pt metric) { return {p.x / metric.x, p.y / metric.y}; }

/** The map that leaves every point where it is. */
inline constexpr Affine identity{1, 0, 0, 1, 0, 0};
/** Where `m` takes `p`. */
constexpr Pt apply(const Affine& m, Pt p) { CURVES_NO_FMA return {m[0] * p.x + m[2] * p.y + m[4], m[1] * p.x + m[3] * p.y + m[5]}; }
/** `m` then `n`: the single map that applies `m` first. */
Affine then(const Affine& m, const Affine& n);
/** A turn of `deg` about `pivot`. */
Affine rotation(double deg, Pt pivot = {0, 0});
/** A shear along the axis at `axis` degrees through `pivot`: lines across the axis lean by `deg`. */
Affine shear(double deg, double axis = 0, Pt pivot = {0, 0});
/** `m`, made in the units of `metric`, as a map of the plain coordinates. */
Affine inMetric(const Affine& m, Pt metric);
/** A scaling by `kx` across and `ky` up about `pivot`; -1 one way mirrors. */
Affine scaling(double kx, double ky, Pt pivot = {0, 0});
/** `m` taken `w` of the way from the identity: exact for shifts and scalings, a shortcut for turns. */
Affine partly(const Affine& m, double w);
/** A move of `dist` along the axis at `axis` degrees. */
Affine shift(double dist, double axis = 0);

}  // namespace curves
