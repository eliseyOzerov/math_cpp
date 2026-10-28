// The C API (include/curves.h) over the C++ one: conversions and buffer copies only, no maths.
#include "curves.h"

#include "curves/bezier.hpp"

#include <bit>
#include <cstring>

using namespace curves;

static_assert(sizeof(cv_pt) == sizeof(Pt) && sizeof(cv_cubic) == sizeof(Cubic) && sizeof(cv_affine) == sizeof(Affine));

namespace {
Pt pt(cv_pt p) { return {p.x, p.y}; }
cv_pt cpt(Pt p) { return {p.x, p.y}; }
Cubic cub(const cv_cubic& c) { return std::bit_cast<Cubic>(c); }
cv_cubic ccub(const Cubic& c) { return std::bit_cast<cv_cubic>(c); }
Affine aff(const cv_affine& m) { return std::bit_cast<Affine>(m); }
cv_affine caff(const Affine& m) { return std::bit_cast<cv_affine>(m); }

/** Copies as many of `v` as fit into `out` and returns how many there are. */
template <class T, class U>
size_t copyOut(const std::vector<T>& v, U* out, size_t cap) {
  for (size_t i = 0; i < v.size() && i < cap; i++) std::memcpy(&out[i], &v[i], sizeof(U));
  return v.size();
}
}  // namespace

extern "C" {

cv_pt cv_bez(cv_cubic c, double t) { return cpt(bez(cub(c), t)); }
cv_pt cv_bez_d(cv_cubic c, double t) { return cpt(bezD(cub(c), t)); }
cv_pt cv_bez_dd(cv_cubic c, double t) { return cpt(bezDD(cub(c), t)); }
double cv_curvature(cv_cubic c, double t) { return curvature(cub(c), t); }
void cv_split(cv_cubic c, double t, cv_cubic out[2]) {
  auto [a, b] = split(cub(c), t);
  out[0] = ccub(a); out[1] = ccub(b);
}
cv_cubic cv_trim(cv_cubic c, double t0, double t1) { return ccub(trim(cub(c), t0, t1)); }
size_t cv_roots(cv_cubic c, int coord, double v, int steps, double* out, size_t cap) {
  return copyOut(roots(cub(c), coord, v, steps), out, cap);
}
int cv_nearest(const cv_cubic* segs, size_t n, cv_pt p, int steps, double* t, cv_pt* at, double* d) {
  if (n == 0) return -1;
  Nearest r = nearest(std::span<const Cubic>(reinterpret_cast<const Cubic*>(segs), n), pt(p), steps);
  if (t) *t = r.t;
  if (at) *at = cpt(r.p);
  if (d) *d = r.d;
  return r.i;
}
double cv_arc_length(cv_cubic c, double t) { return arcLength(cub(c), t); }
double cv_t_at_length(cv_cubic c, double s, const double* total) {
  return tAtLength(cub(c), s, total ? std::optional(*total) : std::nullopt);
}
size_t cv_split_even(cv_cubic c, int n, cv_cubic* out, size_t cap) { return copyOut(splitEven(cub(c), n), out, cap); }
size_t cv_arc_cubics(cv_pt a, cv_pt b, double al, cv_cubic* out, size_t cap) { return copyOut(arcCubics(pt(a), pt(b), al), out, cap); }
cv_cubic cv_arc_cubic(cv_pt a, cv_pt ta, cv_pt b, cv_pt tb) { return ccub(arcCubic(pt(a), pt(ta), pt(b), pt(tb))); }

cv_pt cv_apply(cv_affine m, cv_pt p) { return cpt(curves::apply(aff(m), pt(p))); }
cv_affine cv_then(cv_affine m, cv_affine n) { return caff(then(aff(m), aff(n))); }
cv_affine cv_rotation(double deg, cv_pt pivot) { return caff(rotation(deg, pt(pivot))); }
cv_affine cv_shear(double deg, double axis, cv_pt pivot) { return caff(shear(deg, axis, pt(pivot))); }
cv_affine cv_scaling(double kx, double ky, cv_pt pivot) { return caff(scaling(kx, ky, pt(pivot))); }
cv_affine cv_shift(double dist, double axis) { return caff(shift(dist, axis)); }

size_t cv_cubic_roots(double a, double b, double c, double d, double out[3]) { return copyOut(cubicRoots(a, b, c, d), out, 3); }
size_t cv_cubic_coord_roots(cv_cubic c, int coord, double v, double out[3]) { return copyOut(cubicCoordRoots(cub(c), coord, v), out, 3); }
size_t cv_extrema(cv_cubic c, double out[4]) { return copyOut(extrema(cub(c)).values, out, 4); }
cv_box cv_bbox(cv_cubic c) {
  Box b = bbox(cub(c));
  return {cpt(b[0]), cpt(b[1])};
}
size_t cv_inflections(cv_cubic c, double out[2]) { return copyOut(inflections(cub(c)), out, 2); }
void cv_tight_box(cv_cubic c, cv_pt out[4]) {
  auto b = tightBox(cub(c));
  for (int i = 0; i < 4; i++) out[i] = cpt(b[i]);
}
cv_cubic_kind cv_canonical(cv_cubic c, double eps, cv_pt* at) {
  Canonical k = canonical(cub(c), eps);
  if (at) *at = cpt(k.at);
  return cv_cubic_kind(int(k.kind));
}
void cv_casteljau(cv_cubic c, double t, cv_pt out[10]) {
  auto h = casteljau(cub(c), t);
  for (int i = 0; i < 10; i++) out[i] = cpt(h[i]);
}
cv_pt cv_tangent(cv_cubic c, double t) { return cpt(tangent(cub(c), t)); }
cv_pt cv_normal(cv_cubic c, double t) { return cpt(normal(cub(c), t)); }
int cv_osculating(cv_cubic c, double t, cv_pt* center, double* r, double* k) {
  Osculating o = osculating(cub(c), t);
  if (center && o.center) *center = cpt(*o.center);
  if (r) *r = o.r;
  if (k) *k = o.k;
  return o.center.has_value();
}
double cv_project(cv_cubic c, cv_pt p, cv_pt* at, double* d) {
  Projection r = project(cub(c), pt(p));
  if (at) *at = cpt(r.p);
  if (d) *d = r.d;
  return r.t;
}
cv_cubic cv_quad_through(cv_pt s, cv_pt b, cv_pt e, const double* t) {
  return ccub(quadThrough(pt(s), pt(b), pt(e), t ? std::optional(*t) : std::nullopt));
}
cv_cubic cv_cubic_through(cv_pt s, cv_pt b, cv_pt e, const double* t, const double* d1, const cv_pt* dir) {
  ThroughOptions o;
  if (t) o.t = *t;
  if (d1) o.d1 = *d1;
  if (dir) o.dir = pt(*dir);
  return ccub(cubicThrough(pt(s), pt(b), pt(e), o));
}
cv_cubic cv_mold(cv_cubic c, double t, cv_pt to) { return ccub(mold(cub(c), t, pt(to))); }

}  // extern "C"
