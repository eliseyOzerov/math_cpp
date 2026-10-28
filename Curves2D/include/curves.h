/* The C API of curves: the same functions as the C++ headers (include/curves/ headers), over plain structs, for Swift
 * (through include/module.modulemap) and wasm (through emscripten), and any other language with a C FFI.
 *
 * Conventions:
 * - cv_pt and cv_cubic have the layout of curves::Pt and curves::Cubic, so a C++ caller can pass either.
 * - Results of unknown size go into a caller's buffer `out` of `cap` items; the function returns how many there are,
 *   which may be more than `cap` (call again with a bigger buffer). None of them allocates.
 * - Optional arguments of the C++ API are pointers here: NULL takes the default.
 * - Nothing throws across this boundary. */
#ifndef CURVES_H
#define CURVES_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A point or vector. */
typedef struct cv_pt { double x, y; } cv_pt;
/** A cubic Bezier: start, two handles, end. */
typedef struct cv_cubic { cv_pt p[4]; } cv_cubic;
/** Bounds: lowest x and y, then highest. */
typedef struct cv_box { cv_pt lo, hi; } cv_box;
/** x' = a·x + c·y + e, y' = b·x + d·y + f, as {a, b, c, d, e, f}. */
typedef struct cv_affine { double m[6]; } cv_affine;

/* geometry */

cv_pt cv_bez(cv_cubic c, double t);
cv_pt cv_bez_d(cv_cubic c, double t);
cv_pt cv_bez_dd(cv_cubic c, double t);
double cv_curvature(cv_cubic c, double t);
/** Writes the two halves of `c` cut at t into out[0], out[1]. */
void cv_split(cv_cubic c, double t, cv_cubic out[2]);
cv_cubic cv_trim(cv_cubic c, double t0, double t1);
/** The parameters where coordinate `coord` (0 x, 1 y) equals `v`, sampled over `steps` (geometry's `roots`). */
size_t cv_roots(cv_cubic c, int coord, double v, int steps, double* out, size_t cap);
/** The point of `segs` nearest `p`: the segment index, and its parameter, point and distance through the pointers. */
int cv_nearest(const cv_cubic* segs, size_t n, cv_pt p, int steps, double* t, cv_pt* at, double* d);
double cv_arc_length(cv_cubic c, double t);
/** `total` may be NULL. */
double cv_t_at_length(cv_cubic c, double s, const double* total);
size_t cv_split_even(cv_cubic c, int n, cv_cubic* out, size_t cap);
size_t cv_arc_cubics(cv_pt a, cv_pt b, double al, cv_cubic* out, size_t cap);
cv_cubic cv_arc_cubic(cv_pt a, cv_pt ta, cv_pt b, cv_pt tb);

cv_pt cv_apply(cv_affine m, cv_pt p);
cv_affine cv_then(cv_affine m, cv_affine n);
cv_affine cv_rotation(double deg, cv_pt pivot);
cv_affine cv_shear(double deg, double axis, cv_pt pivot);
cv_affine cv_scaling(double kx, double ky, cv_pt pivot);
cv_affine cv_shift(double dist, double axis);

/* bezier */

/** The real roots of a·t³ + b·t² + c·t + d (at most 3). */
size_t cv_cubic_roots(double a, double b, double c, double d, double out[3]);
/** The exact parameters where coordinate `coord` of `c` equals `v` (at most 3). */
size_t cv_cubic_coord_roots(cv_cubic c, int coord, double v, double out[3]);
/** The extrema of `c` in x and y together, sorted (at most 4). */
size_t cv_extrema(cv_cubic c, double out[4]);
cv_box cv_bbox(cv_cubic c);
/** At most 2. */
size_t cv_inflections(cv_cubic c, double out[2]);
void cv_tight_box(cv_cubic c, cv_pt out[4]);

typedef enum cv_cubic_kind {
  CV_LINE, CV_ARCH, CV_SINGLE_INFLECTION, CV_DOUBLE_INFLECTION, CV_LOOP, CV_CUSP
} cv_cubic_kind;
/** The canonical kind of `c`; where its fourth point lands through `at` (may be NULL). */
cv_cubic_kind cv_canonical(cv_cubic c, double eps, cv_pt* at);

void cv_casteljau(cv_cubic c, double t, cv_pt out[10]);
cv_pt cv_tangent(cv_cubic c, double t);
cv_pt cv_normal(cv_cubic c, double t);
/** The osculating circle at t: the signed curvature, radius and centre through the pointers (may be NULL); returns
 *  0, with no centre, where it runs straight. */
int cv_osculating(cv_cubic c, double t, cv_pt* center, double* r, double* k);
/** The parameter of `c` nearest `p`; the point and distance through the pointers (may be NULL). */
double cv_project(cv_cubic c, cv_pt p, cv_pt* at, double* d);
/** `t` may be NULL. */
cv_cubic cv_quad_through(cv_pt s, cv_pt b, cv_pt e, const double* t);
/** `t`, `d1` and `dir` may be NULL. */
cv_cubic cv_cubic_through(cv_pt s, cv_pt b, cv_pt e, const double* t, const double* d1, const cv_pt* dir);
cv_cubic cv_mold(cv_cubic c, double t, cv_pt to);

#ifdef __cplusplus
}
#endif

#endif
