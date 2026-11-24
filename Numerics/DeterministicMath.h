// The double-precision functions of the deterministic tier (Docs/Foundations.md, "Two tiers of
// arithmetic"): sin, cos, tan, their inverses, exp, log, sinh and hypot, written out here so
// that the same inputs give the same bits on Apple arm64, x86-64 and WebAssembly.
//
// Why they exist: the system libraries (Apple's libm, glibc, the musl in emscripten) each use
// their own algorithms and tables, and they differ in the last bit for some arguments. A result
// that is saved, replayed, checked by a server or compared across machines cannot tolerate that,
// so these functions use one algorithm everywhere: only +, -, *, / and sqrt (which IEEE 754
// requires to be correctly rounded), in a fixed order, with no fused multiply-adds
// (`-ffp-contract=off`) and no `long double`.
//
// Source: fdlibm 5.3, Sun Microsystems, 1993; via FreeBSD msun (lib/msun/src), whose notice is
// in THIRD_PARTY_NOTICES.md. The algorithms and constants are the originals; `hypot` is our own.
//
// Differences from fdlibm, all about being the same on every machine:
// - A NaN result is always the same quiet NaN (0x7FF8000000000000). The hardware picks the sign
//   and payload of a NaN differently (x86 makes a negative one for 0/0, Arm a positive one).
// - Floating-point exception flags and the rounding mode are not touched; the mode is assumed
//   to be round to nearest.
// - The code that only raised "inexact" flags (`huge + x > one` and the like) is left out.
#pragma once

namespace editor::deterministic {

/// The sine of `x` (radians), within 1 ulp.
///
/// Source: fdlibm 5.3 `s_sin.c`, `k_sin.c`, `k_cos.c`, `e_rem_pio2.c`, `k_rem_pio2.c`.
/// If |x| ≤ π/4 a degree-13 polynomial is used directly. Otherwise `x` is reduced to
/// `r = x − n·π/2` with |r| ≤ π/4, using π/2 split in several pieces so that `n·π/2` is exact
/// (for |x| below about 2²⁰·π/2), or the digits of 2/π (for larger |x|, up to the largest
/// double), and then sin(x) is ±sin(r) or ±cos(r) by `n mod 4`.
///
/// Conventions: sin(±0) = ±0; sin(±∞) and sin(NaN) are NaN.
///
/// Use when: rotating by an angle in a simulation that must replay bit for bit; evaluating a
/// saved curve on any machine.
/// Tags: sine, trigonometry, deterministic, cross-platform, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double sin(double x);

/// The cosine of `x` (radians), within 1 ulp. Same method as `sin`.
///
/// Conventions: cos(±0) = 1; cos(±∞) and cos(NaN) are NaN.
///
/// Source: fdlibm 5.3 `s_cos.c`, `k_cos.c`, `k_sin.c`, `e_rem_pio2.c`, `k_rem_pio2.c`.
///
/// Use when: the horizontal part of a direction from an angle, under replay or netcode.
/// Tags: cosine, trigonometry, deterministic, cross-platform, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double cos(double x);

/// The tangent of `x` (radians), within 1 ulp. For |r| above 0.6744 the kernel uses
/// tan(π/4 − r) = (1 − tan r)/(1 + tan r) so the polynomial stays on a short interval; odd
/// multiples of π/2 give a very large number, not infinity.
///
/// Conventions: tan(±0) = ±0; tan(±∞) and tan(NaN) are NaN.
///
/// Source: fdlibm 5.3 `s_tan.c`, `k_tan.c`, `e_rem_pio2.c`, `k_rem_pio2.c`.
///
/// Use when: the slope of a field-of-view or a ramp angle that must match on every machine.
/// Tags: tangent, trigonometry, deterministic, cross-platform, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double tan(double x);

/// The angle in [−π/2, π/2] whose sine is `x`, within 1 ulp. Uses
/// asin(x) = π/2 − 2·asin(√((1 − x)/2)) for |x| ≥ 0.5 and a rational approximation of
/// (asin(x) − x)/x³ below that.
///
/// Conventions: asin(±0) = ±0; |x| > 1 and NaN give NaN.
///
/// Source: fdlibm 5.3 `e_asin.c`.
///
/// Use when: recovering an angle from a sine, such as a hinge angle from a cross product.
/// Tags: arcsine, inverse sine, trigonometry, deterministic, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double asin(double x);

/// The angle in [0, π] whose cosine is `x`, within 1 ulp. Uses
/// acos(x) = 2·asin(√((1 − x)/2)), and acos(−x) = π − acos(x).
///
/// Conventions: acos(1) = +0, acos(−1) = π; |x| > 1 and NaN give NaN.
///
/// Source: fdlibm 5.3 `e_acos.c`.
///
/// Use when: the angle between two unit vectors from their dot product, under replay.
/// Tags: arccosine, inverse cosine, trigonometry, deterministic, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double acos(double x);

/// The angle in [−π/2, π/2] whose tangent is `x`, within 1 ulp. The argument is mapped to
/// [−0.4375, 0.4375] with atan(x) = atan(c) + atan((x − c)/(1 + x·c)) for c in
/// {0.5, 1, 1.5, ∞}, then an odd polynomial of degree 21 is used.
///
/// Conventions: atan(±0) = ±0; atan(±∞) = ±π/2; NaN gives NaN.
///
/// Source: fdlibm 5.3 `s_atan.c`.
///
/// Use when: the slope angle of a gradient; the building block of `atan2`.
/// Tags: arctangent, inverse tangent, trigonometry, deterministic, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double atan(double x);

/// The angle of the point (`x`, `y`) measured from the +x axis, in [−π, π], within 2 ulp.
/// It is atan(y/x) moved into the right quadrant by the signs of `x` and `y`.
///
/// Conventions (C99 Annex F): atan2(±0, +0) = ±0, atan2(±0, −0) = ±π, atan2(±0, x<0) = ±π,
/// atan2(y, ±0) = ±π/2 for y ≠ 0, atan2(±y finite, +∞) = ±0, atan2(±y finite, −∞) = ±π,
/// atan2(±∞, +∞) = ±π/4, atan2(±∞, −∞) = ±3π/4; NaN in either argument gives NaN.
///
/// Source: fdlibm 5.3 `e_atan2.c`.
///
/// Use when: the heading of a velocity vector; the polar angle of a point.
/// Tags: atan2, polar angle, heading, quadrant, trigonometry, deterministic, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double atan2(double y, double x);

/// e raised to `x`, within 1 ulp. Writes `x = k·ln 2 + r` with |r| ≤ ln 2/2, evaluates
/// e^r = 1 + r + r·c/(2 − c) with c a degree-10 polynomial in r², and multiplies by 2ᵏ.
///
/// Conventions: exp(+∞) = +∞, exp(−∞) = +0, exp(NaN) = NaN; overflows to +∞ above
/// 709.782712893384 and underflows (through subnormals) to +0 below −745.1332191019411.
///
/// Source: fdlibm 5.3 `e_exp.c` (FreeBSD's version, with the two-step scaling near the ends).
///
/// Use when: decay and growth in a simulation; the building block of `sinh`.
/// Tags: exponential, e to the x, deterministic, cross-platform, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double exp(double x);

/// `exp(x) − 1`, without losing the digits when `x` is near 0, within 1 ulp.
///
/// Conventions: expm1(±0) = ±0, expm1(+∞) = +∞, expm1(−∞) = −1, expm1(NaN) = NaN.
///
/// Source: fdlibm 5.3 `s_expm1.c`.
///
/// Use when: a small relative change, such as `1 − exp(−rate·dt)` for a short time step.
/// Tags: expm1, exponential minus one, small argument, deterministic, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double expm1(double x);

/// The natural logarithm of `x`, within 1 ulp. Writes `x = 2ᵏ·(1 + f)` with
/// √2/2 < 1 + f < √2 and uses log(1 + f) = f − (f²/2 − s·(f²/2 + R)) with s = f/(2 + f) and R a
/// degree-14 polynomial in s.
///
/// Conventions: log(+0) = log(−0) = −∞; log(x<0) and log(NaN) are NaN; log(+∞) = +∞;
/// log(1) = +0.
///
/// Source: fdlibm 5.3 `e_log.c`.
///
/// Use when: converting a ratio to octaves or decibels; the inverse of `exp`.
/// Tags: natural logarithm, ln, deterministic, cross-platform, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double log(double x);

/// `log(1 + x)`, without losing the digits when `x` is near 0, within 1 ulp.
///
/// Conventions: log1p(±0) = ±0, log1p(−1) = −∞, log1p(x < −1) and log1p(NaN) are NaN,
/// log1p(+∞) = +∞.
///
/// Source: fdlibm 5.3 `s_log1p.c`.
///
/// Use when: a small relative growth turned into a rate, such as the log of 1 + percent/100.
/// Tags: log1p, logarithm of one plus, small argument, deterministic, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double log1p(double x);

/// The hyperbolic sine, (eˣ − e⁻ˣ)/2, within 2 ulp. Uses `expm1` for |x| < 22 so the
/// subtraction never cancels, `exp` above, and a scaled exponential near the overflow limit.
///
/// Conventions: sinh(±0) = ±0, sinh(±∞) = ±∞, sinh(NaN) = NaN; overflows to ±∞ for
/// |x| above 710.4758600739439.
///
/// Source: fdlibm 5.3 `e_sinh.c`, with `__ldexp_exp` from FreeBSD `k_exp.c`.
///
/// Use when: the catenary of a hanging cable; easing curves built on sinh.
/// Tags: sinh, hyperbolic sine, catenary, deterministic, fdlibm
/// Node: none (arithmetic under the deterministic tier)
double sinh(double x);

/// The length √(x² + y²), without overflow or underflow in the intermediate squares.
///
/// Our own, not fdlibm's: the larger magnitude is scaled to [1, 2) by an exact power of two
/// (the smaller one by the same factor), then `√(sx·sx + sy·sy)` is computed in that order and
/// scaled back with one multiplication. Squares of numbers that size cannot overflow, and the
/// smaller one's square is lost only when it is below 2⁻¹⁰⁶ of the larger's, where it cannot
/// change the sum anyway. The error is about 1 ulp.
///
/// Conventions: an infinite argument gives +∞ even when the other is NaN (IEEE 754); otherwise
/// NaN in either gives NaN; hypot(±0, ±0) = +0; results above the largest double are +∞.
///
/// Use when: the distance between two points that may be huge or tiny; the magnitude of a
/// complex number.
/// Tags: hypot, length, distance, magnitude, norm, overflow-safe, deterministic
/// Node: none (arithmetic under the deterministic tier)
double hypot(double x, double y);

/// The length √(x² + y² + z²), scaled like the two-argument `hypot`, summed in the order
/// `(sx·sx + sy·sy) + sz·sz`.
///
/// Conventions: as `hypot(x, y)`: any infinite argument gives +∞, otherwise any NaN gives NaN.
///
/// Use when: the length of a 3D vector whose components may be huge or tiny.
/// Tags: hypot, length, 3d distance, magnitude, norm, overflow-safe, deterministic
/// Node: none (arithmetic under the deterministic tier)
double hypot(double x, double y, double z);

}  // namespace editor::deterministic
