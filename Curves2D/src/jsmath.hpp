// Math.sin, Math.cos and Math.atan2 as V8 computes them (fdlibm, in v8/src/base/ieee754.cc), so results agree with
// the TS in the last bit on every platform. libm's differ in the last bit from one C library to another, and the
// ink's rail search and fits magnify that: a straight stroke's rail is found where a bisection's first sign is
// decided by the rounding of the last bit. The golden values come from node on arm64, whose V8 is built with fused
// multiply-adds (clang contracts each a*b + c of the C), so the places clang fuses are written with std::fma here;
// on x64 V8 does not fuse and its last bits differ. Internal to src/.
#pragma once

#include "curves/geometry.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

// No contraction of our own: every fused operation below is a std::fma.
#pragma STDC FP_CONTRACT OFF

namespace curves::js {

namespace fd {

using std::fma;

inline int32_t hi(double x) { uint64_t u; std::memcpy(&u, &x, 8); return int32_t(u >> 32); }
inline uint32_t lo(double x) { uint64_t u; std::memcpy(&u, &x, 8); return uint32_t(u); }
inline double withHi(int32_t h, uint32_t l) { uint64_t u = (uint64_t(uint32_t(h)) << 32) | l; double x; std::memcpy(&x, &u, 8); return x; }

/** sin on |x| <= pi/4 with the tail y of x. */
inline double kernelSin(double x, double y, int iy) {
  constexpr double S1 = -1.66666666666666324348e-01, S2 = 8.33333333332248946124e-03, S3 = -1.98412698298579493134e-04,
                   S4 = 2.75573137070700676789e-06, S5 = -2.50507602534068634195e-08, S6 = 1.58969099521155010221e-10;
  int32_t ix = hi(x) & 0x7fffffff;
  if (ix < 0x3e400000 && int(x) == 0) return x;
  double z = x * x, v = z * x, r = fma(z, fma(z, fma(z, fma(z, S6, S5), S4), S3), S2);
  if (iy == 0) return fma(v, fma(z, r, S1), x);
  return x - (fma(-v, S1, fma(z, fma(0.5, y, -(v * r)), -y)));
}

/** cos on |x| <= pi/4 with the tail y of x. */
inline double kernelCos(double x, double y) {
  constexpr double C1 = 4.16666666666666019037e-02, C2 = -1.38888888888741095749e-03, C3 = 2.48015872894767294178e-05,
                   C4 = -2.75573143513906633035e-07, C5 = 2.08757232129817482790e-09, C6 = -1.13596475577881948265e-11;
  int32_t ix = hi(x) & 0x7fffffff;
  if (ix < 0x3e400000 && int(x) == 0) return 1.0;
  double z = x * x, r = z * fma(z, fma(z, fma(z, fma(z, fma(z, C6, C5), C4), C3), C2), C1);
  double zr = fma(z, r, -(x * y));
  if (ix < 0x3FD33333) return 1.0 - fma(0.5, z, -zr);
  double qx = ix > 0x3fe90000 ? 0.28125 : withHi(ix - 0x00200000, 0);
  double hz = fma(0.5, z, -qx), a = 1.0 - qx;
  return a - (hz - zr);
}

/** x reduced to y[0] + y[1] in [-pi/4, pi/4], by the quadrant n returned (medium sizes; larger go to libm). */
inline int remPio2(double x, double* y) {
  constexpr double invpio2 = 6.36619772367581382433e-01, pio2_1 = 1.57079632673412561417e+00, pio2_1t = 6.07710050650619224932e-11,
                   pio2_2 = 6.07710050630396597660e-11, pio2_2t = 2.02226624879595063154e-21, pio2_3 = 2.02226624871116645580e-21,
                   pio2_3t = 8.47842766036889956997e-32;
  static const int32_t npio2_hw[] = {
    0x3FF921FB, 0x400921FB, 0x4012D97C, 0x401921FB, 0x401F6A7A, 0x4022D97C, 0x4025FDBB, 0x402921FB, 0x402C463A, 0x402F6A7A,
    0x4031475C, 0x4032D97C, 0x40346B9C, 0x4035FDBB, 0x40378FDB, 0x403921FB, 0x403AB41B, 0x403C463A, 0x403DD85A, 0x403F6A7A,
    0x40407E4C, 0x404147AC, 0x4042106C, 0x4042D97C, 0x4043A28C, 0x40446B9C, 0x404534AC, 0x4045FDBB, 0x4046C6CB, 0x40478FDB,
    0x404858EB, 0x404921FB};
  int32_t hx = hi(x), ix = hx & 0x7fffffff;
  if (ix <= 0x3fe921fb) { y[0] = x; y[1] = 0; return 0; }
  if (ix < 0x4002d97c) {
    double z;
    if (hx > 0) {
      z = x - pio2_1;
      if (ix != 0x3ff921fb) { y[0] = z - pio2_1t; y[1] = (z - y[0]) - pio2_1t; }
      else { z -= pio2_2; y[0] = z - pio2_2t; y[1] = (z - y[0]) - pio2_2t; }
      return 1;
    }
    z = x + pio2_1;
    if (ix != 0x3ff921fb) { y[0] = z + pio2_1t; y[1] = (z - y[0]) + pio2_1t; }
    else { z += pio2_2; y[0] = z + pio2_2t; y[1] = (z - y[0]) + pio2_2t; }
    return -1;
  }
  double t = std::abs(x);
  int n = int(fma(t, invpio2, 0.5));
  double fn = double(n), r = fma(-fn, pio2_1, t), w = fn * pio2_1t;
  if (n < 32 && ix != npio2_hw[n - 1]) y[0] = r - w;
  else {
    int32_t j = ix >> 20;
    y[0] = r - w;
    int32_t i = j - ((hi(y[0]) >> 20) & 0x7ff);
    if (i > 16) {
      t = r; w = fn * pio2_2; r = t - w; w = fma(fn, pio2_2t, -((t - r) - w)); y[0] = r - w;
      i = j - ((hi(y[0]) >> 20) & 0x7ff);
      if (i > 49) { t = r; w = fn * pio2_3; r = t - w; w = fma(fn, pio2_3t, -((t - r) - w)); y[0] = r - w; }
    }
  }
  y[1] = (r - y[0]) - w;
  if (hx < 0) { y[0] = -y[0]; y[1] = -y[1]; return -n; }
  return n;
}

/** Whether x is small enough for remPio2 above (|x| <= 2^19 pi/2). */
inline bool medium(double x) { return (hi(x) & 0x7fffffff) <= 0x413921fb; }

inline double atan(double x) {
  static const double atanhi[] = {4.63647609000806093515e-01, 7.85398163397448278999e-01, 9.82793723247329054082e-01, 1.57079632679489655800e+00};
  static const double atanlo[] = {2.26987774529616870924e-17, 3.06161699786838301793e-17, 1.39033110312309984516e-17, 6.12323399573676603587e-17};
  static const double aT[] = {3.33333333333329318027e-01, -1.99999999998764832476e-01, 1.42857142725034663711e-01, -1.11111104054623557880e-01,
                              9.09088713343650656196e-02, -7.69187620504482999495e-02, 6.66107313738753120669e-02, -5.83357013379057348645e-02,
                              4.97687799461593236017e-02, -3.65315727442169155270e-02, 1.62858201153657823623e-02};
  int32_t hx = hi(x), ix = hx & 0x7fffffff;
  int id;
  if (ix >= 0x44100000) {
    if (ix > 0x7ff00000 || (ix == 0x7ff00000 && lo(x) != 0)) return x + x;
    return hx > 0 ? atanhi[3] + atanlo[3] : -atanhi[3] - atanlo[3];
  }
  if (ix < 0x3fdc0000) {
    if (ix < 0x3e200000 && 1e300 + x > 1.0) return x;
    id = -1;
  } else {
    x = std::abs(x);
    if (ix < 0x3ff30000) {
      if (ix < 0x3fe60000) { id = 0; x = (2.0 * x - 1.0) / (2.0 + x); }
      else { id = 1; x = (x - 1.0) / (x + 1.0); }
    } else if (ix < 0x40038000) { id = 2; x = (x - 1.5) / fma(1.5, x, 1.0); }
    else { id = 3; x = -1.0 / x; }
  }
  double z = x * x, w = z * z;
  double s1 = z * fma(w, fma(w, fma(w, fma(w, fma(w, aT[10], aT[8]), aT[6]), aT[4]), aT[2]), aT[0]);
  double s2 = w * fma(w, fma(w, fma(w, fma(w, aT[9], aT[7]), aT[5]), aT[3]), aT[1]);
  if (id < 0) return fma(-x, s1 + s2, x);
  z = atanhi[id] - (fma(x, s1 + s2, -atanlo[id]) - x);
  return hx < 0 ? -z : z;
}

}  // namespace fd

/** `Math.sin`. */
inline double sin(double x) {
  if (!fd::medium(x)) return std::sin(x);
  int32_t ix = fd::hi(x) & 0x7fffffff;
  if (ix <= 0x3fe921fb) return fd::kernelSin(x, 0, 0);
  double y[2];
  switch (fd::remPio2(x, y) & 3) {
    case 0: return fd::kernelSin(y[0], y[1], 1);
    case 1: return fd::kernelCos(y[0], y[1]);
    case 2: return -fd::kernelSin(y[0], y[1], 1);
    default: return -fd::kernelCos(y[0], y[1]);
  }
}
/** `Math.cos`. */
inline double cos(double x) {
  if (!fd::medium(x)) return std::cos(x);
  int32_t ix = fd::hi(x) & 0x7fffffff;
  if (ix <= 0x3fe921fb) return fd::kernelCos(x, 0);
  double y[2];
  switch (fd::remPio2(x, y) & 3) {
    case 0: return fd::kernelCos(y[0], y[1]);
    case 1: return -fd::kernelSin(y[0], y[1], 1);
    case 2: return -fd::kernelCos(y[0], y[1]);
    default: return fd::kernelSin(y[0], y[1], 1);
  }
}
/** `Math.atan2`. Zeros and infinities go to libm, which agrees on them. */
inline double atan2(double y, double x) {
  if (!std::isfinite(x) || !std::isfinite(y) || x == 0 || y == 0) return std::atan2(y, x);
  constexpr double pi = 3.1415926535897931160E+00, pi_lo = 1.2246467991473531772E-16, pi_o_2 = 1.5707963267948965580E+00;
  int32_t hx = fd::hi(x), ix = hx & 0x7fffffff, hy = fd::hi(y), iy = hy & 0x7fffffff;
  if (hx == 0x3ff00000 && fd::lo(x) == 0) return fd::atan(y);
  int m = ((hy >> 31) & 1) | ((hx >> 30) & 2);
  int k = (iy - ix) >> 20;
  double z;
  if (k > 60) { z = pi_o_2 + 0.5 * pi_lo; m &= 1; }
  else if (hx < 0 && k < -60) z = 0.0;
  else z = fd::atan(std::abs(y / x));
  switch (m) {
    case 0: return z;
    case 1: return -z;
    case 2: return pi - (z - pi_lo);
    default: return (z - pi_lo) - pi;
  }
}

/** geometry.hpp's rotate and fromAngle with V8's sin and cos. */
inline Pt rotate(Pt v, double t) { return {v.x * cos(t) - v.y * sin(t), v.x * sin(t) + v.y * cos(t)}; }
inline Pt fromAngle(double a) { return {cos(a), sin(a)}; }

}  // namespace curves::js
