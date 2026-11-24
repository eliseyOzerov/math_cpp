// The deterministic tier's sin, cos, tan, asin, acos, atan, atan2, exp, expm1, log, log1p, sinh
// and hypot (Numerics/DeterministicMath.h). The functions are Sun's fdlibm 5.3 as FreeBSD's msun
// ships it, with the same algorithms and constants; the layout is changed to read as C++20.
// Constants keep the comment with their exact bit pattern, so a typo shows up against it.
//
// fdlibm's notice (every function below up to `hypot`):
//
//   Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
//   Developed at SunSoft, a Sun Microsystems, Inc. business.
//   Permission to use, copy, modify, and distribute this software is freely granted, provided
//   that this notice is preserved.
//
// The kernels must not be fused into multiply-adds; the build passes -ffp-contract=off, and the
// pragma below asks for the same of clang in case a file is compiled without the flag.
#include "DeterministicMath.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

#if defined(__clang__)
#pragma clang fp contract(off)
#endif

namespace editor::deterministic {

namespace {

// ---------------------------------------------------------------------------------------------
// Bits of a double. fdlibm's GET_HIGH_WORD and friends, through std::bit_cast.
// ---------------------------------------------------------------------------------------------

/// The upper 32 bits of `x`: sign, 11 exponent bits, 20 high bits of the fraction.
uint32_t highWord(double x) { return static_cast<uint32_t>(std::bit_cast<uint64_t>(x) >> 32); }

/// The lower 32 bits of `x`: the rest of the fraction.
uint32_t lowWord(double x) { return static_cast<uint32_t>(std::bit_cast<uint64_t>(x)); }

/// The double whose upper 32 bits are `high` and lower 32 bits are `low`.
double fromWords(uint32_t high, uint32_t low) {
    return std::bit_cast<double>((static_cast<uint64_t>(high) << 32) | low);
}

/// `x` with its upper 32 bits replaced.
double withHighWord(double x, uint32_t high) { return fromWords(high, lowWord(x)); }

/// `x` with its lower 32 bits replaced by zero: the head of `x` with 21 significant bits, whose
/// square and product with another such head are exact.
double withLowWordCleared(double x) { return fromWords(highWord(x), 0); }

/// The absolute value of `x`, by clearing the sign bit (also right for NaN and -0).
double magnitude(double x) { return std::bit_cast<double>(std::bit_cast<uint64_t>(x) & 0x7fffffffffffffffULL); }

/// The one NaN every function returns, so no result depends on how the hardware makes a NaN.
double quietNaN() { return std::bit_cast<double>(0x7ff8000000000000ULL); }

double infinity() { return std::numeric_limits<double>::infinity(); }

/// `value`, with any NaN replaced by `quietNaN()`.
double withCanonicalNaN(double value) { return value != value ? quietNaN() : value; }

/// 2ⁿ for n from -1074 (the smallest subnormal) to 1023, exact.
double powerOfTwo(int exponent) {
    if (exponent >= -1022) return std::bit_cast<double>(static_cast<uint64_t>(exponent + 1023) << 52);
    return std::bit_cast<double>(uint64_t{1} << (exponent + 1074));
}

// ---------------------------------------------------------------------------------------------
// Argument reduction: x − n·π/2 (fdlibm 5.3 e_rem_pio2.c and k_rem_pio2.c).
// ---------------------------------------------------------------------------------------------

/// 2/π to 396 hexadecimal digits: the (24·i)-th to (24·i+23)-th bits after the binary point.
/// The value of entry i is `ipio2[i]·2^(−24(i+1))`. (FreeBSD keeps 66 entries for double.)
constexpr int32_t twoOverPi[] = {
    0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62,
    0x95993C, 0x439041, 0xFE5163, 0xABDEBB, 0xC561B7, 0x246E3A,
    0x424DD2, 0xE00649, 0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129,
    0xA73EE8, 0x8235F5, 0x2EBB44, 0x84E99C, 0x7026B4, 0x5F7E41,
    0x3991D6, 0x398353, 0x39F49C, 0x845F8B, 0xBDF928, 0x3B1FF8,
    0x97FFDE, 0x05980F, 0xEF2F11, 0x8B5A0A, 0x6D1F6D, 0x367ECF,
    0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5,
    0xF17B3D, 0x0739F7, 0x8A5292, 0xEA6BFB, 0x5FB11F, 0x8D5D08,
    0x560330, 0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3,
    0x91615E, 0xE61B08, 0x659985, 0x5F14A0, 0x68408D, 0xFFD880,
    0x4D7327, 0x310606, 0x1556CA, 0x73A8C9, 0x60E27B, 0xC08C6B,
};

/// π/2 as a sum of 24-bit pieces, each exact in a double.
constexpr double piOver2Pieces[] = {
    1.57079625129699707031e+00, /* 0x3FF921FB, 0x40000000 */
    7.54978941586159635335e-08, /* 0x3E74442D, 0x00000000 */
    5.39030252995776476554e-15, /* 0x3CF84698, 0x80000000 */
    3.28200341580791294123e-22, /* 0x3B78CC51, 0x60000000 */
    1.27065575308067607349e-29, /* 0x39F01B83, 0x80000000 */
    1.22933308981111328932e-36, /* 0x387A2520, 0x40000000 */
    2.73370053816464559624e-44, /* 0x36E38222, 0x80000000 */
    2.16741683877804819444e-51, /* 0x3569F31D, 0x00000000 */
};

constexpr double two24 = 1.67772160000000000000e+07;   /* 0x41700000, 0x00000000 */
constexpr double twoMinus24 = 5.96046447753906250000e-08; /* 0x3E700000, 0x00000000 */

/// Reduces a large `x` to `y[0] + y[1] = x − n·π/2` with |y| ≤ π/4 and returns n mod 8.
///
/// `x` is given as `nx` chunks of 24 bits, `x[0] + x[1]·2⁻²⁴ + …` times 2^`e0`. The product of
/// that with the stored bits of 2/π is formed exactly in chunks, the integer part (mod 8) is
/// peeled off, and the fraction is multiplied by π/2 again. If the fraction came out as zero
/// the computation is repeated with more bits of 2/π. Only the `prec == 1` case of fdlibm (two
/// doubles out, 4 chunks of 2/π to start with) is ported; it is the only one `rem_pio2` uses.
int kernelRemainderPio2(const double* x, double* y, int e0, int nx) {
    constexpr int initialTerms = 4;  // fdlibm's init_jk[1]
    int32_t iq[20];
    double f[20];
    double fq[20];
    double q[20];

    const int32_t jk = initialTerms;
    const int32_t jp = jk;

    // Terms of x and of 2/π that matter: 3 > q0.
    const int32_t jx = nx - 1;
    int32_t jv = (e0 - 3) / 24;
    if (jv < 0) jv = 0;
    int32_t q0 = e0 - 24 * (jv + 1);

    // f[0 ... jx+jk] is the slice of 2/π used, f[jx+jk] = twoOverPi[jv+jk].
    int32_t j = jv - jx;
    const int32_t m = jx + jk;
    for (int32_t i = 0; i <= m; i += 1, j += 1) f[i] = (j < 0) ? 0.0 : static_cast<double>(twoOverPi[j]);

    // q[0 ... jk] are the partial products of x and 2/π.
    for (int32_t i = 0; i <= jk; i += 1) {
        double sum = 0.0;
        for (j = 0; j <= jx; j += 1) sum += x[j] * f[jx + i - j];
        q[i] = sum;
    }

    int32_t jz = jk;
    int32_t ih = 0;
    int32_t n = 0;
    double z = 0.0;

recompute:
    // Distill q[] into 24-bit integers iq[], from the last (smallest) term up.
    {
        double top = q[jz];
        int32_t i = 0;
        for (j = jz; j > 0; i += 1, j -= 1) {
            double whole = static_cast<double>(static_cast<int32_t>(twoMinus24 * top));
            iq[i] = static_cast<int32_t>(top - two24 * whole);
            top = q[j - 1] + whole;
        }
        z = top;
    }

    // n: the integer part of the product, mod 8.
    z = z * powerOfTwo(q0);
    z -= 8.0 * std::floor(z * 0.125);
    n = static_cast<int32_t>(z);
    z -= static_cast<double>(n);
    ih = 0;
    if (q0 > 0) {  // iq[jz-1] is needed to find n
        int32_t i = iq[jz - 1] >> (24 - q0);
        n += i;
        iq[jz - 1] -= i << (24 - q0);
        ih = iq[jz - 1] >> (23 - q0);
    } else if (q0 == 0) {
        ih = iq[jz - 1] >> 23;
    } else if (z >= 0.5) {
        ih = 2;
    }

    if (ih > 0) {  // the fraction is above 0.5: use 1 − fraction and flip the sign
        n += 1;
        int32_t carry = 0;
        for (int32_t i = 0; i < jz; i += 1) {
            j = iq[i];
            if (carry == 0) {
                if (j != 0) {
                    carry = 1;
                    iq[i] = 0x1000000 - j;
                }
            } else {
                iq[i] = 0xffffff - j;
            }
        }
        if (q0 > 0) {
            switch (q0) {
                case 1: iq[jz - 1] &= 0x7fffff; break;
                case 2: iq[jz - 1] &= 0x3fffff; break;
            }
        }
        if (ih == 2) {
            z = 1.0 - z;
            if (carry != 0) z -= powerOfTwo(q0);
        }
    }

    // The fraction came out as zero: more bits of 2/π are needed.
    if (z == 0.0) {
        j = 0;
        for (int32_t i = jz - 1; i >= jk; i -= 1) j |= iq[i];
        if (j == 0) {
            int32_t terms = 1;
            while (iq[jk - terms] == 0) terms += 1;
            for (int32_t i = jz + 1; i <= jz + terms; i += 1) {
                f[jx + i] = static_cast<double>(twoOverPi[jv + i]);
                double sum = 0.0;
                for (j = 0; j <= jx; j += 1) sum += x[j] * f[jx + i - j];
                q[i] = sum;
            }
            jz += terms;
            goto recompute;
        }
    }

    // Chop off zero terms, or break z into 24-bit chunks if it is too big for one.
    if (z == 0.0) {
        jz -= 1;
        q0 -= 24;
        while (iq[jz] == 0) {
            jz -= 1;
            q0 -= 24;
        }
    } else {
        z = z * powerOfTwo(-q0);
        if (z >= two24) {
            double whole = static_cast<double>(static_cast<int32_t>(twoMinus24 * z));
            iq[jz] = static_cast<int32_t>(z - two24 * whole);
            jz += 1;
            q0 += 24;
            iq[jz] = static_cast<int32_t>(whole);
        } else {
            iq[jz] = static_cast<int32_t>(z);
        }
    }

    // Convert the integer chunks back to floating point.
    double weight = powerOfTwo(q0);
    for (int32_t i = jz; i >= 0; i -= 1) {
        q[i] = weight * static_cast<double>(iq[i]);
        weight *= twoMinus24;
    }

    // Multiply by π/2 in pieces: fq[jz-i] = Σ piOver2Pieces[k]·q[i+k].
    for (int32_t i = jz; i >= 0; i -= 1) {
        double sum = 0.0;
        for (int32_t k = 0; k <= jp && k <= jz - i; k += 1) sum += piOver2Pieces[k] * q[i + k];
        fq[jz - i] = sum;
    }

    // Compress fq[] into the two doubles y[0] + y[1].
    double sum = 0.0;
    for (int32_t i = jz; i >= 0; i -= 1) sum += fq[i];
    y[0] = (ih == 0) ? sum : -sum;
    sum = fq[0] - sum;
    for (int32_t i = 1; i <= jz; i += 1) sum += fq[i];
    y[1] = (ih == 0) ? sum : -sum;
    return n & 7;
}

constexpr double invpio2 = 6.36619772367581382433e-01; /* 0x3FE45F30, 0x6DC9C883 */
constexpr double pio2_1 = 1.57079632673412561417e+00;  /* 0x3FF921FB, 0x54400000 */
constexpr double pio2_1t = 6.07710050650619224932e-11; /* 0x3DD0B461, 0x1A626331 */
constexpr double pio2_2 = 6.07710050630396597660e-11;  /* 0x3DD0B461, 0x1A600000 */
constexpr double pio2_2t = 2.02226624879595063154e-21; /* 0x3BA3198A, 0x2E037073 */
constexpr double pio2_3 = 2.02226624871116645580e-21;  /* 0x3BA3198A, 0x2E000000 */
constexpr double pio2_3t = 8.47842766036889956997e-32; /* 0x397B839A, 0x252049C1 */

/// Splits finite `x` with |x| > π/4 into `y[0] + y[1] = x − n·π/2` with |y[0] + y[1]| ≤ π/4 and
/// returns n (its low two bits are all the callers use). `y[1]` is the rounding tail of `y[0]`,
/// which keeps about 118 correct bits of the difference.
///
/// Up to 5π/4 and 9π/4 it subtracts a multiple of a two-piece π/2; up to about 2²⁰·π/2 it
/// rounds `x·2/π` to the nearest integer n and subtracts n·(π/2) in as many pieces as the
/// cancellation needs; beyond that, `kernelRemainderPio2` works from the bits of 2/π.
int32_t remainderPio2(double x, double* y) {
    const int32_t hx = static_cast<int32_t>(highWord(x));
    const int32_t ix = hx & 0x7fffffff;
    bool useMediumCase = false;

    if (ix <= 0x400f6a7a) {                        // |x| ~<= 5π/4
        if ((ix & 0xfffff) == 0x921fb) {           // |x| ~= π/2 or π: cancellation, use the medium case
            useMediumCase = true;
        } else if (ix <= 0x4002d97c) {             // |x| ~<= 3π/4
            if (hx > 0) {
                double z = x - pio2_1;
                y[0] = z - pio2_1t;
                y[1] = (z - y[0]) - pio2_1t;
                return 1;
            }
            double z = x + pio2_1;
            y[0] = z + pio2_1t;
            y[1] = (z - y[0]) + pio2_1t;
            return -1;
        } else {
            if (hx > 0) {
                double z = x - 2 * pio2_1;
                y[0] = z - 2 * pio2_1t;
                y[1] = (z - y[0]) - 2 * pio2_1t;
                return 2;
            }
            double z = x + 2 * pio2_1;
            y[0] = z + 2 * pio2_1t;
            y[1] = (z - y[0]) + 2 * pio2_1t;
            return -2;
        }
    } else if (ix <= 0x401c463b) {                 // |x| ~<= 9π/4
        if (ix <= 0x4015fdbc) {                    // |x| ~<= 7π/4
            if (ix == 0x4012d97c) {                // |x| ~= 3π/2
                useMediumCase = true;
            } else if (hx > 0) {
                double z = x - 3 * pio2_1;
                y[0] = z - 3 * pio2_1t;
                y[1] = (z - y[0]) - 3 * pio2_1t;
                return 3;
            } else {
                double z = x + 3 * pio2_1;
                y[0] = z + 3 * pio2_1t;
                y[1] = (z - y[0]) + 3 * pio2_1t;
                return -3;
            }
        } else {
            if (ix == 0x401921fb) {                // |x| ~= 2π
                useMediumCase = true;
            } else if (hx > 0) {
                double z = x - 4 * pio2_1;
                y[0] = z - 4 * pio2_1t;
                y[1] = (z - y[0]) - 4 * pio2_1t;
                return 4;
            } else {
                double z = x + 4 * pio2_1;
                y[0] = z + 4 * pio2_1t;
                y[1] = (z - y[0]) + 4 * pio2_1t;
                return -4;
            }
        }
    }

    if (useMediumCase || ix < 0x413921fb) {        // |x| ~< 2²⁰·(π/2), medium size
        // Round x·2/π to the nearest integer (ties to even) with the 1.5·2⁵² trick.
        const double product = x * invpio2;
        const double fn = (product + 0x1.8p52) - 0x1.8p52;
        const int32_t n = static_cast<int32_t>(fn);
        double r = x - fn * pio2_1;
        double w = fn * pio2_1t;  // first round: good to 85 bits
        const int32_t j = ix >> 20;
        y[0] = r - w;
        int32_t i = j - static_cast<int32_t>((highWord(y[0]) >> 20) & 0x7ff);
        if (i > 16) {  // second iteration, good to 118 bits
            double t = r;
            w = fn * pio2_2;
            r = t - w;
            w = fn * pio2_2t - ((t - r) - w);
            y[0] = r - w;
            i = j - static_cast<int32_t>((highWord(y[0]) >> 20) & 0x7ff);
            if (i > 49) {  // third iteration, 151 bits: covers every possible case
                t = r;
                w = fn * pio2_3;
                r = t - w;
                w = fn * pio2_3t - ((t - r) - w);
                y[0] = r - w;
            }
        }
        y[1] = (r - y[0]) - w;
        return n;
    }

    // Infinity and NaN.
    if (ix >= 0x7ff00000) {
        y[0] = y[1] = quietNaN();
        return 0;
    }

    // All other (large) arguments: scale |x| to z = |x|·2^−e0 in [2²³, 2²⁴), cut it into three
    // 24-bit chunks, and let the kernel multiply them by the bits of 2/π.
    const int32_t e0 = (ix >> 20) - 1046;
    double z = fromWords(static_cast<uint32_t>(ix - static_cast<int32_t>(static_cast<uint32_t>(e0) << 20)), lowWord(x));
    double chunks[3];
    for (int i = 0; i < 2; i += 1) {
        chunks[i] = static_cast<double>(static_cast<int32_t>(z));
        z = (z - chunks[i]) * two24;
    }
    chunks[2] = z;
    int32_t count = 3;
    while (chunks[count - 1] == 0.0) count -= 1;  // skip zero terms
    double tail[2];
    const int32_t n = kernelRemainderPio2(chunks, tail, e0, count);
    if (hx < 0) {
        y[0] = -tail[0];
        y[1] = -tail[1];
        return -n;
    }
    y[0] = tail[0];
    y[1] = tail[1];
    return n;
}

// ---------------------------------------------------------------------------------------------
// Kernels on [−π/4, π/4] (fdlibm 5.3 k_sin.c, k_cos.c, k_tan.c).
// ---------------------------------------------------------------------------------------------

constexpr double S1 = -1.66666666666666324348e-01; /* 0xBFC55555, 0x55555549 */
constexpr double S2 = 8.33333333332248946124e-03;  /* 0x3F811111, 0x1110F8A6 */
constexpr double S3 = -1.98412698298579493134e-04; /* 0xBF2A01A0, 0x19C161D5 */
constexpr double S4 = 2.75573137070700676789e-06;  /* 0x3EC71DE3, 0x57B1FE7D */
constexpr double S5 = -2.50507602534068634195e-08; /* 0xBE5AE5E6, 0x8A2B9CEB */
constexpr double S6 = 1.58969099521155010221e-10;  /* 0x3DE5D93A, 0x5ACFD57C */

/// sin(x + tail) for |x| ≤ π/4, where `tail` is the part of the argument that did not fit in `x`
/// (`hasTail` false means it is zero). A polynomial `x + S1·x³ + … + S6·x¹³`; with a tail,
/// sin(x + y) = sin(x) + cos(x)·y ≈ sin(x) + (1 − x²/2)·y. Not for -0 (callers return it first).
double kernelSin(double x, double tail, bool hasTail) {
    const double z = x * x;
    const double w = z * z;
    const double r = S2 + z * (S3 + z * S4) + z * w * (S5 + z * S6);
    const double v = z * x;
    if (!hasTail) return x + v * (S1 + z * r);
    return x - ((z * (0.5 * tail - v * r) - tail) - v * S1);
}

constexpr double C1 = 4.16666666666666019037e-02;  /* 0x3FA55555, 0x5555554C */
constexpr double C2 = -1.38888888888741095749e-03; /* 0xBF56C16C, 0x16C15177 */
constexpr double C3 = 2.48015872894767294178e-05;  /* 0x3EFA01A0, 0x19CB1590 */
constexpr double C4 = -2.75573143513906633035e-07; /* 0xBE927E4F, 0x809C52AD */
constexpr double C5 = 2.08757232129817482790e-09;  /* 0x3E21EE9E, 0xBDB4B1C4 */
constexpr double C6 = -1.13596475577881948265e-11; /* 0xBDA8FAE9, 0xBE8838D4 */

/// cos(x + tail) for |x| ≤ π/4: 1 − x²/2 + (C1·x⁴ + … + C6·x¹⁴), with 1 − x²/2 split into
/// `w + correction` so the rounding of `w` is recovered, and −x·tail for the tail.
double kernelCos(double x, double tail) {
    const double z = x * x;
    double w = z * z;
    const double r = z * (C1 + z * (C2 + z * C3)) + w * w * (C4 + z * (C5 + z * C6));
    const double halfSquare = 0.5 * z;
    w = 1.0 - halfSquare;
    return w + (((1.0 - w) - halfSquare) + (z * r - x * tail));
}

constexpr double T[] = {
    3.33333333333334091986e-01, /* 3FD55555, 55555563 */
    1.33333333333201242699e-01, /* 3FC11111, 1110FE7A */
    5.39682539762260521377e-02, /* 3FABA1BA, 1BB341FE */
    2.18694882948595424599e-02, /* 3F9664F4, 8406D637 */
    8.86323982359930005737e-03, /* 3F8226E3, E96E8493 */
    3.59207910759131235356e-03, /* 3F6D6D22, C9560328 */
    1.45620945432529025516e-03, /* 3F57DBC8, FEE08315 */
    5.88041240820264096874e-04, /* 3F4344D8, F2F26501 */
    2.46463134818469906812e-04, /* 3F3026F7, 1A8D1068 */
    7.81794442939557092300e-05, /* 3F147E88, A03792A6 */
    7.14072491382608190305e-05, /* 3F12B80F, 32F0A7E9 */
    -1.85586374855275456654e-05, /* BEF375CB, DB605373 */
    2.59073051863633712884e-05, /* 3EFB2A70, 74BF7AD4 */
};
constexpr double pio4 = 7.85398163397448278999e-01;   /* 3FE921FB, 54442D18 */
constexpr double pio4lo = 3.06161699786838301793e-17; /* 3C81A626, 33145C07 */

/// tan(x + tail) for |x| ≤ π/4 when `sign` is 1, and −1/tan(x + tail) when it is −1 (the cotangent
/// case after reduction by an odd multiple of π/2). A polynomial in x of degree 27; above
/// |x| = 0.6744 it uses tan(π/4 − t) = (1 − tan t)/(1 + tan t) with t = π/4 − |x|.
double kernelTan(double x, double tail, int sign) {
    const int32_t hx = static_cast<int32_t>(highWord(x));
    const int32_t ix = hx & 0x7fffffff;
    const bool nearQuarterPi = ix >= 0x3FE59428;  // |x| >= 0.6744
    if (nearQuarterPi) {
        if (hx < 0) {
            x = -x;
            tail = -tail;
        }
        const double z = pio4 - x;
        const double w = pio4lo - tail;
        x = z + w;
        tail = 0.0;
    }
    double z = x * x;
    double w = z * z;
    // x⁵·(T[1] + x⁴·T[3] + … + x²⁰·T[11]) and x⁵·x²·(T[2] + x⁴·T[4] + … + x²²·T[12]).
    double r = T[1] + w * (T[3] + w * (T[5] + w * (T[7] + w * (T[9] + w * T[11]))));
    double v = z * (T[2] + w * (T[4] + w * (T[6] + w * (T[8] + w * (T[10] + w * T[12])))));
    double s = z * x;
    r = tail + z * (s * (r + v) + tail);
    r += T[0] * s;
    w = x + r;
    if (nearQuarterPi) {
        v = static_cast<double>(sign);
        return static_cast<double>(1 - ((hx >> 30) & 2)) * (v - 2.0 * (x - (w * w / (w + v) - r)));
    }
    if (sign == 1) return w;
    // −1/(x + r), computed so the rounding of the division is corrected.
    z = withLowWordCleared(w);
    v = r - (z - x);  // z + v = r + x
    const double a = -1.0 / w;
    const double t = withLowWordCleared(a);
    s = 1.0 + t * z;
    return t + a * (s + t * v);
}

double sineOf(double x) {
    const int32_t ix = static_cast<int32_t>(highWord(x) & 0x7fffffff);
    if (ix <= 0x3fe921fb) {  // |x| ~<= π/4
        if (ix < 0x3e500000) return x;  // |x| < 2⁻²⁶: sin(x) = x, and keeps -0
        return kernelSin(x, 0.0, false);
    }
    if (ix >= 0x7ff00000) return quietNaN();
    double y[2];
    const int32_t n = remainderPio2(x, y);
    switch (n & 3) {
        case 0: return kernelSin(y[0], y[1], true);
        case 1: return kernelCos(y[0], y[1]);
        case 2: return -kernelSin(y[0], y[1], true);
        default: return -kernelCos(y[0], y[1]);
    }
}

double cosineOf(double x) {
    const int32_t ix = static_cast<int32_t>(highWord(x) & 0x7fffffff);
    if (ix <= 0x3fe921fb) {  // |x| ~<= π/4
        if (ix < 0x3e46a09e) return 1.0;  // |x| < 2⁻²⁷·√2: cos(x) rounds to 1
        return kernelCos(x, 0.0);
    }
    if (ix >= 0x7ff00000) return quietNaN();
    double y[2];
    const int32_t n = remainderPio2(x, y);
    switch (n & 3) {
        case 0: return kernelCos(y[0], y[1]);
        case 1: return -kernelSin(y[0], y[1], true);
        case 2: return -kernelCos(y[0], y[1]);
        default: return kernelSin(y[0], y[1], true);
    }
}

double tangentOf(double x) {
    const int32_t ix = static_cast<int32_t>(highWord(x) & 0x7fffffff);
    if (ix <= 0x3fe921fb) {  // |x| ~<= π/4
        if (ix < 0x3e400000) return x;  // |x| < 2⁻²⁷: tan(x) = x
        return kernelTan(x, 0.0, 1);
    }
    if (ix >= 0x7ff00000) return quietNaN();
    double y[2];
    const int32_t n = remainderPio2(x, y);
    return kernelTan(y[0], y[1], 1 - ((n & 1) << 1));  // 1 for even n, −1 for odd n
}

// ---------------------------------------------------------------------------------------------
// asin and acos (fdlibm 5.3 e_asin.c, e_acos.c).
// ---------------------------------------------------------------------------------------------

constexpr double pi = 3.14159265358979311600e+00;      /* 0x400921FB, 0x54442D18 */
constexpr double pio2_hi = 1.57079632679489655800e+00; /* 0x3FF921FB, 0x54442D18 */
constexpr double pio2_lo = 6.12323399573676603587e-17; /* 0x3C91A626, 0x33145C07 */
constexpr double pio4_hi = 7.85398163397448278999e-01; /* 0x3FE921FB, 0x54442D18 */
// Coefficients of the rational approximation R(x²) = (asin(x) − x)/x³.
constexpr double pS0 = 1.66666666666666657415e-01;  /* 0x3FC55555, 0x55555555 */
constexpr double pS1 = -3.25565818622400915405e-01; /* 0xBFD4D612, 0x03EB6F7D */
constexpr double pS2 = 2.01212532134862925881e-01;  /* 0x3FC9C155, 0x0E884455 */
constexpr double pS3 = -4.00555345006794114027e-02; /* 0xBFA48228, 0xB5688F3B */
constexpr double pS4 = 7.91534994289814532176e-04;  /* 0x3F49EFE0, 0x7501B288 */
constexpr double pS5 = 3.47933107596021167570e-05;  /* 0x3F023DE1, 0x0DFDF709 */
constexpr double qS1 = -2.40339491173441421878e+00; /* 0xC0033A27, 0x1C8A2D4B */
constexpr double qS2 = 2.02094576023350569471e+00;  /* 0x40002AE5, 0x9C598AC8 */
constexpr double qS3 = -6.88283971605453293030e-01; /* 0xBFE6066C, 0x1B8D0159 */
constexpr double qS4 = 7.70381505559019352791e-02;  /* 0x3FB3B8C5, 0xB12E9282 */

double arcsineOf(double x) {
    const int32_t hx = static_cast<int32_t>(highWord(x));
    const int32_t ix = hx & 0x7fffffff;
    if (ix >= 0x3ff00000) {  // |x| >= 1
        if (ix == 0x3ff00000 && lowWord(x) == 0) return x * pio2_hi + x * pio2_lo;  // ±π/2
        return quietNaN();                                                          // |x| > 1 or NaN
    }
    if (ix < 0x3fe00000) {  // |x| < 0.5
        if (ix < 0x3e500000) return x;  // |x| < 2⁻²⁶
        const double t = x * x;
        const double p = t * (pS0 + t * (pS1 + t * (pS2 + t * (pS3 + t * (pS4 + t * pS5)))));
        const double q = 1.0 + t * (qS1 + t * (qS2 + t * (qS3 + t * qS4)));
        const double w = p / q;
        return x + x * w;
    }
    // 1 > |x| >= 0.5
    const double halfDistance = (1.0 - magnitude(x)) * 0.5;
    const double p = halfDistance * (pS0 + halfDistance * (pS1 + halfDistance * (pS2 + halfDistance * (pS3 + halfDistance * (pS4 + halfDistance * pS5)))));
    const double q = 1.0 + halfDistance * (qS1 + halfDistance * (qS2 + halfDistance * (qS3 + halfDistance * qS4)));
    const double s = std::sqrt(halfDistance);
    double result;
    if (ix >= 0x3FEF3333) {  // |x| > 0.975
        const double w = p / q;
        result = pio2_hi - (2.0 * (s + s * w) - pio2_lo);
    } else {
        const double head = withLowWordCleared(s);
        const double c = (halfDistance - head * head) / (s + head);
        const double r = p / q;
        const double correction = 2.0 * s * r - (pio2_lo - 2.0 * c);
        const double q2 = pio4_hi - 2.0 * head;
        result = pio4_hi - (correction - q2);
    }
    return hx > 0 ? result : -result;
}

double arccosineOf(double x) {
    const int32_t hx = static_cast<int32_t>(highWord(x));
    const int32_t ix = hx & 0x7fffffff;
    if (ix >= 0x3ff00000) {  // |x| >= 1
        if (ix == 0x3ff00000 && lowWord(x) == 0) {  // |x| == 1
            if (hx > 0) return 0.0;                 // acos(1) = 0
            return pi + 2.0 * pio2_lo;              // acos(-1) = π
        }
        return quietNaN();  // |x| > 1 or NaN
    }
    if (ix < 0x3fe00000) {  // |x| < 0.5
        if (ix <= 0x3c600000) return pio2_hi + pio2_lo;  // |x| < 2⁻⁵⁷
        const double z = x * x;
        const double p = z * (pS0 + z * (pS1 + z * (pS2 + z * (pS3 + z * (pS4 + z * pS5)))));
        const double q = 1.0 + z * (qS1 + z * (qS2 + z * (qS3 + z * qS4)));
        const double r = p / q;
        return pio2_hi - (x - (pio2_lo - x * r));
    }
    if (hx < 0) {  // x < -0.5
        const double z = (1.0 + x) * 0.5;
        const double p = z * (pS0 + z * (pS1 + z * (pS2 + z * (pS3 + z * (pS4 + z * pS5)))));
        const double q = 1.0 + z * (qS1 + z * (qS2 + z * (qS3 + z * qS4)));
        const double s = std::sqrt(z);
        const double r = p / q;
        const double w = r * s - pio2_lo;
        return pi - 2.0 * (s + w);
    }
    // x > 0.5
    const double z = (1.0 - x) * 0.5;
    const double s = std::sqrt(z);
    const double head = withLowWordCleared(s);
    const double c = (z - head * head) / (s + head);
    const double p = z * (pS0 + z * (pS1 + z * (pS2 + z * (pS3 + z * (pS4 + z * pS5)))));
    const double q = 1.0 + z * (qS1 + z * (qS2 + z * (qS3 + z * qS4)));
    const double r = p / q;
    const double w = r * s + c;
    return 2.0 * (head + w);
}

// ---------------------------------------------------------------------------------------------
// atan and atan2 (fdlibm 5.3 s_atan.c, e_atan2.c).
// ---------------------------------------------------------------------------------------------

constexpr double atanhi[] = {
    4.63647609000806093515e-01, /* atan(0.5)hi 0x3FDDAC67, 0x0561BB4F */
    7.85398163397448278999e-01, /* atan(1.0)hi 0x3FE921FB, 0x54442D18 */
    9.82793723247329054082e-01, /* atan(1.5)hi 0x3FEF730B, 0xD281F69B */
    1.57079632679489655800e+00, /* atan(inf)hi 0x3FF921FB, 0x54442D18 */
};

constexpr double atanlo[] = {
    2.26987774529616870924e-17, /* atan(0.5)lo 0x3C7A2B7F, 0x222F65E2 */
    3.06161699786838301793e-17, /* atan(1.0)lo 0x3C81A626, 0x33145C07 */
    1.39033110312309984516e-17, /* atan(1.5)lo 0x3C700788, 0x7AF0CBBD */
    6.12323399573676603587e-17, /* atan(inf)lo 0x3C91A626, 0x33145C07 */
};

constexpr double aT[] = {
    3.33333333333329318027e-01,  /* 0x3FD55555, 0x5555550D */
    -1.99999999998764832476e-01, /* 0xBFC99999, 0x9998EBC4 */
    1.42857142725034663711e-01,  /* 0x3FC24924, 0x920083FF */
    -1.11111104054623557880e-01, /* 0xBFBC71C6, 0xFE231671 */
    9.09088713343650656196e-02,  /* 0x3FB745CD, 0xC54C206E */
    -7.69187620504482999495e-02, /* 0xBFB3B0F2, 0xAF749A6D */
    6.66107313738753120669e-02,  /* 0x3FB10D66, 0xA0D03D51 */
    -5.83357013379057348645e-02, /* 0xBFADDE2D, 0x52DEFD9A */
    4.97687799461593236017e-02,  /* 0x3FA97B4B, 0x24760DEB */
    -3.65315727442169155270e-02, /* 0xBFA2B444, 0x2C6A6C2F */
    1.62858201153657823623e-02,  /* 0x3F90AD3A, 0xE322DA11 */
};

double arctangentOf(double x) {
    const int32_t hx = static_cast<int32_t>(highWord(x));
    const int32_t ix = hx & 0x7fffffff;
    int32_t id;
    if (ix >= 0x44100000) {  // |x| >= 2⁶⁶
        if (ix > 0x7ff00000 || (ix == 0x7ff00000 && lowWord(x) != 0)) return quietNaN();
        return hx > 0 ? atanhi[3] + atanlo[3] : -atanhi[3] - atanlo[3];
    }
    if (ix < 0x3fdc0000) {  // |x| < 0.4375
        if (ix < 0x3e400000) return x;  // |x| < 2⁻²⁷
        id = -1;
    } else {
        x = magnitude(x);
        if (ix < 0x3ff30000) {  // |x| < 1.1875
            if (ix < 0x3fe60000) {  // 7/16 <= |x| < 11/16
                id = 0;
                x = (2.0 * x - 1.0) / (2.0 + x);
            } else {  // 11/16 <= |x| < 19/16
                id = 1;
                x = (x - 1.0) / (x + 1.0);
            }
        } else if (ix < 0x40038000) {  // |x| < 2.4375
            id = 2;
            x = (x - 1.5) / (1.0 + 1.5 * x);
        } else {  // 2.4375 <= |x| < 2⁶⁶
            id = 3;
            x = -1.0 / x;
        }
    }
    // The odd and even parts of Σ aT[i]·z^(i+1), i = 0 ... 10.
    const double z = x * x;
    const double w = z * z;
    const double s1 = z * (aT[0] + w * (aT[2] + w * (aT[4] + w * (aT[6] + w * (aT[8] + w * aT[10])))));
    const double s2 = w * (aT[1] + w * (aT[3] + w * (aT[5] + w * (aT[7] + w * aT[9]))));
    if (id < 0) return x - x * (s1 + s2);
    const double result = atanhi[id] - ((x * (s1 + s2) - atanlo[id]) - x);
    return hx < 0 ? -result : result;
}

constexpr double pi_o_4 = 7.8539816339744827900E-01; /* 0x3FE921FB, 0x54442D18 */
constexpr double pi_o_2 = 1.5707963267948965580E+00; /* 0x3FF921FB, 0x54442D18 */
constexpr double pi_lo = 1.2246467991473531772E-16;  /* 0x3CA1A626, 0x33145C07 */

double arctangent2Of(double y, double x) {
    const int32_t hx = static_cast<int32_t>(highWord(x));
    const uint32_t lx = lowWord(x);
    const int32_t ix = hx & 0x7fffffff;
    const int32_t hy = static_cast<int32_t>(highWord(y));
    const uint32_t ly = lowWord(y);
    const int32_t iy = hy & 0x7fffffff;
    if (x != x || y != y) return quietNaN();
    if (hx == 0x3ff00000 && lx == 0) return arctangentOf(y);  // x = 1.0
    int32_t quadrant = ((hy >> 31) & 1) | ((hx >> 30) & 2);   // 2·sign(x) + sign(y)

    if ((static_cast<uint32_t>(iy) | ly) == 0) {  // y = ±0
        switch (quadrant) {
            case 0:
            case 1: return y;       // atan(±0, +anything) = ±0
            case 2: return pi;      // atan(+0, −anything) = π
            default: return -pi;    // atan(−0, −anything) = −π
        }
    }
    if ((static_cast<uint32_t>(ix) | lx) == 0) return hy < 0 ? -pi_o_2 : pi_o_2;  // x = ±0

    if (ix == 0x7ff00000) {  // x is ±∞
        if (iy == 0x7ff00000) {
            switch (quadrant) {
                case 0: return pi_o_4;
                case 1: return -pi_o_4;
                case 2: return 3.0 * pi_o_4;
                default: return -3.0 * pi_o_4;
            }
        }
        switch (quadrant) {
            case 0: return 0.0;
            case 1: return -0.0;
            case 2: return pi;
            default: return -pi;
        }
    }
    if (iy == 0x7ff00000) return hy < 0 ? -pi_o_2 : pi_o_2;  // y is ±∞

    // y/x is safe to compute unless the exponents are very far apart.
    const int32_t exponentGap = (iy - ix) >> 20;
    double z;
    if (exponentGap > 60) {  // |y/x| > 2⁶⁰
        z = pi_o_2 + 0.5 * pi_lo;
        quadrant &= 1;
    } else if (hx < 0 && exponentGap < -60) {  // 0 > |y|/x > −2⁻⁶⁰
        z = 0.0;
    } else {
        z = arctangentOf(magnitude(y / x));
    }
    switch (quadrant) {
        case 0: return z;                  // atan(+, +)
        case 1: return -z;                 // atan(−, +)
        case 2: return pi - (z - pi_lo);   // atan(+, −)
        default: return (z - pi_lo) - pi;  // atan(−, −)
    }
}

// ---------------------------------------------------------------------------------------------
// exp, expm1, sinh (fdlibm 5.3 e_exp.c, s_expm1.c, e_sinh.c; k_exp.c from FreeBSD).
// ---------------------------------------------------------------------------------------------

constexpr double o_threshold = 7.09782712893383973096e+02;  /* 0x40862E42, 0xFEFA39EF */
constexpr double u_threshold = -7.45133219101941108420e+02; /* 0xc0874910, 0xD52D3051 */
constexpr double ln2HI[2] = {6.93147180369123816490e-01,  /* 0x3fe62e42, 0xfee00000 */
                             -6.93147180369123816490e-01}; /* 0xbfe62e42, 0xfee00000 */
constexpr double ln2LO[2] = {1.90821492927058770002e-10,  /* 0x3dea39ef, 0x35793c76 */
                             -1.90821492927058770002e-10}; /* 0xbdea39ef, 0x35793c76 */
constexpr double halF[2] = {0.5, -0.5};
constexpr double invln2 = 1.44269504088896338700e+00; /* 0x3ff71547, 0x652b82fe */
constexpr double P1 = 1.66666666666666019037e-01;     /* 0x3FC55555, 0x5555553E */
constexpr double P2 = -2.77777777770155933842e-03;    /* 0xBF66C16C, 0x16BEBD93 */
constexpr double P3 = 6.61375632143793436117e-05;     /* 0x3F11566A, 0xAF25DE2C */
constexpr double P4 = -1.65339022054652515390e-06;    /* 0xBEBBBD41, 0xC5D26BF1 */
constexpr double P5 = 4.13813679705723846039e-08;     /* 0x3E663769, 0x72BEA4D0 */
constexpr double twom1000 = 9.33263618503218878990e-302; /* 2**-1000 = 0x01700000, 0 */

double exponentialOf(double x) {
    double hi = 0.0;
    double lo = 0.0;
    int32_t k = 0;
    uint32_t hx = highWord(x);
    const int32_t xsb = static_cast<int32_t>((hx >> 31) & 1);  // sign bit of x
    hx &= 0x7fffffff;                                           // high word of |x|

    // Filter out non-finite arguments and those that overflow or underflow.
    if (hx >= 0x40862E42) {  // |x| >= 709.78...
        if (hx >= 0x7ff00000) {
            if (((hx & 0xfffff) | lowWord(x)) != 0) return quietNaN();
            return xsb == 0 ? x : 0.0;  // exp(±∞) = ∞, 0
        }
        if (x > o_threshold) return infinity();  // overflow
        if (x < u_threshold) return 0.0;         // underflow
    }

    // Argument reduction: x = k·ln2 + r with |r| <= 0.5·ln2.
    if (hx > 0x3fd62e42) {  // |x| > 0.5·ln2
        if (hx < 0x3FF0A2B2) {  // and |x| < 1.5·ln2
            hi = x - ln2HI[xsb];
            lo = ln2LO[xsb];
            k = 1 - xsb - xsb;
        } else {
            k = static_cast<int32_t>(invln2 * x + halF[xsb]);
            const double t = static_cast<double>(k);
            hi = x - t * ln2HI[0];  // t·ln2HI is exact here
            lo = t * ln2LO[0];
        }
        x = hi - lo;
    } else if (hx < 0x3e300000) {  // |x| < 2⁻²⁸
        return 1.0 + x;
    } else {
        k = 0;
    }

    // x is now in the primary range.
    const double t = x * x;
    double twopk;
    if (k >= -1021) {
        twopk = fromWords(static_cast<uint32_t>(0x3ff + k) << 20, 0);
    } else {
        twopk = fromWords(static_cast<uint32_t>(0x3ff + (k + 1000)) << 20, 0);
    }
    const double c = x - t * (P1 + t * (P2 + t * (P3 + t * (P4 + t * P5))));
    if (k == 0) return 1.0 - ((x * c) / (c - 2.0) - x);
    const double y = 1.0 - ((lo - (x * c) / (2.0 - c)) - hi);
    if (k >= -1021) {
        if (k == 1024) return y * 2.0 * 0x1p1023;
        return y * twopk;
    }
    return y * twopk * twom1000;
}

/// exp(x)·2^`exponent` for x between ln(DBL_MAX) and about 1454.9, without overflowing in the
/// middle: exp(x) = exp(x − 1246.97…)·2¹⁷⁹⁹, with the 2¹⁷⁹⁹ kept as an exponent (FreeBSD k_exp.c).
double exponentialTimesPowerOfTwo(double x, int exponent) {
    constexpr int32_t k = 1799;                              // constant for the reduction
    constexpr double kln2 = 1246.97177782734161156;          // k·ln2
    double scaled = exponentialOf(x - kln2);
    const uint32_t hx = highWord(scaled);
    const int32_t scaledExponent = static_cast<int32_t>(hx >> 20) - (0x3ff + 1023) + k;
    scaled = withHighWord(scaled, (hx & 0xfffff) | ((0x3ff + 1023) << 20));  // 2¹⁰²³ <= scaled < 2¹⁰²⁴
    exponent += scaledExponent;
    const double scale = fromWords(static_cast<uint32_t>(0x3ff + exponent) << 20, 0);
    return scaled * scale;
}

constexpr double ln2_hi = 6.93147180369123816490e-01; /* 0x3fe62e42, 0xfee00000 */
constexpr double ln2_lo = 1.90821492927058770002e-10; /* 0x3dea39ef, 0x35793c76 */
// Scaled Q's: Qn_here = 2**n * Qn_above, for R(2*z) where z = hxs = x*x/2.
constexpr double Q1 = -3.33333333333331316428e-02; /* BFA11111 111110F4 */
constexpr double Q2 = 1.58730158725481460165e-03;  /* 3F5A01A0 19FE5585 */
constexpr double Q3 = -7.93650757867487942473e-05; /* BF14CE19 9EAADBB7 */
constexpr double Q4 = 4.00821782732936239552e-06;  /* 3ED0CFCA 86E65239 */
constexpr double Q5 = -2.01099218183624371326e-07; /* BE8AFDB7 6E09C32D */

double exponentialMinusOneOf(double x) {
    double hi;
    double lo;
    double c = 0.0;
    int32_t k;
    uint32_t hx = highWord(x);
    const uint32_t xsb = hx & 0x80000000u;  // sign bit of x
    hx &= 0x7fffffff;                        // high word of |x|

    // Filter out huge and non-finite arguments.
    if (hx >= 0x4043687A) {  // |x| >= 56·ln2
        if (hx >= 0x40862E42) {  // |x| >= 709.78...
            if (hx >= 0x7ff00000) {
                if (((hx & 0xfffff) | lowWord(x)) != 0) return quietNaN();
                return xsb == 0 ? x : -1.0;  // expm1(±∞) = ∞, −1
            }
            if (x > o_threshold) return infinity();  // overflow
        }
        if (xsb != 0) return -1.0;  // x < −56·ln2: −1 up to rounding
    }

    // Argument reduction.
    if (hx > 0x3fd62e42) {  // |x| > 0.5·ln2
        if (hx < 0x3FF0A2B2) {  // and |x| < 1.5·ln2
            if (xsb == 0) {
                hi = x - ln2_hi;
                lo = ln2_lo;
                k = 1;
            } else {
                hi = x + ln2_hi;
                lo = -ln2_lo;
                k = -1;
            }
        } else {
            k = static_cast<int32_t>(invln2 * x + ((xsb == 0) ? 0.5 : -0.5));
            const double t = static_cast<double>(k);
            hi = x - t * ln2_hi;  // t·ln2_hi is exact here
            lo = t * ln2_lo;
        }
        x = hi - lo;
        c = (hi - x) - lo;
    } else if (hx < 0x3c900000) {  // |x| < 2⁻⁵⁴: expm1(x) = x
        return x;
    } else {
        k = 0;
    }

    // x is now in the primary range.
    const double hfx = 0.5 * x;
    const double hxs = x * hfx;
    const double r1 = 1.0 + hxs * (Q1 + hxs * (Q2 + hxs * (Q3 + hxs * (Q4 + hxs * Q5))));
    double t = 3.0 - r1 * hfx;
    double e = hxs * ((r1 - t) / (6.0 - x * t));
    if (k == 0) return x - (x * e - hxs);  // c is 0
    const double twopk = fromWords(static_cast<uint32_t>(0x3ff + k) << 20, 0);  // 2ᵏ (only used for k > 0 or k < −1)
    e = (x * (e - c) - c);
    e -= hxs;
    if (k == -1) return 0.5 * (x - e) - 0.5;
    if (k == 1) {
        if (x < -0.25) return -2.0 * (e - (x + 0.5));
        return 1.0 + 2.0 * (x - e);
    }
    double y;
    if (k <= -2 || k > 56) {  // enough to return exp(x) − 1
        y = 1.0 - (e - x);
        if (k == 1024) {
            y = y * 2.0 * 0x1p1023;
        } else {
            y = y * twopk;
        }
        return y - 1.0;
    }
    t = 1.0;
    if (k < 20) {
        t = withHighWord(t, 0x3ff00000 - (0x200000 >> k));  // t = 1 − 2⁻ᵏ
        y = t - (e - x);
        y = y * twopk;
    } else {
        t = withHighWord(t, static_cast<uint32_t>(0x3ff - k) << 20);  // t = 2⁻ᵏ
        y = x - (e + t);
        y += 1.0;
        y = y * twopk;
    }
    return y;
}

double hyperbolicSineOf(double x) {
    const int32_t jx = static_cast<int32_t>(highWord(x));
    const int32_t ix = jx & 0x7fffffff;

    if (ix >= 0x7ff00000) return x + x;  // ±∞ or NaN

    double h = 0.5;
    if (jx < 0) h = -h;
    // |x| < 22: sign(x)·0.5·(E + E/(E + 1)) with E = expm1(|x|).
    if (ix < 0x40360000) {
        if (ix < 0x3e300000) return x;  // |x| < 2⁻²⁸
        const double t = exponentialMinusOneOf(magnitude(x));
        if (ix < 0x3ff00000) return h * (2.0 * t - t * t / (t + 1.0));
        return h * (t + t / (t + 1.0));
    }
    // |x| in [22, log(maxdouble)]: 0.5·exp(|x|).
    if (ix < 0x40862E42) return h * exponentialOf(magnitude(x));
    // |x| in [log(maxdouble), overflow threshold]: 0.5·exp(|x|) computed as 2·exp(|x|)·2⁻¹.
    if (ix <= 0x408633CE) return h * 2.0 * exponentialTimesPowerOfTwo(magnitude(x), -1);
    // |x| > overflow threshold.
    return x * 1.0e307;
}

// ---------------------------------------------------------------------------------------------
// log and log1p (fdlibm 5.3 e_log.c, s_log1p.c).
// ---------------------------------------------------------------------------------------------

constexpr double two54 = 1.80143985094819840000e+16; /* 43500000 00000000 */
constexpr double Lg1 = 6.666666666666735130e-01;     /* 3FE55555 55555593 */
constexpr double Lg2 = 3.999999999940941908e-01;     /* 3FD99999 9997FA04 */
constexpr double Lg3 = 2.857142874366239149e-01;     /* 3FD24924 94229359 */
constexpr double Lg4 = 2.222219843214978396e-01;     /* 3FCC71C5 1D8E78AF */
constexpr double Lg5 = 1.818357216161805012e-01;     /* 3FC74664 96CB03DE */
constexpr double Lg6 = 1.531383769920937332e-01;     /* 3FC39A09 D078C69F */
constexpr double Lg7 = 1.479819860511658591e-01;     /* 3FC2F112 DF3E5244 */

double logarithmOf(double x) {
    int32_t hx = static_cast<int32_t>(highWord(x));
    const uint32_t lx = lowWord(x);

    int32_t k = 0;
    if (hx < 0x00100000) {  // x < 2⁻¹⁰²²
        if (((static_cast<uint32_t>(hx) & 0x7fffffff) | lx) == 0) return -infinity();  // log(±0) = −∞
        if (hx < 0) return quietNaN();                                                   // log(negative)
        k -= 54;
        x *= two54;  // subnormal number: scale up
        hx = static_cast<int32_t>(highWord(x));
    }
    if (hx >= 0x7ff00000) return x + x;
    k += (hx >> 20) - 1023;
    hx &= 0x000fffff;
    int32_t i = (hx + 0x95f64) & 0x100000;
    x = withHighWord(x, static_cast<uint32_t>(hx | (i ^ 0x3ff00000)));  // normalize x or x/2
    k += (i >> 20);
    const double f = x - 1.0;
    if ((0x000fffff & (2 + hx)) < 3) {  // −2⁻²⁰ <= f < 2⁻²⁰
        if (f == 0.0) {
            if (k == 0) return 0.0;
            const double dk = static_cast<double>(k);
            return dk * ln2_hi + dk * ln2_lo;
        }
        const double R = f * f * (0.5 - 0.33333333333333333 * f);
        if (k == 0) return f - R;
        const double dk = static_cast<double>(k);
        return dk * ln2_hi - ((R - dk * ln2_lo) - f);
    }
    const double s = f / (2.0 + f);
    const double dk = static_cast<double>(k);
    const double z = s * s;
    i = hx - 0x6147a;
    const double w = z * z;
    const int32_t j = 0x6b851 - hx;
    const double t1 = w * (Lg2 + w * (Lg4 + w * Lg6));
    const double t2 = z * (Lg1 + w * (Lg3 + w * (Lg5 + w * Lg7)));
    i |= j;
    const double R = t2 + t1;
    if (i > 0) {
        const double hfsq = 0.5 * f * f;
        if (k == 0) return f - (hfsq - s * (hfsq + R));
        return dk * ln2_hi - ((hfsq - (s * (hfsq + R) + dk * ln2_lo)) - f);
    }
    if (k == 0) return f - s * (f - R);
    return dk * ln2_hi - ((s * (f - R) - dk * ln2_lo) - f);
}

constexpr double Lp1 = 6.666666666666735130e-01; /* 3FE55555 55555593 */
constexpr double Lp2 = 3.999999999940941908e-01; /* 3FD99999 9997FA04 */
constexpr double Lp3 = 2.857142874366239149e-01; /* 3FD24924 94229359 */
constexpr double Lp4 = 2.222219843214978396e-01; /* 3FCC71C5 1D8E78AF */
constexpr double Lp5 = 1.818357216161805012e-01; /* 3FC74664 96CB03DE */
constexpr double Lp6 = 1.531383769920937332e-01; /* 3FC39A09 D078C69F */
constexpr double Lp7 = 1.479819860511658591e-01; /* 3FC2F112 DF3E5244 */

double logarithmOfOnePlus(double x) {
    const int32_t hx = static_cast<int32_t>(highWord(x));
    const int32_t ax = hx & 0x7fffffff;
    double f = 0.0;
    double c = 0.0;
    int32_t hu = 0;

    int32_t k = 1;
    if (hx < 0x3FDA827A) {  // 1 + x < √2 (about)
        if (ax >= 0x3ff00000) {  // x <= −1
            if (x == -1.0) return -infinity();  // log1p(−1) = −∞
            return quietNaN();                  // log1p(x < −1) = NaN
        }
        if (ax < 0x3e200000) {  // |x| < 2⁻²⁹
            if (ax < 0x3c900000) return x;  // |x| < 2⁻⁵⁴
            return x - x * x * 0.5;
        }
        if (hx > 0 || hx <= static_cast<int32_t>(0xbfd2bec4u)) {
            k = 0;  // √2/2 <= 1 + x < √2: no scaling
            f = x;
            hu = 1;
        }
    }
    if (hx >= 0x7ff00000) return x + x;
    if (k != 0) {
        double u;
        if (hx < 0x43400000) {
            u = 1.0 + x;
            hu = static_cast<int32_t>(highWord(u));
            k = (hu >> 20) - 1023;
            c = (k > 0) ? 1.0 - (u - x) : x - (u - 1.0);  // the rounding error of 1 + x
            c /= u;
        } else {
            u = x;
            hu = static_cast<int32_t>(highWord(u));
            k = (hu >> 20) - 1023;
            c = 0;
        }
        hu &= 0x000fffff;
        // The approximation of √2 in the thresholds above is looser than this one, so k == 0 is
        // never reached from here, where the correction term is committed to.
        if (hu < 0x6a09e) {  // u < √2
            u = withHighWord(u, static_cast<uint32_t>(hu | 0x3ff00000));  // normalize u
        } else {
            k += 1;
            u = withHighWord(u, static_cast<uint32_t>(hu | 0x3fe00000));  // normalize u/2
            hu = (0x00100000 - hu) >> 2;
        }
        f = u - 1.0;
    }
    const double hfsq = 0.5 * f * f;
    if (hu == 0) {  // |f| < 2⁻²⁰
        if (f == 0.0) {
            if (k == 0) return 0.0;
            c += k * ln2_lo;
            return k * ln2_hi + c;
        }
        const double R = hfsq * (1.0 - 0.66666666666666666 * f);
        if (k == 0) return f - R;
        return k * ln2_hi - ((R - (k * ln2_lo + c)) - f);
    }
    const double s = f / (2.0 + f);
    const double z = s * s;
    const double R = z * (Lp1 + z * (Lp2 + z * (Lp3 + z * (Lp4 + z * (Lp5 + z * (Lp6 + z * Lp7))))));
    if (k == 0) return f - (hfsq - s * (hfsq + R));
    return k * ln2_hi - ((hfsq - (s * (hfsq + R) + (k * ln2_lo + c))) - f);
}

// ---------------------------------------------------------------------------------------------
// hypot: our own (no fdlibm). √(a² + b² + c²) scaled by a power of two so nothing overflows.
// ---------------------------------------------------------------------------------------------

/// √(a² + b² + c²) for finite, non-negative `a`, `b`, `c`, not all zero.
///
/// The largest is scaled by the exact power of two that brings it into [1, 2); the others are
/// scaled by the same factor (they may lose their lowest bits to underflow, but then their
/// squares are below 2⁻¹⁰⁶ of the largest's and cannot change the sum). The squares are added
/// as `(a² + b²) + c²`, the root taken, and the scale undone with one multiplication, so the
/// only way to overflow is a result that does not fit. A subnormal largest is first lifted by 2²⁰⁰.
double rootOfSumOfSquares(double a, double b, double c) {
    double largest = a > b ? a : b;
    if (c > largest) largest = c;
    int lift = 0;
    if (largest < 0x1p-900) {  // subnormal or tiny: lift everything by 2²⁰⁰ (exact)
        a *= 0x1p200;
        b *= 0x1p200;
        c *= 0x1p200;
        largest *= 0x1p200;
        lift = -200;
    }
    const int exponent = static_cast<int>((std::bit_cast<uint64_t>(largest) >> 52) & 0x7ff) - 1023;
    const double shrink = powerOfTwo(-exponent);
    const double sa = a * shrink;
    const double sb = b * shrink;
    const double sc = c * shrink;
    const double root = std::sqrt((sa * sa + sb * sb) + sc * sc);
    return root * powerOfTwo(exponent + lift);
}

bool isNaNValue(double x) { return x != x; }
bool isInfiniteValue(double x) { return magnitude(x) == infinity(); }

}  // namespace

double sin(double x) { return withCanonicalNaN(sineOf(x)); }
double cos(double x) { return withCanonicalNaN(cosineOf(x)); }
double tan(double x) { return withCanonicalNaN(tangentOf(x)); }
double asin(double x) { return withCanonicalNaN(arcsineOf(x)); }
double acos(double x) { return withCanonicalNaN(arccosineOf(x)); }
double atan(double x) { return withCanonicalNaN(arctangentOf(x)); }
double atan2(double y, double x) { return withCanonicalNaN(arctangent2Of(y, x)); }
double exp(double x) { return withCanonicalNaN(exponentialOf(x)); }
double expm1(double x) { return withCanonicalNaN(exponentialMinusOneOf(x)); }
double log(double x) { return withCanonicalNaN(logarithmOf(x)); }
double log1p(double x) { return withCanonicalNaN(logarithmOfOnePlus(x)); }
double sinh(double x) { return withCanonicalNaN(hyperbolicSineOf(x)); }

double hypot(double x, double y) {
    const double a = magnitude(x);
    const double b = magnitude(y);
    if (isInfiniteValue(a) || isInfiniteValue(b)) return infinity();
    if (isNaNValue(a) || isNaNValue(b)) return quietNaN();
    if (a == 0.0 && b == 0.0) return 0.0;
    return rootOfSumOfSquares(a, b, 0.0);
}

double hypot(double x, double y, double z) {
    const double a = magnitude(x);
    const double b = magnitude(y);
    const double c = magnitude(z);
    if (isInfiniteValue(a) || isInfiniteValue(b) || isInfiniteValue(c)) return infinity();
    if (isNaNValue(a) || isNaNValue(b) || isNaNValue(c)) return quietNaN();
    if (a == 0.0 && b == 0.0 && c == 0.0) return 0.0;
    return rootOfSumOfSquares(a, b, c);
}

}  // namespace editor::deterministic
