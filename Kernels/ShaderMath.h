// The shader library's arithmetic, ported from graphics_metal's Core.metal and Math.metal: named
// constants, floored modulo, the "safe" operations that give 0 instead of NaN or infinity,
// ranges, easing, comparisons, waves, vector helpers that also take plain numbers, and debug
// colours. Most functions are written once for a number and for float2, float3 and float4: they
// use only operators and the prelude's and KernelVectors.h's functions, which work component by
// component, so `Value` below is `float` or a kernel vector of floats.
//
// Differences from graphics_metal, all on purpose:
// - Constants are functions (`pi<float>()`), not global variables. A Metal global must sit in the
//   `constant` address space, a word shared code does not write, and a function also works for any
//   number type `Real`.
// - Where Metal wrote `select(ifFalse, ifTrue, condition)` this writes `chosen(condition, ifTrue,
//   ifFalse)`, in reading order.
// - The functions take their constants as `Value(0.5f)`, a float literal converted to the type.
//   Value must therefore be built from a float (float and the kernel vectors of floats).
// - No kernel matrices exist, so `rotation2` became a function that rotates a Vector2Of.
// - `loopAccumulate` was not ported: it takes a function as an argument, which kernels forbid.
#pragma once

#include "KernelPrelude.h"
#include "KernelVectors.h"

namespace editor {
namespace kernels {

// ---------------------------------------------------------------------------------------------
// Constants

/// π, the ratio of a circle's circumference to its diameter: 3.14159265359.
///
/// How: a float literal converted to `Real`. One form for every constant here: a function,
/// because a shared kernel cannot name Metal's `constant` address space for a global variable.
///
/// Use when: converting angles, building waves and rotations
/// Tags: PI, 3.14159, half turn, M_PI, graphics_metal PI
/// Node: none (shader library)
template <typename Real>
inline Real pi() { return Real(3.14159265359f); }

/// τ = 2π, one full turn in radians: 6.28318530718.
///
/// Use when: the period of a sine wave; a full circle
/// Tags: TAU, two pi, full turn, graphics_metal TAU
/// Node: none (shader library)
template <typename Real>
inline Real tau() { return Real(6.28318530718f); }

/// Euler's number e, the base of the natural logarithm: 2.71828182846.
///
/// Use when: exponential curves written by hand
/// Tags: E, Euler number, natural base, graphics_metal E
/// Node: none (shader library)
template <typename Real>
inline Real eulerNumber() { return Real(2.71828182846f); }

/// √2, the diagonal of a unit square: 1.41421356237.
///
/// Use when: the length of a square's diagonal
/// Tags: sqrt2, root two, diagonal, graphics_metal namedConstant 3
/// Node: none (shader library)
template <typename Real>
inline Real squareRootOfTwo() { return Real(1.41421356237f); }

/// The golden ratio φ = (1 + √5) / 2: 1.61803398875.
///
/// Use when: spiral and spacing patterns such as sunflower seeds
/// Tags: phi, golden section, golden mean, graphics_metal namedConstant 4
/// Node: none (shader library)
template <typename Real>
inline Real goldenRatio() { return Real(1.61803398875f); }

/// One of five constants picked by number: 0 π, 1 τ, 2 e, 3 √2, anything else the golden ratio.
///
/// Use when: a constant chosen by a dropdown
/// Tags: namedConstant, constant by index, graphics_metal namedConstant
/// Node: none (shader library)
template <typename Real>
inline Real namedConstant(int which) {
    switch (which) {
        case 0: return pi<Real>();
        case 1: return tau<Real>();
        case 2: return eulerNumber<Real>();
        case 3: return squareRootOfTwo<Real>();
        default: return goldenRatio<Real>();
    }
}

/// The divisor size below which the "safe" functions treat a divisor as zero: 0.000001.
///
/// Use when: guarding a division in shader code
/// Tags: epsilon, tolerance, safe divisor threshold
/// Node: none (shader library)
template <typename Value>
inline Value smallestSafeDivisor() { return Value(0.000001f); }

/// The largest finite float, 3.40282347e38.
///
/// Use when: telling infinity from a large number
/// Tags: FLT_MAX, largest float, infinity threshold
/// Node: none (shader library)
template <typename Value>
inline Value largestFiniteFloat() { return Value(3.40282347e+38f); }

// ---------------------------------------------------------------------------------------------
// Arithmetic from Core.metal

/// `x` modulo `y` with the sign of the divisor, as GLSL's `mod`: the result is in 0…y for a
/// positive `y` (in y…0 for a negative one), also for a negative `x`.
///
/// How: x − y × ⌊x / y⌋. Examples: −1 mod 3 = −1 − 3×(−1) = 2; 7 mod −3 = 7 − (−3)(−3) = −2.
///
/// Conventions: `y` = 0 gives NaN. This is not C's `fmod`, which keeps the sign of `x`.
///
/// Use when: wrapping an angle or a coordinate into one period, including from below zero
/// Tags: glslMod, mod, modulo, floored modulo, wrap, fmod, Python %
/// Node: none (shader library)
template <typename Value>
inline Value flooredModulo(Value x, Value y) {
    return x - y * roundedDown(x / y);
}
/// The same, with one number as the divisor for every component of a vector `x`.
template <typename Value, typename Scalar>
inline Value flooredModulo(Value x, Scalar y) {
    return x - y * roundedDown(x / y);
}

/// An angle in degrees as radians.
///
/// How: degrees × (π / 180). 180° → π.
///
/// Use when: turning a slider in degrees into the angle a sine takes
/// Tags: toRadians, degrees to radians, deg2rad, radians
/// Node: none (shader library)
template <typename Real>
inline Real degreesToRadians(Real degrees) { return degrees * (pi<Real>() / Real(180.0f)); }

/// `numerator / denominator`, or 0 when the divisor is nearly 0.
///
/// How: 0 where |denominator| < 0.000001, else numerator / denominator.
///
/// Conventions: per component for vectors; the sign of a tiny divisor is ignored.
///
/// Use when: dividing by a value a graph could set to zero without making NaN or infinity
/// Tags: safeDivide, safe division, divide by zero, guarded divide
/// Node: none (shader library)
template <typename Value>
inline Value safeDivide(Value numerator, Value denominator) {
    return chosen(absolute(denominator) < smallestSafeDivisor<Value>(), Value(0.0f), numerator / denominator);
}

/// `value` modulo `divisor`, 0 when the divisor is nearly 0, in 0…divisor for a positive divisor.
///
/// How: flooredModulo(flooredModulo(value, divisor) + divisor, divisor), then 0 where |divisor| < 0.000001.
/// The inner modulo may leave the result at exactly the divisor's edge; the second pass is
/// graphics_metal's, kept so the results match.
///
/// Use when: wrapping a value by a divisor from a graph that may be zero
/// Tags: safeModulo, safe modulo, wrap, remainder
/// Node: none (shader library)
template <typename Value>
inline Value safeModulo(Value value, Value divisor) {
    Value result = flooredModulo(flooredModulo(value, divisor) + divisor, divisor);
    return chosen(absolute(divisor) < smallestSafeDivisor<Value>(), Value(0.0f), result);
}

/// How bright a colour looks: the weighted sum of its red, green and blue.
///
/// How: 0.2126 r + 0.7152 g + 0.0722 b (Rec. 709 / sRGB primaries). The alpha of a four-component colour is ignored.
///
/// Conventions: linear (not gamma-encoded) colour in, 0…1 out.
///
/// Use when: turning a colour to grey; checking contrast
/// Tags: luminance, brightness, grey, Rec.709, graphics_metal luminance
/// Node: none (shader library)
template <typename Real>
inline Real luminance(Vector3Of<Real> color) {
    return dotProduct(color, Vector3Of<Real>(Real(0.2126f), Real(0.7152f), Real(0.0722f)));
}
template <typename Real>
inline Real luminance(Vector4Of<Real> color) {
    return luminance(Vector3Of<Real>(color.x, color.y, color.z));
}

/// A point turned counter-clockwise about the origin.
///
/// How: (x cos a − y sin a, x sin a + y cos a), graphics_metal's `rotation2` matrix
/// [[c, −s], [s, c]] applied to the point. (1, 0) turned by π/2 gives (0, 1).
///
/// Conventions: radians; counter-clockwise seen with x to the right and y up.
///
/// Use when: rotating texture coordinates or a 2D offset
/// Tags: rotation2, rotate 2D, rotation matrix, turn point
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> rotatedCounterClockwise(Vector2Of<Real> point, Real radians) {
    Real cosineOfAngle = cosine(radians);
    Real sineOfAngle = sine(radians);
    return Vector2Of<Real>(point.x * cosineOfAngle - point.y * sineOfAngle, point.x * sineOfAngle + point.y * cosineOfAngle);
}

// ---------------------------------------------------------------------------------------------
// Ranges and smoothing from Math.metal

/// `value` moved linearly from the range `fromMin`…`fromMax` to `toMin`…`toMax`.
///
/// How: toMin + (value − fromMin) / (fromMax − fromMin) × (toMax − toMin). Not clamped. 5 from
/// 0…10 to 100…200 → 100 + 0.5 × 100 = 150. A source range narrower than 0.000001 gives `toMin`.
///
/// Use when: turning a slider's range into a size; Remap and Map Range nodes
/// Tags: mapRange, map range, remap, rescale, Blender Map Range, Unity Remap
/// Node: none (shader library)
template <typename Value>
inline Value remapped(Value value, Value fromMin, Value fromMax, Value toMin, Value toMax) {
    Value span = fromMax - fromMin;
    return chosen(absolute(span) < smallestSafeDivisor<Value>(), toMin, toMin + ((value - fromMin) / span) * (toMax - toMin));
}

/// `remapped`, then kept between `toMin` and `toMax` (whichever is smaller is the lower bound).
///
/// Use when: remapping without overshoot
/// Tags: mapRangeClamped, clamped remap, clamp map range
/// Node: none (shader library)
template <typename Value>
inline Value remappedClamped(Value value, Value fromMin, Value fromMax, Value toMin, Value toMax) {
    Value mapped = remapped(value, fromMin, fromMax, toMin, toMax);
    return clampedPerComponent(mapped, smallerOf(toMin, toMax), largerOf(toMin, toMax));
}

/// The smooth step between two edges: 0 below `edge0`, 1 above `edge1`, an S-curve between.
///
/// How: t = clamp((value − edge0) / (edge1 − edge0), 0, 1); result t² (3 − 2t). Halfway gives 0.5.
///
/// Conventions: equal edges (within 0.000001) make a hard step: 0 below `edge0`, else 1. GLSL's smoothstep is undefined there.
///
/// Use when: soft masks and transitions
/// Tags: safeSmoothstep, smoothstep, hermite, S curve, smooth threshold
/// Node: none (shader library)
template <typename Value>
inline Value safeSmoothStep(Value edge0, Value edge1, Value value) {
    Value span = edge1 - edge0;
    auto isTiny = absolute(span) < smallestSafeDivisor<Value>();
    Value divisor = chosen(isTiny, Value(1.0f), span);
    Value t = clampedPerComponent((value - edge0) / divisor, Value(0.0f), Value(1.0f));
    Value smoothed = t * t * (Value(3.0f) - Value(2.0f) * t);
    return chosen(isTiny, chosen(value < edge0, Value(0.0f), Value(1.0f)), smoothed);
}

/// Ken Perlin's smoother step: like `safeSmoothStep` with zero first and second slope at both ends.
///
/// How: t as in smoothstep; result t³ (t (6t − 15) + 10), Perlin 2002. A span under 0.000001 is replaced by 1 as the divisor (graphics_metal's rule), so no NaN appears.
///
/// Use when: noise interpolation and soft masks that must have no visible crease
/// Tags: smootherstep, quintic smoothstep, Perlin fade, improved smoothstep
/// Node: none (shader library)
template <typename Value>
inline Value smootherStep(Value edge0, Value edge1, Value value) {
    Value span = edge1 - edge0;
    Value divisor = chosen(absolute(span) < smallestSafeDivisor<Value>(), Value(1.0f), span);
    Value t = clampedPerComponent((value - edge0) / divisor, Value(0.0f), Value(1.0f));
    return t * t * t * (t * (t * Value(6.0f) - Value(15.0f)) + Value(10.0f));
}

/// How far `value` is from `start` to `end`, as a fraction: 0 at `start`, 1 at `end`.
///
/// How: (value − start) / (end − start), through `safeDivide`, so equal ends give 0. The inverse of `linearBlend`.
///
/// Use when: finding where a value lies in a range
/// Tags: inverseLerp, inverse lerp, unlerp, normalize range, invlerp
/// Node: none (shader library)
template <typename Value>
inline Value inverseLinearBlend(Value start, Value end, Value value) {
    return safeDivide(value - start, end - start);
}

/// `value` rounded to the nearest multiple of `step` (halves round up).
///
/// How: ⌊value / step + 0.5⌋ × step. 7 with step 5 → ⌊1.9⌋ × 5 = 5. A step under 0.000001 returns `value`.
///
/// Use when: snapping to a grid
/// Tags: snapTo, snap, round to multiple, quantize, grid snap
/// Node: none (shader library)
template <typename Value>
inline Value snappedToMultiple(Value value, Value step) {
    return chosen(absolute(step) < smallestSafeDivisor<Value>(), value, roundedDown(value / step + Value(0.5f)) * step);
}

/// A triangle wave: `value` goes up from 0 to `length`, then back down to 0, and repeats.
///
/// How: with l = max(|length|, 0.000001) and t = flooredModulo(value, 2l): l − |t − l|.
/// For l = 2: 0→0, 1→1, 2→2, 3→1, 4→0, and −1→1. Always in 0…l.
///
/// Use when: bouncing back and forth; mirrored repeat
/// Tags: pingPong, ping pong, triangle wave, mirror repeat, Mathf.PingPong
/// Node: none (shader library)
template <typename Value>
inline Value pingPong(Value value, Value length) {
    Value l = largerOf(absolute(length), smallestSafeDivisor<Value>());
    Value t = flooredModulo(value, l * Value(2.0f));
    return l - absolute(t - l);
}

/// `value` wrapped into the range `low`…`high`, repeating every `high − low`.
///
/// How: low + flooredModulo(value − low, high − low). 12 into 0…10 → 2; −1 → 9. A span under 0.000001 gives `low`.
///
/// Use when: wrapping an angle or a texture coordinate
/// Tags: wrapRange, wrap, repeat, loop, Mathf.Repeat
/// Node: none (shader library)
template <typename Value>
inline Value wrappedInto(Value value, Value low, Value high) {
    Value span = high - low;
    return chosen(absolute(span) < smallestSafeDivisor<Value>(), low, low + flooredModulo(value - low, span));
}

/// `value` rounded to the nearest whole number, halves rounding up (toward +∞): 2.5 → 3, −2.5 → −2.
///
/// How: ⌊value + 0.5⌋.
///
/// Use when: rounding the way a spreadsheet does, not to even
/// Tags: roundHalfUp, round, half up, rounding
/// Node: none (shader library)
template <typename Value>
inline Value roundedHalfUp(Value value) { return roundedDown(value + Value(0.5f)); }

/// 1 / `value`, or 0 when `value` is nearly 0.
///
/// How: safeDivide(1, value).
///
/// Use when: inverting a value a graph could set to zero
/// Tags: safeReciprocal, reciprocal, one over x, invert
/// Node: none (shader library)
template <typename Value>
inline Value safeReciprocal(Value value) { return safeDivide(Value(1.0f), value); }

/// The logarithm of `value` to `base`, with `value` first raised to 0.000001 so it is never ≤ 0.
///
/// How: ln(max(value, 0.000001)) / ln(base). log₂ 8 = 3.
///
/// Conventions: `base` must be positive and not 1; the caller checks.
///
/// Use when: undoing an exponential from a graph
/// Tags: safeLog, logarithm, log base, log2, log10
/// Node: none (shader library)
template <typename Value, typename Real>
inline Value safeLogarithm(Value value, Real base) {
    return naturalLogarithm(largerOf(value, smallestSafeDivisor<Value>())) / naturalLogarithm(base);
}

/// `base` raised to `exponent`, with a negative base treated as 0 so the result is never NaN.
///
/// How: pow(max(base, 0), exponent).
///
/// Use when: gamma and falloff curves on values that might dip below zero
/// Tags: safePower, safe pow, power, exponent
/// Node: none (shader library)
template <typename Value>
inline Value safePower(Value base, Value exponent) {
    return power(largerOf(base, Value(0.0f)), exponent);
}

/// The polynomial smooth minimum of Inigo Quilez: the smaller of `a` and `b`, rounded off within
/// `smoothness` of where they cross, so the union of two shapes has a soft crease.
///
/// How: r = max(smoothness, 0.000001); h = clamp(0.5 + 0.5 (b − a) / r, 0, 1); mix(b, a, h) − r h (1 − h).
/// With a = b = 1 and r = 1: h = 0.5, 1 − 1 × 0.5 × 0.5 = 0.75.
///
/// Use when: blending distance fields; soft union
/// Tags: smoothMin, smooth minimum, smin, polynomial smooth min, metaballs
/// Node: none (shader library)
template <typename Value, typename Real>
inline Value smoothMinimum(Value a, Value b, Real smoothness) {
    Real r = largerOf(smoothness, Real(0.000001f));
    Value h = clampedPerComponent(Real(0.5f) + Real(0.5f) * (b - a) / r, Value(0.0f), Value(1.0f));
    return linearBlendPerComponent(b, a, h) - r * h * (Real(1.0f) - h);
}

/// The smooth maximum: `smoothMinimum` mirrored, −smoothMinimum(−a, −b).
///
/// Use when: soft intersection of distance fields
/// Tags: smoothMax, smooth maximum, smax
/// Node: none (shader library)
template <typename Value, typename Real>
inline Value smoothMaximum(Value a, Value b, Real smoothness) {
    return -smoothMinimum(-a, -b, smoothness);
}

// ---------------------------------------------------------------------------------------------
// Comparison and logic: 1 for true, 0 for false, so they work on numbers in a graph.

/// 1 where a comparison holds, else 0. `mode`: 0 equal, 1 not equal, 2 less, 3 less or equal, 4
/// greater, anything else greater or equal. Equality means within `tolerance`.
///
/// Use when: a Compare node
/// Tags: compareValues, compare, equal within tolerance, comparison
/// Node: none (shader library)
template <typename Value>
inline Value comparisonResult(Value a, Value b, int mode, Value tolerance) {
    Value one = Value(1.0f);
    Value zero = Value(0.0f);
    switch (mode) {
        case 0: return chosen(absolute(a - b) <= tolerance, one, zero);
        case 1: return chosen(absolute(a - b) > tolerance, one, zero);
        case 2: return chosen(a < b, one, zero);
        case 3: return chosen(a <= b, one, zero);
        case 4: return chosen(a > b, one, zero);
        default: return chosen(a >= b, one, zero);
    }
}

/// 1 when both inputs are "true" (above 0.5), else 0.
///
/// Use when: combining flags in a graph
/// Tags: logicAnd, and, both, boolean and
/// Node: none (shader library)
template <typename Value>
inline Value logicalAnd(Value a, Value b) {
    Value one = Value(1.0f);
    Value zero = Value(0.0f);
    Value halfway = Value(0.5f);
    return chosen(a > halfway, chosen(b > halfway, one, zero), zero);
}

/// 1 when either input is "true" (above 0.5), else 0.
///
/// Use when: combining flags in a graph
/// Tags: logicOr, or, either, boolean or
/// Node: none (shader library)
template <typename Value>
inline Value logicalOr(Value a, Value b) {
    Value one = Value(1.0f);
    Value zero = Value(0.0f);
    Value halfway = Value(0.5f);
    return chosen(a > halfway, one, chosen(b > halfway, one, zero));
}

/// 1 when exactly one input is "true" (above 0.5), else 0.
///
/// Use when: combining flags in a graph
/// Tags: logicXor, xor, exclusive or, boolean xor
/// Node: none (shader library)
template <typename Value>
inline Value logicalExclusiveOr(Value a, Value b) {
    Value one = Value(1.0f);
    Value zero = Value(0.0f);
    Value halfway = Value(0.5f);
    return chosen(a > halfway, chosen(b > halfway, zero, one), chosen(b > halfway, one, zero));
}

/// 1 when the input is "false" (0.5 or below), else 0.
///
/// Use when: inverting a flag
/// Tags: logicNot, not, invert, boolean not
/// Node: none (shader library)
template <typename Value>
inline Value logicalNot(Value a) { return chosen(a > Value(0.5f), Value(0.0f), Value(1.0f)); }

/// 1 where the input is NaN (not a number), else 0.
///
/// How: a NaN is the one value for which `value >= value` is false.
///
/// Conventions: relies on IEEE comparisons, so a GPU shader built with fast math may report 0.
///
/// Use when: spotting broken values while debugging a graph
/// Tags: flagNaN, isnan, is NaN, not a number
/// Node: none (shader library)
template <typename Value>
inline Value flagNotANumber(Value a) { return chosen(a >= a, Value(0.0f), Value(1.0f)); }

/// 1 where the input is infinite (+∞ or −∞), else 0.
///
/// How: |value| above the largest finite float. A NaN gives 0. Same fast-math caveat as `flagNotANumber`.
///
/// Use when: spotting overflow while debugging a graph
/// Tags: flagInfinite, isinf, is infinite
/// Node: none (shader library)
template <typename Value>
inline Value flagInfinity(Value a) {
    return chosen(absolute(a) > largestFiniteFloat<Value>(), Value(1.0f), Value(0.0f));
}

// ---------------------------------------------------------------------------------------------
// Easing

/// The bounce-out curve on 0…1, a ball dropping and bouncing to rest; the bounce-in curve mirrors it.
///
/// How: Robert Penner's easing, four parabolas n1 t² + offset with n1 = 7.5625, d1 = 2.75 and
/// offsets 0, 0.75, 0.9375, 0.984375, each starting at 1/d1, 2/d1, 2.5/d1 of the way. 0 → 0, 1 → 1.
///
/// Conventions: `x` should be in 0…1; the caller clamps (`easedCurve` does).
///
/// Use when: a bouncing animation
/// Tags: easeBounceOut, bounce out, Penner, easing
/// Node: none (shader library)
template <typename Real>
inline Real bounceEaseOut(Real x) {
    Real n1 = Real(7.5625f);
    Real d1 = Real(2.75f);
    if (x < Real(1.0f) / d1) return n1 * x * x;
    if (x < Real(2.0f) / d1) { x -= Real(1.5f) / d1; return n1 * x * x + Real(0.75f); }
    if (x < Real(2.5f) / d1) { x -= Real(2.25f) / d1; return n1 * x * x + Real(0.9375f); }
    x -= Real(2.625f) / d1;
    return n1 * x * x + Real(0.984375f);
}

/// An easing curve on 0…1: `t` is clamped, then bent by the curve `mode`.
///
/// How: Robert Penner's and easings.net's formulas. Modes: 0 linear; quad 1 in, 2 out, 3 in-out;
/// cubic 4, 5, 6; expo 7, 8, 9; sine 10, 11, 12; back 13, 14, 15; elastic 16 in, 17 out; bounce
/// 18 in, anything else out. Every curve gives 0 at 0 and 1 at 1; back and elastic overshoot between.
/// Quad in at 0.5: 0.25. Cubic out at 0.5: 1 − 0.5³ = 0.875.
///
/// Use when: the Ease node; animating a value with acceleration
/// Tags: ease, easing, tween, quad, cubic, expo, sine, back, elastic, bounce, easings.net
/// Node: none (shader library)
template <typename Real>
inline Real easedCurve(Real t, int mode) {
    Real x = clamped(t, Real(0.0f), Real(1.0f));
    Real zero = Real(0.0f);
    Real one = Real(1.0f);
    Real two = Real(2.0f);
    Real three = Real(3.0f);
    Real halfway = Real(0.5f);
    switch (mode) {
        case 0: return x;
        case 1: return x * x;
        case 2: return one - (one - x) * (one - x);
        case 3: return x < halfway ? two * x * x : one - power(Real(-2.0f) * x + two, two) / two;
        case 4: return x * x * x;
        case 5: return one - power(one - x, three);
        case 6: return x < halfway ? Real(4.0f) * x * x * x : one - power(Real(-2.0f) * x + two, three) / two;
        case 7: return x == zero ? zero : power(two, Real(10.0f) * x - Real(10.0f));
        case 8: return x == one ? one : one - power(two, Real(-10.0f) * x);
        case 9:
            return x == zero ? zero : x == one ? one
                : x < halfway ? power(two, Real(20.0f) * x - Real(10.0f)) / two
                           : (two - power(two, Real(-20.0f) * x + Real(10.0f))) / two;
        case 10: return one - cosine(x * pi<Real>() / two);
        case 11: return sine(x * pi<Real>() / two);
        case 12: return -(cosine(pi<Real>() * x) - one) / two;
        case 13: {
            Real c1 = Real(1.70158f);
            Real c3 = c1 + one;
            return c3 * x * x * x - c1 * x * x;
        }
        case 14: {
            Real c1 = Real(1.70158f);
            Real c3 = c1 + one;
            return one + c3 * power(x - one, three) + c1 * power(x - one, two);
        }
        case 15: {
            Real c2 = Real(1.70158f) * Real(1.525f);
            return x < halfway ? (power(two * x, two) * ((c2 + one) * two * x - c2)) / two
                            : (power(two * x - two, two) * ((c2 + one) * (x * two - two) + c2) + two) / two;
        }
        case 16:
            return x == zero ? zero : x == one ? one
                : -power(two, Real(10.0f) * x - Real(10.0f)) * sine((x * Real(10.0f) - Real(10.75f)) * (tau<Real>() / three));
        case 17:
            return x == zero ? zero : x == one ? one
                : power(two, Real(-10.0f) * x) * sine((x * Real(10.0f) - Real(0.75f)) * (tau<Real>() / three)) + one;
        case 18: return one - bounceEaseOut(one - x);
        default: return bounceEaseOut(x);
    }
}

// ---------------------------------------------------------------------------------------------
// Vector maths that also works on plain numbers, for nodes that take either.

/// The dot product of two values: the product for numbers, the vector dot product for vectors.
///
/// Use when: a node that takes a number or a vector
/// Tags: vectorDot, dot product, inner product
/// Node: none (shader library)
inline float dotProductOfValues(float a, float b) { return a * b; }
template <typename Value>
inline auto dotProductOfValues(Value a, Value b) -> decltype(dotProduct(a, b)) { return dotProduct(a, b); }

/// The length of a value: |v| for a number, √(v · v) for a vector.
///
/// Use when: the size of a number or a vector
/// Tags: vectorLength, length, magnitude, abs
/// Node: none (shader library)
inline float lengthOfValue(float v) { return absolute(v); }
template <typename Value>
inline auto lengthOfValue(Value v) -> decltype(dotProductOfValues(v, v)) { return squareRoot(dotProductOfValues(v, v)); }

/// The distance between two values: the length of their difference.
///
/// Use when: the gap between two numbers or points
/// Tags: vectorDistance, distance
/// Node: none (shader library)
template <typename Value>
inline auto distanceBetweenValues(Value a, Value b) -> decltype(lengthOfValue(a - b)) { return lengthOfValue(a - b); }

/// The direction of a value with length 1: the sign (−1 or 1, and 1 for 0) for a number;
/// `v / |v|` for a vector, or the zero vector when the length is under 0.000001.
///
/// Use when: normalising a number or a vector without producing NaN
/// Tags: vectorNormalize, normalize, unit vector, sign
/// Node: none (shader library)
inline float normalizedValue(float v) { return v < 0.0f ? -1.0f : 1.0f; }
template <typename Value>
inline Value normalizedValue(Value v) {
    auto length = lengthOfValue(v);
    return chosen(length < decltype(length)(0.000001f), Value(0.0f), v / length);
}

/// The direction of the refracted ray when `incident` meets a surface with `normal`.
///
/// How: Snell's law as in GLSL's `refract`: c = normal · incident; k = 1 − η² (1 − c²); the
/// result is η incident − (η c + √k) normal, or the zero value when k < 0 (total internal reflection).
///
/// Conventions: `incident` and `normal` unit length; `eta` is the ratio of refractive indices n₁ / n₂.
///
/// Use when: glass and water looks
/// Tags: vectorRefract, refract, Snell, bend ray
/// Node: none (shader library)
template <typename Value, typename Real>
inline Value refractedValue(Value incident, Value normal, Real eta) {
    Real cosineOfAngle = dotProductOfValues(normal, incident);
    Real k = Real(1.0f) - eta * eta * (Real(1.0f) - cosineOfAngle * cosineOfAngle);
    return chosen(k < Real(0.0f), Value(0.0f), eta * incident - (eta * cosineOfAngle + squareRoot(k)) * normal);
}

/// The part of `a` that lies along `onto`.
///
/// How: onto × (a · onto) / max(onto · onto, 0.000001). (3, 4) onto (1, 0) → (3, 0).
///
/// Use when: splitting a vector into along and across parts
/// Tags: vectorProject, projection, project onto
/// Node: none (shader library)
template <typename Value>
inline Value projectedValue(Value a, Value onto) {
    auto ontoSquared = dotProductOfValues(onto, onto);
    return onto * (dotProductOfValues(a, onto) / largerOf(ontoSquared, decltype(ontoSquared)(0.000001f)));
}

/// The angle between two values, in radians from 0 to π.
///
/// How: acos(clamp(a · b / max(|a| |b|, 0.000001), −1, 1)). (1, 0) and (0, 1) → π/2. A zero value gives π/2.
///
/// Use when: the angle between two directions
/// Tags: vectorAngle, angle between, acos dot
/// Node: none (shader library)
template <typename Value>
inline auto angleBetweenValues(Value a, Value b) -> decltype(lengthOfValue(a)) {
    auto lengths = largerOf(lengthOfValue(a) * lengthOfValue(b), decltype(lengthOfValue(a))(0.000001f));
    return arcCosine(clamped(dotProductOfValues(a, b) / lengths, decltype(lengths)(-1.0f), decltype(lengths)(1.0f)));
}

/// The component of `v` at `index` (0 x, 1 y, 2 z, 3 w); an index outside 0…3 is moved into it.
///
/// Use when: reading a component by a number from a graph
/// Tags: component by index, v[i], subscript
/// Node: none (shader library)
template <typename Real>
inline Real componentOf(Vector4Of<Real> v, int index) {
    int clampedIndex = clamped(index, 0, 3);
    return clampedIndex == 0 ? v.x : clampedIndex == 1 ? v.y : clampedIndex == 2 ? v.z : v.w;
}

/// Four components of `v` picked by `which`: 0 to 3 a component, 4 zero, 5 one; others are moved into 0…5.
///
/// Use when: the Swizzle node
/// Tags: swizzle4, swizzle, shuffle components, v.xyzw
/// Node: none (shader library)
template <typename Real>
inline Real swizzledComponent(Vector4Of<Real> v, int which) {
    int clampedWhich = clamped(which, 0, 5);
    return clampedWhich == 4 ? Real(0.0f) : clampedWhich == 5 ? Real(1.0f) : componentOf(v, clampedWhich);
}
template <typename Real>
inline Vector4Of<Real> swizzled(Vector4Of<Real> v, Vector4Of<int> which) {
    return Vector4Of<Real>(swizzledComponent(v, which.x), swizzledComponent(v, which.y), swizzledComponent(v, which.z),
        swizzledComponent(v, which.w));
}

/// One of four values, picked by a number rounded to the nearest whole and moved into 0…3.
///
/// How: i = clamp(⌊index + 0.5⌋, 0, 3). 1.6 → 2 → the third value.
///
/// Use when: a Select node
/// Tags: selectIndex, select by index, switch, pick one of four
/// Node: none (shader library)
template <typename Value>
inline Value selectedByIndex(float index, Value a, Value b, Value c, Value d) {
    int i = clamped(static_cast<int>(roundedDown(index + 0.5f)), 0, 3);
    return i == 0 ? a : i == 1 ? b : i == 2 ? c : d;
}

// ---------------------------------------------------------------------------------------------
// Waves, keyframes and debug colours

/// A wave from −1 to 1 over each whole unit of `x`. `shape`: 0 sine, 1 triangle, 2 square, anything else sawtooth.
///
/// How: sine sin(2π x); triangle 1 − 4 |fract(x + 0.25) − 0.5|; square 1 for fract(x) < 0.5 else −1;
/// sawtooth 2 fract(x) − 1. At x = 0.25: sine 1, triangle 1, square 1, sawtooth −0.5.
///
/// Use when: oscillators, stripes, animation
/// Tags: waveShape, wave, oscillator, sine, triangle, square, sawtooth
/// Node: none (shader library)
template <typename Value>
inline Value waveShape(Value x, int shape) {
    switch (shape) {
        case 0: return sine(x * tau<float>());
        case 1: return Value(1.0f) - Value(4.0f) * absolute(fractionalPart(x + Value(0.25f)) - Value(0.5f));
        case 2: return chosen(fractionalPart(x) < Value(0.5f), Value(1.0f), Value(-1.0f));
        default: return Value(2.0f) * fractionalPart(x) - Value(1.0f);
    }
}

/// A looping value through four keyframes spaced evenly over `duration`.
///
/// How: t = flooredModulo(time / max(duration, 0.000001), 1) × 4; the segment is ⌊t⌋ and local = fract(t);
/// the result blends values[segment] to values[segment + 1] (wrapping to the first) by local
/// (`interpolation` 0), by local² (3 − 2 local) (1), or not at all, giving the first (anything else).
///
/// Use when: a Keyframes node; looping animation
/// Tags: keyframes4, keyframe, animation loop, timeline
/// Node: none (shader library)
template <typename Real>
inline Real keyframesOfFour(Real time, Vector4Of<Real> values, Real duration, int interpolation) {
    Real t = flooredModulo(time / largerOf(duration, Real(0.000001f)), Real(1.0f)) * Real(4.0f);
    int index = static_cast<int>(roundedDown(t));
    Real local = fractionalPart(t);
    Real from = componentOf(values, index % 4);
    Real to = componentOf(values, (index + 1) % 4);
    Real eased = interpolation == 0 ? local
        : interpolation == 1 ? local * local * (Real(3.0f) - Real(2.0f) * local) : Real(0.0f);
    return linearBlend(from, to, eased);
}

/// A heat-map colour for `t` in 0…1: blue, cyan, green, yellow, red.
///
/// How: each channel is clamp(1.5 − |4x − k|, 0, 1) with k = 3, 2, 1 for red, green, blue, x = clamp(t, 0, 1).
/// x = 0 gives (0, 0, 0.5), dark blue; x = 0.5 gives (0.5, 1, 0.5); x = 1 gives (0.5, 0, 0), dark red.
///
/// Use when: showing a number as colour while debugging
/// Tags: heatmap, heat map, false colour, jet, debug view
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> heatmap(Real t) {
    Real x = clamped(t, Real(0.0f), Real(1.0f));
    Real red = clamped(Real(1.5f) - absolute(Real(4.0f) * x - Real(3.0f)), Real(0.0f), Real(1.0f));
    Real green = clamped(Real(1.5f) - absolute(Real(4.0f) * x - Real(2.0f)), Real(0.0f), Real(1.0f));
    Real blue = clamped(Real(1.5f) - absolute(Real(4.0f) * x - Real(1.0f)), Real(0.0f), Real(1.0f));
    return Vector3Of<Real>(red, green, blue);
}

/// A colour (red, green, blue, 1) that shows a number or a vector while debugging.
///
/// How, for a number: mode 0 the heat map; mode 1 the heat map dimmed to 0.25 or 0.75 in bands
/// (fract(10 v) < 0.5); otherwise red for negative and green for positive, as bright as |v| up to 1.
/// For a vector: mode 2 maps −1…1 to 0…1 (v × 0.5 + 0.5); otherwise the components as they are
/// (a 2D vector gets blue 0).
///
/// Use when: the Debug view
/// Tags: debugColor, debug view, visualize value, show number as colour
/// Node: none (shader library)
inline Vector4Of<float> debugColor(float v, int mode) {
    switch (mode) {
        case 0: {
            Vector3Of<float> heat = heatmap(v);
            return Vector4Of<float>(heat.x, heat.y, heat.z, 1.0f);
        }
        case 1: {
            Vector3Of<float> heat = heatmap(v);
            float band = fractionalPart(v * 10.0f) < 0.5f ? 0.25f : 0.75f;
            return Vector4Of<float>(band * heat.x, band * heat.y, band * heat.z, 1.0f);
        }
        default: {
            float negative = smallerOf(-v, 1.0f);
            float positive = smallerOf(v, 1.0f);
            return v < 0.0f ? Vector4Of<float>(negative, 0.0f, 0.0f, 1.0f) : Vector4Of<float>(0.0f, positive, 0.0f, 1.0f);
        }
    }
}
template <typename Real>
inline Vector4Of<Real> debugColor(Vector2Of<Real> v, int mode) {
    return mode == 2 ? Vector4Of<Real>(v.x * Real(0.5f) + Real(0.5f), v.y * Real(0.5f) + Real(0.5f), Real(0.0f), Real(1.0f))
                     : Vector4Of<Real>(v.x, v.y, Real(0.0f), Real(1.0f));
}
template <typename Real>
inline Vector4Of<Real> debugColor(Vector3Of<Real> v, int mode) {
    return mode == 2 ? Vector4Of<Real>(v.x * Real(0.5f) + Real(0.5f), v.y * Real(0.5f) + Real(0.5f), v.z * Real(0.5f) + Real(0.5f), Real(1.0f))
                     : Vector4Of<Real>(v.x, v.y, v.z, Real(1.0f));
}
template <typename Real>
inline Vector4Of<Real> debugColor(Vector4Of<Real> v, int mode) {
    return debugColor(Vector3Of<Real>(v.x, v.y, v.z), mode);
}

/// `top` laid over `base` by the alpha of `top`.
///
/// How: colour = base + (top − base) × top.alpha per channel; alpha = max(base.alpha, top.alpha).
///
/// Use when: stacking one colour layer
/// Tags: over, alpha over, composite, layer, Porter-Duff
/// Node: none (shader library)
template <typename Real>
inline Vector4Of<Real> layerOver(Vector4Of<Real> base, Vector4Of<Real> top) {
    return Vector4Of<Real>(linearBlend(base.x, top.x, top.w), linearBlend(base.y, top.y, top.w),
        linearBlend(base.z, top.z, top.w), largerOf(base.w, top.w));
}

/// Up to four colours, each laid over the ones before it by its alpha.
///
/// How: layerOver(layerOver(layerOver(a, b), c), d).
///
/// Use when: a Layers node
/// Tags: layersOver, layers, composite four, stack colours
/// Node: none (shader library)
template <typename Real>
inline Vector4Of<Real> layersOver(Vector4Of<Real> a, Vector4Of<Real> b, Vector4Of<Real> c, Vector4Of<Real> d) {
    return layerOver(layerOver(layerOver(a, b), c), d);
}

} // namespace kernels
} // namespace editor
