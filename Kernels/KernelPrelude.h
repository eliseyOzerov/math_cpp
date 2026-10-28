// What every shared kernel may use: the few math functions whose name differs between the CPU
// and the GPU, and the scalar helpers kernels need. Kernels are headers that compile both as
// ordinary C++ (in the core) and as Metal Shading Language (in a .metal file that includes them),
// so they keep to the rules in Docs/Foundations.md: C++14 features only, no standard library
// beyond this file's <cmath>, no heap, exceptions, recursion, virtual functions or function
// pointers, values passed by copy.
#pragma once

#if defined(__METAL_VERSION__)
#include <metal_stdlib>
#else
#include <cmath>
#endif

namespace editor {
namespace kernels {

// The namespace that holds the system's math functions: `metal::sqrt` on the GPU, `std::sqrt` on
// the CPU. Kernels never name either; they call the functions below.
#if defined(__METAL_VERSION__)
namespace systemMath = metal;
#else
namespace systemMath = std;
#endif

/// The non-negative number whose square is `value`.
///
/// How: the system's `sqrt`, found by argument-dependent lookup, so a number type of our own
/// (such as a deterministic float) that declares its own `sqrt` beside it gets that one instead.
///
/// Conventions: a negative `value` gives NaN, as `sqrt` does. On the GPU Metal may use a faster,
/// less exact square root unless the shader is built without fast math; the CPU result is the
/// reference.
///
/// Use when: the length of a vector or a cosine from a sine inside a kernel
/// Tags: square root, sqrt, root, metal::sqrt, std::sqrt
/// Node: none (kernel helper)
template <typename Real>
inline Real squareRoot(Real value) {
    using systemMath::sqrt;
    return sqrt(value);
}

/// `value` without its sign: `value` itself when it is 0 or more, `−value` when it is below 0.
///
/// How: a comparison, `value < 0 ? −value : value`, so the CPU and the GPU agree; −0 stays −0
/// and a NaN stays NaN.
///
/// Use when: the size of a difference whatever its direction, such as a distance along one axis
/// Tags: absolute value, abs, fabs, magnitude, Mathf.Abs
/// Node: planned (absolute)
template <typename Real>
inline Real absolute(Real value) {
    return value < Real(0) ? -value : value;
}

/// The smaller of two numbers.
///
/// How: `second < first ? second : first`, Swift's `min` rule: on a tie, or when `second` is NaN,
/// `first` is returned.
///
/// Use when: keeping the nearer of two distances; the union of two signed distance fields
/// Tags: min, minimum, smaller, fmin, Mathf.Min
/// Node: planned (minimum)
template <typename Real>
inline Real smallerOf(Real first, Real second) {
    return second < first ? second : first;
}

/// The larger of two numbers.
///
/// How: `second >= first ? second : first`, Swift's `max` rule: on a tie `second` is returned,
/// and when `second` is NaN, `first`.
///
/// Use when: keeping the farther of two distances; the intersection of two signed distance fields
/// Tags: max, maximum, larger, fmax, Mathf.Max
/// Node: planned (maximum)
template <typename Real>
inline Real largerOf(Real first, Real second) {
    return second >= first ? second : first;
}

/// The largest whole number that is not above `value`: 2.7 → 2, −2.3 → −3.
///
/// How: the system's `floor`, found by argument-dependent lookup.
///
/// Use when: the cell a point lies in on a grid; repeating a pattern
/// Tags: floor, round down, integer part, Mathf.Floor, floor()
/// Node: planned (roundDown)
template <typename Real>
inline Real roundedDown(Real value) {
    using systemMath::floor;
    return floor(value);
}

/// How far `value` lies past the whole number below it, from 0 up to (not including) 1:
/// 2.7 → 0.7, −2.3 → 0.7.
///
/// How: `value − ⌊value⌋`. Metal's own `fract` caps the result just below 1; this one does not,
/// so a tiny negative value can give exactly 1 here, on both the CPU and the GPU.
///
/// Use when: the position inside a grid cell; a sawtooth wave; repeating a pattern
/// Tags: fract, fractional part, frac, modulo one, sawtooth
/// Node: planned (fraction)
template <typename Real>
inline Real fractionalPart(Real value) {
    return value - roundedDown(value);
}

/// The sine of `radians`.
///
/// How: the system's `sin`, found by argument-dependent lookup. On the GPU Metal's fast math may
/// differ from the CPU in the last bits; the CPU result is the reference.
///
/// Use when: a wave, a rotation, a point on a circle inside a kernel
/// Tags: sine, sin, trigonometry, wave
/// Node: none (kernel helper)
template <typename Real>
inline Real sine(Real radians) {
    using systemMath::sin;
    return sin(radians);
}

/// The cosine of `radians`.
///
/// How: the system's `cos`, found by argument-dependent lookup, as `sine`.
///
/// Use when: a rotation, a point on a circle, the angle between two directions inside a kernel
/// Tags: cosine, cos, trigonometry
/// Node: none (kernel helper)
template <typename Real>
inline Real cosine(Real radians) {
    using systemMath::cos;
    return cos(radians);
}

/// The tangent of `radians`, sin/cos, from the system's `tan`.
///
/// Conventions: radians; infinite (or very large) at odd multiples of π/2.
///
/// Use when: a focal length from a field of view; the slope of a line at an angle
/// Tags: tan, tangent, tan()
/// Node: none (kernel helper)
template <typename Real>
inline Real tangent(Real radians) {
    using systemMath::tan;
    return tan(radians);
}

/// The angle, in radians from 0 to π, whose cosine is `value`.
///
/// How: the system's `acos`, found by argument-dependent lookup. A `value` outside −1…1 gives NaN.
///
/// Use when: the angle between two unit directions inside a kernel
/// Tags: arc cosine, acos, inverse cosine, angle from cosine
/// Node: none (kernel helper)
template <typename Real>
inline Real arcCosine(Real value) {
    using systemMath::acos;
    return acos(value);
}

/// The angle of the point (`x`, `y`) from the positive x axis, in radians, from −π to π.
///
/// How: the system's `atan2(y, x)`, found by argument-dependent lookup. At the origin it gives 0.
///
/// Use when: turning a point into polar coordinates; the direction of a 2D vector
/// Tags: atan2, arc tangent, angle of a vector, polar angle, heading
/// Node: none (kernel helper)
template <typename Real>
inline Real arcTangent(Real y, Real x) {
    using systemMath::atan2;
    return atan2(y, x);
}

/// `base` raised to `exponent`.
///
/// How: the system's `pow`, found by argument-dependent lookup. A negative `base` with a
/// non-whole `exponent` gives NaN.
///
/// Use when: gamma curves, falloffs, easing with a power
/// Tags: power, pow, exponent, raise to
/// Node: none (kernel helper)
template <typename Real>
inline Real power(Real base, Real exponent) {
    using systemMath::pow;
    return pow(base, exponent);
}

/// e raised to `value`.
///
/// How: the system's `exp`, found by argument-dependent lookup.
///
/// Use when: exponential falloff, smooth minimum, fog
/// Tags: exponential, exp, e to the power
/// Node: none (kernel helper)
template <typename Real>
inline Real exponential(Real value) {
    using systemMath::exp;
    return exp(value);
}

/// The natural logarithm of `value` (the power e must be raised to, to give `value`).
///
/// How: the system's `log`, found by argument-dependent lookup. 0 gives −infinity and a negative
/// value NaN.
///
/// Use when: undoing an exponential; perceptual scales; smooth minimum
/// Tags: logarithm, log, ln, natural log
/// Node: none (kernel helper)
template <typename Real>
inline Real naturalLogarithm(Real value) {
    using systemMath::log;
    return log(value);
}

/// `ifTrue` when `condition` holds, otherwise `ifFalse`. For vectors (KernelVectors.h) the
/// choice is made per component.
///
/// How: the conditional operator.
///
/// Use when: picking between two results without an `if`, as shaders do
/// Tags: select, choose, ternary, conditional, mix by condition
/// Node: planned (switch)
template <typename Real>
inline Real chosen(bool condition, Real ifTrue, Real ifFalse) {
    return condition ? ifTrue : ifFalse;
}

/// The sign of `value`: 1 above 0, −1 below 0, and 0 for 0 (and for NaN), as Metal's `sign`.
///
/// How: comparisons, `value > 0 ? 1 : value < 0 ? −1 : 0`, so the CPU and the GPU agree.
///
/// Conventions: unlike `normalizedValue` (ShaderMath.h), 0 gives 0, not 1.
///
/// Use when: telling inside from outside in a signed distance formula; the side of a line a point is on
/// Tags: sign, signum, sgn, sign(), side of
/// Node: none (kernel helper)
template <typename Real>
inline Real signOf(Real value) {
    return value > Real(0.0f) ? Real(1.0f) : (value < Real(0.0f) ? Real(-1.0f) : Real(0.0f));
}

/// GLSL's `step`: 0 when `value` is below `edge`, otherwise 1.
///
/// How: `value < edge ? 0 : 1`. A NaN `value` gives 1.
///
/// Use when: choosing between two results with a blend amount of exactly 0 or 1
/// Tags: step, stepFunction, threshold, Heaviside, step()
/// Node: none (kernel helper)
template <typename Real>
inline Real stepFunction(Real edge, Real value) {
    return value < edge ? Real(0.0f) : Real(1.0f);
}

/// The 32 bits that store `value`, read as an unsigned integer: 1.0 → 0x3F800000.
///
/// How: a bit cast, no conversion (Metal's `as_type`, clang's `__builtin_bit_cast` on the CPU).
///
/// Use when: hashing a float so that every float, not only every whole number, hashes apart
/// Tags: bit cast, as_type, float bits, reinterpret, floatBitsToUint
/// Node: none (kernel helper)
inline unsigned int bitsOf(float value) {
#if defined(__METAL_VERSION__)
    return as_type<unsigned int>(value);
#else
    return __builtin_bit_cast(unsigned int, value);
#endif
}

/// `value` moved into the range `lower`…`upper`: below the range it becomes `lower`, above it
/// `upper`, inside it stays as it is.
///
/// How: min(max(value, lower), upper), with Swift's tie and NaN rules (a NaN stays NaN), written
/// with comparisons so the CPU and the GPU pick the same number.
///
/// Use when: keeping a value inside a legal range such as 0 to 1 for a color or a slider
/// Tags: clamp, saturate, limit, min max, Mathf.Clamp, clamp()
/// Node: clamp
template <typename Real>
inline Real clamped(Real value, Real lower, Real upper) {
    Real atLeastLower = lower >= value ? lower : value;
    return upper < atLeastLower ? upper : atLeastLower;
}

/// The value a fraction `amount` of the way from `start` to `end`. Graphics code often calls
/// this *lerp* (linear interpolation) or *mix*.
///
/// How: start + (end − start) × amount.
///
///     start ●──────────●────────────────● end
///          amount 0   amount 0.4        amount 1
///
/// `amount` is not clamped: 1.5 goes half the distance again past `end`, and −1 lands as far
/// before `start` as `end` is after it.
///
/// Use when: blending two numbers by a fraction, such as fading a color channel or easing an animation
/// Tags: lerp, mix, linear interpolation, blend, Mathf.Lerp, mix()
/// Node: blend
template <typename Real>
inline Real linearBlend(Real start, Real end, Real amount) {
    return start + (end - start) * amount;
}

} // namespace kernels
} // namespace editor
