// Small operations on single numbers that every other part of the math library leans on:
// blending between two values, clamping into a range, and deciding when a number is "close
// enough to zero" (Engine/Math/Scalar.swift). The Vector3 parts of that file (`distance`,
// `linearBlend`) live in Vector3.h.
#pragma once

#include <cmath>

#include "../Kernels/KernelPrelude.h"

namespace editor {

/// How close to zero a result must be to count as zero in geometric tests.
///
/// Floating-point arithmetic rounds after every step, so a value that is exactly zero on
/// paper (the determinant of a ray parallel to a plane, the length of a degenerate edge)
/// comes out as something like 3e-8. Geometry code compares against this tolerance instead
/// of against 0.
///
/// The value assumes scenes measured in metres with objects between about a millimetre and
/// a kilometre. Float carries about 7 significant digits, so 1e-6 sits safely above rounding
/// noise for values near 1.
///
/// Use when: comparing a computed number with zero where float rounding makes exact equality wrong
/// Tags: epsilon, tolerance, nearly zero, float error, precision, threshold
/// Node: none (a constant)
struct Tolerance {
    /// The tolerance used for geometric tests such as parallel rays and flat triangles.
    static constexpr float geometric = 1e-6f;
};

/// Whether `value` is within `tolerance` of zero: |value| ≤ tolerance.
///
/// Use when: avoiding exact comparison with zero before dividing; deciding that a direction is flat
/// Tags: nearly zero, epsilon test, approximately zero, is zero, Mathf.Approximately
/// Node: none (tolerance helper)
inline bool isNearlyZero(float value, float tolerance = Tolerance::geometric) { return std::fabs(value) <= tolerance; }

/// The smaller of two numbers, with Swift's `min` rules: on a tie, or when `second` is NaN, it
/// is `first`.
///
/// How: second < first ? second : first. `std::min` returns its first argument on a tie too, but
/// its NaN case differs, so the core writes the rule out once here.
///
/// Use when: porting a Swift `min(a, b)` so the core picks the same number bit for bit
/// Tags: min, minimum, smaller, fmin, Swift min
/// Node: none (scalar helper)
inline float smallerOf(float first, float second) { return kernels::smallerOf(first, second); }

/// The larger of two numbers, with Swift's `max` rules: on a tie it is `second`, and when
/// `second` is NaN it is `first`.
///
/// How: second >= first ? second : first.
///
/// Use when: porting a Swift `max(a, b)` so the core picks the same number bit for bit
/// Tags: max, maximum, larger, fmax, Swift max
/// Node: none (scalar helper)
inline float largerOf(float first, float second) { return kernels::largerOf(first, second); }

/// `value` moved into the range `lower`…`upper`: below the range it becomes `lower`, above it
/// `upper`, inside it stays as it is.
///
/// How: min(max(value, lower), upper), with Swift's tie and NaN rules (a NaN stays NaN); the
/// formula lives once, in the shared kernel `kernels::clamped`.
///
/// Use when: keeping a value inside a legal range such as 0 to 1 for a color or a slider
/// Tags: clamp, saturate, limit, min max, Mathf.Clamp, clamp()
/// Node: clamp
inline float clamped(float value, float lower, float upper) { return kernels::clamped(value, lower, upper); }

/// The value a fraction `amount` of the way from `start` to `end`. Graphics code often calls
/// this *lerp* (linear interpolation) or *mix*.
///
/// How: start + (end − start) × amount, the shared kernel `kernels::linearBlend`.
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
inline float linearBlend(float start, float end, float amount) { return kernels::linearBlend(start, end, amount); }

} // namespace editor
