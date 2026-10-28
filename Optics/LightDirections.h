// Conventions shared by every file in Optics/, and the two direction operations they all use
// (Engine/Math/Optics/LightDirections.swift). Swift adds these as extensions of Vector3 and
// Float; the core's Vector3 keeps its members to geometry, so here they are free functions.
//
// All directions are unit vectors in a right-handed, Z-up world. A surface point has a unit
// `normal`. The two light directions are named by where they point *to*, both away from the
// surface, which keeps every shading formula symmetric:
//
//            normal ▲
//     towardLight ╲ │ ╱ towardViewer
//                  ╲│╱
//     ──────────────●────────────── surface
//
// A ray travelling *toward* the surface (used by refraction) is called `incoming` and is the
// negative of the `towardLight` direction of the same ray.
#pragma once

#include <limits>

#include "../Linear/Scalar.h"
#include "../Linear/Vector3.h"

namespace editor {

/// The cosine of the angle between two directions, for two unit vectors (Swift
/// `Vector3.cosineOfAngle(to:)`).
///
/// Formula: `first · second` (the dot product of two unit vectors is the cosine of the
/// angle between them). The result is 1 for the same direction, 0 at right angles and −1
/// for opposite directions. Lengths are not checked: normalize the inputs first.
///
/// Use when: finding the angle between a normal and a light, or between two
///   directions, without calling acos
/// Tags: cosine of angle, dot product, angle between vectors, N dot L, angle cosine
/// Node: none (renderer shading)
inline float cosineOfAngle(Vector3 first, Vector3 second) { return first.dot(second); }

/// The direction halfway between two unit directions, the **half vector** `h` of
/// Blinn (1977) (Swift `Vector3.halfVector(with:)`). With the light and the viewer on the same
/// side of a surface, `h` is the microfacet normal that would mirror light straight into the
/// viewer.
///
/// Formula: `(first + second) / |first + second|`.
///
///         h
///     l ╲ │ ╱ v        h bisects the angle between l and v
///        ╲│╱
///
/// Edge case: if the two directions are exactly opposite, the sum is zero, there is no
/// halfway direction, and (0, 0, 0) comes back (see `Vector3::normalized`).
///
/// Use when: finding the surface normal that would reflect light toward the viewer,
///   for highlights
/// Tags: half vector, halfway vector, Blinn half vector, bisector, microfacet normal
/// Node: none (renderer shading)
inline Vector3 halfVector(Vector3 first, Vector3 second) { return (first + second).normalized(); }

/// `sin²θ` of an angle whose cosine is known: the share of a unit direction that lies
/// along the surface rather than along the normal (Swift `Float.sineSquared(fromCosine:)`).
///
/// Formula: `1 − cos²θ` (from `sin²θ + cos²θ = 1`). The cosine is clamped to −1…1 first,
/// so rounding that pushes a cosine of a unit vector a hair past 1 gives 0, never a
/// small negative number that would break a following square root.
///
/// Conventions: the result is 0 for cosine ±1 and 1 for cosine 0. A NaN cosine gives NaN.
///
/// Use when: getting sin squared from a cosine in Fresnel or Snell formulas without
///   calling sine
/// Tags: sine squared, sin2, 1 - cos2, trigonometric identity, Snell
/// Node: none (renderer shading)
inline float sineSquaredFromCosine(float cosine) {
    float clampedCosine = clamped(cosine, -1, 1);
    return 1 - clampedCosine * clampedCosine;
}

/// `tan²θ` of an angle whose cosine is known: how far a direction leans over, squared (Swift
/// `Float.tangentSquared(fromCosine:)`).
///
/// Formula: `sin²θ / cos²θ = (1 − cos²θ) / cos²θ`.
///
/// Conventions: 0 for cosine ±1, 1 for cosine ±0.7071 (45°). At cosine 0 (the horizon)
/// the tangent is infinite and infinity comes back; callers decide first
/// whether such a direction is meaningful.
///
/// Use when: getting tan squared from a cosine in GGX or masking formulas
/// Tags: tangent squared, tan2, sin2 over cos2, trigonometric identity, GGX
/// Node: none (renderer shading)
inline float tangentSquaredFromCosine(float cosine) { return sineSquaredFromCosine(cosine) / (cosine * cosine); }

} // namespace editor
