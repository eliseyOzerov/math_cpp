// Refraction: how a ray of light bends when it passes from one transparent material (air,
// glass, water) into another (Engine/Math/Optics/Refraction.swift).
#pragma once

#include <optional>

#include "../Linear/Angle.h"
#include "../Linear/Vector3.h"

namespace editor {

/// Refraction of a ray at a smooth surface between two media.
///
/// Sources: Snell's law (Snell 1621); the vector form is Glassner, *An Introduction to Ray
/// Tracing* (1989), and Pharr, Jakob & Humphreys, *Physically Based Rendering* 4th ed. §9.3.
///
/// Use when: bending light as it enters glass or water
/// Tags: refraction, Snell's law, index of refraction, bending light, refract, glass, water
/// Node: none (renderer shading)
struct Refraction {
    /// The refracted direction of a ray, or no value when the light cannot enter the second
    /// medium (**total internal reflection**) (Swift `refractedDirection(incoming:normal:refractiveIndexRatio:)`).
    ///
    /// How. Snell's law links the angle from the normal on each side, with `n₁`, `n₂` the
    /// refractive indices (how slow light is in the material) and `η = n₁/n₂`:
    ///
    ///     n₁ sin θ₁ = n₂ sin θ₂        so        sin θ₂ = η sin θ₁
    ///
    ///          incoming ╲ │ normal
    ///                    ╲│ θ₁            medium 1 (n₁)
    ///     ───────────────●─────────── surface
    ///                    │╲ θ₂            medium 2 (n₂)
    ///                    │ ╲
    ///                       ╲ result
    ///
    /// The refracted ray stays in the plane of the incoming ray and the normal. Its part
    /// along the surface is the incoming one shrunk by η (that is Snell's law), and its part
    /// along the normal is fixed by needing length 1:
    ///
    ///     cos θ₁ = −incoming · normal
    ///     sin²θ₂ = η² (1 − cos²θ₁)
    ///     cos θ₂ = √(1 − sin²θ₂)
    ///     result = η·incoming + (η·cos θ₁ − cos θ₂)·normal
    ///
    /// When `sin²θ₂ > 1` no angle θ₂ exists: the light is reflected instead (use
    /// `Vector3::reflectedAcrossPlane`). That needs `η > 1` (light going into a thinner
    /// medium), at angles beyond `criticalAngle`.
    ///
    /// Conventions: `incoming` points toward the surface and `normal` is the surface normal,
    /// both of length 1. The normal may point to either side: if it points along the
    /// incoming ray, it is flipped, so a ray travelling inside a glass ball can use the
    /// ball's outward normal. In that case `refractiveIndexRatio` is still
    /// `n_current/n_next`, i.e. 1.5 / 1 for glass to air. `refractiveIndexRatio` must be
    /// positive (η ≤ 0 fails an `assert`, as no material has such an index). A NaN
    /// in the ratio or the vectors returns no value, because the comparison `sin²θ₂ ≤ 1` is
    /// then false. η = 1 returns `incoming` unchanged. The result has length 1 (up to rounding)
    /// and lies on the far side of the surface. A ray exactly along the normal passes
    /// straight through. At exactly the critical angle cos θ₂ is 0 and the ray skims along
    /// the surface; rounding decides whether that case returns nothing. At exact grazing
    /// (`incoming · normal = 0`) there is no far side to choose: for η < 1 the result
    /// leans toward either side with equal right, and this code puts it on the side the
    /// given `normal` points to. Do not rely on that.
    ///
    /// Use when: getting the direction a ray takes after entering glass or water
    /// Tags: refract, refracted ray, Snell's law, refraction direction, total internal reflection, GLSL refract
    /// Node: none (renderer shading)
    static std::optional<Vector3> refractedDirection(Vector3 incoming, Vector3 normal, float refractiveIndexRatio);

    /// The **critical angle**: the angle from the normal beyond which light going from
    /// `n₁` into `n₂` is totally reflected. No value when there is none (`η ≤ 1`, light entering
    /// a denser medium always gets through).
    ///
    /// Formula: set θ₂ = 90° in Snell's law: `sin θ_c = n₂/n₁ = 1/η`, so `θ_c = asin(1/η)`.
    /// For glass to air (η = 1.5) that is asin(2/3) ≈ 41.81°.
    ///
    /// Use when: finding when light inside glass or water bounces back instead of leaving
    /// Tags: critical angle, total internal reflection, TIR, Snell's window, brilliant cut
    /// Node: none (renderer shading)
    static std::optional<Angle> criticalAngle(float refractiveIndexRatio);
};

} // namespace editor
