// The exact Fresnel equations for a smooth boundary between two transparent materials
// (air, water, glass): how much light bounces off and how much goes through
// (Engine/Math/Optics/DielectricFresnel.swift).
#pragma once

#include "../Linear/Angle.h"

namespace editor {

/// The share of light reflected at a boundary, for each polarization and averaged.
///
/// Light is a wave that wiggles across its direction of travel. Split the wiggle into the
/// part that lies in the surface (**s**, from German *senkrecht*, perpendicular to the plane
/// of incidence) and the part that lies in the plane of incidence (**p**, parallel to it).
/// The surface treats the two differently, so each has its own reflectance. Most light in a
/// scene is unpolarized: half of each, so the reflectance we draw with is the average.
///
/// Use when: reading how much light a glass or water surface reflects
/// Tags: Fresnel reflectance, Rs Rp, polarized reflectance, reflection share, reflected power
/// Node: none (value type)
struct FresnelReflectance {
    /// Fraction of the power reflected for s-polarized light (0…1).
    float perpendicular = 0;
    /// Fraction of the power reflected for p-polarized light (0…1).
    float parallel = 0;

    constexpr bool operator==(const FresnelReflectance&) const = default;

    /// Fraction of the power reflected for unpolarized light: `(Rs + Rp) / 2`.
    ///
    /// Use when: getting the ordinary reflection strength of glass or water
    /// Tags: unpolarized reflectance, average reflectance, reflection fraction, Fresnel, reflectance
    /// Node: none (renderer shading)
    constexpr float unpolarized() const { return (perpendicular + parallel) / 2; }

    /// Fraction of the power that goes through for unpolarized light: `1 − R`. No light is
    /// absorbed at the boundary of a dielectric, so what is not reflected is transmitted.
    ///
    /// Use when: deciding how much light passes through a window or water surface
    /// Tags: transmittance, transmission, light through glass, 1 - reflectance, refracted share
    /// Node: none (renderer shading)
    constexpr float transmittance() const { return 1 - unpolarized(); }
};

/// The **Fresnel equations** for a boundary between two non-conducting materials.
///
/// Sources: Born & Wolf, *Principles of Optics* §1.5 (the amplitude coefficients, derived
/// from Maxwell's equations by requiring the wave's tangential fields to match across the
/// boundary); Pharr, Jakob & Humphreys, *PBRT* 4th ed. §9.3 (the same, in graphics form).
/// `kernels::SchlickFresnel` is the cheap approximation of this function.
///
/// Use when: deciding how mirror-like glass or water looks at different viewing angles
/// Tags: Fresnel, Fresnel equations, glass reflection, water reflection, dielectric, reflectivity
/// Node: none (renderer shading)
struct DielectricFresnel {
    /// The reflectance of light arriving at angle θᵢ from the normal, going from a material
    /// with refractive index `n₁` into one with `n₂` (Swift `reflectance(cosineIncident:refractiveIndexRatio:)`).
    ///
    /// How. Snell's law gives the angle θₜ of the light that goes through. The reflected
    /// *amplitude* of each polarization is (Born & Wolf 1.5.20, PBRT 9.3):
    ///
    ///     rs = (n₁ cos θᵢ − n₂ cos θₜ) / (n₁ cos θᵢ + n₂ cos θₜ)
    ///     rp = (n₂ cos θᵢ − n₁ cos θₜ) / (n₂ cos θᵢ + n₁ cos θₜ)
    ///
    /// and the reflected *power* is the amplitude squared, `Rs = rs²`, `Rp = rp²`. Only the
    /// ratio η = n₁/n₂ matters, so dividing the top and bottom of both lines by n₂:
    ///
    ///     sin²θₜ = η² (1 − cos²θᵢ)            (Snell's law)
    ///     cos θₜ = √(1 − sin²θₜ)
    ///     rs = (η cos θᵢ − cos θₜ) / (η cos θᵢ + cos θₜ)
    ///     rp = (cos θᵢ − η cos θₜ) / (cos θᵢ + η cos θₜ)
    ///
    ///            θᵢ ╲ │ ╱ reflected: R
    ///                ╲│╱
    ///     ────────────●──────────── boundary
    ///                  │╲
    ///                  │ ╲ transmitted: 1 − R
    ///
    /// Landmarks. At θᵢ = 0 both reduce to `((η − 1)/(η + 1))²`, 4% for air to glass. At
    /// **Brewster's angle**, `tan θᵢ = n₂/n₁`, light leaving the surface and the reflected
    /// light are at right angles, and p-polarized light is not reflected at all (`Rp = 0`).
    /// At grazing incidence (cos θᵢ → 0) both go to 1: every surface is a mirror at the
    /// horizon. When η > 1 and sin²θₜ > 1 there is no transmitted angle: **total internal
    /// reflection**, `Rs = Rp = 1`.
    ///
    /// Conventions. `cosineIncident` is `|incoming · normal|`, the cosine of the angle on
    /// the side the light comes from, clamped to 0…1. A negative cosine means the light
    /// arrives from the other side: pass its absolute value, together with the ratio for
    /// that direction. A negative cosine passed as it is clamps to 0 and returns R = 1 (a
    /// mirror), which is wrong for that light, so callers must take the absolute value.
    /// `refractiveIndexRatio` is `n₁/n₂`, *current over next*, the same
    /// convention as `Refraction::refractedDirection`, and must be positive (an `assert`). A cosine of 0
    /// (exact grazing) returns 1 for both polarizations, avoiding 0/0. A NaN cosine or
    /// ratio gives NaN. The result is symmetric under swapping the sides: light going the
    /// other way through the refracted angle has the same reflectance (Stokes' reversibility).
    ///
    /// Use when: mixing reflection and refraction on glass, water or a gemstone
    /// Tags: Fresnel reflectance, Fresnel equations, reflectance, reflection amount, glass shading, fresnel()
    /// Node: none (renderer shading)
    static FresnelReflectance reflectance(float cosineIncident, float refractiveIndexRatio);

    /// **Brewster's angle**: the angle of incidence at which p-polarized light is not
    /// reflected, for light going from `n₁` into `n₂`.
    ///
    /// Formula: `tan θ_B = n₂/n₁ = 1/η`, so `θ_B = atan(1/η)` (Brewster 1815; Born & Wolf
    /// §1.5). Air to glass (η = 1/1.5): atan(1.5) ≈ 56.31°. At this angle the reflected and
    /// the transmitted ray are at 90° to each other. `refractiveIndexRatio` must be
    /// positive. It exists for every ratio, also for light going into a thinner
    /// medium (then it lies below the critical angle).
    ///
    /// Use when: finding the angle where glare off water or glass disappears behind a
    ///   polarizing filter
    /// Tags: Brewster angle, polarizing angle, no reflection angle, polarization
    /// Node: none (renderer shading)
    static Angle brewsterAngle(float refractiveIndexRatio);
};

} // namespace editor
