// How light fades as it travels through coloured glass, water or any material that absorbs
// (Engine/Math/Optics/BeerLambert.swift).
#pragma once

#include "../Linear/Vector3.h"

namespace editor {

/// **Beer–Lambert absorption** of light inside a material, one value per colour channel.
///
/// Sources: Bouguer (1729) and Lambert (1760) for the decay with distance, Beer (1852) for
/// the dependence on concentration; *PBRT* 4th ed. §11.2 (transmittance in media).
///
/// Why exponential: if a thin layer of thickness `δ` absorbs the fraction `σδ` of the light
/// reaching it, then each further layer takes the same fraction of what is left. Letting
/// `δ → 0` turns that into `dI/dd = −σ·I`, whose solution is `I = I₀·exp(−σ·d)`. Two layers
/// then multiply: `T(a + b) = T(a)·T(b)`.
///
/// Conventions: colour channels are red, green, blue in a `Vector3` (x, y, z), linear light,
/// each 0…1 for transmittance. Thicknesses are in the same length unit as the inverse of the
/// absorption coefficients (metres and 1/metre, say); nothing here assumes which.
///
/// Use when: tinting glass, water or gemstones more the thicker they are
/// Tags: Beer-Lambert, absorption, transmittance, attenuation, colored glass, extinction
/// Node: none (renderer shading)
struct BeerLambert {
    /// The smallest transmittance `absorptionCoefficient` accepts.
    /// A colour channel of exactly 0 would need an infinite coefficient; a millionth is black
    /// for every purpose and keeps the coefficient finite (`−ln 10⁻⁶ ≈ 13.8` per distance).
    static constexpr float minimumTransmittance = 1e-6f;

    /// The fraction of light left after travelling `thickness` through the material, per
    /// channel (Swift `transmittance(absorptionCoefficient:thickness:)`).
    ///
    /// Formula: `T = exp(−σ·d)` for each channel, with `σ` the **absorption coefficient**
    /// (the fraction absorbed per unit length, in the limit of short lengths) and `d` the
    /// thickness.
    ///
    ///     1 ┤╲
    ///       │ ╲___         T falls by the same factor every equal step of d:
    ///       │     ╲___     after d = 1/σ only 1/e = 36.8% is left.
    ///       └──────────────
    ///       0            d
    ///
    /// Edge cases. `thickness ≤ 0` returns (1, 1, 1): no material, no loss (a negative
    /// thickness would amplify light, which no absorber does). A negative coefficient is
    /// treated as 0 for the same reason. An infinite coefficient with positive thickness
    /// gives 0. A NaN input gives NaN.
    ///
    /// Use when: darkening or tinting light that passes through thick glass or deep water
    /// Tags: transmittance, light absorption, exp(-ad), attenuation, Beer's law, tint by thickness
    /// Node: none (renderer shading)
    static Vector3 transmittance(Vector3 absorptionCoefficient, float thickness);

    /// The absorption coefficient that leaves exactly `transmittance` of the light after
    /// `distance`: the artist-friendly way to say "this glass is green at 10 cm" (Swift
    /// `absorptionCoefficient(transmittance:atDistance:)`).
    ///
    /// Formula: invert `T = exp(−σ·d)`, giving `σ = −ln(T) / d` per channel.
    ///
    /// Each channel of `transmittance` is clamped to `minimumTransmittance…1`, so black does
    /// not give an infinite coefficient and a value above 1 (which would need a material
    /// that makes light) gives 0. `distance` must be positive (an `assert`: "the colour
    /// after zero thickness" fixes nothing).
    ///
    /// Use when: picking the tint of a glass so a given thickness lets through a chosen color
    /// Tags: absorption coefficient, find tint, inverse Beer-Lambert, color from transmittance, attenuation color
    /// Node: none (renderer shading)
    static Vector3 absorptionCoefficient(Vector3 transmittance, float distance);
};

} // namespace editor
