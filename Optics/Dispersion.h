// Dispersion: glass bends blue light more than red, which is why a prism makes a rainbow
// (Engine/Math/Optics/Dispersion.swift).
#pragma once

#include "../Color/Wavelength.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A material whose refractive index depends on the colour (wavelength) of the light (Swift
/// protocol `DispersiveMaterial`).
///
/// Use when: writing code that works with any glass or gemstone whose index depends on color
/// Tags: dispersion, refractive index by wavelength, material, rainbow, color-dependent refraction
/// Node: none (renderer shading)
class DispersiveMaterial {
public:
    virtual ~DispersiveMaterial() = default;

    /// The refractive index `n` for light of `wavelength`.
    ///
    /// Use when: refracting red, green and blue differently through a prism or diamond
    /// Tags: index of refraction, IOR, refractive index, wavelength, n(lambda), refraction by color
    /// Node: none (renderer shading)
    virtual float refractiveIndex(Wavelength wavelength) const = 0;

    /// The **Abbe number** `V_d = (n_d − 1) / (n_F − n_C)`: how little the glass disperses.
    ///
    /// The numerator is how strongly the glass bends yellow light (d line); the denominator
    /// is the spread of the index between blue (F) and red (C). High V (crown glass, 60+)
    /// splits colours little; low V (flint glass, below 50) splits them a lot. Source: Abbe
    /// (1880s); ISO 7944.
    ///
    /// Use when: telling a strongly rainbow-splitting glass from a weakly splitting one
    /// Tags: Abbe number, dispersion strength, glass type, V_d, chromatic dispersion
    /// Node: none (renderer shading)
    float abbeNumber() const;
};

/// **Cauchy's equation**, the simplest dispersion law: `n(λ) = A + B/λ²`.
///
/// Source: Cauchy, *Mémoire sur la dispersion de la lumière* (1836). The two coefficients
/// are fitted to a glass over the visible range; it is cheap and fine for drawing, but it
/// drifts outside 0.4–0.7 µm, where `SellmeierDispersion` is the one to use for a named glass.
///
/// Conventions: the wavelength enters in **micrometres** (the unit `B` is quoted for), so
/// `B` is in µm². The index grows as the wavelength shrinks: blue bends more than red. A
/// wavelength of 0 or less fails an `assert`.
///
/// Use when: faking a glass or crystal prism that splits white light into colors with simple
///   numbers
/// Tags: Cauchy, dispersion, prism, rainbow, chromatic aberration, refractive index law
/// Node: none (renderer shading)
class CauchyDispersion : public DispersiveMaterial {
public:
    /// The index at long wavelengths (λ → ∞), dimensionless.
    float coefficientA;
    /// How much the index rises toward short wavelengths, in µm².
    float coefficientB;

    constexpr CauchyDispersion(float coefficientA, float coefficientB)
        : coefficientA(coefficientA), coefficientB(coefficientB) {}

    /// Borosilicate crown glass (Schott N-BK7 class), `A = 1.5046`, `B = 0.00420 µm²`
    /// (the values commonly quoted for BK7 from a fit to its visible indices). Gives
    /// 1.5168 at the d line.
    static CauchyDispersion borosilicateCrown() { return CauchyDispersion(1.5046f, 0.00420f); }

    /// The refractive index for light of `wavelength` from Cauchy's equation `A + B/λ²`.
    ///
    /// Use when: splitting white light into colours with a simple glass model
    /// Tags: Cauchy index, n of lambda, refractive index at wavelength
    /// Node: none (renderer shading)
    float refractiveIndex(Wavelength wavelength) const override;
};

/// The **Sellmeier equation**, which models a glass as a few absorption resonances:
///
///     n²(λ) = 1 + B₁λ²/(λ² − C₁) + B₂λ²/(λ² − C₂) + B₃λ²/(λ² − C₃)
///
/// Source: Sellmeier (1871). Each term is one resonance at wavelength √Cᵢ with strength Bᵢ;
/// away from the resonances the sum is smooth and accurate to about 10⁻⁵ across the
/// visible range for a catalogued glass. The two resonances in the ultraviolet (√C₁, √C₂
/// between about 0.07 and 0.25 µm, depending on the glass) make the index rise towards blue;
/// the one in the infrared (√C₃ near 10 µm) makes it fall towards red.
///
/// Conventions: **λ in micrometres**, so each `Cᵢ` is in µm² (as catalogs quote them). The
/// wavelength must be positive (an `assert`) and not exactly on a resonance (√Cᵢ, far outside
/// the visible range), where `λ² − Cᵢ` is 0.
///
/// Use when: modelling real glass types and diamond accurately for a gem or lens
/// Tags: Sellmeier, dispersion, glass catalogue, accurate IOR, refractive index law, BK7
/// Node: none (renderer shading)
class SellmeierDispersion : public DispersiveMaterial {
public:
    /// The strengths B₁, B₂, B₃ of the three resonances, dimensionless.
    Vector3 strengths;
    /// The squared resonance wavelengths C₁, C₂, C₃, in µm².
    Vector3 squaredResonanceWavelengths;

    constexpr SellmeierDispersion(Vector3 strengths, Vector3 squaredResonanceWavelengths)
        : strengths(strengths), squaredResonanceWavelengths(squaredResonanceWavelengths) {}

    /// Schott **N-BK7** borosilicate crown glass, coefficients from the Schott optical glass
    /// data sheet (Schott 2016): n_d = 1.5168, Abbe number 64.17.
    static SellmeierDispersion schottNBK7();

    /// Schott **N-SF11** dense flint glass, coefficients from the Schott optical glass data
    /// sheet: n_d = 1.78472, Abbe number 25.68. It disperses about 2.5 times as much as
    /// N-BK7, which makes it the glass for pictures of a prism splitting light.
    static SellmeierDispersion schottNSF11();

    /// **Fused silica** (pure SiO₂ glass) at 20 °C, coefficients from Malitson, "Interspecimen
    /// comparison of the refractive index of fused silica", J. Opt. Soc. Am. 55 (1965): the
    /// resonance wavelengths 0.0684043, 0.1162414 and 9.896161 µm are squared here.
    /// n_d = 1.4585, Abbe number 67.8.
    static SellmeierDispersion fusedSilica();

    /// The refractive index for light of `wavelength` from the Sellmeier equation.
    ///
    /// Use when: getting the exact refractive index of a real glass for one colour of light
    /// Tags: Sellmeier index, n of lambda, refractive index at wavelength
    /// Node: none (renderer shading)
    float refractiveIndex(Wavelength wavelength) const override;
};

} // namespace editor
