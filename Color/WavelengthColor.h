// The colour we see for light of a single wavelength, for drawing rainbows and prism light
// (Engine/Math/Optics/WavelengthColor.swift).
#pragma once

#include "../Linear/Vector3.h"
#include "Wavelength.h"

namespace editor {

/// The colour of a pure spectral light, from the CIE 1931 standard observer.
///
/// Sources. The CIE 1931 colour matching functions `x̄, ȳ, z̄` say how strongly the three
/// kinds of cone in an average eye respond to each wavelength. Wyman, Sloan & Shirley,
/// "Simple Analytic Approximations to the CIE XYZ Color Matching Functions", Journal of
/// Computer Graphics Techniques 2(2), 2013, fit them with sums of two or three Gaussian
/// bumps (their "multi-lobe" fit, error under about 1% of the peak). The conversion to
/// linear sRGB is the matrix of IEC 61966-2-1 (D65 white).
///
/// Use when: painting a rainbow or the colors split by a prism or diamond
/// Tags: spectrum color, rainbow, CIE 1931, spectral color, wavelength to color, color matching
/// Node: none (renderer shading)
struct WavelengthColor {
    /// The CIE 1931 colour matching functions at `wavelength`, as (x̄, ȳ, z̄) (Swift
    /// `colorMatching(at:)`).
    ///
    /// Formula (Wyman et al. 2013, multi-lobe fit, λ in nm). Each lobe is a bump that is
    /// narrower on one side than on the other:
    ///
    ///     g(λ; μ, σ₁, σ₂) = exp(−½ ((λ − μ)/σ)²),  σ = σ₁ if λ < μ else σ₂
    ///
    ///     x̄ =  1.056 g(λ; 599.8, 37.9, 31.0) + 0.362 g(λ; 442.0, 16.0, 26.7) − 0.065 g(λ; 501.1, 20.4, 26.2)
    ///     ȳ =  0.821 g(λ; 568.8, 46.9, 40.5) + 0.286 g(λ; 530.9, 16.3, 31.1)
    ///     z̄ =  1.217 g(λ; 437.0, 11.8, 36.0) + 0.681 g(λ; 459.0, 26.0, 13.8)
    ///
    /// ȳ is the eye's brightness response (luminous efficiency): peak 1 near 555 nm (green).
    /// x̄ peaks near 600 nm (and has a small second bump in the blue), z̄ near 445 nm.
    /// The fit is smooth for every wavelength and tiny outside 380–780 nm, but only that
    /// span is meaningful (`Wavelength::visibleRange`). The third x̄ lobe is subtracted, so x̄
    /// dips to about 0 near 500 nm; it is not clamped here.
    ///
    /// Use when: computing how the eye responds to a pure colour of light
    /// Tags: CIE 1931, color matching functions, xyz, standard observer, spectral response
    /// Node: none (renderer shading)
    static Vector3 colorMatching(Wavelength wavelength);

    /// The linear sRGB light of a pure spectral line of unit energy at `wavelength`, negative
    /// channels clamped to 0 (Swift `linearRGB(at:)`).
    ///
    /// How: XYZ = the colour matching values; then, with the sRGB primaries and D65 white
    /// (IEC 61966-2-1, Annex B, 4-digit matrix):
    ///
    ///     R =  3.2406 X − 1.5372 Y − 0.4986 Z
    ///     G = −0.9689 X + 1.8758 Y + 0.0415 Z
    ///     B =  0.0557 X − 0.2040 Y + 1.0570 Z
    ///
    /// Out of gamut. A pure spectral colour is more saturated than any mix of the three
    /// screen primaries, so some channel always comes out negative (for example red is
    /// negative for blue-green light). Those channels are **clamped to 0**, the simplest way to
    /// get a displayable colour of nearly the same hue; the other channels are untouched.
    /// The result is the light's *amount* too: it is dim at the ends of the
    /// spectrum, and the fit's tails beyond about 700 nm are not accurate to the right hue. Use
    /// `displayColor` for a colour at full brightness.
    ///
    /// Use when: turning a wavelength into a linear light color for adding up dispersed rays
    /// Tags: wavelength to RGB, spectrum to RGB, spectral color, linear RGB, dispersion color, rainbow
    /// Node: none (renderer shading)
    static Vector3 linearRGB(Wavelength wavelength);

    /// The wavelengths whose hue `displayColor` computes; others use the nearest end.
    static constexpr WavelengthRange reliableHueRange{400, 680};

    /// The hue of `linearRGB` scaled so its brightest channel is 1: the colour to paint a ray
    /// of that wavelength, whatever its brightness (Swift `displayColor(at:)`).
    ///
    /// Beyond the ends of `reliableHueRange` the hue is held at the nearest end. Past
    /// 680 nm the true colour is the same deep red all the way out (red channel only), but
    /// the fit's x̄ tail dies faster than its ȳ tail, so a small green channel would take
    /// over after about 715 nm and the end of the spectrum would turn green. Below 400 nm
    /// the fit itself goes wrong: its red channel turns negative below about 390 nm (−0.003
    /// at 380 nm), so the clamped colour would become pure blue instead of violet. Even at
    /// 400 nm the fit's red-to-blue ratio is 0.08 where the CIE tables give about 0.165, so
    /// the violet end is bluer than the true colour.
    ///
    /// Use when: drawing the visible color of a ray with a known wavelength
    /// Tags: wavelength color, rainbow color, display color, spectrum color, prism ray color
    /// Node: none (renderer shading)
    static Vector3 displayColor(Wavelength wavelength);
};

} // namespace editor
