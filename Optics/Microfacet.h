// The three ingredients of a physically based microfacet specular BRDF (Cook–Torrance), from
// Engine/Math/Optics/Microfacet.swift:
//
//     f = D(h) · F · G / (4 · (n·l) · (n·v))
//
//   D  how many tiny mirror facets face the right way (GGX),
//   F  how much light a facet reflects instead of absorbing (Schlick's Fresnel),
//   G  how much of that survives self-shadowing and masking (Smith).
//
// A rough surface is modelled as many tiny perfect mirrors ("microfacets") with random
// tilts. Directions follow the conventions in LightDirections.h.
//
// F is the shared kernel `kernels::SchlickFresnel` (Kernels/SchlickFresnel.h), which Swift
// keeps in this file as `SchlickFresnel`; it is not repeated here.
#pragma once

#include "../Linear/Vector3.h"

namespace editor {

// MARK: Roughness

/// The mapping from the artist-facing `roughness` to the `alpha` the formulas use.
///
/// Use when: letting artists set a 0 to 1 roughness slider that feeds the glossy-surface formulas
/// Tags: roughness, alpha, perceptual roughness, microfacet width, roughness slider
/// Node: none (renderer shading)
struct MicrofacetRoughness {
    /// The smallest `alpha` the formulas accept. A perfectly smooth surface (alpha 0) is a
    /// mirror with an infinitely tall, infinitely thin highlight, which float cannot hold;
    /// smaller values are raised to this one. It corresponds to roughness 0.01.
    static constexpr float minimumAlpha = 1e-4f;

    /// The width parameter `alpha` of the microfacet distribution for a `roughness` in 0…1.
    ///
    /// Formula: `alpha = roughness²`, the remapping of Burley, "Physically Based Shading at
    /// Disney" (2012), which makes roughness change the look about evenly across 0…1.
    /// Inputs outside 0…1 are clamped; the result is at least `minimumAlpha`. A NaN
    /// roughness also becomes `minimumAlpha` (the comparison with NaN fails, so the larger of
    /// the two keeps the minimum), which keeps every formula below finite.
    ///
    /// Use when: converting a roughness slider into the number GGX uses
    /// Tags: alpha, roughness squared, GGX alpha, perceptual roughness, Disney roughness
    /// Node: none (renderer shading)
    static float alpha(float roughness);
};

// MARK: Normal distribution

/// **GGX / Trowbridge–Reitz** distribution of microfacet normals.
///
/// Sources: Trowbridge & Reitz (1975); named GGX and applied to rendering by Walter, Marschner,
/// Li & Torrance, "Microfacet Models for Refraction through Rough Surfaces" (2007).
///
/// Use when: shaping the highlight of a rough metal, plastic or leather ball
/// Tags: GGX, Trowbridge-Reitz, normal distribution function, NDF, microfacet, specular highlight
/// Node: none (renderer shading)
struct GGXDistribution {
    /// `D(h)`: the density of microfacets whose normal is `halfVector`, per unit of solid
    /// angle. Large where `halfVector` is near the surface `normal`, with a long tail (Swift
    /// `density(normal:halfVector:roughness:)`).
    ///
    /// Formula, with `α = roughness²` and `c = normal · halfVector`:
    ///
    ///     D = α² / ( π · (c²·(α² − 1) + 1)² )
    ///
    /// Peak: at `c = 1` it is `1 / (π·α²)`, so smoother surfaces have taller, narrower
    /// highlights. It is normalized so that the facets' projected area adds up to the
    /// surface: `∫ D(h)·(normal · h) dω = 1` over the hemisphere.
    ///
    /// Conventions: unit vectors; `roughness` 0…1 (clamped). Zero roughness would be a
    /// delta spike, so alpha is raised to `MicrofacetRoughness::minimumAlpha`. The result is
    /// 0 when `halfVector` is at or below the horizon (`c ≤ 0`) and when `c` is NaN. A NaN
    /// roughness becomes `MicrofacetRoughness::minimumAlpha`.
    ///
    /// Use when: computing the highlight strength for a light, view and normal
    /// Tags: GGX density, NDF, normal distribution, D term, microfacet density, specular D
    /// Node: none (renderer shading)
    static float density(Vector3 normal, Vector3 halfVector, float roughness);

    /// The same as `density(normal, halfVector, roughness)`, taking `c = normal · halfVector`
    /// directly (Swift `density(cosineToNormal:roughness:)`).
    ///
    /// Use when: computing the highlight strength when the dot product with the normal is
    ///   already known
    /// Tags: GGX density, NDF, D term, normal distribution, cosine form, specular D
    /// Node: none (renderer shading)
    static float density(float cosineToNormal, float roughness);
};

// MARK: Masking and shadowing

/// **Smith masking–shadowing** for the GGX distribution, in its **height-correlated** form.
///
/// A microfacet can be hidden from the viewer (masking) or from the light (shadowing) by its
/// neighbours. `G` is the fraction of facets that are neither.
///
/// Source: Heitz, "Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs"
/// (JCGT 2014): Λ for GGX is in §5.3 and the height-correlated G₂ is in §6. (Equation
/// numbers are left out on purpose: they were not verified against the paper.)
///
/// Heitz's full G₂ also carries the factors χ⁺(h·l)·χ⁺(h·v) (the facet must face both
/// directions). They are omitted here: when `h` is the half vector of `l` and `v`, and both
/// are above the surface, they are always 1.
///
/// Why height-correlated and not the separable product G₁(l)·G₁(v): a facet high up is both
/// less masked and less shadowed, so the two events are correlated. Treating them as
/// independent loses energy; Heitz shows the correlated form matches ground-truth
/// simulation better, it is still symmetric in light and viewer, and it costs one more
/// addition. Ours is therefore always at least the separable value.
///
/// Use when: darkening rough surfaces at grazing angles where bumps hide each other
/// Tags: Smith, masking, shadowing, geometry term, G term, height-correlated, microfacet occlusion
/// Node: none (renderer shading)
struct SmithMasking {
    /// Heitz's Λ (lambda) for one direction: how much that direction is obscured. 0 looking
    /// straight on, growing without limit toward the horizon.
    ///
    /// Formula (GGX): `Λ = (√(1 + α²·tan²θ) − 1) / 2`, with `tan²θ = (1 − cos²θ)/cos²θ`
    /// (`tangentSquaredFromCosine`).
    ///
    /// Conventions: `cosine` is `normal · direction`. For `cosine ≤ 0` (below the horizon)
    /// the direction is fully hidden and infinity is returned. Λ is 0 at cosine 1 for
    /// any roughness, and tends to 0 for every cosine as roughness tends to 0. A NaN cosine
    /// fails the `cosine > 0` test and counts as hidden (infinity), so `G₁`, `G₂` and the
    /// visibility are 0 for NaN.
    ///
    /// Use when: building masking and shadowing from the per-direction obscuring amount
    /// Tags: lambda, Smith lambda, Heitz, masking helper, obscuring, GGX
    /// Node: none (renderer shading)
    static float lambda(float cosine, float roughness);

    /// `G₁`: the fraction of facets visible from one direction, `1 / (1 + Λ)`. 1 at normal
    /// incidence, for any roughness; 0 below the horizon.
    ///
    /// Use when: computing how visible a rough surface is from one direction
    /// Tags: G1, masking, single direction visibility, Smith G1, GGX masking
    /// Node: none (renderer shading)
    static float singleDirectionVisibility(float cosine, float roughness);

    /// `G₂`: the fraction of facets visible from both the light and the viewer.
    ///
    /// Formula: `1 / (1 + Λ(light) + Λ(viewer))`.
    ///
    /// Properties: between 0 and 1; exactly 1 when both directions are along the normal
    /// (at any roughness); never more than either `G₁`; symmetric when the two cosines are
    /// swapped. It is 0 if either direction is below the horizon.
    ///
    /// Conventions: `cosineToLight = normal · towardLight`, `cosineToViewer = normal ·
    /// towardViewer`; `roughness` 0…1.
    ///
    /// Use when: computing the shadowing and masking of a rough surface for a light and a viewer
    /// Tags: G2, geometry term, masking shadowing, Smith G2, height correlated, G term
    /// Node: none (renderer shading)
    static float maskingShadowing(float cosineToLight, float cosineToViewer, float roughness);

    /// The **visibility** term `V = G₂ / (4·(normal · l)·(normal · v))`: G₂ together with the
    /// BRDF's denominator, so that `f = D · F · V`. Shaders use it because the cosines
    /// cancel safely. Both cosines must be positive; otherwise (or for NaN) it is 0.
    ///
    /// Formula, in the cancelled form that never divides by a cosine:
    ///
    ///     V = 0.5 / ( cv·√(cl²(1 − α²) + α²) + cl·√(cv²(1 − α²) + α²) )
    ///
    /// Why it equals `G₂/(4·cl·cv)`. Write `sₗ = √(cl²(1 − α²) + α²)` and likewise `sᵥ`.
    /// From `Λ = (√(1 + α²·tan²θ) − 1)/2` and `1 + α²·tan²θ = (cos²θ + α²·sin²θ)/cos²θ
    /// = s²/cos²θ` we get `Λ = (s/c − 1)/2`. Then
    ///
    ///     1 + Λₗ + Λᵥ = (sₗ/cl + sᵥ/cv)/2 = (sₗ·cv + sᵥ·cl) / (2·cl·cv)
    ///     G₂ = 1/(1 + Λₗ + Λᵥ) = 2·cl·cv / (sₗ·cv + sᵥ·cl)
    ///     V  = G₂/(4·cl·cv)    = 1 / (2·(sₗ·cv + sᵥ·cl))
    ///
    /// The `cl·cv` in G₂ and in the BRDF's denominator cancel exactly. A grazing direction
    /// (cl → 0) then costs nothing: `sₗ` tends to α, so V tends to `0.5/(α·cv)`, finite and
    /// continuous, where the uncancelled form divides a small G₂ by a tiny product.
    /// (`cl²(1 − α²) + α²` is evaluated as `cl² + α²·sin²θ`, the same number.)
    ///
    /// Use when: plugging masking and shadowing into a specular formula without dividing
    ///   by the cosines yourself
    /// Tags: visibility term, V term, G over 4 N dot L N dot V, Smith visibility, specular visibility
    /// Node: none (renderer shading)
    static float visibility(float cosineToLight, float cosineToViewer, float roughness);
};

} // namespace editor
