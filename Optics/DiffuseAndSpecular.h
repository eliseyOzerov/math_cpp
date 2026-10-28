// The two classic local shading terms: Lambert's matte diffuse and Blinn's shiny highlight.
// Each is a BRDF (bidirectional reflectance distribution function): how much of the light
// arriving from one direction leaves toward another, per unit of solid angle.
// All directions follow the conventions in LightDirections.h
// (Engine/Math/Optics/DiffuseAndSpecular.swift).
#pragma once

#include "../Linear/Vector3.h"

namespace editor {

/// **Lambertian diffuse**: a perfectly matte surface that looks equally bright from every
/// viewing direction.
///
/// Source: Lambert, *Photometria* (1760).
///
/// Use when: shading chalk, clay, matte paint and unpolished surfaces
/// Tags: Lambert, diffuse, matte, Lambertian, albedo, diffuse shading, N dot L
struct LambertDiffuse {
    /// The BRDF value of a Lambertian surface: the same in every direction pair.
    ///
    /// Formula: `albedo / π`. The π makes the surface conserve energy: the outgoing light
    /// summed over the hemisphere, ∫ (albedo/π)·cos θ dω = (albedo/π)·π = `albedo`, so an
    /// albedo of 1 reflects exactly all the light it receives and never more.
    ///
    /// Conventions: `albedo` is the fraction of light reflected, 0 (black) to 1 (white), in
    /// linear color space (see `Color`).
    ///
    /// Use when: computing the constant diffuse BRDF for a matte color
    /// Tags: Lambert BRDF, diffuse BRDF, albedo over pi, matte reflectance, diffuse term
    /// Node: none (renderer shading)
    static float brdf(float albedo);

    /// Lambert's cosine factor: how much of the light arriving from `towardLight` lands on
    /// the surface. Light hitting at a slant is spread over more area.
    ///
    /// Formula: `max(0, normal · towardLight)`.
    ///
    ///      light ╲ │ normal          1 when the light is overhead,
    ///             ╲│ θ               0 at the horizon,
    ///     ─────────●────────         0 when the surface faces away (clamped, not negative)
    ///
    /// Conventions: both vectors have length 1.
    ///
    /// Use when: darkening a surface that faces away from the light
    /// Tags: N dot L, cosine factor, Lambert cosine, light angle, diffuse falloff,
    ///   saturate(dot)
    /// Node: none (renderer shading)
    static float cosineFactor(Vector3 normal, Vector3 towardLight);

    /// The reflected light per unit of incoming light from one direction: the BRDF times the
    /// cosine factor, `albedo/π · max(0, normal · towardLight)`. Multiply by the light's
    /// radiance (and its solid angle, for an area light) to get the surface's brightness.
    ///
    /// Use when: shading a matte surface in one call for a light direction
    /// Tags: diffuse lighting, Lambert shading, matte surface, diffuse reflectance,
    ///   albedo times cosine
    /// Node: none (renderer shading)
    static float reflectance(float albedo, Vector3 normal, Vector3 towardLight);
};

/// **Blinn–Phong specular**: a glossy highlight whose width is set by one number.
///
/// Source: Blinn, "Models of light reflection for computer synthesized pictures" (1977).
/// The normalization factor `(m+8)/(8π)` is the approximation given in Akenine-Möller, Haines
/// & Hoffman, *Real-Time Rendering* 3rd ed. §7.6, and in Hoffman's SIGGRAPH course notes
/// "Physics and Math of Shading" (equation numbers not cited: not verified here). It is
/// not the NDF normalization `(m+2)/(2π)` of RTR4 §9.8, which is a different quantity.
///
/// Use when: adding a shiny highlight to plastic or a ball
/// Tags: Blinn-Phong, specular, highlight, shininess, glossy, specular highlight
struct BlinnPhong {
    /// The shape of the highlight: how closely the half vector lines up with the normal.
    ///
    /// Formula: `max(0, normal · h) ^ shininess` where `h = halfVector(towardLight,
    /// towardViewer)`.
    ///
    /// It is 1 when the viewer sits exactly in the mirror direction of the light (then
    /// `h = normal`) and falls off as they separate. A larger `shininess` gives a tighter
    /// highlight: 1 is very soft, 100 is a small spot, 1000 near a mirror. Unlike the Phong
    /// model's `reflected · viewer`, the half-vector form stays symmetric in light and
    /// viewer.
    ///
    /// Conventions: unit vectors; `shininess ≥ 0`. A shininess of 0 gives 1 everywhere the
    /// half vector is above the horizon. The result is 0 when the half vector points below
    /// the surface, or when either the light or the viewer is below it
    /// (`normal · towardLight ≤ 0` or `normal · towardViewer ≤ 0`).
    ///
    /// Use when: shaping the highlight on a shiny ball or a plastic object
    /// Tags: specular lobe, half vector power, highlight shape, shininess, N dot H,
    ///   Blinn-Phong
    /// Node: none (renderer shading)
    static float lobe(Vector3 normal, Vector3 towardLight, Vector3 towardViewer, float shininess);

    /// The factor that keeps the highlight's energy about constant as it narrows.
    ///
    /// Formula: `(shininess + 8) / (8π)`. A tight highlight is brighter at its peak than a
    /// wide one; without this factor, raising the shininess would make a surface reflect
    /// less and less light. It is an approximation chosen to keep the total reflected energy
    /// about constant, not an exact identity.
    ///
    /// Derivation of the exact value. Take the viewer along the normal, so the half vector
    /// bisects the normal and the light at angle θ, and `normal · h = cos(θ/2)`. The
    /// directional albedo of `k·(normal · h)^m` is
    ///
    ///     2π ∫₀^{π/2} k·cos^m(θ/2)·cos θ·sin θ dθ
    ///
    /// Substitute φ = θ/2, then u = cos φ (so cos θ = 2u² − 1, sin θ dθ = −4u du):
    ///
    ///     = 8π·k ∫_{1/√2}^{1} u^{m+1}(2u² − 1) du
    ///     = 8π·k·[2u^{m+4}/(m+4) − u^{m+2}/(m+2)] from 1/√2 to 1
    ///     = 8π·k·(m + 2^(−m/2)) / ((m+2)(m+4))
    ///
    /// Setting this to 1 gives `k = (m+2)(m+4) / (8π·(2^(−m/2) + m))`, the exact
    /// normalization at normal incidence. For large m, `2^(−m/2)` vanishes and k tends to
    /// `(m+2)(m+4)/(8π·m)`, which `(m+8)/(8π)` approximates closely: the albedo it gives is
    /// `(m+8)(m + 2^(−m/2)) / ((m+2)(m+4))`, exactly 1 at m = 0, at most about 1.075 (near
    /// m = 8 to 10), and 1.002 at m = 1000. So it can overshoot energy by up to 7.5%.
    /// Conventions: `shininess ≥ 0`.
    ///
    /// Use when: keeping a highlight from getting brighter as it narrows
    /// Tags: normalization factor, energy conservation, Blinn-Phong normalization,
    ///   shininess scale, (n+8)/8pi
    /// Node: none (renderer shading)
    static float normalizationFactor(float shininess);

    /// The normalized Blinn–Phong BRDF value, without the cosine factor:
    /// `normalizationFactor × lobe`. Multiply by `max(0, normal · towardLight)` and the
    /// specular color (see `SchlickFresnel`) when lighting. Symmetric: swapping
    /// `towardLight` and `towardViewer` gives the same value.
    ///
    /// Use when: computing the full specular reflection of a glossy surface for one light
    /// Tags: Blinn-Phong BRDF, specular BRDF, glossy reflection, highlight, normalized
    ///   specular
    /// Node: none (renderer shading)
    static float brdf(Vector3 normal, Vector3 towardLight, Vector3 towardViewer, float shininess);
};

} // namespace editor
