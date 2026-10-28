// Schlick's approximation of how much light a surface reflects at a given viewing angle
// (Engine/Math/Optics/Microfacet.swift, `SchlickFresnel`). The first shared kernel: the same
// header runs in the core's tests and in a Metal shader.
#pragma once

#include "KernelPrelude.h"

namespace editor {
namespace kernels {

/// **Schlick's approximation** of the Fresnel reflectance.
///
/// Source: Christophe Schlick, "An Inexpensive BRDF Model for Physically-based Rendering",
/// Computer Graphics Forum 13(3), 1994.
///
/// Use when: shading glass, water, plastic or a microfacet highlight on the GPU or the CPU
/// Tags: Schlick Fresnel, fresnelSchlick, reflectance, F0, grazing reflection
/// Node: none (renderer shading)
struct SchlickFresnel {
    /// The reflectance when looking straight at the surface (θ = 0°), called `F0`, for light
    /// going from a medium with refractive index `firstIndex` into one with `secondIndex`.
    ///
    /// Formula: `((n₁ − n₂) / (n₁ + n₂))²`, the exact Fresnel result at normal incidence,
    /// with `n₁ = firstIndex` and `n₂ = secondIndex`.
    /// Air to glass (1 → 1.5): (−0.5/2.5)² = 0.04. Symmetric: the two indices can be swapped.
    ///
    /// Conventions: both indices must be positive (no material has an index ≤ 0). Swift checks
    /// that with a precondition; a kernel cannot throw, so here the caller checks, and an index
    /// ≤ 0 gives a meaningless number.
    ///
    /// Use when: getting the base reflectivity of glass, water or plastic from its
    ///   refractive index
    /// Tags: F0, normal incidence reflectance, specular color, IOR to F0, base
    ///   reflectivity, reflectance at zero
    /// Node: none (renderer shading)
    template <typename Real>
    static Real reflectanceAtNormalIncidence(Real firstIndex, Real secondIndex) {
        Real contrast = (firstIndex - secondIndex) / (firstIndex + secondIndex);
        return contrast * contrast;
    }

    /// The fraction of light reflected, given the cosine of the angle to the normal.
    ///
    /// Formula: `F0 + (1 − F0)·(1 − cos θ)⁵`, written as a blend from F0 to 1 by (1 − cos θ)⁵.
    ///
    ///     1 ┤                      ╱     F rises slowly from F0 at cos θ = 1
    ///       │                   ╱╱       (looking straight on) and shoots up to 1
    ///    F0 ┤──────────────────           at cos θ = 0 (grazing).
    ///       └──┬──────────────┬──
    ///       cos θ = 0       cos θ = 1
    ///
    /// Conventions: `cosine` is `normal · towardViewer` (or, for a microfacet, `halfVector ·
    /// towardViewer`), clamped to 0…1. `normalIncidenceReflectance` (F0) is 0…1; for metals it
    /// is a colour and the formula is applied per channel. The result is exactly F0 at cosine 1
    /// and exactly 1 at cosine 0. A NaN cosine gives NaN. Light inside the denser medium needs
    /// the cosine on the less dense side; Microfacet.swift explains why.
    ///
    /// Use when: getting how mirror-like a surface looks at a given viewing angle
    /// Tags: Schlick Fresnel, fresnelSchlick, reflectance, grazing reflection, view
    ///   angle, F0
    /// Node: none (renderer shading)
    template <typename Real>
    static Real reflectance(Real cosine, Real normalIncidenceReflectance) {
        Real oneMinusCosine = Real(1) - clamped(cosine, Real(0), Real(1));
        Real fifthPower = oneMinusCosine * oneMinusCosine * oneMinusCosine * oneMinusCosine * oneMinusCosine;
        return linearBlend(normalIncidenceReflectance, Real(1), fifthPower);
    }
};

} // namespace kernels
} // namespace editor
