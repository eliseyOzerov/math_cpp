// Following a ray of light through glass: at every surface it splits into a reflected and a
// refracted part. We follow the refracted one (or the reflected one when nothing can refract)
// and remember how much light it carries (Engine/Math/Optics/RefractingInterface.swift).
#pragma once

#include "../Geometry/Ray.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A ray of light and how much of the original light it still carries.
///
/// Use when: following a light ray that split into reflected and refracted parts
/// Tags: weighted ray, ray weight, light fraction, split ray, ray with energy
/// Node: none (value type)
struct WeightedRay {
    /// Where the ray is and where it is heading.
    Ray ray;
    /// The fraction of the original power this ray carries, 0…1. Starts at 1 and is
    /// multiplied by a Fresnel transmittance at every surface the ray goes through.
    float weight;

    bool operator==(const WeightedRay&) const = default;
};

/// What a ray does at the boundary between two media.
///
/// Use when: reading what a ray does at a glass or water boundary, whether it reflects,
///   refracts or both
/// Tags: ray continuation, refract or reflect, interface result, total internal reflection,
///   light split
/// Node: none (value type)
struct InterfaceContinuation {
    /// The ray that carries on from the hit point.
    WeightedRay continued;
    /// Whether the ray was reflected (total internal reflection) instead of refracted.
    bool wasReflected;

    bool operator==(const InterfaceContinuation&) const = default;
};

/// Light meeting a smooth boundary between two transparent media.
///
/// Sources: Glassner, *An Introduction to Ray Tracing* (1989), ch. 5, and Pharr, Jakob &
/// Humphreys, *PBRT* 4th ed. §9.3 and §9.5 (specular reflection and transmission).
///
/// Use when: tracing a light ray through a glass ball, a lens or the surface of water
/// Tags: refracting interface, glass surface, water surface, boundary, ray tracing,
///   transparent medium
struct RefractingInterface {
    /// The ray that continues past a surface hit: the **refracted** ray, carrying the
    /// transmitted power, or the **reflected** ray when light cannot enter the second medium
    /// (total internal reflection).
    ///
    /// How. At the hit point:
    ///
    ///     η = n₁/n₂,   cos θᵢ = |direction · normal|
    ///     refracted   = Refraction.refractedDirection(direction, normal, η)   (Snell)
    ///     weight'     = weight × (1 − R),  R from DielectricFresnel (unpolarized)
    ///     if refracted is nil (sin²θₜ > 1):
    ///         reflected = direction.reflectedAcrossPlane(withNormal:)   weight' = weight
    ///
    ///            incoming ╲ │ normal
    ///                      ╲│
    ///     ──────────────────●────────────── surface   medium 1 (n₁) above
    ///                       │╲                         medium 2 (n₂) below
    ///                       │ ╲ refracted, weight × (1 − R)
    ///
    /// Only the refracted part is followed, as a path-following tool for pictures and
    /// energy estimates. The weak reflected part (R, 4% at normal incidence into glass) is
    /// dropped rather than split off. Applying the unpolarized `1 − R` at each surface in
    /// turn is an approximation (as in PBRT): the first surface leaves the light partly
    /// polarized, and the exact two-surface value is `(Ts·Ts′ + Tp·Tp′)/2`. For a glass slab
    /// at 45° that is 0.90379 against 0.90204 here, 0.2% apart. Power, not radiance, is tracked: radiance would also
    /// scale by `(n₂/n₁)²`, which a ray going in and out of the same glass undoes.
    ///
    /// Conventions. `hitPoint` is where the ray meets the surface; the new ray starts exactly
    /// there (no offset), so the next intersection test must ignore a hit at distance 0.
    /// `surfaceNormal` has length 1 and may point to either side, exactly as in
    /// `Refraction.refractedDirection`. `refractiveIndexBefore` is the index of the medium
    /// the ray is in now, `refractiveIndexAfter` the one it enters, both positive.
    ///
    /// Use when: finding the rays that leave a point on glass or water, for caustics or a
    ///   ray-traced ball
    /// Tags: refract and reflect, ray continuation, Fresnel split, ray tracing,
    ///   transmitted ray, glass hit
    /// Node: none (renderer shading)
    static InterfaceContinuation continuation(
        WeightedRay incoming,
        Vector3 hitPoint,
        Vector3 surfaceNormal,
        float refractiveIndexBefore,
        float refractiveIndexAfter);
};

} // namespace editor
