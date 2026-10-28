// Gradient (Perlin) noise in 3D (Engine/Math/Fields/GradientNoise.swift).
#pragma once

#include <array>
#include <cstdint>

#include "../Linear/Vector3.h"

namespace editor {

/// Perlin's smooth ease from 0 to 1: flat at both ends, so blending with it leaves no crease
/// where one lattice cell meets the next. (Swift `Float.quinticFade`.)
///
/// How (Perlin 2002, *Improving Noise*): 6t⁵ − 15t⁴ + 10t³ = t³ (t (6t − 15) + 10). It is 0 at
/// t = 0, 1 at t = 1, 0.5 at t = 0.5, and its first **and second** derivatives are 0 at both ends.
/// The older cubic 3t² − 2t³ has a zero first derivative only, which shows as faint grid lines in
/// lighting that uses the noise's slope.
///
/// Conventions: meant for `fraction` in 0 … 1; outside it the polynomial keeps going (it is not
/// clamped).
///
/// Use when: blending between two values with no visible seam at the ends; easing a transition that must also have smooth slope
/// Tags: quintic fade, smootherstep, fade curve, Perlin fade, ease in out, smoothstep (the quintic version)
/// Node: inside displace, curveNoise, roughenField
float quinticFade(float fraction);

/// The value inside a unit cube blended from the values at its 8 corners: **trilinear
/// interpolation**. (Swift `Float.trilinearBlend`.)
///
/// How: `corners[i + 2j + 4k]` is the value at the corner (i, j, k), each of i, j, k being 0 or 1.
/// Blend the 4 pairs along x by `amounts.x` (8 values → 4), then the 2 pairs along y by
/// `amounts.y` (4 → 2), then the last pair along z by `amounts.z` (2 → 1), each step a linear
/// blend start + (end − start)·amount. The order of the axes does not change the exact result,
/// only the rounding.
///
/// Conventions: `amounts` in 0 … 1 for a point inside the cube; (0, 0, 0) gives corner 0 exactly
/// and (1, 1, 1) gives corner 7.
///
/// Use when: blending lattice values for noise; sampling a 3D grid (a density or a colour table) between its points
/// Tags: trilinear interpolation, trilinear blend, 3D lerp, interpolate cube corners, voxel interpolation, texture3D linear filtering
/// Node: inside displace, curveNoise, roughenField
float trilinearBlend(const std::array<float, 8>& corners, Vector3 amounts);

/// **Gradient noise** (often just "Perlin noise") in 3D: a smooth, repeatable, random-looking
/// field. It gives every point in space a number, near 0, that changes gently from place to place.
/// The same seed gives the same field, and the same field as the Swift engine's `GradientNoise`.
///
/// The idea (Perlin 1985; improved in Perlin 2002, *Improving Noise*). Space is cut into unit
/// cubes by the whole-number *lattice*. Each lattice point gets a random **gradient**: a direction
/// chosen from the 12 edge midpoints of a cube, (±1, ±1, 0), (±1, 0, ±1), (0, ±1, ±1). For a point
/// inside a cube:
///
/// 1. Each of the 8 corners contributes gradient · (point − corner).
/// 2. The 8 contributions are blended by trilinear interpolation (a blend in x, then y, then z),
///    with each fraction first passed through `quinticFade`.
///
/// That makes the field exactly **0 at every lattice point**, smooth (the fade has zero first and
/// second derivative at the cube faces), and with no favoured direction visible.
///
/// The random gradient of a lattice point is chosen by an integer hash of its coordinates and the
/// seed (`pcg3d` of Jarzynski & Olano, *Hash Functions for GPU Rendering*, JCGT 2020). No
/// permutation table is stored and coordinates may be negative.
///
/// Conventions: the lattice points are the integer coordinates, so one unit of the input is one
/// cube; scale the point first for finer or coarser features. Values lie in −1.0364 … 1.0364 (the
/// bound is max Σ weightᵢ · max over gradients (gradient · (point − cornerᵢ))). Float precision:
/// the lattice coordinates are limited to ±2²⁴, so keep coordinates within a few thousand for
/// smooth results.
///
/// Use when: bumpy rocks, wobbly branches, clouds, terrain heights, any irregularity that should look natural but repeat for a given seed
/// Tags: noise, Perlin noise, gradient noise, improved noise, 3D noise, procedural noise, fBm, Mathf.PerlinNoise, noise(), snoise
/// Node: inside displace, curveNoise, roughenField
class GradientNoise {
public:
    /// The seed that selects one field out of all the possible ones.
    std::uint64_t seed;

    /// The noise field for `seed`. Different seeds give different, unrelated fields.
    explicit GradientNoise(std::uint64_t seed);

    bool operator==(const GradientNoise&) const = default;

    /// The random gradient attached to the lattice point (x, y, z): one of 12 vectors of length
    /// √2, fixed by the seed and the three integers.
    ///
    /// How: hash the three integers (plus the seed words) with `pcg3d`, then take the first hash
    /// word modulo 12 to pick a direction from the table. The modulo has a bias of about 12 / 2³²
    /// per direction, far below anything visible.
    ///
    /// Use when: inspecting or testing the field; building a noise variant on the same lattice
    /// Tags: lattice gradient, random gradient, gradient vector, hash to gradient, Perlin gradient
    /// Node: inside displace, curveNoise, roughenField
    Vector3 latticeGradient(std::int64_t x, std::int64_t y, std::int64_t z) const;

    /// The noise value at `point`: 0 at every lattice point, smooth in between, in
    /// −1.0364 … 1.0364.
    ///
    /// How: with `cell` the lattice corner at or below the point and fraction = point − cell
    /// (each component 0 … 1), the contribution of the corner cell + (i, j, k) is
    /// gradient · (fraction − (i, j, k)). With u, v, w the quintic fades of the fraction's x, y, z:
    /// value = blend over z by w of (blend over y by v of (blend over x by u of the corners)).
    ///
    /// Conventions: Z-up, any real input. A point with a non-finite component gives NaN.
    /// Coordinates beyond ±2²⁴ are held at the edge of the lattice.
    ///
    /// Use when: a smooth random height, density or offset at a position; displacing a mesh
    /// Tags: noise value, Perlin noise, sample noise, 3D gradient noise, noise(x, y, z), snoise
    /// Node: noise
    float value(Vector3 point) const;

    /// **Fractal Brownian motion** (fBm): several copies of the noise added together, each at a
    /// higher frequency and a lower strength, for detail at every scale.
    ///
    /// How (Musgrave, in Ebert et al. 2003, *Texturing and Modeling*), for octave
    /// i = 0 … octaves − 1: fbm(p) = Σ gainⁱ · noise(p · lacunarityⁱ). With the defaults, octave 1
    /// has twice the frequency and half the amplitude of octave 0, and so on.
    ///
    /// Conventions: `octaves` from 1 to `maximumOctaves` (24); a value below 1 is treated as 1 and
    /// one above 24 as 24. Octaves whose scaled point no longer fits in a float are left out
    /// instead of turning the sum into NaN. A non-finite `point` gives NaN. The result is **not**
    /// divided by the total strength, so it can reach beyond ±1.
    ///
    /// Use when: natural-looking bumps; terrain; the roughness of a surface at several sizes
    /// Tags: fBm, fractal noise, fractional Brownian motion, octaves, lacunarity, gain, persistence, turbulence, fractal Perlin
    /// Node: displace, curveNoise
    float fractalValue(Vector3 point, std::int64_t octaves = 5, float lacunarity = 2, float gain = 0.5f) const;

    /// The most octaves `fractalValue` adds up; more add nothing.
    static constexpr int maximumOctaves = 24;

private:
    /// Three hash words made from the seed, mixed into every lattice hash so that two seeds give
    /// unrelated fields.
    std::array<std::uint32_t, 3> seedWords;
};

} // namespace editor
