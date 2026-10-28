// The shader library's noise, ported from graphics_metal's Noise.metal: value, Perlin and simplex
// noise, layered (fractal) noise, domain warping, curl, looping and flowing noise, sparse Gabor
// noise, and cellular (Voronoi) noise, all built on the hashes of Hashes.h, so a pattern looks the
// same here and in graphics_metal. The base noises return -1…1; the nodes remap them to 0…1.
//
// Name map from graphics_metal (each function's Tags line repeats its old name):
//   gradientAt → gradientAtCell, quintic → quinticSmoothing, noiseValue3 → valueNoise3,
//   noisePerlin3 → perlinNoise3, noiseSimplex3 → simplexNoise3, noiseBase → baseNoise,
//   noisePerlin4 → perlinNoise4, noiseLoop → loopingNoise, noiseFractal → fractalNoise,
//   noiseDomainWarp → domainWarpedNoise, noiseCurl → curlNoise, noiseFlow → flowNoise,
//   noiseGabor → gaborNoise, noiseBandLimited → bandLimitedNoise, Cellular → CellularResult,
//   cellularMetric → cellularDistance, cellularPoint → cellularFeaturePoint,
//   cellular → cellularNoise, cellularBorder → cellularBorderDistance,
//   whiteNoise → whiteNoise, whiteNoise3 → whiteNoiseVector3.
//
// Differences from graphics_metal, all on purpose: the constants PI and TAU are `pi<float>()` and
// `tau<float>()`, vector swizzles are written out, `mix` is `linearBlend`, `saturate` is
// `clamped(…, 0, 1)`, and float literals carry an `f` (Metal reads them as float anyway).
#pragma once

#include "Hashes.h"
#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

/// The quintic smoothing curve 6t⁵ − 15t⁴ + 10t³, for a number or each component of a vector.
///
/// Source: Ken Perlin, "Improving Noise", SIGGRAPH 2002.
///
/// How: t³ (t (6t − 15) + 10). It is 0 at 0 and 1 at 1, and its first and second derivatives are 0
/// at both ends, so noise blended with it has no creases at the lattice.
///
/// Conventions: meant for t in 0…1; outside it the polynomial keeps going (it is not clamped).
///
/// Use when: easing the blend between lattice points of noise; a smooth 0-to-1 ramp with flat ends
/// Tags: quintic, smootherstep, fade curve, Perlin fade, interpolation weight
/// Node: none (shader library)
template <typename Value>
inline Value quinticSmoothing(Value t) {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

/// A random unit direction for each whole-number cell of a 3D point.
///
/// How: normalize(hash33(cell) × 2 − 1 + 0.0001), the three hashes mapped from 0…1 to −1…1. The
/// 0.0001 keeps the vector from being exactly zero. It is a direction in the cube's directions, not
/// uniform on the sphere (corners of the cube are over-represented), as graphics_metal has it.
///
/// Conventions: the cell of a point is the floor of each coordinate; the result has length 1.
///
/// Use when: the gradients at the lattice points of Perlin and simplex noise
/// Tags: gradientAt, random gradient, lattice gradient, random direction per cell
/// Node: none (shader library)
inline Vector3Of<float> gradientAtCell(Vector3Of<float> cell) {
    return normalized(hash33(cell) * 2.0f - 1.0f + 0.0001f);
}

/// Value noise in 3D, from -1 to 1: random values at the whole-number points, blended smoothly.
///
/// Source: value noise, as in Lewis, "Generalized Stochastic Subdivision", 1987.
///
/// How: i = ⌊p⌋, f = quinticSmoothing(p − i). The value at lattice point c is hash31(c); the eight
/// corners of the cell are blended along x, then y, then z with `linearBlend` using the weights f.
/// The 0-to-1 result is mapped to −1…1 as 2 × result − 1.
///
/// Conventions: at a lattice point the result is 2 × hash31(point) − 1; the range is −1…1 and the
/// noise is continuous. Points are expected near the origin (the hashes' limit).
///
/// Use when: cheap blotchy noise; clouds, dirt masks; a base for layered noise
/// Tags: noiseValue3, value noise, smooth random, lattice noise, cloud
/// Node: none (shader library)
inline float valueNoise3(Vector3Of<float> p) {
    Vector3Of<float> cell = roundedDown(p);
    Vector3Of<float> f = quinticSmoothing(fractionalPart(p));
    float lowZ = linearBlend(linearBlend(hash31(cell), hash31(cell + Vector3Of<float>(1, 0, 0)), f.x),
                             linearBlend(hash31(cell + Vector3Of<float>(0, 1, 0)), hash31(cell + Vector3Of<float>(1, 1, 0)), f.x),
                             f.y);
    float highZ = linearBlend(linearBlend(hash31(cell + Vector3Of<float>(0, 0, 1)), hash31(cell + Vector3Of<float>(1, 0, 1)), f.x),
                              linearBlend(hash31(cell + Vector3Of<float>(0, 1, 1)), hash31(cell + Vector3Of<float>(1, 1, 1)), f.x),
                              f.y);
    return linearBlend(lowZ, highZ, f.z) * 2.0f - 1.0f;
}

/// Perlin gradient noise in 3D, from -1 to 1.
///
/// Source: Ken Perlin, "An Image Synthesizer", SIGGRAPH 1985; improved in "Improving Noise", 2002.
///
/// How: i = ⌊p⌋, f = p − i, w = quinticSmoothing(f). Each of the eight cell corners c (offset
/// (c&1, (c>>1)&1, (c>>2)&1)) contributes dot(gradientAtCell(i + offset), f − offset), the
/// gradient's pull toward the point; the eight are blended with `linearBlend` by w along x, y,
/// then z. The result is multiplied by 1.15 and clamped to −1…1 (the raw range is about ±0.87).
///
/// Conventions: exactly 0 at every lattice point (f − offset is 0 for the corner on the point and the
/// other gradients' weights are 0); continuous; range −1…1.
///
/// Use when: smooth natural-looking noise; terrain, clouds, marble; a base for layered noise
/// Tags: noisePerlin3, Perlin noise, gradient noise, improved noise
/// Node: none (shader library)
inline float perlinNoise3(Vector3Of<float> p) {
    Vector3Of<float> cell = roundedDown(p);
    Vector3Of<float> f = fractionalPart(p);
    Vector3Of<float> w = quinticSmoothing(f);
    float corners[8];
    for (int corner = 0; corner < 8; corner++) {
        Vector3Of<float> offset(float(corner & 1), float((corner >> 1) & 1), float((corner >> 2) & 1));
        corners[corner] = dotProduct(gradientAtCell(cell + offset), f - offset);
    }
    float lowZ = linearBlend(linearBlend(corners[0], corners[1], w.x), linearBlend(corners[2], corners[3], w.x), w.y);
    float highZ = linearBlend(linearBlend(corners[4], corners[5], w.x), linearBlend(corners[6], corners[7], w.x), w.y);
    return clamped(linearBlend(lowZ, highZ, w.z) * 1.15f, -1.0f, 1.0f);
}

/// Simplex noise in 3D, from -1 to 1: gradient noise on tetrahedra instead of cubes.
///
/// Source: Ken Perlin, "Noise hardware", SIGGRAPH 2001 course notes; the form follows Stefan Gustavson,
/// "Simplex noise demystified", 2005.
///
/// How: skew the point by 1/3 of its coordinate sum to find the simplex cell (i = ⌊v + Σv/3⌋), and
/// unskew by 1/6 to get x0, the point relative to the cell's first corner. The order of x0's
/// components picks which of the six tetrahedra holds the point, giving the second and third corner
/// offsets i1 and i2; the fourth is (1, 1, 1). For each corner k with offset vector xk, the
/// contribution is t⁴ · dot(gradientAtCell(corner), xk) with t = max(0.6 − |xk|², 0). The sum is
/// multiplied by 32 and clamped to −1…1.
///
/// Conventions: continuous (each contribution fades to 0 at distance √0.6 from its corner); range
/// −1…1; ties between components of x0 pick the tetrahedron as GLSL's `step` does (equal counts as
/// greater-or-equal).
///
/// Use when: noise with fewer directional artefacts than Perlin and cheaper in high dimensions
/// Tags: noiseSimplex3, simplex noise, Perlin simplex, tetrahedral noise
/// Node: none (shader library)
inline float simplexNoise3(Vector3Of<float> v) {
    const float skew = 1.0f / 3.0f;
    const float unskew = 1.0f / 6.0f;
    Vector3Of<float> cell = roundedDown(v + dotProduct(v, Vector3Of<float>(skew)));
    Vector3Of<float> x0 = v - cell + dotProduct(cell, Vector3Of<float>(unskew));
    // step(x0.yzx, x0.xyz): 1 where x0.x >= x0.y, x0.y >= x0.z, x0.z >= x0.x.
    Vector3Of<float> e(x0.x < x0.y ? 0.0f : 1.0f, x0.y < x0.z ? 0.0f : 1.0f, x0.z < x0.x ? 0.0f : 1.0f);
    Vector3Of<float> eShifted(e.z, e.x, e.y);
    Vector3Of<float> i1 = e * (1.0f - eShifted);
    Vector3Of<float> i2 = 1.0f - eShifted * (1.0f - e);
    Vector3Of<float> x1 = x0 - i1 + unskew;
    Vector3Of<float> x2 = x0 - i2 + 2.0f * unskew;
    Vector3Of<float> x3 = x0 - 1.0f + 3.0f * unskew;
    Vector4Of<float> t = largerOf(
        0.6f - Vector4Of<float>(dotProduct(x0, x0), dotProduct(x1, x1), dotProduct(x2, x2), dotProduct(x3, x3)),
        Vector4Of<float>(0.0f));
    t = t * t * t * t;
    float sum = t.x * dotProduct(gradientAtCell(cell), x0) + t.y * dotProduct(gradientAtCell(cell + i1), x1)
              + t.z * dotProduct(gradientAtCell(cell + i2), x2) + t.w * dotProduct(gradientAtCell(cell + 1.0f), x3);
    return clamped(sum * 32.0f, -1.0f, 1.0f);
}

/// Value (type 0), Perlin (1) or simplex (2) noise, chosen by number.
///
/// How: type 1 → perlinNoise3, type 2 → simplexNoise3, anything else → valueNoise3.
///
/// Use when: a node lets the user pick the kind of noise; layered noise over any base
/// Tags: noiseBase, noise type, choose noise
/// Node: none (shader library)
inline float baseNoise(Vector3Of<float> p, int type) {
    if (type == 1) return perlinNoise3(p);
    if (type == 2) return simplexNoise3(p);
    return valueNoise3(p);
}

/// Perlin-style gradient noise in 4D, from -1 to 1, for noise that loops through time.
///
/// Source: Perlin, "Improving Noise", 2002, extended to four dimensions.
///
/// How: i = ⌊p⌋, f = p − i, w = quinticSmoothing(f). Each of the 16 corners (offset bits
/// c&1, (c>>1)&1, (c>>2)&1, (c>>3)&1) has a gradient normalize(hash44(i + offset) × 2 − 1 + 0.0001)
/// and contributes dot(gradient, f − offset) times its multilinear weight, the product of the four
/// per-axis weights (w where the offset is 1, 1 − w where it is 0). The sum is multiplied by 1.3 and
/// clamped to −1…1.
///
/// Conventions: exactly 0 at lattice points; continuous; range −1…1.
///
/// Use when: noise that must repeat in time (see loopingNoise); animated 3D volumes
/// Tags: noisePerlin4, 4D noise, Perlin 4D, time noise
/// Node: none (shader library)
inline float perlinNoise4(Vector4Of<float> p) {
    Vector4Of<float> cell = roundedDown(p);
    Vector4Of<float> f = fractionalPart(p);
    Vector4Of<float> w = quinticSmoothing(f);
    float sum = 0.0f;
    for (int corner = 0; corner < 16; corner++) {
        Vector4Of<float> offset(float(corner & 1), float((corner >> 1) & 1), float((corner >> 2) & 1), float((corner >> 3) & 1));
        Vector4Of<float> gradient = normalized(hash44(cell + offset) * 2.0f - 1.0f + 0.0001f);
        Vector4Of<float> weights = linearBlendPerComponent(1.0f - w, w, offset);
        sum += weights.x * weights.y * weights.z * weights.w * dotProduct(gradient, f - offset);
    }
    return clamped(sum * 1.3f, -1.0f, 1.0f);
}

/// Perlin noise on a circle through time: after `period` it is back where it began.
///
/// How: angle = τ · time / max(period, 0.0001); the 4D noise is read at (p, cos(angle) · radius,
/// sin(angle) · radius). The time moves the point round a circle of the given radius in the last two
/// dimensions, and a full turn returns to the same point.
///
/// Conventions: range −1…1; `period` below 0.0001 is treated as 0.0001; a larger `radius` makes
/// the pattern change faster over the loop.
///
/// Use when: seamless looping animation of noise
/// Tags: noiseLoop, looping noise, seamless loop, periodic time
/// Node: none (shader library)
inline float loopingNoise(Vector2Of<float> p, float time, float period, float radius) {
    float angle = tau<float>() * time / largerOf(period, 0.0001f);
    return perlinNoise4(Vector4Of<float>(p.x, p.y, cosine(angle) * radius, sine(angle) * radius));
}

/// Layered noise (octaves added up), from 0 to 1. Modes: fBm (0), ridged multifractal (1),
/// turbulence (2), billow (3).
///
/// Source: Musgrave, "Texturing and Modeling: A Procedural Approach", for fractional Brownian
/// motion, ridged multifractal and turbulence; Perlin 1985 for turbulence.
///
/// How: for each octave i (the count is `octaves` clamped to 1…8 and truncated), n =
/// baseNoise(p + (1.7, 9.2, 3.1) · i, type) and p is multiplied by `lacunarity` after each; the
/// amplitude starts at 1 and is multiplied by `gain` after each. Per mode: fBm uses n; ridged
/// uses r = (1 − |n|)² · weight, with weight = clamp(2r, 0, 1) fed to the next octave; turbulence
/// uses |n|; billow uses 2|n| − 1. The result is Σ n·amplitude / max(Σ amplitude, 0.0001). Modes 0
/// and 3 are then mapped from −1…1 to 0…1 (× 0.5 + 0.5); modes 1 and 2 are clamped to 0…1.
///
/// Conventions: range 0…1; the sum is divided by the amplitude total so it stays bounded for any
/// gain; with one octave and mode 0 it is baseNoise × 0.5 + 0.5.
///
/// Use when: clouds, terrain, marble, any natural texture that has detail at several scales
/// Tags: noiseFractal, fBm, fractional Brownian motion, ridged multifractal, turbulence, billow, octaves
/// Node: none (shader library)
inline float fractalNoise(Vector3Of<float> p, int type, int mode, float octaves, float lacunarity, float gain) {
    float sum = 0.0f;
    float total = 0.0f;
    float amplitude = 1.0f;
    float weight = 1.0f;
    int count = int(clamped(octaves, 1.0f, 8.0f));
    for (int octave = 0; octave < count; octave++) {
        float n = baseNoise(p + Vector3Of<float>(1.7f, 9.2f, 3.1f) * float(octave), type);
        if (mode == 1) {
            float ridge = 1.0f - absolute(n);
            ridge *= ridge * weight;
            weight = clamped(ridge * 2.0f, 0.0f, 1.0f);
            n = ridge;
        } else if (mode == 2) {
            n = absolute(n);
        } else if (mode == 3) {
            n = absolute(n) * 2.0f - 1.0f;
        }
        sum += n * amplitude;
        total += amplitude;
        amplitude *= gain;
        p *= lacunarity;
    }
    float result = sum / largerOf(total, 0.0001f);
    return mode == 0 || mode == 3 ? result * 0.5f + 0.5f : clamped(result, 0.0f, 1.0f);
}

/// Layered noise warped by itself twice: x is the value, (y, z) how far the second warp moved the point.
///
/// Source: Inigo Quilez, "Domain Warping", iquilezles.org, 2002.
///
/// How: with f = fractalNoise(·, type, 0, …) at (point, evolution): q = (f(p), f(p + (5.2, 1.3)));
/// r = (f(p + 4·warp·q + (1.7, 9.2)), f(p + 4·warp·q + (8.3, 2.8))); value = f(p + 4·warp·r). The
/// result is (value, r.x, r.y).
///
/// Conventions: each component is in 0…1; `warp` 0 gives plain layered noise; `evolution` is the
/// third noise coordinate, moving the pattern through time.
///
/// Use when: swirling, marbled or flowing patterns; fake fluid and smoke textures
/// Tags: noiseDomainWarp, domain warp, warped fBm, marble, swirl
/// Node: none (shader library)
inline Vector3Of<float> domainWarpedNoise(Vector2Of<float> p, float evolution, float warp, int type, float octaves,
    float lacunarity, float gain) {
    Vector2Of<float> q(fractalNoise(Vector3Of<float>(p.x, p.y, evolution), type, 0, octaves, lacunarity, gain),
        fractalNoise(Vector3Of<float>(p.x + 5.2f, p.y + 1.3f, evolution), type, 0, octaves, lacunarity, gain));
    Vector2Of<float> warpedFirst = p + (4.0f * warp) * q;
    Vector2Of<float> r(
        fractalNoise(Vector3Of<float>(warpedFirst.x + 1.7f, warpedFirst.y + 9.2f, evolution), type, 0, octaves, lacunarity, gain),
        fractalNoise(Vector3Of<float>(warpedFirst.x + 8.3f, warpedFirst.y + 2.8f, evolution), type, 0, octaves, lacunarity, gain));
    Vector2Of<float> warpedSecond = p + (4.0f * warp) * r;
    float value = fractalNoise(Vector3Of<float>(warpedSecond.x, warpedSecond.y, evolution), type, 0, octaves, lacunarity, gain);
    return Vector3Of<float>(value, r.x, r.y);
}

/// The curl of simplex noise: a flow field that never bunches up or thins out.
///
/// How: with n(x, y) = simplexNoise3((x, y, evolution)) and ε = 0.01, the result is
/// ((n(x, y+ε) − n(x, y−ε)), (n(x−ε, y) − n(x+ε, y))) / (4ε), the central differences of the
/// noise rotated a quarter turn, as graphics_metal writes them (the differences are over a 2ε span and
/// the divisor is 4ε, so the field is half the textbook curl). Following the rotated gradient gives
/// a divergence-free flow.
///
/// Conventions: the length is bounded because simplex noise is clamped to −1…1 and has finite slope.
///
/// Use when: advecting particles or smoke along a swirly flow; distorting a texture without tearing
/// Tags: noiseCurl, curl noise, divergence free, flow field, Bridson
/// Node: none (shader library)
inline Vector2Of<float> curlNoise(Vector2Of<float> p, float evolution) {
    const float epsilon = 0.01f;
    float up = simplexNoise3(Vector3Of<float>(p.x, p.y + epsilon, evolution));
    float down = simplexNoise3(Vector3Of<float>(p.x, p.y - epsilon, evolution));
    float right = simplexNoise3(Vector3Of<float>(p.x + epsilon, p.y, evolution));
    float left = simplexNoise3(Vector3Of<float>(p.x - epsilon, p.y, evolution));
    return Vector2Of<float>(up - down, left - right) / (4.0f * epsilon);
}

/// Gradient noise whose gradients spin with time, so it flows instead of just fading.
///
/// Source: flow noise, Perlin and Neyret, "Flow Noise", SIGGRAPH Technical Sketches 2001.
///
/// How: 2D Perlin noise with the quintic blend, where the gradient at each lattice corner is the
/// unit vector at angle τ · hash21(corner) + time. Each corner contributes dot(gradient, f − offset);
/// the four are blended by w. The result is multiplied by 1.4 and clamped to −1…1.
///
/// Conventions: range −1…1; exactly 0 at lattice points for any time; time is in radians, so 2π
/// brings it back to the start.
///
/// Use when: animated noise that swirls in place (water, lava, energy)
/// Tags: noiseFlow, flow noise, rotating gradients, animated Perlin
/// Node: none (shader library)
inline float flowNoise(Vector2Of<float> p, float time) {
    Vector2Of<float> cell = roundedDown(p);
    Vector2Of<float> f = fractionalPart(p);
    Vector2Of<float> w = quinticSmoothing(f);
    float corners[4];
    for (int corner = 0; corner < 4; corner++) {
        Vector2Of<float> offset(float(corner & 1), float(corner >> 1));
        float angle = tau<float>() * hash21(cell + offset) + time;
        corners[corner] = dotProduct(Vector2Of<float>(cosine(angle), sine(angle)), f - offset);
    }
    return clamped(
        linearBlend(linearBlend(corners[0], corners[1], w.x), linearBlend(corners[2], corners[3], w.x), w.y) * 1.4f, -1.0f, 1.0f);
}

/// Sparse Gabor noise: a windowed wave at random spots. `anisotropy` 0 keeps every wave pointing at
/// `angle`, 1 gives each its own direction.
///
/// Source: Lagae, Lefebvre, Drettakis, Dutré, "Procedural Noise using Sparse Gabor Convolution",
/// SIGGRAPH 2009.
///
/// How: in the 3×3 cells around the point, two kernels per cell (neighbour hash seeds offset by
/// 91, 57 for the second). A kernel sits at a random spot h = hash22(neighbour) in its cell; with d
/// the vector from the spot to the point, it adds polarity · e^(−π · bandwidth² · |d|²) ·
/// cos(τ · frequency · dot(d, direction)). The direction is angle + (hash21(neighbour + 13) − 0.5) ·
/// τ · anisotropy, and the polarity is −1 when hash21(neighbour + 29) < 0.5, else +1. The sum is
/// multiplied by 0.7 and clamped to −1…1.
///
/// Conventions: range −1…1; `bandwidth` is the Gaussian's width in cells⁻¹ (larger is narrower);
/// `frequency` is waves per cell; `angle` is in radians.
///
/// Use when: stripy, brushed or wood-grain noise with controlled direction and frequency
/// Tags: noiseGabor, Gabor noise, sparse convolution, oriented noise, stripes
/// Node: none (shader library)
inline float gaborNoise(Vector2Of<float> p, float frequency, float bandwidth, float angle, float anisotropy) {
    Vector2Of<float> cell = roundedDown(p);
    Vector2Of<float> f = fractionalPart(p);
    float sum = 0.0f;
    for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
            for (int k = 0; k < 2; k++) {
                Vector2Of<float> neighbour = cell + Vector2Of<float>(float(i), float(j)) + Vector2Of<float>(float(k) * 91.0f, float(k) * 57.0f);
                Vector2Of<float> h = hash22(neighbour);
                Vector2Of<float> d = f - (Vector2Of<float>(float(i), float(j)) + h);
                float direction = angle + (hash21(neighbour + 13.0f) - 0.5f) * tau<float>() * anisotropy;
                float polarity = hash21(neighbour + 29.0f) < 0.5f ? -1.0f : 1.0f;
                sum += polarity * exponential(-pi<float>() * bandwidth * bandwidth * dotProduct(d, d))
                     * cosine(tau<float>() * frequency * dotProduct(d, Vector2Of<float>(cosine(direction), sine(direction))));
            }
        }
    }
    return clamped(sum * 0.7f, -1.0f, 1.0f);
}

/// Noise with its low frequencies taken out (Perlin minus a blur of itself), the band-limited
/// character of wavelet noise.
///
/// Source: after Cook and DeRose, "Wavelet Noise", SIGGRAPH 2005, which removes the low band; this
/// is graphics_metal's cheap approximation, not true wavelet noise.
///
/// How: blur = 0.2 · (perlinNoise3 at p, p ± 0.6 in x, p ± 0.6 in y); the result is
/// clamp((perlinNoise3(p) − blur) · 1.6, −1, 1).
///
/// Conventions: range −1…1; zero at lattice points of p only where the blur taps are also zero.
///
/// Use when: detail noise without large blotches; grain that tiles without visible low-frequency drift
/// Tags: noiseBandLimited, band limited noise, wavelet noise, high pass noise
/// Node: none (shader library)
inline float bandLimitedNoise(Vector3Of<float> p) {
    float blur = (perlinNoise3(p) + perlinNoise3(p + Vector3Of<float>(0.6f, 0.0f, 0.0f)) + perlinNoise3(p - Vector3Of<float>(0.6f, 0.0f, 0.0f))
                + perlinNoise3(p + Vector3Of<float>(0.0f, 0.6f, 0.0f)) + perlinNoise3(p - Vector3Of<float>(0.0f, 0.6f, 0.0f))) * 0.2f;
    return clamped((perlinNoise3(p) - blur) * 1.6f, -1.0f, 1.0f);
}

/// What cellular noise finds at a point: the two nearest feature-point distances and the nearest cell.
///
/// Use when: reading the result of cellularNoise
/// Tags: Cellular, Voronoi result, worley result
/// Node: none (shader library)
struct CellularResult {
    /// Distance to the nearest feature point.
    float nearest;
    /// Distance to the second nearest feature point.
    float second;
    /// A random number from 0 to 1 for the nearest cell.
    float id;
    /// The nearest feature point, in the coordinates given.
    Vector2Of<float> nearestPoint;
};

/// The distance from the origin to `offset` in a chosen metric: Euclidean (0), Manhattan (1) or Chebyshev (2).
///
/// How: metric 1 → |x| + |y|; metric 2 → max(|x|, |y|); anything else → √(x² + y²).
///
/// Use when: cellular noise with diamond-shaped or square cells
/// Tags: cellularMetric, distance metric, Manhattan, Chebyshev, Euclidean
/// Node: none (shader library)
inline float cellularDistance(Vector2Of<float> offset, int metric) {
    if (metric == 1) return absolute(offset.x) + absolute(offset.y);
    if (metric == 2) return largerOf(absolute(offset.x), absolute(offset.y));
    return lengthOf(offset);
}

/// The feature point of a cell, relative to the cell's corner, optionally wandering with time.
///
/// How: h = hash22(cell); moved = h + 0.3 · (sin(evolution + τ · h.yx) − sin(τ · h.yx)), which equals
/// h at evolution 0; the point is `linearBlend((0.5, 0.5), moved, jitter)`.
///
/// Conventions: `jitter` 0 puts every point at the cell centre (a regular grid), 1 at the random
/// spot; at evolution 0 the point stays in the cell for jitter 0…1.
///
/// Use when: the sites of Voronoi / Worley noise
/// Tags: cellularPoint, feature point, jittered grid, Worley site
/// Node: none (shader library)
inline Vector2Of<float> cellularFeaturePoint(Vector2Of<float> cell, float jitter, float evolution) {
    Vector2Of<float> h = hash22(cell);
    Vector2Of<float> turned(tau<float>() * h.y, tau<float>() * h.x);
    Vector2Of<float> moved = h + 0.3f * (sine(evolution + turned) - sine(turned));
    return linearBlend(Vector2Of<float>(0.5f), moved, jitter);
}

/// Cellular (Voronoi / Worley) noise: the nearest and second nearest feature points among the 3×3 cells around.
///
/// Source: Steven Worley, "A Cellular Texture Basis Function", SIGGRAPH 1996.
///
/// How: for each of the nine neighbour cells, the feature point is the cell plus
/// cellularFeaturePoint; d = cellularDistance(point − p, metric). The smallest d is `nearest` (with
/// the cell's id = hash21(cell + 7) and its point), the next `second`. Both start at 8.
///
/// Conventions: distances are in cells; the nearest point may lie outside the 3×3 block only when a
/// point wandered far (jitter 1, evolution large), which the search then misses.
///
/// Use when: cells, cracks, stones, scales, cobbles, organic cell patterns
/// Tags: cellular, Voronoi, Worley noise, F1 F2, cell id
/// Node: none (shader library)
inline CellularResult cellularNoise(Vector2Of<float> p, float jitter, float evolution, int metric) {
    Vector2Of<float> cell = roundedDown(p);
    CellularResult result;
    result.nearest = 8.0f;
    result.second = 8.0f;
    result.id = 0.0f;
    result.nearestPoint = p;
    for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
            Vector2Of<float> neighbour = cell + Vector2Of<float>(float(i), float(j));
            Vector2Of<float> point = neighbour + cellularFeaturePoint(neighbour, jitter, evolution);
            float distance = cellularDistance(point - p, metric);
            if (distance < result.nearest) {
                result.second = result.nearest;
                result.nearest = distance;
                result.id = hash21(neighbour + 7.0f);
                result.nearestPoint = point;
            } else if (distance < result.second) {
                result.second = distance;
            }
        }
    }
    return result;
}

/// Distance to the border between cells (Euclidean), 0 on the line.
///
/// Source: Inigo Quilez, "Voronoi - distances", iquilezles.org, 2012.
///
/// How: find the nearest feature point (cellularNoise, Euclidean). For every feature point in the
/// 5×5 cells around other than the nearest one (squared distance between them above 0.00001), the
/// border with it is the plane halfway between the two; the distance to it is
/// dot(½ (toNearest + toNeighbour), normalize(toNeighbour − toNearest)). The smallest is the result;
/// it starts at 8.
///
/// Conventions: distances are in cells; 0 exactly on a border; at most half the distance to the
/// nearest neighbouring site.
///
/// Use when: outlines of cells; cracks and mortar lines between stones
/// Tags: cellularBorder, Voronoi edges, cell border, distance to edge
/// Node: none (shader library)
inline float cellularBorderDistance(Vector2Of<float> p, float jitter, float evolution) {
    CellularResult nearestCell = cellularNoise(p, jitter, evolution, 0);
    Vector2Of<float> cell = roundedDown(p);
    float border = 8.0f;
    for (int j = -2; j <= 2; j++) {
        for (int i = -2; i <= 2; i++) {
            Vector2Of<float> neighbour = cell + Vector2Of<float>(float(i), float(j));
            Vector2Of<float> point = neighbour + cellularFeaturePoint(neighbour, jitter, evolution);
            Vector2Of<float> toNeighbour = point - p;
            Vector2Of<float> toNearest = nearestCell.nearestPoint - p;
            if (dotProduct(toNeighbour - toNearest, toNeighbour - toNearest) > 0.00001f) {
                border = smallerOf(border, dotProduct(0.5f * (toNearest + toNeighbour), normalized(toNeighbour - toNearest)));
            }
        }
    }
    return border;
}

/// A random number per pixel (`mode` 0) or per cell of `cells` across (1), changing with `seed`.
///
/// How: key = position for mode 0, else ⌊coordinates · cells⌋ (both components); the result is
/// hash31((key, seed)).
///
/// Conventions: range 0…1; `position` is in pixels, `coordinates` in 0…1 across the picture; mode 1
/// gives the same number for every point of a cell.
///
/// Use when: film grain, static, a random tint per tile
/// Tags: whiteNoise, white noise, random per pixel, per cell, grain
/// Node: none (shader library)
inline float whiteNoise(Vector2Of<float> position, Vector2Of<float> coordinates, float cells, float seed, int mode) {
    Vector2Of<float> key = mode == 0 ? position : roundedDown(coordinates * cells);
    return hash31(Vector3Of<float>(key.x, key.y, seed));
}

/// Three random numbers per pixel (`mode` 0) or per cell of `cells` across (1), changing with `seed`.
///
/// How: as whiteNoise, with hash33 in place of hash31.
///
/// Use when: a random colour per pixel or per tile
/// Tags: whiteNoise3, random colour, coloured static
/// Node: none (shader library)
inline Vector3Of<float> whiteNoiseVector3(Vector2Of<float> position, Vector2Of<float> coordinates, float cells, float seed, int mode) {
    Vector2Of<float> key = mode == 0 ? position : roundedDown(coordinates * cells);
    return hash33(Vector3Of<float>(key.x, key.y, seed));
}

} // namespace kernels
} // namespace editor
