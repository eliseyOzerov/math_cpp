// Wobble: a seeded noise field that moves points a little, and the seeded values it is made of.
#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>

#include "../Numerics/SeededRandom.h"
#include "Warp2D.h"

namespace editor {

namespace {

/// Gradient noise reaches about ±0.7 in 2D; this brings its peaks to about ±1.
constexpr float noiseScale = 1.4f;

/// A float seed as 64 bits for the generator: its bit pattern mixed, so 0 and 1 give unrelated fields.
std::uint64_t seedBits(float seed) {
    std::uint64_t bits = std::bit_cast<std::uint32_t>(seed);
    return (bits + 1) * 0x9e3779b97f4a7c15ull;
}

/// 0…255 for the lattice cell `coordinate` (its floor, wrapped every 256).
int latticeIndex(float cellFloor) {
    if (!std::isfinite(cellFloor)) return 0;
    long long whole = static_cast<long long>(cellFloor);
    return static_cast<int>(((whole % 256) + 256) % 256);
}

/// The fade curve 6t⁵ − 15t⁴ + 10t³ that makes the noise smooth across lattice cells (Perlin, 2002).
float quinticFade(float amount) { return amount * amount * amount * (amount * (amount * 6 - 15) + 10); }

/// The 32-bit integer a number wraps to (the low 32 bits of its integer part).
std::uint32_t wrappedToUnsigned(std::int64_t value) { return static_cast<std::uint32_t>(value); }

} // namespace

/// How far apart a wobble's bends are, in units: 1000 ÷ frequency (frequency held above 1e-6).
///
/// Use when: choosing a frequency for a feature size; sizing the pieces of a warped curve
/// Tags: wavelength, wobble period, bend spacing
/// Node: planned (wobble)
float Warp2D::wavelength(const Wobble& wobble) { return 1000 / std::max(1e-6f, wobble.frequency); }

/// How finely a wobble should cut cubics for its warp to follow the field: a piece per half bend of
/// its finest octave, wavelength ÷ 2 ÷ 2^(octaves − 1).
///
/// Use when: setting `WarpSettings::step` for a wobble
/// Tags: wobble step, wobble detail, warp subdivision
/// Node: planned (wobble)
float Warp2D::wobbleStep(const Wobble& wobble) {
    float octaves = std::max(1.0f, std::round(wobble.octaves));
    return wavelength(wobble) / 2 / std::pow(2.0f, octaves - 1);
}

/// Seeded 2D gradient noise (Perlin, "Improving noise", 2002): about −1…1, smooth, zero at every
/// whole-number point, and the same for the same seed on every machine.
///
/// How: a seeded shuffle of 0…255 (Fisher–Yates over `SeededRandom`) picks one of eight unit
/// gradients for each lattice corner; the four corners of the cell around the point give the dot
/// products of their gradient with the offset to the point, blended with the quintic fade, and the
/// result is scaled by 1.4 so the peaks reach about ±1.
///
/// Deviation: graphics_cpp shuffled with mulberry32, so the fields differ from its goldens; the
/// range, smoothness and lattice zeros are the same.
///
/// Use when: a smooth random field for wobble, roughness or texture
/// Tags: noise, perlin noise, gradient noise, noise2, random field, simplex alternative
/// Node: planned (wobble)
std::function<float(float, float)> Warp2D::noise2(float seed) {
    SeededRandom random(seedBits(seed));
    std::vector<int> shuffle(256);
    for (int index = 0; index < 256; index += 1) shuffle[static_cast<size_t>(index)] = index;
    for (int index = 255; index > 0; index -= 1) {
        int other = std::min(index, static_cast<int>(std::floor(random.nextUnitFloat() * static_cast<float>(index + 1))));
        std::swap(shuffle[static_cast<size_t>(index)], shuffle[static_cast<size_t>(other)]);
    }
    std::vector<int> table = shuffle;
    table.insert(table.end(), shuffle.begin(), shuffle.end());
    std::vector<Vector2> gradients;
    for (int index = 0; index < 8; index += 1) {
        float turn = static_cast<float>(index) * std::numbers::pi_v<float> / 4;
        gradients.push_back(Vector2(std::cos(turn), std::sin(turn)));
    }
    return [table, gradients](float x, float y) {
        float cellX = std::floor(x);
        float cellY = std::floor(y);
        float offsetX = x - cellX;
        float offsetY = y - cellY;
        int column = latticeIndex(cellX);
        int row = latticeIndex(cellY);
        float fadeX = quinticFade(offsetX);
        float fadeY = quinticFade(offsetY);
        auto corner = [&](int hash, float dx, float dy) { return gradients[static_cast<size_t>(hash & 7)].dot(Vector2(dx, dy)); };
        int lowerLeft = table[static_cast<size_t>(table[static_cast<size_t>(column)] + row)];
        int upperLeft = table[static_cast<size_t>(table[static_cast<size_t>(column)] + row + 1)];
        int lowerRight = table[static_cast<size_t>(table[static_cast<size_t>(column + 1)] + row)];
        int upperRight = table[static_cast<size_t>(table[static_cast<size_t>(column + 1)] + row + 1)];
        float lower = corner(lowerLeft, offsetX, offsetY) + fadeX * (corner(lowerRight, offsetX - 1, offsetY) - corner(lowerLeft, offsetX, offsetY));
        float upper = corner(upperLeft, offsetX, offsetY - 1)
            + fadeX * (corner(upperRight, offsetX - 1, offsetY - 1) - corner(upperLeft, offsetX, offsetY - 1));
        return noiseScale * (lower + fadeY * (upper - lower));
    };
}

/// A wobble's displacement field as a warp.
///
/// How: two noise fields (seeds 2·seed + 1 and 2·seed + 2) give the shift in x and y; each of the
/// `octaves` (rounded, at least 1) samples them at twice the frequency of the one before and counts
/// half as much; the sum is divided by the total weight, so the shift stays within `amplitude`
/// (about, since the noise peaks near ±1):
///
///     p' = p + amplitude · Σₖ 2⁻ᵏ · noise(p · 2ᵏ / wavelength) / Σₖ 2⁻ᵏ
///
/// Use when: roughening an outline by hand-drawn wobble
/// Tags: wobble, displacement field, jitter, hand-drawn, roughen edges
/// Node: planned (wobble)
Warp2D::PlaneMap Warp2D::wobbleMap(const Wobble& wobble) {
    float size = wavelength(wobble);
    std::function<float(float, float)> shiftX = noise2(wobble.seed * 2 + 1);
    std::function<float(float, float)> shiftY = noise2(wobble.seed * 2 + 2);
    int octaves = std::max(1, static_cast<int>(std::lround(wobble.octaves)));
    float totalWeight = 0;
    for (int octave = 0; octave < octaves; octave += 1) totalWeight += std::pow(0.5f, static_cast<float>(octave));
    float amplitude = wobble.amplitude;
    return [=](Vector2 point) {
        float dx = 0;
        float dy = 0;
        for (int octave = 0; octave < octaves; octave += 1) {
            float frequency = std::pow(2.0f, static_cast<float>(octave)) / size;
            float weight = std::pow(0.5f, static_cast<float>(octave));
            dx += weight * shiftX(point.x * frequency, point.y * frequency);
            dy += weight * shiftY(point.x * frequency, point.y * frequency);
        }
        return Vector2(point.x + amplitude * dx / totalWeight, point.y + amplitude * dy / totalWeight);
    };
}

/// A wobble along a line `length` units long, as a wave of the place along it (0…1) around 1:
/// 1 ± `amplitude` at most (so here amplitude is a share, 0.3 for ±30%), bending `frequency` times
/// per thousand units. Read along a straight run of the field, or round a circle as long as the
/// line when `closed`, so a ring's ends meet.
///
/// How: the wave at u is 1 + (map(p) − p).x with p = (u·length, 0), or the point of a circle of
/// circumference `length` at angle 2πu when closed, and map the `wobbleMap`.
///
/// Use when: varying a pen width or a petal length with the same noise as the wobble
/// Tags: wobble wave, noise along line, varying width, closed noise loop
/// Node: planned (wobble)
Warp2D::Wave Warp2D::wobbleWave(const Wobble& wobble, float length, bool closed) {
    PlaneMap map = wobbleMap(wobble);
    float radius = length / (2 * std::numbers::pi_v<float>);
    return [=](float position) {
        float angle = 2 * std::numbers::pi_v<float> * position;
        Vector2 point = closed ? Vector2(radius * std::cos(angle), radius * std::sin(angle)) : Vector2(position * length, 0);
        return 1 + map(point).x - point.x;
    };
}

/// A seeded value for `key`, −1…1: the same for the same seed and key on every machine, and
/// unrelated between keys (mulberry32 on the two mixed).
///
/// How: t = (seed·0x9e3779b1 XOR key·0x85ebca6b) + 0x6d2b79f5 in 32-bit arithmetic, then mulberry32's
/// last steps `t = (t ^ t>>15)·(t|1); t ^= t + (t ^ t>>7)·(t|61); r = t ^ t>>14`, and the result
/// is r/2³²·2 − 1. Seed and key wrap to 32 bits as JavaScript's `|0` does.
///
/// Source: mulberry32, Tommy Ettinger (public domain); the mixing is graphics_cpp curves/warp `scatter`.
///
/// Use when: a jitter per item, as a glyph's bounce by its code point
/// Tags: scatter, seeded hash, mulberry32, per item random, stable random
/// Node: planned (scatter)
float Warp2D::scatter(std::int64_t seed, std::int64_t key) {
    std::uint32_t mixed = (wrappedToUnsigned(seed) * 0x9e3779b1u ^ wrappedToUnsigned(key) * 0x85ebca6bu) + 0x6d2b79f5u;
    mixed = (mixed ^ (mixed >> 15)) * (mixed | 1u);
    mixed ^= mixed + (mixed ^ (mixed >> 7)) * (mixed | 61u);
    std::uint32_t bits = mixed ^ (mixed >> 14);
    return static_cast<float>(static_cast<double>(bits) / 4294967296.0 * 2 - 1);
}

} // namespace editor
