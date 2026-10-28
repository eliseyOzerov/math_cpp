// Gradient (Perlin) noise in 3D (Engine/Math/Fields/GradientNoise.swift).
#include "GradientNoise.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../Numerics/SeededRandom.h"
#include "../Linear/Scalar.h"

namespace editor {

namespace {

/// The 12 directions a lattice point's gradient is chosen from: the midpoints of the edges of a
/// cube, seen from its centre (Perlin 2002). They have length √2, and no component is favoured.
const Vector3 gradientDirections[12] = {
    Vector3(1, 1, 0), Vector3(-1, 1, 0), Vector3(1, -1, 0), Vector3(-1, -1, 0),
    Vector3(1, 0, 1), Vector3(-1, 0, 1), Vector3(1, 0, -1), Vector3(-1, 0, -1),
    Vector3(0, 1, 1), Vector3(0, -1, 1), Vector3(0, 1, -1), Vector3(0, -1, -1),
};

/// The largest lattice index used, as a float. A float coordinate beyond it has no fraction left,
/// and converting a larger one to an integer would be undefined.
constexpr float largestLatticeIndex = 16777216.0f;

} // namespace

float quinticFade(float fraction) {
    return fraction * fraction * fraction * (fraction * (fraction * 6 - 15) + 10);
}

float trilinearBlend(const std::array<float, 8>& corners, Vector3 amounts) {
    float alongX[4] = {
        linearBlend(corners[0], corners[1], amounts.x),
        linearBlend(corners[2], corners[3], amounts.x),
        linearBlend(corners[4], corners[5], amounts.x),
        linearBlend(corners[6], corners[7], amounts.x),
    };
    float alongY[2] = {
        linearBlend(alongX[0], alongX[1], amounts.y),
        linearBlend(alongX[2], alongX[3], amounts.y),
    };
    return linearBlend(alongY[0], alongY[1], amounts.z);
}

GradientNoise::GradientNoise(std::uint64_t seed) : seed(seed), seedWords{} {
    SeededRandom random(seed);
    // Evaluated in order, as in Swift.
    seedWords[0] = static_cast<std::uint32_t>(random.nextUInt64());
    seedWords[1] = static_cast<std::uint32_t>(random.nextUInt64());
    seedWords[2] = static_cast<std::uint32_t>(random.nextUInt64());
}

Vector3 GradientNoise::latticeGradient(std::int64_t x, std::int64_t y, std::int64_t z) const {
    std::uint32_t hashX = static_cast<std::uint32_t>(x) ^ seedWords[0];
    std::uint32_t hashY = static_cast<std::uint32_t>(y) ^ seedWords[1];
    std::uint32_t hashZ = static_cast<std::uint32_t>(z) ^ seedWords[2];
    // pcg3d, Jarzynski & Olano (2020): a congruential step, then cross-mixing the three words so
    // no axis of the lattice shows a pattern, twice with an xor-shift between.
    hashX = hashX * 1664525u + 1013904223u;
    hashY = hashY * 1664525u + 1013904223u;
    hashZ = hashZ * 1664525u + 1013904223u;
    hashX += hashY * hashZ;
    hashY += hashZ * hashX;
    hashZ += hashX * hashY;
    hashX ^= hashX >> 16;
    hashY ^= hashY >> 16;
    hashZ ^= hashZ >> 16;
    hashX += hashY * hashZ;
    hashY += hashZ * hashX;
    hashZ += hashX * hashY;
    return gradientDirections[hashX % 12];
}

float GradientNoise::value(Vector3 point) const {
    if (!point.isFinite()) return std::numeric_limits<float>::quiet_NaN();
    // Hold the point inside the lattice's range first, so its fraction is a real fraction.
    float limit = largestLatticeIndex;
    Vector3 held(clamped(point.x, -limit, limit), clamped(point.y, -limit, limit),
                 clamped(point.z, -limit, limit));
    Vector3 floors(std::floor(held.x), std::floor(held.y), std::floor(held.z));
    std::int64_t cornerX = static_cast<std::int64_t>(floors.x);
    std::int64_t cornerY = static_cast<std::int64_t>(floors.y);
    std::int64_t cornerZ = static_cast<std::int64_t>(floors.z);
    Vector3 fraction = held - floors;
    Vector3 fade(quinticFade(fraction.x), quinticFade(fraction.y), quinticFade(fraction.z));

    // The 8 corners' contributions, gradient · (point − corner), numbered i + 2j + 4k for the
    // corner cell + (i, j, k).
    std::array<float, 8> contributions{};
    for (int k = 0; k <= 1; k += 1) {
        for (int j = 0; j <= 1; j += 1) {
            for (int i = 0; i <= 1; i += 1) {
                Vector3 gradient = latticeGradient(cornerX + i, cornerY + j, cornerZ + k);
                Vector3 cornerOffset(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k));
                contributions[static_cast<size_t>(i + 2 * j + 4 * k)] = gradient.dot(fraction - cornerOffset);
            }
        }
    }
    return trilinearBlend(contributions, fade);
}

float GradientNoise::fractalValue(Vector3 point, std::int64_t octaves, float lacunarity, float gain) const {
    if (!point.isFinite()) return std::numeric_limits<float>::quiet_NaN();
    float total = 0;
    float amplitude = 1;
    float frequency = 1;
    std::int64_t octaveCount = std::min<std::int64_t>(std::max<std::int64_t>(octaves, 1), maximumOctaves);
    for (std::int64_t octave = 0; octave < octaveCount; octave += 1) {
        Vector3 scaledPoint = point * frequency;
        if (!scaledPoint.isFinite()) break;
        total += amplitude * value(scaledPoint);
        amplitude *= gain;
        frequency *= lacunarity;
    }
    return total;
}

} // namespace editor
