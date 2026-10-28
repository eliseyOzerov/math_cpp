// Gradient noise, with values worked out by hand
// (EditorTests/Math/Fields/GradientNoise3DTests.swift).
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "../Fields/GradientNoise.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float infinity = std::numeric_limits<float>::infinity();
constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();

const GradientNoise& noise() {
    static const GradientNoise field(1234);
    return field;
}

/// A random point with each coordinate in `lower`…`upper`, drawn x then y then z.
Vector3 randomPoint(SeededRandom& random, float lower, float upper) {
    float x = random.nextFloat(lower, upper);
    float y = random.nextFloat(lower, upper);
    float z = random.nextFloat(lower, upper);
    return Vector3(x, y, z);
}

} // namespace

// MARK: Hand-worked

TEST(noiseQuinticFadeHasTheTextbookValues) {
    // 6t⁵ − 15t⁴ + 10t³ at t = 0, 1/2, 1, 1/4:  0, 1/2, 1, and
    // 6/1024 − 15/256 + 10/64 = 0.005859375 − 0.05859375 + 0.15625 = 0.103515625
    EXPECT(quinticFade(0) == 0);
    EXPECT(quinticFade(1) == 1);
    EXPECT(isApproximatelyEqual(quinticFade(0.5f), 0.5f));
    EXPECT(isApproximatelyEqual(quinticFade(0.25f), 0.103515625f));
    // Symmetric: fade(1 − t) = 1 − fade(t).
    EXPECT(isApproximatelyEqual(quinticFade(0.8f), 1 - quinticFade(0.2f)));
}

TEST(noiseGradientsAreTheTwelveEdgeDirections) {
    std::vector<Vector3> seen;
    for (int x = -3; x <= 3; x += 1) {
        for (int y = -3; y <= 3; y += 1) {
            for (int z = -3; z <= 3; z += 1) {
                Vector3 gradient = noise().latticeGradient(x, y, z);
                // Two components are ±1, one is 0: length √2.
                EXPECT(isApproximatelyEqual(gradient.length(), std::sqrt(2.0f)));
                int zeroCount = (gradient.x == 0 ? 1 : 0) + (gradient.y == 0 ? 1 : 0) + (gradient.z == 0 ? 1 : 0);
                EXPECT_EQUAL(zeroCount, 1);
                bool known = false;
                for (Vector3 other : seen) known = known || other == gradient;
                if (!known) seen.push_back(gradient);
            }
        }
    }
    EXPECT_EQUAL(seen.size(), size_t(12));
}

TEST(noiseCellCentreIsTheAverageOfTheCornerContributions) {
    // At the centre of a cell every fraction is 1/2, the fade is 1/2, so the trilinear blend is
    // the plain average of the 8 corner contributions gradient · (centre − corner), where
    // centre − corner = (±1/2, ±1/2, ±1/2).
    const int cells[][3] = {{0, 0, 0}, {3, -2, 5}, {-7, -7, -7}};
    for (const auto& cell : cells) {
        float total = 0;
        for (int i = 0; i <= 1; i += 1) {
            for (int j = 0; j <= 1; j += 1) {
                for (int k = 0; k <= 1; k += 1) {
                    Vector3 gradient = noise().latticeGradient(cell[0] + i, cell[1] + j, cell[2] + k);
                    total += gradient.dot(Vector3(0.5f - float(i), 0.5f - float(j), 0.5f - float(k)));
                }
            }
        }
        Vector3 centre = Vector3(float(cell[0]), float(cell[1]), float(cell[2])) + Vector3(0.5f, 0.5f, 0.5f);
        EXPECT(isApproximatelyEqual(noise().value(centre), total / 8));
    }
}

TEST(noiseNearALatticePointTheValueIsTheGradientTimesTheOffset) {
    // Close to the lattice point P the other 7 corners' fade weights are of order t³ (about 1e-9
    // here) and P has weight 1 − O(t³), so value ≈ gradient(P) · offset.
    Vector3 offset(1e-3f, 2e-3f, 1e-3f);
    const int lattices[][3] = {{2, 3, 4}, {-1, 0, 5}};
    for (const auto& lattice : lattices) {
        Vector3 origin(static_cast<float>(lattice[0]), static_cast<float>(lattice[1]), static_cast<float>(lattice[2]));
        Vector3 gradient = noise().latticeGradient(lattice[0], lattice[1], lattice[2]);
        EXPECT(isApproximatelyEqual(noise().value(origin + offset), gradient.dot(offset), 1e-5f));
    }
}

// MARK: Properties that always hold

TEST(noiseIsZeroAtEveryLatticePoint) {
    for (int x = -4; x <= 4; x += 1) {
        for (int y = -4; y <= 4; y += 1) {
            for (int z = -4; z <= 4; z += 1) {
                EXPECT(noise().value(Vector3(float(x), float(y), float(z))) == 0);
            }
        }
    }
}

TEST(noiseIsDeterministicAndTheSeedChangesTheField) {
    Vector3 sample(0.3f, 1.7f, -2.2f);
    EXPECT(GradientNoise(1234).value(sample) == noise().value(sample));
    int differing = 0;
    GradientNoise other(1235);
    for (int index = 0; index < 200; index += 1) {
        float scale = float(index);
        Vector3 point = Vector3(scale * 0.137f, scale * 0.291f, scale * 0.053f) + Vector3(0.21f, 0.37f, 0.43f);
        if (noise().value(point) != other.value(point)) differing += 1;
    }
    EXPECT(differing > 190);
}

TEST(noiseIsContinuous) {
    // The slope of the field is bounded: the gradient length √2 times the fade's steepest slope
    // (1.875) times a few terms. Measured: no step of 1e-3 changes the value by more than 0.01.
    SeededRandom random(8);
    float largestChange = 0;
    for (int round = 0; round < 5000; round += 1) {
        Vector3 point = randomPoint(random, -20, 20);
        Vector3 step = random.nextUnitVector3() * 1e-3f;
        largestChange = std::max(largestChange, std::fabs(noise().value(point + step) - noise().value(point)));
    }
    EXPECT(largestChange < 0.01f);
}

TEST(noiseJoinsSmoothlyAcrossCellFaces) {
    // Values just either side of a cell boundary agree: no jump.
    for (float boundary : {-2.0f, 0.0f, 3.0f}) {
        float below = noise().value(Vector3(boundary - 1e-5f, 0.4f, 0.7f));
        float above = noise().value(Vector3(boundary + 1e-5f, 0.4f, 0.7f));
        EXPECT(std::fabs(below - above) < 1e-4f);
    }
}

TEST(noiseValuesStayInsideMinusOneToOneAndFillMostOfIt) {
    SeededRandom random(77);
    float lowest = infinity;
    float highest = -infinity;
    float sum = 0;
    const int count = 200000;
    for (int round = 0; round < count; round += 1) {
        float value = noise().value(randomPoint(random, -50, 50));
        lowest = std::min(lowest, value);
        highest = std::max(highest, value);
        sum += value;
    }
    // The true extremes are ±1.0364; these 200 000 points measured −0.917 … 0.943.
    EXPECT(lowest >= -1.0364f && highest <= 1.0364f);
    EXPECT(lowest < -0.7f && highest > 0.7f);
    // The field has no preferred sign: mean near 0.
    EXPECT(std::fabs(sum / float(count)) < 0.01f);
}

TEST(noiseNonFinitePointGivesNaN) {
    EXPECT(std::isnan(noise().value(Vector3(notANumber, 0, 0))));
    EXPECT(std::isnan(noise().value(Vector3(0, infinity, 0))));
}

TEST(noiseFarAndNegativeCoordinatesWork) {
    // Negative coordinates use floor, not truncation: −0.25 is in cell −1, at 0.75.
    EXPECT(noise().value(Vector3(-0.25f, -0.25f, -0.25f)) != 0);
    EXPECT(noise().value(Vector3(-1, -1, -1)) == 0);
    EXPECT(std::isfinite(noise().value(Vector3(1e9f, 0, 0))));
    EXPECT(std::isfinite(noise().value(Vector3(-1e30f, 5, 5))));
}

// MARK: Fractal sum

TEST(noiseOneOctaveIsThePlainNoise) {
    Vector3 point(0.4f, 1.3f, 2.9f);
    EXPECT(noise().fractalValue(point, 1) == noise().value(point));
    EXPECT(noise().fractalValue(point, 0) == noise().value(point));
}

TEST(noiseOctavesAddAtDoubleFrequencyAndHalfStrength) {
    // Hand-written expansion of the formula for 3 octaves with the default lacunarity and gain.
    Vector3 point(0.4f, 1.3f, 2.9f);
    float expected = noise().value(point) + 0.5f * noise().value(point * 2) + 0.25f * noise().value(point * 4);
    EXPECT(isApproximatelyEqual(noise().fractalValue(point, 3), expected));
    float custom = noise().value(point) + 0.3f * noise().value(point * 3);
    EXPECT(isApproximatelyEqual(noise().fractalValue(point, 2, 3, 0.3f), custom));
}

TEST(noiseFractalSumIsZeroAtLatticePointsAndBoundedByTheStrengths) {
    EXPECT(noise().fractalValue(Vector3(2, -3, 1)) == 0);
    SeededRandom random(12);
    float largest = 0;
    for (int round = 0; round < 20000; round += 1) {
        largest = std::max(largest, std::fabs(noise().fractalValue(randomPoint(random, -9, 9), 6)));
    }
    // Each octave is at most 1.0364, so six add up to at most 1.0364 · (1 + 1/2 + … + 1/32)
    // = 1.0364 · 1.96875 ≈ 2.0404.
    EXPECT(largest <= 2.0404f);
    EXPECT(largest > 0.8f);
}

TEST(noiseOctavesAreCappedAtTheMaximum) {
    // Past 24 octaves the points are lattice points in float and add nothing; asking for more must
    // give the 24-octave sum, not loop for ever.
    Vector3 point(0.4f, 1.3f, 2.9f);
    float capped = noise().fractalValue(point, GradientNoise::maximumOctaves);
    EXPECT_EQUAL(GradientNoise::maximumOctaves, 24);
    EXPECT(noise().fractalValue(point, 25) == capped);
    EXPECT(noise().fractalValue(point, std::numeric_limits<std::int64_t>::max()) == capped);
}

TEST(noiseHugeLacunarityStopsBeforeTheFrequencyOverflows) {
    // A lacunarity of 1e20 overflows the frequency at the third octave: the sum stops there
    // instead of turning NaN.
    Vector3 point(0.4f, 1.3f, 2.9f);
    EXPECT(std::isfinite(noise().fractalValue(point, 8, 1e20f)));
    EXPECT(std::isnan(noise().fractalValue(Vector3(notANumber, 0, 0), 4)));
}

// MARK: Trilinear blend

TEST(noiseTrilinearBlendPicksCornersAndAveragesAtTheCentre) {
    // Corner i + 2j + 4k sits at (i, j, k).
    std::array<float, 8> corners = {1, 2, 3, 4, 5, 6, 7, 8};
    EXPECT(trilinearBlend(corners, Vector3(0, 0, 0)) == 1);
    EXPECT(trilinearBlend(corners, Vector3(1, 1, 1)) == 8);
    EXPECT(trilinearBlend(corners, Vector3(1, 0, 0)) == 2);
    EXPECT(trilinearBlend(corners, Vector3(0, 0, 1)) == 5);
    // Centre: the mean of the eight, 36 / 8 = 4.5.
    EXPECT(isApproximatelyEqual(trilinearBlend(corners, Vector3(0.5f, 0.5f, 0.5f)), 4.5f));
    // Values linear in x + 2y + 4z blend linearly: at (0.25, 0.5, 0.75), 1 + 0.25 + 1 + 3 = 5.25.
    EXPECT(isApproximatelyEqual(trilinearBlend(corners, Vector3(0.25f, 0.5f, 0.75f)), 5.25f));
}

TEST(noiseFractalSumHasFinerDetailThanThePlainNoise) {
    // Over a short step the 6-octave sum changes more than one octave does.
    SeededRandom random(31);
    float plainChange = 0;
    float fractalChange = 0;
    for (int round = 0; round < 2000; round += 1) {
        Vector3 point = randomPoint(random, -9, 9);
        Vector3 nearby = point + Vector3(0.02f, 0, 0);
        plainChange += std::fabs(noise().value(nearby) - noise().value(point));
        fractalChange += std::fabs(noise().fractalValue(nearby, 6) - noise().fractalValue(point, 6));
    }
    EXPECT(fractalChange > 1.5f * plainChange);
}
