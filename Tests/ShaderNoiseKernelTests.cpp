// The shader-library noise (Kernels/ShaderNoise.h) on the CPU: values worked out by hand where the
// formula gives them (lattice points, cell centres, regular cellular grids), and the properties each
// noise must keep: its output range, continuity, determinism and bounded octave sums. The same
// header is compiled as Metal by `make kernels` and compared on the GPU by `make gpu`.
#include <cmath>

#include "../Kernels/ShaderNoise.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;
using Kernel3 = kernels::Vector3Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }

/// Points of a jittered grid that avoid lattice points and cell borders.
Kernel3 samplePoint(int index) {
    float step = static_cast<float>(index);
    return Kernel3(step * 0.173f - 4.1f, step * 0.291f - 6.3f, step * 0.117f - 2.7f);
}
constexpr int sampleCount = 80;

/// The largest change of `noise` when the point moves by `distance` along each axis in turn.
template <typename Noise>
float largestStepChange(Noise noise, float distance) {
    float largest = 0.0f;
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 p = samplePoint(index);
        float here = noise(p);
        largest = std::fmax(largest, std::fabs(noise(Kernel3(p.x + distance, p.y, p.z)) - here));
        largest = std::fmax(largest, std::fabs(noise(Kernel3(p.x, p.y + distance, p.z)) - here));
        largest = std::fmax(largest, std::fabs(noise(Kernel3(p.x, p.y, p.z + distance)) - here));
    }
    return largest;
}

template <typename Noise>
bool staysWithin(Noise noise, float lower, float upper) {
    for (int index = 0; index < sampleCount; index += 1) {
        float value = noise(samplePoint(index));
        if (!(value >= lower && value <= upper)) return false;
    }
    return true;
}
} // namespace

TEST(quinticSmoothingHasFlatEndsAndAMidpointOfOneHalf) {
    // 0 at 0 and 1 at 1; at 0.5: 0.125 (0.5 (3 − 15) + 10) = 0.125 × 4 = 0.5;
    // at 0.25: 0.015625 (0.25 (1.5 − 15) + 10) = 0.015625 × 6.625 = 0.103515625.
    EXPECT(kernels::quinticSmoothing(0.0f) == 0.0f);
    EXPECT(kernels::quinticSmoothing(1.0f) == 1.0f);
    EXPECT(nearly(kernels::quinticSmoothing(0.5f), 0.5f));
    EXPECT(nearly(kernels::quinticSmoothing(0.25f), 0.103515625f));
    // Symmetric about the middle: s(1 − t) = 1 − s(t).
    EXPECT(nearly(kernels::quinticSmoothing(0.8f), 1.0f - kernels::quinticSmoothing(0.2f)));
    Kernel3 byVector = kernels::quinticSmoothing(Kernel3(0.0f, 0.5f, 1.0f));
    EXPECT(nearly(byVector.x, 0.0f) && nearly(byVector.y, 0.5f) && nearly(byVector.z, 1.0f));
}

TEST(gradientAtCellIsAUnitVectorThatDependsOnlyOnTheCell) {
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 g = kernels::gradientAtCell(samplePoint(index));
        EXPECT(nearly(kernels::lengthOf(g), 1.0f, 1e-5f));
    }
    // Every point of one cell gives the same gradient.
    Kernel3 first = kernels::gradientAtCell(Kernel3(2.1f, 3.2f, 4.3f));
    Kernel3 second = kernels::gradientAtCell(Kernel3(2.9f, 3.8f, 4.1f));
    EXPECT(first.x == second.x && first.y == second.y && first.z == second.z);
}

TEST(valueNoiseAtLatticePointsIsTheHashMappedToMinusOneToOne) {
    for (int index = 0; index < 20; index += 1) {
        Kernel3 lattice(static_cast<float>(index - 7), static_cast<float>(index % 5 - 2), static_cast<float>(3 - index % 4));
        EXPECT(nearly(kernels::valueNoise3(lattice), kernels::hash31(lattice) * 2.0f - 1.0f, 1e-6f));
    }
}

TEST(valueNoiseAtACellCentreIsTheAverageOfTheEightCorners) {
    // The quintic weight at 0.5 is 0.5 on every axis, so the trilinear blend is the plain mean.
    Kernel3 cell(1.0f, -2.0f, 3.0f);
    float sum = 0.0f;
    for (int corner = 0; corner < 8; corner += 1) {
        sum += kernels::hash31(Kernel3(cell.x + float(corner & 1), cell.y + float((corner >> 1) & 1), cell.z + float(corner >> 2)));
    }
    EXPECT(nearly(kernels::valueNoise3(Kernel3(1.5f, -1.5f, 3.5f)), sum / 8.0f * 2.0f - 1.0f, 1e-5f));
}

TEST(perlinNoiseIsZeroAtLatticePoints) {
    for (int index = 0; index < 20; index += 1) {
        Kernel3 lattice(static_cast<float>(index - 9), static_cast<float>(index % 7 - 3), static_cast<float>(5 - index % 6));
        EXPECT(nearly(kernels::perlinNoise3(lattice), 0.0f, 1e-6f));
    }
}

TEST(perlinNoiseAtACellCentreMatchesTheCornerFormula) {
    // At (0.5, 0.5, 0.5) every weight is 0.5, so the value is the mean of dot(gradient, f − offset)
    // over the eight corners, times 1.15 (unclamped for these cells).
    Kernel3 f(0.5f, 0.5f, 0.5f);
    float sum = 0.0f;
    for (int corner = 0; corner < 8; corner += 1) {
        Kernel3 offset(float(corner & 1), float((corner >> 1) & 1), float(corner >> 2));
        sum += kernels::dotProduct(kernels::gradientAtCell(Kernel3(2.0f, 3.0f, 4.0f) + offset), f - offset);
    }
    float expected = std::fmax(-1.0f, std::fmin(1.0f, sum / 8.0f * 1.15f));
    EXPECT(nearly(kernels::perlinNoise3(Kernel3(2.5f, 3.5f, 4.5f)), expected, 1e-5f));
}

TEST(perlinNoiseIsInRangeContinuousAndDeterministic) {
    auto noise = [](Kernel3 p) { return kernels::perlinNoise3(p); };
    EXPECT(staysWithin(noise, -1.0f, 1.0f));
    // A step of 0.001 changes the value by far less than 0.02 (slope below 20); an unsmoothed noise would jump by up to 2.
    EXPECT(largestStepChange(noise, 0.001f) < 0.02f);
    EXPECT(kernels::perlinNoise3(samplePoint(5)) == kernels::perlinNoise3(samplePoint(5)));
    // It is not constant.
    EXPECT(kernels::perlinNoise3(samplePoint(5)) != kernels::perlinNoise3(samplePoint(9)));
}

TEST(valueNoiseIsInRangeAndContinuous) {
    auto noise = [](Kernel3 p) { return kernels::valueNoise3(p); };
    EXPECT(staysWithin(noise, -1.0f, 1.0f));
    EXPECT(largestStepChange(noise, 0.001f) < 0.02f);
}

TEST(simplexNoiseAtTheOriginFollowsTheCornerFormula) {
    // v = 0: x0 = 0, the tetrahedron's other corners are (0,0,0) + 1/6, (1,1,1) − 1/3 and (1,1,1) − 1/2 per axis.
    // |x1|² = 3/36 so t1 = 0.6 − 1/12; |x2|² = 4/3 and |x3|² = 3/4 give t = 0; x0 itself contributes dot(g, 0) = 0.
    // sum = t1⁴ · dot(g(0), (1/6, 1/6, 1/6)), times 32.
    Kernel3 g = kernels::gradientAtCell(Kernel3(0.0f));
    float t1 = 0.6f - 1.0f / 12.0f;
    float expected = 32.0f * t1 * t1 * t1 * t1 * (g.x + g.y + g.z) / 6.0f;
    expected = std::fmax(-1.0f, std::fmin(1.0f, expected));
    EXPECT(nearly(kernels::simplexNoise3(Kernel3(0.0f)), expected, 1e-5f));
}

TEST(simplexNoiseIsInRangeContinuousAndDeterministic) {
    auto noise = [](Kernel3 p) { return kernels::simplexNoise3(p); };
    EXPECT(staysWithin(noise, -1.0f, 1.0f));
    EXPECT(largestStepChange(noise, 0.001f) < 0.05f);
    EXPECT(kernels::simplexNoise3(samplePoint(7)) == kernels::simplexNoise3(samplePoint(7)));
    EXPECT(kernels::simplexNoise3(samplePoint(7)) != kernels::simplexNoise3(samplePoint(11)));
}

TEST(baseNoiseChoosesByType) {
    Kernel3 p = samplePoint(3);
    EXPECT(kernels::baseNoise(p, 0) == kernels::valueNoise3(p));
    EXPECT(kernels::baseNoise(p, 1) == kernels::perlinNoise3(p));
    EXPECT(kernels::baseNoise(p, 2) == kernels::simplexNoise3(p));
    EXPECT(kernels::baseNoise(p, 9) == kernels::valueNoise3(p));
}

TEST(perlinNoise4IsZeroAtLatticePointsAndBounded) {
    for (int index = 0; index < 10; index += 1) {
        float n = static_cast<float>(index);
        EXPECT(nearly(kernels::perlinNoise4(Kernel4(n - 3.0f, 2.0f - n, n, 1.0f)), 0.0f, 1e-6f));
    }
    float largest = 0.0f;
    float largestStep = 0.0f;
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 p = samplePoint(index);
        float here = kernels::perlinNoise4(Kernel4(p.x, p.y, p.z, p.x * 0.5f));
        float moved = kernels::perlinNoise4(Kernel4(p.x, p.y, p.z, p.x * 0.5f + 0.001f));
        largest = std::fmax(largest, std::fabs(here));
        largestStep = std::fmax(largestStep, std::fabs(moved - here));
    }
    EXPECT(largest <= 1.0f);
    EXPECT(largest > 0.1f);
    EXPECT(largestStep < 0.02f);
}

TEST(loopingNoiseReturnsToItsStartAfterOnePeriod) {
    for (int index = 0; index < 20; index += 1) {
        Kernel3 p = samplePoint(index);
        float time = static_cast<float>(index) * 0.23f;
        float here = kernels::loopingNoise(Kernel2(p.x, p.y), time, 3.0f, 0.8f);
        float later = kernels::loopingNoise(Kernel2(p.x, p.y), time + 3.0f, 3.0f, 0.8f);
        EXPECT(nearly(here, later, 1e-3f));
        EXPECT(here >= -1.0f && here <= 1.0f);
    }
    // Half a period later it is a different value (the pattern does move).
    EXPECT(!nearly(kernels::loopingNoise(Kernel2(0.3f, 0.7f), 0.0f, 3.0f, 0.8f), kernels::loopingNoise(Kernel2(0.3f, 0.7f), 1.5f, 3.0f, 0.8f), 1e-3f));
    // Time 0 reads the 4D noise at (p, radius, 0).
    EXPECT(nearly(kernels::loopingNoise(Kernel2(0.3f, 0.7f), 0.0f, 3.0f, 0.8f), kernels::perlinNoise4(Kernel4(0.3f, 0.7f, 0.8f, 0.0f)), 1e-6f));
}

TEST(fractalNoiseWithOneOctaveIsTheBaseNoiseRemapped) {
    // One octave: sum / total = n, so mode 0 is n × 0.5 + 0.5, mode 3 is (2|n| − 1) × 0.5 + 0.5 = |n|,
    // mode 2 is |n| clamped, and mode 1 is ridge = (1 − |n|)² (weight 1) clamped.
    for (int type = 0; type < 3; type += 1) {
        Kernel3 p = samplePoint(4 + type);
        float n = kernels::baseNoise(p, type);
        EXPECT(nearly(kernels::fractalNoise(p, type, 0, 1.0f, 2.0f, 0.5f), n * 0.5f + 0.5f, 1e-6f));
        EXPECT(nearly(kernels::fractalNoise(p, type, 1, 1.0f, 2.0f, 0.5f), (1.0f - std::fabs(n)) * (1.0f - std::fabs(n)), 1e-6f));
        EXPECT(nearly(kernels::fractalNoise(p, type, 2, 1.0f, 2.0f, 0.5f), std::fabs(n), 1e-6f));
        EXPECT(nearly(kernels::fractalNoise(p, type, 3, 1.0f, 2.0f, 0.5f), std::fabs(n), 1e-6f));
    }
}

TEST(fractalNoiseTwoOctavesWeighsTheSecondByTheGain) {
    // fBm: (n0 + gain · n1) / (1 + gain). After octave 0, p is scaled by the lacunarity, and octave 1 adds
    // (1.7, 9.2, 3.1) × 1 to the scaled point.
    Kernel3 p = samplePoint(10);
    float lacunarity = 2.0f;
    float gain = 0.5f;
    float n0 = kernels::perlinNoise3(p);
    float n1 = kernels::perlinNoise3(p * lacunarity + Kernel3(1.7f, 9.2f, 3.1f));
    float expected = (n0 + gain * n1) / (1.0f + gain) * 0.5f + 0.5f;
    EXPECT(nearly(kernels::fractalNoise(p, 1, 0, 2.0f, lacunarity, gain), expected, 1e-5f));
}

TEST(fractalNoiseStaysInZeroToOneForEveryModeAndOctaveCount) {
    for (int mode = 0; mode < 4; mode += 1) {
        for (int type = 0; type < 3; type += 1) {
            for (float octaves : {0.0f, 1.0f, 3.5f, 8.0f, 20.0f}) {
                auto noise = [&](Kernel3 p) { return kernels::fractalNoise(p, type, mode, octaves, 2.0f, 0.9f); };
                EXPECT(staysWithin(noise, 0.0f, 1.0f));
            }
        }
    }
    // Continuity of the layered sum.
    auto smooth = [](Kernel3 p) { return kernels::fractalNoise(p, 1, 0, 4.0f, 2.0f, 0.5f); };
    EXPECT(largestStepChange(smooth, 0.0005f) < 0.05f);
    // A count of 20 is limited to 8 octaves: it equals 8.
    EXPECT(kernels::fractalNoise(samplePoint(2), 1, 0, 20.0f, 2.0f, 0.5f) == kernels::fractalNoise(samplePoint(2), 1, 0, 8.0f, 2.0f, 0.5f));
}

TEST(domainWarpedNoiseWithNoWarpIsPlainLayeredNoise) {
    Kernel2 p(0.37f, 1.91f);
    Kernel3 result = kernels::domainWarpedNoise(p, 0.5f, 0.0f, 1, 3.0f, 2.0f, 0.5f);
    EXPECT(nearly(result.x, kernels::fractalNoise(Kernel3(p.x, p.y, 0.5f), 1, 0, 3.0f, 2.0f, 0.5f), 1e-6f));
    // With no warp r is the plain noise read at the fixed offsets (1.7, 9.2) and (8.3, 2.8).
    EXPECT(nearly(result.y, kernels::fractalNoise(Kernel3(p.x + 1.7f, p.y + 9.2f, 0.5f), 1, 0, 3.0f, 2.0f, 0.5f), 1e-6f));
    EXPECT(nearly(result.z, kernels::fractalNoise(Kernel3(p.x + 8.3f, p.y + 2.8f, 0.5f), 1, 0, 3.0f, 2.0f, 0.5f), 1e-6f));
    // With warp, all three stay in 0…1 and the value changes.
    Kernel3 warped = kernels::domainWarpedNoise(p, 0.5f, 1.0f, 1, 3.0f, 2.0f, 0.5f);
    EXPECT(warped.x >= 0.0f && warped.x <= 1.0f && warped.y >= 0.0f && warped.y <= 1.0f && warped.z >= 0.0f && warped.z <= 1.0f);
    EXPECT(!nearly(warped.x, result.x, 1e-4f));
}

TEST(curlNoiseIsBoundedContinuousAndMatchesItsDifferences) {
    Kernel2 p(0.37f, 1.91f);
    Kernel2 curl = kernels::curlNoise(p, 0.5f);
    float epsilon = 0.01f;
    float expectedX = kernels::simplexNoise3(Kernel3(p.x, p.y + epsilon, 0.5f)) - kernels::simplexNoise3(Kernel3(p.x, p.y - epsilon, 0.5f));
    float expectedY = kernels::simplexNoise3(Kernel3(p.x - epsilon, p.y, 0.5f)) - kernels::simplexNoise3(Kernel3(p.x + epsilon, p.y, 0.5f));
    EXPECT(nearly(curl.x, expectedX / (4.0f * epsilon), 1e-4f));
    EXPECT(nearly(curl.y, expectedY / (4.0f * epsilon), 1e-4f));
    // Each difference of two noises in -1…1 is at most 2, so each component is at most 2 / (4 × 0.01) = 50.
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 q = samplePoint(index);
        Kernel2 c = kernels::curlNoise(Kernel2(q.x, q.y), q.z);
        EXPECT(std::fabs(c.x) <= 50.0f && std::fabs(c.y) <= 50.0f);
    }
}

TEST(flowNoiseIsZeroAtLatticePointsBoundedAndPeriodicInTime) {
    for (float time : {0.0f, 0.7f, 2.0f}) {
        EXPECT(nearly(kernels::flowNoise(Kernel2(3.0f, -2.0f), time), 0.0f, 1e-6f));
    }
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 p = samplePoint(index);
        float value = kernels::flowNoise(Kernel2(p.x, p.y), p.z);
        EXPECT(value >= -1.0f && value <= 1.0f);
        EXPECT(nearly(value, kernels::flowNoise(Kernel2(p.x, p.y), p.z + kernels::tau<float>()), 1e-4f));
        EXPECT(std::fabs(kernels::flowNoise(Kernel2(p.x + 0.001f, p.y), p.z) - value) < 0.01f);
    }
    // The pattern moves with time.
    EXPECT(!nearly(kernels::flowNoise(Kernel2(0.3f, 0.6f), 0.0f), kernels::flowNoise(Kernel2(0.3f, 0.6f), 1.0f), 1e-3f));
}

TEST(gaborNoiseIsBoundedContinuousAndDeterministic) {
    float largest = 0.0f;
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 p = samplePoint(index);
        float value = kernels::gaborNoise(Kernel2(p.x, p.y), 2.0f, 1.0f, 0.5f, 0.5f);
        EXPECT(value >= -1.0f && value <= 1.0f);
        EXPECT(value == kernels::gaborNoise(Kernel2(p.x, p.y), 2.0f, 1.0f, 0.5f, 0.5f));
        EXPECT(std::fabs(kernels::gaborNoise(Kernel2(p.x + 0.001f, p.y), 2.0f, 1.0f, 0.5f, 0.5f) - value) < 0.05f);
        largest = std::fmax(largest, std::fabs(value));
    }
    EXPECT(largest > 0.05f);
}

TEST(gaborNoiseWithAHugeBandwidthVanishes) {
    // e^(−π · 100² · |d|²) is 0 unless d is 0 exactly, so away from the kernel centres the sum is 0.
    EXPECT(nearly(kernels::gaborNoise(Kernel2(0.31f, 0.77f), 2.0f, 100.0f, 0.5f, 0.5f), 0.0f, 1e-6f));
}

TEST(bandLimitedNoiseIsBoundedAndContinuous) {
    auto noise = [](Kernel3 p) { return kernels::bandLimitedNoise(p); };
    EXPECT(staysWithin(noise, -1.0f, 1.0f));
    EXPECT(largestStepChange(noise, 0.001f) < 0.05f);
    // By its definition: clamp((perlin(p) − mean of five taps) × 1.6).
    Kernel3 p = samplePoint(6);
    float blur = (kernels::perlinNoise3(p) + kernels::perlinNoise3(p + Kernel3(0.6f, 0, 0)) + kernels::perlinNoise3(p - Kernel3(0.6f, 0, 0))
                  + kernels::perlinNoise3(p + Kernel3(0, 0.6f, 0)) + kernels::perlinNoise3(p - Kernel3(0, 0.6f, 0))) / 5.0f;
    EXPECT(nearly(kernels::bandLimitedNoise(p), std::fmax(-1.0f, std::fmin(1.0f, (kernels::perlinNoise3(p) - blur) * 1.6f)), 1e-5f));
}

TEST(cellularDistanceMeasuresInThreeMetrics) {
    // (3, −4): Euclidean 5, Manhattan 7, Chebyshev 4.
    EXPECT(nearly(kernels::cellularDistance(Kernel2(3.0f, -4.0f), 0), 5.0f));
    EXPECT(nearly(kernels::cellularDistance(Kernel2(3.0f, -4.0f), 1), 7.0f));
    EXPECT(nearly(kernels::cellularDistance(Kernel2(3.0f, -4.0f), 2), 4.0f));
    EXPECT(nearly(kernels::cellularDistance(Kernel2(3.0f, -4.0f), 5), 5.0f));
}

TEST(cellularFeaturePointBlendsTheCentreAndTheRandomSpot) {
    Kernel2 cell(4.0f, -3.0f);
    Kernel2 h = kernels::hash22(cell);
    // At evolution 0 the wandering term sin(0 + a) − sin(a) is 0, so jitter 1 gives the hash, jitter 0 the centre.
    Kernel2 random = kernels::cellularFeaturePoint(cell, 1.0f, 0.0f);
    EXPECT(nearly(random.x, h.x, 1e-6f) && nearly(random.y, h.y, 1e-6f));
    Kernel2 centre = kernels::cellularFeaturePoint(cell, 0.0f, 0.0f);
    EXPECT(centre.x == 0.5f && centre.y == 0.5f);
    Kernel2 half = kernels::cellularFeaturePoint(cell, 0.5f, 0.0f);
    EXPECT(nearly(half.x, 0.5f * (0.5f + h.x), 1e-6f) && nearly(half.y, 0.5f * (0.5f + h.y), 1e-6f));
    // Evolution moves the spot by at most 0.3 × 2 per axis.
    Kernel2 moved = kernels::cellularFeaturePoint(cell, 1.0f, 1.3f);
    EXPECT(std::fabs(moved.x - h.x) <= 0.6f + 1e-5f && std::fabs(moved.y - h.y) <= 0.6f + 1e-5f);
}

TEST(cellularNoiseOnARegularGridMeasuresToTheCellCentres) {
    // Jitter 0: every feature point is the cell centre. At (0.25, 0.5) the nearest is (0.5, 0.5) at 0.25,
    // the second (−0.5, 0.5) at 0.75; the id is hash21 of that cell + 7.
    kernels::CellularResult result = kernels::cellularNoise(Kernel2(0.25f, 0.5f), 0.0f, 0.0f, 0);
    EXPECT(nearly(result.nearest, 0.25f) && nearly(result.second, 0.75f));
    EXPECT(result.nearestPoint.x == 0.5f && result.nearestPoint.y == 0.5f);
    EXPECT(result.id == kernels::hash21(Kernel2(7.0f, 7.0f)));
    // On a centre the nearest distance is 0 and the second is the neighbouring centre, 1 away in every metric.
    for (int metric = 0; metric < 3; metric += 1) {
        kernels::CellularResult onCentre = kernels::cellularNoise(Kernel2(2.5f, -1.5f), 0.0f, 0.0f, metric);
        EXPECT(nearly(onCentre.nearest, 0.0f) && nearly(onCentre.second, 1.0f));
    }
    // At (0.5, 0.25), 0.25 below the centre: the nearest is that centre, the second (0.5, −0.5) at 0.75.
    kernels::CellularResult manhattan = kernels::cellularNoise(Kernel2(0.5f, 0.25f), 0.0f, 0.0f, 1);
    EXPECT(nearly(manhattan.nearest, 0.25f) && nearly(manhattan.second, 0.75f));
    // At (0.9, 0.9): the centre (0.5, 0.5) is 0.4 away in Chebyshev, the centre (1.5, 0.5) is max(0.6, 0.4) = 0.6.
    kernels::CellularResult chebyshev = kernels::cellularNoise(Kernel2(0.9f, 0.9f), 0.0f, 0.0f, 2);
    EXPECT(nearly(chebyshev.nearest, 0.4f) && nearly(chebyshev.second, 0.6f));
}

TEST(cellularNoiseKeepsItsInvariantsWithJitter) {
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 p = samplePoint(index);
        for (int metric = 0; metric < 3; metric += 1) {
            kernels::CellularResult result = kernels::cellularNoise(Kernel2(p.x, p.y), 1.0f, 0.3f, metric);
            EXPECT(result.nearest >= 0.0f && result.nearest <= result.second);
            EXPECT(result.id >= 0.0f && result.id <= 1.0f);
            // The nearest point is exactly `nearest` away in the chosen metric.
            EXPECT(nearly(kernels::cellularDistance(Kernel2(result.nearestPoint.x - p.x, result.nearestPoint.y - p.y), metric), result.nearest, 1e-6f));
        }
        // The nearest distance changes by no more than the step (it is a distance to a fixed set).
        float here = kernels::cellularNoise(Kernel2(p.x, p.y), 1.0f, 0.3f, 0).nearest;
        float moved = kernels::cellularNoise(Kernel2(p.x + 0.001f, p.y), 1.0f, 0.3f, 0).nearest;
        EXPECT(std::fabs(moved - here) <= 0.001f + 1e-6f);
    }
}

TEST(cellularBorderDistanceIsHalfTheGapOnARegularGrid) {
    // Centres 1 apart: the border between (0.5, 0.5) and (1.5, 0.5) is x = 1. At x = 0.75 it is 0.25 away,
    // at x = 1 it is 0, and the border is the same distance from both sides.
    EXPECT(nearly(kernels::cellularBorderDistance(Kernel2(0.75f, 0.5f), 0.0f, 0.0f), 0.25f, 1e-5f));
    EXPECT(nearly(kernels::cellularBorderDistance(Kernel2(0.5f, 0.5f), 0.0f, 0.0f), 0.5f, 1e-5f));
    EXPECT(nearly(kernels::cellularBorderDistance(Kernel2(1.25f, 0.5f), 0.0f, 0.0f), 0.25f, 1e-5f));
    EXPECT(kernels::cellularBorderDistance(Kernel2(1.0f, 0.5f), 0.0f, 0.0f) < 1e-5f);
}

TEST(cellularBorderDistanceIsBoundedByTheTwoNearestDistances) {
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel3 p = samplePoint(index);
        float border = kernels::cellularBorderDistance(Kernel2(p.x, p.y), 1.0f, 0.0f);
        kernels::CellularResult cells = kernels::cellularNoise(Kernel2(p.x, p.y), 1.0f, 0.0f, 0);
        // The bisector with the second site is no farther than the mean of the two distances.
        EXPECT(border >= -1e-5f);
        EXPECT(border <= 0.5f * (cells.nearest + cells.second) + 1e-5f);
    }
}

TEST(whiteNoiseIsAHashPerPixelOrPerCell) {
    Kernel2 position(12.0f, 7.0f);
    Kernel2 coordinates(0.30f, 0.62f);
    // Mode 0 hashes the position and the seed.
    EXPECT(kernels::whiteNoise(position, coordinates, 8.0f, 3.0f, 0) == kernels::hash31(Kernel3(12.0f, 7.0f, 3.0f)));
    // Mode 1 hashes the cell: ⌊0.30 × 8⌋ = 2, ⌊0.62 × 8⌋ = 4; another point of that cell gives the same value.
    EXPECT(kernels::whiteNoise(position, coordinates, 8.0f, 3.0f, 1) == kernels::hash31(Kernel3(2.0f, 4.0f, 3.0f)));
    EXPECT(kernels::whiteNoise(position, Kernel2(0.33f, 0.60f), 8.0f, 3.0f, 1) == kernels::whiteNoise(position, coordinates, 8.0f, 3.0f, 1));
    // A different seed gives a different number, and the colour form is hash33.
    EXPECT(kernels::whiteNoise(position, coordinates, 8.0f, 4.0f, 0) != kernels::whiteNoise(position, coordinates, 8.0f, 3.0f, 0));
    Kernel3 colour = kernels::whiteNoiseVector3(position, coordinates, 8.0f, 3.0f, 1);
    Kernel3 expected = kernels::hash33(Kernel3(2.0f, 4.0f, 3.0f));
    EXPECT(colour.x == expected.x && colour.y == expected.y && colour.z == expected.z);
    EXPECT(colour.x >= 0.0f && colour.x <= 1.0f && colour.y >= 0.0f && colour.y <= 1.0f && colour.z >= 0.0f && colour.z <= 1.0f);
}
