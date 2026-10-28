// The signed-distance kernels (Kernels/ShaderDistances.h) on the CPU, against values worked out by
// hand from each function's formula, and the properties each must keep: the sign, and for the
// exact distances the Lipschitz bound |d(p) − d(q)| ≤ |p − q|. The same header is compiled as
// Metal by `make kernels`.
#include <cmath>

#include "../Kernels/ShaderDistances.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;
using Kernel3 = kernels::Vector3Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }
bool nearly(Kernel3 first, Kernel3 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance);
}

// Checks |d(p) − d(q)| ≤ |p − q| (plus `slack` for rounding) for a grid of points p, each with
// a neighbour q a fixed step away, in 3D.
template <typename Distance>
bool isLipschitzOne(Distance distance, float slack = 1e-4f) {
    const Kernel3 step(0.13f, -0.07f, 0.11f);
    const float stepLength = std::sqrt(0.13f * 0.13f + 0.07f * 0.07f + 0.11f * 0.11f);
    for (int i = -8; i <= 8; i += 1) {
        for (int j = -8; j <= 8; j += 1) {
            for (int k = -8; k <= 8; k += 1) {
                Kernel3 point(0.37f * static_cast<float>(i) + 0.011f, 0.31f * static_cast<float>(j) - 0.017f,
                    0.29f * static_cast<float>(k) + 0.023f);
                float difference = std::fabs(distance(point) - distance(point + step));
                if (difference > stepLength + slack) return false;
            }
        }
    }
    return true;
}
} // namespace

TEST(signOfIsMinusOneZeroOrOne) {
    EXPECT(kernels::signOf(3.5f) == 1.0f);
    EXPECT(kernels::signOf(-0.001f) == -1.0f);
    EXPECT(kernels::signOf(0.0f) == 0.0f);
    EXPECT(kernels::signOf(-0.0f) == 0.0f);
    EXPECT(kernels::signOf(std::nanf("")) == 0.0f);
    Kernel2 signs = kernels::signOf(Kernel2(-2.0f, 0.0f));
    EXPECT(signs.x == -1.0f && signs.y == 0.0f);
}

TEST(distanceFillMaskIsOneInsideZeroOutsideAndHalfOnTheEdge) {
    // pixel 0.1, feather 1: edge 0.1. d = 0 → 0.5; d = −1 → 1; d = 1 → 0
    EXPECT(nearly(kernels::distanceFillMask(0.0f, 0.1f, 1.0f), 0.5f));
    EXPECT(nearly(kernels::distanceFillMask(-1.0f, 0.1f, 1.0f), 1.0f));
    EXPECT(nearly(kernels::distanceFillMask(1.0f, 0.1f, 1.0f), 0.0f));
    // d = 0.05: t = (0.05 + 0.1) / 0.2 = 0.75; t²(3 − 2t) = 0.5625 × 1.5 = 0.84375; mask 0.15625
    EXPECT(nearly(kernels::distanceFillMask(0.05f, 0.1f, 1.0f), 0.15625f));
    // A feather below 1 acts as 1; a feather of 3 triples the edge: d = 0.15, edge 0.3 → t = 0.75 again.
    EXPECT(nearly(kernels::distanceFillMask(0.05f, 0.1f, 0.2f), 0.15625f));
    EXPECT(nearly(kernels::distanceFillMask(0.15f, 0.1f, 3.0f), 0.15625f));
    // Property: falls monotonically as the distance grows, within 0 … 1.
    float previous = 2.0f;
    for (int step = -30; step <= 30; step += 1) {
        float mask = kernels::distanceFillMask(0.01f * static_cast<float>(step), 0.1f, 1.0f);
        EXPECT(mask <= previous && mask >= 0.0f && mask <= 1.0f);
        previous = mask;
    }
    // A zero edge gives a hard step instead of NaN.
    EXPECT(kernels::distanceFillMask(-0.5f, 0.0f, 1.0f) == 1.0f);
    EXPECT(kernels::distanceFillMask(0.5f, 0.0f, 1.0f) == 0.0f);
}

TEST(distanceStrokeMaskDrawsABandWhereTheAlignmentSaysAndHalfOnItsEdges) {
    // width 0.2 centred: band −0.1 … 0.1. Middle → 1; edges → 0.5; beyond → 0.
    EXPECT(nearly(kernels::distanceStrokeMask(0.0f, 0.2f, 0, 0.01f, 1.0f), 1.0f));
    EXPECT(nearly(kernels::distanceStrokeMask(0.1f, 0.2f, 0, 0.01f, 1.0f), 0.5f));
    EXPECT(nearly(kernels::distanceStrokeMask(-0.1f, 0.2f, 0, 0.01f, 1.0f), 0.5f));
    EXPECT(nearly(kernels::distanceStrokeMask(0.3f, 0.2f, 0, 0.01f, 1.0f), 0.0f));
    // Inside: band −0.2 … 0
    EXPECT(nearly(kernels::distanceStrokeMask(-0.1f, 0.2f, 1, 0.01f, 1.0f), 1.0f));
    EXPECT(nearly(kernels::distanceStrokeMask(-0.2f, 0.2f, 1, 0.01f, 1.0f), 0.5f));
    EXPECT(nearly(kernels::distanceStrokeMask(0.0f, 0.2f, 1, 0.01f, 1.0f), 0.5f));
    EXPECT(nearly(kernels::distanceStrokeMask(0.1f, 0.2f, 1, 0.01f, 1.0f), 0.0f));
    // Outside: band 0 … 0.2
    EXPECT(nearly(kernels::distanceStrokeMask(0.1f, 0.2f, 2, 0.01f, 1.0f), 1.0f));
    EXPECT(nearly(kernels::distanceStrokeMask(0.0f, 0.2f, 2, 0.01f, 1.0f), 0.5f));
    EXPECT(nearly(kernels::distanceStrokeMask(0.2f, 0.2f, 2, 0.01f, 1.0f), 0.5f));
    EXPECT(nearly(kernels::distanceStrokeMask(-0.1f, 0.2f, 2, 0.01f, 1.0f), 0.0f));
    // A negative width is no band at all.
    EXPECT(nearly(kernels::distanceStrokeMask(0.0f, -1.0f, 0, 0.01f, 1.0f), 0.5f));
}

TEST(distanceGlowIsFullInsideAndFallsByEPerRadius) {
    EXPECT(nearly(kernels::distanceGlow(-3.0f, 0.5f, 2.0f), 2.0f));
    EXPECT(nearly(kernels::distanceGlow(0.0f, 0.5f, 2.0f), 2.0f));
    // One radius outside: intensity / e = 2 / 2.7182817 = 0.7357589
    EXPECT(nearly(kernels::distanceGlow(0.5f, 0.5f, 2.0f), 0.7357589f));
    // The radius is at least 0.0001: 0.0001 outside with radius 0 is 1/e.
    EXPECT(nearly(kernels::distanceGlow(0.0001f, 0.0f, 1.0f), 0.36787945f, 1e-4f));
}

TEST(distanceShadowMovesTheEdgeAlongTheGradient) {
    // gradient (0, 2) → direction (0, 1); offset (0, 0.5): moved = 0.5 − 0.5 = 0 → 0.8 × 0.5
    EXPECT(nearly(kernels::distanceShadow(0.5f, Kernel2(0.0f, 2.0f), Kernel2(0.0f, 0.5f), 0.1f, 0.8f, 0.01f), 0.4f));
    // Deep inside the moved shape: full opacity; far outside: none.
    EXPECT(nearly(kernels::distanceShadow(-1.0f, Kernel2(0.0f, 1.0f), Kernel2(0.0f, 0.0f), 0.1f, 0.8f, 0.01f), 0.8f));
    EXPECT(nearly(kernels::distanceShadow(1.0f, Kernel2(0.0f, 1.0f), Kernel2(0.0f, 0.0f), 0.1f, 0.8f, 0.01f), 0.0f));
    // A zero gradient does not move the shadow: moved = d = 0 → half the opacity.
    EXPECT(nearly(kernels::distanceShadow(0.0f, Kernel2(0.0f, 0.0f), Kernel2(1.0f, 1.0f), 0.1f, 1.0f, 0.01f), 0.5f));
    // The blur is at least one pixel: blur 0, pixel 0.1, d = 0.05 → t = 0.75 → 1 − 0.84375
    EXPECT(nearly(kernels::distanceShadow(0.05f, Kernel2(1.0f, 0.0f), Kernel2(0.0f, 0.0f), 0.0f, 1.0f, 0.1f), 0.15625f));
}

TEST(smoothOperationsHaveTheirWorkedValuesAndBounds) {
    // a = b = 1, k = 1: h = 0.5; mix = 1; 1 − 0.25 = 0.75
    EXPECT(nearly(kernels::smoothUnion(1.0f, 1.0f, 1.0f), 0.75f));
    // far apart: the smaller one: a = 0, b = 5 → h = 1 → 0
    EXPECT(nearly(kernels::smoothUnion(0.0f, 5.0f, 1.0f), 0.0f));
    // subtraction: a = b = 0, k = 1: h = 0.5; mix(0, −0, 0.5) = 0; + 0.25
    EXPECT(nearly(kernels::smoothSubtraction(0.0f, 0.0f, 1.0f), 0.25f));
    // a = b = 1, k = 1: h = clamp(0.5 − 1) = 0 → a = 1
    EXPECT(nearly(kernels::smoothSubtraction(1.0f, 1.0f, 1.0f), 1.0f));
    // intersection: a = b = 0, k = 1 → 0.25; far apart → the larger one
    EXPECT(nearly(kernels::smoothIntersection(0.0f, 0.0f, 1.0f), 0.25f));
    EXPECT(nearly(kernels::smoothIntersection(0.0f, 5.0f, 1.0f), 5.0f));
    // Properties over a grid of (a, b, k): union ≤ min, intersection ≥ max, subtraction ≥ max(a, −b),
    // and as k → 0 they become min, max and max(a, −b).
    for (int i = -10; i <= 10; i += 1) {
        for (int j = -10; j <= 10; j += 1) {
            float a = 0.23f * static_cast<float>(i);
            float b = 0.19f * static_cast<float>(j) + 0.01f;
            for (float k : {0.1f, 0.5f, 2.0f}) {
                EXPECT(kernels::smoothUnion(a, b, k) <= std::fmin(a, b) + 1e-6f);
                EXPECT(kernels::smoothIntersection(a, b, k) >= std::fmax(a, b) - 1e-6f);
                EXPECT(kernels::smoothSubtraction(a, b, k) >= std::fmax(a, -b) - 1e-6f);
            }
            EXPECT(nearly(kernels::smoothUnion(a, b, 0.0f), std::fmin(a, b), 1e-4f));
            EXPECT(nearly(kernels::smoothIntersection(a, b, 0.0f), std::fmax(a, b), 1e-4f));
            EXPECT(nearly(kernels::smoothSubtraction(a, b, 0.0f), std::fmax(a, -b), 1e-4f));
        }
    }
}

TEST(repeatedCell2FoldsIntoTheNearestCell) {
    // (2.3, −0.4), spacing 1: cell (⌊2.8⌋, ⌊0.1⌋) = (2, 0) → (0.3, −0.4)
    Kernel4 result = kernels::repeatedCell2(Kernel2(2.3f, -0.4f), 1.0f, 0.0f);
    EXPECT(nearly(result.x, 0.3f) && nearly(result.y, -0.4f) && result.z == 2.0f && result.w == 0.0f);
    // A limit of 1 clamps the cell to ±1: cell (1, 0) → (1.3, −0.4)
    result = kernels::repeatedCell2(Kernel2(2.3f, -0.4f), 1.0f, 1.0f);
    EXPECT(nearly(result.x, 1.3f) && nearly(result.y, -0.4f) && result.z == 1.0f && result.w == 0.0f);
    // A negative spacing acts as its size; a zero spacing does not divide by zero.
    EXPECT(nearly(kernels::repeatedCell2(Kernel2(2.3f, -0.4f), -1.0f, 0.0f).x, 0.3f));
    EXPECT(std::isfinite(kernels::repeatedCell2(Kernel2(2.3f, -0.4f), 0.0f, 0.0f).x));
    // Property: with no limit the folded point lies within half a spacing of the origin.
    for (int step = -50; step <= 50; step += 1) {
        Kernel4 folded = kernels::repeatedCell2(Kernel2(0.173f * static_cast<float>(step), -0.291f * static_cast<float>(step)), 0.7f, 0.0f);
        EXPECT(std::fabs(folded.x) <= 0.35f + 1e-5f && std::fabs(folded.y) <= 0.35f + 1e-5f);
    }
}

TEST(sphereDistanceIsTheLengthLessTheRadius) {
    EXPECT(nearly(kernels::sphereDistance(Kernel3(2.0f, 0.0f, 0.0f), 1.0f), 1.0f));
    EXPECT(nearly(kernels::sphereDistance(Kernel3(0.0f, 0.0f, 0.0f), 1.5f), -1.5f));
    EXPECT(nearly(kernels::sphereDistance(Kernel3(1.0f, 0.0f, 0.0f), 1.0f), 0.0f));
    // 3-4-5: (3, 4, 0) is 5 from the centre, 4 from a sphere of radius 1
    EXPECT(nearly(kernels::sphereDistance(Kernel3(3.0f, 4.0f, 0.0f), 1.0f), 4.0f));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::sphereDistance(point, 1.3f); }));
}

TEST(boxDistanceMatchesHandWorkedPointsAndRounding) {
    Kernel3 halfSize(1.0f, 2.0f, 3.0f);
    // Centre: q = (−1, −2, −3); the largest is −1 → minus the smallest half size
    EXPECT(nearly(kernels::boxDistance(Kernel3(0.0f, 0.0f, 0.0f), halfSize, 0.0f), -1.0f));
    // Rounding keeps the centre value: q = (−0.5, −1.5, −2.5) → −0.5 − 0.5
    EXPECT(nearly(kernels::boxDistance(Kernel3(0.0f, 0.0f, 0.0f), halfSize, 0.5f), -1.0f));
    // Face: (3, 0, 0): q = (2, −2, −3) → 2
    EXPECT(nearly(kernels::boxDistance(Kernel3(3.0f, 0.0f, 0.0f), halfSize, 0.0f), 2.0f));
    // Corner: (2, 3, 4): q = (1, 1, 1) → √3
    EXPECT(nearly(kernels::boxDistance(Kernel3(2.0f, 3.0f, 4.0f), halfSize, 0.0f), 1.7320508f));
    // Rounded corner, half size 1, rounding 0.5, point (1.5, 1.5, 1.5): q = (1, 1, 1) → √3 − 0.5
    EXPECT(nearly(kernels::boxDistance(Kernel3(1.5f, 1.5f, 1.5f), Kernel3(1.0f, 1.0f, 1.0f), 0.5f), 1.2320508f));
    // Rounding is limited to the smallest half size: a unit cube rounded by 5 is a sphere of radius 1.
    EXPECT(nearly(kernels::boxDistance(Kernel3(2.0f, 0.0f, 0.0f), Kernel3(1.0f, 1.0f, 1.0f), 5.0f), 1.0f));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::boxDistance(point, Kernel3(1.0f, 0.7f, 1.4f), 0.0f); }));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::boxDistance(point, Kernel3(1.0f, 0.7f, 1.4f), 0.4f); }));
}

TEST(torusDistanceMeasuresFromTheCentreLineOfTheTube) {
    // major 2, minor 0.5: (3, 0, 0): |(3 − 2, 0)| − 0.5 = 0.5
    EXPECT(nearly(kernels::torusDistance(Kernel3(3.0f, 0.0f, 0.0f), 2.0f, 0.5f), 0.5f));
    // On the tube's surface: (2, 0.5, 0) → 0
    EXPECT(nearly(kernels::torusDistance(Kernel3(2.0f, 0.5f, 0.0f), 2.0f, 0.5f), 0.0f));
    // The tube's centre line is inside by the minor radius
    EXPECT(nearly(kernels::torusDistance(Kernel3(0.0f, 0.0f, 2.0f), 2.0f, 0.5f), -0.5f));
    // The middle of the hole: |(−2, 0)| − 0.5 = 1.5
    EXPECT(nearly(kernels::torusDistance(Kernel3(0.0f, 0.0f, 0.0f), 2.0f, 0.5f), 1.5f));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::torusDistance(point, 1.5f, 0.4f); }));
}

TEST(cylinderDistanceHasRoundedEdgeRegionsAndFlatCaps) {
    // radius 1, height 4: d = (|0| − 1, |0| − 2) → min(max(−1, −2), 0) = −1
    EXPECT(nearly(kernels::cylinderDistance(Kernel3(0.0f, 0.0f, 0.0f), 1.0f, 4.0f), -1.0f));
    // Side: (3, 0, 0): d = (2, −2) → 2
    EXPECT(nearly(kernels::cylinderDistance(Kernel3(3.0f, 0.0f, 0.0f), 1.0f, 4.0f), 2.0f));
    // Cap: (0, 3, 0): d = (−1, 1) → 1
    EXPECT(nearly(kernels::cylinderDistance(Kernel3(0.0f, 3.0f, 0.0f), 1.0f, 4.0f), 1.0f));
    // Rim: (2, 3, 0): d = (1, 1) → √2
    EXPECT(nearly(kernels::cylinderDistance(Kernel3(2.0f, 3.0f, 0.0f), 1.0f, 4.0f), 1.4142135f));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::cylinderDistance(point, 1.1f, 2.2f); }));
}

TEST(coneDistanceHasTheTipAtTheTopAndTheBaseAtTheBottom) {
    // radius 1, height 2: inside by the distance to the slanted side, 1/√5, at the origin
    EXPECT(nearly(kernels::coneDistance(Kernel3(0.0f, 0.0f, 0.0f), 1.0f, 2.0f), -0.4472136f));
    // One below the base (y = −1) is 1 away: (0, −2, 0)
    EXPECT(nearly(kernels::coneDistance(Kernel3(0.0f, -2.0f, 0.0f), 1.0f, 2.0f), 1.0f));
    // One above the tip (y = 1): (0, 2, 0)
    EXPECT(nearly(kernels::coneDistance(Kernel3(0.0f, 2.0f, 0.0f), 1.0f, 2.0f), 1.0f));
    // On the base rim (1, −1, 0) and the tip (0, 1, 0)
    EXPECT(nearly(kernels::coneDistance(Kernel3(1.0f, -1.0f, 0.0f), 1.0f, 2.0f), 0.0f));
    EXPECT(nearly(kernels::coneDistance(Kernel3(0.0f, 1.0f, 0.0f), 1.0f, 2.0f), 0.0f));
    // Beside the base, level with it: (2, −1, 0) → 1
    EXPECT(nearly(kernels::coneDistance(Kernel3(2.0f, -1.0f, 0.0f), 1.0f, 2.0f), 1.0f));
    // The cone is round: the same at any angle around y.
    EXPECT(nearly(kernels::coneDistance(Kernel3(0.6f, 0.3f, 0.8f), 1.0f, 2.0f), kernels::coneDistance(Kernel3(1.0f, 0.3f, 0.0f), 1.0f, 2.0f)));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::coneDistance(point, 1.2f, 2.5f); }));
}

TEST(planeDistanceIsTheSignedHeightAboveThePlane) {
    // y = 2 facing up: (0, 5, 0) → 5 − 2 = 3, below the plane negative
    EXPECT(nearly(kernels::planeDistance(Kernel3(0.0f, 5.0f, 0.0f), Kernel3(0.0f, 1.0f, 0.0f), -2.0f), 3.0f, 1e-4f));
    EXPECT(nearly(kernels::planeDistance(Kernel3(7.0f, 0.0f, -3.0f), Kernel3(0.0f, 1.0f, 0.0f), -2.0f), -2.0f, 1e-4f));
    // The normal is normalised: (0, 0, 10) gives the same as (0, 0, 1)
    EXPECT(nearly(kernels::planeDistance(Kernel3(1.0f, 2.0f, 3.0f), Kernel3(0.0f, 0.0f, 10.0f), 0.0f), 3.0f, 1e-4f));
    // A zero normal becomes up: no NaN
    EXPECT(nearly(kernels::planeDistance(Kernel3(1.0f, 2.0f, 3.0f), Kernel3(0.0f, 0.0f, 0.0f), 0.0f), 2.0f, 1e-4f));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::planeDistance(point, Kernel3(1.0f, 2.0f, -0.5f), 0.3f); }));
}

TEST(capsuleDistanceSlidesAlongTheCoreSegment) {
    // radius 0.5, height 2: (2, 0, 0) → 2 − 0.5; (0, 3, 0) → |2| − 0.5; centre → −0.5
    EXPECT(nearly(kernels::capsuleDistance(Kernel3(2.0f, 0.0f, 0.0f), 0.5f, 2.0f), 1.5f));
    EXPECT(nearly(kernels::capsuleDistance(Kernel3(0.0f, 3.0f, 0.0f), 0.5f, 2.0f), 1.5f));
    EXPECT(nearly(kernels::capsuleDistance(Kernel3(0.0f, 0.0f, 0.0f), 0.5f, 2.0f), -0.5f));
    // Along the side the distance does not depend on y inside the core: (2, 0.7, 0) = (2, 0, 0)
    EXPECT(nearly(kernels::capsuleDistance(Kernel3(2.0f, 0.7f, 0.0f), 0.5f, 2.0f), 1.5f));
    EXPECT(isLipschitzOne([](Kernel3 point) { return kernels::capsuleDistance(point, 0.4f, 1.8f); }));
}

TEST(translatedRotatedAndTwistedPointsMoveTheDomain) {
    EXPECT(nearly(kernels::translatedPoint(Kernel3(1.0f, 2.0f, 3.0f), Kernel3(0.5f, -1.0f, 2.0f)), Kernel3(0.5f, 3.0f, 1.0f)));
    // A sphere moved to (2, 0, 0): its centre reads as the origin, distance −radius
    EXPECT(nearly(kernels::sphereDistance(kernels::translatedPoint(Kernel3(2.0f, 0.0f, 0.0f), Kernel3(2.0f, 0.0f, 0.0f)), 1.0f), -1.0f));

    // Rotation about z by 90°: the point turns by −90°: (1, 0, 0) → (0, −1, 0), (0, 1, 0) → (1, 0, 0)
    EXPECT(nearly(kernels::rotatedPoint(Kernel3(1.0f, 0.0f, 0.0f), Kernel3(0.0f, 0.0f, 90.0f)), Kernel3(0.0f, -1.0f, 0.0f)));
    EXPECT(nearly(kernels::rotatedPoint(Kernel3(0.0f, 1.0f, 0.0f), Kernel3(0.0f, 0.0f, 90.0f)), Kernel3(1.0f, 0.0f, 0.0f)));
    // About x by 90° on (0, 1, 0): (x, c y − s z, s y + c z) with s = −1, c = 0 → (0, 0, −1)
    EXPECT(nearly(kernels::rotatedPoint(Kernel3(0.0f, 1.0f, 0.0f), Kernel3(90.0f, 0.0f, 0.0f)), Kernel3(0.0f, 0.0f, -1.0f)));
    // About y by 90° on (0, 0, 1): (c x + s z, y, −s x + c z) with s = −1 → (−1, 0, 0)
    EXPECT(nearly(kernels::rotatedPoint(Kernel3(0.0f, 0.0f, 1.0f), Kernel3(0.0f, 90.0f, 0.0f)), Kernel3(-1.0f, 0.0f, 0.0f)));
    // Order: z first, then x: (1, 0, 0) → (0, −1, 0) → (0, 0, 1)
    EXPECT(nearly(kernels::rotatedPoint(Kernel3(1.0f, 0.0f, 0.0f), Kernel3(90.0f, 0.0f, 90.0f)), Kernel3(0.0f, 0.0f, 1.0f)));
    // Property: a rotation keeps the length, and turning by nothing changes nothing.
    Kernel3 point(0.3f, -1.2f, 0.7f);
    Kernel3 turned = kernels::rotatedPoint(point, Kernel3(33.0f, -71.0f, 125.0f));
    EXPECT(nearly(std::sqrt(turned.x * turned.x + turned.y * turned.y + turned.z * turned.z),
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z), 1e-5f));
    EXPECT(nearly(kernels::rotatedPoint(point, Kernel3(0.0f, 0.0f, 0.0f)), point));

    // Twist: 90° per unit at y = 1 turns about y by 90°: (c x − s z, y, s x + c z) = (0, 1, 1)
    EXPECT(nearly(kernels::twistedPoint(Kernel3(1.0f, 1.0f, 0.0f), 90.0f), Kernel3(0.0f, 1.0f, 1.0f)));
    EXPECT(nearly(kernels::twistedPoint(Kernel3(1.0f, 0.0f, 0.5f), 90.0f), Kernel3(1.0f, 0.0f, 0.5f)));
    EXPECT(nearly(kernels::twistedPoint(Kernel3(1.0f, 2.0f, 0.0f), 0.0f), Kernel3(1.0f, 2.0f, 0.0f)));
}

TEST(repeatedPointAndCellFoldIntoTheGrid) {
    // spacing 2, (3.5, 0, −5.2): cell (⌊2.25⌋, ⌊0.5⌋, ⌊−2.1⌋) = (2, 0, −3) → (3.5 − 4, 0, −5.2 + 6)
    EXPECT(nearly(kernels::repeatedPoint(Kernel3(3.5f, 0.0f, -5.2f), 2.0f, 0.0f), Kernel3(-0.5f, 0.0f, 0.8f)));
    EXPECT(nearly(kernels::repeatedCell3(Kernel3(3.5f, 0.0f, -5.2f), 2.0f, 0.0f), Kernel3(2.0f, 0.0f, -3.0f)));
    // Limit 1: cell (1, 0, −1) → (1.5, 0, −3.2)
    EXPECT(nearly(kernels::repeatedPoint(Kernel3(3.5f, 0.0f, -5.2f), 2.0f, 1.0f), Kernel3(1.5f, 0.0f, -3.2f)));
    EXPECT(nearly(kernels::repeatedCell3(Kernel3(3.5f, 0.0f, -5.2f), 2.0f, 1.0f), Kernel3(1.0f, 0.0f, -1.0f)));
    // Property: point = folded + spacing × cell, and the folded point is within half a spacing.
    for (int step = -40; step <= 40; step += 1) {
        Kernel3 point(0.31f * static_cast<float>(step), 0.17f * static_cast<float>(step), -0.23f * static_cast<float>(step));
        Kernel3 folded = kernels::repeatedPoint(point, 1.3f, 0.0f);
        Kernel3 cell = kernels::repeatedCell3(point, 1.3f, 0.0f);
        EXPECT(nearly(folded + cell * 1.3f, point, 1e-4f));
        EXPECT(std::fabs(folded.x) <= 0.65f + 1e-4f && std::fabs(folded.y) <= 0.65f + 1e-4f && std::fabs(folded.z) <= 0.65f + 1e-4f);
    }
}

TEST(repeatedShapesStayExactWhenTheyFitTheirCell) {
    // A sphere of radius 0.3 in cells 1 apart: the distance to the sphere in the nearest cell.
    // At (2.1, 0, 0) the nearest sphere is at (2, 0, 0): 0.1 − 0.3 = −0.2
    float distanceValue = kernels::sphereDistance(kernels::repeatedPoint(Kernel3(2.1f, 0.0f, 0.0f), 1.0f, 0.0f), 0.3f);
    EXPECT(nearly(distanceValue, -0.2f));
}
