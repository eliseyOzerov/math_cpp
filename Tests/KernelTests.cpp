// The shared kernels run on the CPU: the prelude's scalar helpers, the kernel vectors, and ports of
// SchlickFresnelTests in EditorTests/Math/Optics/MicrofacetTests.swift. The same headers
// are compiled as Metal by `make kernels`; the CPU results here are the reference a GPU run is
// compared against.
#include <cmath>

#include "../Kernels/KernelPrelude.h"
#include "../Kernels/KernelVectors.h"
#include "../Kernels/SchlickFresnel.h"
#include "../Linear/Angle.h"
#include "Testing.h"

using namespace editor;
using editor::kernels::SchlickFresnel;
using editor::testing::isApproximatelyEqual;

namespace {
constexpr float glassNormalIncidence = 0.04f;
} // namespace

TEST(kernelSquareRootOfAPerfectSquare) {
    // 1.5² = 2.25 exactly in binary, so the root is exact.
    EXPECT(kernels::squareRoot(2.25f) == 1.5f);
}

TEST(kernelScalarHelpersMatchHandWorkedValues) {
    EXPECT(kernels::absolute(-2.5f) == 2.5f);
    EXPECT(kernels::smallerOf(3.0f, -1.0f) == -1.0f);
    EXPECT(kernels::largerOf(3.0f, -1.0f) == 3.0f);
    EXPECT(kernels::roundedDown(-2.25f) == -3.0f);
    // −2.25 − (−3) = 0.75, exact in binary.
    EXPECT(kernels::fractionalPart(-2.25f) == 0.75f);
    EXPECT(kernels::chosen(true, 1.0f, 2.0f) == 1.0f);
    EXPECT(kernels::bitsOf(1.0f) == 0x3F800000u);
    EXPECT(isApproximatelyEqual(kernels::power(2.0f, 10.0f), 1024.0f));
    EXPECT(isApproximatelyEqual(kernels::naturalLogarithm(kernels::exponential(1.5f)), 1.5f));
}

TEST(kernelSmallerAndLargerFollowSwiftTieRules) {
    // On a tie smallerOf keeps the first and largerOf the second; −0 and +0 compare equal, so
    // which one comes back shows the rule.
    EXPECT(!std::signbit(kernels::smallerOf(0.0f, -0.0f)));
    EXPECT(std::signbit(kernels::largerOf(0.0f, -0.0f)));
}

TEST(kernelVectorArithmeticWorksPerComponent) {
    using Vector3 = kernels::Vector3Of<float>;
    Vector3 sum = Vector3(1, 2, 3) + Vector3(0.5f);
    EXPECT(sum.x == 1.5f && sum.y == 2.5f && sum.z == 3.5f);
    Vector3 scaled = 2.0f * Vector3(1, -2, 3) / 4.0f;
    EXPECT(scaled.x == 0.5f && scaled.y == -1.0f && scaled.z == 1.5f);
    Vector3 accumulated(1);
    accumulated += Vector3(1, 2, 3);
    accumulated *= 2.0f;
    EXPECT(accumulated.x == 4 && accumulated.y == 6 && accumulated.z == 8);
}

TEST(kernelVectorProductsAndLengths) {
    using Vector3 = kernels::Vector3Of<float>;
    // (1, 2, 3) · (4, −5, 6) = 4 − 10 + 18 = 12
    EXPECT(kernels::dotProduct(Vector3(1, 2, 3), Vector3(4, -5, 6)) == 12);
    // Right-handed: x × y = z.
    Vector3 z = kernels::crossProduct(Vector3(1, 0, 0), Vector3(0, 1, 0));
    EXPECT(z.x == 0 && z.y == 0 && z.z == 1);
    // |(3, 4)| = 5; |(2, 3, 6)| = √49 = 7
    EXPECT(kernels::lengthOf(kernels::Vector2Of<float>(3, 4)) == 5);
    EXPECT(kernels::lengthOf(Vector3(2, 3, 6)) == 7);
    Vector3 unit = kernels::normalized(Vector3(0, 0, -4));
    EXPECT(unit.z == -1);
}

TEST(kernelVectorFunctionsApplyTheScalarOnesPerComponent) {
    using Vector4 = kernels::Vector4Of<float>;
    Vector4 value(-1.25f, 2.5f, 0.0f, 7.0f);
    Vector4 absolute = kernels::absolute(value);
    EXPECT(absolute.x == 1.25f && absolute.y == 2.5f && absolute.w == 7);
    Vector4 fraction = kernels::fractionalPart(value);
    EXPECT(fraction.x == 0.75f && fraction.y == 0.5f && fraction.z == 0 && fraction.w == 0);
    Vector4 inside = kernels::clamped(value, 0.0f, 1.0f);
    EXPECT(inside.x == 0 && inside.y == 1 && inside.z == 0 && inside.w == 1);
    Vector4 halfway = kernels::linearBlend(Vector4(0), Vector4(2, 4, 6, 8), 0.5f);
    EXPECT(halfway.x == 1 && halfway.y == 2 && halfway.z == 3 && halfway.w == 4);
    Vector4 picked = kernels::chosen(value < Vector4(0), Vector4(-9), value);
    EXPECT(picked.x == -9 && picked.y == 2.5f && picked.z == 0);
}

TEST(schlickNormalIncidenceForGlassAndWater) {
    // Glass: ((1 − 1.5)/(1 + 1.5))² = (−0.2)² = 0.04
    EXPECT(isApproximatelyEqual(SchlickFresnel::reflectanceAtNormalIncidence(1.0f, 1.5f), 0.04f));
    // Water: ((1 − 1.33)/(1 + 1.33))² = (−0.33/2.33)² = 0.020060
    EXPECT(isApproximatelyEqual(SchlickFresnel::reflectanceAtNormalIncidence(1.0f, 1.33f), 0.020060f));
}

TEST(schlickNormalIncidenceIsSymmetricAndZeroForEqualIndices) {
    EXPECT(isApproximatelyEqual(SchlickFresnel::reflectanceAtNormalIncidence(1.5f, 1.0f), 0.04f));
    EXPECT(SchlickFresnel::reflectanceAtNormalIncidence(1.3f, 1.3f) == 0);
}

TEST(schlickEndpointsAreNormalIncidenceAndOne) {
    EXPECT(SchlickFresnel::reflectance(1.0f, glassNormalIncidence) == glassNormalIncidence);
    EXPECT(SchlickFresnel::reflectance(0.0f, glassNormalIncidence) == 1);
}

TEST(schlickMidValueMatchesTheHandWorkedResult) {
    // cos = 0.5: (1 − 0.5)⁵ = 0.03125; F = 0.04 + 0.96 · 0.03125 = 0.04 + 0.03 = 0.07
    EXPECT(isApproximatelyEqual(SchlickFresnel::reflectance(0.5f, glassNormalIncidence), 0.07f));
}

TEST(schlickClampsCosinesOutsideZeroToOne) {
    // A cosine above 1 is treated as 1, a negative one as 0 (grazing).
    EXPECT(SchlickFresnel::reflectance(1.5f, glassNormalIncidence) == glassNormalIncidence);
    EXPECT(SchlickFresnel::reflectance(-0.25f, glassNormalIncidence) == 1);
}

TEST(schlickRisesSteadilyTowardGrazingAngles) {
    float previous = 0;
    for (int degrees = 0; degrees <= 90; degrees += 1) {
        float value = SchlickFresnel::reflectance(Angle::degrees(static_cast<float>(degrees)).cosine(), glassNormalIncidence);
        EXPECT(value >= previous);
        EXPECT(value >= glassNormalIncidence - 1e-6f && value <= 1 + 1e-6f);
        previous = value;
    }
}
