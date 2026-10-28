// The shader-library kernels (Kernels/ShaderMath.h) on the CPU, against values worked out by hand
// from each function's formula, and the properties each must keep. The same header is compiled as
// Metal by `make kernels`.
#include <cmath>
#include <limits>

#include "../Kernels/ShaderMath.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;
using Kernel3 = kernels::Vector3Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

constexpr float halfTurn = 3.14159265359f;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }
bool nearly(Kernel3 first, Kernel3 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance);
}
bool nearly(Kernel4 first, Kernel4 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance)
        && nearly(first.w, second.w, tolerance);
}
} // namespace

TEST(shaderMathConstants) {
    EXPECT(kernels::pi<float>() == 3.14159265359f);
    EXPECT(nearly(kernels::tau<float>(), 2.0f * kernels::pi<float>()));
    EXPECT(nearly(kernels::eulerNumber<float>(), std::exp(1.0f)));
    EXPECT(nearly(kernels::squareRootOfTwo<float>() * kernels::squareRootOfTwo<float>(), 2.0f));
    // φ² = φ + 1: 1.618034² = 2.618034
    EXPECT(nearly(kernels::goldenRatio<float>() * kernels::goldenRatio<float>(), kernels::goldenRatio<float>() + 1.0f));
    EXPECT(kernels::namedConstant<float>(0) == kernels::pi<float>());
    EXPECT(kernels::namedConstant<float>(1) == kernels::tau<float>());
    EXPECT(kernels::namedConstant<float>(2) == kernels::eulerNumber<float>());
    EXPECT(kernels::namedConstant<float>(3) == kernels::squareRootOfTwo<float>());
    EXPECT(kernels::namedConstant<float>(4) == kernels::goldenRatio<float>());
    EXPECT(kernels::namedConstant<float>(99) == kernels::goldenRatio<float>());
}

TEST(flooredModuloTakesTheSignOfTheDivisor) {
    // −1 − 3 × ⌊−1/3⌋ = −1 − 3 × (−1) = 2; 7 − (−3) × ⌊7/−3⌋ = 7 − (−3)(−3) = −2
    EXPECT(nearly(kernels::flooredModulo(-1.0f, 3.0f), 2.0f));
    EXPECT(nearly(kernels::flooredModulo(7.0f, -3.0f), -2.0f));
    // 5.5 − 2 × 2 = 1.5; −0.5 − 1 × (−1) = 0.5
    EXPECT(nearly(kernels::flooredModulo(5.5f, 2.0f), 1.5f));
    EXPECT(nearly(kernels::flooredModulo(-0.5f, 1.0f), 0.5f));
    Kernel3 byVector = kernels::flooredModulo(Kernel3(-1.0f, 7.0f, 5.5f), Kernel3(3.0f, -3.0f, 2.0f));
    EXPECT(nearly(byVector, Kernel3(2.0f, -2.0f, 1.5f)));
    // One number as the divisor for every component: (−1, 7, 5.5) mod 3 = (2, 1, 2.5)
    EXPECT(nearly(kernels::flooredModulo(Kernel3(-1.0f, 7.0f, 5.5f), 3.0f), Kernel3(2.0f, 1.0f, 2.5f)));
}

TEST(flooredModuloResultHasTheDivisorsSignAndStaysWithinIt) {
    for (int step = -40; step <= 40; step += 1) {
        float value = static_cast<float>(step) * 0.731f;
        float positive = kernels::flooredModulo(value, 2.5f);
        EXPECT(positive >= 0.0f && positive <= 2.5f);
        float negative = kernels::flooredModulo(value, -2.5f);
        EXPECT(negative <= 0.0f && negative >= -2.5f);
    }
}

TEST(degreesToRadiansConvertsHalfAndQuarterTurns) {
    // 180 × (π / 180) = π; 90 → π/2
    EXPECT(nearly(kernels::degreesToRadians(180.0f), halfTurn));
    EXPECT(nearly(kernels::degreesToRadians(90.0f), halfTurn / 2.0f));
    EXPECT(kernels::degreesToRadians(0.0f) == 0.0f);
}

TEST(safeDivideGivesZeroForANearZeroDivisor) {
    EXPECT(kernels::safeDivide(6.0f, 3.0f) == 2.0f);
    EXPECT(kernels::safeDivide(6.0f, 0.0f) == 0.0f);
    EXPECT(kernels::safeDivide(1.0f, 1e-7f) == 0.0f);
    EXPECT(kernels::safeDivide(1.0f, -1e-7f) == 0.0f);
    EXPECT(nearly(kernels::safeDivide(Kernel3(6.0f, 1.0f, -4.0f), Kernel3(3.0f, 0.0f, -2.0f)), Kernel3(2.0f, 0.0f, 2.0f)));
}

TEST(safeModuloWrapsIntoTheDivisorAndGuardsZero) {
    // mod(7, 3) = 1; +3 = 4; mod(4, 3) = 1
    EXPECT(nearly(kernels::safeModulo(7.0f, 3.0f), 1.0f));
    // mod(−1, 3) = 2; +3 = 5; mod(5, 3) = 2
    EXPECT(nearly(kernels::safeModulo(-1.0f, 3.0f), 2.0f));
    // mod(5, −3) = 5 − (−3)(−2) = −1; + (−3) = −4; mod(−4, −3) = −4 − (−3)(1) = −1
    EXPECT(nearly(kernels::safeModulo(5.0f, -3.0f), -1.0f));
    EXPECT(kernels::safeModulo(5.0f, 0.0f) == 0.0f);
    EXPECT(nearly(kernels::safeModulo(Kernel3(7.0f, -1.0f, 5.0f), Kernel3(3.0f, 3.0f, 0.0f)), Kernel3(1.0f, 2.0f, 0.0f)));
}

TEST(luminanceWeightsGreenMost) {
    EXPECT(nearly(kernels::luminance(Kernel3(1.0f, 0.0f, 0.0f)), 0.2126f));
    EXPECT(nearly(kernels::luminance(Kernel3(0.0f, 1.0f, 0.0f)), 0.7152f));
    EXPECT(nearly(kernels::luminance(Kernel3(0.0f, 0.0f, 1.0f)), 0.0722f));
    // 0.2126 + 0.7152 + 0.0722 = 1
    EXPECT(nearly(kernels::luminance(Kernel3(1.0f, 1.0f, 1.0f)), 1.0f));
    // The alpha of a four-component colour is ignored.
    EXPECT(nearly(kernels::luminance(Kernel4(0.0f, 1.0f, 0.0f, 0.3f)), 0.7152f));
}

TEST(rotatedCounterClockwiseTurnsAPointAboutTheOrigin) {
    // (1, 0) by π/2: (cos − 0, sin + 0) = (0, 1)
    Kernel2 quarter = kernels::rotatedCounterClockwise(Kernel2(1.0f, 0.0f), halfTurn / 2.0f);
    EXPECT(nearly(quarter.x, 0.0f) && nearly(quarter.y, 1.0f));
    // (1, 2) by π: (−1, −2)
    Kernel2 half = kernels::rotatedCounterClockwise(Kernel2(1.0f, 2.0f), halfTurn);
    EXPECT(nearly(half.x, -1.0f) && nearly(half.y, -2.0f));
    // The length is kept: (3, 4) turned by 1 radian is still 5 long.
    Kernel2 turned = kernels::rotatedCounterClockwise(Kernel2(3.0f, 4.0f), 1.0f);
    EXPECT(nearly(kernels::lengthOf(turned), 5.0f, 1e-4f));
}

TEST(remappedMovesBetweenRanges) {
    // 100 + (5 − 0)/(10 − 0) × 100 = 150; past the end it goes on: 100 + 1.5 × 100 = 250
    EXPECT(nearly(kernels::remapped(5.0f, 0.0f, 10.0f, 100.0f, 200.0f), 150.0f));
    EXPECT(nearly(kernels::remapped(15.0f, 0.0f, 10.0f, 100.0f, 200.0f), 250.0f));
    // A zero-width source range gives toMin.
    EXPECT(kernels::remapped(5.0f, 3.0f, 3.0f, 7.0f, 9.0f) == 7.0f);
    Kernel3 mapped = kernels::remapped(Kernel3(5.0f, 15.0f, 0.0f), Kernel3(0.0f), Kernel3(10.0f), Kernel3(100.0f), Kernel3(200.0f));
    EXPECT(nearly(mapped, Kernel3(150.0f, 250.0f, 100.0f)));
}

TEST(remappedClampedStaysBetweenTheTargetEnds) {
    EXPECT(nearly(kernels::remappedClamped(15.0f, 0.0f, 10.0f, 100.0f, 200.0f), 200.0f));
    EXPECT(nearly(kernels::remappedClamped(-5.0f, 0.0f, 10.0f, 100.0f, 200.0f), 100.0f));
    // A reversed target: 200 + 1.5 × (100 − 200) = 50, kept in 100…200 → 100
    EXPECT(nearly(kernels::remappedClamped(15.0f, 0.0f, 10.0f, 200.0f, 100.0f), 100.0f));
    for (int step = -30; step <= 30; step += 1) {
        float result = kernels::remappedClamped(static_cast<float>(step) * 0.9f, -10.0f, 10.0f, 5.0f, -5.0f);
        EXPECT(result >= -5.0f && result <= 5.0f);
    }
}

TEST(safeSmoothStepIsAnSCurveBetweenTheEdges) {
    EXPECT(nearly(kernels::safeSmoothStep(0.0f, 1.0f, 0.5f), 0.5f));
    // t = 0.25: 0.25² × (3 − 0.5) = 0.0625 × 2.5 = 0.15625
    EXPECT(nearly(kernels::safeSmoothStep(0.0f, 1.0f, 0.25f), 0.15625f));
    EXPECT(kernels::safeSmoothStep(0.0f, 1.0f, -3.0f) == 0.0f);
    EXPECT(kernels::safeSmoothStep(0.0f, 1.0f, 3.0f) == 1.0f);
    // Equal edges make a hard step.
    EXPECT(kernels::safeSmoothStep(2.0f, 2.0f, 1.0f) == 0.0f);
    EXPECT(kernels::safeSmoothStep(2.0f, 2.0f, 3.0f) == 1.0f);
    // Reversed edges fall instead of rising: edge0 = 1, edge1 = 0, value 0.25 → t = 0.75 → 0.84375
    EXPECT(nearly(kernels::safeSmoothStep(1.0f, 0.0f, 0.25f), 0.84375f));
    EXPECT(nearly(kernels::safeSmoothStep(Kernel3(0.0f), Kernel3(1.0f, 1.0f, 0.0f), Kernel3(0.5f, 0.25f, 1.0f)),
        Kernel3(0.5f, 0.15625f, 1.0f)));
}

TEST(smootherStepUsesPerlinsQuintic) {
    EXPECT(nearly(kernels::smootherStep(0.0f, 1.0f, 0.5f), 0.5f));
    // t = 0.25: 0.25³ × (0.25 × (6 × 0.25 − 15) + 10) = 0.015625 × (0.25 × (−13.5) + 10) = 0.015625 × 6.625 = 0.103515625
    EXPECT(nearly(kernels::smootherStep(0.0f, 1.0f, 0.25f), 0.103515625f));
    EXPECT(kernels::smootherStep(0.0f, 1.0f, -1.0f) == 0.0f);
    EXPECT(kernels::smootherStep(0.0f, 1.0f, 2.0f) == 1.0f);
    EXPECT(nearly(kernels::smootherStep(Kernel3(0.0f), Kernel3(2.0f), Kernel3(1.0f, 0.5f, 3.0f)),
        Kernel3(0.5f, 0.103515625f, 1.0f)));
}

TEST(inverseLinearBlendIsTheInverseOfLinearBlend) {
    // (15 − 10) / (20 − 10) = 0.5
    EXPECT(nearly(kernels::inverseLinearBlend(10.0f, 20.0f, 15.0f), 0.5f));
    EXPECT(kernels::inverseLinearBlend(3.0f, 3.0f, 5.0f) == 0.0f);
    for (int step = 0; step <= 10; step += 1) {
        float amount = static_cast<float>(step) * 0.1f;
        float blended = kernels::linearBlend(2.0f, 12.0f, amount);
        EXPECT(nearly(kernels::inverseLinearBlend(2.0f, 12.0f, blended), amount, 1e-5f));
    }
}

TEST(snappedToMultipleRoundsToTheNearestStep) {
    // ⌊7/5 + 0.5⌋ × 5 = ⌊1.9⌋ × 5 = 5; ⌊7.5/5 + 0.5⌋ × 5 = ⌊2⌋ × 5 = 10; ⌊−7/5 + 0.5⌋ × 5 = ⌊−0.9⌋ × 5 = −5
    EXPECT(kernels::snappedToMultiple(7.0f, 5.0f) == 5.0f);
    EXPECT(kernels::snappedToMultiple(7.5f, 5.0f) == 10.0f);
    EXPECT(kernels::snappedToMultiple(-7.0f, 5.0f) == -5.0f);
    EXPECT(kernels::snappedToMultiple(7.0f, 0.0f) == 7.0f);
}

TEST(pingPongBouncesBetweenZeroAndLength) {
    // length 2: the period is 4; value 3 → t = 3, 2 − |3 − 2| = 1; −1 → t = 3 → 1
    EXPECT(nearly(kernels::pingPong(0.0f, 2.0f), 0.0f));
    EXPECT(nearly(kernels::pingPong(1.0f, 2.0f), 1.0f));
    EXPECT(nearly(kernels::pingPong(2.0f, 2.0f), 2.0f));
    EXPECT(nearly(kernels::pingPong(3.0f, 2.0f), 1.0f));
    EXPECT(nearly(kernels::pingPong(4.0f, 2.0f), 0.0f));
    EXPECT(nearly(kernels::pingPong(-1.0f, 2.0f), 1.0f));
    // A negative length counts as its size.
    EXPECT(nearly(kernels::pingPong(3.0f, -2.0f), 1.0f));
}

TEST(pingPongStaysInRange) {
    for (int step = -200; step <= 200; step += 1) {
        float value = static_cast<float>(step) * 0.173f;
        float result = kernels::pingPong(value, 1.5f);
        EXPECT(result >= -1e-5f && result <= 1.5f + 1e-5f);
    }
    // A zero length does not divide by zero: the result is tiny.
    EXPECT(kernels::pingPong(3.0f, 0.0f) <= 1e-5f);
    EXPECT(nearly(kernels::pingPong(Kernel3(1.0f, 3.0f, -1.0f), Kernel3(2.0f)), Kernel3(1.0f, 1.0f, 1.0f)));
}

TEST(wrappedIntoRepeatsWithinTheRange) {
    // 0 + mod(12 − 0, 10) = 2; −1 → mod(−1, 10) = 9; −2 + mod(3 + 2, 4) = −2 + 1 = −1
    EXPECT(nearly(kernels::wrappedInto(12.0f, 0.0f, 10.0f), 2.0f));
    EXPECT(nearly(kernels::wrappedInto(-1.0f, 0.0f, 10.0f), 9.0f));
    EXPECT(nearly(kernels::wrappedInto(3.0f, -2.0f, 2.0f), -1.0f));
    EXPECT(kernels::wrappedInto(7.0f, 5.0f, 5.0f) == 5.0f);
}

TEST(roundedHalfUpRoundsHalvesTowardPositiveInfinity) {
    EXPECT(kernels::roundedHalfUp(2.5f) == 3.0f);
    EXPECT(kernels::roundedHalfUp(-2.5f) == -2.0f);
    EXPECT(kernels::roundedHalfUp(2.4f) == 2.0f);
    EXPECT(kernels::roundedHalfUp(-2.6f) == -3.0f);
}

TEST(safeReciprocalGivesZeroForZero) {
    EXPECT(kernels::safeReciprocal(4.0f) == 0.25f);
    EXPECT(kernels::safeReciprocal(0.0f) == 0.0f);
    EXPECT(nearly(kernels::safeReciprocal(Kernel3(2.0f, 0.0f, -4.0f)), Kernel3(0.5f, 0.0f, -0.25f)));
}

TEST(safeLogarithmAndSafePowerNeverMakeNaN) {
    // log₂ 8 = 3; log₁₀ 100 = 2
    EXPECT(nearly(kernels::safeLogarithm(8.0f, 2.0f), 3.0f, 1e-5f));
    EXPECT(nearly(kernels::safeLogarithm(100.0f, 10.0f), 2.0f, 1e-5f));
    // 0 is raised to 0.000001 first: ln(1e-6) / ln 2 = −13.8155106 / 0.6931472 = −19.93157
    EXPECT(nearly(kernels::safeLogarithm(0.0f, 2.0f), -19.93157f, 1e-3f));
    EXPECT(nearly(kernels::safeLogarithm(Kernel3(8.0f, 4.0f, 1.0f), 2.0f), Kernel3(3.0f, 2.0f, 0.0f), 1e-5f));
    EXPECT(nearly(kernels::safePower(2.0f, 3.0f), 8.0f));
    EXPECT(kernels::safePower(-2.0f, 3.0f) == 0.0f);
    EXPECT(kernels::safePower(-2.0f, 0.5f) == 0.0f);
    EXPECT(nearly(kernels::safePower(Kernel3(2.0f, -2.0f, 9.0f), Kernel3(3.0f, 3.0f, 0.5f)), Kernel3(8.0f, 0.0f, 3.0f)));
}

TEST(smoothMinimumRoundsTheCrease) {
    // a = b = 1, r = 1: h = 0.5 + 0 = 0.5; mix = 1; 1 − 1 × 0.5 × 0.5 = 0.75
    EXPECT(nearly(kernels::smoothMinimum(1.0f, 1.0f, 1.0f), 0.75f));
    // Far apart (|b − a| ≥ r) it is the plain minimum: h = clamp(0.5 + 5) = 1 → a = 0
    EXPECT(nearly(kernels::smoothMinimum(0.0f, 10.0f, 1.0f), 0.0f));
    EXPECT(nearly(kernels::smoothMinimum(10.0f, 0.0f, 1.0f), 0.0f));
    // −smoothMin(−1, −1, 1) = −(−1 − 0.25) = 1.25
    EXPECT(nearly(kernels::smoothMaximum(1.0f, 1.0f, 1.0f), 1.25f));
    EXPECT(nearly(kernels::smoothMinimum(Kernel3(1.0f, 0.0f, 10.0f), Kernel3(1.0f, 10.0f, 0.0f), 1.0f), Kernel3(0.75f, 0.0f, 0.0f)));
    for (int step = -20; step <= 20; step += 1) {
        float other = static_cast<float>(step) * 0.2f;
        EXPECT(kernels::smoothMinimum(0.5f, other, 1.0f) <= kernels::smallerOf(0.5f, other) + 1e-6f);
        EXPECT(kernels::smoothMaximum(0.5f, other, 1.0f) >= kernels::largerOf(0.5f, other) - 1e-6f);
    }
}

TEST(comparisonResultGivesOneOrZeroForEveryMode) {
    // Equal within 0.01: |1 − 1.005| = 0.005 ≤ 0.01
    EXPECT(kernels::comparisonResult(1.0f, 1.005f, 0, 0.01f) == 1.0f);
    EXPECT(kernels::comparisonResult(1.0f, 1.005f, 1, 0.01f) == 0.0f);
    EXPECT(kernels::comparisonResult(1.0f, 2.0f, 2, 0.0f) == 1.0f);
    EXPECT(kernels::comparisonResult(2.0f, 2.0f, 2, 0.0f) == 0.0f);
    EXPECT(kernels::comparisonResult(2.0f, 2.0f, 3, 0.0f) == 1.0f);
    EXPECT(kernels::comparisonResult(1.0f, 2.0f, 4, 0.0f) == 0.0f);
    EXPECT(kernels::comparisonResult(3.0f, 2.0f, 4, 0.0f) == 1.0f);
    EXPECT(kernels::comparisonResult(2.0f, 2.0f, 5, 0.0f) == 1.0f);
    EXPECT(kernels::comparisonResult(1.0f, 2.0f, 5, 0.0f) == 0.0f);
    Kernel3 byComponent = kernels::comparisonResult(Kernel3(1.0f, 2.0f, 3.0f), Kernel3(2.0f), 2, Kernel3(0.0f));
    EXPECT(nearly(byComponent, Kernel3(1.0f, 0.0f, 0.0f)));
}

TEST(logicFunctionsTreatAboveHalfAsTrue) {
    EXPECT(kernels::logicalAnd(1.0f, 1.0f) == 1.0f && kernels::logicalAnd(1.0f, 0.0f) == 0.0f && kernels::logicalAnd(0.0f, 1.0f) == 0.0f);
    EXPECT(kernels::logicalOr(0.0f, 0.0f) == 0.0f && kernels::logicalOr(0.0f, 1.0f) == 1.0f && kernels::logicalOr(1.0f, 0.0f) == 1.0f);
    EXPECT(kernels::logicalExclusiveOr(1.0f, 1.0f) == 0.0f && kernels::logicalExclusiveOr(1.0f, 0.0f) == 1.0f
        && kernels::logicalExclusiveOr(0.0f, 1.0f) == 1.0f && kernels::logicalExclusiveOr(0.0f, 0.0f) == 0.0f);
    EXPECT(kernels::logicalNot(1.0f) == 0.0f && kernels::logicalNot(0.0f) == 1.0f);
    // 0.5 itself is false; 0.51 is true.
    EXPECT(kernels::logicalNot(0.5f) == 1.0f && kernels::logicalNot(0.51f) == 0.0f);
    Kernel3 left(1.0f, 1.0f, 0.0f);
    Kernel3 right(1.0f, 0.0f, 0.0f);
    EXPECT(nearly(kernels::logicalAnd(left, right), Kernel3(1.0f, 0.0f, 0.0f)));
    EXPECT(nearly(kernels::logicalOr(left, right), Kernel3(1.0f, 1.0f, 0.0f)));
    EXPECT(nearly(kernels::logicalExclusiveOr(left, right), Kernel3(0.0f, 1.0f, 0.0f)));
    EXPECT(nearly(kernels::logicalNot(left), Kernel3(0.0f, 0.0f, 1.0f)));
}

TEST(flagsSpotNotANumberAndInfinity) {
    float notANumber = std::numeric_limits<float>::quiet_NaN();
    float infinity = std::numeric_limits<float>::infinity();
    EXPECT(kernels::flagNotANumber(notANumber) == 1.0f);
    EXPECT(kernels::flagNotANumber(1.0f) == 0.0f);
    EXPECT(kernels::flagNotANumber(infinity) == 0.0f);
    EXPECT(kernels::flagInfinity(infinity) == 1.0f);
    EXPECT(kernels::flagInfinity(-infinity) == 1.0f);
    EXPECT(kernels::flagInfinity(1e30f) == 0.0f);
    EXPECT(kernels::flagInfinity(notANumber) == 0.0f);
    Kernel3 flagged = kernels::flagNotANumber(Kernel3(1.0f, notANumber, 3.0f));
    EXPECT(flagged.x == 0.0f && flagged.y == 1.0f && flagged.z == 0.0f);
}

TEST(bounceEaseOutFollowsItsFourParabolas) {
    EXPECT(kernels::bounceEaseOut(0.0f) == 0.0f);
    // 0.1 < 1/2.75: 7.5625 × 0.01 = 0.075625
    EXPECT(nearly(kernels::bounceEaseOut(0.1f), 0.075625f));
    // 0.5 < 2/2.75: x = 0.5 − 1.5/2.75 = −0.0454545; 7.5625 × 0.00206612 + 0.75 = 0.765625
    EXPECT(nearly(kernels::bounceEaseOut(0.5f), 0.765625f));
    // 1: x = 1 − 2.625/2.75 = 0.0454545; 7.5625 × 0.00206612 + 0.984375 = 1
    EXPECT(nearly(kernels::bounceEaseOut(1.0f), 1.0f, 1e-4f));
}

TEST(easedCurveMatchesHandWorkedValues) {
    EXPECT(nearly(kernels::easedCurve(0.3f, 0), 0.3f));
    // quad: in 0.25, out 1 − 0.25 = 0.75, in-out at 0.25 is 2 × 0.0625 = 0.125 and at 0.75 is 1 − 0.25/2 = 0.875
    EXPECT(nearly(kernels::easedCurve(0.5f, 1), 0.25f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 2), 0.75f));
    EXPECT(nearly(kernels::easedCurve(0.25f, 3), 0.125f));
    EXPECT(nearly(kernels::easedCurve(0.75f, 3), 0.875f));
    // cubic: in 0.125, out 1 − 0.125 = 0.875, in-out at 0.25 is 4 × 0.015625 = 0.0625
    EXPECT(nearly(kernels::easedCurve(0.5f, 4), 0.125f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 5), 0.875f));
    EXPECT(nearly(kernels::easedCurve(0.25f, 6), 0.0625f));
    // expo: in 2⁻⁵ = 0.03125, out 1 − 2⁻⁵ = 0.96875, in-out at 0.5 is (2 − 2⁰)/2 = 0.5
    EXPECT(nearly(kernels::easedCurve(0.5f, 7), 0.03125f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 8), 0.96875f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 9), 0.5f));
    // sine: in 1 − cos(π/4) = 0.292893, out sin(π/4) = 0.707107, in-out −(cos(π/2) − 1)/2 = 0.5
    EXPECT(nearly(kernels::easedCurve(0.5f, 10), 0.292893f, 1e-4f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 11), 0.707107f, 1e-4f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 12), 0.5f, 1e-4f));
    // back in at 0.5: 2.70158 × 0.125 − 1.70158 × 0.25 = 0.3376975 − 0.425395 = −0.0876975 (it dips below 0);
    // back out is its mirror: 1 + 0.0876975
    EXPECT(nearly(kernels::easedCurve(0.5f, 13), -0.0876975f, 1e-4f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 14), 1.0876975f, 1e-4f));
    // back in-out at 0.5: x ≥ 0.5 branch: (1 × ((c2 + 1)(−1) + c2) + 2) / 2 = (−1 + 2) / 2 = 0.5
    EXPECT(nearly(kernels::easedCurve(0.5f, 15), 0.5f, 1e-4f));
    // bounce out at 0.5 is 0.765625; bounce in is 1 − bounceOut(0.5) = 0.234375
    EXPECT(nearly(kernels::easedCurve(0.5f, 19), 0.765625f));
    EXPECT(nearly(kernels::easedCurve(0.5f, 18), 0.234375f));
    // The input is clamped to 0…1.
    EXPECT(kernels::easedCurve(-1.0f, 1) == 0.0f);
    EXPECT(kernels::easedCurve(2.0f, 1) == 1.0f);
}

TEST(everyEasingCurveStartsAtZeroAndEndsAtOne) {
    for (int mode = 0; mode <= 19; mode += 1) {
        EXPECT(nearly(kernels::easedCurve(0.0f, mode), 0.0f, 1e-4f));
        EXPECT(nearly(kernels::easedCurve(1.0f, mode), 1.0f, 1e-4f));
    }
}

TEST(dotLengthDistanceAndNormalizeTakeNumbersAndVectors) {
    EXPECT(kernels::dotProductOfValues(3.0f, 4.0f) == 12.0f);
    // (1, 2, 3) · (4, −5, 6) = 4 − 10 + 18 = 12
    EXPECT(kernels::dotProductOfValues(Kernel3(1, 2, 3), Kernel3(4, -5, 6)) == 12.0f);
    EXPECT(kernels::lengthOfValue(-3.0f) == 3.0f);
    // √(4 + 9 + 36) = 7
    EXPECT(kernels::lengthOfValue(Kernel3(2, 3, 6)) == 7.0f);
    EXPECT(kernels::lengthOfValue(Kernel2(3, 4)) == 5.0f);
    EXPECT(kernels::distanceBetweenValues(1.0f, 4.0f) == 3.0f);
    EXPECT(kernels::distanceBetweenValues(Kernel2(1, 1), Kernel2(4, 5)) == 5.0f);
    // A number's direction is its sign, with 0 counting as positive.
    EXPECT(kernels::normalizedValue(-2.0f) == -1.0f && kernels::normalizedValue(5.0f) == 1.0f && kernels::normalizedValue(0.0f) == 1.0f);
    EXPECT(nearly(kernels::normalizedValue(Kernel3(0, 0, -4)), Kernel3(0, 0, -1)));
    // A zero vector stays zero instead of becoming NaN.
    EXPECT(nearly(kernels::normalizedValue(Kernel3(0, 0, 0)), Kernel3(0, 0, 0)));
}

TEST(refractedValueFollowsSnellsLaw) {
    // Straight in: the direction does not change.
    EXPECT(nearly(kernels::refractedValue(Kernel3(0, 0, -1), Kernel3(0, 0, 1), 0.5f), Kernel3(0, 0, -1)));
    // Oblique, eta 0.5: c = −0.6, k = 1 − 0.25 × (1 − 0.36) = 0.84, √k = 0.916515;
    // result = 0.5 × (0.8, 0, −0.6) − (0.5 × (−0.6) + 0.916515) × (0, 0, 1) = (0.4, 0, −0.3 − 0.616515)
    Kernel3 bent = kernels::refractedValue(Kernel3(0.8f, 0, -0.6f), Kernel3(0, 0, 1), 0.5f);
    EXPECT(nearly(bent, Kernel3(0.4f, 0.0f, -0.916515f), 1e-5f));
    // Total internal reflection: k = 1 − 2.25 × 0.64 = −0.44 < 0 → zero.
    EXPECT(nearly(kernels::refractedValue(Kernel3(0.8f, 0, -0.6f), Kernel3(0, 0, 1), 1.5f), Kernel3(0, 0, 0)));
}

TEST(projectedValueAndAngleBetweenValues) {
    EXPECT(nearly(kernels::projectedValue(Kernel3(3, 4, 0), Kernel3(1, 0, 0)), Kernel3(3, 0, 0)));
    EXPECT(kernels::projectedValue(3.0f, 2.0f) == 3.0f);
    // Onto a zero vector: 0 × (0 / 0.000001) = 0.
    EXPECT(nearly(kernels::projectedValue(Kernel3(3, 4, 0), Kernel3(0, 0, 0)), Kernel3(0, 0, 0)));
    EXPECT(nearly(kernels::angleBetweenValues(Kernel3(1, 0, 0), Kernel3(0, 1, 0)), halfTurn / 2.0f));
    EXPECT(nearly(kernels::angleBetweenValues(Kernel3(1, 0, 0), Kernel3(-1, 0, 0)), halfTurn));
    EXPECT(nearly(kernels::angleBetweenValues(Kernel3(1, 0, 0), Kernel3(2, 0, 0)), 0.0f));
    // A zero vector counts as at right angles: acos(0) = π/2.
    EXPECT(nearly(kernels::angleBetweenValues(Kernel3(0, 0, 0), Kernel3(1, 0, 0)), halfTurn / 2.0f));
    EXPECT(nearly(kernels::angleBetweenValues(2.0f, -3.0f), halfTurn));
}

TEST(componentsSwizzleAndIndexSelection) {
    Kernel4 value(10, 20, 30, 40);
    EXPECT(kernels::componentOf(value, 2) == 30.0f);
    EXPECT(kernels::componentOf(value, 9) == 40.0f);
    EXPECT(kernels::componentOf(value, -3) == 10.0f);
    // which = (3, 2, 4, 5) picks (w, z, 0, 1); 4 is zero and 5 is one.
    kernels::Vector4Of<int> which(3, 2, 4, 5);
    EXPECT(nearly(kernels::swizzled(value, which), Kernel4(40, 30, 0, 1)));
    // Out of range is moved in: −1 → x, 9 → one.
    EXPECT(nearly(kernels::swizzled(value, kernels::Vector4Of<int>(-1, 9, 0, 1)), Kernel4(10, 1, 10, 20)));
    // ⌊1.6 + 0.5⌋ = 2 → the third; ⌊0.4 + 0.5⌋ = 0; ⌊0.5 + 0.5⌋ = 1; clamped at both ends.
    EXPECT(kernels::selectedByIndex(1.6f, 1.0f, 2.0f, 3.0f, 4.0f) == 3.0f);
    EXPECT(kernels::selectedByIndex(0.4f, 1.0f, 2.0f, 3.0f, 4.0f) == 1.0f);
    EXPECT(kernels::selectedByIndex(0.5f, 1.0f, 2.0f, 3.0f, 4.0f) == 2.0f);
    EXPECT(kernels::selectedByIndex(-3.0f, 1.0f, 2.0f, 3.0f, 4.0f) == 1.0f);
    EXPECT(kernels::selectedByIndex(9.0f, 1.0f, 2.0f, 3.0f, 4.0f) == 4.0f);
    EXPECT(nearly(kernels::selectedByIndex(2.0f, Kernel3(1), Kernel3(2), Kernel3(3), Kernel3(4)), Kernel3(3)));
}

TEST(waveShapesHitTheirHandWorkedValues) {
    // sine: sin(2π × 0.25) = 1, sin(π) ≈ 0
    EXPECT(nearly(kernels::waveShape(0.25f, 0), 1.0f));
    EXPECT(nearly(kernels::waveShape(0.5f, 0), 0.0f, 1e-5f));
    // triangle 1 − 4 |fract(x + 0.25) − 0.5|: x = 0 → 1 − 4 × 0.25 = 0; 0.25 → 1; 0.5 → 0; 0.75 → fract(1) = 0 → −1
    EXPECT(nearly(kernels::waveShape(0.0f, 1), 0.0f));
    EXPECT(nearly(kernels::waveShape(0.25f, 1), 1.0f));
    EXPECT(nearly(kernels::waveShape(0.5f, 1), 0.0f));
    EXPECT(nearly(kernels::waveShape(0.75f, 1), -1.0f));
    // square: 1 while fract(x) < 0.5, else −1
    EXPECT(kernels::waveShape(0.25f, 2) == 1.0f && kernels::waveShape(0.5f, 2) == -1.0f && kernels::waveShape(0.75f, 2) == -1.0f);
    // sawtooth 2 fract(x) − 1: 0 → −1, 0.25 → −0.5, 0.5 → 0
    EXPECT(nearly(kernels::waveShape(0.0f, 3), -1.0f) && nearly(kernels::waveShape(0.25f, 3), -0.5f) && nearly(kernels::waveShape(0.5f, 3), 0.0f));
    EXPECT(nearly(kernels::waveShape(Kernel3(0.25f, 0.5f, 0.75f), 1), Kernel3(1.0f, 0.0f, -1.0f)));
    EXPECT(nearly(kernels::waveShape(Kernel3(0.25f, 0.5f, 0.0f), 0), Kernel3(1.0f, 0.0f, 0.0f), 1e-5f));
}

TEST(everyWaveStaysWithinMinusOneAndOne) {
    for (int shape = 0; shape <= 3; shape += 1) {
        for (int step = -100; step <= 100; step += 1) {
            float value = kernels::waveShape(static_cast<float>(step) * 0.0731f, shape);
            EXPECT(value >= -1.0f - 1e-5f && value <= 1.0f + 1e-5f);
        }
    }
}

TEST(keyframesOfFourLoopsThroughItsValues) {
    Kernel4 values(0, 10, 20, 30);
    // duration 4: time 1 → t = 1 → segment 1 at local 0 → 10
    EXPECT(nearly(kernels::keyframesOfFour(1.0f, values, 4.0f, 0), 10.0f));
    // time 1.5 → t = 1.5 → 10 → 20 at 0.5 → 15, in both linear and smooth modes (smooth(0.5) = 0.5)
    EXPECT(nearly(kernels::keyframesOfFour(1.5f, values, 4.0f, 0), 15.0f));
    EXPECT(nearly(kernels::keyframesOfFour(1.5f, values, 4.0f, 1), 15.0f));
    // time 1.25 → local 0.25: linear 12.5; smooth 0.25² × (3 − 0.5) = 0.15625 → 11.5625; mode 2 holds the first: 10
    EXPECT(nearly(kernels::keyframesOfFour(1.25f, values, 4.0f, 0), 12.5f));
    EXPECT(nearly(kernels::keyframesOfFour(1.25f, values, 4.0f, 1), 11.5625f));
    EXPECT(nearly(kernels::keyframesOfFour(1.25f, values, 4.0f, 2), 10.0f));
    // The last segment blends back to the first: time 3.5 → 30 → 0 at 0.5 → 15; time 5 loops to 1.
    EXPECT(nearly(kernels::keyframesOfFour(3.5f, values, 4.0f, 0), 15.0f));
    EXPECT(nearly(kernels::keyframesOfFour(5.0f, values, 4.0f, 0), 10.0f));
}

TEST(heatmapRunsFromBlueThroughGreenToRed) {
    // x = 0: red clamp(1.5 − 3) = 0, green clamp(1.5 − 2) = 0, blue clamp(1.5 − 1) = 0.5
    EXPECT(nearly(kernels::heatmap(0.0f), Kernel3(0.0f, 0.0f, 0.5f)));
    // x = 0.25: green 1.5 − |1 − 2| = 0.5, blue 1.5 − |1 − 1| = 1.5 → 1
    EXPECT(nearly(kernels::heatmap(0.25f), Kernel3(0.0f, 0.5f, 1.0f)));
    // x = 0.5: red 1.5 − |2 − 3| = 0.5, green 1.5 → 1, blue 1.5 − |2 − 1| = 0.5
    EXPECT(nearly(kernels::heatmap(0.5f), Kernel3(0.5f, 1.0f, 0.5f)));
    // x = 1: red 1.5 − 1 = 0.5, green 1.5 − 2 → 0, blue 0
    EXPECT(nearly(kernels::heatmap(1.0f), Kernel3(0.5f, 0.0f, 0.0f)));
    // Clamped outside 0…1.
    EXPECT(nearly(kernels::heatmap(5.0f), kernels::heatmap(1.0f)));
}

TEST(debugColorShowsNumbersAndVectors) {
    // mode 0: the heat map of 0.25 with alpha 1
    EXPECT(nearly(kernels::debugColor(0.25f, 0), Kernel4(0.0f, 0.5f, 1.0f, 1.0f)));
    // mode 1: fract(2.5) = 0.5 is not < 0.5, so the band is 0.75: (0, 0.5, 1) × 0.75
    EXPECT(nearly(kernels::debugColor(0.25f, 1), Kernel4(0.0f, 0.375f, 0.75f, 1.0f)));
    // fract(0.4) < 0.5 → band 0.25; heat map of 0.04 is (0, 0, 0.66) → (0, 0, 0.165)
    EXPECT(nearly(kernels::debugColor(0.04f, 1), Kernel4(0.0f, 0.0f, 0.165f, 1.0f)));
    // mode 2: red for negative, green for positive, up to 1
    EXPECT(nearly(kernels::debugColor(-0.5f, 2), Kernel4(0.5f, 0.0f, 0.0f, 1.0f)));
    EXPECT(nearly(kernels::debugColor(2.0f, 2), Kernel4(0.0f, 1.0f, 0.0f, 1.0f)));
    EXPECT(nearly(kernels::debugColor(0.3f, 2), Kernel4(0.0f, 0.3f, 0.0f, 1.0f)));
    // Vectors: mode 2 maps −1…1 to 0…1; other modes show the components.
    Kernel4 plane = kernels::debugColor(Kernel2(-1.0f, 1.0f), 2);
    EXPECT(nearly(plane, Kernel4(0.0f, 1.0f, 0.0f, 1.0f)));
    EXPECT(nearly(kernels::debugColor(Kernel2(-1.0f, 1.0f), 0), Kernel4(-1.0f, 1.0f, 0.0f, 1.0f)));
    EXPECT(nearly(kernels::debugColor(Kernel3(-1.0f, 0.0f, 1.0f), 2), Kernel4(0.0f, 0.5f, 1.0f, 1.0f)));
    EXPECT(nearly(kernels::debugColor(Kernel3(0.1f, 0.2f, 0.3f), 0), Kernel4(0.1f, 0.2f, 0.3f, 1.0f)));
    EXPECT(nearly(kernels::debugColor(Kernel4(-1.0f, 0.0f, 1.0f, 0.3f), 2), Kernel4(0.0f, 0.5f, 1.0f, 1.0f)));
    EXPECT(nearly(kernels::debugColor(Kernel4(0.1f, 0.2f, 0.3f, 0.4f), 0), Kernel4(0.1f, 0.2f, 0.3f, 1.0f)));
}

TEST(layersOverCompositesByAlpha) {
    // base (1, 0, 0, 1) with (0, 0, 1) at alpha 0.25: 1 + (0 − 1) × 0.25 = 0.75, 0, 0 + 1 × 0.25 = 0.25; alpha max(1, 0.25) = 1
    EXPECT(nearly(kernels::layerOver(Kernel4(1, 0, 0, 1), Kernel4(0, 0, 1, 0.25f)), Kernel4(0.75f, 0.0f, 0.25f, 1.0f)));
    // Layer b at alpha 0.5 turns red to (0.5, 0.5, 0); c and d have alpha 0 and change nothing.
    Kernel4 stacked = kernels::layersOver(Kernel4(1, 0, 0, 1), Kernel4(0, 1, 0, 0.5f), Kernel4(0, 0, 1, 0), Kernel4(0, 0, 0, 0));
    EXPECT(nearly(stacked, Kernel4(0.5f, 0.5f, 0.0f, 1.0f)));
}
