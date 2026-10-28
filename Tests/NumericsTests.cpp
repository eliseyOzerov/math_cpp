// Quadratic and cubic roots, Gauss-Legendre integration and the seeded random generator, with
// values worked out by hand (EditorTests/Math/Numerics/).
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "../Numerics/CubicRoots.h"
#include "../Numerics/Integration.h"
#include "../Numerics/QuadraticRoots.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float infinity = std::numeric_limits<float>::infinity();
// Swift's `Float.pi`: rounded toward zero (0x1.921fb4p1), one step below the nearest float.
constexpr float swiftPi = 0x1.921fb4p1f;

std::string describe(const std::vector<float>& values) {
    std::string text = "[";
    for (size_t index = 0; index < values.size(); index += 1) {
        if (index > 0) text += ", ";
        text += std::to_string(values[index]);
    }
    return text + "]";
}

} // namespace

// MARK: Quadratic roots

namespace {

void expectTwo(const QuadraticRoots& roots, float expectedSmaller, float expectedLarger, float relativeTolerance = 1e-5f) {
    EXPECT(roots.kind == QuadraticRoots::Kind::two);
    if (roots.kind != QuadraticRoots::Kind::two) return;
    float scale = std::max({1.0f, std::fabs(expectedSmaller), std::fabs(expectedLarger)});
    EXPECT(isApproximatelyEqual(roots.smaller, expectedSmaller, relativeTolerance * scale));
    EXPECT(isApproximatelyEqual(roots.larger, expectedLarger, relativeTolerance * scale));
}

} // namespace

TEST(quadraticTwoRootsOfAHandWorkedEquation) {
    // x² − 3x + 2: discriminant 9 − 8 = 1. Roots (3 ± 1) / 2 = 1 and 2.
    expectTwo(QuadraticRoots::solve(1, -3, 2), 1, 2);
    // 2x² + 4x − 6 = 2(x + 3)(x − 1): roots −3 and 1. Discriminant 16 + 48 = 64.
    expectTwo(QuadraticRoots::solve(2, 4, -6), -3, 1);
    // −x² + 1: roots −1 and 1, found even though the leading coefficient is negative.
    expectTwo(QuadraticRoots::solve(-1, 0, 1), -1, 1);
}

TEST(quadraticADoubleRootIsReportedOnce) {
    // x² − 2x + 1 = (x − 1)²: discriminant 4 − 4 = 0, root 2 / 2 = 1.
    EXPECT(QuadraticRoots::solve(1, -2, 1) == QuadraticRoots::one(1));
}

TEST(quadraticNoRealRootsWhenTheDiscriminantIsNegative) {
    // x² + 1: discriminant 0 − 4 = −4.
    EXPECT(QuadraticRoots::solve(1, 0, 1) == QuadraticRoots::none());
}

TEST(quadraticAnOverflowingDiscriminantReportsNoRoots) {
    // b² = 1e40 and 4ac = 4e40 both overflow float to infinity, so b² − 4ac = ∞ − ∞ = NaN. NaN
    // fails `>= 0`, so the answer is none rather than two NaN roots.
    EXPECT(QuadraticRoots::solve(1e20f, 1e20f, 1e20f) == QuadraticRoots::none());
}

TEST(quadraticAZeroQuadraticTermMakesItLinear) {
    // 2x − 6 = 0 → x = 3.
    EXPECT(QuadraticRoots::solve(0, 2, -6) == QuadraticRoots::one(3));
    // 0 = 5 has no solution; 0 = 0 has no single solution. Both report none.
    EXPECT(QuadraticRoots::solve(0, 0, 5) == QuadraticRoots::none());
    EXPECT(QuadraticRoots::solve(0, 0, 0) == QuadraticRoots::none());
}

TEST(quadraticRootAtZeroIsFound) {
    // x² − 5x = x(x − 5): roots 0 and 5.
    expectTwo(QuadraticRoots::solve(1, -5, 0), 0, 5);
}

TEST(quadraticSmallRootSurvivesCancellation) {
    // x² + 10000x + 1: roots −(10000 ± √(10⁸ − 4)) / 2. By Vieta the product is 1, so the small
    // root is 1 / (large root) = −1.00000001e-4 (to 9 digits). The textbook formula
    // (−10000 + √(10⁸ − 4)) / 2 would lose almost all its digits in float.
    QuadraticRoots roots = QuadraticRoots::solve(1, 10000, 1);
    EXPECT(roots.kind == QuadraticRoots::Kind::two);
    EXPECT(std::fabs(roots.smaller / -10000 - 1) < 1e-5f);
    EXPECT(std::fabs(roots.larger / -1.00000001e-4f - 1) < 1e-4f);
}

TEST(quadraticRootsBuiltFromKnownRootsAreRecovered) {
    // (quadratic)(x − r1)(x − r2) = quadratic·x² − quadratic(r1 + r2)·x + quadratic·r1·r2.
    for (float scale : {0.5f, 2.0f, -3.0f}) {
        for (float firstRoot : {-4.0f, -1.0f, 0.5f, 3.0f, 8.0f}) {
            for (float secondRoot : {-4.0f, -1.0f, 0.5f, 3.0f, 8.0f}) {
                if (!(secondRoot > firstRoot)) continue;
                QuadraticRoots roots = QuadraticRoots::solve(
                    scale, -scale * (firstRoot + secondRoot), scale * firstRoot * secondRoot);
                expectTwo(roots, firstRoot, secondRoot);
            }
        }
    }
}

TEST(quadraticBothRootsSatisfyTheEquation) {
    const float coefficients[] = {-7, -1, 0.25f, 3, 40};
    for (float quadratic : coefficients) {
        for (float linear : coefficients) {
            for (float constant : coefficients) {
                QuadraticRoots roots = QuadraticRoots::solve(quadratic, linear, constant);
                if (roots.kind != QuadraticRoots::Kind::two) continue;
                EXPECT(roots.smaller < roots.larger);
                for (float root : {roots.smaller, roots.larger}) {
                    float residual = quadratic * root * root + linear * root + constant;
                    float magnitude = std::fabs(quadratic * root * root) + std::fabs(linear * root) + std::fabs(constant);
                    EXPECT(std::fabs(residual) <= 1e-5f * magnitude);
                }
            }
        }
    }
}

// MARK: Cubic roots

namespace {

/// Compares the roots with the expected ones, each within `relativeTolerance` of its own size
/// plus `absoluteFloor` (for an expected root of 0), and checks the count. The default is 2e-7,
/// about three float steps: every exact case below measured 0 error.
void expectRoots(const std::vector<float>& roots, const std::vector<float>& expected, float relativeTolerance = 2e-7f,
                 float absoluteFloor = 0) {
    bool sameCount = roots.size() == expected.size();
    if (!sameCount) {
        testing::fail(__FILE__, __LINE__, "got " + describe(roots) + ", expected " + describe(expected));
        return;
    }
    for (size_t index = 0; index < roots.size(); index += 1) {
        float tolerance = relativeTolerance * std::fabs(expected[index]) + absoluteFloor;
        if (!isApproximatelyEqual(roots[index], expected[index], tolerance)) {
            testing::fail(__FILE__, __LINE__, "got " + describe(roots) + ", expected " + describe(expected));
        }
    }
}

/// The largest error of the roots relative to the expected ones (each against the larger of its
/// own size and 1e-30), or infinity when the counts differ.
double worstRelativeError(const std::vector<float>& roots, const std::vector<double>& expected) {
    if (roots.size() != expected.size()) return std::numeric_limits<double>::infinity();
    double worst = 0;
    for (size_t index = 0; index < roots.size(); index += 1) {
        double error = std::fabs(static_cast<double>(roots[index]) - expected[index]) / std::max(std::fabs(expected[index]), 1e-30);
        worst = std::max(worst, error);
    }
    return worst;
}

/// Solves cubic·(t − r₁)(t − r₂)(t − r₃), expanded in double and rounded to float.
std::vector<float> solveExpanded(const std::vector<double>& roots, double cubic = 1) {
    double sum = roots[0] + roots[1] + roots[2];
    double pairs = roots[0] * roots[1] + roots[0] * roots[2] + roots[1] * roots[2];
    double product = roots[0] * roots[1] * roots[2];
    return CubicRoots::solve(static_cast<float>(cubic), static_cast<float>(-cubic * sum), static_cast<float>(cubic * pairs),
                             static_cast<float>(-cubic * product));
}

/// Expands (t − r₁)(t − r₂)(t − r₃) for whole numbers, exact in float up to |r| ≤ 60.
std::vector<float> solveProduct(float first, float second, float third) {
    float sum = first + second + third;
    float pairs = first * second + first * third + second * third;
    float product = first * second * third;
    return CubicRoots::solve(1, -sum, pairs, -product);
}

/// A deterministic stream of numbers in −1…1, so the property tests are repeatable.
struct Sequence {
    std::uint64_t state = 0x9E3779B97F4A7C15ull;

    float next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<float>(state >> 40) / static_cast<float>(1 << 23) - 1;
    }

    /// A leading coefficient of either sign whose size is spread evenly over the powers of two
    /// from 2⁻⁴⁰ (about 1e-12) to 2¹⁰ (about 1e3), so tiny ones are as common as big ones.
    float nextLeadingCoefficient() {
        int exponent = static_cast<int>((next() + 1) * 25) - 40;
        float significand = 1.5f + next() / 2;
        bool negative = next() < 0;
        float magnitude = std::ldexp(significand, exponent);
        return negative ? -magnitude : magnitude;
    }
};

} // namespace

TEST(cubicThreeDistinctRoots) {
    // (t − 1)(t − 2)(t − 3) = t³ − 6t² + 11t − 6.
    expectRoots(CubicRoots::solve(1, -6, 11, -6), {1, 2, 3});
}

TEST(cubicANegativeLeadingCoefficientGivesTheSameRoots) {
    expectRoots(CubicRoots::solve(-1, 6, -11, 6), {1, 2, 3});
    // 2(t − 1)(t − 2)(t − 3) as well: the overall factor does not move the roots.
    expectRoots(CubicRoots::solve(2, -12, 22, -12), {1, 2, 3});
}

TEST(cubicRootsOnBothSidesOfZeroIncludingZero) {
    // t(t − 1)(t + 2) = t³ + t² − 2t. Roots −2, 0, 1.
    expectRoots(CubicRoots::solve(1, 1, -2, 0), {-2, 0, 1}, 2e-7f, 1e-7f);
}

TEST(cubicATightClusterFarFromTheOrigin) {
    // (t − 100)(t − 101)(t − 102) = t³ − 303t² + 30602t − 1030200 (all exact in float).
    expectRoots(CubicRoots::solve(1, -303, 30602, -1030200), {100, 101, 102});
}

TEST(cubicATripleRoot) {
    // (t − 2)³ = t³ − 6t² + 12t − 8.
    EXPECT(CubicRoots::solve(1, -6, 12, -8) == std::vector<float>({2}));
    // The cubic term alone: 2t³ = 0 has the triple root 0.
    EXPECT(CubicRoots::solve(2, 0, 0, 0) == std::vector<float>({0}));
}

TEST(cubicADoubleRootAndASimpleRoot) {
    // (t − 1)²(t − 2) = t³ − 4t² + 5t − 2.
    expectRoots(CubicRoots::solve(1, -4, 5, -2), {1, 2});
    // (t + 1)(t − 3)² = t³ − 5t² + 3t + 9: roots −1 and 3.
    expectRoots(CubicRoots::solve(1, -5, 3, 9), {-1, 3});
    // Double root below the simple one: (t − 1)²(t + 2) = t³ − 3t + 2.
    expectRoots(CubicRoots::solve(1, 0, -3, 2), {-2, 1});
}

TEST(cubicADoubleRootStaysFoundAtPowerOfTwoScales) {
    // (t − 0.5)²(t − 2) = t³ − 3t² + 2.25t − 0.5. All coefficients are exact binary fractions, so
    // the discriminant is exactly 0 in theory and rounding must not turn it into "no double root".
    expectRoots(CubicRoots::solve(1, -3, 2.25f, -0.5f), {0.5f, 2});
    // t = 512·s: (s − 0.5)²(s − 2) with s = t/512 gives coefficients scaled by 512^k.
    expectRoots(CubicRoots::solve(1, -3 * 512, 2.25f * 512 * 512, -0.5f * 512 * 512 * 512), {256, 1024});
}

TEST(cubicOneRealRoot) {
    // t³ + t + 1: the real root is −0.6823278038.
    expectRoots(CubicRoots::solve(1, 0, 1, 1), {-0.6823278f});
    // t³ − 2: the cube root of 2, 1.2599210.
    expectRoots(CubicRoots::solve(1, 0, 0, -2), {1.259921f});
    // (t − 1)(t² + 1) = t³ − t² + t − 1: the pair ±i is complex, the root is 1.
    expectRoots(CubicRoots::solve(1, -1, 1, -1), {1});
}

TEST(cubicALargeLinearTermDoesNotLoseTheSmallRoot) {
    // t³ + 1e6·t + 1: the root is near −1e-6. Cardano's formula loses about 9 digits; Newton's
    // iteration does not cancel. Measured exact to the float nearest −1e-6; the tolerance is 2
    // float steps (1e-13 each).
    std::vector<float> roots = CubicRoots::solve(1, 0, 1e6f, 1);
    EXPECT(roots.size() == 1);
    if (roots.size() == 1) EXPECT(isApproximatelyEqual(roots[0], -1e-6f, 2e-13f));
}

TEST(cubicAZeroLeadingCoefficientFallsBackToTheQuadratic) {
    // t² − 3t + 2 = (t − 1)(t − 2)
    expectRoots(CubicRoots::solve(0, 1, -3, 2), {1, 2});
    // A double root of the quadratic is reported once: t² − 2t + 1.
    expectRoots(CubicRoots::solve(0, 1, -2, 1), {1});
    // No real roots: t² + 1.
    EXPECT(CubicRoots::solve(0, 1, 0, 1).empty());
    // Linear: 2t − 6 = 0.
    expectRoots(CubicRoots::solve(0, 0, 2, -6), {3});
    // Constant: 5 = 0 has no solution.
    EXPECT(CubicRoots::solve(0, 0, 0, 5).empty());
}

TEST(cubicAllZeroCoefficientsGiveNoRoots) { EXPECT(CubicRoots::solve(0, 0, 0, 0).empty()); }

TEST(cubicATinyLeadingCoefficientIsStillACubic) {
    // 1e-15·t³ + t² − 3t + 2 is a cubic, not a quadratic: besides the roots near 1 and 2 it has a
    // far root, about −1e15. Measured 9.4e-9 relative; the tolerance is 3e-8.
    std::vector<float> roots = CubicRoots::solve(1e-15f, 1, -3, 2);
    expectRoots(roots, {-1 / 1e-15f, 1, 2}, 3e-8f);
}

TEST(cubicASmallLeadingCoefficientInventsNoRoots) {
    // 1e-7·t³ + t² − 3t + 2. Within 0…3 exactly two roots come back (an earlier version reported
    // a third, invented root here). Measured 1.9e-8 relative; the tolerance is 6e-8.
    expectRoots(CubicRoots::solve(1e-7f, 1, -3, 2, 0, 3), {1.0000001f, 1.9999992f}, 6e-8f);
    // 1e-6·t³ + t² − t + 0.21: the quadratic's roots are 0.3 and 0.7; the cubic term moves them to
    // 0.30000005 and 0.69999916. Measured 2.8e-8 relative; the tolerance is 8e-8.
    expectRoots(CubicRoots::solve(1e-6f, 1, -1, 0.21f, 0, 1), {0.30000005f, 0.69999916f}, 8e-8f);
}

TEST(cubicASmallButNotNegligibleLeadingCoefficientKeepsAllRoots) {
    // 1e-6·(t + 1e6)(t − 1)(t − 2), expanded in float: roots −1e6, 1 and 2.
    float a = 1e-6f;
    float roots[3] = {-1e6f, 1, 2};
    float quadratic = -a * (roots[0] + roots[1] + roots[2]);
    float linear = a * (roots[0] * roots[1] + roots[0] * roots[2] + roots[1] * roots[2]);
    float constant = -a * roots[0] * roots[1] * roots[2];
    // Measured 6.0e-8 relative; the tolerance is 1.8e-7.
    expectRoots(CubicRoots::solve(a, quadratic, linear, constant), {-1e6f, 1, 2}, 1.8e-7f);
}

TEST(cubicARootBeyondFloatIsDropped) {
    // 1e-40·t³ + t² − 3t + 2: the far root, about −1e40, is larger than any float, so only 1 and 2
    // come back. 1e-40 is subnormal; built from bits.
    float tiny = std::bit_cast<float>(std::uint32_t(71362)); // 1.00001e-40
    EXPECT(CubicRoots::solve(tiny, 1, -3, 2) == std::vector<float>({1, 2}));
}

TEST(cubicTheCubicTermIsNeverDroppedForBeingSmall) {
    // t³ − 1e13: the root is ∛1e13 = 21544.347. Measured 3.0e-8 relative; the tolerance is 1e-7.
    expectRoots(CubicRoots::solve(1, 0, 0, -1e13f), {21544.347f}, 1e-7f);
    // t³ − 8e15: the cube root is 2e5 exactly.
    EXPECT(CubicRoots::solve(1, 0, 0, -8e15f) == std::vector<float>({200000}));
}

TEST(cubicNonFiniteCoefficientsGiveNoRoots) {
    EXPECT(CubicRoots::solve(NAN, 1, 1, 1).empty());
    EXPECT(CubicRoots::solve(1, NAN, 1, 1).empty());
    EXPECT(CubicRoots::solve(1, 1, infinity, 1).empty());
    EXPECT(CubicRoots::solve(1, 1, 1, -infinity).empty());
    EXPECT(CubicRoots::solve(0, 1, 1, NAN).empty());
}

TEST(cubicScalingAllCoefficientsDoesNotMoveTheRoots) {
    // (t − 1)(t − 2)(t − 3) times a constant has the same roots, from 1e-30 to 1e30. The
    // coefficients 11·scale and 6·scale are rounded to float, which moves a root by up to 7.7e-7
    // (measured); the tolerance is 2.5e-6, about 3 times that.
    for (float scale : {1e-30f, 1e-12f, 1e12f, 1e30f}) {
        expectRoots(CubicRoots::solve(scale, -6 * scale, 11 * scale, -6 * scale), {1, 2, 3}, 2.5e-6f);
    }
}

TEST(cubicScalingTheVariableScalesTheRoots) {
    // Roots s, 2s, 3s: t³ − 6s·t² + 11s²·t − 6s³. The coefficients 11·s² and 6·s³ are rounded to
    // float, which moves a root by up to 6.5e-7 relative (measured, at s = 0.002); the tolerance
    // is 2e-6, about 3 times that.
    for (float scale : {1e-10f, 1e-3f, 1e-2f, 1e2f, 1e3f, 1e10f}) {
        std::vector<float> roots = CubicRoots::solve(1, -6 * scale, 11 * scale * scale, -6 * scale * scale * scale);
        expectRoots(roots, {scale, 2 * scale, 3 * scale}, 2e-6f);
    }
}

TEST(cubicRootsWithinAnInterval) {
    // t³ − t has roots −1, 0, 1: the interval 0…1 keeps the last two, which sit on its ends.
    EXPECT(CubicRoots::solve(1, 0, -1, 0, 0, 1) == std::vector<float>({0, 1}));
    // (t − 0.25)(t − 0.5)(t − 2) = t³ − 2.75t² + 1.625t − 0.25 → 0.25 and 0.5 in 0…1.
    expectRoots(CubicRoots::solve(1, -2.75f, 1.625f, -0.25f, 0, 1), {0.25f, 0.5f});
    // Nothing inside.
    EXPECT(CubicRoots::solve(1, 0, 0, -8, 0, 1).empty());
}

TEST(cubicAnInfiniteEndAcceptsEverythingOnItsSide) {
    // t³ + 1 has the single root −1. 0…∞ holds nothing; the infinite end adds no slack.
    EXPECT(CubicRoots::solve(1, 0, 0, 1, 0, infinity).empty());
    EXPECT(CubicRoots::solve(1, 0, 0, 1, -infinity, 0) == std::vector<float>({-1}));
    EXPECT(CubicRoots::solve(1, 0, 0, 1, -infinity, infinity) == std::vector<float>({-1}));
}

TEST(cubicIntervalsOtherThanZeroToOne) {
    // (t − 1)(t − 2)(t − 3) in 1.5…3: the 2 and the 3.
    expectRoots(CubicRoots::solve(1, -6, 11, -6, 1.5f, 3), {2, 3});
}

TEST(cubicEveryDoubleRootOfWholeNumbersIsExact) {
    // (t − r)²(t − s) for every r ≠ s in −60…60: 14,520 cubics with exact coefficients. A double
    // root is where rounding hurts most, and every one must come back exactly as [r, s] sorted.
    int failureCount = 0;
    std::string firstFailure;
    for (int twice = -60; twice <= 60; twice += 1) {
        for (int simple = -60; simple <= 60; simple += 1) {
            if (simple == twice) continue;
            std::vector<float> found = solveProduct(float(twice), float(twice), float(simple));
            std::vector<float> expected = {float(twice), float(simple)};
            std::sort(expected.begin(), expected.end());
            if (found != expected) {
                if (failureCount == 0) firstFailure = "(" + std::to_string(twice) + ", " + std::to_string(simple) + ") -> " + describe(found);
                failureCount += 1;
            }
        }
    }
    if (failureCount > 0) testing::fail(__FILE__, __LINE__, std::to_string(failureCount) + " failures, first " + firstFailure);
}

TEST(cubicEveryTripleRootOfWholeNumbersIsExact) {
    // (t − r)³ for r in −60…60.
    for (int triple = -60; triple <= 60; triple += 1) {
        EXPECT(solveProduct(float(triple), float(triple), float(triple)) == std::vector<float>({float(triple)}));
    }
}

TEST(cubicEveryThreeDistinctWholeRootsAreExact) {
    // (t − r)(t − s)(t − u) for every r < s < u in −20…22: 12,341 cubics.
    int failureCount = 0;
    for (int first = -20; first <= 20; first += 1) {
        for (int second = first + 1; second <= 21; second += 1) {
            for (int third = second + 1; third <= 22; third += 1) {
                std::vector<float> expected = {float(first), float(second), float(third)};
                if (solveProduct(expected[0], expected[1], expected[2]) != expected) failureCount += 1;
            }
        }
    }
    EXPECT_EQUAL(failureCount, 0);
}

TEST(cubicRootsOfVeryDifferentSizes) {
    // (t − 1e-6)(t − 1)(t − 1e3), expanded in double and rounded to float. The smallest root is a
    // millionth of the middle one and a billionth of the largest. Measured 1.1e-7 relative; the
    // limit is 3.5e-7.
    std::vector<double> spread = {1e-6, 1, 1e3};
    EXPECT(worstRelativeError(solveExpanded(spread), spread) < 3.5e-7);
    // Two tiny roots next to a large one: (t − 1e-6)(t − 2e-6)(t − 1e3). Measured 1.2e-7.
    std::vector<double> tinyPair = {1e-6, 2e-6, 1e3};
    EXPECT(worstRelativeError(solveExpanded(tinyPair), tinyPair) < 3.5e-7);
}

TEST(cubicIsNearZeroAtEveryRoot) {
    // Random coefficients. At each reported root, the value of the cubic is tiny next to the sum
    // of the sizes of its terms (the backward-error measure). Measured 7.4e-8 (about one float
    // step); the bound is 2.2e-7.
    Sequence random;
    int rootsChecked = 0;
    double worstRatio = 0;
    for (int round = 0; round < 2000; round += 1) {
        float cubic = random.nextLeadingCoefficient();
        float quadratic = 5 * random.next();
        float linear = 5 * random.next();
        float constant = 5 * random.next();
        for (float root : CubicRoots::solve(cubic, quadratic, linear, constant)) {
            double t = root;
            double value = ((double(cubic) * t + double(quadratic)) * t + double(linear)) * t + double(constant);
            double termSizes = std::fabs(double(cubic) * t * t * t) + std::fabs(double(quadratic) * t * t)
                + std::fabs(double(linear) * t) + std::fabs(double(constant));
            worstRatio = std::max(worstRatio, std::fabs(value) / termSizes);
            rootsChecked += 1;
        }
    }
    EXPECT(rootsChecked > 2000);
    EXPECT(worstRatio < 2.2e-7);
}

TEST(cubicRootsBuiltFromAProductAreRecovered) {
    // Pick three roots at least 0.2 apart in −5…5, expand (t − r₁)(t − r₂)(t − r₃), and solve: the
    // same roots must come back. Measured worst error 1.1e-5 relative; the tolerance is 3.5e-5.
    Sequence random;
    int cases = 0;
    for (int round = 0; round < 500; round += 1) {
        float picked[3];
        for (float& root : picked) root = 5 * random.next();
        std::sort(picked, picked + 3);
        if (!(picked[1] - picked[0] > 0.2f && picked[2] - picked[1] > 0.2f)) continue;
        float sum = picked[0] + picked[1] + picked[2];
        float pairs = picked[0] * picked[1] + picked[0] * picked[2] + picked[1] * picked[2];
        float product = picked[0] * picked[1] * picked[2];
        std::vector<float> found = CubicRoots::solve(1, -sum, pairs, -product);
        expectRoots(found, {picked[0], picked[1], picked[2]}, 3.5e-5f, 3.5e-5f);
        cases += 1;
    }
    EXPECT(cases > 50);
}

TEST(cubicAFarRootFromATinyLeadingCoefficientIsRecovered) {
    // a·(t + 1/a)(t − r₁)(t − r₂) with |a| from 1e-12 to 1e-3 and r₁, r₂ at least 0.2 apart in
    // −5…5: the far root −1/a and both near roots come back. Measured 1.4e-6; the limit is 4e-6.
    Sequence random;
    int cases = 0;
    double worst = 0;
    for (int round = 0; round < 2000; round += 1) {
        double cubic = random.nextLeadingCoefficient();
        if (!(std::fabs(cubic) < 1e-3)) continue;
        double nearRoots[2];
        for (double& root : nearRoots) root = 5 * double(random.next());
        std::sort(nearRoots, nearRoots + 2);
        if (!(nearRoots[1] - nearRoots[0] > 0.2)) continue;
        std::vector<double> expected = {-1 / cubic, nearRoots[0], nearRoots[1]};
        std::sort(expected.begin(), expected.end());
        std::vector<float> found = solveExpanded(expected, cubic);
        if (found.size() != 3) {
            testing::fail(__FILE__, __LINE__, "a far root test returned " + describe(found));
            continue;
        }
        for (size_t index = 0; index < 3; index += 1) {
            worst = std::max(worst, std::fabs(double(found[index]) - expected[index]) / std::max(std::fabs(expected[index]), 1.0));
        }
        cases += 1;
    }
    EXPECT(cases > 500);
    EXPECT(worst < 4e-6);
}

TEST(cubicCountsMatchSignChanges) {
    // A cubic with a sign change between two sampled points must have a root between them
    // (intermediate value theorem), so the number of roots is at least the number of sign changes
    // on a fine grid, and at most 3.
    Sequence random;
    for (int round = 0; round < 500; round += 1) {
        float cubic = random.nextLeadingCoefficient();
        float quadratic = 3 * random.next();
        float linear = 3 * random.next();
        float constant = 3 * random.next();
        std::vector<float> roots = CubicRoots::solve(cubic, quadratic, linear, constant);
        size_t signChanges = 0;
        std::optional<double> previous;
        for (int step = -4000; step <= 4000; step += 1) {
            double t = double(step) / 400;
            double value = ((double(cubic) * t + double(quadratic)) * t + double(linear)) * t + double(constant);
            if (previous && (*previous < 0) != (value < 0)) signChanges += 1;
            previous = value;
        }
        EXPECT(roots.size() >= signChanges && roots.size() <= 3);
    }
}

TEST(cubicRootsComeBackSortedAndDistinct) {
    Sequence random;
    for (int round = 0; round < 500; round += 1) {
        float cubic = random.nextLeadingCoefficient();
        float quadratic = 3 * random.next();
        float linear = 3 * random.next();
        float constant = 3 * random.next();
        std::vector<float> roots = CubicRoots::solve(cubic, quadratic, linear, constant);
        EXPECT(std::is_sorted(roots.begin(), roots.end()));
        EXPECT(std::set<float>(roots.begin(), roots.end()).size() == roots.size());
    }
}

// MARK: Integration

TEST(integrationWeightsAddUpToTheIntervalWidth) {
    // On −1…1 the integral of the constant 1 is the width, 2.
    float sum = 0;
    for (float weight : GaussLegendre::weights) sum += weight;
    EXPECT(isApproximatelyEqual(sum, 2));
}

TEST(integrationIntegratesAConstantAndALine) {
    // ∫₀⁴ 3 dx = 12.
    EXPECT(isApproximatelyEqual(GaussLegendre::integral([](float) { return 3.0f; }, 0, 4), 12));
    // ∫₁³ x dx = (9 − 1)/2 = 4.
    EXPECT(isApproximatelyEqual(GaussLegendre::integral([](float x) { return x; }, 1, 3), 4));
}

TEST(integrationIsExactForEveryPowerUpToDegreeNine) {
    // ∫₀² xᵏ dx = 2^(k+1)/(k+1). Compared relatively, since the values reach 102.4.
    for (int degree = 0; degree <= 9; degree += 1) {
        float exponent = float(degree);
        float expected = std::pow(2.0f, exponent + 1) / (exponent + 1);
        float computed = GaussLegendre::integral([exponent](float x) { return std::pow(x, exponent); }, 0, 2);
        EXPECT(std::fabs(computed - expected) <= 1e-5f * expected);
    }
}

TEST(integrationIsNotExactForDegreeTen) {
    // ∫₋₁¹ x¹⁰ dx = 2/11 = 0.181818. The rule's error term for this function is about 2.9e-3, so a
    // 5-point rule visibly misses it.
    float computed = GaussLegendre::integral([](float x) { return std::pow(x, 10.0f); }, -1, 1);
    EXPECT(std::fabs(computed - 2.0f / 11.0f) > 1e-3f);
}

TEST(integrationReversedLimitsGiveANegativeResult) {
    // ∫₃¹ x dx = −4.
    EXPECT(isApproximatelyEqual(GaussLegendre::integral([](float x) { return x; }, 3, 1), -4));
}

TEST(integrationMorePiecesImproveATranscendentalIntegral) {
    // ∫₀^π sin x dx = 2 exactly. One piece is already close, ten are within float noise.
    auto sine = [](float x) { return std::sin(x); };
    float onePiece = GaussLegendre::integral(sine, 0, swiftPi);
    float tenPieces = GaussLegendre::integral(sine, 0, swiftPi, 10);
    EXPECT(std::fabs(onePiece - 2) < 1e-3f);
    EXPECT(isApproximatelyEqual(tenPieces, 2, 2e-6f));
}

TEST(integrationNeedsAtLeastOnePiece) {
    EXPECT_PROBLEM(GaussLegendre::integral([](float x) { return x; }, 0, 1, 0), "An integral needs at least one piece.");
}

// MARK: Seeded random

namespace {
float degreesOf(Angle angle) { return static_cast<float>(double(angle.radians) * 180.0 / 3.14159265358979323846); }
} // namespace

TEST(randomSeedZeroGivesTheReferenceOutputs) {
    // Computed from the reference algorithm (Steele, Lea & Flood 2014; Vigna's splitmix64.c).
    SeededRandom random(0);
    EXPECT(random.nextUInt64() == 0xE220A8397B1DCDAFull);
    EXPECT(random.nextUInt64() == 0x6E789E6AA1B965F4ull);
    EXPECT(random.nextUInt64() == 0x06C45D188009454Full);
}

TEST(randomSeed1234567GivesTheReferenceOutputs) {
    // The first five outputs for seed 1234567, computed from the reference algorithm.
    SeededRandom random(1234567);
    const std::uint64_t expected[] = {6457827717110365317ull, 3203168211198807973ull, 9817491932198370423ull,
                                      4593380528125082431ull, 16408922859458223821ull};
    for (std::uint64_t value : expected) EXPECT(random.nextUInt64() == value);
}

TEST(randomUnitFloatIsTheTop24BitsOverTwoToThe24) {
    // First output for seed 0 is 0xE220A8397B1DCDAF; its top 24 bits are 0xE220A8 = 14 819 496,
    // and 14 819 496 / 16 777 216 = 0.88331079...
    SeededRandom random(0);
    EXPECT(random.nextUnitFloat() == float(14819496) / 16777216);
    // The second output starts 0x6E789E → 0.43152797...
    EXPECT(isApproximatelyEqual(random.nextUnitFloat(), 0.43152797f));
}

TEST(randomFloatInRangeBlendsTheUnitFloat) {
    // 10 + (20 − 10) × 0.88331079 = 18.8331079
    SeededRandom random(0);
    EXPECT(isApproximatelyEqual(random.nextFloat(10, 20), 18.833108f));
}

TEST(randomIntegerInRangeIsTheHighHalfOfTheProduct) {
    // x = 0xE220A8397B1DCDAF is 0.88331... of 2⁶⁴, so x × 10 / 2⁶⁴ rounds down to 8 and
    // x × 7 / 2⁶⁴ to 6 (6.18...); the range's lower bound is added.
    SeededRandom random(0);
    EXPECT_EQUAL(random.nextInt(0, 10), 8);
    random = SeededRandom(0);
    EXPECT_EQUAL(random.nextInt(100, 107), 106);
}

TEST(randomAngleInRangeBlendsTheUnitFloat) {
    SeededRandom random(0);
    Angle angle = random.nextAngle(Angle::degrees(90), Angle::degrees(180));
    // 90° + 90° × 0.88331079 = 169.4979...°
    EXPECT(isApproximatelyEqual(degreesOf(angle), 169.49797f, 1e-3f));
}

TEST(randomAngleBoundsMayComeInEitherOrder) {
    // 180° + (90° − 180°) × 0.88331079 = 180° − 79.49797° = 100.50203°
    SeededRandom random(0);
    Angle angle = random.nextAngle(Angle::degrees(180), Angle::degrees(90));
    EXPECT(isApproximatelyEqual(degreesOf(angle), 100.50203f, 1e-3f));
    for (int round = 0; round < 1000; round += 1) {
        float degrees = degreesOf(random.nextAngle(Angle::degrees(180), Angle::degrees(90)));
        EXPECT(degrees > 90 - 1e-3f && degrees <= 180 + 1e-3f);
    }
}

TEST(randomIntegerRangeMayBeWiderThanIntMax) {
    // Int.min ..< Int.max holds 2⁶⁴ − 1 values. x = 0xE220A8397B1DCDAF; x × (2⁶⁴ − 1) =
    // (x − 1)·2⁶⁴ + (2⁶⁴ − x), so the high half is x − 1 and the low half 2⁶⁴ − x is above the
    // rejection limit 2⁶⁴ mod (2⁶⁴ − 1) = 1. Int.min + (x − 1), wrapping, is
    // 0xE220A8397B1DCDAE − 2⁶³ = 0x6220A8397B1DCDAE.
    const std::int64_t smallest = std::numeric_limits<std::int64_t>::min();
    const std::int64_t largest = std::numeric_limits<std::int64_t>::max();
    SeededRandom random(0);
    EXPECT_EQUAL(random.nextInt(smallest, largest), 0x6220A8397B1DCDAELL);
    for (int round = 0; round < 100; round += 1) EXPECT(random.nextInt(smallest, largest) < largest);
}

TEST(randomIntegerRangeMustNotBeEmpty) {
    SeededRandom random(0);
    EXPECT_PROBLEM(random.nextInt(5, 5), "A random integer needs a non-empty range");
}

TEST(randomSameSeedGivesTheSameSequenceAndCopiesAreIndependent) {
    SeededRandom first(42);
    SeededRandom second(42);
    for (int round = 0; round < 100; round += 1) EXPECT(first.nextUnitFloat() == second.nextUnitFloat());

    SeededRandom original(7);
    original.nextUInt64();
    SeededRandom copy = original; // a value: the copy continues from the same place
    EXPECT(original.nextUInt64() == copy.nextUInt64());
    original.nextUInt64();
    EXPECT(!(original == copy));
}

TEST(randomDifferentSeedsGiveDifferentSequences) {
    SeededRandom first(1);
    SeededRandom second(2);
    bool anyDifference = false;
    for (int round = 0; round < 10; round += 1) anyDifference = anyDifference || first.nextUInt64() != second.nextUInt64();
    EXPECT(anyDifference);
}

TEST(randomUnitFloatsStayInTheHalfOpenIntervalWithMeanOneHalf) {
    SeededRandom random(99);
    float sum = 0;
    const int count = 100000;
    bool allInRange = true;
    for (int round = 0; round < count; round += 1) {
        float value = random.nextUnitFloat();
        allInRange = allInRange && value >= 0 && value < 1;
        sum += value;
    }
    EXPECT(allInRange);
    // Standard error of the mean of 100 000 uniform numbers is 9.1e-4; 5e-3 is ~5 standard errors.
    EXPECT(isApproximatelyEqual(sum / float(count), 0.5f, 5e-3f));
}

TEST(randomFloatsInRangeStayInsideIt) {
    SeededRandom random(5);
    float lowest = infinity;
    float highest = -infinity;
    for (int round = 0; round < 10000; round += 1) {
        float value = random.nextFloat(-3, 2);
        EXPECT(value >= -3 && value <= 2);
        lowest = std::min(lowest, value);
        highest = std::max(highest, value);
    }
    EXPECT(lowest < -2.9f && highest > 1.9f);
}

TEST(randomIntegersCoverTheRangeEvenlyWithoutLeavingIt) {
    SeededRandom random(3);
    int counts[6] = {0, 0, 0, 0, 0, 0};
    for (int round = 0; round < 60000; round += 1) {
        std::int64_t value = random.nextInt(-2, 4);
        counts[value + 2] += 1;
    }
    // Each of 6 values is expected 10 000 times, standard deviation 91. 600 is more than 6
    // standard deviations.
    for (int count : counts) EXPECT(std::abs(count - 10000) < 600);
}

TEST(randomOneValueRangeAlwaysGivesThatValue) {
    SeededRandom random(0);
    for (int round = 0; round < 20; round += 1) EXPECT_EQUAL(random.nextInt(5, 6), 5);
}

TEST(randomUnitVectorsHaveLengthOneAndAreSpreadEvenly) {
    SeededRandom random(2024);
    const int count = 50000;
    int aboveHalf = 0;
    float meanSquaredHeight = 0;
    float meanSquaredX = 0;
    Vector3 sum(0, 0, 0);
    bool allUnit = true;
    for (int round = 0; round < count; round += 1) {
        Vector3 vector = random.nextUnitVector3();
        allUnit = allUnit && isApproximatelyEqual(vector.length(), 1, 1e-5f);
        if (vector.z > 0.5f) aboveHalf += 1;
        meanSquaredHeight += vector.z * vector.z;
        meanSquaredX += vector.x * vector.x;
        sum += vector;
    }
    EXPECT(allUnit);
    // Uniform on the sphere: z is uniform on −1…1, so P(z > 0.5) = 1/4 and E[z²] = 1/3, and the
    // mean vector is the origin.
    EXPECT(isApproximatelyEqual(float(aboveHalf) / float(count), 0.25f, 0.012f));
    EXPECT(isApproximatelyEqual(meanSquaredHeight / float(count), 1.0f / 3, 0.012f));
    EXPECT((sum / float(count)).length() < 0.015f);
    // x and y are as spread as z: E[x²] = E[y²] = 1/3.
    EXPECT(isApproximatelyEqual(meanSquaredX / float(count), 1.0f / 3, 0.012f));
}
