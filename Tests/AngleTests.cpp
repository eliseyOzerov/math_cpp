// Ports of EditorTests/Math/AngleTests.swift, ScalarTests.swift and the Vector3 parts of
// VectorTests.swift. (Angle.signedTurn and Vector4.perspectiveDivided are not ported.)
#include <limits>

#include "../Linear/Angle.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
constexpr float pi = 3.14159265f;
} // namespace

// MARK: Angle

TEST(halfTurnInDegreesIsPiRadians) {
    // 180° × π/180 = π
    EXPECT(isApproximatelyEqual(Angle::degrees(180).radians, pi));
}

TEST(quarterTurnInRadiansIsNinetyDegrees) {
    // (π/2) × 180/π = 90
    EXPECT(isApproximatelyEqual(Angle::fromRadians(pi / 2).degrees(), 90));
}

TEST(degreesSurviveTheRoundTrip) { EXPECT(isApproximatelyEqual(Angle::degrees(37.5f).degrees(), 37.5f)); }

TEST(sineAndCosineOfAQuarterTurn) {
    // Turning 90° from the x axis on the unit circle lands on (0, 1).
    EXPECT(isApproximatelyEqual(Angle::degrees(90).cosine(), 0));
    EXPECT(isApproximatelyEqual(Angle::degrees(90).sine(), 1));
}

TEST(cosineOfSixtyDegreesIsOneHalf) {
    // The textbook 30-60-90 triangle: cos 60° = 1/2, sin 60° = √3/2 ≈ 0.866025.
    EXPECT(isApproximatelyEqual(Angle::degrees(60).cosine(), 0.5f));
    EXPECT(isApproximatelyEqual(Angle::degrees(60).sine(), 0.866025f));
}

TEST(multiplyingScalesTheAngle) {
    // 45° × 2 = 90°
    EXPECT(isApproximatelyEqual((Angle::degrees(45) * 2).degrees(), 90));
}

TEST(zeroIsNoTurn) { EXPECT(Angle::zero().radians == 0); }

TEST(tangentOfFortyFiveDegreesIsOne) {
    // tan 45° = sin 45° / cos 45° = (√2/2) / (√2/2) = 1
    EXPECT(isApproximatelyEqual(Angle::degrees(45).tangent(), 1));
}

TEST(dividingSplitsTheAngle) { EXPECT(isApproximatelyEqual((Angle::degrees(90) / 2).degrees(), 45)); }

TEST(arcCosineOfKnownValues) {
    // cos 0° = 1, cos 60° = 1/2, cos 90° = 0, cos 180° = −1.
    EXPECT(Angle::arcCosine(1).radians == 0);
    // Swift measured 3.8e-6° at 60° (half a float step of 60), the others exact. The tolerance is 1.2e-5°.
    EXPECT(isApproximatelyEqual(Angle::arcCosine(0.5f).degrees(), 60, 1.2e-5f));
    EXPECT(isApproximatelyEqual(Angle::arcCosine(0).degrees(), 90, 1.2e-5f));
    EXPECT(isApproximatelyEqual(Angle::arcCosine(-1).degrees(), 180, 1.2e-5f));
    // √2/2 is the cosine of 45°.
    EXPECT(isApproximatelyEqual(Angle::arcCosine(0.70710678f).degrees(), 45, 1.2e-5f));
}

TEST(arcCosineClampsRoundingOvershoot) {
    // A dot product of two unit vectors can come out as 1.0000001 or −1.0000001.
    EXPECT(Angle::arcCosine(1.0000001f).radians == 0);
    // Swift measured exact; the tolerance is one float step at 180 (1.5e-5).
    EXPECT(isApproximatelyEqual(Angle::arcCosine(-1.0000001f).degrees(), 180, 1.5e-5f));
    EXPECT(Angle::arcCosine(5).radians == 0);
    EXPECT(std::isnan(Angle::arcCosine(notANumber).radians));
}

TEST(arcCosineUndoesCosine) {
    // Swift measured worst 1.5e-5° (at 175° and 5°, where the cosine is flattest); the tolerance is 5e-5°.
    for (float degrees : {5.0f, 30.0f, 77.0f, 120.0f, 175.0f}) {
        EXPECT(isApproximatelyEqual(Angle::arcCosine(Angle::degrees(degrees).cosine()).degrees(), degrees, 5e-5f));
    }
}

TEST(arcCosineCannotSeeAnglesBelowAboutThreeAndAHalfTenThousandthsOfARadian) {
    // cos θ ≈ 1 − θ²/2, and the float just below 1 is 1 − 2⁻²⁴, so the smallest angle whose cosine
    // is not rounded to 1 is about √(2·2⁻²⁴) = 3.45e-4 rad. A 1e-4 rad angle comes back as 0;
    // `Angle::between`, which uses atan2, keeps it.
    Angle small = Angle::fromRadians(1e-4f);
    EXPECT(Angle::arcCosine(small.cosine()).radians == 0);
    Angle between = Angle::between(Vector3::xAxis(), Vector3(small.cosine(), small.sine(), 0));
    // The tolerance is a few float steps, 1e-6 relative.
    EXPECT(isApproximatelyEqual(between.radians, 1e-4f, 1e-10f));
}

TEST(subtractingAnglesGivesTheTurnBetweenThem) {
    // 90° − 30° = 60°; 30° − 90° = −60° (not wrapped to 300°).
    EXPECT(isApproximatelyEqual((Angle::degrees(90) - Angle::degrees(30)).degrees(), 60, 1e-4f));
    EXPECT(isApproximatelyEqual((Angle::degrees(30) - Angle::degrees(90)).degrees(), -60, 1e-4f));
    EXPECT(Angle::degrees(45) - Angle::degrees(45) == Angle::zero());
}

TEST(addingAnglesAddsTheTurns) {
    // 30° + 60° = 90°; 300° + 90° = 390° (not wrapped to 30°); in radians 0.5 + 0.25 = 0.75
    // exactly, since all three are dyadic.
    EXPECT(isApproximatelyEqual((Angle::degrees(30) + Angle::degrees(60)).degrees(), 90, 1e-4f));
    EXPECT(isApproximatelyEqual((Angle::degrees(300) + Angle::degrees(90)).degrees(), 390, 1e-4f));
    EXPECT(Angle::fromRadians(0.5f) + Angle::fromRadians(0.25f) == Angle::fromRadians(0.75f));
    // Adding then subtracting the same turn gives the start back.
    EXPECT(isApproximatelyEqual(((Angle::degrees(10) + Angle::degrees(25)) - Angle::degrees(25)).degrees(), 10, 1e-4f));
}

TEST(negatingAnAngleTurnsTheOtherWay) {
    EXPECT(isApproximatelyEqual((-Angle::degrees(30)).degrees(), -30, 1e-4f));
    EXPECT(isApproximatelyEqual((-Angle::degrees(30)).sine(), -0.5f));
    EXPECT(-(-Angle::degrees(30)) == Angle::degrees(30));
}

// MARK: Scalar

TEST(nearlyZeroAcceptsRoundingNoise) {
    EXPECT(isNearlyZero(3e-8f));
    EXPECT(isNearlyZero(-3e-8f));
    EXPECT(!isNearlyZero(0.001f));
}

TEST(clampingKeepsValuesInsideAndPinsValuesOutside) {
    EXPECT(clamped(0.5f, 0, 1) == 0.5f);
    EXPECT(clamped(-2, 0, 1) == 0);
    EXPECT(clamped(7, 0, 1) == 1);
}

TEST(linearBlendOfNumbers) {
    // 10 + (20 − 10) × 0.25 = 12.5
    EXPECT(linearBlend(10, 20, 0.25f) == 12.5f);
    EXPECT(linearBlend(10, 20, 0) == 10);
    EXPECT(linearBlend(10, 20, 1) == 20);
    // Past the end: 10 + 10 × 1.5 = 25
    EXPECT(linearBlend(10, 20, 1.5f) == 25);
}

TEST(linearBlendOfPoints) {
    // Halfway from (0, 0, 0) to (2, 4, −6) is (1, 2, −3).
    Vector3 middle = Vector3::linearBlend(Vector3(0, 0, 0), Vector3(2, 4, -6), 0.5f);
    EXPECT(middle == Vector3(1, 2, -3));
}

TEST(distanceIsTheLengthOfTheDifference) {
    // From (1, 1, 1) to (4, 5, 1): (3, 4, 0), length 5.
    EXPECT(isApproximatelyEqual(Vector3(1, 1, 1).distance(Vector3(4, 5, 1)), 5));
}

// MARK: Vector3

TEST(dotProductSumsTheComponentProducts) {
    // 1·4 + 2·5 + 3·6 = 4 + 10 + 18 = 32
    EXPECT(Vector3(1, 2, 3).dot(Vector3(4, 5, 6)) == 32);
}

TEST(dotProductOfPerpendicularAxesIsZero) {
    EXPECT(Vector3::xAxis().dot(Vector3::yAxis()) == 0);
    EXPECT(Vector3::yAxis().dot(Vector3::zAxis()) == 0);
}

TEST(dotProductOfOppositeUnitVectorsIsMinusOne) { EXPECT(Vector3::xAxis().dot(-Vector3::xAxis()) == -1); }

TEST(lengthIsPythagoras) {
    // √(3² + 4² + 0²) = √25 = 5
    EXPECT(isApproximatelyEqual(Vector3(3, 4, 0).length(), 5));
    // √(1² + 2² + 2²) = √9 = 3
    EXPECT(isApproximatelyEqual(Vector3(1, 2, 2).length(), 3));
    // The squared length is the same without the root: 1 + 4 + 4 = 9.
    EXPECT(Vector3(1, 2, 2).lengthSquared() == 9);
}

TEST(normalizedKeepsDirectionAndHasLengthOne) {
    // (0, 3, 4) has length 5, so it becomes (0, 3/5, 4/5).
    Vector3 unit = Vector3(0, 3, 4).normalized();
    EXPECT(isApproximatelyEqual(unit, Vector3(0, 0.6f, 0.8f)));
    EXPECT(isApproximatelyEqual(unit.length(), 1));
}

TEST(normalizingZeroGivesZero) { EXPECT(Vector3(0, 0, 0).normalized() == Vector3(0, 0, 0)); }

TEST(crossProductOfAxesFollowsTheRightHandRule) {
    EXPECT(Vector3::xAxis().cross(Vector3::yAxis()) == Vector3::zAxis());
    EXPECT(Vector3::yAxis().cross(Vector3::zAxis()) == Vector3::xAxis());
    EXPECT(Vector3::zAxis().cross(Vector3::xAxis()) == Vector3::yAxis());
}

TEST(swappingTheCrossProductFlipsIt) { EXPECT(Vector3::yAxis().cross(Vector3::xAxis()) == -Vector3::zAxis()); }

TEST(crossProductOfGeneralVectors) {
    // (1, 2, 3) × (4, 5, 6)
    //   x = 2·6 − 3·5 = 12 − 15 = −3
    //   y = 3·4 − 1·6 = 12 −  6 =  6
    //   z = 1·5 − 2·4 =  5 −  8 = −3
    EXPECT(Vector3(1, 2, 3).cross(Vector3(4, 5, 6)) == Vector3(-3, 6, -3));
}

TEST(reflectionFlipsOnlyThePartAlongTheNormal) {
    // Mirror (3, 4, 5) in the plane z = 0 (normal (0, 0, 2), length need not be 1):
    // (3,4,5) − 2·(10/4)·(0,0,2) = (3, 4, −5).
    EXPECT(Vector3(3, 4, 5).reflectedAcrossPlane(Vector3(0, 0, 2)) == Vector3(3, 4, -5));
    // Mirror (1, 0, 0) in the plane with normal (1, 1, 0): (1,0,0) − 2·(1/2)·(1,1,0) = (0, −1, 0).
    EXPECT(isApproximatelyEqual(Vector3(1, 0, 0).reflectedAcrossPlane(Vector3(1, 1, 0)), Vector3(0, -1, 0)));
    // A zero normal defines no plane: unchanged.
    EXPECT(Vector3(1, 2, 3).reflectedAcrossPlane(Vector3(0, 0, 0)) == Vector3(1, 2, 3));
    EXPECT(!Vector3(0, 0, 0).definesReflectionPlane());
    EXPECT(Vector3(0, 0, 2).definesReflectionPlane());
}

TEST(reflectionKeepsLengthAndUndoesItself) {
    Vector3 normals[] = {Vector3(1, 2, 3), Vector3(-1, 0.5f, 0), Vector3(0, 0, -4)};
    Vector3 vectors[] = {Vector3(1, 0, 0), Vector3(2, -3, 1), Vector3(-0.5f, 0.25f, 4)};
    for (Vector3 normal : normals) {
        for (Vector3 vector : vectors) {
            Vector3 mirrored = vector.reflectedAcrossPlane(normal);
            EXPECT(isApproximatelyEqual(mirrored.length(), vector.length()));
            EXPECT(isApproximatelyEqual(mirrored.reflectedAcrossPlane(normal), vector));
        }
    }
}

TEST(unitDirectionSurvivesTinyAndHugeLengths) {
    // (3, 4, 0) has length 5 at any scale: the direction is (0.6, 0.8, 0).
    for (float factor : {1.0f, 1e-30f, 1e-38f, 1e30f, 1e37f}) {
        std::optional<Vector3> direction = (Vector3(3, 4, 0) * factor).unitDirection();
        EXPECT(direction.has_value() && isApproximatelyEqual(*direction, Vector3(0.6f, 0.8f, 0)));
    }
    std::optional<Vector3> down = Vector3(0, 0, -2).unitDirection();
    EXPECT(down.has_value() && *down == Vector3(0, 0, -1));
}

TEST(unitDirectionOfZeroOrNonFiniteIsNothing) {
    // NaN in y or z too: a max would quietly drop a NaN that is not its first argument.
    Vector3 bad[] = {Vector3(0, 0, 0),         Vector3(notANumber, 1, 1), Vector3(1, notANumber, 1),
                     Vector3(1, 1, notANumber), Vector3(infinity, 0, 0),   Vector3(0, 0, -infinity)};
    for (Vector3 vector : bad) EXPECT(!vector.unitDirection().has_value());
}

TEST(isFiniteChecksEveryComponent) {
    EXPECT(Vector3(1, -2, 3e38f).isFinite());
    EXPECT(!Vector3(notANumber, 0, 0).isFinite());
    EXPECT(!Vector3(0, infinity, 0).isFinite());
    EXPECT(!Vector3(0, 0, notANumber).isFinite());
}
