// Vector3 and Angle, with values worked out by hand.
#include "../Linear/Angle.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

TEST(vectorArithmeticWorksPerComponent) {
    Vector3 first(1, 2, 3);
    Vector3 second(4, -5, 6);
    EXPECT(isApproximatelyEqual(first + second, Vector3(5, -3, 9)));
    EXPECT(isApproximatelyEqual(first - second, Vector3(-3, 7, -3)));
    EXPECT(isApproximatelyEqual(first * 2, Vector3(2, 4, 6)));
}

TEST(dotAndCrossFollowTheRightHandRule) {
    // 1·4 + 2·(−5) + 3·6 = 12.
    EXPECT(isApproximatelyEqual(Vector3(1, 2, 3).dot(Vector3(4, -5, 6)), 12.0f));
    // x × y = z in a right-handed frame.
    EXPECT(isApproximatelyEqual(Vector3(1, 0, 0).cross(Vector3(0, 1, 0)), Vector3(0, 0, 1)));
}

TEST(vectorKnowsWhetherItIsFinite) {
    EXPECT(Vector3::repeating(2).isFinite());
    EXPECT(!Vector3(1, INFINITY, 0).isFinite());
    EXPECT(!Vector3(NAN, 0, 0).isFinite());
}

TEST(degreesBecomeRadians) {
    EXPECT(isApproximatelyEqual(Angle::degrees(180).radians, 3.14159265f));
    EXPECT(isApproximatelyEqual(Angle::degrees(90).radians, 1.57079633f));
}

TEST(angleBetweenPerpendicularVectorsIsARightAngle) {
    EXPECT(isApproximatelyEqual(Angle::between(Vector3(1, 0, 0), Vector3(0, 0, 3)).radians, 1.57079633f));
}
