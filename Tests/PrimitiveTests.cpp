// Vector2 and the geometric primitives, with values worked out by hand
// (EditorTests/Math/Vector2Tests.swift and EditorTests/Math/Primitives/).
#include <cmath>
#include <limits>
#include <vector>

#include "../Numerics/SeededRandom.h"
#include "../Geometry/Bounds.h"
#include "../Geometry/Bounds2.h"
#include "../Geometry/Line.h"
#include "../Geometry/Plane.h"
#include "../Geometry/Ray.h"
#include "../Geometry/Segment.h"
#include "../Geometry/Segment2.h"
#include "../Geometry/Sphere.h"
#include "../Geometry/Triangle.h"
#include "../Linear/Vector2.h"
#include "Testing.h"

using namespace editor;

namespace {

bool isApproximatelyEqual(float first, float second, float tolerance = 1e-5f) {
    return testing::isApproximatelyEqual(first, second, tolerance);
}

bool isApproximatelyEqual(Vector3 first, Vector3 second, float tolerance = 1e-5f) {
    return testing::isApproximatelyEqual(first, second, tolerance);
}

bool isApproximatelyEqual(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

float distanceBetween(Vector3 first, Vector3 second) { return (second - first).length(); }

float distanceBetween(Vector2 first, Vector2 second) { return first.distance(second); }

Vector3 xAxis() { return Vector3(1, 0, 0); }
Vector3 zAxis() { return Vector3(0, 0, 1); }

} // namespace

// MARK: Vector2

TEST(vector2AxesAreTheUnitVectors) {
    EXPECT(Vector2::xAxis() == Vector2(1, 0));
    EXPECT(Vector2::yAxis() == Vector2(0, 1));
}

TEST(vector2DotProductSumsTheComponentProducts) {
    // 1·4 + 2·5 = 14
    EXPECT(Vector2(1, 2).dot(Vector2(4, 5)) == 14);
    EXPECT(Vector2::xAxis().dot(Vector2::yAxis()) == 0);
    EXPECT(Vector2::xAxis().dot(-Vector2::xAxis()) == -1);
}

TEST(vector2CrossProductIsTheSignedArea) {
    // x × y = 1·1 − 0·0 = +1: y is counterclockwise of x.
    EXPECT(Vector2::xAxis().cross(Vector2::yAxis()) == 1);
    // Swapping flips the sign.
    EXPECT(Vector2::yAxis().cross(Vector2::xAxis()) == -1);
    // (2, 1) × (1, 3) = 2·3 − 1·1 = 5: the parallelogram they span has area 5.
    EXPECT(Vector2(2, 1).cross(Vector2(1, 3)) == 5);
    // Parallel and opposite vectors span nothing.
    EXPECT(Vector2(2, 1).cross(Vector2(4, 2)) == 0);
    EXPECT(Vector2(2, 1).cross(Vector2(-2, -1)) == 0);
}

TEST(vector2CrossProductMatchesTheZComponentOfTheVector3Cross) {
    Vector2 first(0.7f, -1.3f);
    Vector2 second(-2.1f, 0.4f);
    Vector3 crossInSpace = Vector3(first.x, first.y, 0).cross(Vector3(second.x, second.y, 0));
    EXPECT(isApproximatelyEqual(first.cross(second), crossInSpace.z));
}

TEST(vector2LengthIsPythagoras) {
    // √(3² + 4²) = 5
    EXPECT(isApproximatelyEqual(Vector2(3, 4).length(), 5));
    EXPECT(Vector2(3, 4).lengthSquared() == 25);
}

TEST(vector2NormalizedHasLengthOneAndKeepsTheDirection) {
    // (3, 4) / 5 = (0.6, 0.8)
    EXPECT(isApproximatelyEqual(Vector2(3, 4).normalized(), Vector2(0.6f, 0.8f)));
    EXPECT(isApproximatelyEqual(Vector2(-0.003f, 0.004f).normalized(), Vector2(-0.6f, 0.8f)));
}

TEST(vector2NormalizingZeroGivesZero) { EXPECT(Vector2().normalized() == Vector2()); }

TEST(vector2PerpendicularTurnsNinetyDegreesCounterclockwise) {
    // (1, 0) → (0, 1) → (−1, 0) → (0, −1) → (1, 0)
    EXPECT(Vector2::xAxis().perpendicular() == Vector2(0, 1));
    EXPECT(Vector2::yAxis().perpendicular() == Vector2(-1, 0));
    EXPECT(Vector2::xAxis().perpendicular().perpendicular().perpendicular().perpendicular() == Vector2::xAxis());
    // Same length, at right angles, and counterclockwise: self × perpendicular = |self|².
    Vector2 vector(3, -2);
    EXPECT(isApproximatelyEqual(vector.perpendicular().length(), vector.length()));
    EXPECT(vector.dot(vector.perpendicular()) == 0);
    EXPECT(vector.cross(vector.perpendicular()) == 13);
}

TEST(vector2RightNormalIsTheNegatedPerpendicular) {
    EXPECT(Vector2(3, -2).rightNormal() == Vector2(-2, -3));
    EXPECT(Vector2(3, -2).rightNormal() == -Vector2(3, -2).perpendicular());
}

TEST(vector2UnitDirectionIsSafeAtEveryScale) {
    EXPECT(!Vector2().unitDirection().has_value());
    EXPECT(!Vector2(1, NAN).unitDirection().has_value());
    EXPECT(!Vector2(INFINITY, 0).unitDirection().has_value());
    EXPECT(isApproximatelyEqual(*Vector2(3e-20f, 4e-20f).unitDirection(), Vector2(0.6f, 0.8f)));
    EXPECT(isApproximatelyEqual(*Vector2(3e20f, 4e20f).unitDirection(), Vector2(0.6f, 0.8f)));
}

TEST(vector2DistanceIsTheLengthOfTheDifference) {
    // (1, 2) to (4, 6): (3, 4), distance 5.
    EXPECT(isApproximatelyEqual(Vector2(1, 2).distance(Vector2(4, 6)), 5));
    EXPECT(Vector2(1, 2).distance(Vector2(1, 2)) == 0);
}

TEST(vector2LinearBlendFollowsTheStraightLine) {
    Vector2 start(2, 0);
    Vector2 end(6, 8);
    EXPECT(Vector2::linearBlend(start, end, 0) == start);
    EXPECT(Vector2::linearBlend(start, end, 1) == end);
    // 0.25 of the way: (2 + 0.25·4, 0 + 0.25·8) = (3, 2)
    EXPECT(Vector2::linearBlend(start, end, 0.25f) == Vector2(3, 2));
    // Not clamped: −0.5 goes backwards, (2 − 2, 0 − 4) = (0, −4).
    EXPECT(Vector2::linearBlend(start, end, -0.5f) == Vector2(0, -4));
}

TEST(vector2AreaOfASizeIsWidthTimesHeight) {
    EXPECT(Vector2(3, 4).area() == 12);
    EXPECT(Vector2(3, 0).area() == 0);
}

TEST(vector2CellCentresSitHalfACellInFromTheCorner) {
    // A grid starting at (−1, 2) with cells 0.5 wide and 2 tall: cell (0, 0) is centred at
    // (−1 + 0.25, 2 + 1) = (−0.75, 3); cell (3, 2) at (−1 + 3.5·0.5, 2 + 2.5·2) = (0.75, 7).
    Vector2 corner(-1, 2);
    Vector2 cell(0.5f, 2);
    EXPECT(Vector2::cellCentre(0, 0, corner, cell) == Vector2(-0.75f, 3));
    EXPECT(Vector2::cellCentre(3, 2, corner, cell) == Vector2(0.75f, 7));
}

TEST(vector3LengthSquaredIsTheDotWithItself) {
    // 1² + 2² + 2² = 9, and its square root is the length, 3.
    EXPECT(Vector3(1, 2, 2).dot(Vector3(1, 2, 2)) == 9);
    EXPECT(isApproximatelyEqual(Vector3(1, 2, 2).length(), 3));
    EXPECT(Vector3().dot(Vector3()) == 0);
}

// MARK: Line

TEST(lineDirectionIsNormalized) {
    Line line(Vector3(1, 1, 1), Vector3(0, 0, 5));
    EXPECT(line.direction == Vector3(0, 0, 1));
    // Even a very long or very short direction is accepted and normalized.
    EXPECT(isApproximatelyEqual(Line(Vector3(), Vector3(0, 3e-20f, 0)).direction, Vector3(0, 1, 0)));
}

TEST(lineDirectionOfAnySizeGivesTheSameUnitDirection) {
    // (3, 4, 0)·size has length 5·size, so the unit direction is (0.6, 0.8, 0). Squaring the
    // components directly would underflow at 1e-25 and overflow at 1e20; dividing by the largest
    // component first avoids both. Measured exact at every size; the tolerance is one float step
    // at 0.8 (6e-8).
    for (float size : {1e-25f, 1e-3f, 1.0f, 1e20f}) {
        Line line(Vector3(), Vector3(3, 4, 0) * size);
        EXPECT(isApproximatelyEqual(line.direction, Vector3(0.6f, 0.8f, 0), 6e-8f));
    }
}

TEST(lineRefusesADirectionWithoutAHeading) {
    EXPECT_PROBLEM(Line(Vector3(), Vector3()), "A line needs a finite direction that is not zero, but got [0.0, 0.0, 0.0].");
    EXPECT_PROBLEM(Line(Vector3(), Vector3(NAN, 0, 0)), "A line needs a finite direction");
}

TEST(lineDistanceAlongIsTheShadowOnTheLine) {
    // A vertical line through (1, 1, 1): the point (4, 5, 9) is 8 above the origin.
    Line line(Vector3(1, 1, 1), zAxis());
    EXPECT(isApproximatelyEqual(line.distanceAlong(Vector3(4, 5, 9)), 8));
    // Behind the origin it is negative.
    EXPECT(isApproximatelyEqual(line.distanceAlong(Vector3(4, 5, -2)), -3));
}

TEST(lineClosestPointOnAVerticalLine) {
    Line line(Vector3(1, 1, 1), zAxis());
    EXPECT(isApproximatelyEqual(line.closestPoint(Vector3(4, 5, 9)), Vector3(1, 1, 9)));
}

TEST(lineDistanceToAVerticalLineIsThePerpendicularOffset) {
    // The offset in x and y is (3, 4), so the distance is 5 whatever the height.
    Line line(Vector3(1, 1, 1), zAxis());
    EXPECT(isApproximatelyEqual(line.distance(Vector3(4, 5, 9)), 5));
    EXPECT(isApproximatelyEqual(line.distance(Vector3(4, 5, -100)), 5));
}

TEST(lineDistanceToATiltedLine) {
    // The line through the origin along (1, 1, 0). The point (2, 0, 3) projects onto it at
    // distance (2 + 0)/√2 = √2, the foot is (1, 1, 0), and the offset (1, −1, 3) has length √11.
    Line line(Vector3(), Vector3(1, 1, 0));
    Vector3 point(2, 0, 3);
    EXPECT(isApproximatelyEqual(line.distanceAlong(point), std::sqrt(2.0f)));
    EXPECT(isApproximatelyEqual(line.closestPoint(point), Vector3(1, 1, 0)));
    EXPECT(isApproximatelyEqual(line.distance(point), std::sqrt(11.0f)));
}

TEST(lineAPointOnTheLineIsAtDistanceZero) {
    Line line(Vector3(1, 2, 3), Vector3(1, -2, 2));
    Vector3 onTheLine = line.origin + line.direction * 7.5f;
    EXPECT(isApproximatelyEqual(line.distance(onTheLine), 0, 1e-5f));
}

TEST(lineHasNoEnds) {
    // Unlike a segment, the foot of the perpendicular is not clamped.
    Line line(Vector3(), xAxis());
    EXPECT(line.closestPoint(Vector3(1000, 2, 0)) == Vector3(1000, 0, 0));
    EXPECT(line.closestPoint(Vector3(-1000, 2, 0)) == Vector3(-1000, 0, 0));
}

TEST(lineFootIsPerpendicularAndAsNearAsAnyOtherPoint) {
    Line line(Vector3(0.5f, -1, 2), Vector3(2, 1, -1));
    for (Vector3 point : {Vector3(3, 4, 5), Vector3(-2, 0, 1), Vector3(0, 0, 0)}) {
        Vector3 foot = line.closestPoint(point);
        EXPECT(isApproximatelyEqual((point - foot).dot(line.direction), 0, 1e-5f));
        for (float step : {-2.0f, -0.5f, 0.5f, 2.0f}) {
            Vector3 other = foot + line.direction * step;
            EXPECT(distanceBetween(point, other) >= distanceBetween(point, foot));
        }
    }
}

TEST(lineDistanceScalesWithTheScene) {
    // The vertical-line example scaled by 1 mm and 1 km: the distance scales the same way.
    for (float scale : {1e-3f, 1.0f, 1e3f}) {
        Line line(Vector3(1, 1, 1) * scale, zAxis());
        float distance = line.distance(Vector3(4, 5, 9) * scale);
        EXPECT(isApproximatelyEqual(distance, 5 * scale, 1e-5f * scale));
    }
}

TEST(lineDistanceAKilometreFromTheOrigin) {
    // The same example with the line and the point both moved a kilometre out. The distance is
    // still 5·scale: it depends only on the point relative to the line's origin.
    for (float scale : {1.0f, 1e3f}) {
        Vector3 far(1000, -1000, 500);
        Line line(far + Vector3(1, 1, 1) * scale, Vector3(0, 0, 2));
        float distance = line.distance(far + Vector3(4, 5, 9) * scale);
        // Measured exact; the tolerance is about two float steps of the distance, 2.4e-7 relative.
        EXPECT(isApproximatelyEqual(distance, 5 * scale, 2.4e-7f * 5 * scale));
        // A tilted line: through far along (1, 1, 0), the point far + (2, 0, 3) is √11 away.
        Line tilted(far, Vector3(1, 1, 0));
        EXPECT(isApproximatelyEqual(tilted.distance(far + Vector3(2, 0, 3)), std::sqrt(11.0f), 4.8e-7f));
    }
}

// MARK: Plane

TEST(planeFromPointAndNormalNormalizesAndFindsDistance) {
    // Normal (0, 0, 5) → (0, 0, 1); distance = (0,0,1)·(7, 8, 2) = 2.
    Plane plane(Vector3(7, 8, 2), Vector3(0, 0, 5));
    EXPECT(isApproximatelyEqual(plane.normal, Vector3(0, 0, 1)));
    EXPECT(isApproximatelyEqual(plane.distance, 2));
    EXPECT_PROBLEM(Plane(Vector3(), Vector3()), "A plane needs a normal that is not zero.");
}

TEST(planeFromThreeCounterclockwisePointsFollowsTheRightHandRule) {
    // (1,0,0) × (0,1,0) = (0,0,1): counterclockwise seen from +z gives an upward normal.
    Plane up(Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
    EXPECT(isApproximatelyEqual(up.normal, Vector3(0, 0, 1)));
    EXPECT(isApproximatelyEqual(up.distance, 0));
    // Swapping two corners flips the facing.
    Plane down(Vector3(0, 0, 0), Vector3(0, 1, 0), Vector3(1, 0, 0));
    EXPECT(isApproximatelyEqual(down.normal, Vector3(0, 0, -1)));
    // Lifted by 3: the plane z = 3.
    Plane lifted(Vector3(0, 0, 3), Vector3(2, 0, 3), Vector3(0, 5, 3));
    EXPECT(isApproximatelyEqual(lifted.distance, 3));
    // Corners in a line define no plane.
    EXPECT_PROBLEM(Plane(Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0)),
                   "Three corners in a line do not define a plane.");
}

TEST(planeMillimetreTriangleDefinesAPlane) {
    // Legs of 0.9 mm: the cross product has length 8.1e-7, below an absolute 1e-6 test, yet the
    // triangle is perfectly valid. Normal +z (counterclockwise from above), distance 0.
    Plane plane(Vector3(0, 0, 0), Vector3(0.0009f, 0, 0), Vector3(0, 0.0009f, 0));
    EXPECT(isApproximatelyEqual(plane.normal, Vector3(0, 0, 1)));
    EXPECT(isApproximatelyEqual(plane.distance, 0));
}

TEST(planeSignedDistanceHasASidePerNormal) {
    Plane plane(Vector3(0, 0, 2), Vector3(0, 0, 1));
    // z = 5 is 5 − 2 = 3 in front; z = −1 is −1 − 2 = −3 behind; z = 2 is on the plane.
    EXPECT(isApproximatelyEqual(plane.signedDistance(Vector3(3, 4, 5)), 3));
    EXPECT(isApproximatelyEqual(plane.signedDistance(Vector3(0, 0, -1)), -3));
    EXPECT(isApproximatelyEqual(plane.signedDistance(Vector3(9, 9, 2)), 0));
}

TEST(planeProjectionLandsOnThePlane) {
    Plane plane(Vector3(0, 0, 2), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(plane.projection(Vector3(3, 4, 5)), Vector3(3, 4, 2)));
    // A tilted plane x + y = 2 (normal (1,1,0)/√2, distance √2): (2,2,0) is 4/√2 − √2 = √2 away
    // and projects to (1, 1, 0).
    Plane tilted(Vector3(1, 1, 0), Vector3(1, 1, 0));
    EXPECT(isApproximatelyEqual(tilted.projection(Vector3(2, 2, 0)), Vector3(1, 1, 0)));
}

TEST(planeProjectionProperties) {
    std::vector<Plane> planes = {
        Plane(Vector3(1, 2, 3), Vector3(1, 2, -1)),
        Plane(Vector3(-4, 0, 1), Vector3(0, -1, 0)),
    };
    const float values[] = {-5, -1, 0, 2.5f, 6};
    for (const Plane& plane : planes) {
        for (float x : values) {
            for (float y : values) {
                for (float z : values) {
                    Vector3 point(x, y, z);
                    Vector3 projected = plane.projection(point);
                    EXPECT(isApproximatelyEqual(plane.signedDistance(projected), 0, 1e-4f));
                    // The slide is exactly the signed distance long.
                    EXPECT(isApproximatelyEqual(distanceBetween(point, projected), std::fabs(plane.signedDistance(point)), 1e-4f));
                    // Projecting twice changes nothing.
                    EXPECT(isApproximatelyEqual(plane.projection(projected), projected, 1e-4f));
                }
            }
        }
    }
}

// MARK: Ray

TEST(rayDirectionIsNormalized) {
    // (0, 3, 4) has length √(0 + 9 + 16) = 5, so the unit direction is (0, 0.6, 0.8).
    Ray ray(Vector3(1, 1, 1), Vector3(0, 3, 4));
    EXPECT(isApproximatelyEqual(ray.direction, Vector3(0, 0.6f, 0.8f)));
    EXPECT_PROBLEM(Ray(Vector3(), Vector3()), "A ray needs a direction that is not zero.");
}

TEST(rayPointAtDistance) {
    Ray ray(Vector3(1, 2, 3), Vector3(0, 0, 2));
    // Distance 5 along +z: (1, 2, 3 + 5).
    EXPECT(isApproximatelyEqual(ray.point(5), Vector3(1, 2, 8)));
    EXPECT(ray.point(0) == ray.origin);
    // Behind the origin: (1, 2, 3 − 1).
    EXPECT(isApproximatelyEqual(ray.point(-1), Vector3(1, 2, 2)));
}

TEST(rayPointsAreTheRequestedDistanceFromTheOrigin) {
    for (Vector3 direction : {Vector3(1, 0, 0), Vector3(1, 1, 0), Vector3(-2, 3, 5), Vector3(0, 0, -7)}) {
        Ray ray(Vector3(4, -2, 1), direction);
        EXPECT(isApproximatelyEqual(ray.direction.length(), 1));
        for (float distance : {0.0f, 0.5f, 3.0f, 12.0f}) {
            EXPECT(isApproximatelyEqual(distanceBetween(ray.origin, ray.point(distance)), distance, 1e-4f));
        }
    }
}

// MARK: Segment2

TEST(segment2APointBesideTheSegmentDropsStraightOntoIt) {
    Segment2 segment{Vector2(0, 0), Vector2(4, 0)};
    // The foot of the perpendicular from (1, 3) is (1, 0), 3 away.
    EXPECT(isApproximatelyEqual(segment.closestPoint(Vector2(1, 3)), Vector2(1, 0)));
    EXPECT(isApproximatelyEqual(segment.distance(Vector2(1, 3)), 3));
}

TEST(segment2APointPastAnEndIsNearestThatEnd) {
    Segment2 segment{Vector2(0, 0), Vector2(4, 0)};
    // (7, 4) is past the end (4, 0): the offset (3, 4) is 5 long.
    EXPECT(isApproximatelyEqual(segment.closestPoint(Vector2(7, 4)), Vector2(4, 0)));
    EXPECT(isApproximatelyEqual(segment.distance(Vector2(7, 4)), 5));
    // (−3, −4) is before the start.
    EXPECT(isApproximatelyEqual(segment.distance(Vector2(-3, -4)), 5));
}

TEST(segment2OfZeroLengthIsItsStartPoint) {
    Segment2 point{Vector2(1, 1), Vector2(1, 1)};
    EXPECT(point.closestPoint(Vector2(4, 5)) == Vector2(1, 1));
    EXPECT(isApproximatelyEqual(point.distance(Vector2(4, 5)), 5));
}

TEST(segment2NearestPointIsNeverFartherThanEitherEnd) {
    SeededRandom random(11);
    auto randomPoint = [&random]() {
        float x = random.nextFloat(-3, 3);
        float y = random.nextFloat(-3, 3);
        return Vector2(x, y);
    };
    for (int round = 0; round < 200; round += 1) {
        Vector2 start = randomPoint();
        Vector2 end = randomPoint();
        Segment2 side{start, end};
        Vector2 point = randomPoint();
        float distance = side.distance(point);
        EXPECT(distance <= distanceBetween(point, side.start) + 1e-5f);
        EXPECT(distance <= distanceBetween(point, side.end) + 1e-5f);
    }
}

// MARK: Segment

TEST(segmentClosestPointInTheMiddleIsThePerpendicularFoot) {
    Segment segment{Vector3(0, 0, 0), Vector3(4, 0, 0)};
    // direction (4,0,0); t = (1,3,0)·(4,0,0) / 16 = 4/16 = 0.25 → (1, 0, 0).
    EXPECT(isApproximatelyEqual(segment.closestPoint(Vector3(1, 3, 0)), Vector3(1, 0, 0)));
    // A point off in z too: t = 12/16 = 0.75 → (3, 0, 0).
    EXPECT(isApproximatelyEqual(segment.closestPoint(Vector3(3, -2, 5)), Vector3(3, 0, 0)));
}

TEST(segmentClosestPointBeyondTheEndsIsAnEnd) {
    Segment segment{Vector3(0, 0, 0), Vector3(4, 0, 0)};
    // t = −2·4 / 16 = −0.5, clamped to 0 → start.
    EXPECT(segment.closestPoint(Vector3(-2, 1, 0)) == segment.start);
    // t = 36 / 16 = 2.25, clamped to 1 → end.
    EXPECT(segment.closestPoint(Vector3(9, 0, 5)) == segment.end);
}

TEST(segmentDegenerateSegmentIsAPoint) {
    Segment point{Vector3(1, 2, 3), Vector3(1, 2, 3)};
    EXPECT(point.closestPoint(Vector3(9, 9, 9)) == Vector3(1, 2, 3));
}

TEST(segmentClosestPointIsNoFartherThanAnyPointOfTheSegment) {
    Segment tilted{Vector3(-1, 2, 0), Vector3(3, -1, 2)};
    const float probes[] = {-6, -2, 0, 1.5f, 4, 7};
    for (float x : probes) {
        for (float y : probes) {
            for (float z : probes) {
                Vector3 probe(x, y, z);
                Vector3 closest = tilted.closestPoint(probe);
                float closestDistance = distanceBetween(probe, closest);
                for (int step = 0; step <= 20; step += 1) {
                    float fraction = static_cast<float>(step) * 0.05f;
                    Vector3 sample = Vector3::linearBlend(tilted.start, tilted.end, fraction);
                    EXPECT(closestDistance <= distanceBetween(probe, sample) + 1e-4f);
                }
            }
        }
    }
}

// MARK: Sphere and bounds

TEST(sphereStoresCentreAndRadius) {
    Sphere sphere(Vector3(1, 2, 3), 4);
    EXPECT(sphere.centre == Vector3(1, 2, 3));
    EXPECT(sphere.radius == 4);
    EXPECT_PROBLEM(Sphere(Vector3(), -1), "A sphere cannot have a negative radius.");
}

namespace {
const std::vector<Vector3> boundsPoints = {Vector3(1, 2, 3), Vector3(-1, 5, 0), Vector3(4, -2, 2)};
}

TEST(boundsEnclosingPointsTakesTheExtremesOnEachAxis) {
    // x: min(1, −1, 4) = −1, max = 4. y: min(2, 5, −2) = −2, max = 5. z: min 0, max 3.
    Bounds bounds = Bounds::enclosing(boundsPoints);
    EXPECT(bounds.minimum == Vector3(-1, -2, 0));
    EXPECT(bounds.maximum == Vector3(4, 5, 3));
    EXPECT_PROBLEM(Bounds::enclosing({}), "Cannot enclose an empty list of points.");
}

TEST(boundsSinglePointGivesAFlatBox) {
    Bounds bounds = Bounds::enclosing({Vector3(2, 3, 4)});
    EXPECT(bounds.minimum == bounds.maximum);
    EXPECT(bounds.size() == Vector3(0, 0, 0));
}

TEST(boundsCentreAndSize) {
    Bounds bounds = Bounds::enclosing(boundsPoints);
    // Centre: ((−1+4)/2, (−2+5)/2, (0+3)/2) = (1.5, 1.5, 1.5). Size: (5, 7, 3).
    EXPECT(isApproximatelyEqual(bounds.centre(), Vector3(1.5f, 1.5f, 1.5f)));
    EXPECT(isApproximatelyEqual(bounds.size(), Vector3(5, 7, 3)));
}

TEST(boundsCentreAndSizeGiveTheCorners) {
    // Half of (5, 7, 3) is (2.5, 3.5, 1.5): the corners are (1.5, 1.5, 1.5) ∓ that.
    Bounds bounds = Bounds::fromCentre(Vector3(1.5f, 1.5f, 1.5f), Vector3(5, 7, 3));
    EXPECT(isApproximatelyEqual(bounds.minimum, Vector3(-1, -2, 0)));
    EXPECT(isApproximatelyEqual(bounds.maximum, Vector3(4, 5, 3)));
}

TEST(boundsRefuseToBeInsideOut) {
    EXPECT_PROBLEM(Bounds::checked(Vector3(1, 0, 0), Vector3(0, 1, 1)), "Bounds needs minimum ≤ maximum on every axis.");
    EXPECT_PROBLEM(Bounds::fromCentre(Vector3(), Vector3(-1, 1, 1)), "Bounds needs minimum ≤ maximum");
}

TEST(boundsUnionTakesTheSmallerMinimumAndLargerMaximum) {
    Bounds first = Bounds::checked(Vector3(-1, -2, 0), Vector3(4, 5, 3));
    Bounds second = Bounds::checked(Vector3(0, 0, -5), Vector3(10, 1, 1));
    Bounds combined = first.unionWith(second);
    EXPECT(combined.minimum == Vector3(-1, -2, -5));
    EXPECT(combined.maximum == Vector3(10, 5, 3));
    EXPECT(second.unionWith(first) == combined);
    EXPECT(first.unionWith(first) == first);
}

TEST(boundsContainsIncludesTheSurface) {
    Bounds bounds = Bounds::checked(Vector3(0, 0, 0), Vector3(1, 2, 3));
    EXPECT(bounds.contains(Vector3(0.5f, 1, 1.5f)));
    EXPECT(bounds.contains(Vector3(0, 0, 0)));
    EXPECT(bounds.contains(Vector3(1, 2, 3)));
    EXPECT(!bounds.contains(Vector3(1.001f, 1, 1)));
    EXPECT(!bounds.contains(Vector3(0.5f, -0.001f, 1)));
    EXPECT(!bounds.contains(Vector3(0.5f, 1, 3.001f)));
}

TEST(boundsEnclosingBoxContainsEveryPoint) {
    const float values[] = {-3, -0.5f, 0, 2, 7};
    std::vector<Vector3> grid;
    for (float x : values) for (float y : values) for (float z : values) grid.push_back(Vector3(x, y, z));
    // Use growing prefixes so many different boxes are tested.
    for (size_t count = 1; count <= grid.size(); count += 7) {
        std::vector<Vector3> subset(grid.begin(), grid.begin() + static_cast<long>(count));
        std::vector<Vector3> suffix(grid.end() - static_cast<long>(count), grid.end());
        Bounds bounds = Bounds::enclosing(subset);
        for (Vector3 point : subset) EXPECT(bounds.contains(point));
        Bounds combined = bounds.unionWith(Bounds::enclosing(suffix));
        for (Vector3 point : subset) EXPECT(combined.contains(point));
        for (Vector3 point : suffix) EXPECT(combined.contains(point));
    }
}

TEST(bounds2EnclosingContainsAndSizes) {
    Bounds2 bounds = Bounds2::enclosing({Vector2(1, 2), Vector2(-1, 5), Vector2(4, -2)});
    EXPECT(bounds.minimum == Vector2(-1, -2));
    EXPECT(bounds.maximum == Vector2(4, 5));
    EXPECT(bounds.size() == Vector2(5, 7));
    EXPECT(bounds.contains(Vector2(4, 5)));
    EXPECT(!bounds.contains(Vector2(4.001f, 0)));
    Bounds2 combined = bounds.unionWith(Bounds2::checked(Vector2(0, 0), Vector2(10, 1)));
    EXPECT(combined.minimum == Vector2(-1, -2));
    EXPECT(combined.maximum == Vector2(10, 5));
    EXPECT_PROBLEM(Bounds2::checked(Vector2(1, 0), Vector2(0, 1)), "Bounds2 needs minimum ≤ maximum on both axes.");
    EXPECT_PROBLEM(Bounds2::enclosing({}), "Cannot enclose an empty list of points.");
}

// MARK: Triangle

namespace {

// A right triangle in the z = 0 plane: corners (0,0,0), (4,0,0), (0,3,0). Hypotenuse 5.
Triangle rightTriangle() { return Triangle{Vector3(0, 0, 0), Vector3(4, 0, 0), Vector3(0, 3, 0)}; }
// Three collinear corners on the x axis.
Triangle flatTriangle() { return Triangle{Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0)}; }
Triangle skewedTriangle() { return Triangle{Vector3(1, 0, 0), Vector3(0, 2, 1), Vector3(-1, -1, 2)}; }

} // namespace

TEST(triangleNormalFollowsTheRightHandRule) {
    Triangle triangle = rightTriangle();
    // edge1 × edge2 = (4,0,0) × (0,3,0) = (0, 0, 12) → unit (0, 0, 1).
    EXPECT(isApproximatelyEqual(triangle.normal(), Vector3(0, 0, 1)));
    Triangle reversed{triangle.first, triangle.third, triangle.second};
    EXPECT(isApproximatelyEqual(reversed.normal(), Vector3(0, 0, -1)));
}

TEST(triangleAreaAndCentroid) {
    Triangle triangle = rightTriangle();
    // Half of base × height = 4 × 3 / 2 = 6. Also |(0,0,12)| / 2.
    EXPECT(isApproximatelyEqual(triangle.area(), 6));
    // ((0+4+0)/3, (0+0+3)/3, 0) = (4/3, 1, 0).
    EXPECT(isApproximatelyEqual(triangle.centroid(), Vector3(4.0f / 3.0f, 1, 0)));
}

TEST(triangleDegenerateTriangleHasNoFacing) {
    Triangle flat = flatTriangle();
    EXPECT(flat.isDegenerate());
    EXPECT(!rightTriangle().isDegenerate());
    EXPECT(flat.normal() == Vector3(0, 0, 0));
    EXPECT(isApproximatelyEqual(flat.area(), 0));
    EXPECT(!flat.barycentricCoordinates(Vector3(1, 0, 0)).has_value());
}

TEST(triangleThinButValidTriangleIsNotDegenerate) {
    // Height 1 over longest edge 100: flatness 1e-2, far above the 1e-6 threshold.
    Triangle thin{Vector3(0, 0, 0), Vector3(100, 0, 0), Vector3(50, 1, 0)};
    EXPECT(!thin.isDegenerate());
}

TEST(triangleBarycentricCoordinatesOfWorkedPoints) {
    Triangle triangle = rightTriangle();
    // A corner is all one weight.
    auto atSecond = triangle.barycentricCoordinates(Vector3(4, 0, 0));
    EXPECT(atSecond.has_value());
    EXPECT(isApproximatelyEqual(atSecond->first, 0));
    EXPECT(isApproximatelyEqual(atSecond->second, 1));
    EXPECT(isApproximatelyEqual(atSecond->third, 0));
    // The centroid is a third of each.
    auto atCentroid = triangle.barycentricCoordinates(triangle.centroid());
    EXPECT(atCentroid.has_value());
    EXPECT(isApproximatelyEqual(atCentroid->first, 1.0f / 3.0f));
    EXPECT(isApproximatelyEqual(atCentroid->second, 1.0f / 3.0f));
    EXPECT(isApproximatelyEqual(atCentroid->third, 1.0f / 3.0f));
    // (1,1,0) = 0.25·(4,0,0) + (1/3)·(0,3,0), so second = 0.25, third = 1/3, first = 5/12.
    auto inside = triangle.barycentricCoordinates(Vector3(1, 1, 0));
    EXPECT(inside.has_value());
    EXPECT(isApproximatelyEqual(inside->first, 5.0f / 12.0f));
    EXPECT(isApproximatelyEqual(inside->second, 0.25f));
    EXPECT(isApproximatelyEqual(inside->third, 1.0f / 3.0f));
    EXPECT(inside->isInsideTriangle());
    // (5,0,0) is past the second corner: second = 1.25, first = −0.25.
    auto outside = triangle.barycentricCoordinates(Vector3(5, 0, 0));
    EXPECT(outside.has_value());
    EXPECT(isApproximatelyEqual(outside->first, -0.25f));
    EXPECT(!outside->isInsideTriangle());
}

TEST(triangleBarycentricCoordinatesReproduceThePoint) {
    Triangle skewed = skewedTriangle();
    const float weights[] = {-0.5f, 0, 0.2f, 0.5f, 1};
    for (float second : weights) {
        for (float third : weights) {
            float first = 1 - second - third;
            Vector3 point = skewed.first * first + skewed.second * second + skewed.third * third;
            auto found = skewed.barycentricCoordinates(point);
            EXPECT(found.has_value());
            if (!found) continue;
            EXPECT(isApproximatelyEqual(found->first, first, 1e-4f));
            EXPECT(isApproximatelyEqual(found->second, second, 1e-4f));
            EXPECT(isApproximatelyEqual(found->third, third, 1e-4f));
            EXPECT(isApproximatelyEqual(found->first + found->second + found->third, 1, 1e-5f));
        }
    }
}

TEST(triangleClosestPointInEachRegion) {
    Triangle triangle = rightTriangle();
    // Face region: straight down onto the face.
    EXPECT(isApproximatelyEqual(triangle.closestPoint(Vector3(1, 1, 5)), Vector3(1, 1, 0)));
    // Corner regions.
    EXPECT(isApproximatelyEqual(triangle.closestPoint(Vector3(-1, -1, 2)), triangle.first));
    EXPECT(isApproximatelyEqual(triangle.closestPoint(Vector3(5, -1, 0)), triangle.second));
    EXPECT(isApproximatelyEqual(triangle.closestPoint(Vector3(-1, 4, 0)), triangle.third));
    // Edge first→second (y = 0): (2, −3, 1) lands on (2, 0, 0).
    EXPECT(isApproximatelyEqual(triangle.closestPoint(Vector3(2, -3, 1)), Vector3(2, 0, 0)));
    // Edge first→third (x = 0): (−2, 1.5, 0) lands on (0, 1.5, 0).
    EXPECT(isApproximatelyEqual(triangle.closestPoint(Vector3(-2, 1.5f, 0)), Vector3(0, 1.5f, 0)));
    // Hypotenuse second→third: from (4,3,0), offset from second is (0,3,0), direction (−4,3,0);
    // t = 9 / 25 = 0.36 → (4 − 1.44, 1.08, 0) = (2.56, 1.08, 0).
    EXPECT(isApproximatelyEqual(triangle.closestPoint(Vector3(4, 3, 0)), Vector3(2.56f, 1.08f, 0)));
}

TEST(trianglePointsInsideAreTheirOwnClosestPoint) {
    Triangle triangle = rightTriangle();
    for (Vector3 point : {Vector3(1, 1, 0), Vector3(0.5f, 0.5f, 0), triangle.centroid()}) {
        EXPECT(isApproximatelyEqual(triangle.closestPoint(point), point));
    }
}

TEST(triangleDegenerateTriangleFallsBackToItsEdges) {
    Triangle flat = flatTriangle();
    // The line from x = 0 to x = 2.
    EXPECT(isApproximatelyEqual(flat.closestPoint(Vector3(0.5f, 1, 0)), Vector3(0.5f, 0, 0)));
    EXPECT(isApproximatelyEqual(flat.closestPoint(Vector3(3, 1, 0)), Vector3(2, 0, 0)));
    EXPECT(isApproximatelyEqual(flat.closestPoint(Vector3(-3, 1, 0)), Vector3(0, 0, 0)));
}

TEST(triangleClosestPointProperties) {
    Triangle skewed = skewedTriangle();
    Plane plane(skewed.first, skewed.second, skewed.third);
    const float probes[] = {-6, -2, 0, 1.5f, 4, 7};
    for (float x : probes) {
        for (float y : probes) {
            for (float z : probes) {
                Vector3 probe(x, y, z);
                Vector3 closest = skewed.closestPoint(probe);
                float closestDistance = distanceBetween(probe, closest);

                // It is on the triangle: on its plane, with weights in [0, 1].
                EXPECT(isApproximatelyEqual(plane.signedDistance(closest), 0, 1e-4f));
                auto weights = skewed.barycentricCoordinates(closest);
                EXPECT(weights.has_value());
                if (weights) EXPECT(weights->isInsideTriangle(1e-4f));

                // It is no farther than the corners, or than a grid of points inside.
                for (Vector3 corner : {skewed.first, skewed.second, skewed.third}) {
                    EXPECT(closestDistance <= distanceBetween(probe, corner) + 1e-4f);
                }
                for (int secondStep = 0; secondStep <= 10; secondStep += 1) {
                    float second = static_cast<float>(secondStep) * 0.1f;
                    for (int thirdStep = 0; static_cast<float>(thirdStep) * 0.1f <= 1 - second; thirdStep += 1) {
                        float third = static_cast<float>(thirdStep) * 0.1f;
                        Vector3 sample = skewed.first * (1 - second - third) + skewed.second * second + skewed.third * third;
                        EXPECT(closestDistance <= distanceBetween(probe, sample) + 1e-4f);
                    }
                }
            }
        }
    }
}

// MARK: Ray against a plane

namespace {
Plane groundAtTwo() { return Plane(Vector3(0, 0, 2), Vector3(0, 0, 1)); }
} // namespace

TEST(rayHitsPlaneAtTheComputedDistance) {
    // Straight up from the origin: t = (2 − 0) / 1 = 2.
    Ray straight(Vector3(0, 0, 0), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(*straight.intersectionDistance(groundAtTwo()), 2));
    // Direction (1,0,1)/√2: normal·direction = 1/√2, t = 2 / (1/√2) = 2√2 ≈ 2.8284271.
    Ray slanted(Vector3(0, 0, 0), Vector3(1, 0, 1));
    EXPECT(isApproximatelyEqual(*slanted.intersectionDistance(groundAtTwo()), 2.8284271f));
    // From above, pointing down: t = (2 − 5) / (−1) = 3. Planes are two-sided.
    Ray falling(Vector3(0, 0, 5), Vector3(0, 0, -1));
    EXPECT(isApproximatelyEqual(*falling.intersectionDistance(groundAtTwo()), 3));
}

TEST(rayMissesPlaneBehindOrParallel) {
    // Pointing away: t = (2 − 0) / (−1) = −2, behind.
    EXPECT(!Ray(Vector3(), Vector3(0, 0, -1)).intersectionDistance(groundAtTwo()).has_value());
    // Parallel and above the plane.
    EXPECT(!Ray(Vector3(0, 0, 5), Vector3(1, 0, 0)).intersectionDistance(groundAtTwo()).has_value());
    // Parallel and lying in the plane: no first hit.
    EXPECT(!Ray(Vector3(0, 0, 2), Vector3(1, 0, 0)).intersectionDistance(groundAtTwo()).has_value());
}

TEST(rayStartingOnPlaneHitsAtZero) {
    Ray ray(Vector3(3, 3, 2), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(groundAtTwo()), 0));
}

TEST(rayPlaneHitPointIsOnThePlane) {
    Plane plane(Vector3(1, 2, 3), Vector3(1, -2, 1));
    const float values[] = {-4, -1, 0, 2, 5};
    for (float x : values) for (float y : values) for (float z : values) {
        Ray ray(Vector3(x, y, z), Vector3(0.3f, 1, -0.7f));
        auto distance = ray.intersectionDistance(plane);
        if (!distance) continue;
        EXPECT(*distance >= 0);
        EXPECT(isApproximatelyEqual(plane.signedDistance(ray.point(*distance)), 0, 1e-3f));
    }
}

// MARK: Ray against a triangle

TEST(rayHitsTriangleWithWorkedBarycentrics) {
    // Down from (1,1,5): distance 5. Weights as in the barycentric test for (1,1,0):
    // second = 0.25, third = 1/3, first = 5/12.
    Ray ray(Vector3(1, 1, 5), Vector3(0, 0, -1));
    auto hit = ray.intersection(rightTriangle());
    EXPECT(hit.has_value());
    if (!hit) return;
    EXPECT(isApproximatelyEqual(hit->distance, 5));
    EXPECT(isApproximatelyEqual(hit->barycentric.first, 5.0f / 12.0f));
    EXPECT(isApproximatelyEqual(hit->barycentric.second, 0.25f));
    EXPECT(isApproximatelyEqual(hit->barycentric.third, 1.0f / 3.0f));
}

TEST(rayHitsTriangleFromBothSides) {
    Ray fromBelow(Vector3(1, 1, -5), Vector3(0, 0, 1));
    auto hit = fromBelow.intersection(rightTriangle());
    EXPECT(hit.has_value());
    if (hit) EXPECT(isApproximatelyEqual(hit->distance, 5));
}

TEST(rayMissesTriangle) {
    Triangle triangle = rightTriangle();
    // Beside it: (5,5) is outside.
    EXPECT(!Ray(Vector3(5, 5, 5), Vector3(0, 0, -1)).intersection(triangle).has_value());
    // Inside the bounding box but across the hypotenuse: 3/4 + 2/3 > 1.
    EXPECT(!Ray(Vector3(3, 2, 5), Vector3(0, 0, -1)).intersection(triangle).has_value());
    // Triangle behind the origin.
    EXPECT(!Ray(Vector3(1, 1, -5), Vector3(0, 0, -1)).intersection(triangle).has_value());
    // Parallel to the triangle.
    EXPECT(!Ray(Vector3(1, 1, 5), Vector3(1, 0, 0)).intersection(triangle).has_value());
}

TEST(rayNeverHitsADegenerateTriangle) {
    EXPECT(!Ray(Vector3(1, 0, 5), Vector3(0, 0, -1)).intersection(flatTriangle()).has_value());
}

TEST(rayHitsAMillimetreTriangle) {
    // Legs of 0.9 mm: the raw determinant is 8.1e-7, under the 1e-6 tolerance, so an absolute test
    // would call this ray parallel. Head-on from 1 m above (0.3 mm, 0.3 mm): distance 1, weights
    // u = v = 0.3 / 0.9 = 1/3.
    Triangle tiny{Vector3(0, 0, 0), Vector3(0.0009f, 0, 0), Vector3(0, 0.0009f, 0)};
    Ray headOn(Vector3(0.0003f, 0.0003f, 1), Vector3(0, 0, -1));
    auto hit = headOn.intersection(tiny);
    EXPECT(hit.has_value());
    if (!hit) return;
    EXPECT(isApproximatelyEqual(hit->distance, 1));
    EXPECT(isApproximatelyEqual(hit->barycentric.second, 1.0f / 3, 1e-3f));
    EXPECT(isApproximatelyEqual(hit->barycentric.third, 1.0f / 3, 1e-3f));
    // 60° from the normal: direction (sin 60°, 0, −cos 60°), starting 2 back from the target.
    float sixtyDegrees = Angle::degrees(60).radians;
    Vector3 slanted(std::sin(sixtyDegrees), 0, -std::cos(sixtyDegrees));
    Vector3 target(0.0003f, 0.0003f, 0);
    Ray glancing(target - slanted * 2, slanted);
    auto glancingHit = glancing.intersection(tiny);
    EXPECT(glancingHit.has_value());
    if (glancingHit) EXPECT(isApproximatelyEqual(glancingHit->distance, 2, 1e-4f));
}

TEST(rayTriangleHitsLieOnThePlaneWithValidBarycentrics) {
    // Aim at chosen points of a skewed triangle from several places, on both sides.
    Triangle skewed = skewedTriangle();
    const float weights[][2] = {{0.1f, 0.1f}, {0.3f, 0.2f}, {0.5f, 0.4f}, {0.05f, 0.9f}, {0.6f, 0.3f}, {0.25f, 0.25f}};
    const Vector3 offsets[] = {Vector3(0, 0, 4), Vector3(2, -1, 3), Vector3(-3, 2, -5), Vector3(1, 1, -3)};
    for (const auto& pair : weights) {
        float second = pair[0];
        float third = pair[1];
        float first = 1 - second - third;
        Vector3 target = skewed.first * first + skewed.second * second + skewed.third * third;
        for (Vector3 offset : offsets) {
            Vector3 origin = target + offset;
            Ray ray(origin, target - origin);
            auto hit = ray.intersection(skewed);
            EXPECT(hit.has_value());
            if (!hit) continue;
            EXPECT(isApproximatelyEqual(hit->distance, offset.length(), 1e-4f));
            EXPECT(isApproximatelyEqual(ray.point(hit->distance), target, 1e-4f));
            EXPECT(isApproximatelyEqual(hit->barycentric.first, first, 1e-4f));
            EXPECT(isApproximatelyEqual(hit->barycentric.second, second, 1e-4f));
            EXPECT(isApproximatelyEqual(hit->barycentric.third, third, 1e-4f));
            EXPECT(hit->barycentric.isInsideTriangle());
            float sum = hit->barycentric.first + hit->barycentric.second + hit->barycentric.third;
            EXPECT(isApproximatelyEqual(sum, 1, 1e-5f));
        }
    }
}

// MARK: Ray against a sphere

TEST(rayHitsSphereAtEntry) {
    Sphere sphereOfRadiusTwo(Vector3(0, 0, 0), 2);
    // From (0,0,−5) up: offset·direction = −5, offset² − r² = 25 − 4 = 21.
    // t² − 10t + 21 = 0 → roots 3 and 7. Entry 3.
    Ray ray(Vector3(0, 0, -5), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(sphereOfRadiusTwo), 3));
    // Off-centre sphere (1,2,3) radius 1, ray from (1,2,−3): entry at z = 2, so t = 5.
    Sphere moved(Vector3(1, 2, 3), 1);
    Ray up(Vector3(1, 2, -3), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(*up.intersectionDistance(moved), 5));
}

TEST(rayStartingInsideSphereReportsTheExit) {
    // From the centre: t² − 4 = 0 → roots −2 and 2. Exit at 2.
    Ray ray(Vector3(0, 0, 0), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(Sphere(Vector3(0, 0, 0), 2)), 2));
}

TEST(rayMissesSphereBehindOrBeside) {
    Sphere sphereOfRadiusTwo(Vector3(0, 0, 0), 2);
    // Sphere behind: roots −7 and −3.
    EXPECT(!Ray(Vector3(0, 0, 5), Vector3(0, 0, 1)).intersectionDistance(sphereOfRadiusTwo).has_value());
    // Passes 3 away from the axis, radius 2.
    EXPECT(!Ray(Vector3(3, 0, -5), Vector3(0, 0, 1)).intersectionDistance(sphereOfRadiusTwo).has_value());
}

TEST(rayGrazingSphereTouchesOnce) {
    // Origin (2,0,−5): offset·direction = −5, offset² − r² = 29 − 4 = 25.
    // t² − 10t + 25 = (t − 5)²: a double root at 5.
    Ray ray(Vector3(2, 0, -5), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(Sphere(Vector3(0, 0, 0), 2)), 5));
}

TEST(rayHitsAFarSmallSphereHeadOn) {
    // Centre (600, 480, 640) is √(360000 + 230400 + 409600) = 1000 from the origin; radius 0.1.
    // Aimed at the centre, the ray enters at 1000 − 0.1 = 999.9. Without the geometric
    // discriminant, radius² = 0.01 is lost against |offset|² = 10⁶ and this ray misses.
    Sphere far(Vector3(600, 480, 640), 0.1f);
    Ray ray(Vector3(0, 0, 0), far.centre);
    auto distance = ray.intersectionDistance(far);
    EXPECT(distance.has_value());
    if (distance) EXPECT(isApproximatelyEqual(*distance, 999.9f, 1e-2f));
}

TEST(raySphereHitPointIsOnTheSurface) {
    Sphere sphere(Vector3(1, -1, 2), 1.5f);
    const float values[] = {-5, -2, 0, 3, 6};
    for (float x : values) for (float y : values) for (float z : values) {
        Vector3 origin(x, y, z);
        Ray ray(origin, sphere.centre - origin + Vector3(0.4f, -0.2f, 0.3f));
        if (!(distanceBetween(origin, sphere.centre) > 0.1f)) continue;
        auto distance = ray.intersectionDistance(sphere);
        if (!distance) continue;
        EXPECT(isApproximatelyEqual(distanceBetween(ray.point(*distance), sphere.centre), sphere.radius, 1e-3f));
        // The first surface: from outside, no point of the ray before the hit is inside.
        if (distanceBetween(origin, sphere.centre) > sphere.radius) {
            EXPECT(distanceBetween(ray.point(*distance * 0.99f), sphere.centre) > sphere.radius);
        }
    }
}

// MARK: Ray against a box

namespace {
Bounds cube() { return Bounds::checked(Vector3(-1, -1, -1), Vector3(1, 1, 1)); }
} // namespace

TEST(rayHitsBoxAtTheNearFace) {
    // From (−5,0,0) along +x: the face x = −1 is 4 away.
    Ray straight(Vector3(-5, 0, 0), Vector3(1, 0, 0));
    EXPECT(isApproximatelyEqual(*straight.intersectionDistance(cube()), 4));
    // From (−3,−1,0) along (2,1,0): x = −1 at parameter 1, where y = 0 (inside). Distance √5.
    Ray oblique(Vector3(-3, -1, 0), Vector3(2, 1, 0));
    EXPECT(isApproximatelyEqual(*oblique.intersectionDistance(cube()), std::sqrt(5.0f)));
}

TEST(rayStartingInsideBoxReportsTheExit) {
    Ray ray(Vector3(0, 0, 0), Vector3(1, 0, 0));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(cube()), 1));
}

TEST(boxBehindOrBesideIsMissed) {
    EXPECT(!Ray(Vector3(5, 0, 0), Vector3(1, 0, 0)).intersectionDistance(cube()).has_value());
    // Zero y-component and the origin is outside the y slab: parallel, never enters.
    EXPECT(!Ray(Vector3(-5, 3, 0), Vector3(1, 0, 0)).intersectionDistance(cube()).has_value());
    // Heading toward the box's x range but passing above it.
    EXPECT(!Ray(Vector3(-5, 0, 0), Vector3(1, 1, 0)).intersectionDistance(cube()).has_value());
}

TEST(rayGrazingAFaceCountsAsAHit) {
    // Runs along the face y = 1 from x = −5: enters at x = −1, distance 4.
    Ray ray(Vector3(-5, 1, 0), Vector3(1, 0, 0));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(cube()), 4));
}

TEST(rayParallelToTwoSlabsStillHits) {
    // Two zero components (y and z): only x matters... here the ray runs along z.
    Ray ray(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0, -3));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(cube()), 1.5f));
}

TEST(rayVanishingDirectionComponentCountsAsParallel) {
    // Starting on the face y = 1, moving along +x with a y component of −1e-39 (a denormal, whose
    // reciprocal overflows to −∞). The ray stays inside the y slab, so it enters at x = −1, 4
    // away. Without the parallel rule, 0·∞ = NaN made this a miss.
    float denormal = -std::numeric_limits<float>::min() / 2;
    Ray ray(Vector3(-5, 1, 0), Vector3(1, denormal, 0));
    EXPECT(isApproximatelyEqual(*ray.intersectionDistance(cube()), 4));
    // A −0 component is parallel too.
    Ray negativeZero(Vector3(-5, 1, 0), Vector3(1, -0.0f, 0));
    EXPECT(isApproximatelyEqual(*negativeZero.intersectionDistance(cube()), 4));
}

TEST(rayAimedAtAFacePointHitsThere) {
    Bounds box = Bounds::checked(Vector3(-1, -2, -3), Vector3(1, 2, 3));
    const float across[] = {-0.8f, 0, 0.7f};
    const Vector3 skews[] = {Vector3(0, 0, 0), Vector3(1.5f, -0.5f, 0.25f), Vector3(-2, 1, 0.75f)};
    auto component = [](Vector3& vector, int axis) -> float& { return axis == 0 ? vector.x : (axis == 1 ? vector.y : vector.z); };
    for (int axis = 0; axis < 3; axis += 1) {
        for (float sign : {-1.0f, 1.0f}) {
            for (float first : across) for (float second : across) {
                // Place the target on the face: `axis` pinned to a face, the others scaled into range.
                Vector3 target(0, 0, 0);
                Vector3 maximum = box.maximum;
                Vector3 minimum = box.minimum;
                component(target, axis) = sign > 0 ? component(maximum, axis) : component(minimum, axis);
                int others[2];
                int count = 0;
                for (int candidate = 0; candidate < 3; candidate += 1) {
                    if (candidate != axis) others[count++] = candidate;
                }
                component(target, others[0]) = first * component(maximum, others[0]);
                component(target, others[1]) = second * component(maximum, others[1]);
                for (Vector3 skew : skews) {
                    Vector3 outward(0, 0, 0);
                    component(outward, axis) = sign * 4;
                    // The skew is only along the other axes, so the origin stays outside this face's half-space.
                    Vector3 tangential = skew;
                    component(tangential, axis) = 0;
                    Vector3 origin = target + outward + tangential;
                    Ray ray(origin, target - origin);
                    auto distance = ray.intersectionDistance(box);
                    EXPECT(distance.has_value());
                    if (!distance) continue;
                    EXPECT(isApproximatelyEqual(*distance, distanceBetween(origin, target), 1e-4f));
                    Vector3 hitPoint = ray.point(*distance);
                    Bounds slightlyLarger = Bounds::checked(box.minimum - Vector3(1e-3f, 1e-3f, 1e-3f),
                                                            box.maximum + Vector3(1e-3f, 1e-3f, 1e-3f));
                    EXPECT(slightlyLarger.contains(hitPoint));
                }
            }
        }
    }
}
