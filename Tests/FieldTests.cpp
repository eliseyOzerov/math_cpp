// Distance fields, their boxes, the combinations and modifiers, and surface nets, with values
// worked out by hand (EditorTests/Math/Fields/*.swift). The last tests compare the meshes with
// the ones the Swift engine made, bit for bit.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "../Fields/BoundedField.h"
#include "../Fields/DistanceField.h"
#include "../Fields/Ellipsoid.h"
#include "../Fields/FieldBounds.h"
#include "../Fields/FieldCombinations.h"
#include "../Fields/FieldModifiers.h"
#include "../Fields/FieldUnion.h"
#include "../Fields/NarrowBand.h"
#include "../Fields/SignedDistances.h"
#include "../Fields/SphereWrappedField.h"
#include "../Fields/SurfaceNets.h"
#include "../Fields/VectorComponents.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float infinity = std::numeric_limits<float>::infinity();
constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float pi = 0x1.921fb4p1f;

Vector3 randomPoint(SeededRandom& random, float lower, float upper) {
    float x = random.nextFloat(lower, upper);
    float y = random.nextFloat(lower, upper);
    float z = random.nextFloat(lower, upper);
    return Vector3(x, y, z);
}

using editor::testing::isApproximatelyEqual;

bool isApproximatelyEqual(const Bounds& first, const Bounds& second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.minimum, second.minimum, tolerance)
        && isApproximatelyEqual(first.maximum, second.maximum, tolerance);
}

Bounds boundsOfCorners(Vector3 minimum, Vector3 maximum) { return Bounds::checked(minimum, maximum); }

/// Two unit balls, one at the origin and one at (1, 0, 0).
Sphere leftBall() { return Sphere(Vector3(0, 0, 0), 1); }
Sphere rightBall() { return Sphere(Vector3(1, 0, 0), 1); }
Sphere unitBall() { return Sphere(Vector3(0, 0, 0), 1); }

/// The exact fields the property tests run over, about unit size around the origin.
struct ExactTestShapes {
    Sphere sphere = Sphere(Vector3(0.2f, -0.1f, 0.3f), 0.8f);
    Capsule capsule = Capsule(Segment{Vector3(-0.6f, 0.2f, 0), Vector3(0.7f, -0.3f, 0.5f)}, 0.3f);
    Bounds box = Bounds::checked(Vector3(-0.5f, -0.3f, -0.4f), Vector3(0.6f, 0.4f, 0.5f));
    RoundCone roundCone = RoundCone(Vector3(-0.5f, 0, 0.1f), Vector3(0.6f, 0.3f, -0.2f), 0.45f, 0.12f);

    std::vector<std::pair<std::string, DistanceField>> all() const {
        return {{"sphere", sphere}, {"capsule", capsule}, {"box", box}, {"roundCone", roundCone}};
    }
};

/// Whether every edge of the mesh is shared by exactly two triangles that traverse it in
/// opposite directions.
bool isClosedAndConsistentlyOriented(const TriangleMesh& mesh) {
    std::map<std::uint64_t, int> directedEdges;
    for (int index = 0; index < mesh.triangleCount(); index += 1) {
        for (int side = 0; side < 3; side += 1) {
            std::uint64_t from = mesh.triangleIndices[3 * index + side];
            std::uint64_t to = mesh.triangleIndices[3 * index + (side + 1) % 3];
            directedEdges[from << 32 | to] += 1;
        }
    }
    for (const auto& [key, count] : directedEdges) {
        std::uint64_t reverse = (key & 0xFFFFFFFFull) << 32 | key >> 32;
        auto found = directedEdges.find(reverse);
        if (count != 1 || found == directedEdges.end() || found->second != 1) return false;
    }
    return true;
}

/// FNV-1a over the bits of every position and every index: the fingerprint the Swift engine's
/// mesh was reduced to in the same way.
std::uint64_t fingerprint(const TriangleMesh& mesh) {
    std::uint64_t checksum = 1469598103934665603ull;
    for (Vector3 position : mesh.positions) {
        for (float component : {position.x, position.y, position.z}) {
            std::uint32_t bits;
            std::memcpy(&bits, &component, sizeof bits);
            checksum = (checksum ^ bits) * 1099511628211ull;
        }
    }
    for (std::uint32_t index : mesh.triangleIndices) checksum = (checksum ^ index) * 1099511628211ull;
    return checksum;
}

std::uint32_t bitsOf(float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof bits);
    return bits;
}

} // namespace

// MARK: Signed distances

TEST(fieldSphereDistanceIsDistanceToCentreMinusRadius) {
    Sphere sphere(Vector3(1, 2, 3), 2);
    EXPECT(isApproximatelyEqual(signedDistanceOf(sphere, Vector3(4, 2, 3)), 1));
    EXPECT(isApproximatelyEqual(signedDistanceOf(sphere, Vector3(1, 2, 3)), -2));
    EXPECT(isApproximatelyEqual(signedDistanceOf(sphere, Vector3(1, 2, 5)), 0));
    EXPECT(isApproximatelyEqual(signedDistanceOf(sphere, Vector3(4, 6, 3)), 3));
    EXPECT(isApproximatelyEqual(signedDistanceOf(Sphere(Vector3(0, 0, 0), 0), Vector3(3, 4, 0)), 5));
}

TEST(fieldPlaneIsAHalfSpace) {
    Plane floor(Vector3(0, 0, 1), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(signedDistanceOf(floor, Vector3(5, 5, 3)), 2));
    EXPECT(isApproximatelyEqual(signedDistanceOf(floor, Vector3(0, 0, 0)), -1));
}

TEST(fieldCapsuleDistanceIsDistanceToAxisMinusRadius) {
    Capsule capsule(Segment{Vector3(0, 0, 0), Vector3(4, 0, 0)}, 1);
    EXPECT(isApproximatelyEqual(capsule.signedDistance(Vector3(2, 3, 0)), 2));
    EXPECT(isApproximatelyEqual(capsule.signedDistance(Vector3(6, 0, 0)), 1));
    EXPECT(isApproximatelyEqual(capsule.signedDistance(Vector3(-3, 4, 0)), 4));
    EXPECT(isApproximatelyEqual(capsule.signedDistance(Vector3(2, 0, 0)), -1));
    EXPECT(isApproximatelyEqual(capsule.signedDistance(Vector3(2, 0, 1)), 0));
    // A degenerate axis is a sphere; a radius of 0 is the segment.
    Capsule ball(Segment{Vector3(1, 1, 1), Vector3(1, 1, 1)}, 2);
    Sphere sphere(Vector3(1, 1, 1), 2);
    for (Vector3 point : {Vector3(4, 1, 1), Vector3(1, 1, 1), Vector3(-1, 3, 0)}) {
        EXPECT(isApproximatelyEqual(ball.signedDistance(point), signedDistanceOf(sphere, point)));
    }
    Capsule thin(Segment{Vector3(0, 0, 0), Vector3(4, 0, 0)}, 0);
    EXPECT(isApproximatelyEqual(thin.signedDistance(Vector3(2, 3, 4)), 5));
    EXPECT_PROBLEM(Capsule(Segment{Vector3(0, 0, 0), Vector3(4, 0, 0)}, -1), "A capsule cannot have a negative radius.");
}

TEST(fieldBoxDistanceIsToFaceEdgeOrCornerAndMinusTheNearestFaceInside) {
    Bounds box = Bounds::checked(Vector3(-1, -2, -3), Vector3(1, 2, 3));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(3, 0, 0)), 2));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(2, 3, 0)), std::sqrt(2.0f)));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(2, 3, 5)), std::sqrt(6.0f)));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(-3, 0, 0)), 2));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(0.5f, 0, 0)), -0.5f));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(0, 0, 0)), -1));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(0, 0, 2.9f)), -0.1f));
    EXPECT(isApproximatelyEqual(signedDistanceOf(box, Vector3(1, 0.5f, -1)), 0));
    Bounds point = Bounds::checked(Vector3(1, 1, 1), Vector3(1, 1, 1));
    EXPECT(isApproximatelyEqual(signedDistanceOf(point, Vector3(4, 5, 1)), 5));
    Bounds square = Bounds::checked(Vector3(-1, -1, 0), Vector3(1, 1, 0));
    EXPECT(isApproximatelyEqual(signedDistanceOf(square, Vector3(0, 0, 2)), 2));
    EXPECT(isApproximatelyEqual(signedDistanceOf(square, Vector3(2, 0, 2)), std::sqrt(5.0f)));
}

TEST(fieldRoundConeHasTheHandWorkedValues) {
    // Start sphere radius 4 at the origin, end sphere radius 1 at (5,0,0): L = 5, d = 3,
    // tilt 0.6, cosine 0.8, a 3-4-5 triangle.
    RoundCone cone(Vector3(0, 0, 0), Vector3(5, 0, 0), 4, 1);
    EXPECT(isApproximatelyEqual(cone.signedDistance(Vector3(4.8f, 3.9f, 0)), 2));
    EXPECT(isApproximatelyEqual(cone.signedDistance(Vector3(4.8f, 0, 3.9f)), 2));
    EXPECT(isApproximatelyEqual(cone.signedDistance(Vector3(3.6f, 2.3f, 0)), 0));
    EXPECT(isApproximatelyEqual(cone.signedDistance(Vector3(-6, 0, 0)), 2));
    EXPECT(isApproximatelyEqual(cone.signedDistance(Vector3(8, 0, 0)), 2));
    EXPECT(isApproximatelyEqual(cone.signedDistance(Vector3(1, 0, 0)), -3.4f));
    EXPECT(isApproximatelyEqual(cone.signedDistance(Vector3(-1, 0, 0)), -3));

    // The mirror image: start radius 1, end radius 4. A point is (axial, radial, 0).
    RoundCone mirrored(Vector3(0, 0, 0), Vector3(5, 0, 0), 1, 4);
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(-0.6f, 3.3f, 0)), 2));
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(0.6f, 1.7f, 0)), 0));
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(1.2f, 0.9f, 0)), -1));
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(9, 4, 0)), std::sqrt(32.0f) - 4));
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(6, 3, 0)), std::sqrt(10.0f) - 4));
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(6, 0, 0)), -3));
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(-2, 0, 0)), 1));
    EXPECT(isApproximatelyEqual(mirrored.signedDistance(Vector3(3, 0, 0)), -2.8f));
}

TEST(fieldRoundConeSpecialCases) {
    // Equal radii: a capsule.
    RoundCone equal(Vector3(0, 0, 0), Vector3(4, 0, 0), 1, 1);
    Capsule capsule(Segment{Vector3(0, 0, 0), Vector3(4, 0, 0)}, 1);
    SeededRandom random(1);
    for (int count = 0; count < 500; count += 1) {
        Vector3 point = randomPoint(random, -4, 6);
        EXPECT(isApproximatelyEqual(equal.signedDistance(point), capsule.signedDistance(point), 1e-5f));
    }
    // One sphere swallows the other.
    RoundCone swallowed(Vector3(0, 0, 0), Vector3(1, 0, 0), 5, 1);
    EXPECT(isApproximatelyEqual(swallowed.signedDistance(Vector3(7, 0, 0)), 2));
    RoundCone reversed(Vector3(1, 0, 0), Vector3(0, 0, 0), 1, 5);
    EXPECT(isApproximatelyEqual(reversed.signedDistance(Vector3(7, 0, 0)), 2));
    RoundCone concentric(Vector3(0, 0, 0), Vector3(0, 0, 0), 2, 3);
    EXPECT(isApproximatelyEqual(concentric.signedDistance(Vector3(5, 0, 0)), 2));
    // A pointed tip.
    RoundCone pointed(Vector3(0, 0, 0), Vector3(5, 0, 0), 3, 0);
    EXPECT(isApproximatelyEqual(pointed.signedDistance(Vector3(8, 0, 0)), 3));
    EXPECT(isApproximatelyEqual(pointed.signedDistance(Vector3(5, 0, 0)), 0));
    EXPECT_PROBLEM(RoundCone(Vector3(0, 0, 0), Vector3(1, 0, 0), -1, 1), "A round cone cannot have a negative radius.");
}

TEST(fieldExactDistancesAreOneLipschitzAndGradientsHaveLengthOne) {
    SeededRandom random(7);
    for (const auto& [name, field] : ExactTestShapes().all()) {
        for (int count = 0; count < 300; count += 1) {
            Vector3 start = randomPoint(random, -1.5f, 1.5f);
            Vector3 end = randomPoint(random, -1.5f, 1.5f);
            float walked = start.distance(end);
            float change = std::fabs(field.signedDistance(end) - field.signedDistance(start));
            EXPECT(change <= walked + 1e-6f);
        }
        int checked = 0;
        while (checked < 200) {
            Vector3 point = randomPoint(random, -2, 2);
            if (!(field.signedDistance(point) > 0.1f)) continue;
            Vector3 gradient = field.gradientForFeatureSize(point, 1);
            EXPECT(std::fabs(gradient.length() - 1) < 2e-4f);
            checked += 1;
        }
    }
}

TEST(fieldGradientPointsAwayFromTheSurface) {
    DistanceField sphere = unitBall();
    EXPECT(isApproximatelyEqual(sphere.gradientForFeatureSize(Vector3(3, 0, 0), 1), Vector3(1, 0, 0), 1e-6f));
    Vector3 unitDiagonal = Vector3(1, 1, 1) / std::sqrt(3.0f);
    EXPECT(isApproximatelyEqual(sphere.surfaceNormalForFeatureSize(unitDiagonal * 2, 1), unitDiagonal, 1e-6f));
}

// MARK: Smooth minimum, maximum and the unions

TEST(fieldSmoothMinimumHandWorkedValues) {
    EXPECT(isApproximatelyEqual(smoothMinimum(1, 1, 2), 0.5f, 1e-7f));
    EXPECT(isApproximatelyEqual(smoothMinimum(1, 2, 2), 0.875f, 1e-7f));
    EXPECT(isApproximatelyEqual(smoothMinimum(2, 1, 2), 0.875f, 1e-7f));
    EXPECT(isApproximatelyEqual(smoothMinimum(1, 3, 2), 1, 1e-7f));
    EXPECT(isApproximatelyEqual(smoothMinimum(0, 1, 4), -0.5625f, 1e-7f));
    EXPECT(smoothMinimum(1, 1, 0) == 1);
    EXPECT(smoothMinimum(3, -2, -1) == -2);
    for (float blend : {1.0f, 0.0f, -1.0f}) {
        EXPECT(std::isnan(smoothMinimum(notANumber, 1, blend)));
        EXPECT(std::isnan(smoothMinimum(1, notANumber, blend)));
        EXPECT(std::isnan(smoothMinimum(-infinity, notANumber, blend)));
        EXPECT(smoothMinimum(infinity, infinity, blend) == infinity);
        EXPECT(smoothMinimum(infinity, 3, blend) == 3);
        EXPECT(smoothMinimum(-infinity, infinity, blend) == -infinity);
    }
}

TEST(fieldSmoothMaximumIsTheSmoothMinimumUpsideDown) {
    EXPECT(smoothMaximum(1, 2, 0) == 2);
    EXPECT(smoothMaximum(1, 2, -1) == 2);
    EXPECT(isApproximatelyEqual(smoothMaximum(0, 0, 1), 0.25f));
    EXPECT(isApproximatelyEqual(smoothMaximum(1, 1.5f, 1), 1.5625f));
    EXPECT(smoothMaximum(1, 3, 1) == 3);
    EXPECT(std::isnan(smoothMaximum(notANumber, 1, 1)));
    EXPECT(smoothMaximum(infinity, 1, 1) == infinity);
    EXPECT(smoothMaximum(-infinity, 1, 1) == 1);
}

TEST(fieldLargestDipIsAQuarterOfTheBlendRadiusAndAChainNeverDipsMoreThanTheRadius) {
    EXPECT(smoothMinimumLargestDip(1) == 0.25f);
    EXPECT(smoothMinimumLargestDip(0) == 0);
    EXPECT(smoothMinimumLargestDip(-2) == 0);
    EXPECT(smoothMinimumChainLargestDip(1, 0) == 0);
    EXPECT(smoothMinimumChainLargestDip(1, 1) == 0.25f);
    EXPECT(smoothMinimumChainLargestDip(1, 3) == 0.75f);
    EXPECT(smoothMinimumChainLargestDip(1, 4) == 1);
    EXPECT(smoothMinimumChainLargestDip(1, 149) == 1);
    EXPECT(smoothMinimumChainLargestDip(0.01f, 149) == 0.01f);
    EXPECT(smoothMinimumChainLargestDip(0, 5) == 0);
    EXPECT(smoothMinimumChainLargestDip(-1, 5) == 0);
    EXPECT(smoothMinimumChainLargestDip(1, -1) == 0);
    EXPECT(smoothMinimumChainLargestDip(infinity, 0) == 0);
    EXPECT(SmoothUnionOfMany({leftBall()}, infinity).bounds() == boundsOf(leftBall()));
    // The chain of equal inputs falls towards -k but never passes it.
    float chain = 0;
    std::vector<float> values;
    for (int count = 0; count < 2000; count += 1) {
        chain = smoothMinimum(chain, 0, 1);
        values.push_back(chain);
    }
    EXPECT(values[0] == -0.25f);
    EXPECT(values[1] == -0.390625f);
    for (float value : values) EXPECT(value >= -1);
    EXPECT(chain < -0.99f);
}

TEST(fieldUnionIsTheSmallerDistanceAndTheSmoothUnionDipsByAQuarter) {
    Sphere left(Vector3(-2, 0, 0), 1);
    Sphere right(Vector3(2, 0, 0), 1);
    FieldUnion twoBalls(left, right);
    EXPECT(isApproximatelyEqual(twoBalls.signedDistance(Vector3(0, 0, 0)), 1));
    EXPECT(isApproximatelyEqual(twoBalls.signedDistance(Vector3(3, 0, 0)), 0));
    SmoothUnion smooth(Sphere(Vector3(-1, 0, 0), 1), Sphere(Vector3(1, 0, 0), 1), 1);
    EXPECT(isApproximatelyEqual(smooth.signedDistance(Vector3(0, 0, 0)), -0.25f, 1e-6f));
    EXPECT(isApproximatelyEqual(smooth.signedDistance(Vector3(-1, 0, 0)), -1, 1e-6f));
    // Blend radius 0 is the plain union.
    ExactTestShapes shapes;
    FieldUnion plain(shapes.capsule, shapes.sphere);
    SmoothUnion blended(shapes.capsule, shapes.sphere, 0);
    SmoothUnion softer(shapes.capsule, shapes.sphere, 0.5f);
    SeededRandom random(5);
    for (int count = 0; count < 300; count += 1) {
        Vector3 point = randomPoint(random, -2, 2);
        EXPECT(blended.signedDistance(point) == plain.signedDistance(point));
        float lowering = plain.signedDistance(point) - softer.signedDistance(point);
        EXPECT(lowering >= 0);
        EXPECT(lowering <= 0.5f / 4 + 1e-6f);
    }
    // The boxes: the union box, grown by a quarter of the radius for the smooth union.
    EXPECT(twoBalls.bounds() == boundsOf(left)->unionWith(*boundsOf(right)));
    EXPECT(isApproximatelyEqual(*smooth.bounds(), boundsOfCorners(Vector3(-2.25f, -1.25f, -1.25f), Vector3(2.25f, 1.25f, 1.25f))));
    EXPECT(!FieldUnion(left, Plane(Vector3(0, 0, 0), Vector3(0, 0, 1))).bounds());
}

TEST(fieldSubtractionAndIntersectionHaveTheHandWorkedValues) {
    SmoothSubtraction carved(leftBall(), rightBall(), 0);
    EXPECT(isApproximatelyEqual(carved.signedDistance(Vector3(-2, 0, 0)), 1));
    EXPECT(isApproximatelyEqual(carved.signedDistance(Vector3(-0.5f, 0, 0)), -0.5f));
    EXPECT(isApproximatelyEqual(carved.signedDistance(Vector3(0.5f, 0, 0)), 0.5f));
    EXPECT(carved.bounds() == boundsOf(leftBall()));

    SmoothIntersection lens(leftBall(), rightBall(), 0);
    EXPECT(isApproximatelyEqual(lens.signedDistance(Vector3(0.5f, 0, 0)), -0.5f));
    EXPECT(isApproximatelyEqual(lens.signedDistance(Vector3(-2, 0, 0)), 2));
    EXPECT(isApproximatelyEqual(*lens.bounds(), boundsOfCorners(Vector3(0, -1, -1), Vector3(1, 1, 1))));

    // The blend only removes from the hard result, and by no more than k/4.
    SmoothSubtraction softCarve(leftBall(), rightBall(), 0.4f);
    SmoothIntersection softLens(leftBall(), rightBall(), 0.4f);
    SeededRandom random(41);
    for (int count = 0; count < 500; count += 1) {
        Vector3 point = randomPoint(random, -2, 3);
        EXPECT(softCarve.signedDistance(point) >= carved.signedDistance(point));
        EXPECT(softLens.signedDistance(point) >= lens.signedDistance(point));
        EXPECT(softLens.signedDistance(point) - lens.signedDistance(point) <= 0.1f + 1e-6f);
    }

    // Two balls that do not meet intersect in nothing: the box is one point.
    SmoothIntersection empty(leftBall(), Sphere(Vector3(5, 0, 0), 1), 0.3f);
    EXPECT(*empty.bounds() == boundsOfCorners(Vector3(0, 0, 0), Vector3(0, 0, 0)));
    for (int count = 0; count < 500; count += 1) EXPECT(empty.signedDistance(randomPoint(random, -2, 7)) > 0);

    // An unbounded part leaves the other box.
    Plane floor(Vector3(0, 0, 0), Vector3(0, 0, 1));
    EXPECT(SmoothIntersection(leftBall(), floor, 0.2f).bounds() == boundsOf(leftBall()));
    EXPECT(SmoothIntersection(floor, leftBall(), 0.2f).bounds() == boundsOf(leftBall()));
    EXPECT(!SmoothIntersection(floor, floor, 0.2f).bounds());
    EXPECT(!SmoothSubtraction(floor, leftBall(), 0.2f).bounds());
}

TEST(fieldSmoothUnionOfManyBlendsInOrderAndSkippingChangesNothing) {
    SmoothUnionOfMany single({leftBall()}, 0.5f);
    SmoothUnionOfMany pair({leftBall(), rightBall()}, 0.5f);
    SmoothUnion smoothPair(leftBall(), rightBall(), 0.5f);
    SeededRandom random(43);
    for (int count = 0; count < 300; count += 1) {
        Vector3 point = randomPoint(random, -2, 3);
        EXPECT(single.signedDistance(point) == signedDistanceOf(leftBall(), point));
        EXPECT(pair.signedDistance(point) == smoothPair.signedDistance(point));
    }
    std::vector<Sphere> parts = {leftBall(), rightBall(), Sphere(Vector3(2, 0.3f, 0), 0.8f), Sphere(Vector3(8, 0, 0), 0.5f)};
    std::vector<DistanceField> fields(parts.begin(), parts.end());
    SmoothUnionOfMany union_(fields, 0.6f);
    for (int count = 0; count < 500; count += 1) {
        Vector3 point = randomPoint(random, -2, 9);
        float chain = signedDistanceOf(parts[0], point);
        for (std::size_t index = 1; index < parts.size(); index += 1) {
            chain = smoothMinimum(chain, signedDistanceOf(parts[index], point), 0.6f);
        }
        EXPECT(union_.signedDistance(point) == chain);
    }
    // The box grows by the chain's largest dip: three parts, k = 0.4, 2 × 0.4/4 = 0.2.
    SmoothUnionOfMany three({leftBall(), rightBall(), Sphere(Vector3(0, 3, 0), 0.5f)}, 0.4f);
    EXPECT(isApproximatelyEqual(*three.bounds(), boundsOfCorners(Vector3(-1.2f, -1.2f, -1.2f), Vector3(2.2f, 3.7f, 1.2f))));
    EXPECT(!SmoothUnionOfMany({leftBall(), Plane(Vector3(0, 0, 0), Vector3(0, 0, 1))}, 0.4f).bounds());
    EXPECT_PROBLEM(SmoothUnionOfMany({}, 1), "A smooth union needs at least one part.");
}

// MARK: Boxes

TEST(fieldTheExactShapesHaveTheirTightBoxes) {
    EXPECT(*boundsOf(Sphere(Vector3(1, 2, 3), 0.5f)) == boundsOfCorners(Vector3(0.5f, 1.5f, 2.5f), Vector3(1.5f, 2.5f, 3.5f)));
    Capsule capsule(Segment{Vector3(0, 0, 0), Vector3(2, 1, 0)}, 0.5f);
    EXPECT(*capsule.bounds() == boundsOfCorners(Vector3(-0.5f, -0.5f, -0.5f), Vector3(2.5f, 1.5f, 0.5f)));
    RoundCone cone(Vector3(0, 0, 0), Vector3(3, 0, 0), 1, 0.25f);
    EXPECT(*cone.bounds() == boundsOfCorners(Vector3(-1, -1, -1), Vector3(4, 1, 1)));
    EXPECT(!boundsOf(Plane(Vector3(0, 0, 0), Vector3(0, 0, 1))));
    Bounds box = boundsOfCorners(Vector3(-1, 0, 0), Vector3(1, 1, 1));
    EXPECT(*boundsOf(box) == box);
    EXPECT(!DistanceFieldFunction([](Vector3 point) { return point.length() - 1; }).bounds());
}

TEST(fieldGrowingOverlappingAndEnclosingBoxes) {
    Bounds unit = boundsOfCorners(Vector3(0, 0, 0), Vector3(1, 1, 1));
    EXPECT(expanded(unit, 0.5f) == boundsOfCorners(Vector3(-0.5f, -0.5f, -0.5f), Vector3(1.5f, 1.5f, 1.5f)));
    EXPECT(expanded(unit, 0) == unit);
    EXPECT_PROBLEM(expanded(unit, -1), "A box can only be grown by a margin of 0 or more.");
    EXPECT_PROBLEM(expanded(unit, notANumber), "A box can only be grown by a margin of 0 or more.");
    Bounds shifted = boundsOfCorners(Vector3(0.5f, -1, 0.25f), Vector3(2, 0.5f, 0.75f));
    EXPECT(*intersection(unit, shifted) == boundsOfCorners(Vector3(0.5f, 0, 0.25f), Vector3(1, 0.5f, 0.75f)));
    Bounds touching = boundsOfCorners(Vector3(1, 0, 0), Vector3(2, 1, 1));
    EXPECT(*intersection(unit, touching) == boundsOfCorners(Vector3(1, 0, 0), Vector3(1, 1, 1)));
    EXPECT(!intersection(unit, boundsOfCorners(Vector3(3, 0, 0), Vector3(4, 1, 1))));
    EXPECT(*enclosing(unit, touching) == boundsOfCorners(Vector3(0, 0, 0), Vector3(2, 1, 1)));
    EXPECT(!enclosing(unit, std::nullopt));
    EXPECT(!enclosing(std::nullopt, unit));
}

TEST(fieldATurnedBoxGetsTheBoxAroundItsCorners) {
    Bounds box = boundsOfCorners(Vector3(0, 0, 0), Vector3(2, 1, 1));
    Transform turn;
    turn.rotation = Quaternion::fromAxis(Vector3::zAxis(), Angle::degrees(90));
    EXPECT(isApproximatelyEqual(transformed(box, turn), boundsOfCorners(Vector3(-1, 0, 0), Vector3(0, 2, 1)), 1e-6f));
    Transform move;
    move.translation = Vector3(10, 0, 0);
    move.scale = Vector3(2, 2, 2);
    EXPECT(isApproximatelyEqual(transformed(box, move), boundsOfCorners(Vector3(10, 0, 0), Vector3(14, 2, 2))));
    Bounds square = boundsOfCorners(Vector3(-0.5f, -0.5f, 0), Vector3(0.5f, 0.5f, 1));
    Transform diamond;
    diamond.rotation = Quaternion::fromAxis(Vector3::zAxis(), Angle::degrees(45));
    EXPECT(isApproximatelyEqual(transformed(square, diamond),
                                boundsOfCorners(Vector3(-0.70711f, -0.70711f, 0), Vector3(0.70711f, 0.70711f, 1)), 1e-5f));
    // Across the plane x = 1: x becomes 2 − x, so 0…0.5 becomes 1.5…2.
    Bounds half = boundsOfCorners(Vector3(0, 0, 0), Vector3(0.5f, 1, 1));
    Plane plane(Vector3(1, 0, 0), Vector3(1, 0, 0));
    EXPECT(isApproximatelyEqual(reflected(half, plane), boundsOfCorners(Vector3(1.5f, 0, 0), Vector3(2, 1, 1))));
}

TEST(fieldRaisingTakesTheLargerAndKeepsNotANumber) {
    Bounds box = boundsOfCorners(Vector3(0, 0, 0), Vector3(1, 1, 1));
    EXPECT(isApproximatelyEqual(raising(box, 0.5f, Vector3(3, 0.5f, 0.5f)), 2));
    EXPECT(raising(box, 5, Vector3(3, 0.5f, 0.5f)) == 5);
    EXPECT(isApproximatelyEqual(raising(box, -2, Vector3(0.5f, 0.5f, 0.5f)), -0.5f));
    EXPECT(std::isnan(raising(box, notANumber, Vector3(3, 0, 0))));
    EXPECT(std::isnan(raising(box, 1, Vector3(notANumber, 0, 0))));
}

TEST(fieldAPlaneReflectsAPointToTheOtherSide) {
    Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(reflection(plane, Vector3(2, 3, 4)), Vector3(2, 3, -2)));
    EXPECT(isApproximatelyEqual(reflection(plane, Vector3(2, 3, 1)), Vector3(2, 3, 1)));
    Plane tilted(Vector3(0.3f, 0, 0), Vector3(1, 0.2f, -0.5f));
    SeededRandom random(61);
    for (int count = 0; count < 100; count += 1) {
        Vector3 point = randomPoint(random, -5, 5);
        EXPECT(isApproximatelyEqual(reflection(tilted, reflection(tilted, point)), point, 1e-5f));
        EXPECT(isApproximatelyEqual(tilted.signedDistance(reflection(tilted, point)), -tilted.signedDistance(point), 1e-5f));
    }
}

TEST(fieldABoxIsCutToTheFrontOfAnAxisAlignedPlane) {
    Bounds box = boundsOfCorners(Vector3(-1.5f, -1, -1), Vector3(0.5f, 1, 1));
    EXPECT(frontPart(box, Plane(Vector3(0, 0, 0), Vector3(1, 0, 0))) == boundsOfCorners(Vector3(0, -1, -1), Vector3(0.5f, 1, 1)));
    EXPECT(frontPart(box, Plane(Vector3(0, 0, 0), Vector3(-1, 0, 0))) == boundsOfCorners(Vector3(-1.5f, -1, -1), Vector3(0, 1, 1)));
    EXPECT(frontPart(box, Plane(Vector3(0, 0, 0.25f), Vector3(0, 0, 1))) == boundsOfCorners(Vector3(-1.5f, -1, 0.25f), Vector3(0.5f, 1, 1)));
    EXPECT(frontPart(box, Plane(Vector3(0, 0, 0), Vector3(1, 1, 0))) == box);
    EXPECT(frontPart(box, Plane(Vector3(2, 0, 0), Vector3(1, 0, 0))) == box);
}

TEST(fieldVectorComponents) {
    EXPECT(smallestComponent(Vector3(3, -1, 2)) == -1);
    EXPECT(smallestComponent(Vector3(0.5f, 0.5f, 0.25f)) == 0.25f);
    EXPECT(largestComponent(Vector3(3, -1, 2)) == 3);
    EXPECT(componentwiseMaximum(Vector3(-1, 2, 0), 0) == Vector3(0, 2, 0));
    EXPECT(componentwiseAbsolute(Vector3(-1, 2, -0.5f)) == Vector3(1, 2, 0.5f));
}

// MARK: BoundedField

TEST(fieldABoundedFieldAgreesInSignAndInValueInsideItsBox) {
    Ellipsoid inner(Vector3(0.1f, -0.2f, 0.3f), Vector3(0.9f, 0.4f, 0.25f),
                    Quaternion::fromAxis(Vector3(1, 2, 3).normalized(), Angle::degrees(50)));
    BoundedField bounded(inner);
    Bounds box = *inner.bounds();
    SeededRandom random(62);
    for (int count = 0; count < 2000; count += 1) {
        Vector3 point = randomPoint(random, -3, 3);
        float value = bounded.signedDistance(point);
        float innerValue = inner.signedDistance(point);
        EXPECT((value < 0) == (innerValue < 0));
        if (signedDistanceOf(box, point) > 0) EXPECT(value == signedDistanceOf(box, point));
        else EXPECT(value == innerValue);
    }
    BoundedField floor{DistanceField(Plane(Vector3(0, 0, 0), Vector3(0, 0, 1)))};
    EXPECT(!floor.bounds());
    EXPECT(floor.signedDistance(Vector3(4, 5, 6)) == 6);
}

TEST(fieldABoundedFieldMustNotBeUnderAnOffsetShellOrSmoothBlend) {
    Sphere ball = unitBall();
    Vector3 corner(1.2f, 1.2f, 1.2f);
    EXPECT(isApproximatelyEqual(OffsetField(ball, 0.5f).signedDistance(corner), 0.578f, 1e-3f));
    EXPECT(isApproximatelyEqual(OffsetField(BoundedField(ball), 0.5f).signedDistance(corner), -0.154f, 1e-3f));
    EXPECT(ShellField(BoundedField(ball), 1).signedDistance(corner) < 0);
    EXPECT(ShellField(ball, 1).signedDistance(corner) > 0);
    Sphere left(Vector3(-1, 0, 0), 1);
    Sphere right(Vector3(1, 0, 0), 1);
    Vector3 point(0, 1.1f, 1.1f);
    EXPECT(SmoothUnion(left, right, 1).signedDistance(point) > 0.5f);
    EXPECT(SmoothUnion(BoundedField(left), BoundedField(right), 1).signedDistance(point) < 0);
}

// MARK: Ellipsoid

TEST(fieldEllipsoidHasTheHandWorkedValues) {
    Ellipsoid ellipsoid(Vector3(0, 0, 0), Vector3(3, 2, 1));
    EXPECT(isApproximatelyEqual(ellipsoid.signedDistance(Vector3(5, 0, 0)), 2));
    EXPECT(isApproximatelyEqual(ellipsoid.signedDistance(Vector3(0, -4, 0)), 2));
    EXPECT(isApproximatelyEqual(ellipsoid.signedDistance(Vector3(0, 0, 3)), 2));
    EXPECT(isApproximatelyEqual(ellipsoid.signedDistance(Vector3(0, 0, 0)), -1));
    EXPECT(isApproximatelyEqual(ellipsoid.signedDistance(Vector3(0, 0, 0.5f)), -0.5f));
    EXPECT(isApproximatelyEqual(ellipsoid.signedDistance(Vector3(3, 0, 0)), 0));
    EXPECT(isApproximatelyEqual(ellipsoid.signedDistance(Vector3(2, 2, 0)), 0.20185f, 1e-5f));
    EXPECT(std::isnan(ellipsoid.signedDistance(Vector3(notANumber, 0, 0))));
    EXPECT_PROBLEM(Ellipsoid(Vector3(0, 0, 0), Vector3(1, 0, 1)), "An ellipsoid needs three positive, finite radii.");
}

TEST(fieldEllipsoidWithEqualRadiiIsASphereAndATurnedOneMeasuresAlongItsAxes) {
    Ellipsoid ball(Vector3(1, 2, 3), Vector3(2, 2, 2));
    Sphere sphere(Vector3(1, 2, 3), 2);
    SeededRandom random(21);
    for (int count = 0; count < 200; count += 1) {
        Vector3 point = randomPoint(random, -4, 6);
        EXPECT(isApproximatelyEqual(ball.signedDistance(point), signedDistanceOf(sphere, point), 2e-6f));
    }
    Ellipsoid turned(Vector3(0, 0, 0), Vector3(3, 2, 1), Quaternion::fromAxis(Vector3::zAxis(), Angle::degrees(90)));
    EXPECT(isApproximatelyEqual(turned.signedDistance(Vector3(4, 0, 0)), 2, 2e-6f));
    EXPECT(isApproximatelyEqual(turned.signedDistance(Vector3(0, 5, 0)), 2, 2e-6f));
    EXPECT(isApproximatelyEqual(*turned.bounds(), boundsOfCorners(Vector3(-2, -3, -1), Vector3(2, 3, 1)), 2e-6f));
    Ellipsoid slanted(Vector3(1, 0, 0), Vector3(2, 1, 1), Quaternion::fromAxis(Vector3::zAxis(), Angle::degrees(45)));
    EXPECT(isApproximatelyEqual(*slanted.bounds(),
                                boundsOfCorners(Vector3(1 - 1.58114f, -1.58114f, -1), Vector3(1 + 1.58114f, 1.58114f, 1)), 1e-5f));
}

// MARK: Modifiers

TEST(fieldMirrorTransformOffsetAndShell) {
    // A ball of radius 0.5 at (1, 0, 0), mirrored across x = 0.
    Sphere ball(Vector3(1, 0, 0), 0.5f);
    MirroredField pair(ball, Plane(Vector3(0, 0, 0), Vector3(1, 0, 0)));
    EXPECT(isApproximatelyEqual(pair.signedDistance(Vector3(1, 0, 0)), -0.5f));
    EXPECT(isApproximatelyEqual(pair.signedDistance(Vector3(-1, 0, 0)), -0.5f));
    EXPECT(isApproximatelyEqual(pair.signedDistance(Vector3(-3, 0, 0)), 1.5f));
    EXPECT(isApproximatelyEqual(*pair.bounds(), boundsOfCorners(Vector3(-1.5f, -0.5f, -0.5f), Vector3(1.5f, 0.5f, 0.5f))));
    // The dropped back part does not widen the box.
    MirroredField cut(Sphere(Vector3(-0.5f, 0, 0), 1), Plane(Vector3(0, 0, 0), Vector3(1, 0, 0)));
    EXPECT(isApproximatelyEqual(*cut.bounds(), boundsOfCorners(Vector3(-0.5f, -1, -1), Vector3(0.5f, 1, 1))));
    EXPECT(isApproximatelyEqual(cut.signedDistance(Vector3(0.6f, 0, 0)), 0.1f, 1e-5f));

    // The unit ball scaled by 2 and moved to (5, 0, 0).
    Transform placement;
    placement.translation = Vector3(5, 0, 0);
    placement.scale = Vector3(2, 2, 2);
    TransformedField moved(unitBall(), placement);
    EXPECT(isApproximatelyEqual(moved.signedDistance(Vector3(5, 0, 0)), -2));
    EXPECT(isApproximatelyEqual(moved.signedDistance(Vector3(10, 0, 0)), 3));
    EXPECT(isApproximatelyEqual(*moved.bounds(), boundsOfCorners(Vector3(3, -2, -2), Vector3(7, 2, 2))));
    Transform stretch;
    stretch.scale = Vector3(1, 3, 1);
    TransformedField stretched(unitBall(), stretch);
    EXPECT(isApproximatelyEqual(stretched.signedDistance(Vector3(0, 0, 0)), -1));
    EXPECT(isApproximatelyEqual(stretched.signedDistance(Vector3(0, 5, 0)), 2));
    Transform flat;
    flat.scale = Vector3(1, 0, 1);
    EXPECT_PROBLEM(TransformedField(unitBall(), flat), "A transformed field needs finite, non-zero scale factors.");

    OffsetField grown(unitBall(), 0.5f);
    EXPECT(isApproximatelyEqual(grown.signedDistance(Vector3(0, 0, 0)), -1.5f));
    EXPECT(isApproximatelyEqual(grown.signedDistance(Vector3(3, 0, 0)), 1.5f));
    EXPECT(isApproximatelyEqual(*grown.bounds(), boundsOfCorners(Vector3(-1.5f, -1.5f, -1.5f), Vector3(1.5f, 1.5f, 1.5f))));
    OffsetField shrunk(unitBall(), -0.25f);
    EXPECT(isApproximatelyEqual(shrunk.signedDistance(Vector3(1, 0, 0)), 0.25f));
    EXPECT(shrunk.bounds() == boundsOf(unitBall()));
    EXPECT_PROBLEM(OffsetField(unitBall(), infinity), "An offset needs a finite amount.");

    ShellField shell(unitBall(), 0.2f);
    EXPECT(isApproximatelyEqual(shell.signedDistance(Vector3(0, 0, 0)), 0.9f));
    EXPECT(isApproximatelyEqual(shell.signedDistance(Vector3(1, 0, 0)), -0.1f));
    EXPECT(isApproximatelyEqual(shell.signedDistance(Vector3(0, 2, 0)), 0.9f));
    EXPECT(isApproximatelyEqual(*shell.bounds(), boundsOfCorners(Vector3(-1.1f, -1.1f, -1.1f), Vector3(1.1f, 1.1f, 1.1f))));
    EXPECT_PROBLEM(ShellField(unitBall(), -1), "A shell needs a finite thickness of 0 or more.");
}

TEST(fieldDisplacementKeepsTheSurfaceWithinTheAmplitude) {
    DisplacedField still(unitBall(), GradientNoise(3), 0, 4);
    SeededRandom random(53);
    for (int count = 0; count < 200; count += 1) {
        Vector3 point = randomPoint(random, -2, 2);
        EXPECT(still.signedDistance(point) == signedDistanceOf(unitBall(), point));
    }
    DisplacedField bumpy(unitBall(), GradientNoise(9), 0.1f, 3);
    DistanceField field = bumpy;
    // Points where the field changes sign, found by walking out from the centre along rays.
    int found = 0;
    for (int ray = 0; ray < 500; ray += 1) {
        Vector3 direction = random.nextUnitVector3();
        float previous = field.signedDistance(direction * 0.5f);
        for (int step = 1; step <= 100; step += 1) {
            Vector3 point = direction * (0.5f + 0.01f * static_cast<float>(step));
            float value = field.signedDistance(point);
            if (previous < 0 && value >= 0) {
                EXPECT(std::fabs(signedDistanceOf(unitBall(), point)) <= 0.1f * gradientNoiseLargestValue + 0.011f);
                found += 1;
            }
            previous = value;
        }
    }
    EXPECT(found > 400);
    EXPECT_PROBLEM(DisplacedField(unitBall(), GradientNoise(1), notANumber, 1), "A displacement needs a finite amplitude.");
    EXPECT_PROBLEM(DisplacedField(unitBall(), GradientNoise(1), 1, -1), "A displacement needs a finite frequency of 0 or more.");
}

TEST(fieldNoiseNeverChangesFasterThanItsSlopeBound) {
    SeededRandom random(54);
    float steepest = 0;
    float largest = 0;
    for (int seed = 0; seed < 20; seed += 1) {
        GradientNoise noise(static_cast<std::uint64_t>(seed));
        for (int count = 0; count < 2000; count += 1) {
            Vector3 point = randomPoint(random, -8, 8);
            Vector3 step = randomPoint(random, -1e-2f, 1e-2f);
            if (!(step.length() > 1e-3f)) continue;
            float value = noise.value(point);
            steepest = std::max(steepest, std::fabs(noise.value(point + step) - value) / step.length());
            largest = std::max(largest, std::fabs(value));
        }
    }
    EXPECT(steepest < gradientNoiseSlopeBound);
    EXPECT(largest <= gradientNoiseLargestValue);
}

// MARK: Sphere wrap

namespace {
Bounds plate() { return boundsOfCorners(Vector3(-0.5f, -0.3f, -0.05f), Vector3(0.5f, 0.3f, 0.05f)); }
} // namespace

TEST(fieldSphereWrapMapsAreInverseAndHaveTheHandWorkedValues) {
    SphereWrappedField wrapped(plate(), 1);
    EXPECT(isApproximatelyEqual(wrapped.wrappedPoint(Vector3(0, 0, 0)), Vector3(0, 0, 0)));
    EXPECT(isApproximatelyEqual(wrapped.flatPoint(Vector3(0, 0, 0)), Vector3(0, 0, 0)));
    EXPECT(isApproximatelyEqual(wrapped.wrappedPoint(Vector3(0, 0, 0.5f)), Vector3(0, 0, 0.5f)));
    EXPECT(isApproximatelyEqual(wrapped.flatPoint(Vector3(0, 0, -0.5f)), Vector3(0, 0, -0.5f)));
    EXPECT(isApproximatelyEqual(wrapped.wrappedPoint(Vector3(pi / 2, 0, 0)), Vector3(1, 0, -1)));
    EXPECT(isApproximatelyEqual(wrapped.flatPoint(Vector3(1, 0, -1)), Vector3(pi / 2, 0, 0)));
    EXPECT(isApproximatelyEqual(wrapped.wrappedPoint(Vector3(0, pi / 2, 0.5f)), Vector3(0, 1.5f, -1)));
    SeededRandom random(9);
    for (int count = 0; count < 200; count += 1) {
        float x = random.nextFloat(-1.5f, 1.5f);
        float y = random.nextFloat(-1.5f, 1.5f);
        float z = random.nextFloat(-0.5f, 0.5f);
        Vector3 flat(x, y, z);
        EXPECT(isApproximatelyEqual(wrapped.flatPoint(wrapped.wrappedPoint(flat)), flat, 1e-4f));
    }
    // The guard region by hand (see SphereWrappedFieldTests.swift).
    EXPECT(isApproximatelyEqual(wrapped.guardRadius, 0.7125f));
    EXPECT(isApproximatelyEqual(wrapped.guardAngle.radians, 1.08310f, 1e-4f));
    EXPECT(isApproximatelyEqual(wrapped.stretch, 1.72073f, 1e-4f));
    EXPECT(isApproximatelyEqual(wrapped.margin, 0.13802f, 1e-4f));
    EXPECT(isApproximatelyEqual(wrapped.signedDistance(wrapped.wrappedPoint(Vector3(0.55f, 0, 0))), 0.029058f, 1e-5f));
    float above = wrapped.signedDistance(Vector3(0, 0, 0.5f));
    EXPECT(above > 0 && above <= 0.45f + 1e-6f);
}

TEST(fieldSphereWrapStretchAndCapBoxes) {
    EXPECT(isApproximatelyEqual(SphereWrappedField::largestStretch(1, 0.5f, Angle::degrees(90)), pi));
    EXPECT(isApproximatelyEqual(SphereWrappedField::largestStretch(1, 0.5f, Angle::fromRadians(0)), 2));
    EXPECT(SphereWrappedField::largestStretch(1, 2, Angle::fromRadians(0)) == 1);
    Bounds box = boundsOfCorners(Vector3(-1, -3, -5), Vector3(2, 1, 5));
    EXPECT(isApproximatelyEqual(SphereWrappedField::widestDistanceFromAxis(box), std::sqrt(13.0f)));
    Bounds narrow = SphereWrappedField::boundsOfCap(Vector3(0, 0, 0), 1, 2, Angle::degrees(60));
    EXPECT(isApproximatelyEqual(narrow, boundsOfCorners(Vector3(-std::sqrt(3.0f), -std::sqrt(3.0f), 0.5f),
                                                        Vector3(std::sqrt(3.0f), std::sqrt(3.0f), 2))));
    Bounds wide = SphereWrappedField::boundsOfCap(Vector3(0, 0, 0), 1, 2, Angle::degrees(120));
    EXPECT(isApproximatelyEqual(wide, boundsOfCorners(Vector3(-2, -2, -1), Vector3(2, 2, 2))));
    EXPECT_PROBLEM(SphereWrappedField(plate(), 0), "A sphere wrap needs a positive radius.");
    EXPECT_PROBLEM(SphereWrappedField(Plane(Vector3(0, 0, 0), Vector3(0, 0, 1)), 1), "A sphere wrap needs a bounded inner field.");
}

// MARK: Surface nets

TEST(fieldPlaneThroughATwoByTwoByTwoGridMakesOneQuad) {
    Bounds unitBox = boundsOfCorners(Vector3(0, 0, 0), Vector3(2, 2, 2));
    DistanceField plane = DistanceFieldFunction([](Vector3 point) { return point.x - 0.25f; });
    TriangleMesh mesh = plane.extractSurface(unitBox, 1);
    EXPECT_EQUAL(mesh.positions.size(), 4u);
    std::set<std::array<float, 3>> expected = {{0.25f, 0.5f, 0.5f}, {0.25f, 1.5f, 0.5f}, {0.25f, 0.5f, 1.5f}, {0.25f, 1.5f, 1.5f}};
    std::set<std::array<float, 3>> actual;
    for (Vector3 position : mesh.positions) actual.insert({position.x, position.y, position.z});
    EXPECT(actual == expected);
    EXPECT_EQUAL(mesh.triangleCount(), 2);
    float area = 0;
    for (int index = 0; index < mesh.triangleCount(); index += 1) {
        EXPECT(isApproximatelyEqual(mesh.triangle(index).normal(), Vector3(1, 0, 0)));
        area += mesh.triangle(index).area();
    }
    EXPECT(isApproximatelyEqual(area, 1));

    // The winding is right along each axis, for inside on either side.
    std::vector<std::pair<DistanceField, Vector3>> cases = {
        {DistanceFieldFunction([](Vector3 point) { return point.y - 0.25f; }), Vector3(0, 1, 0)},
        {DistanceFieldFunction([](Vector3 point) { return point.z - 0.25f; }), Vector3(0, 0, 1)},
        {DistanceFieldFunction([](Vector3 point) { return 0.25f - point.x; }), Vector3(-1, 0, 0)},
        {DistanceFieldFunction([](Vector3 point) { return 1.25f - point.y; }), Vector3(0, -1, 0)},
        {DistanceFieldFunction([](Vector3 point) { return 1.25f - point.z; }), Vector3(0, 0, -1)},
    };
    for (const auto& [field, normal] : cases) {
        TriangleMesh oriented = field.extractSurface(unitBox, 1);
        EXPECT_EQUAL(oriented.triangleCount(), 2);
        for (int index = 0; index < oriented.triangleCount(); index += 1) {
            EXPECT(isApproximatelyEqual(oriented.triangle(index).normal(), normal));
        }
    }
}

TEST(fieldWithoutASurfaceOrWithNotANumberCornersMakesNoNanVertices) {
    Bounds unitBox = boundsOfCorners(Vector3(0, 0, 0), Vector3(2, 2, 2));
    TriangleMesh none = DistanceField(DistanceFieldFunction([](Vector3) { return 1.0f; })).extractSurface(unitBox, 0.5f);
    EXPECT(none.positions.empty() && none.triangleIndices.empty());
    EXPECT_EQUAL(DistanceField(DistanceFieldFunction([](Vector3) { return notANumber; })).extractSurface(unitBox, 0.5f).triangleCount(), 0);
    DistanceField withHole = DistanceFieldFunction([](Vector3 point) { return point == Vector3(0, 1, 1) ? notANumber : point.x - 0.25f; });
    TriangleMesh mesh = withHole.extractSurface(unitBox, 1);
    for (Vector3 position : mesh.positions) EXPECT(position.isFinite());
    EXPECT_EQUAL(mesh.positions.size(), 4u);
    EXPECT_EQUAL(mesh.triangleCount(), 0);
    EXPECT_PROBLEM(DistanceField(unitBall()).extractSurface(unitBox, 0), "The cell size must be positive.");
}

TEST(fieldCellCountsRoundUpUnlessTheRatioIsWholeWithinRounding) {
    using Counts = SurfaceNetsGrid::CellCounts;
    auto counts = [](float size, float cellSize) { return SurfaceNetsGrid::cellCountsForSize({size, size, size}, cellSize); };
    EXPECT((counts(2.5f, 1) == Counts{3, 3, 3}));
    EXPECT((counts(3, 1) == Counts{3, 3, 3}));
    EXPECT((counts(0.5f, 1) == Counts{1, 1, 1}));
    EXPECT((counts(0, 1) == Counts{1, 1, 1}));
    EXPECT((counts(3.05f, 0.1f) == Counts{31, 31, 31}));
    EXPECT((SurfaceNetsGrid::cellCountsForSize({2, 3, 5}, 1) == Counts{2, 3, 5}));
    for (float scale : {1e-3f, 1e-2f, 0.37f, 1.0f, 7.3f, 100.0f, 1000.0f, 12345.0f}) {
        EXPECT((SurfaceNetsGrid::cellCountsForSize({3 * scale, 3 * scale, 3 * scale}, 0.1f * scale) == Counts{30, 30, 30}));
    }
    EXPECT_PROBLEM(SurfaceNetsGrid::cellCountsForSize({infinity, 1, 1}, 1), "must be finite");
    EXPECT_PROBLEM(SurfaceNetsGrid::cellCountsForSize({1000, 1000, 1000}, 1), "the limit is 16777216");
}

namespace {

const Vector3 sphereCentre(0.13f, -0.07f, 0.21f);

TriangleMesh sphereMesh(float scale = 1, Vector3 offset = Vector3(0, 0, 0)) {
    DistanceField sphere = Sphere(sphereCentre * scale + offset, 1 * scale);
    return sphere.extractSurface(
        boundsOfCorners(Vector3(-1.5f, -1.5f, -1.5f) * scale + offset, Vector3(1.5f, 1.5f, 1.5f) * scale + offset), 0.1f * scale);
}

} // namespace

TEST(fieldSphereMeshIsClosedOutwardAndAboutTheSphereVolume) {
    TriangleMesh mesh = sphereMesh();
    EXPECT(mesh.triangleCount() > 1000);
    EXPECT(isClosedAndConsistentlyOriented(mesh));
    float worst = 0;
    for (Vector3 position : mesh.positions) worst = std::max(worst, std::fabs(position.distance(sphereCentre) - 1));
    EXPECT(worst < 6e-3f);
    float smallestAgreement = 1;
    for (int index = 0; index < mesh.triangleCount(); index += 1) {
        Triangle triangle = mesh.triangle(index);
        if (triangle.isDegenerate()) continue;
        smallestAgreement = std::min(smallestAgreement, triangle.normal().dot((triangle.centroid() - sphereCentre).normalized()));
    }
    EXPECT(smallestAgreement > 0.99f);
    float exact = 4.0f / 3.0f * pi;
    EXPECT(std::fabs(mesh.enclosedVolume() - exact) / exact < 0.015f);
    // The same grid at another scale gives the same triangles.
    for (float scale : {1e-3f, 1000.0f}) EXPECT_EQUAL(sphereMesh(scale).triangleCount(), mesh.triangleCount());
}

// MARK: Narrow band

TEST(fieldNarrowBandBound) {
    EXPECT(isApproximatelyEqual(NarrowBand::cubeHalfDiagonal(2), 1.7320508f));
    EXPECT(isApproximatelyEqual(NarrowBand::cubeHalfDiagonal(0.5f), 0.4330127f));
    EXPECT(NarrowBand::cubeHalfDiagonal(0) == 0);
    EXPECT(NarrowBand::surfaceCannotReach(1.85f, 2));
    EXPECT(NarrowBand::surfaceCannotReach(-1.85f, 2));
    EXPECT(!NarrowBand::surfaceCannotReach(1.8f, 2));
    EXPECT(!NarrowBand::surfaceCannotReach(-1.75f, 2));
    EXPECT(!NarrowBand::surfaceCannotReach(0, 2));
    EXPECT(!NarrowBand::surfaceCannotReach(notANumber, 2));
    EXPECT(NarrowBand::surfaceCannotReach(infinity, 2));
    EXPECT(NarrowBand::surfaceCannotReach(-infinity, 2));
    EXPECT(NarrowBand::roundingIsSmall(65536, 1));
    EXPECT(!NarrowBand::roundingIsSmall(65537, 1));
    EXPECT(!NarrowBand::roundingIsSmall(notANumber, 1));
    EXPECT(!NarrowBand::roundingIsSmall(infinity, 1));
}

TEST(fieldNarrowBandMeshesTheSameAsTheFullGrid) {
    ExactTestShapes shapes;
    Transform stretch;
    stretch.translation = Vector3(0.1f, 0, 0);
    stretch.rotation = Quaternion::fromAxis(Vector3(0, 0, 1), Angle::degrees(40));
    stretch.scale = Vector3(3, 0.4f, 1);
    Ellipsoid ellipsoid(Vector3(0, 0, 0), Vector3(0.9f, 0.3f, 0.15f), Quaternion::fromAxis(Vector3(1, 1, 0).normalized(), Angle::degrees(30)));
    std::vector<std::pair<std::string, DistanceField>> fields = shapes.all();
    fields.push_back({"ellipsoid", ellipsoid});
    fields.push_back({"union", SmoothUnionOfMany({shapes.sphere, shapes.capsule, shapes.roundCone}, 0.2f)});
    fields.push_back({"subtraction", SmoothSubtraction(shapes.box, shapes.sphere, 0.15f)});
    fields.push_back({"intersection", SmoothIntersection(shapes.box, shapes.sphere, 0.15f)});
    fields.push_back({"offset", OffsetField(shapes.capsule, 0.1f)});
    fields.push_back({"shell", ShellField(shapes.sphere, 0.05f)});
    fields.push_back({"mirrored", MirroredField(shapes.roundCone, Plane(Vector3(0, 0, 0), Vector3(1, 0, 0)))});
    fields.push_back({"transformed", TransformedField(shapes.capsule, stretch)});
    fields.push_back({"displaced", DisplacedField(shapes.sphere, GradientNoise(7), 0.08f, 6)});
    fields.push_back({"displaced rough", DisplacedField(ellipsoid, GradientNoise(3), 0.05f, 20)});
    fields.push_back({"bounded", BoundedField(shapes.sphere)});
    fields.push_back({"wrapped", SphereWrappedField(boundsOfCorners(Vector3(-0.3f, -0.2f, -0.025f), Vector3(0.3f, 0.2f, 0.025f)), 0.6f)});
    for (const auto& [name, field] : fields) {
        Bounds box = field.paddedMeshingBox(0.04f);
        TriangleMesh narrow = field.extractSurface(box, 0.04f);
        EXPECT(narrow.triangleCount() > 0);
        EXPECT(narrow == field.extractSurfaceSamplingEveryCorner(box, 0.04f));
        if (!(narrow == field.extractSurfaceSamplingEveryCorner(box, 0.04f))) std::cerr << "  field: " << name << "\n";
    }
    // 37 x 23 x 51 cells: no axis is a whole number of blocks.
    DistanceField sphere = Sphere(Vector3(0.1f, 0.05f, 0.2f), 0.4f);
    Bounds odd = boundsOfCorners(Vector3(-0.74f, -0.46f, -0.82f), Vector3(0.74f, 0.46f, 1.22f));
    TriangleMesh narrow = sphere.extractSurface(odd, 0.04f);
    EXPECT(narrow.triangleCount() > 0);
    EXPECT(narrow == sphere.extractSurfaceSamplingEveryCorner(odd, 0.04f));
}

TEST(fieldMeshingCellCountAndPaddedBox) {
    DistanceField sphere = unitBall();
    EXPECT(isApproximatelyEqual(sphere.paddedMeshingBox(0.5f), boundsOfCorners(Vector3(-2, -2, -2), Vector3(2, 2, 2))));
    EXPECT_EQUAL(sphere.meshingCellCount(0.5f), 512);
    EXPECT_PROBLEM(DistanceField(Plane(Vector3(0, 0, 0), Vector3(0, 0, 1))).paddedMeshingBox(1), "Meshing a field needs a bounded field.");
}

// MARK: The Swift engine's meshes, bit for bit

TEST(fieldSphereMeshIsTheSwiftMeshBitForBit) {
    // Printed by the Swift engine for Sphere(centre (0.13, -0.07, 0.21), radius 1) meshed in
    // ±1.5 with cells of 0.1.
    TriangleMesh mesh = sphereMesh();
    EXPECT_EQUAL(mesh.positions.size(), 1886u);
    EXPECT_EQUAL(mesh.triangleCount(), 3768);
    EXPECT_EQUAL(bitsOf(mesh.positions[0].x), 3185131685u);
    EXPECT_EQUAL(bitsOf(mesh.positions[0].y), 3201243868u);
    EXPECT_EQUAL(bitsOf(mesh.positions[0].z), 3207834403u);
    EXPECT_EQUAL(bitsOf(mesh.positions[628].x), 3208009492u);
    EXPECT_EQUAL(bitsOf(mesh.positions[628].y), 1051096920u);
    EXPECT_EQUAL(bitsOf(mesh.positions[628].z), 3189348760u);
    EXPECT_EQUAL(bitsOf(mesh.positions[1885].x), 1046165207u);
    EXPECT_EQUAL(bitsOf(mesh.positions[1885].y), 1013358705u);
    EXPECT_EQUAL(bitsOf(mesh.positions[1885].z), 1067045159u);
    std::vector<std::uint32_t> firstSix = {7, 8, 0, 8, 1, 0};
    EXPECT(std::vector<std::uint32_t>(mesh.triangleIndices.begin(), mesh.triangleIndices.begin() + 6) == firstSix);
    std::vector<std::uint32_t> lastThree = {1881, 1885, 1884};
    EXPECT(std::vector<std::uint32_t>(mesh.triangleIndices.end() - 3, mesh.triangleIndices.end()) == lastThree);
    EXPECT_EQUAL(fingerprint(mesh), 2187304766781072081ull);
}

namespace {

Capsule firstCapsule() { return Capsule(Segment{Vector3(-0.6f, 0.2f, 0), Vector3(0.7f, -0.3f, 0.5f)}, 0.3f); }
Capsule secondCapsule() { return Capsule(Segment{Vector3(0, -0.5f, -0.4f), Vector3(0.3f, 0.6f, 0.4f)}, 0.25f); }

} // namespace

TEST(fieldSmoothUnionOfTwoCapsulesIsTheSwiftMeshBitForBit) {
    DistanceField blend = SmoothUnion(firstCapsule(), secondCapsule(), 0.2f);
    Bounds box = blend.paddedMeshingBox(0.04f);
    EXPECT_EQUAL(bitsOf(box.minimum.x), 3213088523u);
    EXPECT_EQUAL(bitsOf(box.maximum.z), 1064178811u);
    EXPECT_EQUAL(blend.meshingCellCount(0.04f), 111155);
    TriangleMesh mesh = blend.extractSurface(box, 0.04f);
    EXPECT_EQUAL(mesh.positions.size(), 5596u);
    EXPECT_EQUAL(mesh.triangleCount(), 11188);
    EXPECT_EQUAL(bitsOf(mesh.positions[0].x), 3173430851u);
    EXPECT_EQUAL(bitsOf(mesh.positions[0].y), 3206202430u);
    EXPECT_EQUAL(bitsOf(mesh.positions[0].z), 3206501084u);
    EXPECT_EQUAL(bitsOf(mesh.positions[1865].x), 3195943493u);
    EXPECT_EQUAL(bitsOf(mesh.positions[1865].y), 1052006594u);
    EXPECT_EQUAL(bitsOf(mesh.positions[1865].z), 3173943551u);
    EXPECT_EQUAL(bitsOf(mesh.positions[5595].x), 1060862869u);
    EXPECT_EQUAL(bitsOf(mesh.positions[5595].y), 3192637380u);
    EXPECT_EQUAL(bitsOf(mesh.positions[5595].z), 1061669152u);
    EXPECT_EQUAL(fingerprint(mesh), 2511943982099726452ull);

    // The Taubin-smoothed polygon mesh of it (4 rounds).
    Mesh smoothed = mesh.mesh().taubinSmoothed(4);
    EXPECT_EQUAL(smoothed.positions.size(), 5596u);
    EXPECT_EQUAL(smoothed.faces.size(), 11188u);
    EXPECT_EQUAL(bitsOf(smoothed.positions[0].x), 3172494695u);
    EXPECT_EQUAL(bitsOf(smoothed.positions[0].y), 3206197768u);
    EXPECT_EQUAL(bitsOf(smoothed.positions[0].z), 3206526272u);
    std::uint64_t checksum = 1469598103934665603ull;
    for (Vector3 position : smoothed.positions) {
        for (float component : {position.x, position.y, position.z}) checksum = (checksum ^ bitsOf(component)) * 1099511628211ull;
    }
    EXPECT_EQUAL(checksum, 220680372800029477ull);
    EXPECT_EQUAL(blend.meshedSurface(0.04f, 4).positions.size(), 5596u);
}

TEST(fieldSmoothUnionOfManyIsTheSwiftMeshBitForBit) {
    Sphere ball(Vector3(0.13f, -0.07f, 0.21f), 1);
    DistanceField many = SmoothUnionOfMany({firstCapsule(), secondCapsule(), ball}, 0.15f);
    TriangleMesh mesh = many.extractSurface(many.paddedMeshingBox(0.05f), 0.05f);
    EXPECT_EQUAL(mesh.positions.size(), 7610u);
    EXPECT_EQUAL(mesh.triangleCount(), 15216);
    EXPECT_EQUAL(bitsOf(mesh.positions[0].x), 1014439747u);
    EXPECT_EQUAL(bitsOf(mesh.positions[2536].y), 3203908183u);
    EXPECT_EQUAL(bitsOf(mesh.positions[7609].z), 1066919007u);
    EXPECT_EQUAL(fingerprint(mesh), 14157677239860049930ull);
}
