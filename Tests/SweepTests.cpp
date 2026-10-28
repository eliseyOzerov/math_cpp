// Revolve and sweep (Mesh+Revolve.swift, Mesh+Sweep.swift): counts, volumes and refusals, worked
// out by hand as in EditorTests/Geometry/.
#include <cmath>

#include "../Mesh/Mesh.h"
#include "../Curves/CatmullRomSpline.h"
#include "../Curves/CubicBezier.h"
#include "Testing.h"

using namespace editor;

namespace {

const std::vector<Vector2> unitSquare = {Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)};

/// A straight cubic Bézier from the origin to (0, 0, length).
CubicBezier<Vector3> straightUp(float length) {
    return CubicBezier<Vector3>(Vector3(0, 0, 0), Vector3(0, 0, length / 3), Vector3(0, 0, length * 2 / 3), Vector3(0, 0, length));
}

float constantScale(float) { return 1; }

} // namespace

TEST(droppingRepeatedCornersTurnsQuadsIntoTriangles) {
    EXPECT(((Mesh::Face::droppingRepeatedCorners({7, 8, 3, 3})->corners) == std::vector<int>{7, 8, 3}));
    EXPECT(!Mesh::Face::droppingRepeatedCorners({3, 3, 9, 9}).has_value());
    // The ring wraps: corner 0 equal to the last is dropped.
    EXPECT(((Mesh::Face::droppingRepeatedCorners({3, 4, 5, 3})->corners) == std::vector<int>{4, 5, 3}));
    // A corner that comes back later, not next to itself, stays.
    EXPECT(((Mesh::Face::droppingRepeatedCorners({1, 2, 1, 3})->corners) == std::vector<int>{1, 2, 1, 3}));
}

TEST(revolveOfTheUnitSquareIsAPrismWithPoles) {
    // 2 axis points + 2 off-axis rings of 8 = 18 points; the bottom and top rows are 8 triangles each, the side 8 quads.
    Mesh prism = Mesh::revolve(unitSquare, 8);
    EXPECT_EQUAL(prism.positions.size(), size_t(18));
    EXPECT_EQUAL(prism.faces.size(), size_t(24));
    EXPECT(prism.isClosed());
    EXPECT(prism.isValid());
    // Volume of a regular 8-gon prism of circumradius 1 and height 1: (8/2) · sin(2π/8).
    EXPECT(testing::isApproximatelyEqual(prism.signedVolume(), 4 * std::sin(2 * 3.14159265f / 8), 1e-4f));
    EXPECT(testing::isApproximatelyEqual(prism.positions[1], Vector3(1, 0, 0)));
    EXPECT(testing::isApproximatelyEqual(prism.positions[2], Vector3(std::sqrt(0.5f), std::sqrt(0.5f), 0), 1e-6f));
}

TEST(revolveOfAPartialTurnLeavesTheSidesOpen) {
    // 3 segments over 90° need 4 columns: 2 axis points + 2 · 4 = 10 points, 3 rows · 3 = 9 faces.
    Mesh wedge = Mesh::revolve(unitSquare, 3, Angle::degrees(90));
    EXPECT_EQUAL(wedge.positions.size(), size_t(10));
    EXPECT_EQUAL(wedge.faces.size(), size_t(9));
    EXPECT(!wedge.isClosed());
    EXPECT(testing::isApproximatelyEqual(wedge.positions[8], Vector3(0, 1, 1), 1e-6f));
}

TEST(revolveRefusesWhatSwiftsPreconditionsRefuse) {
    EXPECT_PROBLEM(Mesh::revolve({Vector2(0, 0)}, 8), "A revolved profile needs 2...1024 points, but has 1.");
    EXPECT_PROBLEM(Mesh::revolve({Vector2(-1, 0), Vector2(1, 1)}, 8), "A revolved profile needs finite points with a radius of 0 or more.");
    EXPECT_PROBLEM(Mesh::revolve(unitSquare, 2), "A full revolve needs at least 3 segments, a partial one at least 1.");
    EXPECT_PROBLEM(Mesh::revolve(unitSquare, 8, Angle::degrees(0)), "A revolve sweeps more than 0° and at most 360°.");
    EXPECT_PROBLEM(Mesh::revolve(unitSquare, 2000), "A revolve makes at most 1024 segments, but got 2000.");
}

TEST(circleProfileRunsCounterclockwiseFromTheXAxis) {
    std::vector<Vector2> ring = Mesh::circleProfile(2, 4);
    EXPECT_EQUAL(ring.size(), size_t(4));
    EXPECT(std::fabs(ring[0].x - 2) < 1e-6f && std::fabs(ring[0].y) < 1e-6f);
    EXPECT(std::fabs(ring[1].x) < 1e-6f && std::fabs(ring[1].y - 2) < 1e-6f);
    EXPECT(std::fabs(ring[2].x + 2) < 1e-6f);
    EXPECT_PROBLEM(Mesh::circleProfile(0, 8), "A circle profile needs a finite positive radius.");
    EXPECT_PROBLEM(Mesh::circleProfile(1, 2), "A circle profile needs 3...1024 segments, but got 2.");
}

TEST(sweepOfACircleAlongAStraightCurveIsAPrism) {
    // 8 corners of circumradius 0.5, 5 rings: 40 points, 8 · 4 quads and 2 caps; volume (8/2) · 0.25 · sin(2π/8) · 1.
    Mesh prism = Mesh::sweep(Mesh::circleProfile(0.5f, 8), straightUp(1), 5, constantScale, true);
    EXPECT_EQUAL(prism.positions.size(), size_t(40));
    EXPECT_EQUAL(prism.faces.size(), size_t(34));
    EXPECT(prism.isClosed());
    EXPECT(testing::isApproximatelyEqual(prism.signedVolume(), 4 * 0.25f * std::sin(2 * 3.14159265f / 8), 1e-4f));
}

TEST(sweepWithoutCapsLeavesTheEndsOpen) {
    Mesh open = Mesh::sweep(Mesh::circleProfile(0.5f, 8), straightUp(1), 5, constantScale, false);
    EXPECT_EQUAL(open.faces.size(), size_t(32));
    EXPECT(!open.isClosed());
}

TEST(sweepWithAZeroScaleEndMakesATip) {
    // Scale 1 − fraction: the last ring is one point. 4 full rings of 8 + 1 tip = 33 points; 32 side faces
    // (the last 8 are triangles) and the cap at the start.
    Mesh horn = Mesh::sweep(Mesh::circleProfile(0.5f, 8), straightUp(1), 5, [](float fraction) { return 1 - fraction; }, true);
    EXPECT_EQUAL(horn.positions.size(), size_t(33));
    EXPECT_EQUAL(horn.faces.size(), size_t(33));
    EXPECT(horn.isClosed());
    EXPECT(testing::isApproximatelyEqual(horn.positions[32], Vector3(0, 0, 1), 1e-5f));
}

TEST(tubeIsASweepOfTheUnitCircleScaledByTheRadius) {
    Mesh tube = Mesh::tube(straightUp(2), [](float) { return 0.25f; }, 8, 3);
    Mesh swept = Mesh::sweep(Mesh::circleProfile(1, 8), straightUp(2), 3, [](float) { return 0.25f; }, true);
    EXPECT(tube == swept);
    EXPECT_EQUAL(tube.positions.size(), size_t(24));
}

TEST(sweepAlongASplineFollowsItsPoints) {
    CatmullRomSpline spline({Vector3(0, 0, 0), Vector3(0, 0, 1), Vector3(0, 0, 2)});
    Mesh tube = Mesh::sweep(Mesh::circleProfile(1, 6), spline, 3, constantScale, true);
    EXPECT(tube.isClosed());
    // Every ring is a circle of radius 1 about the Z axis, at heights 0, 1 and 2.
    for (int ring = 0; ring < 3; ++ring) {
        for (int corner = 0; corner < 6; ++corner) {
            Vector3 point = tube.positions[ring * 6 + corner];
            EXPECT(testing::isApproximatelyEqual(std::sqrt(point.x * point.x + point.y * point.y), 1, 1e-5f));
            EXPECT(testing::isApproximatelyEqual(point.z, static_cast<float>(ring), 1e-5f));
        }
    }
}

TEST(sweepRefusesWhatSwiftsPreconditionsRefuse) {
    EXPECT_PROBLEM(Mesh::sweep(Mesh::circleProfile(1, 8), straightUp(1), 1, constantScale, true), "A sweep needs 2...1024 samples, but got 1.");
    EXPECT_PROBLEM(Mesh::sweep({Vector2(0, 0), Vector2(1, 0)}, straightUp(1), 4, constantScale, true),
                   "A sweep profile needs 3...1024 points, but has 2.");
    EXPECT_PROBLEM(Mesh::sweep(Mesh::circleProfile(1, 8), straightUp(1), 4, [](float) { return -1.0f; }, true),
                   "A sweep's scale must be finite and not negative.");
}
