// Mesh operations, with counts worked out by hand.
#include "../Mesh/HalfEdgeMesh.h"
#include "../Mesh/Mesh.h"
#include "Testing.h"

using namespace editor;

TEST(boxHasEightCornersAndSixFaces) {
    Mesh box = Mesh::box(Vector3(2, 4, 6));
    EXPECT_EQUAL(box.positions.size(), size_t(8));
    EXPECT_EQUAL(box.faces.size(), size_t(6));
    EXPECT_EQUAL(box.triangleCount(), size_t(12));
    EXPECT(box.isWatertight());
    std::optional<Bounds> bounds = box.bounds();
    EXPECT(bounds && testing::isApproximatelyEqual(bounds->minimum, Vector3(-1, -2, -3)));
}

TEST(insetGivesEachFaceItsOwnInnerCorners) {
    Mesh box = Mesh::box(Vector3::repeating(1));
    InsetResult result = box.inset({0, 1, 2, 3, 4, 5}, 0.1f);
    // 8 corners + 4 inner corners on each of 6 faces = 32; each face becomes 1 inner + 4 rim faces.
    EXPECT_EQUAL(result.mesh.positions.size(), size_t(32));
    EXPECT_EQUAL(result.mesh.faces.size(), size_t(30));
    EXPECT_EQUAL(result.inner.size(), size_t(6));
    EXPECT(result.mesh.isWatertight());
}

TEST(extrudeAddsAWallOfSides) {
    Mesh box = Mesh::box(Vector3::repeating(1));
    ExtrudeResult result = box.extruded({4}, 0.5f);
    // The top's 4 corners are copied up; 4 side quads join them to the old ones.
    EXPECT_EQUAL(result.mesh.positions.size(), size_t(12));
    EXPECT_EQUAL(result.mesh.faces.size(), size_t(10));
    EXPECT_EQUAL(result.top.size(), size_t(1));
    EXPECT_EQUAL(result.sides.size(), size_t(4));
    EXPECT(result.mesh.isWatertight());
    EXPECT(testing::isApproximatelyEqual(result.mesh.bounds()->maximum.z, 1.0f));
}

TEST(oneSubdivisionOfABoxHasTwentyFourQuads) {
    // Catmull–Clark: 8 corners + 12 edge points + 6 face points = 26; each quad splits into 4.
    Mesh smooth = Mesh::box(Vector3::repeating(1)).subdivided(1);
    EXPECT_EQUAL(smooth.positions.size(), size_t(26));
    EXPECT_EQUAL(smooth.faces.size(), size_t(24));
    EXPECT(smooth.isWatertight());
}

TEST(aBoxHasTwelveEdgesAllBendingAtRightAngles) {
    Mesh box = Mesh::box(Vector3::repeating(1));
    HalfEdgeMesh edges(box);
    EXPECT_EQUAL(edges.edgeHalfEdges().size(), size_t(12));
    EXPECT_EQUAL(edges.edgesBendingMoreThan(Angle::degrees(45)).size(), size_t(12));
    EXPECT_EQUAL(edges.edgesBendingMoreThan(Angle::degrees(95)).size(), size_t(0));
}
