// The primitives, moving and joining, normals, displacement, smoothing and volume of a mesh,
// with counts and values worked out by hand. Mirrors EditorTests/Geometry/.
#include <cmath>

#include "../Linear/GeometryProblem.h"
#include "../Mesh/Mesh.h"
#include "Testing.h"

using namespace editor;

TEST(gridOfTwoDivisionsHasNinePointsAndFourQuads) {
    // 3 × 3 points, point (column, row) is row · 3 + column; the first quad is [0, 1, 4, 3].
    Mesh grid = Mesh::grid(2, 2);
    EXPECT_EQUAL(grid.positions.size(), size_t(9));
    EXPECT_EQUAL(grid.faces.size(), size_t(4));
    EXPECT(testing::isApproximatelyEqual(grid.positions[0], Vector3(-1, -1, 0)));
    EXPECT(testing::isApproximatelyEqual(grid.positions[4], Vector3(0, 0, 0)));
    EXPECT(testing::isApproximatelyEqual(grid.positions[8], Vector3(1, 1, 0)));
    EXPECT(grid.faces[0].corners == std::vector<int>({0, 1, 4, 3}));
    EXPECT(!grid.isWatertight());
}

TEST(gridRefusesWhatItCannotBuild) {
    EXPECT_PROBLEM(Mesh::grid(0, 2), "positive size");
    EXPECT_PROBLEM(Mesh::grid(1, 0), "at least 1 division");
}

TEST(uvSphereCountsAndTheOctahedron) {
    // segments · (rings − 1) + 2 points, segments · rings faces.
    Mesh ball = Mesh::uvSphere(0.5f, 32, 16);
    EXPECT_EQUAL(ball.positions.size(), size_t(482));
    EXPECT_EQUAL(ball.faces.size(), size_t(512));
    EXPECT(ball.isWatertight());
    // 4 segments and 2 rings is an octahedron: volume 4/3 · r³.
    Mesh octahedron = Mesh::uvSphere(1, 4, 2);
    EXPECT_EQUAL(octahedron.positions.size(), size_t(6));
    EXPECT_EQUAL(octahedron.faces.size(), size_t(8));
    EXPECT(testing::isApproximatelyEqual(octahedron.signedVolume(), 4.0f / 3.0f));
    EXPECT(testing::isApproximatelyEqual(octahedron.positions[0], Vector3(0, 0, 1)));
    EXPECT(testing::isApproximatelyEqual(octahedron.positions[5], Vector3(0, 0, -1)));
}

TEST(icosphereCountsFollowTenTimesFourToTheNPlusTwo) {
    for (int subdivisions = 0; subdivisions <= 3; ++subdivisions) {
        Mesh sphere = Mesh::icosphere(2, subdivisions);
        size_t triangles = 20;
        for (int round = 0; round < subdivisions; ++round) triangles *= 4;
        EXPECT_EQUAL(sphere.faces.size(), triangles);
        EXPECT_EQUAL(sphere.positions.size(), triangles / 2 + 2);
        EXPECT(sphere.isWatertight());
        for (Vector3 position : sphere.positions) EXPECT(testing::isApproximatelyEqual(position.length(), 2.0f, 1e-5f));
    }
}

TEST(icosahedronEdgesAreRadiusOverSineSeventyTwoDegrees) {
    Mesh icosahedron = Mesh::icosphere(1, 0);
    float expected = 1 / std::sin(72 * 3.14159265f / 180);
    for (const Mesh::Face& face : icosahedron.faces) {
        for (size_t corner = 0; corner < 3; ++corner) {
            Vector3 first = icosahedron.positions[face.corners[corner]];
            Vector3 second = icosahedron.positions[face.corners[(corner + 1) % 3]];
            EXPECT(testing::isApproximatelyEqual(first.distance(second), expected, 1e-5f));
        }
    }
}

TEST(icosphereVolumeApproachesTheBallFromBelow) {
    float ball = 4.0f / 3.0f * 3.14159265f;
    float coarse = Mesh::icosphere(1, 1).signedVolume();
    float fine = Mesh::icosphere(1, 3).signedVolume();
    EXPECT(coarse < fine && fine < ball);
    EXPECT(fine > 0.98f * ball);
}

TEST(icosphereRefusesTooManySubdivisions) {
    EXPECT_PROBLEM(Mesh::icosphere(1, 8), "0 to 7 subdivisions");
    EXPECT_PROBLEM(Mesh::icosphere(0, 1), "positive radius");
}

TEST(cylinderOfFourSegmentsIsASquarePrism) {
    // radius 1, height 3: the square inscribed in the unit circle has area 2, so volume 6.
    Mesh prism = Mesh::cylinder(1, 3, 4);
    EXPECT_EQUAL(prism.positions.size(), size_t(8));
    EXPECT_EQUAL(prism.faces.size(), size_t(6));
    EXPECT(prism.isWatertight());
    EXPECT(testing::isApproximatelyEqual(prism.signedVolume(), 6.0f, 1e-4f));
    EXPECT(testing::isApproximatelyEqual(prism.bounds()->minimum.z, -1.5f));
    EXPECT(testing::isApproximatelyEqual(prism.bounds()->maximum.z, 1.5f));
}

TEST(coneOfFourSegmentsIsASquarePyramid) {
    // base area 2, height 3: volume ⅓ · 2 · 3 = 2.
    Mesh pyramid = Mesh::cone(1, 3, 4);
    EXPECT_EQUAL(pyramid.positions.size(), size_t(5));
    EXPECT_EQUAL(pyramid.faces.size(), size_t(5));
    EXPECT(pyramid.isWatertight());
    EXPECT(testing::isApproximatelyEqual(pyramid.signedVolume(), 2.0f, 1e-4f));
    EXPECT(testing::isApproximatelyEqual(pyramid.positions[4], Vector3(0, 0, 1.5f)));
}

TEST(torusCountsAndVolume) {
    Mesh ring = Mesh::torus(0.5f, 0.15f, 48, 16);
    EXPECT_EQUAL(ring.positions.size(), size_t(768));
    EXPECT_EQUAL(ring.faces.size(), size_t(768));
    EXPECT(ring.isWatertight());
    // 2π² · R · r² = 2 · 9.8696 · 0.5 · 0.0225 = 0.22207, approached from below.
    float volume = ring.signedVolume();
    EXPECT(volume < 0.22207f && volume > 0.21f);
    EXPECT_PROBLEM(Mesh::torus(0.5f, 0.5f, 8, 8), "minorRadius < majorRadius");
}

TEST(transformedScalesThenRotatesThenTranslates) {
    // A 2 × 1 × 1 box scaled by 2 is 4 × 2 × 2, turned 90° about Z is 2 × 4 × 2, moved by (1, 2, 3).
    Transform placement{Vector3(1, 2, 3), Quaternion::fromAxis(Vector3::zAxis(), Angle::degrees(90)), Vector3::repeating(2)};
    Bounds bounds = *Mesh::box(Vector3(2, 1, 1)).transformed(placement).bounds();
    EXPECT(testing::isApproximatelyEqual(bounds.minimum, Vector3(0, 0, 2), 1e-5f));
    EXPECT(testing::isApproximatelyEqual(bounds.maximum, Vector3(2, 4, 4), 1e-5f));
}

TEST(aMirrorKeepsTheOutsideOutside) {
    Transform mirror{Vector3(0, 0, 0), Quaternion::identity(), Vector3(-1, 1, 1)};
    Mesh mirrored = Mesh::box(Vector3(1, 2, 3)).transformed(mirror);
    EXPECT(testing::isApproximatelyEqual(mirrored.signedVolume(), 6.0f));
    // Without the reversal the volume would be −6; the first face's corners run the other way.
    EXPECT(mirrored.faces[0].corners == std::vector<int>({5, 6, 2, 1}));
}

TEST(joinedPutsMeshesSideBySide) {
    Mesh box = Mesh::box(Vector3(1, 1, 1));
    Mesh joined = Mesh::joined({box, box, Mesh::cone(1, 1, 3)});
    EXPECT_EQUAL(joined.positions.size(), size_t(8 + 8 + 4));
    EXPECT_EQUAL(joined.faces.size(), size_t(6 + 6 + 4));
    EXPECT(joined.faces[6].corners == std::vector<int>({9, 10, 14, 13}));
    EXPECT(joined.faces[12].corners == std::vector<int>({16, 17, 19}));
    EXPECT(Mesh::joined({}).positions.empty());
}

TEST(vertexNormalOfABoxCornerPointsAlongTheDiagonal) {
    // Three faces of area 1 meet at a corner; the summed Newell vectors point along (1, 1, 1).
    std::vector<Vector3> normals = Mesh::box(Vector3(1, 1, 1)).vertexNormals();
    float component = 1 / std::sqrt(3.0f);
    EXPECT(testing::isApproximatelyEqual(normals[6], Vector3(component, component, component)));
    EXPECT(testing::isApproximatelyEqual(normals[0], Vector3(-component, -component, -component)));
}

TEST(displacedMovesPointsAlongTheirNormals) {
    Mesh ball = Mesh::uvSphere(1, 8, 4);
    Mesh grown = ball.displaced([](Vector3) { return 0.5f; });
    EXPECT_EQUAL(grown.faces.size(), ball.faces.size());
    EXPECT(testing::isApproximatelyEqual(grown.positions[0], Vector3(0, 0, 1.5f)));
    Mesh unchanged = ball.displaced([](Vector3) { return 0.0f; });
    EXPECT(unchanged == ball);
}

TEST(laplacianSmoothingPullsABumpTowardItsNeighboursAndKeepsTheBoundary) {
    Mesh sheet = Mesh::grid(2, 2);
    sheet.positions[4].z = 1;
    // The middle point has 4 edge neighbours at z = 0: 1 + (0 − 1) · 0.5 = 0.5. The 8 outer
    // points are on the boundary and stay.
    Mesh smoothed = sheet.laplacianSmoothed(1, 0.5f);
    EXPECT(testing::isApproximatelyEqual(smoothed.positions[4].z, 0.5f));
    for (int point : {0, 1, 2, 3, 5, 6, 7, 8}) EXPECT(smoothed.positions[point] == sheet.positions[point]);
    EXPECT(smoothed.faces == sheet.faces);
}

TEST(taubinSmoothingWithZeroIterationsChangesNothing) {
    Mesh ball = Mesh::icosphere(1, 2);
    EXPECT(ball.taubinSmoothed(0) == ball);
    EXPECT_PROBLEM(ball.taubinSmoothed(201), "0 to 200 iterations");
    EXPECT_PROBLEM(ball.taubinSmoothed(1, 0.5f, -0.4f), "below minus the shrink factor");
}

TEST(taubinSmoothingKeepsTheSizeWhereLaplacianShrinksIt) {
    Mesh ball = Mesh::icosphere(1, 2);
    float original = ball.signedVolume();
    float taubin = ball.taubinSmoothed(30).signedVolume();
    float laplacian = ball.laplacianSmoothed(30).signedVolume();
    EXPECT(taubin > 0.97f * original);
    EXPECT(laplacian < 0.5f * original);
}

TEST(signedVolumeOfABoxAndOfASingleTetrahedron) {
    EXPECT(testing::isApproximatelyEqual(Mesh::box(Vector3(2, 4, 6)).signedVolume(), 48.0f, 1e-4f));
    // The corner of the unit cube: (1·(1·1 − 0)) / 6 = 1/6.
    Triangle corner{Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)};
    EXPECT(testing::isApproximatelyEqual(signedTetrahedronVolume(corner), 1.0f / 6.0f));
    EXPECT_EQUAL(Mesh().signedVolume(), 0.0f);
}
