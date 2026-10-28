// Tests of Curves/Warp2D. Golden values are transcribed from graphics_cpp tests/golden/warp.json
// (affine m = [1.1, 0.2, -0.3, 0.9, 5, -7], mesh, scatter, wavelength, meshGrid); the rest are worked by hand.
#include <cmath>
#include <vector>

#include "../Curves/Warp2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// The golden affine map: x' = 1.1x − 0.3y + 5, y' = 0.2x + 0.9y − 7.
AffineTransform2 goldenAffine() { return AffineTransform2{Vector2(1.1f, 0.2f), Vector2(-0.3f, 0.9f), Vector2(5, -7)}; }

} // namespace

// MARK: Derivative and nodes

TEST(jacobianOfAnAffineMapIsItsMatrix) {
    // golden jacobianAt: [[1.1, 0.2], [-0.3, 0.9]] at (50, 50).
    std::array<Vector2, 2> derivative = Warp2D::jacobian(Warp2D::affine(goldenAffine()), Vector2(50, 50));
    EXPECT(isClose(derivative[0], Vector2(1.1f, 0.2f), 5e-3f) && isClose(derivative[1], Vector2(-0.3f, 0.9f), 5e-3f));
    EXPECT_PROBLEM(Warp2D::jacobian(Warp2D::affine(goldenAffine()), Vector2(0, 0), 0), "above 0");
    // A quadratic map x' = x², y' = y: the derivative at x = 3 is 6 (central differences are exact on quadratics).
    Warp2D::PlaneMap square = [](Vector2 point) { return Vector2(point.x * point.x, point.y); };
    std::array<Vector2, 2> squared = Warp2D::jacobian(square, Vector2(3, 1), 0.1f);
    EXPECT(isClose(squared[0], Vector2(6, 0), 1e-2f) && isClose(squared[1], Vector2(0, 1), 1e-2f));
}

TEST(aWarpedNodeMovesItsPointAndCarriesItsHandles) {
    // golden warpNodeAt: point (10, 20) → (10, 13). Handles carry by the matrix: (1, 0) → (1.1, 0.2), (0, 1) → (-0.3, 0.9).
    WarpNode node{Vector2(10, 20), Vector2(1, 0), Vector2(0, 1)};
    WarpNode warped = Warp2D::warpedNode(node, Warp2D::affine(goldenAffine()));
    EXPECT(isClose(warped.point, Vector2(10, 13)));
    EXPECT(warped.incoming && isClose(*warped.incoming, Vector2(1.1f, 0.2f), 5e-3f));
    EXPECT(warped.outgoing && isClose(*warped.outgoing, Vector2(-0.3f, 0.9f), 5e-3f));
    // An end without handles stays without.
    EXPECT(!Warp2D::warpedNode(WarpNode{Vector2(1, 1), std::nullopt, std::nullopt}, Warp2D::affine(goldenAffine())).incoming.has_value());
}

// MARK: Cubics, paths, shapes

TEST(anAffineWarpOfACubicIsExact) {
    // golden warpCubicAt: [[0,0],[0,100],[100,100],[100,0]] → [[5,-7],[-25,83],[85,103],[115,13]].
    CubicBezier<Vector2> cubic(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0));
    std::vector<CubicBezier<Vector2>> warped = Warp2D::warpedCubic(cubic, Warp2D::affine(goldenAffine()));
    EXPECT(warped.size() == 1);
    EXPECT(isClose(warped[0].start, Vector2(5, -7)) && isClose(warped[0].end, Vector2(115, 13)));
    EXPECT(isClose(warped[0].startHandle, Vector2(-25, 83), 0.1f) && isClose(warped[0].endHandle, Vector2(85, 103), 0.1f));
    // Property: in pieces the warped curve still passes through the mapped points of the original.
    WarpSettings pieces;
    pieces.pieces = 4;
    std::vector<CubicBezier<Vector2>> four = Warp2D::warpedCubic(cubic, Warp2D::affine(goldenAffine()), pieces);
    EXPECT(four.size() == 4);
    EXPECT(isClose(four[1].end, goldenAffine().applied(cubic.point(0.5f)), 1e-2f));
    EXPECT(Warp2D::warpedCubics({}, Warp2D::affine(goldenAffine())).empty());
}

TEST(aBowKeepsAnEvenlyRisingStrokeExact) {
    // A stroke up the y axis from (0,0) to (0,100) under spine(bow 10): x(u) = 40·u·(1 − u). The handles carry by the
    // slope ±0.4 at the ends to (13.33, 33.33) and (13.33, 66.67), so the middle is
    // 3/8·13.33 + 3/8·13.33 = 10 across at height 50, exactly the bow.
    CubicBezier<Vector2> stroke = Path2D::evenlyPacedCubic(Vector2(0, 0), Vector2(0, 100));
    std::vector<CubicBezier<Vector2>> bent = Warp2D::warpedCubic(stroke, Warp2D::spine(10, 0, 100));
    EXPECT(bent.size() == 1);
    EXPECT(isClose(bent[0].point(0.5f), Vector2(10, 50), 5e-2f));
    EXPECT(isClose(bent[0].startHandle, Vector2(13.333f, 33.333f), 5e-2f));
    // Every point of the warped curve lies on the bowed line x = 40u(1 − u), u = y/100.
    for (float parameter : {0.1f, 0.3f, 0.7f, 0.9f}) {
        Vector2 point = bent[0].point(parameter);
        EXPECT(isApproximatelyEqual(point.x, 4 * 10 * (point.y / 100) * (1 - point.y / 100), 0.1f));
    }
}

TEST(stepCutsACubicByItsControlPolygonLength) {
    CubicBezier<Vector2> cubic(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0));
    // The control polygon is 100 + 100 + 100 = 300 long; a step of 100 gives three pieces, a step of 0 means one.
    WarpSettings byStep;
    byStep.step = 100;
    EXPECT(Warp2D::warpedCubic(cubic, Warp2D::affine(goldenAffine()), byStep).size() == 3);
    byStep.step = 0;
    EXPECT(Warp2D::warpedCubic(cubic, Warp2D::affine(goldenAffine()), byStep).size() == 1);
    // `pieces` wins when both are given, and is held at 1 or more.
    WarpSettings both;
    both.pieces = 2;
    both.step = 100;
    EXPECT(Warp2D::warpedCubic(cubic, Warp2D::affine(goldenAffine()), both).size() == 2);
    both.pieces = -3;
    EXPECT(Warp2D::warpedCubic(cubic, Warp2D::affine(goldenAffine()), both).size() == 1);
}

TEST(aWarpedPathAgreesWithMappingItWhenTheWarpIsAffine) {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(50, 0));
    path.addQuarter(Vector2(80, 30), Heading::horizontal);
    path.addLine(Vector2(0, 30));
    path.close();
    AffineTransform2 transform = goldenAffine();
    Path2D warped = Warp2D::warpedPath(path, Warp2D::affine(transform));
    Path2D mapped = path.mapped([&](Vector2 point) { return transform.applied(point); });
    // The implied closing line becomes a piece of its own, so the warped path has one piece more.
    EXPECT(warped.isClosed() && warped.segments().size() == mapped.segments().size() + 1);
    for (size_t index = 0; index < mapped.segments().size(); index += 1) {
        EXPECT(isClose(warped.segments()[index].end(), mapped.segments()[index].end(), 1e-2f));
    }
    // The curve piece's handles agree too, and areas scale by the determinant 1.1·0.9 + 0.3·0.2 = 1.05.
    EXPECT(isClose(warped.segments()[1].startHandle, mapped.segments()[1].startHandle, 0.1f));
    EXPECT(isApproximatelyEqual(warped.signedArea(), path.signedArea() * 1.05f, 1.0f));
    // Node marks survive when no piece was cut.
    Path2D marked = path;
    marked.setNodeKind(1, PathNodeKind::smooth);
    EXPECT(Warp2D::warpedPath(marked, Warp2D::affine(transform)).nodeKind(1) == PathNodeKind::smooth);
    // A lone start point is only mapped.
    EXPECT(isClose(Warp2D::warpedPath(Path2D(Vector2(10, 20)), Warp2D::affine(transform)).start, Vector2(10, 13)));
}

TEST(aWarpedShapeKeepsItsFillRuleAndOutlines) {
    Shape2D shape;
    shape.fillRule = FillRule::evenOdd;
    Path2D outer(Vector2(0, 0));
    outer.addLine(Vector2(10, 0));
    outer.addLine(Vector2(10, 10));
    outer.close();
    shape.outlines = {outer, outer};
    Shape2D warped = Warp2D::warpedShape(shape, Warp2D::affine(AffineTransform2::translationBy(Vector2(5, 5))));
    EXPECT(warped.fillRule == FillRule::evenOdd && warped.outlines.size() == 2);
    EXPECT(isClose(warped.outlines[0].start, Vector2(5, 5)) && warped.outlines[0].isClosed());
}

// MARK: Mesh, spine, pinned

TEST(meshGridListsNinePointsFromTheBottom) {
    // golden meshGrid [0,0]–[100,200].
    std::vector<Vector2> grid = Warp2D::meshGrid(Vector2(0, 0), Vector2(100, 200));
    std::vector<Vector2> expected{{0, 0}, {50, 0}, {100, 0}, {0, 100}, {50, 100}, {100, 100}, {0, 200}, {50, 200}, {100, 200}};
    EXPECT(grid.size() == 9);
    for (size_t index = 0; index < 9; index += 1) EXPECT(isClose(grid[index], expected[index], 1e-4f));
}

TEST(aMeshAtRestIsTheIdentityEvenOutsideItsBox) {
    // golden warpAt (mesh at rest): points map to themselves, including (-30, 250) and (130, -20).
    Warp2D::PlaneMap rest = Warp2D::mesh(Vector2(0, 0), Vector2(100, 200), Warp2D::meshGrid(Vector2(0, 0), Vector2(100, 200)));
    for (Vector2 point : {Vector2(0, 0), Vector2(50, 100), Vector2(25, 150), Vector2(-30, 250), Vector2(130, -20), Vector2(73.5f, 41.2f)}) {
        EXPECT(isClose(rest(point), point, 1e-3f));
    }
    EXPECT_PROBLEM(Warp2D::mesh(Vector2(0, 0), Vector2(1, 1), {}), "nine points");
}

TEST(aMeshWithAMovedGridBendsThroughTheNinePoints) {
    // Lifting the middle point by 20 lifts the middle of the box by 20 and leaves the corners and edges' ends alone.
    std::vector<Vector2> grid = Warp2D::meshGrid(Vector2(0, 0), Vector2(100, 200));
    grid[4] = grid[4] + Vector2(0, 20);
    Warp2D::PlaneMap lifted = Warp2D::mesh(Vector2(0, 0), Vector2(100, 200), grid);
    EXPECT(isClose(lifted(Vector2(50, 100)), Vector2(50, 120)));
    EXPECT(isClose(lifted(Vector2(0, 0)), Vector2(0, 0)) && isClose(lifted(Vector2(100, 200)), Vector2(100, 200)));
    // Property: moving every point by the same offset moves every plane point by it.
    for (Vector2& point : grid) point = Warp2D::meshGrid(Vector2(0, 0), Vector2(100, 200))[0] + (point - Vector2(0, 0)) + Vector2(5, 7);
    Warp2D::PlaneMap shifted = Warp2D::mesh(Vector2(0, 0), Vector2(100, 200), Warp2D::meshGrid(Vector2(5, 7), Vector2(105, 207)));
    EXPECT(isClose(shifted(Vector2(30, 60)), Vector2(35, 67), 1e-3f));
}

TEST(aSpineBowsAcrossAndCanFollowAnyWave) {
    // 4·bow·u·(1 − u): at u = 0.25, 0.5, 1 with bow 10 → 7.5, 10, 0; the y never moves.
    Warp2D::PlaneMap bow = Warp2D::spine(10, 0, 100);
    EXPECT(isClose(bow(Vector2(1, 25)), Vector2(8.5f, 25)) && isClose(bow(Vector2(0, 50)), Vector2(10, 50)));
    EXPECT(isClose(bow(Vector2(3, 100)), Vector2(3, 100)) && isClose(bow(Vector2(3, 0)), Vector2(3, 0)));
    Warp2D::PlaneMap lean = Warp2D::spine([](float share) { return 2 * share; }, 0, 100);
    EXPECT(isClose(lean(Vector2(0, 50)), Vector2(1, 50)));
    // Equal heights count as one unit apart.
    EXPECT(isClose(Warp2D::spine([](float share) { return share; }, 5, 5)(Vector2(0, 6)), Vector2(1, 6)));
}

TEST(pinnedFadesTheVerticalPushNearItsLines) {
    Warp2D::PlaneMap lift = [](Vector2 point) { return Vector2(point.x + 1, point.y + 10); };
    Warp2D::PlaneMap held = Warp2D::pinned(lift, {0}, 10);
    // On the line: no push up, but the sideways move stays. At half the reach: smoothStep(0.5) = 0.5. Past it: all.
    EXPECT(isClose(held(Vector2(0, 0)), Vector2(1, 0)));
    EXPECT(isClose(held(Vector2(0, 5)), Vector2(1, 10)));
    EXPECT(isClose(held(Vector2(0, 40)), Vector2(1, 50)));
    // No reach pins nothing.
    EXPECT(isClose(Warp2D::pinned(lift, {0}, 0)(Vector2(0, 0)), Vector2(1, 10)));
}

// MARK: Wobble and noise

TEST(wavelengthAndStepFollowTheFrequency) {
    // golden wavelength 125 and wobbleStep 62.5 at the default frequency 8 and one octave.
    Wobble wobble;
    EXPECT(isApproximatelyEqual(Warp2D::wavelength(wobble), 125));
    EXPECT(isApproximatelyEqual(Warp2D::wobbleStep(wobble), 62.5f));
    wobble.octaves = 3;
    EXPECT(isApproximatelyEqual(Warp2D::wobbleStep(wobble), 125.0f / 2 / 4));
    wobble.frequency = 0;
    EXPECT(std::isfinite(Warp2D::wavelength(wobble)));
}

TEST(noiseIsSmoothSeededAndZeroOnTheLattice) {
    std::function<float(float, float)> noise = Warp2D::noise2(0);
    // Gradient noise vanishes at whole-number points, whatever the seed.
    for (float x : {-3.0f, 0.0f, 2.0f, 41.0f}) EXPECT(isApproximatelyEqual(noise(x, 5), 0, 1e-6f));
    // Bounded near ±1, and not flat.
    float largest = 0;
    for (int step = 0; step < 4000; step += 1) {
        float value = noise(static_cast<float>(step) * 0.037f - 20, static_cast<float>(step) * 0.0113f + 3);
        largest = std::max(largest, std::fabs(value));
        EXPECT(std::fabs(value) < 1.45f);
    }
    EXPECT(largest > 0.3f);
    // Smooth: a tiny step changes the value a tiny amount.
    EXPECT(std::fabs(noise(7.3f, 1.1f) - noise(7.3001f, 1.1f)) < 1e-3f);
    // Seeded: the same seed gives the same field, another seed another.
    EXPECT(Warp2D::noise2(0)(3.3f, 4.4f) == noise(3.3f, 4.4f));
    EXPECT(Warp2D::noise2(1)(3.3f, 4.4f) != noise(3.3f, 4.4f));
}

TEST(aWobbleMovesPointsWithinItsAmplitudeAndNotTheOrigin) {
    Wobble wobble;
    wobble.amplitude = 12;
    wobble.octaves = 2;
    Warp2D::PlaneMap map = Warp2D::wobbleMap(wobble);
    EXPECT(isClose(map(Vector2(0, 0)), Vector2(0, 0), 1e-5f));
    float farthest = 0;
    for (int step = 0; step < 500; step += 1) {
        Vector2 point(static_cast<float>(step) * 3.1f, static_cast<float>(step) * 1.7f);
        float moved = (map(point) - point).length();
        farthest = std::max(farthest, moved);
        EXPECT(moved < 12 * 1.45f * 1.5f);
    }
    EXPECT(farthest > 1);
    // Zero amplitude moves nothing.
    EXPECT(isClose(Warp2D::wobbleMap(Wobble{})(Vector2(33, 44)), Vector2(33, 44), 1e-6f));
}

TEST(aWobbleWaveStartsAtOneAndClosesIntoARing) {
    // golden wobbleWaveAt: amplitude 0.3, length 800: the first value is 1.
    Wobble wobble;
    wobble.amplitude = 0.3f;
    Warp2D::Wave open = Warp2D::wobbleWave(wobble, 800);
    EXPECT(isApproximatelyEqual(open(0), 1, 1e-5f));
    // Round a ring the ends meet: u = 0 and u = 1 are the same place.
    Warp2D::Wave ring = Warp2D::wobbleWave(wobble, 800, true);
    EXPECT(isApproximatelyEqual(ring(0), ring(1), 1e-3f));
    EXPECT(std::fabs(ring(0.4f) - 1) < 0.3f * 1.5f);
}

TEST(scatterIsAStableValueForEachKey) {
    // golden scatterAt seed 0: keys 0, 1, 2, 65, 97, 120, 128512, -4, 4294967305.
    std::vector<std::int64_t> keys{0, 1, 2, 65, 97, 120, 128512, -4, 4294967305};
    std::vector<float> expected{-0.4671415826f, 0.9213489699f, 0.0808540410f, -0.6246951362f, 0.1138928407f,
                                0.1532864189f,  0.7280157413f, -0.6549568753f, -0.3188579013f};
    for (size_t index = 0; index < keys.size(); index += 1) EXPECT(isApproximatelyEqual(Warp2D::scatter(0, keys[index]), expected[index], 1e-5f));
    // Property: the same inputs give the same value, always inside -1…1, and another seed changes it.
    for (std::int64_t key = 0; key < 200; key += 1) {
        float value = Warp2D::scatter(5, key);
        EXPECT(value >= -1 && value <= 1 && value == Warp2D::scatter(5, key));
    }
    EXPECT(Warp2D::scatter(1, 65) != Warp2D::scatter(0, 65));
}
