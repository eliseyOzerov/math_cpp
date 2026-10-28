// Tests of Curves/PathAttach (slice S5): the affine maps through three and through many pairs, and
// placing a path by one, two, three and more pairs, by crossings, and aligned to a tangent. Values
// are worked out by hand in the comments; two come from graphics_cpp's attach.json golden file.
#include <cmath>
#include <vector>

#include "../Curves/PathAttach.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// The wave of PathAnalysisTests: two arches, 200 across.
Path2D wavePath() {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(30, 60), Vector2(70, 60), Vector2(100, 0));
    path.addCubic(Vector2(130, -60), Vector2(170, -60), Vector2(200, 0));
    return path;
}

/// A straight run from (0, 0) up to (0, 100), as one cubic (the golden's attached path).
Path2D verticalCubic() {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 100.0f / 3), Vector2(0, 200.0f / 3), Vector2(0, 100));
    return path;
}

/// A single point as a path, which a target can be "on".
Path2D pointPath(Vector2 point) { return Path2D(point); }

/// An L: along x to (10, 0), then up to (10, 10); the points at progress 0, 0.5, 1 are its three
/// corners.
Path2D lShape() {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(10, 0));
    path.addLine(Vector2(10, 10));
    return path;
}

AttachPair pairAt(Vector2 point, std::optional<float> from = std::nullopt) { return AttachPair{TargetPoint{point}, from}; }

} // namespace

TEST(affineThroughTriangleScalesAndMovesTheUnitTriangleExactly) {
    // (0,0),(1,0),(0,1) → (2,3),(4,3),(2,7): x axis (2,0), y axis (0,4), translation (2,3).
    std::optional<AffineTransform2> map = PathAttachment::affineThroughTriangle(
        {Vector2(0, 0), Vector2(1, 0), Vector2(0, 1)}, {Vector2(2, 3), Vector2(4, 3), Vector2(2, 7)});
    EXPECT(map.has_value());
    EXPECT(isClose(map->xAxis, Vector2(2, 0), 1e-5f));
    EXPECT(isClose(map->yAxis, Vector2(0, 4), 1e-5f));
    EXPECT(isClose(map->translation, Vector2(2, 3), 1e-5f));
}

TEST(affineThroughTriangleTakesEverySourceOntoItsTargetAndRefusesALine) {
    std::vector<Vector2> sources = {Vector2(1, 2), Vector2(6, 3), Vector2(2, 9)};
    std::vector<Vector2> targets = {Vector2(-4, 5), Vector2(0, 11), Vector2(-9, 8)};
    std::optional<AffineTransform2> map = PathAttachment::affineThroughTriangle(sources, targets);
    EXPECT(map.has_value());
    for (size_t index = 0; index < 3; index += 1) EXPECT(isClose(map->applied(sources[index]), targets[index], 1e-4f));
    EXPECT(!PathAttachment::affineThroughTriangle({Vector2(0, 0), Vector2(1, 1), Vector2(2, 2)}, targets).has_value());
    EXPECT_PROBLEM(PathAttachment::affineThroughTriangle({Vector2(0, 0)}, {Vector2(0, 0)}), "three points");
}

TEST(affineLeastSquaresRecoversAnExactMapAndRefusesALine) {
    // The unit square's corners doubled and moved by (1, 1): x axis (2,0), y axis (0,2), translation (1,1).
    std::vector<Vector2> sources = {Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)};
    std::vector<Vector2> targets = {Vector2(1, 1), Vector2(3, 1), Vector2(3, 3), Vector2(1, 3)};
    std::optional<AffineTransform2> map = PathAttachment::affineLeastSquares(sources, targets);
    EXPECT(map.has_value());
    EXPECT(isClose(map->xAxis, Vector2(2, 0), 1e-4f));
    EXPECT(isClose(map->yAxis, Vector2(0, 2), 1e-4f));
    EXPECT(isClose(map->translation, Vector2(1, 1), 1e-4f));
    EXPECT(!PathAttachment::affineLeastSquares({Vector2(0, 0), Vector2(1, 1), Vector2(2, 2), Vector2(3, 3)}, targets).has_value());
}

TEST(affineLeastSquaresOfThreePairsEqualsTheExactMap) {
    std::vector<Vector2> sources = {Vector2(1, 2), Vector2(6, 3), Vector2(2, 9)};
    std::vector<Vector2> targets = {Vector2(-4, 5), Vector2(0, 11), Vector2(-9, 8)};
    std::optional<AffineTransform2> exact = PathAttachment::affineThroughTriangle(sources, targets);
    std::optional<AffineTransform2> fitted = PathAttachment::affineLeastSquares(sources, targets);
    EXPECT(exact.has_value() && fitted.has_value());
    EXPECT(isClose(exact->xAxis, fitted->xAxis, 1e-3f));
    EXPECT(isClose(exact->yAxis, fitted->yAxis, 1e-3f));
    EXPECT(isClose(exact->translation, fitted->translation, 1e-2f));
}

TEST(affineLeastSquaresMinimisesTheErrorWhenNoMapFits) {
    // Four corners of a square with one target pulled up by 4: the best map is the fit of the mean
    // shift, so the residuals (target − fitted) sum to zero in each coordinate.
    std::vector<Vector2> sources = {Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)};
    std::vector<Vector2> targets = {Vector2(0, 0), Vector2(1, 0), Vector2(1, 5), Vector2(0, 1)};
    std::optional<AffineTransform2> map = PathAttachment::affineLeastSquares(sources, targets);
    EXPECT(map.has_value());
    Vector2 residual;
    for (size_t index = 0; index < 4; index += 1) residual = residual + (targets[index] - map->applied(sources[index]));
    EXPECT(isClose(residual, Vector2(0, 0), 1e-3f));
}

TEST(attachedWithOneMountShiftsTheStartOntoThePlaceOnThePath) {
    // Golden "attach": the vertical run mounted at 0.3 of the wave starts at (63.673, 41.928).
    Path2D placed = PathAttachment::attached(verticalCubic(), AttachMount{wavePath(), 0.3f, std::nullopt, std::nullopt});
    EXPECT(isClose(placed.start, Vector2(63.6735f, 41.9276f), 0.06f));
    EXPECT(isClose(placed.currentPoint(), placed.start + Vector2(0, 100), 1e-3f));
    // With `from` 0.5 the middle of the run goes to 0.6 of the wave: start (114.887, −73.942).
    Path2D middle = PathAttachment::attached(verticalCubic(), AttachMount{wavePath(), 0.6f, 0.5f, std::nullopt});
    EXPECT(isClose(middle.start, Vector2(114.8868f, -73.9424f), 0.06f));
}

TEST(attachedWithTwoMountsTurnsAndStretchesAlongTheLineKeepingDepth) {
    // The run (0,0)→(20,0) onto (3,4)→(3,14): half as long and turned up. (10, 0) → (3, 9); a point
    // 4 to the left of the run, (10, 4), keeps its depth 4 and lands 4 to the left of the target
    // line: (−1, 9).
    Path2D run(Vector2(0, 0));
    run.addLine(Vector2(10, 4));
    run.addLine(Vector2(20, 0));
    Path2D placed = PathAttachment::attached(
        run, AttachMount{pointPath(Vector2(3, 4)), 0, std::nullopt, std::nullopt},
        AttachMount{pointPath(Vector2(3, 14)), 0, std::nullopt, std::nullopt});
    EXPECT(isClose(placed.start, Vector2(3, 4), 1e-4f));
    EXPECT(isClose(placed.segments()[0].end(), Vector2(-1, 9), 1e-4f));
    EXPECT(isClose(placed.currentPoint(), Vector2(3, 14), 1e-4f));
}

TEST(attachedWithTwoMountsOnOnePointPlacesByTheFirstAlone) {
    Path2D run(Vector2(0, 0));
    run.addLine(Vector2(20, 0));
    AttachMount mount{pointPath(Vector2(5, 5)), 0, std::nullopt, std::nullopt};
    Path2D placed = PathAttachment::attached(run, mount, mount);
    EXPECT(isClose(placed.start, Vector2(5, 5), 1e-4f));
    EXPECT(isClose(placed.currentPoint(), Vector2(25, 5), 1e-4f));
}

TEST(targetPointReadsFixedPointsPlacesAndCrossings) {
    EXPECT(PathAttachment::targetPoint(TargetPoint{Vector2(7, 8)}) == Vector2(7, 8));
    // Golden "target": 0.4 of the wave is (85.113, 23.942).
    EXPECT(isClose(PathAttachment::targetPoint(TargetOnPath{wavePath(), 0.4f, std::nullopt}), Vector2(85.1132f, 23.9424f), 0.06f));
    // Two diagonals of a square cross at its centre.
    Path2D rising(Vector2(0, 0));
    rising.addLine(Vector2(10, 10));
    Path2D falling(Vector2(0, 10));
    falling.addLine(Vector2(10, 0));
    EXPECT(isClose(PathAttachment::targetPoint(TargetCrossing{rising, falling, std::nullopt}), Vector2(5, 5), 1e-3f));
    EXPECT(isClose(PathAttachment::targetPoint(TargetCrossing{rising, falling, CrossingChoice::last()}), Vector2(5, 5), 1e-3f));
}

TEST(targetPointThrowsWhereThePathsDoNotCross) {
    Path2D low(Vector2(0, 0));
    low.addLine(Vector2(10, 0));
    Path2D high(Vector2(0, 5));
    high.addLine(Vector2(10, 5));
    EXPECT_PROBLEM(PathAttachment::targetPoint(TargetCrossing{low, high, std::nullopt}), "don't cross at all");
}

TEST(placedWithThreePairsMapsTheThreePointsExactlyOntoTheirTargets) {
    // The L's corners (0,0), (10,0), (10,10) at progress 0, 0.5, 1 doubled and moved by (5, 5).
    Path2D placed = PathAttachment::placed(
        lShape(), {pairAt(Vector2(5, 5), 0.0f), pairAt(Vector2(25, 5), 0.5f), pairAt(Vector2(25, 25), 1.0f)});
    EXPECT(isClose(placed.start, Vector2(5, 5), 1e-3f));
    EXPECT(isClose(placed.segments()[0].end(), Vector2(25, 5), 1e-3f));
    EXPECT(isClose(placed.currentPoint(), Vector2(25, 25), 1e-3f));
}

TEST(placedWithOneOrTwoPairsActsLikeAttached) {
    Path2D one = PathAttachment::placed(lShape(), {pairAt(Vector2(5, 5))});
    EXPECT(isClose(one.start, Vector2(5, 5), 1e-4f));
    EXPECT(isClose(one.currentPoint(), Vector2(15, 15), 1e-4f));
    // Two pairs at the default progress 0 and 1: start to (0, 0)→(0, 20) is the corner-to-corner chord.
    Path2D two = PathAttachment::placed(lShape(), {pairAt(Vector2(0, 0)), pairAt(Vector2(0, 28.2842712f))});
    EXPECT(isClose(two.currentPoint(), Vector2(0, 28.2842712f), 1e-3f));
    // No pairs leave the path as it is.
    EXPECT(PathAttachment::placed(lShape(), {}) == lShape());
}

TEST(placedWithManyPairsLandsABentNodeOnEveryTarget) {
    // Four pairs whose targets no affine map meets: the path is bent so a node sits on each.
    std::vector<AttachPair> pairs = {
        pairAt(Vector2(0, 0)), pairAt(Vector2(8, 3)), pairAt(Vector2(13, 6)), pairAt(Vector2(10, 12))};
    Path2D placed = PathAttachment::placed(lShape(), pairs);
    std::vector<Vector2> nodes = {placed.start};
    for (const Path2D::Segment& segment : placed.segments()) nodes.push_back(segment.end());
    for (const AttachPair& pair : pairs) {
        Vector2 goal = std::get<TargetPoint>(pair.target).point;
        bool found = false;
        for (Vector2 node : nodes) found = found || isClose(node, goal, 1e-2f);
        EXPECT(found);
    }
}

TEST(placedWithManyPairsOnAnAffineImageMovesTheWholePathByThatMap) {
    // Four pairs that are exactly "double and move by (5, 5)": no bending is left, so the end is the
    // doubled end.
    std::vector<AttachPair> pairs;
    MeasuredPath2D measured(lShape());
    for (float progress : {0.0f, 1.0f / 3, 2.0f / 3, 1.0f}) {
        pairs.push_back(pairAt(measured.pointAt(progress) * 2 + Vector2(5, 5), progress));
    }
    Path2D placed = PathAttachment::placed(lShape(), pairs);
    EXPECT(isClose(placed.start, Vector2(5, 5), 1e-2f));
    EXPECT(isClose(placed.currentPoint(), Vector2(25, 25), 1e-2f));
    EXPECT(isApproximatelyEqual(MeasuredPath2D(placed).length(), 40.0f, 0.1f));
}

TEST(placedWithPairsOnALineIsPlacedByTheOuterTwoAndBent) {
    // Three collinear targets decide no affine map; the run is still placed end to end.
    Path2D run(Vector2(0, 0));
    run.addLine(Vector2(10, 0));
    Path2D placed = PathAttachment::placed(run, {pairAt(Vector2(0, 0)), pairAt(Vector2(5, 5)), pairAt(Vector2(10, 10))});
    EXPECT(isClose(placed.start, Vector2(0, 0), 1e-3f));
    EXPECT(isClose(placed.currentPoint(), Vector2(10, 10), 1e-3f));
}

TEST(placedAlignedTurnsTheDirectionOntoTheTangentOrTheNormal) {
    Path2D horizontal(Vector2(0, 0));
    horizontal.addLine(Vector2(10, 0));
    // The run points up and is 100 long; pinned at (3, 4) and turned onto the horizontal's tangent
    // (1, 0) it points right, so its end goes to (103, 4).
    AttachAlign tangent{horizontal, 0.5f, std::nullopt, AlignTo::tangent, Angle(), std::nullopt};
    Path2D onTangent = PathAttachment::placed(verticalCubic(), {pairAt(Vector2(3, 4))}, tangent);
    EXPECT(isClose(onTangent.start, Vector2(3, 4), 1e-3f));
    EXPECT(isClose(onTangent.currentPoint(), Vector2(3 + 100, 4), 1e-2f));
    // Onto the normal (0, 1) it already points that way, end (3, 104); a further 90° turns it left,
    // so it points toward −x and the end is (−97, 4).
    AttachAlign normal{horizontal, 0.5f, std::nullopt, AlignTo::normal, Angle(), std::nullopt};
    Path2D onNormal = PathAttachment::placed(verticalCubic(), {pairAt(Vector2(3, 4))}, normal);
    EXPECT(isClose(onNormal.currentPoint(), Vector2(3, 104), 1e-2f));
    normal.extraAngle = Angle::degrees(90);
    Path2D turned = PathAttachment::placed(verticalCubic(), {pairAt(Vector2(3, 4))}, normal);
    EXPECT(isClose(turned.currentPoint(), Vector2(3 - 100, 4), 1e-2f));
}

TEST(placedRefusesAnAlignmentWithTwoPairs) {
    Path2D horizontal(Vector2(0, 0));
    horizontal.addLine(Vector2(10, 0));
    AttachAlign align{horizontal, 0.5f, std::nullopt, AlignTo::tangent, Angle(), std::nullopt};
    EXPECT_PROBLEM(PathAttachment::placed(lShape(), {pairAt(Vector2(0, 0)), pairAt(Vector2(1, 1))}, align), "one pair at most");
}

TEST(placedInAMetricMeasuresLengthsAndAnglesOnThePage) {
    // In metric (1, 2) the L's progress 0.5 is the corner only if the legs count equal: legs 10 and 10
    // measure 10 and 20, so progress 1/3 is the corner. Pin it to (1, 1) alone.
    Path2D placed = PathAttachment::placed(lShape(), {pairAt(Vector2(1, 1), 1.0f / 3)}, std::nullopt, Vector2(1, 2));
    EXPECT(isClose(placed.start, Vector2(1 - 10, 1), 1e-3f));
}
