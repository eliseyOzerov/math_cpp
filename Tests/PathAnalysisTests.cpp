// Tests of the path-level analysis of slice S5 (Path2D+Analysis, +Intersections, +Picking): extrema,
// inflections, sides, curvature, normal, osculating circle, mold, projection, nearness, line and
// circle crossings, arcs and node picking. Values worked out by hand in the comments, and a few
// taken from graphics_cpp's analysis.json golden file.
#include <cmath>
#include <vector>

#include "../Curves/MeasuredPath2D.h"
#include "../Curves/Path2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// Two cubics: an arch up to y = 45 over x 0…100, then a mirrored arch down to y = −45 over 100…200.
/// By the symmetry each cubic is half the length, and the extrema are at the arches' tops.
Path2D wavePath() {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(30, 60), Vector2(70, 60), Vector2(100, 0));
    path.addCubic(Vector2(130, -60), Vector2(170, -60), Vector2(200, 0));
    return path;
}

/// The square (0,0)…(10,10), counterclockwise, closed by its implicit closing line (length 40).
Path2D smallSquare() {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(10, 0));
    path.addLine(Vector2(10, 10));
    path.addLine(Vector2(0, 10));
    path.close();
    return path;
}

} // namespace

TEST(pathExtremaOfTwoArchesAreTheirTopsAtAQuarterAndThreeQuarters) {
    // Golden analysis.json "extrema": [0.25, 0.75].
    std::vector<float> extrema = wavePath().extremaProgresses();
    EXPECT(extrema.size() == 2);
    EXPECT(isApproximatelyEqual(extrema[0], 0.25f, 1e-3f));
    EXPECT(isApproximatelyEqual(extrema[1], 0.75f, 1e-3f));
    // A straight-sided path has none inside its pieces.
    EXPECT(smallSquare().extremaProgresses().empty());
}

TEST(pathInflectionsOfTwoArchesAreNoneAndOfAnSAreInTheMiddle) {
    EXPECT(wavePath().inflectionProgresses().empty());
    // An S: bends one way then the other, the inflection at the centre by point symmetry.
    Path2D curve(Vector2(0, 0));
    curve.addCubic(Vector2(40, 0), Vector2(60, 100), Vector2(100, 100));
    std::vector<float> inflections = curve.inflectionProgresses();
    EXPECT(inflections.size() == 1);
    EXPECT(isApproximatelyEqual(inflections[0], 0.5f, 1e-3f));
}

TEST(pathSidesReachTheExtremesAndTiesTakeTheFirst) {
    Path2D path = wavePath();
    EXPECT(isApproximatelyEqual(path.progressOfSide(PathSide::left), 0.0f, 1e-3f));
    EXPECT(isApproximatelyEqual(path.progressOfSide(PathSide::right), 1.0f, 1e-3f));
    EXPECT(isApproximatelyEqual(path.progressOfSide(PathSide::top), 0.25f, 1e-3f));
    EXPECT(isApproximatelyEqual(path.progressOfSide(PathSide::bottom), 0.75f, 1e-3f));
    // The bottom of the square is the whole side from progress 0 to 0.25; the first is 0.
    EXPECT(isApproximatelyEqual(smallSquare().progressOfSide(PathSide::bottom), 0.0f, 1e-4f));
}

TEST(pathCurvatureNormalAndOsculatingCircleAtTheStartOfAnArch) {
    Path2D path = wavePath();
    // At the start of a cubic, curvature = (2/3)·(P1−P0)×(P2−P1)/|P1−P0|³ = (2/3)(−2400)/67.08³ = −0.0053003.
    EXPECT(isApproximatelyEqual(path.curvatureAt(0), -0.0053003f, 1e-5f));
    // Direction (30, 60)/67.08; the left normal is (−60, 30)/67.08 = (−0.894427, 0.447214).
    EXPECT(isClose(path.normalAt(0), Vector2(-0.894427f, 0.447214f), 1e-4f));
    OsculatingCircle circle = path.osculatingCircleAt(0);
    EXPECT(circle.centre.has_value());
    EXPECT(isClose(*circle.centre, Vector2(168.75f, -84.375f), 0.05f));
    EXPECT(isApproximatelyEqual(circle.radius, 188.668f, 0.1f));
    EXPECT(isApproximatelyEqual(circle.curvature, -0.0053003f, 1e-5f));
    // A straight piece has no centre and an infinite radius.
    OsculatingCircle straight = smallSquare().osculatingCircleAt(0.1f);
    EXPECT(!straight.centre.has_value());
    EXPECT(std::isinf(straight.radius));
}

TEST(pathNormalIsAUnitVectorAcrossTheDirectionEverywhere) {
    Path2D path = wavePath();
    MeasuredPath2D measured(path);
    for (float progress : {0.0f, 0.1f, 0.33f, 0.5f, 0.8f, 1.0f}) {
        Vector2 normal = path.normalAt(progress);
        Vector2 direction = measured.directionAt(progress);
        EXPECT(isApproximatelyEqual(normal.length(), 1.0f, 1e-5f));
        EXPECT(isApproximatelyEqual(normal.dot(direction), 0.0f, 1e-5f));
        EXPECT(direction.cross(normal) > 0);
    }
}

TEST(pathMetricStretchesTheNormalAndTheProgress) {
    // In metric (1, 2) the vertical run of an L counts double, so the corner is at 10 / (10 + 40) = 0.2.
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(10, 0));
    path.addLine(Vector2(10, 20));
    Vector2 metric(1, 2);
    EXPECT(isApproximatelyEqual(path.projection(Vector2(10, 0), metric).progress, 0.2f, 1e-4f));
    // Direction at 0.5 is up; its left normal points left.
    EXPECT(isClose(path.normalAt(0.5f, metric), Vector2(-1, 0), 1e-5f));
}

TEST(pathProjectionFindsTheNearestPlaceAndItsDistance) {
    // Golden "project": (50, 50) → progress 0.25, point (50, 45), distance 5 (the arch's top is y = 45).
    PathProjection found = wavePath().projection(Vector2(50, 50));
    EXPECT(isApproximatelyEqual(found.progress, 0.25f, 1e-3f));
    EXPECT(isClose(found.point, Vector2(50, 45), 1e-3f));
    EXPECT(isApproximatelyEqual(found.distance, 5.0f, 1e-3f));
}

TEST(pathProjectionOfAPointOnThePathIsItselfAtDistanceZero) {
    Path2D path = wavePath();
    MeasuredPath2D measured(path);
    for (float progress : {0.1f, 0.4f, 0.6f, 0.9f}) {
        PathProjection found = path.projection(measured.pointAt(progress));
        EXPECT(isApproximatelyEqual(found.distance, 0.0f, 1e-2f));
        EXPECT(isApproximatelyEqual(found.progress, progress, 2e-3f));
    }
}

TEST(pathMoldedAtPassesThroughTheDestinationAndKeepsTheEnds) {
    Path2D path = wavePath();
    Vector2 destination(30, 60);
    Path2D molded = path.moldedAt(0.1f, destination);
    EXPECT(molded.start == path.start);
    EXPECT(molded.currentPoint() == path.currentPoint());
    EXPECT(molded.segments().size() == path.segments().size());
    EXPECT(isApproximatelyEqual(molded.projection(destination).distance, 0.0f, 0.02f));
    // The piece it did not touch is unchanged apart from the handle the node's kind carries along.
    EXPECT(molded.segments()[1].endHandle == path.segments()[1].endHandle);
}

TEST(pathMoldedAtCanBendTheClosingLineOfAClosedPath) {
    Path2D square = smallSquare();
    // 0.9 is on the closing line, 1 up from (0, 0); pull it 3 out to the left.
    Vector2 destination(-3, 1);
    Path2D molded = square.moldedAt(0.9f, destination);
    EXPECT(molded.isClosed());
    EXPECT(isApproximatelyEqual(molded.projection(destination).distance, 0.0f, 0.02f));
    EXPECT(molded.start == square.start);
}

TEST(pathNearnessIsOneAtAPlaceAndZeroFromReachOn) {
    // Golden "nearness" with places 0.3 and 0.8, reach 60, smooth falloff.
    Path2D path = wavePath();
    std::function<float(float)> nearness = path.nearnessTo({0.3f, 0.8f}, 60);
    EXPECT(isApproximatelyEqual(nearness(0.3f), 1.0f, 1e-6f));
    EXPECT(isApproximatelyEqual(nearness(0.8f), 1.0f, 1e-6f));
    EXPECT(isApproximatelyEqual(nearness(0.0f), 0.0f, 1e-6f));
    EXPECT(isApproximatelyEqual(nearness(0.1f), 0.0098829f, 2e-3f));
    EXPECT(isApproximatelyEqual(nearness(0.25f), 0.85989f, 2e-3f));
    EXPECT(isApproximatelyEqual(nearness(0.7f), 0.54386f, 2e-3f));
    // A reach of 0 reaches nowhere, even at the place itself.
    EXPECT(isApproximatelyEqual(path.nearnessTo({0.3f}, 0)(0.3f), 0.0f, 1e-6f));
}

TEST(pathNearnessOfAClosedPathWrapsRoundTheStart) {
    // 40 long, place at 0; 0.05 and 0.95 are both 2 away, one each way.
    std::function<float(float)> nearness = smallSquare().nearnessTo({0.0f}, 8, Falloff::linear);
    EXPECT(isApproximatelyEqual(nearness(0.05f), nearness(0.95f), 1e-5f));
    EXPECT(isApproximatelyEqual(nearness(0.05f), 0.75f, 1e-4f)); // linear: 1 − 2/8
}

TEST(pathLineCrossingsListTheHitsWithProgressAndLineParameter) {
    // Golden "lineCrossings": the line x = 50 from y = −100 to 100 meets the first arch once, at
    // (50, 45): s = (45 + 100)/200 = 0.725.
    std::vector<PathLineCrossing> hits = wavePath().lineCrossings(Vector2(50, -100), Vector2(50, 100));
    EXPECT(hits.size() == 1);
    EXPECT(isApproximatelyEqual(hits[0].progress, 0.25f, 1e-3f));
    EXPECT(isApproximatelyEqual(hits[0].lineParameter, 0.725f, 1e-3f));
    EXPECT(isClose(hits[0].point, Vector2(50, 45)));
    // The horizontal axis crosses the wave at its three zero points, in order along the path.
    std::vector<PathLineCrossing> axis = wavePath().lineCrossings(Vector2(-10, 0), Vector2(210, 0), false);
    EXPECT(axis.size() >= 2);
    for (size_t index = 1; index < axis.size(); index += 1) EXPECT(axis[index].progress >= axis[index - 1].progress);
}

TEST(pathLineCrossingsOnlyBetweenThePointsUnlessTheLineIsInfinite) {
    Path2D path = smallSquare();
    // The line x = 5 meets the bottom (5, 0) and the top (5, 10). Cut short at y = 5 it meets neither.
    EXPECT(path.lineCrossings(Vector2(5, 3), Vector2(5, 7)).empty());
    EXPECT(path.lineCrossings(Vector2(5, 3), Vector2(5, 7), false).size() == 2);
    EXPECT(path.lineCrossings(Vector2(5, -2), Vector2(5, 12)).size() == 2);
}

TEST(pathCircleCrossingsAreOnTheCircleAndMatchTheGolden) {
    // Golden "circleCrossings": circle (60, 60) radius 30.
    std::vector<PathCircleCrossing> hits = wavePath().circleCrossings(Vector2(60, 60), 30);
    EXPECT(hits.size() == 2);
    EXPECT(isApproximatelyEqual(hits[0].progress, 0.199213f, 2e-3f));
    EXPECT(isClose(hits[0].point, Vector2(36.1243f, 41.8354f), 0.02f));
    EXPECT(isApproximatelyEqual(hits[1].progress, 0.349753f, 2e-3f));
    EXPECT(isClose(hits[1].point, Vector2(75.3782f, 34.2413f), 0.02f));
    for (const PathCircleCrossing& hit : hits) EXPECT(isApproximatelyEqual(hit.point.distance(Vector2(60, 60)), 30.0f, 0.02f));
}

TEST(pathCircularArcsCoverThePathWithArcsThatStartAndEndOnTheirCircles) {
    Path2D path = wavePath();
    std::vector<PathArc> arcs = path.circularArcs(0.5f);
    EXPECT(arcs.size() >= 2);
    EXPECT(isApproximatelyEqual(arcs.front().startProgress, 0.0f, 1e-4f));
    EXPECT(isApproximatelyEqual(arcs.back().endProgress, 1.0f, 1e-4f));
    for (size_t index = 0; index < arcs.size(); index += 1) {
        const PathArc& piece = arcs[index];
        if (index > 0) EXPECT(isApproximatelyEqual(piece.startProgress, arcs[index - 1].endProgress, 1e-4f));
        EXPECT(piece.endProgress > piece.startProgress);
        if (piece.arc.centre) {
            EXPECT(isApproximatelyEqual(piece.arc.from.distance(*piece.arc.centre), piece.arc.radius, 0.05f));
            EXPECT(isApproximatelyEqual(piece.arc.to.distance(*piece.arc.centre), piece.arc.radius, 0.05f));
        }
    }
    EXPECT(isClose(arcs.front().arc.from, path.start));
    EXPECT(isClose(arcs.back().arc.to, path.currentPoint()));
}

TEST(pathPickingNodesWeightsTheChosenNodesWholeAndCountsNegativeFromTheEnd) {
    Path2D path = wavePath();
    EXPECT(path.nodeCount() == 3);
    PickedNodes picked = path.pickingNodes({0, -1});
    EXPECT((picked.weights == std::vector<float>{1, 0, 1}));
    EXPECT(picked.path == path);
    EXPECT_PROBLEM(path.pickingNodes({3}), "no such node");
}

TEST(pathPickingCubicsPicksBothEndsOfEachAndWrapsOnAClosedPath) {
    Path2D path = wavePath();
    EXPECT((path.pickingCubics({0}).weights == std::vector<float>{1, 1, 0}));
    EXPECT((path.pickingCubics({-1}).weights == std::vector<float>{0, 1, 1}));
    Path2D square = smallSquare();
    EXPECT(square.nodeCount() == 4);
    // The closing line is piece 3, from node 3 to node 0.
    EXPECT((square.pickingCubics({3}).weights == std::vector<float>{1, 0, 0, 1}));
}

TEST(pathPickingSectionCutsAtBothProgressesAndPicksTheNodesBetween) {
    // Cut at 0.25 and 0.75 (inside the two arches): nodes at 0, 0.25, 0.5, 0.75, 1.
    PickedNodes picked = wavePath().pickingSection(0.75f, 0.25f);
    EXPECT(picked.path.nodeCount() == 5);
    EXPECT((picked.weights == std::vector<float>{0, 1, 1, 1, 0}));
}
