// Tests of the cubic Bézier's frame and shape functions, sampling, projection, curves through
// points and arcs from end points (Curves/CubicBezier+Frame, +Sampling, +Projection, +Through,
// +ArcFromEnds), ported from graphics_cpp's Curves2D. Hand-worked values come first; values
// marked "golden" are transcribed from Curves2D/tests/golden/bezier.json and geometry.json.
#include <cmath>
#include <vector>

#include "../Curves/CubicBezier.h"
#include "../Linear/MathProblem.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Cubic = CubicBezier<Vector2>;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// The arch used throughout: up the left side, across, and down the right.
Cubic arch() { return Cubic(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0)); }
/// An S: one inflection.
Cubic sCurve() { return Cubic(Vector2(0, 0), Vector2(50, 100), Vector2(50, -100), Vector2(100, 0)); }
/// A loop.
Cubic loopCurve() { return Cubic(Vector2(0, 0), Vector2(150, 100), Vector2(-50, 100), Vector2(100, 0)); }

} // namespace

// MARK: - coordinateRoots, parametersAtCoordinate

TEST(coordinateRootsOfALinearRampIsTheShare) {
    // Values 0, 1/3, 2/3, 1 are the line y = t: it equals 0.25 at t = 0.25.
    std::vector<float> roots = Cubic::coordinateRoots(0, 1.0f / 3, 2.0f / 3, 1, 0.25f);
    EXPECT(roots.size() == 1);
    if (roots.size() == 1) EXPECT(isApproximatelyEqual(roots[0], 0.25f, 1e-5f));
}

TEST(theArchReachesXFiftyAtHalfAndYTwentyTwice) {
    // golden: x = 50 at t = 0.5; y = 20 at t = 0.0718256 and 0.9281744
    std::vector<float> atX = arch().parametersAtCoordinate(CurveAxis::x, 50);
    EXPECT(atX.size() == 1);
    if (atX.size() == 1) EXPECT(isApproximatelyEqual(atX[0], 0.5f, 1e-4f));
    std::vector<float> atY = arch().parametersAtCoordinate(CurveAxis::y, 20);
    EXPECT(atY.size() == 2);
    if (atY.size() == 2) {
        EXPECT(isApproximatelyEqual(atY[0], 0.0718256f, 1e-4f));
        EXPECT(isApproximatelyEqual(atY[1], 0.9281744f, 1e-4f));
    }
}

TEST(everyCoordinateRootLandsOnTheValue) {
    for (float level : {5.0f, 20.0f, 60.0f, 74.0f}) {
        for (float parameter : sCurve().parametersAtCoordinate(CurveAxis::y, level)) {
            EXPECT(isApproximatelyEqual(sCurve().point(parameter).y, level, 1e-2f));
        }
    }
    EXPECT(arch().parametersAtCoordinate(CurveAxis::y, 80).empty()); // the arch peaks at 75
}

// MARK: - alignedTo, tightBox

TEST(aligningToTheXAxisChordChangesNothing) {
    Cubic aligned = arch().alignedToChord();
    EXPECT(isClose(aligned.startHandle, Vector2(0, 100)));
    EXPECT(isClose(aligned.end, Vector2(100, 0)));
}

TEST(aligningToAVerticalLineTurnsTheCurveAQuarterClockwise) {
    // golden: line (0,0)→(0,10) takes (0,100)→(100,0), (100,100)→(100,-100), (100,0)→(0,-100)
    Cubic aligned = arch().alignedTo(Vector2(0, 0), Vector2(0, 10));
    EXPECT(isClose(aligned.startHandle, Vector2(100, 0)));
    EXPECT(isClose(aligned.endHandle, Vector2(100, -100)));
    EXPECT(isClose(aligned.end, Vector2(0, -100)));
}

TEST(alignmentKeepsLengthsAndPutsTheChordOnTheXAxis) {
    Cubic moved(Vector2(10, 5), Vector2(20, 40), Vector2(60, 50), Vector2(70, 20));
    Cubic aligned = moved.alignedToChord();
    EXPECT(isClose(aligned.start, Vector2(0, 0)));
    EXPECT(isApproximatelyEqual(aligned.end.y, 0, 1e-3f));
    EXPECT(isApproximatelyEqual(aligned.end.x, moved.start.distance(moved.end), 1e-3f));
    EXPECT(isApproximatelyEqual(aligned.totalLength(), moved.totalLength(), 1e-2f));
}

TEST(theArchsTightBoxIsItsChordFrameBox) {
    // golden: corners (0,0), (100,0), (100,75), (0,75)
    std::array<Vector2, 4> box = arch().tightBox();
    EXPECT(isClose(box[0], Vector2(0, 0)));
    EXPECT(isClose(box[1], Vector2(100, 0)));
    EXPECT(isClose(box[2], Vector2(100, 75)));
    EXPECT(isClose(box[3], Vector2(0, 75)));
}

TEST(theSCurvesTightBoxStraddlesItsChord) {
    // golden: y from -28.8675 to 28.8675 along x 0…100
    std::array<Vector2, 4> box = sCurve().tightBox();
    EXPECT(isClose(box[0], Vector2(0, -28.8675f)));
    EXPECT(isClose(box[2], Vector2(100, 28.8675f)));
}

TEST(aTurnedCurvesTightBoxIsTheTurnedBox) {
    // The arch turned a quarter counter-clockwise has the same box, turned: (x, y) → (-y, x).
    Cubic turned(Vector2(0, 0), Vector2(-100, 0), Vector2(-100, 100), Vector2(0, 100));
    std::array<Vector2, 4> box = turned.tightBox();
    EXPECT(isClose(box[0], Vector2(0, 0)));
    EXPECT(isClose(box[1], Vector2(0, 100)));
    EXPECT(isClose(box[2], Vector2(-75, 100)));
    EXPECT(isClose(box[3], Vector2(-75, 0)));
}

// MARK: - isStraight, reversed

TEST(handlesOnTheChordMakeALine) {
    EXPECT(Cubic(Vector2(0, 0), Vector2(10, 0), Vector2(70, 0), Vector2(100, 0)).isStraight());
    EXPECT(Cubic(Vector2(0, 0), Vector2(30, 60), Vector2(60, 120), Vector2(100, 200)).isStraight());
}

TEST(aHandleOffTheChordOrNoChordIsNotALine) {
    EXPECT(!arch().isStraight()); // golden: false
    EXPECT(!Cubic(Vector2(0, 0), Vector2(10, 1), Vector2(70, 0), Vector2(100, 0)).isStraight());
    EXPECT(!Cubic(Vector2(5, 5), Vector2(6, 6), Vector2(7, 7), Vector2(5, 5)).isStraight());
}

TEST(reversingSwapsTheControlPointsAndMirrorsTheParameter) {
    Cubic reversed = sCurve().reversed();
    EXPECT(reversed.start == sCurve().end);
    EXPECT(reversed.startHandle == sCurve().endHandle);
    for (float parameter : {0.0f, 0.2f, 0.5f, 0.9f}) {
        EXPECT(isClose(reversed.point(parameter), sCurve().point(1 - parameter)));
    }
    EXPECT(reversed.reversed() == sCurve());
}

// MARK: - canonicalForm

TEST(thePrimersExampleCurvesHaveTheirClasses) {
    // golden: arch at (1, 0); S at (1, 2) single inflection; loop at (-0.5, 0)
    CanonicalForm archForm = arch().canonicalForm();
    EXPECT(archForm.kind == CubicShapeKind::arch);
    EXPECT(archForm.fourthPoint && isClose(*archForm.fourthPoint, Vector2(1, 0)));
    CanonicalForm sForm = sCurve().canonicalForm();
    EXPECT(sForm.kind == CubicShapeKind::singleInflection);
    EXPECT(sForm.fourthPoint && isClose(*sForm.fourthPoint, Vector2(1, 2)));
    CanonicalForm loopForm = loopCurve().canonicalForm();
    EXPECT(loopForm.kind == CubicShapeKind::loop);
    EXPECT(loopForm.fourthPoint && isClose(*loopForm.fourthPoint, Vector2(-0.5f, 0)));
}

TEST(theCanonicalFormPutsTheFourthPointByHand) {
    // P0 = (0,0), P1 = (0,1), P2 = (1,1) already are the canonical points, so P3 stays where it is.
    // (0.5, 2) has y > 1: one inflection. (2, 0.5) has x > 1: arch. (0.5, 0.5) is below the cusp
    // curve (0.9375) and below the loop edge (sqrt(3(2 - 0.25)) - 0.5)/2 = 0.896: arch.
    auto kindWithFourth = [](Vector2 fourth) {
        return Cubic(Vector2(0, 0), Vector2(0, 1), Vector2(1, 1), fourth).canonicalForm().kind;
    };
    EXPECT(kindWithFourth(Vector2(0.5f, 2)) == CubicShapeKind::singleInflection);
    EXPECT(kindWithFourth(Vector2(2, 0.5f)) == CubicShapeKind::arch);
    EXPECT(kindWithFourth(Vector2(0.5f, 0.5f)) == CubicShapeKind::arch);
    // (0, 0.9): above the cusp curve at x = 0 (0.75) and y <= 1: two inflections.
    EXPECT(kindWithFourth(Vector2(0, 0.9f)) == CubicShapeKind::doubleInflection);
    // (0, 0.75) lies on the cusp curve at x = 0.
    EXPECT(kindWithFourth(Vector2(0, 0.75f)) == CubicShapeKind::cusp);
    // (-0.5, 0): the loop edge at x = -0.5 is (-0.25 - 1.5)/3 = -0.583, and 0 >= -0.583: a loop.
    EXPECT(kindWithFourth(Vector2(-0.5f, 0)) == CubicShapeKind::loop);
}

TEST(theClassDoesNotChangeUnderAMoveTurnAndScale) {
    // Affine maps keep the class: turn the S by 30 degrees, scale by 3, move.
    float cosine = std::cos(0.5236f), sine = std::sin(0.5236f);
    auto map = [&](Vector2 point) {
        return Vector2(3 * (point.x * cosine - point.y * sine) + 40, 3 * (point.x * sine + point.y * cosine) - 7);
    };
    Cubic mapped(map(sCurve().start), map(sCurve().startHandle), map(sCurve().endHandle), map(sCurve().end));
    CanonicalForm form = mapped.canonicalForm();
    EXPECT(form.kind == CubicShapeKind::singleInflection);
    EXPECT(form.fourthPoint && isClose(*form.fourthPoint, Vector2(1, 2), 1e-2f));
}

TEST(collinearControlPointsHaveNoFourthPoint) {
    CanonicalForm straight = Cubic(Vector2(0, 0), Vector2(10, 0), Vector2(70, 0), Vector2(100, 0)).canonicalForm();
    EXPECT(straight.kind == CubicShapeKind::line && !straight.fourthPoint);
    CanonicalForm doubling = Cubic(Vector2(0, 0), Vector2(10, 0), Vector2(30, 0), Vector2(5, 20)).canonicalForm();
    EXPECT(doubling.kind == CubicShapeKind::arch && !doubling.fourthPoint);
}

TEST(everyKindHasAName) {
    EXPECT(cubicShapeKindName(CubicShapeKind::line) == "line");
    EXPECT(cubicShapeKindName(CubicShapeKind::arch) == "arch");
    EXPECT(cubicShapeKindName(CubicShapeKind::singleInflection) == "single inflection");
    EXPECT(cubicShapeKindName(CubicShapeKind::doubleInflection) == "double inflection");
    EXPECT(cubicShapeKindName(CubicShapeKind::loop) == "loop");
    EXPECT(cubicShapeKindName(CubicShapeKind::cusp) == "cusp");
}

// MARK: - deCasteljauPoints, lookupTable

TEST(deCasteljauPointsAtAQuarterMatchTheGolden) {
    // golden: levels at t = 0.25 on the arch
    std::array<Vector2, 10> points = arch().deCasteljauPoints(0.25f);
    EXPECT(isClose(points[4], Vector2(0, 25)));
    EXPECT(isClose(points[5], Vector2(25, 100)));
    EXPECT(isClose(points[6], Vector2(100, 75)));
    EXPECT(isClose(points[7], Vector2(6.25f, 43.75f)));
    EXPECT(isClose(points[8], Vector2(43.75f, 93.75f)));
    EXPECT(isClose(points[9], Vector2(15.625f, 56.25f)));
}

TEST(deCasteljauPointsHoldTheControlsAndEndAtTheCurvePoint) {
    std::array<Vector2, 10> points = sCurve().deCasteljauPoints(0.7f);
    EXPECT(points[0] == sCurve().start && points[3] == sCurve().end);
    EXPECT(isClose(points[9], sCurve().point(0.7f)));
    // The first half of the split is made of points 0, 4, 7, 9.
    CubicBezierSplit<Vector2> halves = sCurve().split(0.7f);
    EXPECT(isClose(halves.first.startHandle, points[4]) && isClose(halves.first.endHandle, points[7]));
    EXPECT(isClose(halves.second.startHandle, points[8]) && isClose(halves.second.endHandle, points[6]));
}

TEST(aLookupTableHasStepsPlusOnePointsOnTheCurve) {
    std::vector<Vector2> table = arch().lookupTable(4);
    EXPECT(table.size() == 5);
    EXPECT(isClose(table[0], Vector2(0, 0)));
    EXPECT(isClose(table[2], Vector2(50, 75)));
    EXPECT(isClose(table[4], Vector2(100, 0)));
    for (size_t index = 0; index < table.size(); index += 1) {
        EXPECT(isClose(table[index], arch().point(static_cast<float>(index) / 4)));
    }
    EXPECT_PROBLEM(arch().lookupTable(0), "at least one step");
}

// MARK: - tangentDirection, unitNormal, osculatingCircle

TEST(theArchsTangentsMatchTheGolden) {
    // golden: (0, 1) at 0, (0.6, 0.8) at 0.25, (1, 0) at 0.5
    EXPECT(isClose(arch().tangentDirection(0), Vector2(0, 1)));
    EXPECT(isClose(arch().tangentDirection(0.25f), Vector2(0.6f, 0.8f)));
    EXPECT(isClose(arch().tangentDirection(0.5f), Vector2(1, 0)));
}

TEST(aHandleOnItsNodeStillHasADirection) {
    Cubic cornerStart(Vector2(0, 0), Vector2(0, 0), Vector2(50, 0), Vector2(100, 0));
    EXPECT(isClose(cornerStart.tangentDirection(0), Vector2(1, 0)));
    Cubic point(Vector2(3, 3), Vector2(3, 3), Vector2(3, 3), Vector2(3, 3));
    EXPECT(point.tangentDirection(0.5f) == Vector2());
}

TEST(theNormalIsAUnitVectorAQuarterTurnLeftOfTheTangent) {
    for (float parameter : {0.0f, 0.25f, 0.5f, 0.8f, 1.0f}) {
        Vector2 tangent = sCurve().tangentDirection(parameter);
        Vector2 normal = sCurve().unitNormal(parameter);
        EXPECT(isApproximatelyEqual(normal.length(), 1, 1e-4f));
        EXPECT(isApproximatelyEqual(tangent.dot(normal), 0, 1e-4f));
        EXPECT(tangent.cross(normal) > 0);
    }
    EXPECT(isClose(arch().unitNormal(0.5f), Vector2(0, 1)));
}

TEST(theOsculatingCircleOfTheArchMatchesTheGolden) {
    // golden: t = 0: centre (150, 0), r = 150, k = -1/150; t = 0.5: centre (50, 37.5), r = 37.5
    OsculatingCircle atStart = arch().osculatingCircle(0);
    EXPECT(atStart.centre && isClose(*atStart.centre, Vector2(150, 0), 1e-2f));
    EXPECT(isApproximatelyEqual(atStart.radius, 150, 1e-2f));
    EXPECT(isApproximatelyEqual(atStart.curvature, -1.0f / 150, 1e-6f));
    OsculatingCircle atTop = arch().osculatingCircle(0.5f);
    EXPECT(atTop.centre && isClose(*atTop.centre, Vector2(50, 37.5f), 1e-2f));
    EXPECT(isApproximatelyEqual(atTop.radius, 37.5f, 1e-3f));
}

TEST(theOsculatingCentreIsOneRadiusFromThePointAlongTheNormal) {
    for (float parameter : {0.1f, 0.3f, 0.6f, 0.9f}) {
        OsculatingCircle circle = arch().osculatingCircle(parameter);
        EXPECT(circle.centre.has_value());
        if (circle.centre) EXPECT(isApproximatelyEqual(circle.centre->distance(arch().point(parameter)), circle.radius, 1e-2f));
    }
}

TEST(aStraightCurveHasNoOsculatingCentre) {
    Cubic line(Vector2(0, 0), Vector2(1, 0), Vector2(2, 0), Vector2(3, 0));
    OsculatingCircle circle = line.osculatingCircle(0.5f);
    EXPECT(!circle.centre && std::isinf(circle.radius) && circle.curvature == 0);
}

// MARK: - cutAtLengthShares, splitIntoEqualLengths, splitAtEvenParameters

TEST(cuttingAtQuarterAndHalfOfTheArchsLengthMatchesTheGolden) {
    // golden: cutAt(arch, [0.25, 0.5]) joins at (10.5893, 48.3524) and (50, 75)
    std::vector<Cubic> pieces = arch().cutAtLengthShares({0.25f, 0.5f});
    EXPECT(pieces.size() == 3);
    if (pieces.size() != 3) return;
    EXPECT(isClose(pieces[0].end, Vector2(10.5893f, 48.3524f), 0.05f));
    EXPECT(isClose(pieces[1].end, Vector2(50, 75), 0.05f));
    EXPECT(pieces[0].end == pieces[1].start && pieces[1].end == pieces[2].start);
    EXPECT(isApproximatelyEqual(pieces[0].totalLength() / arch().totalLength(), 0.25f, 1e-3f));
}

TEST(theCutPiecesAddUpToTheCurveAndRunInOrder) {
    float total = 0;
    for (const Cubic& piece : sCurve().cutAtLengthShares({0.1f, 0.4f, 0.4f, 0.9f})) total += piece.totalLength();
    EXPECT(isApproximatelyEqual(total, sCurve().totalLength(), 0.05f));
    EXPECT(arch().cutAtLengthShares({}).size() == 1);
}

TEST(aMetricWeighsTheCutsByLengthInThatMetric) {
    // A straight line along (3, 4): a metric of (1, 0) sees only its x, so its half is still the middle.
    Cubic line = Cubic(Vector2(0, 0), Vector2(1, 4.0f / 3), Vector2(2, 8.0f / 3), Vector2(3, 4));
    std::vector<Cubic> halves = line.cutAtLengthShares({0.5f}, Vector2(1, 0));
    EXPECT(halves.size() == 2);
    if (halves.size() == 2) EXPECT(isClose(halves[0].end, Vector2(1.5f, 2), 1e-2f));
}

TEST(sharesOutOfOrderOrRangeAreProblems) {
    EXPECT_PROBLEM(arch().cutAtLengthShares({0.6f, 0.4f}), "ascend");
    EXPECT_PROBLEM(arch().cutAtLengthShares({1.2f}), "ascend");
}

TEST(equalLengthPiecesHaveEqualLengths) {
    std::vector<Cubic> pieces = arch().splitIntoEqualLengths(4);
    EXPECT(pieces.size() == 4);
    float quarter = arch().totalLength() / 4;
    for (const Cubic& piece : pieces) EXPECT(isApproximatelyEqual(piece.totalLength(), quarter, 0.05f));
    EXPECT(pieces.front().start == arch().start && pieces.back().end == arch().end);
    EXPECT_PROBLEM(arch().splitIntoEqualLengths(0), "at least one piece");
}

TEST(evenParameterPiecesMeetAtThirdsOfTheParameter) {
    // golden: splitEven(arch, 3) joins at (25.9259, 66.6667) and (74.0741, 66.6667), the points at 1/3 and 2/3
    std::vector<Cubic> pieces = arch().splitAtEvenParameters(3);
    EXPECT(pieces.size() == 3);
    if (pieces.size() != 3) return;
    EXPECT(isClose(pieces[0].end, Vector2(25.9259f, 66.6667f)));
    EXPECT(isClose(pieces[1].end, Vector2(74.0741f, 66.6667f)));
    EXPECT(isClose(pieces[0].end, arch().point(1.0f / 3)));
}

// MARK: - nearestPoint, nearest

TEST(theNearestPointToTheArchsMiddleIsItsTop) {
    // golden: from (50, 50): t = 0.5, point (50, 75), distance 25
    PathPoint2 found = arch().nearestPoint(Vector2(50, 50));
    EXPECT(found.pieceIndex == 0);
    EXPECT(isApproximatelyEqual(found.parameter, 0.5f, 1e-3f));
    EXPECT(isClose(found.point, Vector2(50, 75), 1e-2f));
    EXPECT(isApproximatelyEqual(found.distance, 25, 1e-2f));
}

TEST(nearestPointsFromOutsideMatchTheGolden) {
    // golden: from (-20, 30): t = 0.0931456, point (2.4412, 25.3408), d = 22.9198
    PathPoint2 left = arch().nearestPoint(Vector2(-20, 30));
    EXPECT(isApproximatelyEqual(left.parameter, 0.0931456f, 1e-3f));
    EXPECT(isClose(left.point, Vector2(2.4412f, 25.3408f), 1e-2f));
    EXPECT(isApproximatelyEqual(left.distance, 22.9198f, 1e-2f));
    // golden: from (130, 120): t = 0.69557, d = 76.8769
    PathPoint2 above = arch().nearestPoint(Vector2(130, 120));
    EXPECT(isApproximatelyEqual(above.parameter, 0.69557f, 1e-3f));
    EXPECT(isApproximatelyEqual(above.distance, 76.8769f, 1e-2f));
}

TEST(theVectorFromTheNearestPointIsAcrossTheTangent) {
    for (Vector2 target : {Vector2(-20, 30), Vector2(130, 120), Vector2(60, 10), Vector2(40, 200)}) {
        PathPoint2 found = arch().nearestPoint(target);
        bool atEnd = found.parameter <= 0 || found.parameter >= 1;
        if (!atEnd) {
            EXPECT(isApproximatelyEqual((target - found.point).normalized().dot(arch().tangentDirection(found.parameter)), 0, 1e-2f));
        }
        // No sampled point of the curve is nearer.
        for (Vector2 sample : arch().lookupTable(200)) EXPECT(sample.distance(target) >= found.distance - 1e-2f);
    }
}

TEST(aPointOnTheCurveIsItsOwnNearestPoint) {
    Vector2 onCurve = sCurve().point(0.37f);
    PathPoint2 found = sCurve().nearestPoint(onCurve);
    EXPECT(isApproximatelyEqual(found.parameter, 0.37f, 2e-3f));
    EXPECT(found.distance < 0.05f);
}

TEST(theNearestOfSeveralCurvesNamesTheCurve) {
    std::vector<Cubic> curves = {arch(), Cubic(Vector2(0, 200), Vector2(30, 200), Vector2(60, 200), Vector2(90, 200))};
    PathPoint2 found = Cubic::nearest(curves, Vector2(45, 190));
    EXPECT(found.pieceIndex == 1);
    EXPECT(isClose(found.point, Vector2(45, 200), 1e-2f));
    EXPECT(isApproximatelyEqual(found.distance, 10, 1e-2f));
    EXPECT(Cubic::nearest(curves, Vector2(50, 50)).pieceIndex == 0);
    EXPECT_PROBLEM(Cubic::nearest({}, Vector2()), "no curve");
}

// MARK: - quadraticThrough, cubicThrough, molded

TEST(aQuadraticThroughThreePointsMatchesTheGolden) {
    // golden: quadThrough((0,0), (40,60), (100,0)) has handles (25.3592, 80.5307), (58.6926, 80.5307)
    Cubic curve = Cubic::quadraticThrough(Vector2(0, 0), Vector2(40, 60), Vector2(100, 0));
    EXPECT(isClose(curve.startHandle, Vector2(25.3592f, 80.5307f), 1e-2f));
    EXPECT(isClose(curve.endHandle, Vector2(58.6926f, 80.5307f), 1e-2f));
    // golden with t = 0.3: handles (49.2063, 95.2381), (82.5397, 95.2381)
    Cubic withParameter = Cubic::quadraticThrough(Vector2(0, 0), Vector2(40, 60), Vector2(100, 0), 0.3f);
    EXPECT(isClose(withParameter.startHandle, Vector2(49.2063f, 95.2381f), 1e-2f));
    EXPECT(isClose(withParameter.endHandle, Vector2(82.5397f, 95.2381f), 1e-2f));
}

TEST(theQuadraticPassesThroughThePointAtItsParameter) {
    Cubic curve = Cubic::quadraticThrough(Vector2(0, 0), Vector2(40, 60), Vector2(100, 0), 0.3f);
    EXPECT(isClose(curve.point(0.3f), Vector2(40, 60), 1e-2f));
    // And it is a true quadratic: its handles sit two thirds along to one control point.
    EXPECT(isApproximatelyEqual(curve.startHandle.y, curve.endHandle.y, 1e-3f));
}

TEST(aCubicThroughThreePointsMatchesTheGolden) {
    // golden: cubicThrough((0,0), (40,60), (100,0)) has handles (13.2337, 92.6234), (72.9607, 66.3012)
    Cubic curve = Cubic::cubicThrough(Vector2(0, 0), Vector2(40, 60), Vector2(100, 0));
    EXPECT(isClose(curve.startHandle, Vector2(13.2337f, 92.6234f), 2e-2f));
    EXPECT(isClose(curve.endHandle, Vector2(72.9607f, 66.3012f), 2e-2f));
    // golden with t = 0.4: handles (23.4873, 111.1111), (81.4357, 41.6667)
    ThroughOptions withParameter;
    withParameter.parameter = 0.4f;
    Cubic atFour = Cubic::cubicThrough(Vector2(0, 0), Vector2(40, 60), Vector2(100, 0), withParameter);
    EXPECT(isClose(atFour.startHandle, Vector2(23.4873f, 111.1111f), 2e-2f));
    EXPECT(isClose(atFour.endHandle, Vector2(81.4357f, 41.6667f), 2e-2f));
    // golden with strut 20 and direction (1, 0.2): handles (6.8562, 76.8301), (80.465, 84.8852)
    ThroughOptions withDirection;
    withDirection.strutLength = 20;
    withDirection.direction = Vector2(1, 0.2f);
    Cubic steered = Cubic::cubicThrough(Vector2(0, 0), Vector2(40, 60), Vector2(100, 0), withDirection);
    EXPECT(isClose(steered.startHandle, Vector2(6.8562f, 76.8301f), 2e-2f));
    EXPECT(isClose(steered.endHandle, Vector2(80.465f, 84.8852f), 2e-2f));
}

TEST(theCubicPassesThroughThePointAtItsParameterAlongTheDirection) {
    ThroughOptions options;
    options.parameter = 0.4f;
    options.direction = Vector2(1, 0.2f);
    Cubic curve = Cubic::cubicThrough(Vector2(0, 0), Vector2(40, 60), Vector2(100, 0), options);
    EXPECT(isClose(curve.point(0.4f), Vector2(40, 60), 1e-2f));
    EXPECT(isClose(curve.tangentDirection(0.4f), Vector2(1, 0.2f).normalized(), 1e-3f));
    EXPECT(curve.start == Vector2(0, 0) && curve.end == Vector2(100, 0));
}

TEST(moldingTheArchsTopMatchesTheGolden) {
    // golden: drag the arch's point at 0.5 to (60, 40): handles (13.3333, 53.3333), (113.3333, 53.3333)
    Cubic molded = arch().molded(0.5f, Vector2(60, 40));
    EXPECT(isClose(molded.startHandle, Vector2(13.3333f, 53.3333f), 1e-2f));
    EXPECT(isClose(molded.endHandle, Vector2(113.3333f, 53.3333f), 1e-2f));
    EXPECT(molded.start == arch().start && molded.end == arch().end);
}

TEST(aMoldedCurvePassesThroughTheDestination) {
    for (float parameter : {0.2f, 0.5f, 0.7f}) {
        Vector2 destination(30 + 40 * parameter, 20);
        Cubic molded = sCurve().molded(parameter, destination);
        EXPECT(isClose(molded.point(parameter), destination, 1e-2f));
    }
}

TEST(moldingToTheCurvesOwnPointChangesNothing) {
    Cubic same = sCurve().molded(0.4f, sCurve().point(0.4f));
    EXPECT(isClose(same.startHandle, sCurve().startHandle, 1e-2f));
    EXPECT(isClose(same.endHandle, sCurve().endHandle, 1e-2f));
}

// MARK: - arcCubic, arcBetween

TEST(aQuarterCircleFromTangentsHasTheClassicHandles) {
    // golden: (0,0) heading up to (100,100) heading right: handles 55.2285 long
    Cubic quarter = Cubic::arcCubic(Vector2(0, 0), Vector2(0, 1), Vector2(100, 100), Vector2(1, 0));
    EXPECT(isClose(quarter.startHandle, Vector2(0, 55.2285f), 1e-2f));
    EXPECT(isClose(quarter.endHandle, Vector2(44.7715f, 100), 1e-2f));
}

TEST(aNoTurnArcIsAThirdChordLine) {
    Cubic line = Cubic::arcCubic(Vector2(0, 0), Vector2(1, 0), Vector2(90, 0), Vector2(1, 0));
    EXPECT(isClose(line.startHandle, Vector2(30, 0)) && isClose(line.endHandle, Vector2(60, 0)));
}

TEST(aQuarterTurnArcBetweenEndsIsOneCubicOnTheCircle) {
    // From (0,0) to (2,0) leaving 45 degrees above the chord: the circle round (1,-1) of radius sqrt(2).
    std::vector<Cubic> arcs = Cubic::arcBetween(Vector2(0, 0), Vector2(2, 0), Angle::degrees(45));
    EXPECT(arcs.size() == 1);
    if (arcs.size() != 1) return;
    EXPECT(arcs[0].start == Vector2(0, 0) && arcs[0].end == Vector2(2, 0));
    EXPECT(isApproximatelyEqual(arcs[0].point(0.5f).distance(Vector2(1, -1)), std::sqrt(2.0f), 2e-3f));
    EXPECT(isClose(arcs[0].tangentDirection(0), Vector2(std::sqrt(0.5f), std::sqrt(0.5f)), 1e-3f));
    EXPECT(arcs[0].point(0.5f).y > 0); // it bulges up, above the chord
}

TEST(aNegativeAngleBulgesTheOtherWay) {
    std::vector<Cubic> up = Cubic::arcBetween(Vector2(0, 0), Vector2(2, 0), Angle::degrees(45));
    std::vector<Cubic> down = Cubic::arcBetween(Vector2(0, 0), Vector2(2, 0), Angle::degrees(-45));
    EXPECT(up.size() == 1 && down.size() == 1);
    if (up.size() != 1 || down.size() != 1) return;
    // The mirror image in the chord: same x, opposite y.
    EXPECT(isClose(up[0].point(0.5f), Vector2(1, -down[0].point(0.5f).y), 1e-3f));
    EXPECT(down[0].point(0.5f).y < 0);
}

TEST(aLongArcHasSeveralPiecesAllOnOneCircle) {
    // Leaving the chord at 120 degrees turns by 240: three quarter-or-less pieces, all on one circle.
    std::vector<Cubic> arcs = Cubic::arcBetween(Vector2(0, 0), Vector2(2, 0), Angle::degrees(120));
    EXPECT(arcs.size() == 3);
    float radius = 2 / (2 * std::sin(Angle::degrees(120).radians));
    for (size_t index = 0; index + 1 < arcs.size(); index += 1) EXPECT(isClose(arcs[index].end, arcs[index + 1].start, 1e-4f));
    EXPECT(arcs.front().start == Vector2(0, 0) && arcs.back().end == Vector2(2, 0));
    // Every piece's middle is one radius from the centre found from the first piece's start and tangent.
    Vector2 tangent = arcs[0].tangentDirection(0);
    Vector2 circleCentre = arcs[0].start + tangent.rightNormal() * radius;
    for (const Cubic& piece : arcs) EXPECT(isApproximatelyEqual(piece.point(0.5f).distance(circleCentre), radius, radius * 3e-3f));
}

TEST(impossibleArcsAreProblems) {
    EXPECT_PROBLEM(Cubic::arcBetween(Vector2(0, 0), Vector2(2, 0), Angle::zero()), "non-zero");
    EXPECT_PROBLEM(Cubic::arcBetween(Vector2(0, 0), Vector2(0, 0), Angle::degrees(30)), "two different");
    EXPECT_PROBLEM(Cubic::arcBetween(Vector2(0, 0), Vector2(2, 0), Angle::degrees(180)), "half turn");
}
