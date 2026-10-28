// Tests of Curves/Boolean2D (unite, intersect, subtract, exclude, cut under a fill rule), Curves/Shape2D
// measurements, and the polyline helpers. Hand-worked values come from squares of side 100; the circle
// cases are transcribed from graphics_cpp Curves2D/tests/golden/boolean.json (a square [0,100]² against a
// circle of radius 40 round (100, 50)), compared by area at a tolerance of 1e-3 because the outlines come out
// in another order and with other start points.
#include <cmath>
#include <vector>

#include "../Curves/Boolean2D.h"
#include "../Curves/Shape2D+Pieces.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

Path2D rectanglePath(float left, float bottom, float right, float top, bool isCounterclockwise = true) {
    std::vector<Vector2> corners{{left, bottom}, {right, bottom}, {right, top}, {left, top}};
    if (!isCounterclockwise) std::swap(corners[1], corners[3]);
    Path2D path(corners[0]);
    for (size_t index = 1; index < 4; index += 1) path.addLine(corners[index]);
    path.close();
    return path;
}

Path2D circlePath(Vector2 centre, float radius) {
    Path2D path(centre + Vector2(radius, 0));
    path.addArc(centre, radius, Angle::zero(), Angle::degrees(360));
    path.close();
    return path;
}

Shape2D shapeOf(std::vector<Path2D> outlines, FillRule fillRule = FillRule::nonzero) {
    return Shape2D{std::move(outlines), fillRule};
}

Shape2D square() { return shapeOf({rectanglePath(0, 0, 100, 100)}); }
Shape2D shiftedSquare() { return shapeOf({rectanglePath(50, 50, 150, 150)}); }
Shape2D circle() { return shapeOf({circlePath(Vector2(100, 50), 40)}); }
Shape2D ring() { return shapeOf({rectanglePath(0, 0, 100, 100), rectanglePath(25, 25, 75, 75, false)}); }

bool isNear(float value, float expected, float relative = 1e-3f) {
    return std::fabs(value - expected) <= relative * std::max(1.0f, std::fabs(expected));
}

/// The result of `operation`, which must not fail.
Shape2D combine(const Shape2D& first, const Shape2D& second, BooleanOperation operation) {
    std::optional<Shape2D> result = tryBooleanShapes(first, second, operation);
    EXPECT(result.has_value());
    return result.value_or(Shape2D{});
}

} // namespace

// MARK: Shape2D

TEST(shapeAreaIsTheSumOfItsOutlines) {
    EXPECT(isNear(ring().signedArea(), 7500));
    EXPECT(isNear(ring().reversed().signedArea(), -7500));
}

TEST(shapeContainsFollowsTheFillRule) {
    Shape2D doubled = shapeOf({rectanglePath(0, 0, 100, 100), rectanglePath(25, 25, 75, 75)});
    EXPECT(doubled.contains(Vector2(50, 50)));
    doubled.fillRule = FillRule::evenOdd;
    EXPECT(!doubled.contains(Vector2(50, 50)));
    EXPECT(doubled.contains(Vector2(10, 10)));
    EXPECT(!ring().contains(Vector2(50, 50)));
    EXPECT(ring().contains(Vector2(10, 10)));
    EXPECT(ring().windingNumber(Vector2(50, 50)) == 0);
    EXPECT(ring().windingNumber(Vector2(10, 10)) == 1);
}

TEST(shapeBoundsCoverEveryOutline) {
    Bounds2 bounds = shapeOf({rectanglePath(0, 0, 100, 100), rectanglePath(500, 500, 510, 510)}).bounds();
    EXPECT(bounds.minimum == Vector2(0, 0) && bounds.maximum == Vector2(510, 510));
    EXPECT_PROBLEM(Shape2D{}.bounds(), "no bounds");
}

TEST(reversedPathRunsTheOtherWayRound) {
    Path2D square = rectanglePath(0, 0, 100, 100);
    Path2D backwards = reversedPath(square);
    EXPECT(isNear(backwards.signedArea(), -10000));
    EXPECT(backwards.isClosed());
    EXPECT(backwards.start == Vector2(0, 0));
    EXPECT(backwards.segments().front().end() == Vector2(0, 100));
    EXPECT(isNear(reversedPath(backwards).signedArea(), 10000));
    EXPECT(reversedPath(backwards).segments().size() == 4);
    // A curve keeps its shape: the handles swap places.
    Path2D curve(Vector2(0, 0));
    curve.addCubic(Vector2(10, 20), Vector2(30, 20), Vector2(40, 0));
    Path2D back = reversedPath(curve);
    EXPECT(back.start == Vector2(40, 0) && !back.isClosed());
    EXPECT(back.segments()[0].startHandle == Vector2(30, 20) && back.segments()[0].endHandle == Vector2(10, 20));
}

TEST(orientedForFillMakesTheBiggestOutlinePositiveAndTheRestHoles) {
    Shape2D mixed = shapeOf({rectanglePath(0, 0, 100, 100, false), rectanglePath(25, 25, 75, 75)});
    Shape2D oriented = mixed.orientedForFill();
    EXPECT(isNear(oriented.outlines[0].signedArea(), 10000));
    EXPECT(isNear(oriented.outlines[1].signedArea(), -2500));
    EXPECT(isNear(oriented.signedArea(), 7500));
    EXPECT(oriented.orientedForFill() == oriented);
}

// MARK: Two squares

TEST(uniteOfOverlappingSquaresIsOneOutlineOfTheCombinedArea) {
    Shape2D result = combine(square(), shiftedSquare(), BooleanOperation::unite);
    EXPECT(result.outlines.size() == 1);
    EXPECT(isNear(result.signedArea(), 17500));
    // Golden (boolean.json case 35): 8 corners, 0…150 across.
    EXPECT(result.outlines[0].segments().size() == 8);
    Bounds2 bounds = result.bounds();
    EXPECT(bounds.minimum == Vector2(0, 0) && bounds.maximum == Vector2(150, 150));
    EXPECT(result.fillRule == FillRule::nonzero);
}

TEST(intersectOfOverlappingSquaresIsTheOverlap) {
    Shape2D result = combine(square(), shiftedSquare(), BooleanOperation::intersect);
    EXPECT(result.outlines.size() == 1);
    EXPECT(isNear(result.signedArea(), 2500));
    Bounds2 bounds = result.bounds();
    EXPECT(bounds.minimum == Vector2(50, 50) && bounds.maximum == Vector2(100, 100));
}

TEST(subtractOfOverlappingSquaresIsAnLShape) {
    Shape2D result = combine(square(), shiftedSquare(), BooleanOperation::subtract);
    EXPECT(result.outlines.size() == 1);
    EXPECT(isNear(result.signedArea(), 7500));
    EXPECT(result.outlines[0].segments().size() == 6);
    EXPECT(result.contains(Vector2(10, 10)) && !result.contains(Vector2(75, 75)));
}

TEST(excludeOfOverlappingSquaresIsTheTwoLShapesWithoutTheOverlap) {
    // The two L shapes meet at two corner points, so the outline may be one pinched loop (as here) or
    // the union with the overlap as a hole (as graphics_cpp chooses): the fill is the same.
    Shape2D result = combine(square(), shiftedSquare(), BooleanOperation::exclude);
    EXPECT(isNear(result.signedArea(), 15000));
    EXPECT(!result.contains(Vector2(75, 75)) && result.contains(Vector2(10, 10)) && result.contains(Vector2(140, 140)));
    EXPECT(result.contains(Vector2(75, 25)) && result.contains(Vector2(25, 75)) && !result.contains(Vector2(200, 200)));
}

TEST(cutSplitsAlongTheSecondOutlineAndKeepsEveryPiece) {
    Shape2D result = combine(square(), shiftedSquare(), BooleanOperation::cut);
    EXPECT(result.outlines.size() == 2);
    EXPECT(isNear(result.signedArea(), 10000));
    EXPECT(result.contains(Vector2(75, 75)) && result.contains(Vector2(10, 10)) && !result.contains(Vector2(140, 140)));
}

TEST(everyOperationKeepsOuterOutlinesCounterclockwise) {
    for (BooleanOperation operation :
         {BooleanOperation::unite, BooleanOperation::intersect, BooleanOperation::subtract}) {
        Shape2D result = combine(square(), shiftedSquare(), operation);
        EXPECT(result.outlines.front().signedArea() > 0);
    }
}

TEST(operandsMayRunEitherWayRound) {
    Shape2D clockwise = shapeOf({rectanglePath(50, 50, 150, 150, false)});
    EXPECT(isNear(combine(square(), clockwise, BooleanOperation::unite).signedArea(), 17500));
    EXPECT(isNear(combine(clockwise, square(), BooleanOperation::subtract).signedArea(), 7500));
}

// MARK: Edges that touch, overlap or never meet

TEST(squaresSharingAnEdgeUniteIntoOneRectangle) {
    Shape2D neighbour = shapeOf({rectanglePath(100, 0, 200, 100)});
    Shape2D result = combine(square(), neighbour, BooleanOperation::unite);
    EXPECT(result.outlines.size() == 1);
    EXPECT(isNear(result.signedArea(), 20000));
    // Golden (case 37): the shared edge is gone and the straight sides are joined: 4 pieces.
    EXPECT(result.outlines[0].segments().size() == 4);
    EXPECT(combine(square(), neighbour, BooleanOperation::intersect).outlines.empty());
    EXPECT(isNear(combine(square(), neighbour, BooleanOperation::subtract).signedArea(), 10000));
    EXPECT(isNear(combine(square(), neighbour, BooleanOperation::exclude).signedArea(), 20000));
}

TEST(identicalShapesUniteToOneCopyAndSubtractToNothing) {
    EXPECT(isNear(combine(square(), square(), BooleanOperation::unite).signedArea(), 10000));
    EXPECT(combine(square(), square(), BooleanOperation::unite).outlines.size() == 1);
    EXPECT(isNear(combine(square(), square(), BooleanOperation::intersect).signedArea(), 10000));
    EXPECT(combine(square(), square(), BooleanOperation::subtract).outlines.empty());
    EXPECT(combine(square(), square(), BooleanOperation::exclude).outlines.empty());
}

TEST(squaresThatNeverMeetStayApart) {
    Shape2D far = shapeOf({rectanglePath(500, 500, 510, 510)});
    Shape2D united = combine(square(), far, BooleanOperation::unite);
    EXPECT(united.outlines.size() == 2 && isNear(united.signedArea(), 10100));
    EXPECT(combine(square(), far, BooleanOperation::intersect).outlines.empty());
    EXPECT(isNear(combine(square(), far, BooleanOperation::subtract).signedArea(), 10000));
}

TEST(aSquareInsideAnotherUnitesToTheBigOneAndSubtractsToARing) {
    Shape2D inner = shapeOf({rectanglePath(25, 25, 75, 75)});
    EXPECT(isNear(combine(square(), inner, BooleanOperation::unite).signedArea(), 10000));
    Shape2D punched = combine(square(), inner, BooleanOperation::subtract);
    EXPECT(punched.outlines.size() == 2);
    EXPECT(isNear(punched.signedArea(), 7500));
    EXPECT(!punched.contains(Vector2(50, 50)));
}

TEST(aSharedCornerAloneChangesNothing) {
    Shape2D corner = shapeOf({rectanglePath(100, 100, 200, 200)});
    Shape2D result = combine(square(), corner, BooleanOperation::unite);
    EXPECT(isNear(result.signedArea(), 20000));
    EXPECT(combine(square(), corner, BooleanOperation::intersect).outlines.empty());
}

// MARK: Curves (golden: boolean.json, a circle of radius 40 round (100, 50))

TEST(squareAndCircleGiveTheGoldenAreas) {
    EXPECT(isNear(combine(square(), circle(), BooleanOperation::unite).signedArea(), 12513.95f));
    EXPECT(isNear(combine(square(), circle(), BooleanOperation::intersect).signedArea(), 2513.94f));
    EXPECT(isNear(combine(square(), circle(), BooleanOperation::subtract).signedArea(), 7486.05f));
    EXPECT(isNear(combine(circle(), square(), BooleanOperation::subtract).signedArea(), 2513.95f));
    Shape2D exclusive = combine(square(), circle(), BooleanOperation::exclude);
    EXPECT(isNear(exclusive.signedArea(), 12513.95f - 2513.94f));
}

TEST(squareAndCircleCurvesAreKeptAsCurves) {
    Shape2D result = combine(square(), circle(), BooleanOperation::unite);
    EXPECT(result.outlines.size() == 1);
    // Golden: 4 lines and 2 cubics and one more line piece.
    int curves = 0;
    for (const Path2D::Segment& segment : result.outlines[0].segments()) curves += segment.isLine() ? 0 : 1;
    EXPECT(curves == 2 || curves == 3);
    Bounds2 bounds = result.bounds();
    EXPECT(isNear(bounds.maximum.x, 140) && isNear(bounds.minimum.x, 0));
    EXPECT(result.contains(Vector2(130, 50)) && !result.contains(Vector2(130, 100)));
}

TEST(circleOutsideAndInsideTheSquare) {
    Shape2D smallCircle = shapeOf({circlePath(Vector2(50, 50), 20)});
    EXPECT(isNear(combine(square(), smallCircle, BooleanOperation::unite).signedArea(), 10000));
    EXPECT(isNear(combine(square(), smallCircle, BooleanOperation::intersect).signedArea(), 1256.64f, 2e-3f));
    Shape2D bitten = combine(square(), smallCircle, BooleanOperation::subtract);
    EXPECT(bitten.outlines.size() == 2);
    EXPECT(isNear(bitten.signedArea(), 10000 - 1256.64f, 2e-3f));
}

TEST(twoCirclesMeetingAtTwoPoints) {
    Shape2D first = shapeOf({circlePath(Vector2(0, 0), 50)});
    Shape2D second = shapeOf({circlePath(Vector2(60, 0), 50)});
    float lens = combine(first, second, BooleanOperation::intersect).signedArea();
    // Two circles of radius r with centres d apart share 2r² acos(d/2r) − (d/2)√(4r² − d²).
    float expected = 2 * 2500 * std::acos(0.6f) - 30 * std::sqrt(10000 - 3600.0f);
    EXPECT(isNear(lens, expected, 3e-3f));
    float together = combine(first, second, BooleanOperation::unite).signedArea();
    EXPECT(isNear(together, 2 * 3.14159265f * 2500 - expected, 3e-3f));
}

// MARK: Ring with a hole (golden cases 38, 44, 50)

TEST(ringAndCircleGiveTheGoldenAreas) {
    EXPECT(isNear(combine(ring(), circle(), BooleanOperation::unite).signedArea(), 12513.95f - 1888.82f));
    EXPECT(isNear(combine(ring(), circle(), BooleanOperation::intersect).signedArea(), 1902.78f));
    Shape2D difference = combine(ring(), circle(), BooleanOperation::subtract);
    EXPECT(difference.outlines.size() == 1);
    EXPECT(isNear(difference.signedArea(), 5597.21f));
}

TEST(aHoleStaysAHoleWhenTheOtherOperandMissesIt) {
    Shape2D beside = shapeOf({rectanglePath(120, 0, 160, 40)});
    Shape2D result = combine(ring(), beside, BooleanOperation::unite);
    EXPECT(result.outlines.size() == 3);
    EXPECT(isNear(result.signedArea(), 7500 + 1600));
    EXPECT(!result.contains(Vector2(50, 50)));
}

// MARK: One shape against itself

TEST(unitedRemovesTheOverlapOfOneShape) {
    Shape2D overlapping = shapeOf({rectanglePath(0, 0, 100, 100), rectanglePath(50, 50, 150, 150)});
    Shape2D result = united(overlapping);
    EXPECT(result.outlines.size() == 1);
    EXPECT(isNear(result.signedArea(), 17500));
}

TEST(unitedUnderEvenOddKeepsTheOverlapAsAHole) {
    Shape2D overlapping = shapeOf({rectanglePath(0, 0, 100, 100), rectanglePath(50, 50, 150, 150)}, FillRule::evenOdd);
    Shape2D result = united(overlapping);
    EXPECT(isNear(result.signedArea(), 15000));
    EXPECT(!result.contains(Vector2(75, 75)) && result.contains(Vector2(25, 75)) && result.contains(Vector2(125, 75)));
}

TEST(aHoleDrawnTheSameWayRoundIsFilledByNonzeroAndOpenedByEvenOdd) {
    std::vector<Path2D> outlines{rectanglePath(0, 0, 100, 100), rectanglePath(25, 25, 75, 75)};
    EXPECT(isNear(united(shapeOf(outlines)).signedArea(), 10000));
    EXPECT(isNear(united(shapeOf(outlines, FillRule::evenOdd)).signedArea(), 7500));
}

TEST(uniteOfOneGroupWithACurveSelfOverlapGivesTheGoldenOutline) {
    // Golden case 64: the circle against the square, group 0 only.
    std::vector<Shape2D> groups{circle(), shiftedSquare()};
    BooleanRule firstGroupOnly = [](const std::vector<bool>& inside) { return inside[0]; };
    std::optional<Shape2D> result = tryBooleanByRule(groups, firstGroupOnly);
    EXPECT(result.has_value());
    EXPECT(isNear(result->signedArea(), 5027.89f));
}

// MARK: Rules over several groups, tracing, failures

TEST(aRuleOverThreeGroups) {
    // In the first and in exactly one of the others.
    std::vector<Shape2D> groups{square(), shiftedSquare(), shapeOf({rectanglePath(0, 50, 50, 100)})};
    BooleanRule rule = [](const std::vector<bool>& inside) { return inside[0] && (inside[1] != inside[2]); };
    std::optional<Shape2D> result = tryBooleanByRule(groups, rule);
    EXPECT(result.has_value());
    // The overlap with the second square (2500) plus the third square (2500); the two do not meet.
    EXPECT(isNear(result->signedArea(), 5000));
}

TEST(tracedBooleanSaysWhichOperandEachPieceCameFrom) {
    std::vector<Shape2D> groups{square(), shiftedSquare()};
    BooleanRule intersection = [](const std::vector<bool>& inside) { return inside[0] && inside[1]; };
    std::optional<TracedBoolean> traced = traceBoolean(groups, intersection);
    EXPECT(traced.has_value());
    if (!traced) return;
    EXPECT(traced->shape.outlines.size() == 1 && traced->sourceGroups.size() == 1);
    std::vector<int> sources = traced->sourceGroups[0];
    EXPECT(sources.size() == traced->shape.outlines[0].segments().size());
    // The overlap is bounded by two sides of each square.
    int fromFirst = 0;
    for (int source : sources) fromFirst += source == 0 ? 1 : 0;
    EXPECT(fromFirst == 2 && sources.size() == 4);
}

TEST(booleanReturnsTheInputsWhenTheOperandsAreEmpty) {
    std::vector<Shape2D> groups{Shape2D{}, square()};
    BooleanRule everything = [](const std::vector<bool>& inside) { return inside[0] || inside[1]; };
    EXPECT(isNear(booleanByRule(groups, everything).signedArea(), 10000));
    EXPECT(booleanByRule({}, everything).outlines.empty());
}

TEST(aToleranceThatIsNotPositiveIsAProblem) {
    EXPECT_PROBLEM(tryBooleanShapes(square(), shiftedSquare(), BooleanOperation::unite, 0.0f), "tolerance");
    EXPECT_PROBLEM(tryBooleanShapes(square(), shiftedSquare(), BooleanOperation::unite, -1.0f), "tolerance");
}

TEST(aGivenToleranceOfAQuarterUnitGivesTheSameSquares) {
    std::optional<Shape2D> result = tryBooleanShapes(square(), shiftedSquare(), BooleanOperation::unite, 0.25f);
    EXPECT(result.has_value() && isNear(result->signedArea(), 17500));
}

TEST(smallShapesWorkAtTheirOwnScale) {
    Shape2D tiny = shapeOf({rectanglePath(0, 0, 0.01f, 0.01f)});
    Shape2D shifted = shapeOf({rectanglePath(0.005f, 0.005f, 0.015f, 0.015f)});
    EXPECT(isNear(combine(tiny, shifted, BooleanOperation::unite).signedArea(), 0.000175f, 5e-3f));
}

TEST(outlinePiecesOfAnOpenPathIncludeTheClosingLine) {
    Path2D open(Vector2(0, 0));
    open.addLine(Vector2(10, 0));
    open.addLine(Vector2(10, 10));
    std::vector<OutlinePiece> pieces = outlinePieces(open);
    EXPECT(pieces.size() == 3 && pieces[2].end() == Vector2(0, 0));
    EXPECT(pathOfPieces(pieces, true).isClosed());
    EXPECT_PROBLEM(pathOfPieces({}, true), "at least one");
}

// MARK: Polyline helpers

TEST(scanlineCrossingsOfASquareAreItsTwoSidesWithOppositeWinding) {
    std::vector<Segment2> edges = scanlineEdges({{{0, 0}, {100, 0}, {100, 100}, {0, 100}}});
    EXPECT(edges.size() == 2);
    std::vector<ScanCrossing> crossings = scanlineCrossings(edges, 50);
    EXPECT(crossings.size() == 2);
    EXPECT(isApproximatelyEqual(crossings[0].x, 100) && crossings[0].winding == 1);
    EXPECT(isApproximatelyEqual(crossings[1].x, 0) && crossings[1].winding == -1);
    EXPECT(scanlineCrossings(edges, 150).empty());
    // A vertex is crossed once: the lower end counts, the upper one does not.
    EXPECT(scanlineCrossings(edges, 0).size() == 2 && scanlineCrossings(edges, 100).empty());
}

TEST(scanlineCrossingsInterpolateAlongASlantedSide) {
    std::vector<Segment2> edges = scanlineEdges({{{0, 0}, {40, 0}, {0, 20}}});
    std::vector<ScanCrossing> crossings = scanlineCrossings(edges, 10);
    EXPECT(crossings.size() == 2);
    float largest = std::max(crossings[0].x, crossings[1].x);
    EXPECT(isApproximatelyEqual(largest, 20));
}

TEST(narrowestPointOfARectangleIsItsShortSide) {
    std::vector<Vector2> rectangle{{0, 0}, {10, 0}, {10, 2}, {0, 2}};
    std::optional<ClosestPair> pair = narrowestPoint(rectangle, 3);
    EXPECT(pair.has_value() && isApproximatelyEqual(pair->distance, 10));
    // Nothing is that far apart along a triangle's three corners.
    EXPECT(!narrowestPoint({{0, 0}, {1, 0}, {0, 1}}, 100).has_value());
    // With neighbours allowed the short side is found.
    std::optional<ClosestPair> short_ = narrowestPoint(rectangle, 1);
    EXPECT(short_.has_value() && isApproximatelyEqual(short_->distance, 2));
}

TEST(closestPointsBetweenTwoPolylines) {
    ClosestPair gap = closestPointsBetween({{0, 0}, {100, 0}, {100, 100}, {0, 100}}, {{140, 50}, {160, 50}, {150, 80}});
    EXPECT(isApproximatelyEqual(gap.distance, std::sqrt(40.0f * 40 + 50 * 50), 1e-3f) || gap.distance <= 70.0f);
    EXPECT(gap.first == Vector2(100, 0) || gap.first == Vector2(100, 100));
    EXPECT_PROBLEM(closestPointsBetween({}, {{0, 0}}), "needs a point");
    ClosestPair same = closestPointsBetween({{1, 2}}, {{1, 2}});
    EXPECT(same.distance == 0);
}
