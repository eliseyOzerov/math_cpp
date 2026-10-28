// Tests of Curves/BooleanTree2D (slice S5): the five operations over an operand tree, the sources of
// the pieces, the corners where operands meet and the outward directions. The overlapping squares
// are the graphics_cpp bool.json golden case: A = [0,100]², B = [50,150]².
#include <cmath>
#include <memory>
#include <vector>

#include "../Curves/BooleanTree2D.h"
#include "../Curves/Shape2D+Pieces.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// The square of side `side` with its lower left corner at `corner`, counterclockwise.
Path2D squareAt(Vector2 corner, float side) {
    Path2D path(corner);
    path.addLine(corner + Vector2(side, 0));
    path.addLine(corner + Vector2(side, side));
    path.addLine(corner + Vector2(0, side));
    path.close();
    return path;
}

BooleanTree2D overlapping(BooleanOperation operation) {
    return BooleanTree2D(operation, {squareAt(Vector2(0, 0), 100), Shape2D{{squareAt(Vector2(50, 50), 100)}, FillRule::nonzero}});
}

/// The sum of the signed areas of the contours (holes count negative).
float totalArea(const std::vector<Path2D>& contours) {
    float area = 0;
    for (const Path2D& contour : contours) area += contour.signedArea();
    return area;
}

} // namespace

TEST(booleanTreeUnionOfOverlappingSquaresIsOneOutlineOfTheRightArea) {
    // 100² + 100² − 50² = 17500, one outline counterclockwise.
    BooleanTree2D tree = overlapping(BooleanOperation::unite);
    EXPECT(tree.ok());
    std::vector<Path2D> contours = tree.contours();
    EXPECT(contours.size() == 1);
    EXPECT(isApproximatelyEqual(totalArea(contours), 17500.0f, 5.0f));
}

TEST(booleanTreeEveryOperationGivesTheAreaWorkedOutByHand) {
    // intersect 2500; subtract 10000 − 2500 = 7500; exclude 17500 − 2500 = 15000 (two L shapes that touch at the two crossings, traced as one loop); cut
    // keeps the 2500 inside and the 7500 outside, 10000 in two outlines.
    EXPECT(isApproximatelyEqual(totalArea(overlapping(BooleanOperation::intersect).contours()), 2500.0f, 5.0f));
    EXPECT(isApproximatelyEqual(totalArea(overlapping(BooleanOperation::subtract).contours()), 7500.0f, 5.0f));
    std::vector<Path2D> exclude = overlapping(BooleanOperation::exclude).contours();
    EXPECT(!exclude.empty());
    EXPECT(isApproximatelyEqual(totalArea(exclude), 15000.0f, 5.0f));
    std::vector<Path2D> cut = overlapping(BooleanOperation::cut).contours();
    EXPECT(cut.size() == 2);
    EXPECT(isApproximatelyEqual(totalArea(cut), 10000.0f, 5.0f));
}

TEST(booleanTreeSourcesNameTheOperandOfEveryPiece) {
    BooleanTree2D tree = overlapping(BooleanOperation::unite);
    std::vector<std::vector<int>> sources = tree.sources();
    EXPECT(sources.size() == 1);
    EXPECT(sources[0].size() == outlinePieces(tree.contours()[0]).size());
    int fromFirst = 0;
    int fromSecond = 0;
    for (int source : sources[0]) {
        EXPECT(source == 0 || source == 1);
        fromFirst += source == 0 ? 1 : 0;
        fromSecond += source == 1 ? 1 : 0;
    }
    // Golden: 8 pieces, 4 from each square.
    EXPECT(sources[0].size() == 8);
    EXPECT(fromFirst == 4 && fromSecond == 4);
}

TEST(booleanTreeCornersWhereTheSquaresMeetAreTheTwoPitsAtTheCrossings) {
    // Golden "corners": pits at (50, 100) and (100, 50), outward (−√½, √½) and (√½, −√½), not convex.
    BooleanTree2D tree = overlapping(BooleanOperation::unite);
    std::vector<BooleanCorner> corners = tree.corners();
    EXPECT(corners.size() == 2);
    for (const BooleanCorner& corner : corners) {
        EXPECT(!corner.isConvex);
        EXPECT((corner.between[0] == 0 && corner.between[1] == 1) || (corner.between[0] == 1 && corner.between[1] == 0));
        if (isClose(corner.point, Vector2(50, 100))) {
            EXPECT(isClose(corner.outward, Vector2(-0.70710678f, 0.70710678f), 1e-4f));
        } else {
            EXPECT(isClose(corner.point, Vector2(100, 50)));
            EXPECT(isClose(corner.outward, Vector2(0.70710678f, -0.70710678f), 1e-4f));
        }
        EXPECT(isClose(tree.outward(corner.contourIndex, corner.nodeIndex), corner.outward, 1e-5f));
    }
    EXPECT(tree.corners(0, 1).size() == 2);
    EXPECT(tree.corners(1, 0).size() == 2);
    EXPECT(tree.corners(0).size() == 2);
    EXPECT(tree.corners(5).empty());
    EXPECT(tree.corners(std::nullopt, std::nullopt, true).size() == 8);
}

TEST(booleanTreeCornersOfASingleSquareAreAllConvexAndPointOutwards) {
    BooleanTree2D tree(BooleanOperation::unite, {squareAt(Vector2(0, 0), 10)});
    std::vector<BooleanCorner> corners = tree.corners(std::nullopt, std::nullopt, true);
    EXPECT(corners.size() == 4);
    for (const BooleanCorner& corner : corners) {
        EXPECT(corner.isConvex);
        // Outward is away from the centre (5, 5), along the diagonal.
        Vector2 away = (corner.point - Vector2(5, 5)).normalized();
        EXPECT(isClose(corner.outward, away, 1e-4f));
    }
}

TEST(booleanTreeWithReplacesTheOperandsAndKeepsTheOperation) {
    BooleanTree2D tree = overlapping(BooleanOperation::subtract);
    BooleanTree2D moved = tree.with({squareAt(Vector2(0, 0), 100), squareAt(Vector2(300, 300), 10)});
    EXPECT(moved.operation() == BooleanOperation::subtract);
    EXPECT(isApproximatelyEqual(totalArea(moved.contours()), 10000.0f, 5.0f));
    // The original is unchanged.
    EXPECT(isApproximatelyEqual(totalArea(tree.contours()), 7500.0f, 5.0f));
}

TEST(booleanTreeOperandsCanBeBooleansThemselves) {
    // (A ∩ B) united with C, where C is the square (0,0)…(10,10) inside A: still 2500.
    auto inner = std::make_shared<const BooleanTree2D>(overlapping(BooleanOperation::intersect));
    BooleanTree2D tree(BooleanOperation::unite, {inner, squareAt(Vector2(50, 50), 10)});
    EXPECT(tree.ok());
    EXPECT(isApproximatelyEqual(totalArea(tree.contours()), 2500.0f, 5.0f));
}

TEST(booleanTreeSubtractionMakesAHoleRunningClockwise) {
    BooleanTree2D tree(BooleanOperation::subtract, {squareAt(Vector2(0, 0), 100), squareAt(Vector2(40, 40), 20)});
    std::vector<Path2D> contours = tree.contours();
    EXPECT(contours.size() == 2);
    float outer = 0;
    float hole = 0;
    for (const Path2D& contour : contours) {
        if (contour.signedArea() > 0) outer = contour.signedArea();
        else hole = contour.signedArea();
    }
    EXPECT(isApproximatelyEqual(outer, 10000.0f, 1.0f));
    EXPECT(isApproximatelyEqual(hole, -400.0f, 1.0f));
}

TEST(booleanTreeRefusesNoOperandsAndACutOfOne) {
    EXPECT_PROBLEM(BooleanTree2D(BooleanOperation::unite, {}), "at least one operand");
    EXPECT_PROBLEM(BooleanTree2D(BooleanOperation::cut, {squareAt(Vector2(0, 0), 1)}), "at least two");
    EXPECT_PROBLEM(overlapping(BooleanOperation::unite).outward(3, 0), "no such outline");
    EXPECT_PROBLEM(overlapping(BooleanOperation::unite).outward(0, 99), "no such node");
}
