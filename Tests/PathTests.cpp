// Ports of EditorTests/Math/Paths/ (Path2DTests, Path2DFlatteningTests, Polygon2DTests), of
// EditorTests/Math/Curves/CubicBezierCrossingsTests, of `Angle.signedTurn` and of `BSpline.path`,
// plus bit-for-bit checks against values printed by the Swift engine (run with swiftc on the
// Engine/Math files). The tests that draw random curves with Swift's own generator are replaced
// by ones that use the core's `SeededRandom` (same checks, other numbers); the Double-precision
// "exact" references are left out.
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "../Curves/BSpline.h"
#include "../Curves/CubicBezier.h"
#include "../Numerics/SeededRandom.h"
#include "../Curves/Path2D.h"
#include "../Curves/Polygon2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
// Swift's `Float.pi`: rounded toward zero, one step below the nearest float.
constexpr float swiftPi = 0x1.921fb4p1f;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

bool sameBits(float value, std::uint32_t bits) { return std::bit_cast<std::uint32_t>(value) == bits; }

bool sameBits(Vector2 value, std::uint32_t x, std::uint32_t y) { return sameBits(value.x, x) && sameBits(value.y, y); }

using Curve2 = CubicBezier<Vector2>;

/// A square of side `side` with its lower-left corner at `corner`, counterclockwise.
Path2D square(float side = 1, Vector2 corner = Vector2()) {
    Path2D path(corner);
    path.addLine(corner + Vector2(side, 0));
    path.addLine(corner + Vector2(side, side));
    path.addLine(corner + Vector2(0, side));
    path.close();
    return path;
}

/// The circle of four arcs, counterclockwise, `turns` times round.
Path2D circle(float radius = 1, float turns = 1) {
    Path2D path(Vector2(radius, 0));
    path.addArc(Vector2(), radius, Angle::degrees(0), Angle::degrees(360 * turns));
    path.close();
    return path;
}

/// The right lobe of a figure-eight: a loop from the origin out to x = 2.25 and back.
Path2D rightLobe() {
    Path2D path{Vector2()};
    path.addCubic(Vector2(3, 3), Vector2(3, -3), Vector2());
    return path;
}

/// The figure-eight: the right lobe (clockwise) then its mirror image (counterclockwise).
Path2D figureEight() {
    Path2D path = rightLobe();
    path.addCubic(Vector2(-3, 3), Vector2(-3, -3), Vector2());
    path.close();
    return path;
}

/// x = 3t, and y = 3t(1 − t); its mirror y = 1 − 3t(1 − t); a cubic crossing the x axis three times.
Curve2 arch() { return Curve2(Vector2(0, 0), Vector2(1, 1), Vector2(2, 1), Vector2(3, 0)); }
Curve2 mirroredArch() { return Curve2(Vector2(0, 1), Vector2(1, 0), Vector2(2, 0), Vector2(3, 1)); }
Curve2 threeRoots() { return Curve2(Vector2(0, -9), Vector2(1, 13), Vector2(2, -13), Vector2(3, 9)); }
/// A cubic with a loop: σ = 1, π = 0.1, the loop closes at t = ½ ± √60/20.
Curve2 loopCurve() { return Curve2(Vector2(0, 0), Vector2(2, 1), Vector2(-1, 1), Vector2(1, 0)); }

} // namespace

// MARK: - Angle.signedTurn

TEST(signedTurnBetweenDirections) {
    EXPECT(isApproximatelyEqual(Angle::signedTurn(Vector2(1, 0), Vector2(0, 3)).degrees(), 90));
    EXPECT(isApproximatelyEqual(Angle::signedTurn(Vector2(2, 0), Vector2(1, -1)).degrees(), -45));
    // atan2 keeps tiny angles that arccos of a dot product rounds to 0.
    EXPECT(isApproximatelyEqual(Angle::signedTurn(Vector2(1, 0), Vector2(1, 1e-6f)).radians, 1e-6f, 1e-12f));
    // Scale does not matter: both directions are reduced first.
    EXPECT(isApproximatelyEqual(Angle::signedTurn(Vector2(1e-30f, 0), Vector2(0, 1e30f)).degrees(), 90));
    EXPECT(Angle::signedTurn(Vector2(), Vector2(0, 1)) == Angle::zero());
    EXPECT(Angle::signedTurn(Vector2(1, notANumber), Vector2(0, 1)) == Angle::zero());
    // Swift's value, bit for bit.
    EXPECT(sameBits(Angle::signedTurn(Vector2(2, 0), Vector2(1, -1)).radians, 0xbf490fdb));
}

// MARK: - Path2D building

TEST(buildingTracksTheCurrentPoint) {
    Path2D path(Vector2(1, 2));
    EXPECT(path.currentPoint() == Vector2(1, 2));
    EXPECT(path.segments().empty());
    path.addLine(Vector2(3, 4));
    EXPECT(path.currentPoint() == Vector2(3, 4));
    path.addCubic(Vector2(5, 5), Vector2(6, 5), Vector2(7, 4));
    EXPECT(path.currentPoint() == Vector2(7, 4));
    EXPECT(path.segments().size() == 2);
    EXPECT(!path.isClosed());
    path.close();
    EXPECT(path.isClosed());
}

TEST(aClosedPathCannotGrow) {
    Path2D path = square();
    EXPECT_PROBLEM(path.addLine(Vector2(5, 5)), "A closed path cannot grow: add every piece before calling close().");
    EXPECT_PROBLEM(path.addCubic(Vector2(), Vector2(), Vector2()), "A closed path cannot grow");
    EXPECT_PROBLEM(path.addArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(90)), "A closed path cannot grow");
}

TEST(anArcFarFromTheOriginContinuesThePath) {
    Vector2 centre(1e5f, 0);
    float firstStart = -2.61f;
    float firstSweep = 3.59f;
    Path2D path(centre + Vector2(Angle::fromRadians(firstStart).cosine(), Angle::fromRadians(firstStart).sine()) * 179);
    path.addArc(centre, 179, Angle::fromRadians(firstStart), Angle::fromRadians(firstSweep));
    size_t piecesOfFirstArc = path.segments().size();
    path.addArc(centre, 179, Angle::fromRadians(firstStart + firstSweep), Angle::fromRadians(1));
    EXPECT(path.segments().size() == piecesOfFirstArc + 1);
    for (const Path2D::Segment& segment : path.segments()) EXPECT(!segment.isLine());
    EXPECT(isApproximatelyEqual(Path2D::arcJoinTolerance(Vector2(), 2), 2e-5f, 1e-12f));
    // 4·ulp(100179) = 4 · 2^-7 = 0.03125, the Swift value.
    EXPECT(sameBits(Path2D::arcJoinTolerance(centre, 179), 0x3d000000));
}

TEST(anArcContinuingThePathAddsNoLine) {
    EXPECT(circle().segments().size() == 4);
    Path2D path(Vector2(5, 5));
    path.addArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(90));
    EXPECT(path.segments().size() == 2);
    EXPECT(path.segments()[0] == Path2D::Segment::line(Vector2(1, 0)));
    Path2D empty{Vector2()};
    empty.addArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(0));
    EXPECT(empty.segments().empty());
}

TEST(edgesKnowWhereTheyStart) {
    Path2D path(Vector2(1, 1));
    path.addLine(Vector2(2, 1));
    path.addCubic(Vector2(3, 1), Vector2(3, 2), Vector2(2, 2));
    std::vector<Path2D::Edge> edges = path.edges();
    EXPECT(edges.size() == 2);
    EXPECT(edges[0].start == Vector2(1, 1) && !edges[0].cubic().has_value());
    EXPECT(edges[1].start == Vector2(2, 1) && edges[1].cubic().has_value());
    EXPECT(edges[1].cubic()->end == Vector2(2, 2));
    // The closing line joins the pen back to the start; none is added when it is already there.
    EXPECT(path.edgesWithClosingLine().size() == 3);
    EXPECT(path.edgesWithClosingLine().back().segment == Path2D::Segment::line(Vector2(1, 1)));
    // The square has 3 drawn lines; the fill's outline adds the line back to the start.
    EXPECT(square().edgesWithClosingLine().size() == 4);
    EXPECT(square().edges().size() == 3);
}

// MARK: - Area

TEST(areaOfTheUnitSquare) {
    EXPECT(isApproximatelyEqual(square().signedArea(), 1));
    Path2D clockwise{Vector2()};
    clockwise.addLine(Vector2(0, 1));
    clockwise.addLine(Vector2(1, 1));
    clockwise.addLine(Vector2(1, 0));
    clockwise.close();
    EXPECT(isApproximatelyEqual(clockwise.signedArea(), -1));
}

TEST(anOpenPathIsMeasuredAsIfClosed) {
    Path2D open{Vector2()};
    open.addLine(Vector2(2, 0));
    open.addLine(Vector2(0, 3));
    EXPECT(!open.isClosed());
    EXPECT(isApproximatelyEqual(open.signedArea(), 3));
    EXPECT(Path2D(Vector2(4, 4)).signedArea() == 0);
}

TEST(areaOfTheArchPath) {
    Path2D path{Vector2()};
    path.addCubic(Vector2(0, 1), Vector2(1, 1), Vector2(1, 0));
    EXPECT(isApproximatelyEqual(path.signedArea(), -0.6f));
    EXPECT(sameBits(path.signedArea(), 0xbf19999a));
}

TEST(areaOfTheFigureEightLobes) {
    EXPECT(isApproximatelyEqual(rightLobe().signedArea(), -2.7f, 3e-6f));
    EXPECT(isApproximatelyEqual(figureEight().signedArea(), 0, 3e-6f));
    EXPECT(sameBits(rightLobe().signedArea(), 0xc02cccd0));
    EXPECT(sameBits(figureEight().signedArea(), 0));
}

TEST(areaOfACircleOfFourArcs) {
    float area = circle().signedArea();
    EXPECT(isApproximatelyEqual(area, swiftPi, 2.7e-3f));
    EXPECT(sameBits(area, 0x40491e44));
    EXPECT(sameBits(circle(2).signedArea(), 0x41491e44));
    EXPECT(isApproximatelyEqual(circle(1, 2).signedArea(), 2 * area, 1.5e-6f));
    Path2D clockwise(Vector2(1, 0));
    clockwise.addArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(-360));
    EXPECT(isApproximatelyEqual(clockwise.signedArea(), -area, 1.5e-6f));
}

TEST(areaDoesNotDependOnWhereTheShapeIs) {
    float area = circle(2).signedArea();
    for (Vector2 offset : {Vector2(0, 0), Vector2(100, -50), Vector2(1024, 2048), Vector2(-4096, 512)}) {
        Path2D path(Vector2(2, 0) + offset);
        path.addArc(offset, 2, Angle::degrees(0), Angle::degrees(360));
        path.close();
        EXPECT(isApproximatelyEqual(path.signedArea(), area, 4e-4f));
    }
    EXPECT(square(2, Vector2(4096, -4096)).signedArea() == 4);
}

TEST(areaScalesWithTheSquareOfTheSize) {
    for (float scale : {1e-3f, 1.0f, 1e3f}) {
        float expected = circle().signedArea() * scale * scale;
        EXPECT(std::fabs(circle(scale).signedArea() - expected) <= 1e-6f * scale * scale);
        EXPECT(std::fabs(square(scale).signedArea() - scale * scale) <= 1e-6f * scale * scale);
    }
}

// MARK: - Bounds

TEST(boundsOfPaths) {
    EXPECT(square(2, Vector2(1, 1)).bounds() == (Bounds2{Vector2(1, 1), Vector2(3, 3)}));
    EXPECT(Path2D(Vector2(4, 4)).bounds() == (Bounds2{Vector2(4, 4), Vector2(4, 4)}));
    Bounds2 bounds = rightLobe().bounds();
    EXPECT(isClose(bounds.minimum, Vector2(0, -0.8660254f)));
    EXPECT(isClose(bounds.maximum, Vector2(2.25f, 0.8660254f)));
    // Swift's bits.
    EXPECT(sameBits(bounds.minimum, 0x0, 0xbf5db3d8));
    EXPECT(sameBits(bounds.maximum, 0x40100000, 0x3f5db3d7));
}

TEST(boundsOfACircleAreThePlusAndMinusRadius) {
    Bounds2 bounds = circle().bounds();
    EXPECT(sameBits(bounds.minimum, 0xbf800000, 0xbf800000));
    EXPECT(sameBits(bounds.maximum, 0x3f800000, 0x3f800000));
}

// MARK: - Winding number

TEST(windingOfTheSquare) {
    Path2D path = square(2);
    EXPECT(path.windingNumber(Vector2(1, 1)) == 1);
    EXPECT(path.windingNumber(Vector2(3, 1)) == 0);
    EXPECT(path.windingNumber(Vector2(-1, 1)) == 0);
    EXPECT(path.windingNumber(Vector2(1, 3)) == 0);
    EXPECT(path.windingNumber(Vector2(1, -1)) == 0);
    Path2D clockwise{Vector2()};
    clockwise.addLine(Vector2(0, 2));
    clockwise.addLine(Vector2(2, 2));
    clockwise.addLine(Vector2(2, 0));
    clockwise.close();
    EXPECT(clockwise.windingNumber(Vector2(1, 1)) == -1);
}

TEST(aRayThroughAVertexCountsOnce) {
    Path2D diamond(Vector2(0, -1));
    diamond.addLine(Vector2(1, 0));
    diamond.addLine(Vector2(0, 1));
    diamond.addLine(Vector2(-1, 0));
    diamond.close();
    EXPECT(diamond.windingNumber(Vector2(0.5f, 0)) == 1);
    EXPECT(diamond.windingNumber(Vector2(-0.5f, 0)) == 1);
    EXPECT(diamond.windingNumber(Vector2(0, 0)) == 1);
    EXPECT(diamond.windingNumber(Vector2(-2, 0)) == 0);
    EXPECT(diamond.windingNumber(Vector2(2, 0)) == 0);
    EXPECT(diamond.windingNumber(Vector2(0, 0.5f)) == 1);
    EXPECT(diamond.windingNumber(Vector2(-0.5f, 1)) == 0);
    EXPECT(diamond.windingNumber(Vector2(-0.5f, -1)) == 0);
}

TEST(aRayAlongAnEdgeIsHandled) {
    Path2D path = square(2);
    EXPECT(path.windingNumber(Vector2(-1, 0)) == 0);
    EXPECT(path.windingNumber(Vector2(-1, 2)) == 0);
    EXPECT(path.windingNumber(Vector2(1, 0.0001f)) == 1);
    EXPECT(path.windingNumber(Vector2(1, 1.9999f)) == 1);
}

TEST(windingOfTheCircle) {
    Path2D path = circle();
    EXPECT(path.windingNumber(Vector2()) == 1);
    EXPECT(path.windingNumber(Vector2(0.99f, 0)) == 1);
    EXPECT(path.windingNumber(Vector2(-0.5f, 0.5f)) == 1);
    EXPECT(path.windingNumber(Vector2(1.01f, 0)) == 0);
    EXPECT(path.windingNumber(Vector2(2, 2)) == 0);
}

TEST(aRayTouchingTheCircleAtItsTopOrBottomCountsNothing) {
    Path2D path = circle();
    EXPECT(path.windingNumber(Vector2(-0.5f, 1)) == 0);
    EXPECT(path.windingNumber(Vector2(-0.5f, -1)) == 0);
    EXPECT(path.windingNumber(Vector2(-2, 0)) == 0);
    EXPECT(path.windingNumber(Vector2(0.5f, 0)) == 1);
    EXPECT(path.windingNumber(Vector2(2, 0)) == 0);
}

TEST(windingOfTheFigureEight) {
    Path2D path = figureEight();
    EXPECT(path.windingNumber(Vector2(1.5f, 0)) == -1);
    EXPECT(path.windingNumber(Vector2(1, 0.3f)) == -1);
    EXPECT(path.windingNumber(Vector2(-1.5f, 0)) == 1);
    EXPECT(path.windingNumber(Vector2(-1, -0.3f)) == 1);
    EXPECT(path.windingNumber(Vector2(0, 3)) == 0);
    EXPECT(path.windingNumber(Vector2(3, 0)) == 0);
    EXPECT(path.windingNumber(Vector2(-3, 0)) == 0);
    EXPECT(path.windingNumber(Vector2(2, 0.8f)) == 0);
}

TEST(aStraightEdgeFigureEightHasTheSameSigns) {
    Path2D bowTie(Vector2(-2, -1));
    bowTie.addLine(Vector2(2, 1));
    bowTie.addLine(Vector2(2, -1));
    bowTie.addLine(Vector2(-2, 1));
    bowTie.close();
    EXPECT(bowTie.windingNumber(Vector2(1, 0)) == -1);
    EXPECT(bowTie.windingNumber(Vector2(-1, 0)) == 1);
    EXPECT(bowTie.windingNumber(Vector2(0, 0.9f)) == 0);
    EXPECT(isApproximatelyEqual(bowTie.signedArea(), 0));
}

TEST(aDoublyWoundCircleHasWindingTwo) {
    Path2D path = circle(1, 2);
    EXPECT(path.windingNumber(Vector2()) == 2);
    EXPECT(path.windingNumber(Vector2(0.9f, 0.1f)) == 2);
    EXPECT(path.windingNumber(Vector2(1.5f, 0)) == 0);
    Path2D clockwise(Vector2(1, 0));
    clockwise.addArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(-720));
    EXPECT(clockwise.windingNumber(Vector2()) == -2);
}

TEST(anOpenPathWindsAsIfClosed) {
    Path2D open{Vector2()};
    open.addLine(Vector2(2, 0));
    open.addLine(Vector2(0, 2));
    EXPECT(open.windingNumber(Vector2(0.5f, 0.5f)) == 1);
    EXPECT(open.windingNumber(Vector2(1.5f, 1.5f)) == 0);
}

TEST(windingMatchesTheAreaSignOverASampleGrid) {
    // The integral of the winding number over the plane is the signed area.
    for (const Path2D& path : {circle(), figureEight(), rightLobe(), square(2)}) {
        float cell = 0.02f;
        int sum = 0;
        for (int column = -200; column < 200; column += 1) {
            for (int row = -200; row < 200; row += 1) {
                sum += path.windingNumber(
                    Vector2((static_cast<float>(column) + 0.5f) * cell, (static_cast<float>(row) + 0.5f) * cell));
            }
        }
        EXPECT(std::fabs(static_cast<float>(sum) * cell * cell - path.signedArea()) < 7.5e-3f);
    }
}

TEST(aRayAtTheHeightOfACurveToLineJoinCountsOnce) {
    // Found by the Swift review: the cubic's end height must come from the end point itself.
    Vector2 vertex(1.459292f, -0.4645524f);
    Path2D path(Vector2(-1.953633f, 0.16576731f));
    path.addCubic(Vector2(-1.9274813f, -0.21493074f), Vector2(1.3412727f, 0.34206843f), vertex);
    path.addLine(Vector2(-2.0182548f, 1.5227158f));
    EXPECT(path.windingNumber(Vector2(-4, vertex.y)) == 0);
}

// MARK: - Flattening

TEST(flatteningAPath) {
    Path2D path{Vector2()};
    path.addLine(Vector2(1, 0));
    path.addLine(Vector2(1, 1));
    EXPECT((path.flattened(0.01f) == std::vector<Vector2>{Vector2(0, 0), Vector2(1, 0), Vector2(1, 1)}));
    path.close();
    EXPECT((path.flattened(0.01f) == std::vector<Vector2>{Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 0)}));
}

TEST(aClosedCircleEndsExactlyAtItsStartWithNoNearDuplicate) {
    std::vector<Vector2> polyline = circle().flattened(0.01f);
    EXPECT(polyline.back() == polyline.front());
    EXPECT(polyline[polyline.size() - 2].distance(polyline[0]) > 0.01f);
}

TEST(flatteningACircleStaysWithinToleranceOfTheArcs) {
    std::vector<Vector2> polyline = circle().flattened(0.005f);
    EXPECT(polyline.front() == Vector2(1, 0) && polyline.back() == Vector2(1, 0));
    EXPECT(polyline.size() > 20);
    // Swift's flattened circle: 37 points, with these bits at 1, 7 and the one before the last.
    EXPECT(polyline.size() == 37);
    EXPECT(sameBits(polyline[1], 0x3f7be020, 0x3e3810b8));
    EXPECT(sameBits(polyline[7], 0x3eb2a743, 0x3f6ffbe7));
    EXPECT(sameBits(polyline[polyline.size() - 2], 0x3f7be021, 0xbe3810a8));
    std::vector<Vector2> eight = figureEight().flattened(0.01f);
    EXPECT(eight.size() == 55);
    EXPECT(sameBits(eight[3], 0x3f638e38, 0x3f30fcd7));
}

// MARK: - Polygon2D

namespace {
Polygon2D unitSquare() { return Polygon2D({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)}); }
} // namespace

TEST(aClosingDuplicateCornerIsDropped) {
    Polygon2D closed({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1), Vector2(0, 0)});
    EXPECT(closed == unitSquare());
    EXPECT(closed.sides().size() == 4);
    EXPECT(closed.sides().back() == (Segment2{Vector2(0, 1), Vector2(0, 0)}));
}

TEST(aPolygonNeedsThreeCorners) {
    EXPECT_PROBLEM(Polygon2D({Vector2(0, 0), Vector2(1, 0)}), "A polygon needs at least 3 corners, but got 2.");
    EXPECT_PROBLEM(Polygon2D({Vector2(0, 0), Vector2(1, 0), Vector2(0, 0)}), "but got 2.");
}

TEST(theBoxHoldsEveryCorner) {
    Polygon2D triangle({Vector2(-1, 2), Vector2(3, -1), Vector2(0, 4)});
    EXPECT(isClose(triangle.bounds().minimum, Vector2(-1, -1)));
    EXPECT(isClose(triangle.bounds().maximum, Vector2(3, 4)));
}

TEST(theWindingNumberCountsLoopsAndTheirDirection) {
    Polygon2D square = unitSquare();
    EXPECT(square.windingNumber(Vector2(0.5f, 0.5f)) == 1);
    EXPECT(square.windingNumber(Vector2(2, 0.5f)) == 0);
    EXPECT(square.windingNumber(Vector2(-1, 0.5f)) == 0);
    std::vector<Vector2> reversed(square.corners().rbegin(), square.corners().rend());
    EXPECT(Polygon2D(reversed).windingNumber(Vector2(0.5f, 0.5f)) == -1);
    EXPECT(square.windingNumber(Vector2(-1, 1)) == 0);
    EXPECT(square.windingNumber(Vector2(-1, 0)) == 0);
    Polygon2D diamond({Vector2(1, 0), Vector2(0, 1), Vector2(-1, 0), Vector2(0, -1)});
    EXPECT(diamond.windingNumber(Vector2(0, 0)) == 1);
    EXPECT(diamond.windingNumber(Vector2(-2, 0)) == 0);
}

TEST(aFivePointedStarWindsTwiceRoundItsCentre) {
    std::vector<Vector2> corners;
    for (int index = 0; index < 5; index += 1) {
        Angle angle = Angle::degrees(90 + 144 * static_cast<float>(index));
        corners.push_back(Vector2(angle.cosine(), angle.sine()));
    }
    Polygon2D star(corners);
    EXPECT(star.windingNumber(Vector2(0, 0)) == 2);
    EXPECT(star.windingNumber(Vector2(0, 0.9f)) == 1);
    EXPECT(star.windingNumber(Vector2(0, 1.5f)) == 0);
    EXPECT(star.signedDistance(Vector2(0, 0)) < 0);
}

TEST(theSignedDistanceIsTheNearestSideNegativeInside) {
    Polygon2D square = unitSquare();
    EXPECT(isApproximatelyEqual(square.signedDistance(Vector2(0.5f, 0.25f)), -0.25f));
    EXPECT(isApproximatelyEqual(square.signedDistance(Vector2(2, 0.5f)), 1));
    EXPECT(isApproximatelyEqual(square.signedDistance(Vector2(2, 2)), std::sqrt(2.0f)));
    EXPECT(isApproximatelyEqual(square.signedDistance(Vector2(1, 0.5f)), 0));
}

TEST(theSignedDistanceChangesNoFasterThanDistance) {
    Polygon2D lShape({Vector2(0, 0), Vector2(2, 0), Vector2(2, 1), Vector2(1, 1), Vector2(1, 2), Vector2(0, 2)});
    SeededRandom random(5);
    for (int trial = 0; trial < 500; trial += 1) {
        Vector2 first(random.nextFloat(-1, 3), random.nextFloat(-1, 3));
        Vector2 second(random.nextFloat(-1, 3), random.nextFloat(-1, 3));
        float change = std::fabs(lShape.signedDistance(first) - lShape.signedDistance(second));
        EXPECT(change <= first.distance(second) + 1e-5f);
    }
    // Swift's values for the L shape, bit for bit.
    EXPECT(sameBits(lShape.signedDistance(Vector2(1.7f, 1.6f)), 0x3f19999a));
    EXPECT(sameBits(lShape.signedDistance(Vector2(0.4f, 0.3f)), 0xbe99999a));
}

// MARK: - Crossings of a cubic with a line

TEST(cubicAgainstTheXAxisCrossesAtTheThreeRoots) {
    std::vector<CurveCrossing> crossings = threeRoots().crossings(Vector2(0, 0), Vector2(1, 0));
    EXPECT(crossings.size() == 3);
    float expectedParameters[3] = {0.25f, 0.5f, 0.75f};
    float expectedX[3] = {0.75f, 1.5f, 2.25f};
    for (size_t index = 0; index < crossings.size() && index < 3; index += 1) {
        EXPECT(isApproximatelyEqual(crossings[index].firstParameter, expectedParameters[index]));
        EXPECT(isApproximatelyEqual(crossings[index].secondParameter, expectedX[index]));
        EXPECT(isClose(crossings[index].point, Vector2(expectedX[index], 0)));
    }
    // Swift's bits.
    EXPECT(sameBits(crossings[0].firstParameter, 0x3e800000) && sameBits(crossings[0].secondParameter, 0x3f400000));
    EXPECT(sameBits(crossings[1].point, 0x3fc00000, 0x0));
    EXPECT(sameBits(crossings[2].firstParameter, 0x3f400000) && sameBits(crossings[2].point, 0x40100000, 0x0));
}

TEST(withinSegmentKeepsOnlyCrossingsOnTheSegment) {
    std::vector<CurveCrossing> crossings = threeRoots().crossings(Vector2(0, 0), Vector2(1, 0), true);
    EXPECT(crossings.size() == 1);
    EXPECT(isApproximatelyEqual(crossings[0].firstParameter, 0.25f));
    std::vector<CurveCrossing> ending = threeRoots().crossings(Vector2(0, 0), Vector2(1.5f, 0), true);
    EXPECT(ending.size() == 2);
    EXPECT(ending.back().secondParameter == 1);
}

TEST(lineThroughTheEndPointMeetsItExactly) {
    std::vector<CurveCrossing> crossings = arch().crossings(Vector2(3, 0), Vector2(4, 1));
    EXPECT(crossings.size() == 1);
    EXPECT(crossings[0].firstParameter == 1);
    EXPECT(crossings[0].secondParameter == 0);
    EXPECT(crossings[0].point == arch().end);
}

TEST(curveLyingOnTheLineHasNoIsolatedCrossings) {
    Curve2 straight = Path2D::evenlyPacedCubic(Vector2(0, 0), Vector2(3, 0));
    EXPECT(straight.crossings(Vector2(-1, 0), Vector2(5, 0)).empty());
    EXPECT(arch().crossings(Vector2(0, 0), Vector2(0, 0)).empty());
}

// MARK: - Crossings of two cubics

TEST(mirroredArchesCrossAtTheKnownPoints) {
    std::optional<std::vector<CurveCrossing>> crossings = arch().crossings(mirroredArch());
    EXPECT(crossings.has_value() && crossings->size() == 2);
    if (!crossings || crossings->size() != 2) return;
    float parameters[2] = {0.5f - std::sqrt(1.0f / 12), 0.5f + std::sqrt(1.0f / 12)};
    for (size_t index = 0; index < 2; index += 1) {
        EXPECT(isApproximatelyEqual((*crossings)[index].firstParameter, parameters[index]));
        EXPECT(isApproximatelyEqual((*crossings)[index].secondParameter, parameters[index]));
        EXPECT(isClose((*crossings)[index].point, Vector2(3 * parameters[index], 0.5f)));
    }
    // The Swift engine finds exactly these floats.
    EXPECT(sameBits((*crossings)[0].firstParameter, 0x3e58658c) && sameBits((*crossings)[0].secondParameter, 0x3e58658c));
    EXPECT(sameBits((*crossings)[0].point, 0x3f224c2a, 0x3f000000));
    EXPECT(sameBits((*crossings)[1].firstParameter, 0x3f49e69d) && sameBits((*crossings)[1].point, 0x40176cf5, 0x3f000000));
}

TEST(curvesSharingAnEndCrossThereExactly) {
    Curve2 other(Vector2(3, 0), Vector2(4, 1), Vector2(5, -1), Vector2(6, 0));
    std::optional<std::vector<CurveCrossing>> crossings = arch().crossings(other);
    EXPECT(crossings.has_value() && crossings->size() == 1);
    if (!crossings || crossings->size() != 1) return;
    EXPECT((*crossings)[0].firstParameter == 1);
    EXPECT((*crossings)[0].secondParameter == 0);
    EXPECT((*crossings)[0].point == arch().end);
}

TEST(separateCurvesDoNotCross) {
    std::optional<std::vector<CurveCrossing>> crossings = arch().crossings(arch().translated(Vector2(0, 2)));
    EXPECT(crossings.has_value() && crossings->empty());
}

TEST(aTangentTouchIsOneCrossing) {
    Curve2 upper(Vector2(-1, 1), Vector2(-1.0f / 3, -1.0f / 3), Vector2(1.0f / 3, -1.0f / 3), Vector2(1, 1));
    Curve2 lower(Vector2(-1, -1), Vector2(-1.0f / 3, 1.0f / 3), Vector2(1.0f / 3, 1.0f / 3), Vector2(1, -1));
    std::optional<std::vector<CurveCrossing>> crossings = upper.crossings(lower);
    EXPECT(crossings.has_value() && crossings->size() == 1);
    if (!crossings || crossings->size() != 1) return;
    EXPECT(isApproximatelyEqual((*crossings)[0].firstParameter, 0.5f, 1e-3f));
    EXPECT(isClose((*crossings)[0].point, Vector2(), 1e-3f));
}

TEST(overlappingCurvesHaveNoTrustworthyCrossings) {
    EXPECT(!arch().crossings(arch()).has_value());
    EXPECT(!arch().crossings(arch().split(0.5f).first).has_value());
    Curve2 nearlyTheSame(Vector2(0, 1e-5f), arch().startHandle, arch().endHandle, Vector2(3, 1e-5f));
    EXPECT(!arch().crossings(nearlyTheSame).has_value());
}

TEST(crossingsLieOnBothCurves) {
    SeededRandom random(51);
    auto point = [&] { return Vector2(random.nextFloat(-1, 1), random.nextFloat(-1, 1)); };
    int found = 0;
    float worst = 0;
    for (int trial = 0; trial < 200; trial += 1) {
        Curve2 first(point(), point(), point(), point());
        Curve2 second(point(), point(), point(), point());
        std::optional<std::vector<CurveCrossing>> crossings = first.crossings(second);
        EXPECT(crossings.has_value());
        if (!crossings) continue;
        for (const CurveCrossing& crossing : *crossings) {
            found += 1;
            float onFirst = (first.point(crossing.firstParameter) - crossing.point).length();
            float onSecond = (second.point(crossing.secondParameter) - crossing.point).length();
            worst = largerOf(worst, largerOf(onFirst, onSecond));
        }
    }
    EXPECT(found > 50);
    EXPECT(worst < 8e-7f);
}

TEST(lineCrossingsLieOnBothCurveAndLine) {
    SeededRandom random(52);
    auto point = [&] { return Vector2(random.nextFloat(-1, 1), random.nextFloat(-1, 1)); };
    int found = 0;
    float worstCurve = 0;
    float worstLine = 0;
    for (int trial = 0; trial < 200; trial += 1) {
        Curve2 curve(point(), point(), point(), point());
        Vector2 lineStart = point();
        Vector2 lineEnd = point();
        for (const CurveCrossing& crossing : curve.crossings(lineStart, lineEnd)) {
            found += 1;
            Vector2 onLine = Vector2::linearBlend(lineStart, lineEnd, crossing.secondParameter);
            worstCurve = largerOf(worstCurve, (curve.point(crossing.firstParameter) - crossing.point).length());
            worstLine = largerOf(worstLine, (onLine - crossing.point).length());
        }
    }
    EXPECT(found > 50);
    EXPECT(worstCurve < 1e-6f);
    EXPECT(worstLine < 2e-6f);
}

TEST(crossingsAtEveryScale) {
    struct Placement {
        float scale;
        Vector2 shift;
    };
    for (Placement placement : {Placement{1e-3f, Vector2()}, Placement{1e3f, Vector2()}, Placement{1, Vector2(1000, 1000)}}) {
        auto placed = [&](const Curve2& curve) {
            return Curve2(
                curve.start * placement.scale + placement.shift, curve.startHandle * placement.scale + placement.shift,
                curve.endHandle * placement.scale + placement.shift, curve.end * placement.scale + placement.shift);
        };
        float parameters[2] = {0.5f - std::sqrt(1.0f / 12), 0.5f + std::sqrt(1.0f / 12)};
        std::optional<std::vector<CurveCrossing>> pair = placed(arch()).crossings(placed(mirroredArch()));
        EXPECT(pair.has_value() && pair->size() == 2);
        if (!pair || pair->size() != 2) continue;
        float worst = 0;
        for (size_t index = 0; index < 2; index += 1) {
            worst = largerOf(worst, std::fabs((*pair)[index].firstParameter - parameters[index]));
            worst = largerOf(worst, std::fabs((*pair)[index].secondParameter - parameters[index]));
        }
        std::vector<CurveCrossing> line =
            placed(threeRoots()).crossings(placement.shift, placement.shift + Vector2(placement.scale, 0));
        EXPECT(line.size() == 3);
        for (size_t index = 0; index < line.size() && index < 3; index += 1) {
            worst = largerOf(worst, std::fabs(line[index].firstParameter - (0.25f + 0.25f * static_cast<float>(index))));
        }
        std::optional<SelfCrossingParameters> selfCrossing = placed(loopCurve()).selfCrossingParameters();
        EXPECT(selfCrossing.has_value());
        if (selfCrossing) {
            worst = largerOf(worst, std::fabs(selfCrossing->first - (0.5f - std::sqrt(60.0f) / 20)));
            worst = largerOf(worst, std::fabs(selfCrossing->second - (0.5f + std::sqrt(60.0f) / 20)));
        }
        EXPECT(worst < 8e-7f);
    }
}

// MARK: - A cubic crossing itself

TEST(loopCrossesItselfAtTheKnownPoint) {
    std::optional<SelfCrossingParameters> parameters = loopCurve().selfCrossingParameters();
    EXPECT(parameters.has_value());
    if (!parameters) return;
    EXPECT(isApproximatelyEqual(parameters->first, 0.5f - std::sqrt(60.0f) / 20));
    EXPECT(isApproximatelyEqual(parameters->second, 0.5f + std::sqrt(60.0f) / 20));
    EXPECT(isClose(loopCurve().point(parameters->first), Vector2(0.5f, 0.3f)));
    EXPECT(isClose(loopCurve().point(parameters->second), Vector2(0.5f, 0.3f)));
    EXPECT(sameBits(parameters->first, 0x3de6d025) && sameBits(parameters->second, 0x3f6325fc));
}

TEST(aTeardropHasNoLoop) {
    Curve2 teardrop(Vector2(), Vector2(2, 1), Vector2(2, -1), Vector2());
    EXPECT(!teardrop.selfCrossingParameters().has_value());
}

TEST(thinLoopsAreFound) {
    for (float height : {1e-4f, 1e-5f, 1e-6f, 1e-8f}) {
        Curve2 thin(Vector2(), Vector2(3, height), Vector2(-2, height), Vector2(1, 0));
        std::optional<SelfCrossingParameters> parameters = thin.selfCrossingParameters();
        EXPECT(parameters.has_value());
        if (!parameters) continue;
        EXPECT(isApproximatelyEqual(parameters->first, 0.0669873f, 1e-5f));
        EXPECT(isApproximatelyEqual(parameters->second, 0.9330127f, 1e-5f));
    }
}

TEST(curvesWithoutALoopHaveNoSelfCrossing) {
    EXPECT(!arch().selfCrossingParameters().has_value());
    EXPECT(!threeRoots().selfCrossingParameters().has_value());
    EXPECT(!Path2D::evenlyPacedCubic(Vector2(), Vector2(1, 1)).selfCrossingParameters().has_value());
    Curve2 nan(Vector2(notANumber, 0), Vector2(2, 1), Vector2(-1, 1), Vector2(1, 0));
    EXPECT(!nan.selfCrossingParameters().has_value());
    EXPECT(!nan.crossings(arch()).has_value());
    EXPECT(nan.crossings(Vector2(), Vector2(1, 0)).empty());
    (void)infinity;
}

// MARK: - A path crossing itself

TEST(aBowTieCrossesItselfAtTheOrigin) {
    Path2D bowTie(Vector2(-2, -1));
    bowTie.addLine(Vector2(2, 1));
    bowTie.addLine(Vector2(2, -1));
    bowTie.addLine(Vector2(-2, 1));
    bowTie.close();
    std::optional<std::vector<PathSelfCrossing>> crossings = bowTie.selfCrossings();
    EXPECT(crossings.has_value() && crossings->size() == 1);
    if (!crossings || crossings->size() != 1) return;
    EXPECT((*crossings)[0].firstPieceIndex == 0 && (*crossings)[0].secondPieceIndex == 2);
    EXPECT(sameBits((*crossings)[0].firstParameter, 0x3f000000) && sameBits((*crossings)[0].secondParameter, 0x3f000000));
    EXPECT(sameBits((*crossings)[0].point, 0x0, 0x0));
}

TEST(aSingleCubicLoopIsOneSelfCrossing) {
    Path2D path{Vector2()};
    path.addCubic(Vector2(2, 1), Vector2(-1, 1), Vector2(1, 0));
    std::optional<std::vector<PathSelfCrossing>> crossings = path.selfCrossings();
    EXPECT(crossings.has_value() && crossings->size() == 1);
    if (!crossings || crossings->size() != 1) return;
    EXPECT((*crossings)[0].firstPieceIndex == 0 && (*crossings)[0].secondPieceIndex == 0);
    EXPECT(sameBits((*crossings)[0].firstParameter, 0x3de6d025) && sameBits((*crossings)[0].secondParameter, 0x3f6325fc));
    EXPECT(sameBits((*crossings)[0].point, 0x3f000001, 0x3e99999b));
}

TEST(simplePathsHaveNoSelfCrossings) {
    EXPECT(square().selfCrossings()->empty());
    EXPECT(circle().selfCrossings()->empty());
}

// MARK: - BSpline.path

TEST(aClampedCubicSplineBecomesAPath) {
    std::vector<Vector2> points{Vector2(0, 0), Vector2(1, 2), Vector2(2, -1), Vector2(3, 1), Vector2(4, 3), Vector2(5, 0)};
    std::optional<BSpline<Vector2>> spline = BSpline<Vector2>::make(3, points, {0, 0, 0, 0, 1, 2, 3, 3, 3, 3});
    EXPECT(spline.has_value());
    if (!spline) return;
    std::optional<Path2D> path = spline->path();
    EXPECT(path.has_value());
    if (!path) return;
    EXPECT(path->segments().size() == 3);
    EXPECT(path->start == points[0]);
    // Swift's area and end point, bit for bit.
    EXPECT(sameBits(path->signedArea(), 0xc08e2aac));
    EXPECT(sameBits(path->currentPoint(), 0x40a00000, 0x0));
}

TEST(anUnclampedSplineHasNoPath) {
    std::vector<Vector2> points{Vector2(0, 0), Vector2(1, 2), Vector2(2, -1), Vector2(3, 1)};
    std::optional<BSpline<Vector2>> spline = BSpline<Vector2>::make(3, points, {0, 1, 2, 3, 4, 5, 6, 7});
    EXPECT(spline.has_value());
    if (spline) EXPECT(!spline->path().has_value());
}
