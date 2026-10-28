// Tests of Curves/CornerShapes: corners of shapes, pits and peaks, replaced and shaped corners, corner frames
// and caps. Outlines run counterclockwise (the inside on the left), the mirror of graphics_cpp's goldens
// (boolean.json, corners.json), which are transcribed here by hand: the plus shape, the triangle, the square
// and the stroke capped by a triangle.
#include <cmath>
#include <vector>

#include "../Curves/CornerShapes.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float pi = Angle::pi;

Path2D polygonPath(const std::vector<Vector2>& points) {
    Path2D path(points[0]);
    for (size_t index = 1; index < points.size(); index += 1) path.addLine(points[index]);
    path.close();
    return path;
}

Shape2D shapeOf(std::vector<Path2D> outlines) { return Shape2D{std::move(outlines), FillRule::nonzero}; }

Shape2D squareShape() { return shapeOf({polygonPath({{0, 0}, {100, 0}, {100, 100}, {0, 100}})}); }

/// The plus sign of graphics_cpp's corner goldens (arms 40 long and 20 wide), counterclockwise.
Shape2D plusShape() {
    return shapeOf({polygonPath(
        {{0, 40}, {40, 40}, {40, 0}, {60, 0}, {60, 40}, {100, 40}, {100, 60}, {60, 60}, {60, 100}, {40, 100}, {40, 60}, {0, 60}})});
}

bool isNear(float value, float expected, float relative = 1e-3f) {
    return std::fabs(value - expected) <= relative * std::max(1.0f, std::fabs(expected));
}

float cubicArcArea(float radius) { return pi * radius * radius; }

} // namespace

// MARK: Corners, pits, peaks

TEST(cornersOfASquareAreFourLeftTurns) {
    std::vector<ShapeCorner> corners = cornersOfShape(squareShape());
    EXPECT(corners.size() == 4);
    for (const ShapeCorner& corner : corners) {
        EXPECT(isApproximatelyEqual(corner.turn.radians, pi / 2, 1e-5f) && corner.isConvex());
        EXPECT(isApproximatelyEqual(corner.outsideAngle().degrees(), 270, 1e-3f));
    }
    // Golden (corners.json "corners"), mirrored: the corner at the origin arrives along -y and leaves along +x.
    EXPECT(corners[0].point == Vector2(0, 0));
    EXPECT(isApproximatelyEqual(corners[0].outgoingDirection.x, 1, 1e-5f) && isApproximatelyEqual(corners[0].incomingDirection.y, -1, 1e-5f));
}

TEST(cornersBelowTheMinimumTurnAreIgnored) {
    Shape2D slight = shapeOf({polygonPath({{0, 0}, {100, 0}, {200, 1}, {200, 100}, {0, 100}})});
    // The kink at (100, 0) turns by 0.57 degrees.
    EXPECT(cornersOfShape(slight, Angle::degrees(0.1f)).size() == 5);
    EXPECT(cornersOfShape(slight, Angle::degrees(1)).size() == 4);
    EXPECT(cornersOfShape(slight, Angle::degrees(80)).size() == 4);
}

TEST(aPlusShapeHasFourPitsAndEightPeaks) {
    std::vector<ShapeCorner> pits = pitsOfShape(plusShape());
    EXPECT(pits.size() == 4);
    for (const ShapeCorner& pit : pits) {
        EXPECT(!pit.isConvex() && isApproximatelyEqual(pit.outsideAngle().degrees(), 90, 1e-3f));
    }
    EXPECT(peaksOfShape(plusShape()).size() == 8);
    // Golden: pits with a limit of 91 degrees are all four, the corners at (40,60) (60,60) (60,40) (40,40).
    EXPECT(pitsOfShape(plusShape(), Angle::degrees(91)).size() == 4);
    EXPECT(pitsOfShape(plusShape(), Angle::degrees(89)).empty());
}

TEST(aTriangleHasThreePeaksWithTheGoldenOutsideAngles) {
    Shape2D triangle = shapeOf({polygonPath({{0, 0}, {100, 0}, {50, 100}})});
    std::vector<ShapeCorner> peaks = peaksOfShape(triangle);
    EXPECT(peaks.size() == 3);
    EXPECT(isApproximatelyEqual(peaks[0].outsideAngle().degrees(), 296.565f, 1e-2f));
    EXPECT(isApproximatelyEqual(peaks[1].outsideAngle().degrees(), 296.565f, 1e-2f));
    EXPECT(isApproximatelyEqual(peaks[2].outsideAngle().degrees(), 306.870f, 1e-2f));
    EXPECT(peaksOfShape(triangle, Angle::degrees(120)).size() == 1);
    EXPECT(pitsOfShape(triangle).empty());
}

TEST(aHoleHasPitsAtItsCorners) {
    Shape2D ring = shapeOf({polygonPath({{0, 0}, {100, 0}, {100, 100}, {0, 100}}), polygonPath({{25, 25}, {25, 75}, {75, 75}, {75, 25}})});
    std::vector<ShapeCorner> pits = pitsOfShape(ring);
    EXPECT(pits.size() == 4);
    for (const ShapeCorner& pit : pits) EXPECT(pit.outlineIndex == 1);
    EXPECT(peaksOfShape(ring).size() == 4);
}

// MARK: Replaced corners

TEST(chamferingThePlusRemovesTheHandWorkedArea) {
    // 8 peaks lose a right triangle of legs 10 (50 each); 4 pits gain one (50 each): 3600 - 400 + 200.
    Shape2D chamfered = replacedCorners(
        plusShape(), [](const ShapeCorner&) { return 10.0f; },
        [](Vector2 cutBefore, Vector2, Vector2 cutAfter, Vector2, const ShapeCorner&) {
            return std::vector<OutlinePiece>{OutlinePiece::line(cutBefore, cutAfter)};
        });
    EXPECT(isNear(plusShape().signedArea(), 3600));
    EXPECT(isNear(chamfered.signedArea(), 3400));
    EXPECT(chamfered.outlines[0].segments().size() == 20);
    // 12 cuts and the 8 long sides; the 4 arm ends (20 long) are cut away whole by the two cuts of 10.
    // Agrees with the path's own chamfer.
    EXPECT(isNear(chamfered.signedArea(), plusShape().outlines[0].chamfered(10).signedArea()));
}

TEST(aCutLongerThanHalfASideIsHeldToHalfOfIt) {
    Shape2D small = shapeOf({polygonPath({{0, 0}, {10, 0}, {10, 10}, {0, 10}})});
    Shape2D diamond = replacedCorners(
        small, [](const ShapeCorner&) { return 100.0f; },
        [](Vector2 cutBefore, Vector2, Vector2 cutAfter, Vector2, const ShapeCorner&) {
            return std::vector<OutlinePiece>{OutlinePiece::line(cutBefore, cutAfter)};
        });
    EXPECT(diamond.outlines[0].segments().size() == 4);
    EXPECT(isNear(diamond.signedArea(), 50));
    EXPECT(diamond.contains(Vector2(5, 5)) && !diamond.contains(Vector2(1, 1)));
    // A share of 0.25 cuts back 2.5 only.
    CornerOptions quarter;
    quarter.share = 0.25f;
    Shape2D quartered = replacedCorners(
        small, [](const ShapeCorner&) { return 100.0f; },
        [](Vector2 cutBefore, Vector2, Vector2 cutAfter, Vector2, const ShapeCorner&) {
            return std::vector<OutlinePiece>{OutlinePiece::line(cutBefore, cutAfter)};
        },
        quarter);
    EXPECT(isNear(quartered.signedArea(), 100 - 4 * 2.5f * 2.5f / 2));
}

TEST(cornersWithNoReachAreLeftSharp) {
    Shape2D same = replacedCorners(
        squareShape(), [](const ShapeCorner&) { return 0.0f; },
        [](Vector2, Vector2, Vector2, Vector2, const ShapeCorner&) { return std::vector<OutlinePiece>{}; });
    EXPECT(same == squareShape());
    // Only the corner at the origin is replaced.
    Shape2D one = replacedCorners(
        squareShape(), [](const ShapeCorner& corner) { return corner.point == Vector2(0, 0) ? 10.0f : 0.0f; },
        [](Vector2 cutBefore, Vector2, Vector2 cutAfter, Vector2, const ShapeCorner&) {
            return std::vector<OutlinePiece>{OutlinePiece::line(cutBefore, cutAfter)};
        });
    EXPECT(isNear(one.signedArea(), 10000 - 50));
}

// MARK: Shaped corners

TEST(aTiltOfZeroRoundsLikeTheFillet) {
    // Hand-worked: each corner of a 100 square rounded by r = 8 loses r^2 (1 - pi/4).
    float radius = 8;
    Shape2D rounded = shapedCorners(
        squareShape(), [&](const ShapeCorner&) { return radius; }, [](const ShapeCorner&) { return CornerShape(); });
    float expected = 10000 - 4 * radius * radius * (1 - pi / 4);
    EXPECT(isNear(rounded.signedArea(), expected, 2e-4f));
    EXPECT(isNear(rounded.signedArea(), squareShape().outlines[0].filleted(radius).signedArea(), 2e-4f));
    EXPECT(rounded.outlines[0].segments().size() == 8);
    // Golden (corners.json roundCorners): the first cubic starts at (8, 0).
    bool hasCutAtEight = false;
    for (const Path2D::Segment& segment : rounded.outlines[0].segments()) {
        hasCutAtEight = hasCutAtEight || (isApproximatelyEqual(segment.end().x, 8, 1e-4f) && isApproximatelyEqual(segment.end().y, 0, 1e-4f));
    }
    EXPECT(hasCutAtEight);
}

TEST(aTiltOfOneAtAPeakAddsThreeQuartersOfADisc) {
    // At tilt 1 the arc leaves the sides square: for a right angle cut r back it is the 270 degree arc of the
    // circle of radius r round the corner, which adds 3/4 pi r^2 per corner.
    float radius = 10;
    Shape2D outset = shapedCorners(
        squareShape(), [&](const ShapeCorner&) { return radius; }, [](const ShapeCorner&) { return CornerShape(1); });
    EXPECT(isNear(outset.signedArea(), 10000 + 3 * cubicArcArea(radius), 5e-4f));
    EXPECT(outset.outlines[0].segments().size() == 4 * 3 + 4);
}

TEST(aTiltOfMinusOneAtAPeakBitesAQuarterDisc) {
    float radius = 10;
    Shape2D inset = shapedCorners(
        squareShape(), [&](const ShapeCorner&) { return radius; }, [](const ShapeCorner&) { return CornerShape(-1); });
    EXPECT(isNear(inset.signedArea(), 10000 - cubicArcArea(radius), 5e-4f));
}

TEST(tiltFlipsDirectionAtAPit) {
    // Plus shape, pits only (size 0 at peaks). Outset (tilt 1) adds ink at a pit as at a peak: the arc leaves
    // the sides square, bulging into the empty wedge, adding pi r^2 / 4 per pit. Inset (tilt -1) is the arc
    // round the pit itself through the ink, taking 3/4 pi r^2 per pit.
    float radius = 5;
    auto size = [&](const ShapeCorner& corner) { return corner.isConvex() ? 0.0f : radius; };
    Shape2D plain = shapedCorners(plusShape(), size, [](const ShapeCorner&) { return CornerShape(); });
    Shape2D outset = shapedCorners(plusShape(), size, [](const ShapeCorner&) { return CornerShape(1); });
    Shape2D inset = shapedCorners(plusShape(), size, [](const ShapeCorner&) { return CornerShape(-1); });
    // Rounding a pit fills the corner: r^2 (1 - pi/4) per pit.
    EXPECT(isNear(plain.signedArea(), 3600 + 4 * radius * radius * (1 - pi / 4), 5e-4f));
    EXPECT(isNear(outset.signedArea(), 3600 + cubicArcArea(radius), 1e-3f));
    EXPECT(isNear(inset.signedArea(), 3600 - 3 * cubicArcArea(radius), 1e-3f));
}

TEST(aProfileIsDrawnAcrossTheCornersChord) {
    // A triangle profile with its apex halfway up the box: its apex lands at the arc's bulge,
    // (chord/2) tan(alpha/2) off the chord, where alpha = 45 degrees for a right angle at tilt 0.
    Path2D profile(Vector2(-1, 0));
    profile.addLine(Vector2(0, 1));
    profile.addLine(Vector2(1, 0));
    float radius = 10;
    CornerShape cornerShape;
    cornerShape.profile = profile;
    Shape2D shaped = shapedCorners(
        squareShape(), [&](const ShapeCorner&) { return radius; }, [&](const ShapeCorner&) { return cornerShape; });
    float chord = radius * std::sqrt(2.0f);
    float bulge = chord / 2 * std::tan(pi / 8);
    float perCorner = -radius * radius / 2 + chord * bulge / 2;
    EXPECT(isNear(shaped.signedArea(), 10000 + 4 * perCorner, 1e-3f));
    EXPECT(shaped.outlines[0].segments().size() == 12);
}

TEST(aCurveInTheCornerFrameThroughTheCornerLeavesItSharp) {
    // A curve drawn from (-1, 0) up to (0, 1) and down to (1, 0) passes through the corner itself in the frame.
    Path2D through(Vector2(-1, 0));
    through.addLine(Vector2(0, 1));
    through.addLine(Vector2(1, 0));
    CornerShape cornerShape;
    cornerShape.curve = through;
    Shape2D shaped = shapedCorners(
        squareShape(), [](const ShapeCorner&) { return 10.0f; }, [&](const ShapeCorner&) { return cornerShape; });
    EXPECT(isNear(shaped.signedArea(), 10000));
    // A curve along the chord is a bevel.
    Path2D chord(Vector2(-1, 0));
    chord.addLine(Vector2(1, 0));
    CornerShape bevel;
    bevel.curve = chord;
    Shape2D bevelled = shapedCorners(
        squareShape(), [](const ShapeCorner&) { return 10.0f; }, [&](const ShapeCorner&) { return bevel; });
    EXPECT(isNear(bevelled.signedArea(), 10000 - 4 * 50));
}

TEST(seenHearsEveryCornersCuts) {
    int heard = 0;
    Vector2 firstCutBefore;
    Vector2 firstCutAfter;
    shapedCorners(
        squareShape(), [](const ShapeCorner&) { return 10.0f; }, [](const ShapeCorner&) { return CornerShape(); },
        [&](const ShapeCorner&, Vector2 before, Vector2 after) {
            if (heard == 0) {
                firstCutBefore = before;
                firstCutAfter = after;
            }
            heard += 1;
        });
    EXPECT(heard == 4);
    EXPECT(firstCutBefore.distance(firstCutAfter) > 0);
}

TEST(shapedCornersOfAPathReturnsAPath) {
    Path2D rounded = shapedCorners(
        squareShape().outlines[0], [](const ShapeCorner&) { return 8.0f; }, [](const ShapeCorner&) { return CornerShape(); });
    EXPECT(isNear(rounded.signedArea(), 10000 - 4 * 64 * (1 - pi / 4), 2e-4f));
}

// MARK: Corner frame

TEST(cornerFrameGolden) {
    // corners.json cornerFrameAt: a (0,0), b (10,0), p (5,4); to (0.5, 1.5) = (7.5, 6); from the same = (-0.9, 0.375).
    CornerFrame frame = cornerFrame(Vector2(0, 0), Vector2(10, 0), Vector2(5, 4));
    Vector2 plane = frame.toPlane(Vector2(0.5f, 1.5f));
    EXPECT(isApproximatelyEqual(plane.x, 7.5f, 1e-5f) && isApproximatelyEqual(plane.y, 6, 1e-5f));
    Vector2 local = frame.fromPlane(Vector2(0.5f, 1.5f));
    EXPECT(isApproximatelyEqual(local.x, -0.9f, 1e-5f) && isApproximatelyEqual(local.y, 0.375f, 1e-5f));
}

TEST(cornerFrameMapsItsDefiningPoints) {
    CornerFrame frame = cornerFrame(Vector2(2, 1), Vector2(8, 5), Vector2(9, 0));
    EXPECT(frame.toPlane(Vector2(-1, 0)).distance(Vector2(2, 1)) < 1e-5f);
    EXPECT(frame.toPlane(Vector2(1, 0)).distance(Vector2(8, 5)) < 1e-5f);
    EXPECT(frame.toPlane(Vector2(0, 1)).distance(Vector2(9, 0)) < 1e-5f);
}

TEST(cornerFrameRoundTrips) {
    CornerFrame frame = cornerFrame(Vector2(2, 1), Vector2(8, 5), Vector2(9, 0));
    for (Vector2 local : {Vector2(0, 0), Vector2(-0.7f, 2.2f), Vector2(1.5f, -1)}) {
        Vector2 back = frame.fromPlane(frame.toPlane(local));
        EXPECT(back.distance(local) < 1e-4f);
    }
}

TEST(aCornerOnItsChordGetsAUnitSquareFrame) {
    CornerFrame frame = cornerFrame(Vector2(0, 0), Vector2(10, 0), Vector2(5, 0));
    EXPECT(frame.determinant != 0);
    EXPECT(frame.toPlane(Vector2(0, 1)).distance(Vector2(5, 5)) < 1e-4f);
    EXPECT_PROBLEM(cornerFrame(Vector2(1, 1), Vector2(1, 1), Vector2(2, 2)), "two different");
}

// MARK: Caps

namespace {

/// The golden stroke of corners.json "capped": 20 wide, 100 long, its end on top, here counterclockwise.
Shape2D strokeShape() { return shapeOf({polygonPath({{0, 0}, {20, 0}, {20, 100}, {0, 100}})}); }

StrokeEnd topEnd() { return StrokeEnd{Vector2(10, 100), Vector2(0, 1), 10}; }

} // namespace

TEST(capShapesAreTheStandardCurves) {
    Path2D triangle = CapShape::triangle().curve;
    EXPECT(triangle.start == Vector2(-1, 0) && triangle.segments().size() == 2);
    EXPECT(triangle.segments()[0].end() == Vector2(0, 1) && triangle.segments()[1].end() == Vector2(1, 0));
    EXPECT(CapShape::triangle(3).curve.segments()[0].end() == Vector2(0, 3));
    Path2D rectangle = CapShape::rectangle().curve;
    EXPECT(rectangle.segments().size() == 3 && rectangle.segments()[1].end() == Vector2(1, 0.5f));
    Path2D arc = CapShape::arc(2).curve;
    EXPECT(arc.segments().size() == 2 && arc.segments()[0].end() == Vector2(0, 2) && arc.segments()[1].end() == Vector2(1, 0));
    // The half circle of height 1 is a circle of radius 1: its middle handle points are at kappa.
    EXPECT(isApproximatelyEqual(CapShape::arc().curve.segments()[0].startHandle.y, 0.5522847f, 1e-5f));
    EXPECT(!CapShape::triangle().align.has_value());
}

TEST(aTriangleCapGivesTheGoldenPoint) {
    // corners.json capped, align 1: the end gains a triangle 10 high on a 20 wide stroke: area 2000 + 100.
    Shape2D result = cappedStroke(strokeShape(), topEnd(), CapShape::triangle());
    EXPECT(result.outlines.size() == 1);
    EXPECT(isNear(result.signedArea(), 2100, 2e-3f));
    EXPECT(isNear(result.bounds().maximum.y, 110, 1e-3f));
    EXPECT(result.contains(Vector2(10, 105)) && !result.contains(Vector2(1, 105)) && result.contains(Vector2(1, 50)));
}

TEST(alignMovesTheCapIntoTheStrokeByHalfTheCapsHeight) {
    // Golden: align a gives the tip 110 - 10 (1 - a) / 2... a = 1, 0.5, 0, -0.5, -1 put the tip at 110, 107.5, 105, 102.5, 100.
    std::vector<std::pair<float, float>> expectedTips{{1, 110}, {0.5f, 107.5f}, {0, 105}, {-0.5f, 102.5f}, {-1, 100}};
    for (const auto& [align, tip] : expectedTips) {
        CapShape cap = CapShape::triangle();
        cap.align = align;
        Shape2D result = cappedStroke(strokeShape(), topEnd(), cap);
        EXPECT(isNear(result.bounds().maximum.y, tip, 2e-3f));
        // The sides end 10 below the tip; the area is the stroke up to there plus the triangle.
        EXPECT(isNear(result.signedArea(), 20 * (tip - 10) + 100, 3e-3f));
    }
}

TEST(aRectangleCapRaisesASquareEnd) {
    Shape2D result = cappedStroke(strokeShape(), topEnd(), CapShape::rectangle(0.5f));
    // 0.5 high on a box 2 wide, scaled to the stroke's half-width 10: 5 units of square end.
    EXPECT(isNear(result.signedArea(), 2000 + 20 * 5, 3e-3f));
    EXPECT(isNear(result.bounds().maximum.y, 105, 2e-3f));
}

TEST(anArcCapRaisesAHalfRoundEnd) {
    Shape2D result = cappedStroke(strokeShape(), topEnd(), CapShape::arc(1));
    EXPECT(isNear(result.signedArea(), 2000 + pi * 100 / 2, 3e-3f));
    EXPECT(isNear(result.bounds().maximum.y, 110, 2e-3f));
}

TEST(aStrokeTurnedAnyWayIsCappedTheSameWay) {
    // The same stroke drawn clockwise, and a stroke lying on its side, give the same cap.
    Shape2D clockwise = strokeShape().reversed();
    EXPECT(isNear(std::fabs(cappedStroke(clockwise, topEnd(), CapShape::triangle()).signedArea()), 2100, 3e-3f));
    Shape2D sideways = shapeOf({polygonPath({{0, 0}, {100, 0}, {100, 20}, {0, 20}})});
    Shape2D result = cappedStroke(sideways, StrokeEnd{Vector2(100, 10), Vector2(1, 0), 10}, CapShape::triangle());
    EXPECT(isNear(result.signedArea(), 2100, 3e-3f));
    EXPECT(isNear(result.bounds().maximum.x, 110, 2e-3f));
}

TEST(anEmptyCapLeavesTheStrokeAlone) {
    CapShape nothing(Path2D(Vector2(0, 0)));
    EXPECT(cappedStroke(strokeShape(), topEnd(), nothing) == strokeShape());
}
