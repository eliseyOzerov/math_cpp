// Tests of shape recognition (Curves/ShapeRecognition.h): respacing, chord turns and corners
// worked by hand, then whole strokes: exact lines, arcs, circles and ellipses come back to 1e-4,
// polygons come back with their corners at the crossings of their sides, a "D" is one line and
// one arc, a stadium has smooth joints, and a squiggle stays freeform.
#include <cmath>
#include <numbers>
#include <vector>

#include "../Curves/ShapeRecognition.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-4f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// `count` points of an arc of `circle` from `startDegrees` through `sweepDegrees`; a whole turn
/// leaves out the repeated end.
std::vector<Vector2> arcPoints(Circle2 circle, int count, float startDegrees, float sweepDegrees) {
    std::vector<Vector2> points;
    bool whole = std::fabs(sweepDegrees) >= 360;
    int steps = whole ? count : count - 1;
    for (int index = 0; index < count; index += 1) {
        Angle angle = Angle::degrees(startDegrees + sweepDegrees * static_cast<float>(index) / static_cast<float>(steps));
        points.push_back(circle.centre + Vector2(angle.cosine(), angle.sine()) * circle.radius);
    }
    return points;
}

/// The corners joined by straight sides, `perSide` points to a side, closed back to the first
/// corner (without repeating it) when `closed`.
std::vector<Vector2> polygonPoints(const std::vector<Vector2>& corners, int perSide, bool closed) {
    std::vector<Vector2> points;
    size_t sides = closed ? corners.size() : corners.size() - 1;
    for (size_t side = 0; side < sides; side += 1) {
        Vector2 start = corners[side];
        Vector2 end = corners[(side + 1) % corners.size()];
        for (int index = 0; index < perSide; index += 1) {
            points.push_back(Vector2::linearBlend(start, end, static_cast<float>(index) / static_cast<float>(perSide)));
        }
    }
    if (!closed) points.push_back(corners.back());
    return points;
}

std::vector<Vector2> withNoise(std::vector<Vector2> points, float size, uint64_t seed) {
    SeededRandom random(seed);
    for (Vector2& point : points) point = point + Vector2(random.nextFloat(-size, size), random.nextFloat(-size, size));
    return points;
}

} // namespace

// Respacing.

TEST(evenlySpacedPointsOfAnOpenChainAreEvenAndKeepTheEnds) {
    // An L of legs 3 and 1 (length 4) at five points: spacing 1, so (0,0) (1,0) (2,0) (3,0) (3,1).
    std::vector<Vector2> spaced = evenlySpacedPoints({Vector2(0, 0), Vector2(3, 0), Vector2(3, 1)}, 5, false);
    EXPECT(spaced.size() == 5);
    EXPECT(isClose(spaced[1], Vector2(1, 0)));
    EXPECT(isClose(spaced[3], Vector2(3, 0)));
    EXPECT(spaced[4] == Vector2(3, 1));
}

TEST(evenlySpacedPointsOfAClosedChainGoRoundOnce) {
    // The unit square's corners, closed (length 4), at eight points: spacing ½, starting at (0,0).
    std::vector<Vector2> spaced =
        evenlySpacedPoints({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)}, 8, true);
    EXPECT(spaced.size() == 8);
    EXPECT(isClose(spaced[0], Vector2(0, 0)));
    EXPECT(isClose(spaced[1], Vector2(0.5f, 0)));
    EXPECT(isClose(spaced[3], Vector2(1, 0.5f)));
    EXPECT(isClose(spaced[7], Vector2(0, 0.5f)));
}

TEST(evenlySpacedPointsLeaveADegenerateChainAlone) {
    std::vector<Vector2> single{Vector2(2, 2)};
    EXPECT(evenlySpacedPoints(single, 10, false) == single);
    std::vector<Vector2> repeated{Vector2(1, 1), Vector2(1, 1)};
    EXPECT(evenlySpacedPoints(repeated, 10, false) == repeated);
}

// Chord turns and corners.

TEST(chordTurnsAreTheCornerAngleAtACornerAndZeroOnAStraight) {
    // An L with ten points to a leg: at the corner the chords over 3 points run along x and then
    // along y, a 90° turn; three points into the first leg both chords are along x, 0°.
    std::vector<Vector2> points = polygonPoints({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1)}, 10, false);
    std::vector<float> turns = chordTurns(points, 3, false);
    EXPECT(isApproximatelyEqual(turns[10], std::numbers::pi_v<float> / 2));
    EXPECT(isApproximatelyEqual(turns[4], 0));
    EXPECT(turns[0] == 0 && turns[2] == 0 && turns[20] == 0);
}

TEST(chordTurnsOnACircleAreTheWindowOverTheRadius) {
    // 72 points round a circle (5° apart): chords k points either side turn by k·5°.
    std::vector<Vector2> points = arcPoints(Circle2{Vector2(0, 0), 3}, 72, 0, 360);
    std::vector<float> turns = chordTurns(points, 4, true);
    for (float turn : turns) EXPECT(isApproximatelyEqual(turn, Angle::degrees(20).radians, 1e-4f));
}

TEST(strokeCornersFindTheCornersOfAPolygonAndNotOfACircle) {
    std::vector<Vector2> square = polygonPoints({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)}, 20, true);
    std::vector<int> corners = strokeCorners(square, 4, Angle::degrees(35), 0.7f, true);
    EXPECT(corners == (std::vector<int>{0, 20, 40, 60}));
    std::vector<int> openCorners = strokeCorners(polygonPoints({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1)}, 20, false),
                                                 4, Angle::degrees(35), 0.7f, false);
    EXPECT(openCorners == (std::vector<int>{20}));
    std::vector<Vector2> circle = arcPoints(Circle2{Vector2(0, 0), 1}, 80, 0, 360);
    EXPECT(strokeCorners(circle, 4, Angle::degrees(35), 0.7f, true).empty());
}

TEST(strokeCornersSkipATightSmoothBend) {
    // An ellipse of 4 by 1 turns by more than 35° over the window at its ends, but turns only
    // about half as much over half the window, so its ends are bends, not corners.
    Ellipse2 ellipse{Vector2(0, 0), 4, 1, Angle::degrees(0)};
    std::vector<Vector2> points;
    for (int index = 0; index < 400; index += 1) points.push_back(ellipse.point(Angle::degrees(0.9f * static_cast<float>(index))));
    std::vector<Vector2> spaced = evenlySpacedPoints(points, 256, true);
    std::vector<float> turns = chordTurns(spaced, 10, true);
    float largest = 0;
    for (float turn : turns) largest = std::fmax(largest, turn);
    EXPECT(largest > Angle::degrees(35).radians);
    EXPECT(strokeCorners(spaced, 10, Angle::degrees(35), 0.7f, true).empty());
}

// Single shapes.

TEST(recognisedShapeOfAStraightStrokeIsTheLine) {
    std::vector<Vector2> points = polygonPoints({Vector2(1, 2), Vector2(7, 10)}, 50, false);
    RecognisedShape shape = recognisedShape(points);
    EXPECT(shape.kind == RecognisedKind::line);
    EXPECT(!shape.closed);
    EXPECT(shape.parts.size() == 1 && shape.joints.empty());
    EXPECT(isClose(shape.parts[0].line.start, Vector2(1, 2)));
    EXPECT(isClose(shape.parts[0].line.end, Vector2(7, 10)));
    EXPECT(shape.rmsResidual < 1e-4f);
}

TEST(recognisedShapeOfAnArcIsThatArc) {
    Circle2 circle{Vector2(2, -1), 4};
    RecognisedShape shape = recognisedShape(arcPoints(circle, 60, 30, 120));
    EXPECT(shape.kind == RecognisedKind::arc);
    EXPECT(shape.parts.size() == 1);
    const ShapeArc& arc = shape.parts[0].arc;
    EXPECT(isClose(arc.circle.centre, circle.centre));
    EXPECT(isApproximatelyEqual(arc.circle.radius, 4, 1e-4f));
    EXPECT(isApproximatelyEqual(arc.start.degrees(), 30, 1e-2f));
    EXPECT(isApproximatelyEqual(arc.sweep.degrees(), 120, 1e-2f));
    // Drawn the other way, the sweep is negative and the tangent at the start turns with it.
    RecognisedShape backwards = recognisedShape(arcPoints(circle, 60, 150, -120));
    EXPECT(backwards.kind == RecognisedKind::arc);
    EXPECT(isApproximatelyEqual(backwards.parts[0].arc.sweep.degrees(), -120, 1e-2f));
    // Clockwise at 150° the stroke heads along −(−sin 150°, cos 150°) = (½, √3/2).
    EXPECT(isClose(backwards.parts[0].startDirection(), Vector2(0.5f, std::sqrt(3.0f) / 2)));
}

TEST(recognisedShapeOfAMoreThanHalfArcKeepsItsSweep) {
    RecognisedShape shape = recognisedShape(arcPoints(Circle2{Vector2(0, 0), 1}, 90, -90, 270));
    EXPECT(shape.kind == RecognisedKind::arc);
    EXPECT(isApproximatelyEqual(shape.parts[0].arc.sweep.degrees(), 270, 1e-2f));
}

TEST(recognisedShapeOfACircleIsThatCircle) {
    Circle2 circle{Vector2(-3, 5), 2.5f};
    RecognisedShape shape = recognisedShape(arcPoints(circle, 100, 17, 360));
    EXPECT(shape.kind == RecognisedKind::circle);
    EXPECT(shape.closed && shape.parts.empty());
    EXPECT(shape.circle.has_value());
    EXPECT(isClose(shape.circle->centre, circle.centre));
    EXPECT(isApproximatelyEqual(shape.circle->radius, 2.5f, 1e-4f));
}

TEST(recognisedShapeOfAnEllipseIsThatEllipse) {
    Ellipse2 drawn{Vector2(1, 1), 3, 1, Angle::degrees(25)};
    std::vector<Vector2> points;
    for (int index = 0; index < 120; index += 1) points.push_back(drawn.point(Angle::degrees(3.0f * static_cast<float>(index))));
    RecognisedShape shape = recognisedShape(points);
    EXPECT(shape.kind == RecognisedKind::ellipse);
    EXPECT(shape.ellipse.has_value());
    EXPECT(isClose(shape.ellipse->centre, drawn.centre));
    EXPECT(isApproximatelyEqual(shape.ellipse->majorRadius, 3, 1e-4f));
    EXPECT(isApproximatelyEqual(shape.ellipse->minorRadius, 1, 1e-4f));
    EXPECT(isApproximatelyEqual(shape.ellipse->rotation.degrees(), 25, 1e-2f));
}

TEST(recognisedShapeOfANoisyCircleAndLineStaysThemselves) {
    Circle2 circle{Vector2(0, 0), 10};
    RecognisedShape round = recognisedShape(withNoise(arcPoints(circle, 150, 0, 355), 0.15f, 7));
    EXPECT(round.kind == RecognisedKind::circle);
    EXPECT(round.circle->centre.distance(circle.centre) < 0.1f);
    EXPECT(isApproximatelyEqual(round.circle->radius, 10, 0.1f));
    RecognisedShape straight = recognisedShape(withNoise(polygonPoints({Vector2(0, 0), Vector2(20, 5)}, 80, false), 0.1f, 8));
    EXPECT(straight.kind == RecognisedKind::line);
    EXPECT(straight.parts[0].line.start.distance(Vector2(0, 0)) < 0.2f);
    EXPECT(straight.parts[0].line.end.distance(Vector2(20, 5)) < 0.2f);
}

// Polylines and chains.

TEST(recognisedShapeOfASquareIsAClosedPolylineWithRightAngleCorners) {
    std::vector<Vector2> corners{Vector2(0, 0), Vector2(4, 0), Vector2(4, 4), Vector2(0, 4)};
    RecognisedShape shape = recognisedShape(polygonPoints(corners, 30, true));
    EXPECT(shape.kind == RecognisedKind::polyline);
    EXPECT(shape.closed);
    EXPECT(shape.parts.size() == 4 && shape.joints.size() == 4);
    for (const ShapeJoint& joint : shape.joints) {
        EXPECT(!joint.smooth);
        EXPECT(isApproximatelyEqual(joint.turn.degrees(), 90, 1e-2f));
        bool atACorner = false;
        for (Vector2 corner : corners) atACorner = atACorner || isClose(joint.point, corner);
        EXPECT(atACorner);
    }
    for (size_t index = 0; index < shape.parts.size(); index += 1) {
        EXPECT(isClose(shape.parts[index].end(), shape.joints[index].point));
        EXPECT(isClose(shape.parts[(index + 1) % 4].start(), shape.joints[index].point));
    }
}

TEST(recognisedShapeOfAnOpenZigzagIsAPolylineWithItsCornersAtTheCrossings) {
    std::vector<Vector2> corners{Vector2(0, 0), Vector2(3, 3), Vector2(6, 0), Vector2(9, 3)};
    RecognisedShape shape = recognisedShape(polygonPoints(corners, 30, false));
    EXPECT(shape.kind == RecognisedKind::polyline);
    EXPECT(!shape.closed);
    EXPECT(shape.parts.size() == 3 && shape.joints.size() == 2);
    EXPECT(isClose(shape.joints[0].point, Vector2(3, 3)));
    EXPECT(isClose(shape.joints[1].point, Vector2(6, 0)));
    EXPECT(isApproximatelyEqual(shape.joints[0].turn.degrees(), -90, 1e-2f));
    EXPECT(isApproximatelyEqual(shape.joints[1].turn.degrees(), 90, 1e-2f));
}

TEST(recognisedShapeOfANoisyTriangleKeepsThreeCorners) {
    std::vector<Vector2> corners{Vector2(0, 0), Vector2(10, 0), Vector2(5, 8)};
    RecognisedShape shape = recognisedShape(withNoise(polygonPoints(corners, 60, true), 0.05f, 11));
    EXPECT(shape.kind == RecognisedKind::polyline);
    EXPECT(shape.parts.size() == 3);
    for (const ShapeJoint& joint : shape.joints) {
        float nearest = 1e9f;
        for (Vector2 corner : corners) nearest = std::fmin(nearest, joint.point.distance(corner));
        EXPECT(nearest < 0.15f);
    }
}

TEST(recognisedShapeOfADIsOneLineAndOneArc) {
    // A "D": up the straight side from (0,−2) to (0,2), then round a half circle of radius 2
    // about the origin back to the start. Two corners of −90° each where the arc meets the line.
    std::vector<Vector2> points = polygonPoints({Vector2(0, -2), Vector2(0, 2)}, 40, false);
    points.pop_back();
    std::vector<Vector2> half = arcPoints(Circle2{Vector2(0, 0), 2}, 63, 90, -180);
    points.insert(points.end(), half.begin(), half.end() - 1);
    RecognisedShape shape = recognisedShape(points);
    EXPECT(shape.kind == RecognisedKind::linesAndArcs);
    EXPECT(shape.closed);
    EXPECT(shape.parts.size() == 2 && shape.joints.size() == 2);
    int lines = 0;
    int arcs = 0;
    for (const ShapePart& part : shape.parts) {
        if (part.kind == ShapePart::Kind::line) lines += 1;
        if (part.kind == ShapePart::Kind::arc) {
            arcs += 1;
            EXPECT(isClose(part.arc.circle.centre, Vector2(0, 0)));
            EXPECT(isApproximatelyEqual(part.arc.circle.radius, 2, 1e-4f));
            EXPECT(isApproximatelyEqual(part.arc.sweep.degrees(), -180, 1e-2f));
        }
    }
    EXPECT(lines == 1 && arcs == 1);
    for (const ShapeJoint& joint : shape.joints) {
        EXPECT(!joint.smooth);
        EXPECT(isApproximatelyEqual(joint.turn.degrees(), -90, 1e-2f));
        EXPECT(isClose(joint.point, Vector2(0, 2)) || isClose(joint.point, Vector2(0, -2)));
    }
}

TEST(recognisedShapeOfAStadiumHasSmoothJoints) {
    // Two straight sides of length 6 and two half circles of radius 2, every joint tangent.
    std::vector<Vector2> points;
    std::vector<Vector2> bottom = polygonPoints({Vector2(-3, -2), Vector2(3, -2)}, 60, false);
    points.insert(points.end(), bottom.begin(), bottom.end() - 1);
    std::vector<Vector2> right = arcPoints(Circle2{Vector2(3, 0), 2}, 64, -90, 180);
    points.insert(points.end(), right.begin(), right.end() - 1);
    std::vector<Vector2> top = polygonPoints({Vector2(3, 2), Vector2(-3, 2)}, 60, false);
    points.insert(points.end(), top.begin(), top.end() - 1);
    std::vector<Vector2> left = arcPoints(Circle2{Vector2(-3, 0), 2}, 64, 90, 180);
    points.insert(points.end(), left.begin(), left.end() - 1);
    RecognisedShape shape = recognisedShape(points);
    EXPECT(shape.kind == RecognisedKind::linesAndArcs);
    EXPECT(shape.closed);
    EXPECT(shape.parts.size() == 4 && shape.joints.size() == 4);
    for (const ShapeJoint& joint : shape.joints) {
        EXPECT(joint.smooth);
        EXPECT(std::fabs(joint.turn.degrees()) < 1);
        bool atATangentPoint = isClose(joint.point, Vector2(3, -2), 1e-3f) || isClose(joint.point, Vector2(3, 2), 1e-3f)
            || isClose(joint.point, Vector2(-3, 2), 1e-3f) || isClose(joint.point, Vector2(-3, -2), 1e-3f);
        EXPECT(atATangentPoint);
    }
}

TEST(recognisedShapeOfASquiggleIsFreeform) {
    // A wave, y = 1.5 sin x over two periods: no corners, and it bends both ways inside that
    // one stretch, which no chain of lines and same-bending arcs follows.
    std::vector<Vector2> points;
    for (int index = 0; index <= 300; index += 1) {
        float x = 4 * std::numbers::pi_v<float> * static_cast<float>(index) / 300;
        points.push_back(Vector2(x, 1.5f * std::sin(x)));
    }
    RecognisedShape shape = recognisedShape(points);
    EXPECT(shape.kind == RecognisedKind::freeform);
    EXPECT(shape.parts.empty() && shape.rmsResidual == 0);
}

TEST(recognisedShapeOfDegenerateInputIsFreeform) {
    EXPECT(recognisedShape({}).kind == RecognisedKind::freeform);
    EXPECT(recognisedShape({Vector2(1, 1), Vector2(1, 1), Vector2(1, 1)}).kind == RecognisedKind::freeform);
    float notANumber = std::nanf("");
    EXPECT(recognisedShape({Vector2(0, 0), Vector2(notANumber, 1), Vector2(1, 0)}).kind == RecognisedKind::freeform);
}

// Properties.

TEST(recognisedShapeDoesNotDependOnScaleOrPlace) {
    // The D and a square, drawn a hundred times larger and moved: the same kind and parts, with
    // every joint moved and scaled the same way.
    std::vector<Vector2> square = polygonPoints({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)}, 25, true);
    std::vector<Vector2> moved;
    for (Vector2 point : square) moved.push_back(point * 100.0f + Vector2(-40, 70));
    RecognisedShape small = recognisedShape(square);
    RecognisedShape large = recognisedShape(moved);
    EXPECT(small.kind == large.kind && small.parts.size() == large.parts.size());
    for (size_t index = 0; index < small.joints.size() && index < large.joints.size(); index += 1) {
        EXPECT(isClose(small.joints[index].point * 100.0f + Vector2(-40, 70), large.joints[index].point, 1e-2f));
    }
}

TEST(recognisedShapePartsAlwaysMeetAtTheirJoints) {
    // Random triangles, quadrilaterals and pentagons (every corner turning well past 35°): each
    // part ends where its joint is and the next part starts there.
    SeededRandom random(31);
    for (int trial = 0; trial < 21; trial += 1) {
        std::vector<Vector2> corners;
        int count = 3 + trial % 3;
        for (int index = 0; index < count; index += 1) {
            Angle angle = Angle::degrees(360.0f * static_cast<float>(index) / static_cast<float>(count) + random.nextFloat(-5, 5));
            corners.push_back(Vector2(angle.cosine(), angle.sine()) * random.nextFloat(4.5f, 5.5f));
        }
        RecognisedShape shape = recognisedShape(polygonPoints(corners, 40, true));
        EXPECT(shape.kind == RecognisedKind::polyline);
        EXPECT(static_cast<int>(shape.parts.size()) == count);
        for (size_t index = 0; index < shape.joints.size(); index += 1) {
            EXPECT(isClose(shape.parts[index].end(), shape.joints[index].point, 1e-3f));
            EXPECT(isClose(shape.parts[(index + 1) % shape.parts.size()].start(), shape.joints[index].point, 1e-3f));
        }
    }
}
