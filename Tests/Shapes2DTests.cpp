// Tests of Curves/Shapes2D: bowed sides, soft polylines, ovals, knobs, runs, bends and lobes.
// Values are worked out by hand in the comments, or taken from graphics_cpp's golden file
// Curves2D/tests/golden/shape.json (double precision, so compared with about 1e-3).
#include <cmath>
#include <vector>

#include "../Curves/Shapes2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Cubic = CubicBezier<Vector2>;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// Whether `cubic` has these four points, given as eight numbers.
bool isCubic(const Cubic& cubic, const std::vector<float>& numbers, float tolerance = 1e-3f) {
    return isClose(cubic.start, Vector2(numbers[0], numbers[1]), tolerance) &&
           isClose(cubic.startHandle, Vector2(numbers[2], numbers[3]), tolerance) &&
           isClose(cubic.endHandle, Vector2(numbers[4], numbers[5]), tolerance) &&
           isClose(cubic.end, Vector2(numbers[6], numbers[7]), tolerance);
}

/// Whether each cubic starts where the one before ended.
bool isContinuous(const std::vector<Cubic>& cubics) {
    for (size_t index = 1; index < cubics.size(); index++)
        if (!isClose(cubics[index - 1].end, cubics[index].start, 1e-4f)) return false;
    return true;
}

const std::vector<Vector2> rightAngle{Vector2(0, 0), Vector2(100, 0), Vector2(100, 100)};

} // namespace

TEST(bowedPutsTheMiddleOfTheCurveABulgeOfTheChordOut) {
    // Golden: (0,0) to (100,40), bulge 0.2: handles on the thirds pushed by 0.2/0.75 of the normal (-40, 100).
    Cubic bowed = Shapes2D::bowed(Vector2(0, 0), Vector2(100, 40), 0.2f);
    EXPECT(isCubic(bowed, {0, 0, 22.6667f, 40, 56, 53.3333f, 100, 40}));
    // Property: the curve's middle lies bulge · chord length from the chord's middle, on its left.
    float chord = Vector2(100, 40).length();
    Vector2 offset = bowed.point(0.5f) - Vector2(50, 20);
    EXPECT(isApproximatelyEqual(offset.length(), 0.2f * chord, 1e-3f));
    EXPECT(offset.cross(Vector2(100, 40)) < 0); // left of the direction of travel
    // No bulge: the handles sit on the chord's thirds.
    EXPECT(isCubic(Shapes2D::bowed(Vector2(0, 0), Vector2(100, 40)), {0, 0, 33.3333f, 13.3333f, 66.6667f, 26.6667f, 100, 40}));
}

TEST(fitBulgesScalesDownOnlyWhereACornerWouldTurnPastTheLimit) {
    // Golden: a right angle with bulges 0.3 each. Limit 165° = 90° + 2·atan(4·s·0.3) gives s = 0.6394, bulge 0.19183.
    std::vector<float> fitted = Shapes2D::fitBulges(rightAngle, {0.3f, 0.3f});
    EXPECT(isApproximatelyEqual(fitted[0], 0.19183f, 1e-4f));
    EXPECT(isApproximatelyEqual(fitted[1], 0.19183f, 1e-4f));
    // Bowing one side out and the other in cancels, so nothing changes.
    std::vector<float> kept = Shapes2D::fitBulges(rightAngle, {2.0f, -2.0f});
    EXPECT(isApproximatelyEqual(kept[0], 2.0f) && isApproximatelyEqual(kept[1], -2.0f));
    // A corner that already turns back (turn about 174°) leaves no room: both go to 0.
    std::vector<Vector2> sharp{Vector2(0, 0), Vector2(100, 0), Vector2(0, 10)};
    std::vector<float> flat = Shapes2D::fitBulges(sharp, {0.3f, 0.3f});
    EXPECT(isApproximatelyEqual(flat[0], 0.0f, 1e-6f) && isApproximatelyEqual(flat[1], 0.0f, 1e-6f));
    // Property: after fitting, the corner turn is within the limit.
    float turn = Angle::signedTurn(Vector2(100, 0), Vector2(0, 100)).radians;
    float turned = std::fabs(turn + std::atan(4 * fitted[0] * 1) * 0 + std::atan(4 * fitted[0]) + std::atan(4 * fitted[1]));
    EXPECT(turned <= Angle::pi * 11 / 12 + 1e-3f);
    EXPECT(Shapes2D::fitBulges({}, {}).empty());
    EXPECT_PROBLEM(Shapes2D::fitBulges(rightAngle, {0.3f}), "one bulge per side");
}

TEST(softPolylineMakesStraightSidesBowedSidesAndArcCorners) {
    // Golden, radius 0 and no bulge: the two sides as straight cubics.
    std::vector<Cubic> straight = Shapes2D::softPolyline(rightAngle);
    EXPECT_EQUAL(straight.size(), size_t(2));
    EXPECT(isCubic(straight[0], {0, 0, 33.3333f, 0, 66.6667f, 0, 100, 0}));
    EXPECT(isCubic(straight[1], {100, 0, 100, 33.3333f, 100, 66.6667f, 100, 100}));
    // Golden, bulge 0.15: handles 0.15/0.75 · 100 = 20 to the left of each side.
    Softness bowedSides;
    bowedSides.bulge = 0.15f;
    std::vector<Cubic> bowed = Shapes2D::softPolyline(rightAngle, bowedSides);
    EXPECT(isCubic(bowed[0], {0, 0, 33.3333f, 20, 66.6667f, 20, 100, 0}));
    EXPECT(isCubic(bowed[1], {100, 0, 80, 33.3333f, 80, 66.6667f, 100, 100}));
    // Radius 1 on a right angle between two sides of 100: each side is cut away whole (the room of an end
    // side is all of it), leaving one quarter circle of radius 100 from (0,0) to (100,100) about (0,100).
    // Its handles are 4/3·tan(90°/4)·100 = 55.228 long.
    Softness fullyRound;
    fullyRound.radius = 1;
    std::vector<Cubic> round = Shapes2D::softPolyline(rightAngle, fullyRound);
    EXPECT_EQUAL(round.size(), size_t(1));
    EXPECT(isCubic(round[0], {0, 0, 55.2285f, 0, 100, 44.7715f, 100, 100}));
    // Radius 0.5 keeps half of each side straight, with a corner between; the whole is continuous.
    Softness halfRound;
    halfRound.radius = 0.5f;
    std::vector<Cubic> mixed = Shapes2D::softPolyline(rightAngle, halfRound);
    EXPECT_EQUAL(mixed.size(), size_t(3));
    EXPECT(isContinuous(mixed));
    EXPECT(isClose(mixed[0].end, Vector2(50, 0)) && isClose(mixed[2].start, Vector2(100, 50)));
    // Fewer than two points: nothing.
    EXPECT(Shapes2D::softPolyline({Vector2(1, 2)}).empty());
    // Bulges per side: 0.1 on the first, -0.1 on the second, and missing ones are 0.
    Softness perSide;
    perSide.sideBulges = {0.1f};
    std::vector<Cubic> some = Shapes2D::softPolyline(rightAngle, perSide);
    EXPECT(isApproximatelyEqual(some[0].startHandle.y, 13.3333f, 1e-3f));
    EXPECT(isApproximatelyEqual(some[1].startHandle.x, 100.0f, 1e-4f));
}

TEST(softWorksInTheMetricAndComesBackInPlainUnits) {
    // Golden: (0,0)-(100,0), bulge 0.1, metric (2,1). The side is 200 long in the metric, so its handles sit
    // 0.1/0.75 · 200 = 26.67 up, at the thirds of the side (x = 33.33, 66.67).
    Softness softness;
    softness.radius = 1;
    softness.bulge = 0.1f;
    Path2D path = Shapes2D::soft({Vector2(0, 0), Vector2(100, 0)}, softness, Vector2(2, 1));
    EXPECT_EQUAL(path.segments().size(), size_t(1));
    EXPECT(!path.isClosed());
    EXPECT(isCubic(Shapes2D::cubicsOf(path)[0], {0, 0, 33.3333f, 26.6667f, 66.6667f, 26.6667f, 100, 0}));
    EXPECT_PROBLEM(Shapes2D::soft({Vector2(0, 0)}), "at least two points");
}

TEST(ovalIsAClosedEllipseOfFourQuartersFromItsStart) {
    // Golden: the box (0,0)-(100,200) from the bottom: the first quarter leaves horizontally, handle 55.23% along.
    Path2D oval = Shapes2D::oval(0, 0, 100, 200);
    EXPECT(oval.isClosed());
    EXPECT(isClose(oval.start, Vector2(50, 0)));
    std::vector<Cubic> cubics = Shapes2D::cubicsOf(oval);
    EXPECT_EQUAL(cubics.size(), size_t(4));
    EXPECT(isCubic(cubics[0], {50, 0, 77.615f, 0, 100, 44.77f, 100, 100}, 3e-3f));
    EXPECT(isCubic(cubics[1], {100, 100, 100, 155.23f, 77.615f, 200, 50, 200}, 3e-3f));
    EXPECT(isCubic(cubics[3], {0, 100, 0, 44.77f, 22.385f, 0, 50, 0}, 3e-3f));
    EXPECT(isContinuous(cubics));
    // Golden, a flat box with tension 0.6: the cubics collapse onto the line y = 5, handles 0.6 of the way.
    OvalSettings flat;
    flat.tension = 0.6f;
    std::vector<Cubic> flatCubics = Shapes2D::cubicsOf(Shapes2D::oval(-10, 5, 40, 5, flat));
    EXPECT(isCubic(flatCubics[0], {15, 5, 30, 5, 40, 5, 40, 5}));
    // Starting on another side starts at its middle and keeps the area and the winding.
    float area = oval.signedArea();
    OvalSettings fromLeft;
    fromLeft.start = OvalStart::left;
    Path2D rotated = Shapes2D::oval(0, 0, 100, 200, fromLeft);
    EXPECT(isClose(rotated.start, Vector2(0, 100)));
    EXPECT(isApproximatelyEqual(rotated.signedArea(), area, 1.0f));
    EXPECT(area > 0); // counter-clockwise
    OvalSettings fromTop;
    fromTop.start = OvalStart::top;
    EXPECT(isClose(Shapes2D::oval(0, 0, 100, 200, fromTop).start, Vector2(50, 200)));
}

TEST(knobIsAnEllipseOnTheEndOfACurve) {
    // Golden: at (10,20) leaving along +x, half width 30, default shape (size 1.6): radius 48, centred on the end.
    std::vector<Cubic> knob = Shapes2D::knob(Vector2(10, 20), Vector2(1, 0), 30);
    EXPECT_EQUAL(knob.size(), size_t(4));
    EXPECT(isCubic(knob[0], {10, -28, 36.5104f, -28, 58, -6.5104f, 58, 20}));
    EXPECT(isContinuous(knob));
    // Set to the left (side +1), the centre moves by radius - half = 18 along the left normal (0,1).
    KnobShape toLeft;
    toLeft.side = 1;
    std::vector<Cubic> leaning = Shapes2D::knob(Vector2(10, 20), Vector2(1, 0), 30, toLeft);
    EXPECT(isClose(leaning[0].start, Vector2(10, -10)));
    // Its edge on the other side (the right) runs on from the curve's edge: the lowest point is 20 - 30 = -10.
    EXPECT(isApproximatelyEqual(leaning[0].start.y, 20 - 30, 1e-3f));
}

TEST(runOnTurnsACircularArcOrRunsStraight) {
    // Golden: straight for 60.
    EXPECT(isCubic(Shapes2D::runOn(Vector2(10, 20), Vector2(1, 0), 60), {10, 20, 30, 20, 50, 20, 70, 20}));
    // Golden: turning 120° to the left over 60. R = 60/(2π/3) = 28.648; the end is (R sin120°, R(1-cos120°)) = (24.81, 42.97) from the start.
    Cubic turning = Shapes2D::runOn(Vector2(10, 20), Vector2(1, 0), 60, Angle::degrees(120));
    EXPECT(isCubic(turning, {10, 20, 32.0532f, 20, 45.8364f, 43.8732f, 34.8098f, 62.9718f}));
    // Property: it is an arc of radius R, so every sampled point lies R from the centre (10, 20 + R).
    float radius = 60 / Angle::degrees(120).radians;
    for (float parameter : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f})
        EXPECT(isApproximatelyEqual(turning.point(parameter).distance(Vector2(10, 20 + radius)), radius, 0.15f));
    // Turning right bends the other way.
    EXPECT(Shapes2D::runOn(Vector2(0, 0), Vector2(1, 0), 10, Angle::degrees(-90)).end.y < 0);
}

TEST(bendBowsEveryPieceAwayFromAPoint) {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 50), Vector2(0, 100), Vector2(0, 150));
    path.addCubic(Vector2(40, 200), Vector2(80, 200), Vector2(120, 150));
    // Golden, 0.08 away from (50,50): the first piece (chord up, normal (-150, 0)) bows left by 16 on both
    // handles; the second (chord (120,0), normal (0,120)) bows up by 0.08/0.75 · 120 = 12.8.
    Path2D away = Shapes2D::bend(path, 0.08f, Vector2(50, 50));
    std::vector<Cubic> cubics = Shapes2D::cubicsOf(away);
    EXPECT(isCubic(cubics[0], {0, 0, -16, 50, -16, 100, 0, 150}));
    EXPECT(isCubic(cubics[1], {0, 150, 40, 212.8f, 80, 212.8f, 120, 150}));
    // From a point above, the second piece bows down instead.
    std::vector<Cubic> towards = Shapes2D::cubicsOf(Shapes2D::bend(path, 0.08f, Vector2(-20, 300)));
    EXPECT(isCubic(towards[0], {0, 0, 16, 50, 16, 100, 0, 150}));
    EXPECT(isCubic(towards[1], {0, 150, 40, 187.2f, 80, 187.2f, 120, 150}));
    // Nothing happens for 0, and a line becomes a bowed cubic.
    EXPECT(Shapes2D::bend(path, 0, Vector2(50, 50)) == path);
    Path2D line(Vector2(0, 0));
    line.addLine(Vector2(100, 0));
    Cubic bowedLine = Shapes2D::cubicsOf(Shapes2D::bend(line, 0.15f, Vector2(50, -10)))[0];
    EXPECT(isApproximatelyEqual(bowedLine.point(0.5f).y, 15.0f, 1e-3f)); // sagitta 0.15 · 100
    // Ends and the turn at them are kept.
    EXPECT(isClose(away.currentPoint(), path.currentPoint()) && isClose(away.start, path.start));
}

TEST(lobeHangsASoftPolygonOffALine) {
    // Golden: from (0,100) round to (0,0) with one corner 60 out: a single cubic bulging right.
    LobeSettings point;
    point.nodes = 1;
    point.reach = 60;
    Path2D onePoint = Shapes2D::lobe(Vector2(0, 100), Vector2(0, 0), point);
    std::vector<Cubic> cubics = Shapes2D::cubicsOf(onePoint);
    EXPECT_EQUAL(cubics.size(), size_t(1));
    EXPECT(isCubic(cubics[0], {0, 100, 31.225f, 73.9792f, 31.225f, 26.0208f, 0, 0}));
    // Golden: reach 50 about the axis 90° (a stem straight up), the default two nodes at radius 1: a half
    // circle of radius 50 made of two quarters.
    LobeSettings halfCircle;
    halfCircle.reach = 50;
    halfCircle.axis = Angle::degrees(90);
    std::vector<Cubic> bowl = Shapes2D::cubicsOf(Shapes2D::lobe(Vector2(0, 100), Vector2(0, 0), halfCircle));
    EXPECT_EQUAL(bowl.size(), size_t(2));
    EXPECT(isCubic(bowl[0], {0, 100, 27.6142f, 100, 50, 77.6142f, 50, 50}));
    EXPECT(isCubic(bowl[1], {50, 50, 50, 22.3858f, 27.6142f, 0, 0, 0}));
    // Golden, nodes 0 counts as one: from (0,100) to (5,50) off the chord.
    LobeSettings single;
    single.nodes = 0;
    single.reach = 60;
    std::vector<Cubic> chordLobe = Shapes2D::cubicsOf(Shapes2D::lobe(Vector2(0, 100), Vector2(5, 50), single));
    EXPECT_EQUAL(chordLobe.size(), size_t(1));
    EXPECT(isCubic(chordLobe[0], {0, 100, 23.1084f, 92.9304f, 26.2509f, 61.5056f, 5, 50}, 2e-3f));
    // Golden, metric (2,1): reach counts in the first unit, and the corners stay round in the metric.
    LobeSettings wide;
    wide.reach = 40;
    std::vector<Cubic> stretched = Shapes2D::cubicsOf(Shapes2D::lobe(Vector2(0, 100), Vector2(5, 50), wide, Vector2(2, 1)));
    EXPECT_EQUAL(stretched.size(), size_t(4));
    EXPECT(isContinuous(stretched));
    EXPECT(isCubic(stretched[1], {26.7232f, 110.6893f, 33.6268f, 113.4507f, 40.3425f, 104.4964f, 41.7232f, 90.6893f}, 3e-3f));
    // The lobe starts and ends on its ends.
    EXPECT(isClose(stretched.front().start, Vector2(0, 100), 2e-3f) && isClose(stretched.back().end, Vector2(5, 50), 2e-3f));
}

TEST(lobeThrowsWhenItHasNoRoom) {
    LobeSettings flat;
    flat.reach = 0;
    EXPECT_PROBLEM(Shapes2D::lobe(Vector2(0, 100), Vector2(0, 0), flat), "needs room");
    // Axis along the chord's wrong way round: the start lies behind the end, so there is no span.
    LobeSettings backwards;
    backwards.axis = Angle::degrees(-90);
    EXPECT_PROBLEM(Shapes2D::lobe(Vector2(0, 100), Vector2(0, 0), backwards), "needs room");
}

TEST(pathOfCubicsAndCubicsOfAreInverses) {
    Path2D square(Vector2(0, 0));
    square.addLine(Vector2(10, 0));
    square.addLine(Vector2(10, 10));
    square.close();
    // A closed outline with a gap counts its closing line: three pieces, the last straight back to the start.
    std::vector<Cubic> pieces = Shapes2D::cubicsOf(square);
    EXPECT_EQUAL(pieces.size(), size_t(3));
    EXPECT(isCubic(pieces[2], {10, 10, 6.6667f, 6.6667f, 3.3333f, 3.3333f, 0, 0}));
    Path2D rebuilt = Shapes2D::pathOfCubics(pieces, true);
    EXPECT(isContinuous(Shapes2D::cubicsOf(rebuilt)));
    EXPECT(isApproximatelyEqual(rebuilt.signedArea(), square.signedArea(), 1e-3f));
    EXPECT(Shapes2D::pathOfCubics({}, false).segments().empty());
}
