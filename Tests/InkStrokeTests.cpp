// Tests of Curves/InkStroke: the nib stamped along a stroke, its sweeps round corners, its cut ends and tapers, and
// the hulls that join the stamps. Values marked "golden" are transcribed from graphics_cpp's tests/golden/
// ink.json (double precision; the case number is given). The hull counts and run-on points agree; only the values the
// original computes from bisections would not, and none of those are used here.
#include <algorithm>
#include <cmath>
#include <vector>

#include "../Curves/InkStroke.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Segment = CubicBezier<Vector2>;

bool isNear(float actual, float expected, float tolerance = 1e-2f) { return isApproximatelyEqual(actual, expected, tolerance); }

bool isNear(Vector2 actual, Vector2 expected, float tolerance = 1e-2f) {
    return isNear(actual.x, expected.x, tolerance) && isNear(actual.y, expected.y, tolerance);
}

/// The nib of the goldens: `width` by `height`, 3 points per corner, 6 stamps per segment.
Nib goldenNib(float width, float height, Angle angle = Angle::zero()) {
    Nib nib;
    nib.width = width;
    nib.height = height;
    nib.angle = angle;
    nib.pointsPerCorner = 3;
    nib.stampsPerSpan = 6;
    return nib;
}

InkStroke strokeOf(std::vector<Segment> segments) {
    InkStroke stroke;
    stroke.segments = std::move(segments);
    return stroke;
}

InkStroke levelLine() { return strokeOf({Segment({0, 0}, {100, 0}, {200, 0}, {300, 0})}); }

/// The arch of the goldens: up, over and down.
InkStroke arch() { return strokeOf({Segment({0, 0}, {0, 200}, {300, 300}, {300, 0})}); }

/// The V of the goldens (golden 14): down to a point at (150, 0) and up again, a sharp corner.
InkStroke vee() { return strokeOf({Segment({0, 300}, {50, 200}, {100, 100}, {150, 0}), Segment({150, 0}, {200, 100}, {250, 200}, {300, 300})}); }

/// The closed rounded square of the goldens (golden 12).
InkStroke roundedSquare() {
    InkStroke stroke = strokeOf({Segment({300, 150}, {300, 232.845f}, {232.845f, 300}, {150, 300}), Segment({150, 300}, {67.155f, 300}, {0, 232.845f}, {0, 150}),
                                 Segment({0, 150}, {0, 67.155f}, {67.155f, 0}, {150, 0}), Segment({150, 0}, {232.845f, 0}, {300, 67.155f}, {300, 150})});
    stroke.closed = true;
    return stroke;
}

float degreesOf(const std::optional<Angle>& angle) { return angle ? angle->degrees() : 999; }

Cut cutOf(CutMode mode, std::optional<float> taper = std::nullopt, std::optional<float> reach = std::nullopt) {
    Cut cut;
    cut.mode = mode;
    cut.taper = taper;
    cut.reach = reach;
    return cut;
}

} // namespace

TEST(aLevelStrokeIsStampedAtEvenPointsAndTrimmedSquare) {
    // Golden 0: a 300 line, a 44 disc, 6 stamps: points every 50, two planes cutting square, 6 hulls.
    InkResult ink = levelLine().ink(NibInput(goldenNib(44, 44)));
    EXPECT(ink.points.size() == 7 && ink.stamps.size() == 7 && ink.frames.size() == 7 && ink.widths.size() == 7);
    for (size_t index = 0; index < 7; index += 1) {
        EXPECT(isNear(ink.points[index], Vector2(50 * static_cast<float>(index), 0)));
        EXPECT(isNear(ink.shares[index], static_cast<float>(index) / 6, 1e-4f));
        EXPECT(isNear(ink.widths[index], 44));
    }
    EXPECT(isNear(ink.weight, 44));
    EXPECT(ink.nodes == std::vector<int>({0, 6}));
    EXPECT(ink.planes.size() == 2);
    EXPECT(ink.planes[0].end == InkEnd::start && isNear(ink.planes[0].point, Vector2(0, 0)) && isNear(ink.planes[0].normal, Vector2(1, 0)));
    EXPECT(ink.planes[1].end == InkEnd::end && isNear(ink.planes[1].point, Vector2(300, 0)) && isNear(ink.planes[1].normal, Vector2(-1, 0)));
    // Golden hull sizes 11, 15, 15, 15, 15, 11: the ends are cut flat.
    EXPECT(ink.hulls.size() == 6);
    if (ink.hulls.size() == 6) {
        EXPECT(ink.hulls[0].size() == 11 && ink.hulls[1].size() == 15 && ink.hulls[5].size() == 11);
    }
    // The first stamp is a 16-point disc about the origin: golden stamp point 1 is (19.0526, 11).
    EXPECT(ink.stamps[0].size() == 16);
    EXPECT(isNear(ink.stamps[0][1], Vector2(19.052559f, 11)));
    EXPECT(isNear(ink.frames[0].centre, Vector2(0, 0)) && isNear(ink.frames[0].width, 44) && isNear(ink.frames[0].rotation.radians, 0));
    EXPECT(ink.segments.size() == 1 && !ink.closed && ink.caps == InkCaps::butt);
}

TEST(theInkOfALevelStrokeFillsItsBandAndNothingBeyond) {
    // Property (hand-worked): square ends trim the ink to x in 0…300, and a disc of 44 reaches 22 either way.
    InkResult ink = levelLine().ink(NibInput(goldenNib(44, 44)));
    float lowestX = 1e9f, highestX = -1e9f, highestY = -1e9f;
    for (const std::vector<Vector2>& hull : ink.hulls) {
        for (Vector2 point : hull) {
            lowestX = std::min(lowestX, point.x);
            highestX = std::max(highestX, point.x);
            highestY = std::max(highestY, std::fabs(point.y));
        }
    }
    EXPECT(isNear(lowestX, 0, 1e-3f) && isNear(highestX, 300, 1e-3f) && isNear(highestY, 22, 1e-3f));
}

TEST(inkMovesWithTheStroke) {
    // Property: a stroke moved by (100, -40) gives the same ink moved by the same amount.
    InkResult base = arch().ink(NibInput(goldenNib(50, 24, Angle::degrees(30))));
    InkStroke archMoved = arch();
    for (Segment& segment : archMoved.segments) segment = segment.translated(Vector2(100, -40));
    InkResult shifted = archMoved.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))));
    EXPECT(base.points.size() == shifted.points.size() && base.hulls.size() == shifted.hulls.size());
    for (size_t index = 0; index < base.points.size(); index += 1) {
        EXPECT(isNear(base.points[index] + Vector2(100, -40), shifted.points[index], 1e-3f));
        EXPECT(isNear(base.widths[index], shifted.widths[index], 1e-3f));
        EXPECT(isNear(base.stamps[index][3] + Vector2(100, -40), shifted.stamps[index][3], 1e-3f));
    }
}

TEST(aSharpCornerIsSweptRoundInPlace) {
    // Golden 14: two straight-ish segments meeting at (150, 0), arriving at -63.43° and leaving at 63.43°: a turn of
    // 126.87°, in steps of at most 6°, so 22 copies of the corner point.
    InkResult ink = vee().ink(NibInput(goldenNib(44, 44)));
    EXPECT(ink.points.size() == 35 && ink.hulls.size() == 34);
    EXPECT(ink.nodes == std::vector<int>({0, 6, 28, 34}));
    for (size_t index = 6; index <= 28; index += 1) {
        EXPECT(isNear(ink.points[index], Vector2(150, 0), 1e-3f));
        EXPECT(isNear(ink.shares[index], 0.5f, 1e-4f));
    }
    EXPECT(!ink.headings[5] && !ink.headings[29]);
    EXPECT(isNear(degreesOf(ink.headings[6]), -63.43f) && isNear(degreesOf(ink.headings[7]), -57.67f));
    EXPECT(isNear(degreesOf(ink.headings[17]), 0) && isNear(degreesOf(ink.headings[28]), 63.43f));
    // The planes cut square across each end (golden normals).
    EXPECT(ink.planes.size() == 2 && isNear(ink.planes[0].normal, Vector2(0.4472136f, -0.8944272f)) && isNear(ink.planes[1].normal, Vector2(-0.4472136f, -0.8944272f)));
    // Hand-worked: a disc nib swept round a corner keeps its width, within the polygon's hair (3 per corner).
    for (float width : ink.widths) EXPECT(width > 42.5f && width <= 44.001f);
}

TEST(aSmoothTurnIsNotACorner) {
    // Golden 12: the closed rounded square runs smoothly through its four joins (the turns are under 20°): no
    // sweeps, 24 hulls, no planes (a closed stroke has no ends), nodes at the joins.
    InkResult ink = roundedSquare().ink(NibInput(goldenNib(44, 44)));
    EXPECT(ink.closed && ink.points.size() == 25 && ink.hulls.size() == 24 && ink.planes.empty());
    EXPECT(ink.nodes == std::vector<int>({0, 6, 12, 18, 24}));
    for (const std::optional<Angle>& heading : ink.headings) EXPECT(!heading);
    EXPECT(isNear(ink.points[1], Vector2(294.642f, 189.877f), 0.2f));
    EXPECT(isNear(ink.widths[0], 44) && isNear(ink.widths[1], 42.528f, 0.05f) && isNear(ink.widths[3], 42.501f, 0.05f));
}

TEST(anAngledCutRunsTheEndOnToTheCutLine) {
    // Golden 134: a level line with the start cut at 8° (too close to the stroke's direction: cut square) and the end
    // at 90° (a plumb cut: the run-on is as long as the stamp reaches, in 4 points of at most 8 units).
    InkStroke stroke = levelLine();
    Cut start = cutOf(CutMode::angle);
    start.angle = Angle::degrees(8);
    Cut end = cutOf(CutMode::angle);
    end.angle = Angle::degrees(90);
    stroke.cuts = Cuts{start, end};
    InkResult ink = stroke.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))));
    EXPECT(ink.points.size() == 11 && ink.hulls.size() == 10 && ink.segments.size() == 2);
    EXPECT(ink.nodes == std::vector<int>({0, 6}));
    EXPECT(isNear(ink.points[7], Vector2(306.933f, 0), 1e-2f) && isNear(ink.points[10], Vector2(327.731f, 0), 1e-2f));
    for (size_t index = 7; index < 11; index += 1) EXPECT(ink.shares[index] == 1.0f);
    EXPECT(ink.planes.size() == 2 && isNear(ink.planes[0].normal, Vector2(1, 0)) && isNear(ink.planes[1].normal, Vector2(-1, 0), 1e-4f));
    EXPECT(isNear(ink.widths[0], 32.043f, 0.02f));
    EXPECT(isNear(ink.frames[0].rotation.degrees(), 30, 1e-3f));
    // The run-on segment joins the stroke's end to its last point.
    EXPECT(isNear(ink.segments[1].start, Vector2(300, 0)) && isNear(ink.segments[1].end, Vector2(327.731f, 0), 1e-2f));
}

TEST(aLevelCutAtTheStartRunsOnBeforeIt) {
    // Golden 137: a plumb stroke, nib caps, the start cut level: 4 run-on points before the start (so the nodes begin
    // at 4), one plane (the other end is the nib's own), 9 hulls (the first is trimmed away).
    InkStroke stroke = strokeOf({Segment({0, 0}, {0, 100}, {0, 200}, {0, 300})});
    stroke.cuts = Cuts{cutOf(CutMode::level), std::nullopt};
    InkOptions options;
    options.caps = InkCaps::nib;
    InkResult ink = stroke.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))), options);
    EXPECT(ink.points.size() == 11 && ink.hulls.size() == 9);
    EXPECT(ink.nodes == std::vector<int>({4, 10}));
    EXPECT(isNear(ink.points[0], Vector2(0, -27.731f)) && isNear(ink.points[3], Vector2(0, -6.933f)) && isNear(ink.points[4], Vector2(0, 0)));
    EXPECT(ink.shares[0] == 0.0f && ink.shares[3] == 0.0f && isNear(ink.shares[5], 1.0f / 6, 1e-4f));
    EXPECT(ink.planes.size() == 1 && ink.planes[0].end == InkEnd::start && isNear(ink.planes[0].normal, Vector2(0, 1)));
    EXPECT(isNear(ink.widths[0], 43.5f, 0.05f));
    EXPECT(ink.caps == InkCaps::nib && ink.segments.size() == 2);
    // Property: no ink falls below the cut line y = 0 beyond rounding.
    for (const std::vector<Vector2>& hull : ink.hulls) for (Vector2 point : hull) EXPECT(point.y >= -1e-3f);
}

TEST(aBareEndOnALevelIsCutLevelThere) {
    // The start is bare (no cut given) and lies on y = 0, one of the levels: it is cut level, as a `level` cut is.
    InkStroke stroke = strokeOf({Segment({0, 0}, {0, 100}, {0, 200}, {0, 300})});
    InkOptions options;
    options.caps = InkCaps::nib;
    options.levels = {0};
    InkResult ink = stroke.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))), options);
    EXPECT(ink.points.size() == 11 && ink.nodes == std::vector<int>({4, 10}));
    EXPECT(ink.planes.size() == 1 && isNear(ink.planes[0].normal, Vector2(0, 1)));
    // With cuts off the levels are ignored.
    options.cuts = false;
    EXPECT(stroke.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))), options).points.size() == 7);
}

TEST(aTaperScalesTheNibTowardsItsEnds) {
    // Golden 154: a taper of -1 at both ends of a flat-cut 300 line shrinks the nib to nothing at the ends and
    // eases back to 44 by the middle (reach 150): widths 0, 11.407, 32.593, 44, 32.593, 11.407, 0.
    InkStroke stroke = levelLine();
    stroke.cuts = Cuts{cutOf(CutMode::flat, -1.0f), cutOf(CutMode::flat, -1.0f)};
    InkResult ink = stroke.ink(NibInput(goldenNib(44, 44)));
    std::vector<float> expected = {0, 11.407f, 32.593f, 44, 32.593f, 11.407f, 0};
    for (size_t index = 0; index < 7; index += 1) EXPECT(isNear(ink.widths[index], expected[index], 0.15f));
    EXPECT(isNear(ink.frames[0].width, 0) && isNear(ink.frames[3].width, 44, 1e-3f));
    // Hand: at the middle the smoothStep of (1 - 150/150) is 0, so the taper is gone and the width is the nib's.
    EXPECT(ink.hulls.size() == 6 && ink.hulls[0].size() >= 3);
}

TEST(aLevelCutWithATaperKeepsTheTaperOffTheRunOn) {
    // Golden 150: arch, level cuts at both ends, taper -0.5 reaching 30: 4 run-on points each end (nodes 4 and 10),
    // two planes, 10 hulls; the run-on stamps are at half the width the tapered end gives.
    InkStroke stroke = arch();
    stroke.cuts = Cuts{cutOf(CutMode::level, -0.5f, 30.0f), cutOf(CutMode::level, -0.5f, 30.0f)};
    InkResult ink = stroke.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))));
    EXPECT(ink.points.size() == 15 && ink.nodes == std::vector<int>({4, 10}) && ink.planes.size() == 2 && ink.hulls.size() == 10);
    EXPECT(isNear(ink.points[0], Vector2(0, -27.731f)) && isNear(ink.points[14], Vector2(300, -27.731f)));
    EXPECT(isNear(ink.widths[0], 21.75f, 0.1f) && isNear(ink.widths[5], 33.873f, 0.2f) && isNear(ink.widths[9], 49.691f, 0.3f));
    EXPECT(isNear(ink.frames[0].width, 25, 1e-3f) && isNear(ink.frames[0].height, 12, 1e-3f));
}

TEST(aRoundEndEasesTheNibIntoADisc) {
    // Golden 232: arch, round ends, a pointed pen: widths vary along the arch with the disc's ease at the ends.
    InkStroke stroke = arch();
    stroke.cuts = Cuts{cutOf(CutMode::round), cutOf(CutMode::round)};
    InkOptions options;
    options.model = InkModel::pointed;
    InkResult ink = stroke.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))), options);
    EXPECT(ink.planes.empty() && ink.points.size() == 7 && ink.hulls.size() == 6);
    std::vector<float> widths = {36.208f, 33.812f, 23.616f, 28.103f, 43.181f, 49.384f, 44.778f};
    for (size_t index = 0; index < 7; index += 1) EXPECT(isNear(ink.widths[index], widths[index], 0.1f));
    EXPECT(isNear(ink.frames[0].width, 37.2888f, 0.01f) && isNear(ink.frames[0].height, 37.2888f, 0.01f));
    EXPECT(isNear(ink.points[1], Vector2(22.222f, 90.278f), 0.01f));
    // A round end alone (no pointed pen) also lets the end stamp reach a disc as wide as the ink there.
    InkResult broad = stroke.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))));
    EXPECT(broad.planes.empty() && isNear(broad.frames[0].width, broad.frames[0].height, 1e-2f));
}

TEST(aPointedPenStampsADiscAsWideAsTheNibIsAcross) {
    // Golden 218: a pointed pen on a level line: a disc 32.043 wide, the 50x24 nib turned 30° measured across the
    // stroke, at every point.
    InkOptions options;
    options.model = InkModel::pointed;
    InkResult ink = levelLine().ink(NibInput(goldenNib(50, 24, Angle::degrees(30))), options);
    for (size_t index = 0; index < 7; index += 1) {
        EXPECT(isNear(ink.widths[index], 32.043f, 0.02f));
        EXPECT(isNear(ink.frames[index].width, 32.043f, 0.02f) && isNear(ink.frames[index].height, ink.frames[index].width, 1e-4f));
        EXPECT(isNear(ink.frames[index].rotation.radians, 0));
    }
    EXPECT(isNear(ink.widths[0], Nib::nibWidth(NibInput(goldenNib(50, 24, Angle::degrees(30))), Angle::zero()), 1e-3f));
}

TEST(spacingSetsTheStampsByDistance) {
    // Hand: spacing 120 on a 300 line is ceil(2.5) = 3 parts: points at 0, 100, 200, 300.
    InkOptions options;
    options.spacing = 120;
    InkResult ink = levelLine().ink(NibInput(goldenNib(44, 44)), options);
    EXPECT(ink.points.size() == 4 && isNear(ink.points[1], Vector2(100, 0)) && isNear(ink.points[2], Vector2(200, 0)));
    // The stamps per span in the options override the nib's; at least one.
    InkOptions fewer;
    fewer.stampsPerSpan = 2;
    EXPECT(levelLine().ink(NibInput(goldenNib(44, 44)), fewer).points.size() == 3);
    fewer.stampsPerSpan = 0;
    EXPECT(levelLine().ink(NibInput(goldenNib(44, 44)), fewer).points.size() == 2);
}

TEST(theCurvesAlongTheStrokeSizeAndMoveTheNib) {
    // Size: a constant factor of 2 doubles the stamp. Weight: the stroke's scale does the same and scales `weight`.
    InkStroke doubled = levelLine();
    doubled.size = profile::Keys<float>{profile::Key<float>{0.0f, 2.0f}};
    InkResult big = doubled.ink(NibInput(goldenNib(44, 44)));
    EXPECT(isNear(big.frames[3].width, 88, 1e-3f) && isNear(big.widths[3], 88, 1.5f));
    InkStroke heavy = levelLine();
    heavy.weight = 2.0f;
    InkResult heavier = heavy.ink(NibInput(goldenNib(44, 44)));
    EXPECT(isNear(heavier.weight, 88) && isNear(heavier.frames[3].width, 88, 1e-3f));
    // Offset 1 puts the right edge on the centreline of a stroke running right: the stamps lie above y = 0.
    InkStroke aside = levelLine();
    aside.offset = profile::Keys<float>{profile::Key<float>{0.0f, 1.0f}};
    InkResult shifted = aside.ink(NibInput(goldenNib(44, 44)));
    EXPECT(isNear(shifted.frames[2].centre.y, shifted.widths[2] / 2, 1e-2f));
    for (const std::vector<Vector2>& stamp : shifted.stamps) for (Vector2 point : stamp) EXPECT(point.y >= -1e-3f);
    // Rotate keys are in degrees and add to the nib's angle.
    InkStroke turned = levelLine();
    turned.rotate = profile::Keys<float>{profile::Key<float>{0.0f, 40.0f}};
    InkResult rotated = turned.ink(NibInput(goldenNib(50, 24, Angle::degrees(30))));
    EXPECT(isNear(rotated.frames[1].rotation.degrees(), 70, 1e-3f));
    // Follow turns the nib with the stroke: a stroke heading up (90°) leaves a followed nib upright-relative.
    InkStroke upright = strokeOf({Segment({0, 0}, {0, 100}, {0, 200}, {0, 300})});
    Nib following = goldenNib(50, 24);
    following.follow = 1;
    InkResult followed = upright.ink(NibInput(following));
    EXPECT(isNear(followed.frames[2].rotation.radians, 0, 1e-4f));
    InkResult level = levelLine().ink(NibInput(following));
    EXPECT(isNear(level.frames[2].rotation.degrees(), -90, 1e-3f));
}

TEST(shapesMorphTheNibAlongTheStroke) {
    // Golden 176: shapes keyed halfway to a wider square: at the middle the stamp is the key's 1.5 times as wide.
    InkStroke stroke = levelLine();
    stroke.shapes = profile::Keys<NibShape>{profile::Key<NibShape>{0.5f, NibShape{1.5f, 1.0f, 0.0f, 0.0f}}};
    InkResult ink = stroke.ink(NibInput(goldenNib(40, 20)));
    EXPECT(isNear(ink.frames[3].width, 60, 1e-3f) && isNear(ink.frames[3].height, 20, 1e-3f));
    EXPECT(Nib::radii(ink.frames[3].radiusX)[0] == 0);
    // Before the key the shape is the key's too (keys hold level).
    EXPECT(isNear(ink.frames[0].width, 60, 1e-3f));
}

TEST(badStrokesThrowAReadableProblem) {
    EXPECT_PROBLEM(InkStroke{}.ink(), "segment");
    InkStroke stroke = levelLine();
    stroke.cuts = Cuts{cutOf(CutMode::angle), std::nullopt};
    EXPECT_PROBLEM(stroke.ink(NibInput(goldenNib(44, 44))), "angle");
}
