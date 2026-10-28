// Tests of Curves/InkOutline: the outline of an inked stroke, its plan and the plan's key. Values marked "golden" are
// transcribed from graphics_cpp's tests/golden/outline.json (double precision; the case number is given).
// Plan keys and piece counts are not compared with the goldens where they come from bisections.
#include <algorithm>
#include <cmath>
#include <vector>

#include "../Curves/InkOutline.h"
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

/// A round nib `width` wide with the default sampling (the goldens' `{w, h}` nibs).
Nib discNib(float width) {
    Nib nib;
    nib.width = width;
    nib.height = width;
    return nib;
}

InkStroke strokeOf(std::vector<Segment> segments) {
    InkStroke stroke;
    stroke.segments = std::move(segments);
    return stroke;
}

InkStroke levelLine() { return strokeOf({Segment({0, 0}, {100, 0}, {200, 0}, {300, 0})}); }

InkStroke arch() { return strokeOf({Segment({0, 0}, {0, 200}, {300, 300}, {300, 0})}); }

/// The closed rounded square of the goldens.
InkStroke roundedSquare() {
    InkStroke stroke = strokeOf({Segment({300, 150}, {300, 232.845f}, {232.845f, 300}, {150, 300}), Segment({150, 300}, {67.155f, 300}, {0, 232.845f}, {0, 150}),
                                 Segment({0, 150}, {0, 67.155f}, {67.155f, 0}, {150, 0}), Segment({150, 0}, {232.845f, 0}, {300, 67.155f}, {300, 150})});
    stroke.closed = true;
    return stroke;
}

InkOptions nibCaps() {
    InkOptions options;
    options.caps = InkCaps::nib;
    return options;
}

/// The largest distance from `point` to the outline's pieces, sampled: how far the point is from the contour.
float distanceToContour(const Path2D& contour, Vector2 point) {
    float nearest = 1e9f;
    Vector2 start = contour.start;
    for (const Path2D::Segment& segment : contour.segments()) {
        for (int step = 0; step <= 40; step++) {
            float parameter = static_cast<float>(step) / 40;
            Vector2 sample = segment.isLine() ? Vector2::linearBlend(start, segment.end(), parameter)
                                              : Segment(start, segment.startHandle, segment.endHandle, segment.end()).point(parameter);
            nearest = std::min(nearest, sample.distance(point));
        }
        start = segment.end();
    }
    return nearest;
}

/// Whether the contour runs clockwise (negative area by the shoelace formula over its corners and handles).
float areaOfContour(const Path2D& contour) {
    std::vector<Vector2> corners = {contour.start};
    for (const Path2D::Segment& segment : contour.segments()) corners.push_back(segment.end());
    float twiceArea = 0;
    for (size_t index = 0; index < corners.size(); index++) twiceArea += corners[index].cross(corners[(index + 1) % corners.size()]);
    return twiceArea / 2;
}

} // namespace

TEST(aLevelStrokeOutlinesAsABandWithTwoLinesAtItsEnds) {
    // Golden 0: a 300 long stroke inked with a 44 disc is the band y = ±22, x from 0 to 300: a cubic along the top, a
    // line down the end, a cubic back along the bottom and a line up the start.
    InkResult ink = levelLine().ink(NibInput(goldenNib(44, 44)));
    InkOutlineResult result = InkOutline::outline(ink);
    const auto& pieces = result.contour.segments();
    EXPECT(pieces.size() == 4);
    EXPECT(!pieces[0].isLine() && pieces[1].isLine() && !pieces[2].isLine() && pieces[3].isLine());
    EXPECT(result.contour.isClosed());
    EXPECT(isNear(result.contour.start, Vector2(0, 22)));
    EXPECT(isNear(pieces[0].end(), Vector2(300, 22)));
    EXPECT(isNear(pieces[1].end(), Vector2(300, -22)));
    EXPECT(isNear(pieces[2].end(), Vector2(0, -22)));
    EXPECT(result.anchors == 4 && result.contours.outlines.size() == 1);
    EXPECT(result.error < 0.01f);
    EXPECT(InkOutline::planKey(result.plan) == "L1 Eline R1 Sline");
}

TEST(theOutlineRunsClockwise) {
    // Property: left rail, then the end, then the right rail back: clockwise with y up, so the area is negative (-300 x 44).
    InkOutlineResult result = InkOutline::outline(levelLine().ink(NibInput(goldenNib(44, 44))));
    EXPECT(isNear(areaOfContour(result.contour), -300 * 44, 1e-3f));
}

TEST(aNibCapRoundsTheEndsOfTheOutline) {
    // Golden 1: with nib caps the same stroke's ends are rounded: the outline reaches x = 322 and x = -22 on the axis,
    // and each end is two cubics (6 pieces in all).
    InkResult ink = levelLine().ink(NibInput(goldenNib(44, 44)), nibCaps());
    InkOutlineResult result = InkOutline::outline(ink);
    EXPECT(result.contour.segments().size() == 6);
    EXPECT(result.plan.start.has_value() && result.plan.end.has_value());
    EXPECT(result.plan.runs && result.plan.runs->end == "c" && result.plan.runs->start == "c");
    EXPECT(isNear(result.contour.segments()[1].end(), Vector2(322, 0), 0.05f));
    EXPECT(isNear(result.contour.segments()[4].end(), Vector2(-22, 0), 0.05f));
    EXPECT(result.error < 1);
}

TEST(aTiltedNibGivesTheThinnerBandItLeaves) {
    // Golden 2: a 50 x 24 nib turned 30 degrees leaves a band y = +-16.2558 on a level stroke.
    InkResult ink = levelLine().ink(NibInput(goldenNib(50, 24, Angle::degrees(30))));
    InkOutlineResult result = InkOutline::outline(ink);
    EXPECT(result.contour.segments().size() == 4);
    EXPECT(isNear(result.contour.start.y, 16.2558f, 0.05f));
    EXPECT(isNear(result.contour.segments()[2].end().y, -16.2558f, 0.05f));
}

TEST(anArchNeedsOnePieceOnOneSideAndTwoOnTheOther) {
    // Golden 24: the arch inked with a 44 disc plans left [1], right [2], both ends lines; 5 anchors, worst error 0.924.
    InkResult ink = arch().ink(NibInput(discNib(44)));
    InkOutlineResult result = InkOutline::outline(ink);
    EXPECT(InkOutline::planKey(result.plan) == "L1 Eline R2 Sline");
    EXPECT(result.anchors == 5);
    EXPECT(isNear(result.error, 0.924f, 0.05f));
    EXPECT(isNear(result.contour.start, Vector2(-22.1f, 0), 0.05f));
    EXPECT(isNear(result.contour.segments()[0].end(), Vector2(322.05f, 0), 0.05f));
}

TEST(theOutlineStaysWithinTheToleranceOfTheInk) {
    // Property: every anchor of the outline of an arch is on the ink's edge, 22 from the centreline of a disc nib, and
    // the fit error is at most the tolerance.
    InkResult ink = arch().ink(NibInput(discNib(44)));
    InkOutlineResult result = InkOutline::outline(ink, std::nullopt, 0.5f);
    EXPECT(result.error <= 0.5f);
    Segment centreline = ink.segments[0];
    Vector2 cursor = result.contour.start;
    for (const Path2D::Segment& segment : result.contour.segments()) {
        float nearest = 1e9f;
        for (int step = 0; step <= 400; step++) nearest = std::min(nearest, centreline.point(static_cast<float>(step) / 400).distance(cursor));
        // The two ends are lines across the stroke, so an anchor on a rail is 22 away, one on the end line 22 or less.
        EXPECT(nearest <= 22.5f);
        cursor = segment.end();
    }
}

TEST(aClosedStrokeIsTwoRings) {
    // A closed stroke has two rings, outer and inner, each closed on itself (no ends); the first is the contour.
    InkResult ink = roundedSquare().ink(NibInput(goldenNib(30, 30)));
    InkOutlineResult result = InkOutline::outline(ink);
    EXPECT(result.contours.outlines.size() == 2);
    EXPECT(result.contour == result.contours.outlines[0]);
    EXPECT(!result.plan.start.has_value() && !result.plan.end.has_value());
    // The rings are 15 outside and inside the centreline: the left rail (the stroke runs counterclockwise) is the inner.
    float innerReach = distanceToContour(result.contours.outlines[0], Vector2(150, 150));
    float outerReach = distanceToContour(result.contours.outlines[1], Vector2(150, 150));
    EXPECT(innerReach > 0 && outerReach > 0);
    EXPECT(std::fabs(innerReach - outerReach) > 20);
    EXPECT(result.contours.outlines[0].segments().size() == result.contours.outlines[1].segments().size());
    EXPECT(result.anchors == static_cast<int>(result.contours.outlines[0].segments().size() * 2));
}

TEST(aPlanFitsTheOutlineTheSameWayEachTime) {
    // Property: fitting to a stroke's own plan gives the contour the stroke made alone.
    InkResult ink = arch().ink(NibInput(discNib(44)));
    InkOutlinePlan plan = InkOutline::plan(ink);
    InkOutlineResult alone = InkOutline::outline(ink);
    InkOutlineResult given = InkOutline::outline(ink, plan);
    EXPECT(alone.plan == plan && given.plan == plan);
    EXPECT(alone.contour == given.contour);
}

TEST(aMorePiecesPlanFitsAtLeastAsWell) {
    // Property: asking for more pieces never makes the worst error larger.
    InkResult ink = arch().ink(NibInput(discNib(44)));
    InkOutlinePlan coarse;
    coarse.left = {1};
    coarse.right = {1};
    InkOutlinePlan fine = coarse;
    fine.left = {4};
    fine.right = {4};
    InkOutlineResult coarseResult = InkOutline::outline(ink, coarse), fineResult = InkOutline::outline(ink, fine);
    EXPECT(coarseResult.anchors < fineResult.anchors);
    EXPECT(fineResult.error <= coarseResult.error + 1e-3f);
}

TEST(theSharedPlanTakesTheMostPiecesInEachPlace) {
    InkOutlinePlan first;
    first.left = {1, 3};
    first.right = {2, 1};
    InkOutlinePlan second;
    second.left = {2, 1};
    second.right = {1, 1};
    std::optional<InkOutlinePlan> shared = InkOutline::sharedPlan({first, second});
    EXPECT(shared && shared->left == std::vector<int>({2, 3}) && shared->right == std::vector<int>({2, 1}));
    EXPECT(shared && !shared->start && !shared->end);
    // Rounded ends take the larger count.
    first.start = 2;
    first.end = 4;
    second.start = 3;
    second.end = 2;
    std::optional<InkOutlinePlan> rounded = InkOutline::sharedPlan({first, second});
    first.runs = InkOutlineRuns{"c", "c"};
    second.runs = InkOutlineRuns{"c", "c"};
    rounded = InkOutline::sharedPlan({first, second});
    EXPECT(rounded && rounded->start == 3 && rounded->end == 4);
}

TEST(plansThatCannotShareGiveNothing) {
    InkOutlinePlan one;
    one.left = {1};
    one.right = {1};
    InkOutlinePlan two = one;
    two.left = {1, 1};
    two.right = {1, 1};
    EXPECT(!InkOutline::sharedPlan({one, two}));
    InkOutlinePlan rounded = one;
    rounded.start = 2;
    EXPECT(!InkOutline::sharedPlan({one, rounded}));
    EXPECT(!InkOutline::sharedPlan({}));
}

TEST(aSharedPlanOutlinesInkOfDifferentWeightsWithTheSamePieces) {
    // The point of a shared plan: two weights of one stroke outline to the same number of pieces, so they interpolate.
    InkResult thin = arch().ink(NibInput(discNib(20)));
    InkResult thick = arch().ink(NibInput(discNib(60)));
    std::optional<InkOutlinePlan> shared = InkOutline::sharedPlan({InkOutline::plan(thin), InkOutline::plan(thick)});
    EXPECT(shared.has_value());
    InkOutlineResult thinResult = InkOutline::outline(thin, shared), thickResult = InkOutline::outline(thick, shared);
    EXPECT(thinResult.contour.segments().size() == thickResult.contour.segments().size());
    EXPECT(InkOutline::planKey(thinResult.plan) == InkOutline::planKey(thickResult.plan));
}

TEST(aPlanKeyWritesTheSpansEndsAndRuns) {
    // Golden planKey of {left [1,2], right [3], start 2, end nullopt, runs c,l} is "L1.2 Eline R3 S2:c".
    InkOutlinePlan plan;
    plan.left = {1, 2};
    plan.right = {3};
    plan.start = 2;
    plan.runs = InkOutlineRuns{"c", "l"};
    EXPECT(InkOutline::planKey(plan) == "L1.2 Eline R3 S2:c");
    plan.end = 3;
    EXPECT(InkOutline::planKey(plan) == "L1.2 E3:l R3 S2:c");
}

TEST(anInkWithNoPointsHasNoOutline) {
    EXPECT_PROBLEM(InkOutline::outline(InkResult()), "no outline");
}
