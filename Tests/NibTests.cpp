// Tests of Curves/Nib: naming a nib, its corners and stamp, the width of its ink, the transition keys and the weight
// compensation. Values marked "golden" are transcribed from graphics_cpp's tests/golden/nib.json (computed
// in double precision, compared here with a float tolerance); the rest are worked by hand.
#include <cmath>
#include <vector>

#include "../Curves/Nib.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float pi = Angle::pi;
using Segment = CubicBezier<Vector2>;

bool isNear(float actual, float expected, float tolerance = 1e-3f) { return isApproximatelyEqual(actual, expected, tolerance); }

bool isNear(Vector2 actual, Vector2 expected, float tolerance = 1e-3f) {
    return isNear(actual.x, expected.x, tolerance) && isNear(actual.y, expected.y, tolerance);
}

NibShape shapeOf(float width, float height, CornerRadius radiusX, CornerRadius radiusY) { return NibShape{width, height, radiusX, radiusY}; }

Nib rectangleNib(float width, float height, float radius, Angle angle = Angle::zero()) {
    Nib nib;
    nib.width = width;
    nib.height = height;
    nib.radiusX = radius;
    nib.radiusY = radius;
    nib.angle = angle;
    return nib;
}

float signedArea(const std::vector<Vector2>& polygon) {
    float area = 0;
    for (size_t index = 0; index < polygon.size(); index += 1) area += polygon[index].cross(polygon[(index + 1) % polygon.size()]);
    return area / 2;
}

} // namespace

// MARK: Naming a nib

TEST(nothingIsTheDefaultNib) {
    Nib nib = Nib::from();
    EXPECT(nib == Nib{});
    EXPECT(isNear(nib.width, 72) && isNear(nib.height, 72));
    EXPECT(nib.pointsPerCorner == 5 && nib.stampsPerSpan == 16);
}

TEST(aNumberIsARoundNibThatWide) {
    Nib nib = Nib::from(NibInput(40.0f));
    EXPECT(isNear(nib.width, 40) && isNear(nib.height, 40));
    EXPECT(Nib::radii(nib.radiusX)[0] == 1.0f);
}

TEST(aPenIsSolvedByItsStem) {
    // Golden nibOf {weight 80, contrast 0.4, stress 20}.
    Nib nib = Nib::from(NibInput(StemPen{80, 0.4f, Angle::degrees(20)}));
    EXPECT(isNear(nib.width, 83.17400f));
    EXPECT(isNear(nib.height, 49.90440f));
    EXPECT(isNear(nib.angle.degrees(), 20));
}

TEST(aPenWithoutContrastIsRoundAndAContrastOfOneIsHeldBelow) {
    EXPECT(isNear(Nib::stemNib(StemPen{60}).width, 60));
    // Golden stemNib {weight 40, contrast 0.99, stress 10}.
    Nib thin = Nib::stemNib(StemPen{40, 1.0f, Angle::degrees(10)});
    EXPECT(isNear(thin.width, 40.617f));
    EXPECT(isNear(thin.height, 0.40617f, 1e-4f));
}

TEST(aPenStemIsTheUprightWidthOfItsNib) {
    // Property: the stem of the nib solved from a pen is the pen's weight (a little under: the stamp is a polygon).
    // Golden stemWidth of {weight 80, contrast 0.4, stress 20} is 79.607.
    float stem = Nib::stemWidth(NibInput(StemPen{80, 0.4f, Angle::degrees(20)}));
    EXPECT(isNear(stem, 79.607f));
    EXPECT(stem <= 80 && stem > 79);
}

TEST(penOfReadsTheStemContrastAndStressBack) {
    // Golden stemPenOf of {w 70, h 30}: weight 70, contrast 0.57.
    StemPen pen = Nib::penOf(NibInput(rectangleNib(70, 30, 1)));
    EXPECT(isNear(pen.weight, 70) && isNear(pen.contrast, 0.57f) && isNear(pen.stress.degrees(), 0));
    StemPen given{80, 0.4f, Angle::degrees(20)};
    EXPECT(Nib::penOf(NibInput(given)) == given);
    EXPECT(isNear(Nib::penOf().weight, 72));
}

// MARK: Radii and shapes

TEST(radiiReadAsFourWhateverTheForm) {
    std::array<float, 4> one = Nib::radii(1.0f);
    EXPECT(one[0] == 1 && one[3] == 1);
    std::array<float, 4> short1 = Nib::radii(std::vector<float>{0.3f});
    EXPECT(isNear(short1[0], 0.3f) && isNear(short1[3], 0.3f));
    std::array<float, 4> twoValues = Nib::radii(std::vector<float>{0.5f, 1});
    EXPECT(isNear(twoValues[0], 0.5f) && isNear(twoValues[1], 1) && isNear(twoValues[2], 0.5f));
    EXPECT(std::isnan(Nib::radii(std::vector<float>{})[0]));
}

TEST(uniformGivesBackOneNumberWhenTheFourAgree) {
    CornerRadius same = Nib::uniform(std::vector<float>{0.3f});
    EXPECT(std::holds_alternative<float>(same) && isNear(std::get<float>(same), 0.3f));
    CornerRadius different = Nib::uniform(std::vector<float>{0.5f, 0.5f, 0.5f, 0.6f});
    EXPECT(std::holds_alternative<std::vector<float>>(different));
    CornerRadius nearlySame = Nib::uniform(std::vector<float>{0.5f, 0.5f, 0.5f, 0.5000001f});
    EXPECT(std::holds_alternative<float>(nearlySame));
}

TEST(mixingShapesBlendsSizesAndRadii) {
    // Golden mixShape: 60x40 round to 100x20 half-round, by 1.5 (not clamped).
    NibShape first = shapeOf(60, 40, 1.0f, 1.0f);
    NibShape second = shapeOf(100, 20, 0.5f, 0.5f);
    NibShape beyond = Nib::mixShape(first, second, 1.5f);
    EXPECT(isNear(beyond.width, 120) && isNear(beyond.height, 10));
    EXPECT(isNear(std::get<float>(beyond.radiusX), 0.25f) && isNear(std::get<float>(beyond.radiusY), 0.25f));
    EXPECT(Nib::mixShape(first, second, 0) == first);
    // Golden: per-corner radii blend corner by corner and tidy to one number when they agree.
    NibShape squareAndRound = shapeOf(50, 50, std::vector<float>{1, 0, 1, 0}, std::vector<float>{1, 0, 1, 0});
    NibShape target = shapeOf(80, 30, std::vector<float>{0.2f, 0.4f, 0.6f, 0.8f}, 0.3f);
    NibShape halfway = Nib::mixShape(squareAndRound, target, 0.5f);
    EXPECT(isNear(halfway.width, 65) && isNear(halfway.height, 40));
    std::array<float, 4> mixedX = Nib::radii(halfway.radiusX);
    EXPECT(isNear(mixedX[0], 0.6f) && isNear(mixedX[1], 0.2f) && isNear(mixedX[2], 0.8f) && isNear(mixedX[3], 0.4f));
    std::array<float, 4> mixedY = Nib::radii(halfway.radiusY);
    EXPECT(isNear(mixedY[0], 0.65f) && isNear(mixedY[1], 0.15f));
    NibShape same = Nib::mixShape(squareAndRound, squareAndRound, 0.5f);
    EXPECT(isNear(same.width, 50) && Nib::radii(same.radiusX)[1] == 0);
}

TEST(mixingBlendsAlongAStraightLine) {
    // Property: the blend at u of a, b is the blend of the blends (the amount is linear).
    NibShape a = shapeOf(10, 20, 0.0f, 1.0f);
    NibShape b = shapeOf(30, 60, 1.0f, 0.0f);
    NibShape quarter = Nib::mixShape(a, b, 0.25f);
    NibShape viaHalf = Nib::mixShape(a, Nib::mixShape(a, b, 0.5f), 0.5f);
    EXPECT(isNear(quarter.width, viaHalf.width) && isNear(quarter.height, viaHalf.height));
    EXPECT(isNear(std::get<float>(quarter.radiusX), 0.25f));
}

TEST(cornerArcsFindEachQuarterEllipse) {
    // Golden cornerArcs of 60x40, all round: radii (30, 20) about the origin.
    std::array<NibCornerArc, 4> round = Nib::cornerArcs(shapeOf(60, 40, 1.0f, 1.0f));
    for (const NibCornerArc& arc : round) {
        EXPECT(isNear(arc.radii, Vector2(30, 20)) && isNear(arc.arcCentre, Vector2(0, 0)));
    }
    EXPECT(round[1].signs == Vector2(-1, 1) && round[3].signs == Vector2(1, -1));
    // Golden: 50x50 with corners round, square, round, square.
    std::array<NibCornerArc, 4> mixed = Nib::cornerArcs(shapeOf(50, 50, std::vector<float>{1, 0, 1, 0}, std::vector<float>{1, 0, 1, 0}));
    EXPECT(isNear(mixed[1].arcCentre, Vector2(-25, 25)) && isNear(mixed[1].radii, Vector2(0, 0)));
    EXPECT(isNear(mixed[3].arcCentre, Vector2(25, -25)));
    // A radius past 1 or below 0 is held in range (golden {rx 2, ry -1}: square corners).
    std::array<NibCornerArc, 4> held = Nib::cornerArcs(shapeOf(30, 90, 2.0f, -1.0f));
    EXPECT(isNear(held[0].radii, Vector2(15, 0)));
}

// MARK: The stamp

TEST(aRoundStampFollowsItsEllipse) {
    // Golden stampPolygon of 60x40, all round, 5 per corner: 24 points, the first at (30, 0).
    std::vector<Vector2> stamp = Nib::stampPolygon(shapeOf(60, 40, 1.0f, 1.0f));
    EXPECT(stamp.size() == 24);
    EXPECT(isNear(stamp[0], Vector2(30, 0)));
    EXPECT(isNear(stamp[1], Vector2(28.531695f, 6.180340f)));
    EXPECT(isNear(stamp[2], Vector2(24.270510f, 11.755705f)));
    EXPECT(isNear(stamp[5], Vector2(0, 20)));
    // Property: every point lies on the ellipse x²/30² + y²/20² = 1.
    for (Vector2 point : stamp) EXPECT(isNear(point.x * point.x / 900 + point.y * point.y / 400, 1, 1e-4f));
}

TEST(squareCornersGiveTheFourCorners) {
    // Golden: {60x40, square} has 4 points; hand: (30, 20), (-30, 20), (-30, -20), (30, -20).
    std::vector<Vector2> stamp = Nib::stampPolygon(shapeOf(60, 40, 0.0f, 0.0f));
    EXPECT(stamp.size() == 4);
    EXPECT(isNear(stamp[0], Vector2(30, 20)) && isNear(stamp[2], Vector2(-30, -20)));
    EXPECT(isNear(signedArea(stamp), 60 * 40));
}

TEST(aStampIsCounterclockwiseAndHoldsItsHalfSides) {
    // Property: for any radii the area is positive and the points stay within the half-sides.
    for (float radius : {0.0f, 0.3f, 0.7f, 1.0f}) {
        std::vector<Vector2> stamp = Nib::stampPolygon(shapeOf(80, 30, radius, radius), Angle::zero(), 4);
        EXPECT(signedArea(stamp) > 0);
        for (Vector2 point : stamp) EXPECT(std::fabs(point.x) <= 40.001f && std::fabs(point.y) <= 15.001f);
    }
    // A rounded rectangle of a flat 50x50 with radius 1 gives an octagon-like disc smaller than the square.
    EXPECT(signedArea(Nib::stampPolygon(shapeOf(50, 50, 1.0f, 1.0f))) < 2500);
}

TEST(aStampTurnsWithItsRotation) {
    // Hand: a quarter turn takes the corner (30, 20) to (-20, 30).
    std::vector<Vector2> stamp = Nib::stampPolygon(shapeOf(60, 40, 0.0f, 0.0f), Angle::fromRadians(pi / 2));
    EXPECT(isNear(stamp[0], Vector2(-20, 30)));
    EXPECT(isNear(signedArea(stamp), 2400));
    EXPECT_PROBLEM(Nib::stampPolygon(shapeOf(60, 40, 1.0f, 1.0f), Angle::zero(), 0), "point");
}

// MARK: The width of the ink

TEST(aSquareNibLeavesItsSideAsInk) {
    // Hand: a 50x20 box, no turn, no rounding: upright (stem) 50 across, level 20 across, and a stroke at 45° is
    // the extent of the box across the diagonal: (50 + 20)·cos45° = 49.497.
    Nib box = rectangleNib(50, 20, 0);
    EXPECT(isNear(Nib::stemWidth(NibInput(box)), 50));
    EXPECT(isNear(Nib::nibWidth(NibInput(box), Angle::zero()), 20));
    EXPECT(isNear(Nib::nibWidth(NibInput(box), Angle::fromRadians(pi / 4)), 49.497f));
    EXPECT(isNear(Nib::stemWidth(NibInput(box), 0.5f), 25));
}

TEST(inkWidthMatchesTheGolden) {
    // Golden nibWidth: the default nib at 0.4 and π/4 radians.
    EXPECT(isNear(Nib::nibWidth({}, Angle::fromRadians(0.4f)), 71.734892f));
    EXPECT(isNear(Nib::nibWidth({}, Angle::fromRadians(pi / 4)), 71.113561f));
    // Golden: {w 50, h 20, square, angle -20} at angle -1 rad, scale 2.
    EXPECT(isNear(Nib::nibWidth(NibInput(rectangleNib(50, 20, 0, Angle::degrees(-20))), Angle::fromRadians(-1), 2), 92.413705f, 2e-3f));
    // Golden stemWidth of {w 60, h 30, angle 30} at scale 0.5.
    EXPECT(isNear(Nib::stemWidth(NibInput(rectangleNib(60, 30, 1, Angle::degrees(30))), 0.5f), 27.026801f));
    // Golden: following the stroke fully gives the nib's own height across whichever way it goes.
    Nib following = rectangleNib(70, 30, 1);
    following.follow = 1;
    following.radiusX = std::vector<float>{0.2f, 0.4f, 0.6f, 0.8f};
    following.radiusY = std::vector<float>{1, 0.5f, 0, 0.25f};
    EXPECT(isNear(Nib::nibWidth(NibInput(following), Angle::fromRadians(0.4f)), 70));
}

TEST(aStrokeHasNoWayRound) {
    // Property: the ink's width going one way equals going the opposite way.
    Nib nib = rectangleNib(60, 20, 0.5f, Angle::degrees(25));
    nib.follow = 0.5f;
    for (float angle : {0.2f, 1.1f, 2.5f, -0.7f}) {
        EXPECT(isNear(Nib::nibWidth(NibInput(nib), Angle::fromRadians(angle)), Nib::nibWidth(NibInput(nib), Angle::fromRadians(angle + pi)), 1e-3f));
    }
}

// MARK: Transition keys

TEST(noTransitionOrARoundNibLeavesNoKeys) {
    auto along = [](float) { return Angle::zero(); };
    EXPECT(!Nib::transitionKeys(NibInput(StemPen{90, 0.6f, Angle::degrees(20)}), 0, along));
    // A nib with no size is as wide every way. (A round nib is not: its polygon is a hair wider on the corners.)
    EXPECT(!Nib::transitionKeys(NibInput(0.0f), 0.6f, along, 17));
    EXPECT_PROBLEM(Nib::transitionKeys(NibInput(StemPen{90, 0.6f}), 0.6f, along, 1), "sample");
}

TEST(transitionKeysMatchTheGolden) {
    // Golden transitionKeysAt: pen {90, 0.6, 20°}, transition 0.6, the stroke's direction 0.3 + 2·share radians.
    auto along = [](float share) { return Angle::fromRadians(0.3f + 2.0f * share); };
    auto keys = Nib::transitionKeys(NibInput(StemPen{90, 0.6f, Angle::degrees(20)}), 0.6f, along);
    EXPECT(keys.has_value() && keys->size() == 129);
    if (!keys) return;
    EXPECT(isNear((*keys)[0].value, 0.9998328f, 1e-4f));
    EXPECT(isNear((*keys)[10].value, 0.9716330f, 1e-4f));
    EXPECT(isNear((*keys)[32].value, 0.8420089f, 1e-4f));
    EXPECT(isNear((*keys)[64].value, 1.1055015f, 1e-4f));
    EXPECT(isNear((*keys)[96].value, 1.0071698f, 1e-4f));
    EXPECT(isNear((*keys)[128].value, 1.0646359f, 1e-4f));
}

TEST(transitionKeysKeepThickAndThinAndNothingBetweenThem) {
    // Properties: key i sits at i/(n − 1); factor·width stays between the nib's thinnest and thickest ink; and the
    // thickest and thinnest ink are left as they are (factor 1).
    NibInput pen(StemPen{90, 0.6f, Angle::degrees(20)});
    float thinnest = 1e9f;
    float thickest = 0;
    for (int degrees = 0; degrees < 180; degrees += 1) {
        float width = Nib::nibWidth(pen, Angle::degrees(static_cast<float>(degrees)));
        thinnest = std::min(thinnest, width);
        thickest = std::max(thickest, width);
    }
    auto along = [](float share) { return Angle::fromRadians(pi * share); };
    for (float transition : {-1.0f, -0.4f, 0.6f, 1.0f}) {
        auto keys = Nib::transitionKeys(pen, transition, along, 33);
        EXPECT(keys.has_value() && keys->size() == 33);
        if (!keys) continue;
        for (size_t index = 0; index < keys->size(); index += 1) {
            float share = static_cast<float>(index) / 32;
            EXPECT(isNear((*keys)[index].time, share, 1e-6f));
            float width = Nib::nibWidth(pen, along(share));
            float placed = (*keys)[index].value * width;
            EXPECT(placed >= thinnest - 0.01f && placed <= thickest + 0.01f);
            if (isNear(width, thinnest, 0.01f) || isNear(width, thickest, 0.01f)) EXPECT(isNear((*keys)[index].value, 1, 1e-3f));
        }
    }
}

// MARK: Shapes given loosely

TEST(shapesOfNormalisesWhatItIsGiven) {
    Nib nib = rectangleNib(72, 72, 1);
    EXPECT(!Nib::shapesOf(std::nullopt, nib));
    // Golden: one partial shape {w 1, h 1} is one key at 0 with the nib's own radii.
    auto one = Nib::shapesOf(NibShapesInput(std::vector<PartialNibShape>{PartialNibShape{1.0f, 1.0f, std::nullopt, std::nullopt}}), nib);
    EXPECT(one.has_value() && one->size() == 1);
    if (one) {
        EXPECT(isNear((*one)[0].time, 0) && isNear((*one)[0].value.width, 1) && std::get<float>((*one)[0].value.radiusX) == 1.0f);
    }
    // Several partial shapes are spread evenly; width and height left out are 1, radii the nib's.
    auto three = Nib::shapesOf(NibShapesInput(std::vector<PartialNibShape>{PartialNibShape{}, PartialNibShape{2.0f, std::nullopt, 0.0f, std::nullopt}, PartialNibShape{}}), nib);
    EXPECT(three.has_value() && three->size() == 3);
    if (three) {
        EXPECT(isNear((*three)[1].time, 0.5f) && isNear((*three)[1].value.width, 2) && isNear((*three)[1].value.height, 1));
        EXPECT(std::get<float>((*three)[1].value.radiusX) == 0.0f && std::get<float>((*three)[1].value.radiusY) == 1.0f);
    }
    profile::Keys<NibShape> keys = {profile::Key<NibShape>{0.25f, shapeOf(3, 4, 0.5f, 0.5f)}};
    auto same = Nib::shapesOf(NibShapesInput(keys), nib);
    EXPECT(same.has_value() && (*same)[0].time == 0.25f);
    auto empty = Nib::shapesOf(NibShapesInput(std::vector<PartialNibShape>{}), nib);
    EXPECT(empty.has_value() && empty->empty());
}

// MARK: Weight compensation

TEST(aCurveIsARoundAndALineIsByItsLean) {
    WeightCompensation compensation{1.1f, 0.9f};
    // Golden: this curve counts as a round.
    EXPECT(isNear(Nib::segmentWeight(Segment({0, 0}, {0, 150}, {150, 300}, {300, 300}), compensation), 1.1f));
    // Lines: level, upright and the golden 28.8° line (55/100) at full diagonal.
    EXPECT(isNear(Nib::segmentWeight(Segment({0, 0}, {100, 0}, {200, 0}, {300, 0}), compensation), 1));
    EXPECT(isNear(Nib::segmentWeight(Segment({0, 0}, {0, 100}, {0, 200}, {0, 300}), compensation), 1));
    EXPECT(isNear(Nib::segmentWeight(Segment({0, 0}, {100, 55}, {200, 110}, {300, 165}), compensation), 0.9f));
    EXPECT(isNear(Nib::segmentWeight(Segment({0, 0}, {-100, 100}, {-200, 200}, {-300, 300}), compensation), 0.9f));
}

TEST(aNearlyLevelLineIsOnlyPartlyDiagonal) {
    // Hand: a line leaning 7.5° has u = 7.5/15 = 0.5, smoothStep(0.5) = 0.5, so a factor of 1 + (0.9 − 1)·0.5 = 0.95.
    float lean = std::tan(Angle::degrees(7.5f).radians);
    Segment line({0, 0}, {100, 100 * lean}, {200, 200 * lean}, {300, 300 * lean});
    EXPECT(isNear(Nib::segmentWeight(line, {1, 0.9f}), 0.95f, 2e-3f));
    // Property: leaning the other way, or measured off the vertical, gives the same factor.
    Segment mirrored({0, 0}, {100, -100 * lean}, {200, -200 * lean}, {300, -300 * lean});
    EXPECT(isNear(Nib::segmentWeight(mirrored, {1, 0.9f}), 0.95f, 2e-3f));
    Segment steep({0, 0}, {100 * lean, 100}, {200 * lean, 200}, {300 * lean, 300});
    EXPECT(isNear(Nib::segmentWeight(steep, {1, 0.9f}), 0.95f, 2e-3f));
    // With no compensation every segment is 1.
    EXPECT(isNear(Nib::segmentWeight(Segment({0, 0}, {0, 150}, {150, 300}, {300, 300}), {}), 1));
}

TEST(noCompensationLeavesNoWeightKeys) {
    std::vector<Segment> segments = {Segment({0, 0}, {0, 150}, {150, 300}, {300, 300}), Segment({300, 300}, {450, 300}, {600, 150}, {600, 0})};
    EXPECT(!Nib::weightCurve(segments, false, {}, 40));
    EXPECT(!Nib::weightCurve({}, false, {1.1f, 1}, 40));
    // Golden: all rounds under round 1.1 is one level, 1.1 from end to end.
    auto level = Nib::weightCurve(segments, false, {1.1f, 1}, 40);
    EXPECT(level.has_value() && level->size() == 2);
    if (level) EXPECT(isNear((*level)[0].value, 1.1f) && isNear((*level)[1].value, 1.1f) && isNear((*level)[1].time, 1));
}

TEST(weightEasesAcrossEachNode) {
    // Golden weightCurve: a level line, a round, a diagonal, with round 1.1 and ramp 40.
    std::vector<Segment> segments = {Segment({0, 0}, {100, 0}, {200, 0}, {300, 0}), Segment({0, 0}, {0, 150}, {150, 300}, {300, 300}),
                                     Segment({0, 0}, {100, 100}, {200, 200}, {300, 300})};
    auto keys = Nib::weightCurve(segments, false, {1.1f, 1}, 40);
    EXPECT(keys.has_value() && keys->size() == 6);
    if (!keys) return;
    EXPECT(isNear((*keys)[0].time, 0) && isNear((*keys)[0].value, 1));
    EXPECT(isNear((*keys)[1].time, 0.235507f, 2e-4f) && isNear((*keys)[1].value, 1));
    EXPECT(isNear((*keys)[2].time, 0.269151f, 2e-4f) && isNear((*keys)[2].value, 1.1f));
    EXPECT(isNear((*keys)[3].time, 0.626331f, 2e-4f) && isNear((*keys)[3].value, 1.1f));
    EXPECT(isNear((*keys)[4].time, 0.659975f, 2e-4f) && isNear((*keys)[4].value, 1));
    EXPECT(isNear((*keys)[5].time, 1) && isNear((*keys)[5].value, 1));
    // The ease leaves flat: handles half the ramp's share long, with no change in value.
    EXPECT(isNear((*keys)[1].handles->out->x, 0.016822f, 2e-5f) && isNear((*keys)[1].handles->out->y, 0));
    EXPECT(isNear((*keys)[2].handles->in->x, -0.016822f, 2e-5f));
    // Property: the eases are centred on the nodes (the keys either side of a node are equally far from it) and
    // the value is level between them.
    EXPECT(isNear(((*keys)[1].time + (*keys)[2].time) / 2 - 300.0f / (300 + 464.7f + 424.26f), 0, 5e-3f));
    EXPECT(isNear(profile::keysAt<float>(keys, 0.45f, 0), 1.1f));
}

TEST(aClosedStrokeEasesAcrossItsSeam) {
    // Golden: the same three segments closed under a diagonal of 0.9: the seam is the mean of the last and first.
    std::vector<Segment> segments = {Segment({0, 0}, {100, 0}, {200, 0}, {300, 0}), Segment({0, 0}, {0, 150}, {150, 300}, {300, 300}),
                                     Segment({0, 0}, {100, 100}, {200, 200}, {300, 300})};
    auto keys = Nib::weightCurve(segments, true, {1, 0.9f}, 40);
    EXPECT(keys.has_value() && keys->size() == 6);
    if (!keys) return;
    EXPECT(isNear((*keys)[0].time, 0) && isNear((*keys)[0].value, 0.95f));
    EXPECT(isNear((*keys)[1].time, 0.016822f, 2e-4f) && isNear((*keys)[1].value, 1));
    EXPECT(isNear((*keys)[2].time, 0.626331f, 2e-4f) && isNear((*keys)[2].value, 1));
    EXPECT(isNear((*keys)[3].time, 0.659975f, 2e-4f) && isNear((*keys)[3].value, 0.9f));
    EXPECT(isNear((*keys)[4].time, 0.983178f, 2e-4f) && isNear((*keys)[4].value, 0.9f));
    EXPECT(isNear((*keys)[5].time, 1) && isNear((*keys)[5].value, 0.95f));
}
