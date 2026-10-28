// Tests of AffineTransform2 (Linear/AffineTransform2.cpp) and of the plane helpers added with it:
// Angle::halfTurn, Angle::direction, Vector2::fromAngle / rotated / mean / toMetric / fromMetric and
// smoothStep. Hand-worked values, plus a few transcribed from graphics_cpp Curves2D/tests/golden/geometry.json.
#include <cmath>
#include <vector>

#include "../Linear/AffineTransform2.h"
#include "../Linear/Angle.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

bool isClose(const AffineTransform2& first, const AffineTransform2& second, float tolerance = 1e-5f) {
    return isClose(first.xAxis, second.xAxis, tolerance) && isClose(first.yAxis, second.yAxis, tolerance)
        && isClose(first.translation, second.translation, tolerance);
}

AffineTransform2 sampleMap() { return AffineTransform2::rotation(Angle::degrees(30), Vector2(10, 20)); }

} // namespace

TEST(affineRotationTurnsAQuarterCounterclockwise) {
    // (1, 0) turned 90° counterclockwise is (0, 1); (0, 1) goes to (-1, 0).
    AffineTransform2 quarter = AffineTransform2::rotation(Angle::degrees(90));
    EXPECT(isClose(quarter.applied(Vector2(1, 0)), Vector2(0, 1)));
    EXPECT(isClose(quarter.applied(Vector2(0, 1)), Vector2(-1, 0)));
}

TEST(affineRotationAboutAPivotMatchesTheGoldenValues) {
    // graphics_cpp geometry.json: rotation(30, [10, 20]) = [0.8660254, 0.5, -0.5, 0.8660254, 11.339746, -2.3205081].
    AffineTransform2 map = sampleMap();
    EXPECT(isClose(map.xAxis, Vector2(0.8660254f, 0.5f), 1e-5f));
    EXPECT(isClose(map.yAxis, Vector2(-0.5f, 0.8660254f), 1e-5f));
    EXPECT(isClose(map.translation, Vector2(11.339746f, -2.3205081f), 1e-4f));
    EXPECT(isClose(map.applied(Vector2(10, 20)), Vector2(10, 20), 1e-4f)); // the pivot stays
}

TEST(affineScalingAboutAPivotAndMirroring) {
    // golden: scaling(2, 3, [1, 1]) = [2, 0, 0, 3, -1, -2]; the pivot (1, 1) stays, (2, 2) goes to (3, 4).
    AffineTransform2 map = AffineTransform2::scaling(Vector2(2, 3), Vector2(1, 1));
    EXPECT(isClose(map.translation, Vector2(-1, -2)));
    EXPECT(isClose(map.applied(Vector2(1, 1)), Vector2(1, 1)));
    EXPECT(isClose(map.applied(Vector2(2, 2)), Vector2(3, 4)));
    EXPECT(isApproximatelyEqual(map.determinant(), 6));
    EXPECT(AffineTransform2::scaling(Vector2(-1, 1)).determinant() < 0); // a negative factor mirrors
}

TEST(affineShearLeansLinesAcrossTheAxis) {
    // golden: shear(12°) = [1, 0, -0.2125566, 1, 0, 0]; with a 45° lean (x, y) goes to (x - y, y).
    AffineTransform2 twelve = AffineTransform2::shear(Angle::degrees(12));
    EXPECT(isClose(twelve.xAxis, Vector2(1, 0)));
    EXPECT(isClose(twelve.yAxis, Vector2(-0.2125566f, 1), 1e-5f));
    AffineTransform2 fortyFive = AffineTransform2::shear(Angle::degrees(45));
    EXPECT(isClose(fortyFive.applied(Vector2(3, 2)), Vector2(1, 2), 1e-5f));
    EXPECT(isApproximatelyEqual(fortyFive.determinant(), 1, 1e-5f)); // a shear keeps area
}

TEST(affineShearAboutAPivotAlongTheYAxisMatchesTheGolden) {
    // golden: shear(12, 90, [5, 5]) = [1, 0.2125566, ~0, 1, 0, -1.0627828].
    AffineTransform2 map = AffineTransform2::shear(Angle::degrees(12), Angle::degrees(90), Vector2(5, 5));
    EXPECT(isClose(map.xAxis, Vector2(1, 0.2125566f), 1e-5f));
    EXPECT(isClose(map.yAxis, Vector2(0, 1), 1e-5f));
    EXPECT(isClose(map.translation, Vector2(0, -1.0627828f), 1e-4f));
    EXPECT(isClose(map.applied(Vector2(5, 5)), Vector2(5, 5), 1e-4f));
}

TEST(affineTranslationAlongAnAxis) {
    // golden: shift(10, 45) = translation (7.0710678, 7.0710678); shift(10) = (10, 0).
    EXPECT(isClose(AffineTransform2::translationAlong(Angle::degrees(45), 10).translation, Vector2(7.0710678f, 7.0710678f)));
    EXPECT(isClose(AffineTransform2::translationAlong(Angle(), 10).translation, Vector2(10, 0)));
    EXPECT(isClose(AffineTransform2::translationBy(Vector2(2, -3)).applied(Vector2(1, 1)), Vector2(3, -2)));
}

TEST(affineAppliedToVectorIgnoresTheMove) {
    AffineTransform2 map = AffineTransform2::translationBy(Vector2(5, 5)).then(AffineTransform2::scaling(Vector2(2, 2)));
    EXPECT(isClose(map.appliedToVector(Vector2(1, 1)), Vector2(2, 2)));
    EXPECT(isClose(map.applied(Vector2(1, 1)), Vector2(12, 12)));
}

TEST(affineThenAppliesTheFirstMapFirst) {
    // Move by (1, 0) then turn a quarter: (0, 0) -> (1, 0) -> (0, 1). The other order gives (1, 0).
    AffineTransform2 move = AffineTransform2::translationBy(Vector2(1, 0));
    AffineTransform2 turn = AffineTransform2::rotation(Angle::degrees(90));
    EXPECT(isClose(move.then(turn).applied(Vector2(0, 0)), Vector2(0, 1)));
    EXPECT(isClose(turn.then(move).applied(Vector2(0, 0)), Vector2(1, 0)));
}

TEST(affineThenMatchesApplyingBothMapsInTurn) {
    AffineTransform2 first = sampleMap();
    AffineTransform2 second = AffineTransform2::shear(Angle::degrees(12));
    AffineTransform2 both = first.then(second);
    for (Vector2 point : {Vector2(0, 0), Vector2(3, -4), Vector2(-7, 2.5f)}) {
        EXPECT(isClose(both.applied(point), second.applied(first.applied(point)), 1e-4f));
    }
    EXPECT(isClose(first.then(AffineTransform2::identity()), first)); // the identity changes nothing
}

TEST(affineInverseUndoesTheMap) {
    AffineTransform2 map = sampleMap().then(AffineTransform2::scaling(Vector2(2, 0.5f)));
    std::optional<AffineTransform2> inverse = map.inverse();
    EXPECT(inverse.has_value());
    EXPECT(isClose(map.then(*inverse), AffineTransform2::identity(), 1e-4f));
    EXPECT(isClose(inverse->applied(map.applied(Vector2(3, 4))), Vector2(3, 4), 1e-4f));
    // Scaling by (2, 4) has the hand-worked inverse scaling by (0.5, 0.25).
    EXPECT(isClose(*AffineTransform2::scaling(Vector2(2, 4)).inverse(), AffineTransform2::scaling(Vector2(0.5f, 0.25f))));
}

TEST(affineInverseOfAFlatteningMapIsNothing) {
    EXPECT(!AffineTransform2::scaling(Vector2(0, 1)).inverse().has_value());
    EXPECT(isApproximatelyEqual(AffineTransform2::scaling(Vector2(0, 1)).determinant(), 0));
}

TEST(affinePartlyBlendsFromTheIdentity) {
    // golden: partly(rotation(30°, [10, 20]), 0.25) has xAxis.x = 0.9665064 (1 + (0.8660254 - 1) / 4).
    AffineTransform2 quarter = sampleMap().partly(0.25f);
    EXPECT(isApproximatelyEqual(quarter.xAxis.x, 0.9665064f, 1e-5f));
    EXPECT(isApproximatelyEqual(quarter.xAxis.y, 0.125f, 1e-5f));
    // Exact for moves: a quarter of a move by (8, 4) is a move by (2, 1).
    EXPECT(isClose(AffineTransform2::translationBy(Vector2(8, 4)).partly(0.25f).translation, Vector2(2, 1)));
    EXPECT(isClose(sampleMap().partly(0), AffineTransform2::identity()));
    EXPECT(isClose(sampleMap().partly(1), sampleMap()));
}

TEST(affineInMetricMatchesTheGolden) {
    // golden: inMetric(rotation(30°, [10, 20]), [1, 0.5]) = [0.8660254, 1, -0.25, 0.8660254, 11.339746, -4.6410162].
    AffineTransform2 map = sampleMap().inMetric(Vector2(1, 0.5f));
    EXPECT(isClose(map.xAxis, Vector2(0.8660254f, 1), 1e-5f));
    EXPECT(isClose(map.yAxis, Vector2(-0.25f, 0.8660254f), 1e-5f));
    EXPECT(isClose(map.translation, Vector2(11.339746f, -4.6410162f), 1e-4f));
}

TEST(affineInMetricEqualsScalingThereAndBack) {
    // The documented identity: put the point in metric units, map it, put it back.
    Vector2 metric(3, 0.5f);
    AffineTransform2 direct = sampleMap().inMetric(metric);
    AffineTransform2 composed =
        AffineTransform2::scaling(metric).then(sampleMap()).then(AffineTransform2::scaling(Vector2(1 / metric.x, 1 / metric.y)));
    EXPECT(isClose(direct, composed, 1e-4f));
}

// MARK: Plane helpers

TEST(angleHalfTurnFoldsADirectionIntoMinusNinetyExclusiveToNinetyInclusive) {
    // golden: halfTurn(2) = 2 - pi = -1.1415927; 190° -> 10°; -100° -> 80°.
    EXPECT(isApproximatelyEqual(Angle::fromRadians(2).halfTurn().radians, -1.1415927f, 1e-5f));
    EXPECT(isApproximatelyEqual(Angle::degrees(190).halfTurn().degrees(), 10, 1e-3f));
    EXPECT(isApproximatelyEqual(Angle::degrees(-100).halfTurn().degrees(), 80, 1e-3f));
    EXPECT(isApproximatelyEqual(Angle::degrees(90).halfTurn().degrees(), 90, 1e-3f));
    EXPECT(isApproximatelyEqual(Angle::degrees(-90).halfTurn().degrees(), 90, 1e-3f)); // -90° is excluded
    EXPECT(isApproximatelyEqual(Angle().halfTurn().radians, 0));
}

TEST(angleHalfTurnLandsInRangeAndKeepsTheLine) {
    // Property: the result is in (-90°, 90°] and differs from the input by a whole number of half turns.
    for (float degrees = -700; degrees <= 700; degrees += 13.7f) {
        float folded = Angle::degrees(degrees).halfTurn().degrees();
        EXPECT(folded > -90.01f && folded <= 90.01f);
        float halfTurns = (degrees - folded) / 180;
        EXPECT(isApproximatelyEqual(halfTurns, std::round(halfTurns), 1e-3f));
    }
    EXPECT(std::isnan(Angle::fromRadians(std::nanf("")).halfTurn().radians)); // a NaN stays NaN
}

TEST(vectorFromAngleIsTheUnitDirection) {
    EXPECT(isClose(Vector2::fromAngle(Angle::degrees(60)), Vector2(0.5f, 0.8660254f)));
    EXPECT(isClose(Vector2::fromAngle(Angle()), Vector2(1, 0)));
    EXPECT(isApproximatelyEqual(Vector2::fromAngle(Angle::degrees(123)).length(), 1));
    EXPECT(isClose(Angle::degrees(90).direction(), Vector2(0, 1)));
}

TEST(vectorRotatedTurnsAboutAPivot) {
    // (2, 1) about (1, 1) by 90° counterclockwise: offset (1, 0) -> (0, 1), so the point is (1, 2).
    EXPECT(isClose(Vector2(2, 1).rotated(Angle::degrees(90), Vector2(1, 1)), Vector2(1, 2)));
    EXPECT(isClose(Vector2(1, 0).rotated(Angle::degrees(90)), Vector2(0, 1)));
    // Properties: lengths about the pivot are kept, and a turn and its opposite cancel.
    Vector2 pivot(3, -2);
    Vector2 point(7, 5);
    Vector2 turned = point.rotated(Angle::degrees(37), pivot);
    EXPECT(isApproximatelyEqual(turned.distance(pivot), point.distance(pivot), 1e-4f));
    EXPECT(isClose(turned.rotated(Angle::degrees(-37), pivot), point, 1e-4f));
}

TEST(vectorMeanOfPoints) {
    // golden: mean([50, 50], [-20, 30], [130, 120], [0, 0]) = (40, 50).
    EXPECT(isClose(Vector2::mean({Vector2(50, 50), Vector2(-20, 30), Vector2(130, 120), Vector2(0, 0)}), Vector2(40, 50)));
    EXPECT(isClose(Vector2::mean({Vector2(3, 4)}), Vector2(3, 4)));
    EXPECT_PROBLEM(Vector2::mean({}), "mean of no points");
}

TEST(vectorMetricUnitsRoundTrip) {
    Vector2 metric(2, 0.5f);
    EXPECT(isClose(Vector2(3, 4).toMetric(metric), Vector2(6, 2)));
    EXPECT(isClose(Vector2(3, 4).toMetric(metric).fromMetric(metric), Vector2(3, 4)));
    EXPECT(isClose(Vector2(6, 2).fromMetric(metric), Vector2(3, 4)));
}

TEST(smoothStepIsLevelAtBothEnds) {
    EXPECT(isApproximatelyEqual(smoothStep(-1), 0));
    EXPECT(isApproximatelyEqual(smoothStep(0), 0));
    EXPECT(isApproximatelyEqual(smoothStep(0.25f), 0.15625f)); // 0.0625 * (3 - 0.5)
    EXPECT(isApproximatelyEqual(smoothStep(0.5f), 0.5f));
    EXPECT(isApproximatelyEqual(smoothStep(1), 1));
    EXPECT(isApproximatelyEqual(smoothStep(2), 1));
    // Property: smoothStep(1 - u) = 1 - smoothStep(u), and it never decreases.
    float previous = 0;
    for (float amount = 0; amount <= 1; amount += 0.05f) {
        EXPECT(isApproximatelyEqual(smoothStep(1 - amount), 1 - smoothStep(amount), 1e-5f));
        EXPECT(smoothStep(amount) >= previous - 1e-6f);
        previous = smoothStep(amount);
    }
    EXPECT(std::isnan(smoothStep(std::nanf(""))));
}
