// Tests of the circle constructions (Geometry/CircleConstruction.h) and of Ellipse2: hand-worked
// crossings and tangents, then the properties every answer must have (a crossing lies on both
// figures; a tangent touch point is a radius away and at right angles to its radius).
#include <cmath>
#include <vector>

#include "../Geometry/CircleConstruction.h"
#include "../Geometry/Ellipse2.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// How far `point` is from the infinite line.
float distanceFromLine(Vector2 point, const Line2& line) {
    return std::fabs((point - line.origin).cross(line.direction)) / line.direction.length();
}

} // namespace

// MARK: Circle and line

TEST(circleLineCrossingsOfTheUnitCircleAndAHorizontalLine) {
    // y = 0.6 meets x² + y² = 1 at x = ±0.8 (the 3-4-5 triangle), ordered along +x.
    Circle2 circle{Vector2(0, 0), 1};
    std::vector<Vector2> crossings = circleLineCrossings(circle, Line2{Vector2(-5, 0.6f), Vector2(2, 0)});
    EXPECT(crossings.size() == 2);
    EXPECT(isClose(crossings[0], Vector2(-0.8f, 0.6f)));
    EXPECT(isClose(crossings[1], Vector2(0.8f, 0.6f)));
    // Running the other way reverses the order.
    std::vector<Vector2> reversed = circleLineCrossings(circle, Line2{Vector2(5, 0.6f), Vector2(-1, 0)});
    EXPECT(reversed.size() == 2 && isClose(reversed[0], Vector2(0.8f, 0.6f)));
}

TEST(circleLineCrossingsTouchMissAndRefuse) {
    Circle2 circle{Vector2(2, 3), 5};
    // x = 7 touches the circle of radius 5 round (2, 3) at (7, 3).
    std::vector<Vector2> touch = circleLineCrossings(circle, Line2{Vector2(7, -10), Vector2(0, 1)});
    EXPECT(touch.size() == 1 && isClose(touch[0], Vector2(7, 3)));
    EXPECT(circleLineCrossings(circle, Line2{Vector2(8, 0), Vector2(0, 1)}).empty());
    EXPECT(circleLineCrossings(circle, Line2{Vector2(0, 0), Vector2(0, 0)}).empty());
    EXPECT(circleLineCrossings(Circle2{Vector2(0, 0), -1}, Line2{Vector2(0, 0), Vector2(1, 0)}).empty());
}

TEST(circleLineCrossingsLieOnBothFigures) {
    SeededRandom random(17);
    for (int trial = 0; trial < 200; trial += 1) {
        Circle2 circle{Vector2(random.nextFloat(-5, 5), random.nextFloat(-5, 5)), random.nextFloat(0.5f, 4)};
        Line2 line{Vector2(random.nextFloat(-5, 5), random.nextFloat(-5, 5)),
                   Vector2(random.nextFloat(-1, 1), random.nextFloat(-1, 1))};
        if (line.direction.length() < 0.1f) continue;
        for (Vector2 crossing : circleLineCrossings(circle, line)) {
            EXPECT(isApproximatelyEqual(crossing.distance(circle.centre), circle.radius, 1e-4f));
            EXPECT(distanceFromLine(crossing, line) < 1e-4f);
        }
    }
}

// MARK: Two circles

TEST(circleCircleCrossingsOfTwoHandWorkedCircles) {
    // Radius 5 round the origin and radius 5 round (8, 0): the radical line is x = 4, and
    // h = √(25 − 16) = 3, so (4, 3) on the left of the line from c₁ to c₂ and (4, −3) on the right.
    std::vector<Vector2> crossings = circleCircleCrossings(Circle2{Vector2(0, 0), 5}, Circle2{Vector2(8, 0), 5});
    EXPECT(crossings.size() == 2);
    EXPECT(isClose(crossings[0], Vector2(4, 3)));
    EXPECT(isClose(crossings[1], Vector2(4, -3)));
    // Unequal: r₁ = 4, r₂ = 3, d = 5 gives a = (25 + 16 − 9)/10 = 3.2, h = √(16 − 10.24) = 2.4.
    std::vector<Vector2> unequal = circleCircleCrossings(Circle2{Vector2(0, 0), 4}, Circle2{Vector2(5, 0), 3});
    EXPECT(unequal.size() == 2 && isClose(unequal[0], Vector2(3.2f, 2.4f)) && isClose(unequal[1], Vector2(3.2f, -2.4f)));
}

TEST(circleCircleCrossingsTouchApartNestedAndSame) {
    // Touching from outside at (2, 0), and from inside at (3, 0).
    std::vector<Vector2> outside = circleCircleCrossings(Circle2{Vector2(0, 0), 2}, Circle2{Vector2(5, 0), 3});
    EXPECT(outside.size() == 1 && isClose(outside[0], Vector2(2, 0)));
    std::vector<Vector2> inside = circleCircleCrossings(Circle2{Vector2(0, 0), 3}, Circle2{Vector2(1, 0), 2});
    EXPECT(inside.size() == 1 && isClose(inside[0], Vector2(3, 0)));
    EXPECT(circleCircleCrossings(Circle2{Vector2(0, 0), 1}, Circle2{Vector2(5, 0), 1}).empty());
    EXPECT(circleCircleCrossings(Circle2{Vector2(0, 0), 5}, Circle2{Vector2(1, 0), 1}).empty());
    EXPECT(circleCircleCrossings(Circle2{Vector2(1, 1), 2}, Circle2{Vector2(1, 1), 2}).empty());
}

TEST(circleCircleCrossingsLieOnBothCircles) {
    SeededRandom random(23);
    for (int trial = 0; trial < 200; trial += 1) {
        Circle2 first{Vector2(random.nextFloat(-3, 3), random.nextFloat(-3, 3)), random.nextFloat(0.5f, 3)};
        Circle2 second{Vector2(random.nextFloat(-3, 3), random.nextFloat(-3, 3)), random.nextFloat(0.5f, 3)};
        for (Vector2 crossing : circleCircleCrossings(first, second)) {
            EXPECT(isApproximatelyEqual(crossing.distance(first.centre), first.radius, 1e-4f));
            EXPECT(isApproximatelyEqual(crossing.distance(second.centre), second.radius, 1e-4f));
        }
    }
}

// MARK: Tangents from a point

TEST(tangentPointsFromAPointOutsideTheUnitCircle) {
    // From (2, 0): cos α = r/d = ½, so α = 60° and the touches are (½, ±√3/2), the
    // counterclockwise one first.
    std::vector<Vector2> touches = tangentPointsFrom(Circle2{Vector2(0, 0), 1}, Vector2(2, 0));
    EXPECT(touches.size() == 2);
    EXPECT(isClose(touches[0], Vector2(0.5f, std::sqrt(3.0f) / 2)));
    EXPECT(isClose(touches[1], Vector2(0.5f, -std::sqrt(3.0f) / 2)));
}

TEST(tangentPointsFromInsideOnAndOutside) {
    Circle2 circle{Vector2(1, 2), 3};
    EXPECT(tangentPointsFrom(circle, Vector2(2, 2)).empty());
    std::vector<Vector2> on = tangentPointsFrom(circle, Vector2(4, 2));
    EXPECT(on.size() == 1 && isClose(on[0], Vector2(4, 2)));
    EXPECT(tangentPointsFrom(circle, Vector2(9, 9)).size() == 2);
}

TEST(tangentFromAPointTouchesAtARightAngleToTheRadius) {
    SeededRandom random(31);
    for (int trial = 0; trial < 300; trial += 1) {
        Circle2 circle{Vector2(random.nextFloat(-4, 4), random.nextFloat(-4, 4)), random.nextFloat(0.5f, 3)};
        Vector2 point(random.nextFloat(-8, 8), random.nextFloat(-8, 8));
        for (Vector2 touch : tangentPointsFrom(circle, point)) {
            // On the circle, and the radius is perpendicular to the tangent line.
            EXPECT(isApproximatelyEqual(touch.distance(circle.centre), circle.radius, 1e-4f));
            Vector2 radius = (touch - circle.centre).normalized();
            Vector2 alongTangent = point - touch;
            if (alongTangent.length() > 1e-3f) EXPECT(std::fabs(radius.dot(alongTangent.normalized())) < 1e-4f);
            // The tangent line meets the circle once: the line through the touch point in the
            // tangent direction is at distance r from the centre.
            Line2 tangentLine{touch, radius.perpendicular()};
            EXPECT(isApproximatelyEqual(distanceFromLine(circle.centre, tangentLine), circle.radius, 1e-4f));
            EXPECT(circleLineCrossings(circle, tangentLine).size() == 1);
        }
    }
}

// MARK: Common tangents

TEST(commonTangentsOfTwoEqualCirclesApart) {
    // Radius 1 round (0, 0) and (4, 0): outer tangents y = ±1, inner ones through (2, 0) with
    // n·û = −2/4, so θ = 120° and n = (−½, ±√3/2): touches c₁ − n = (½, ∓√3/2).
    std::vector<CircleTangent> tangents = commonTangents(Circle2{Vector2(0, 0), 1}, Circle2{Vector2(4, 0), 1});
    EXPECT(tangents.size() == 4);
    EXPECT(!tangents[0].crossesBetween && !tangents[1].crossesBetween);
    EXPECT(tangents[2].crossesBetween && tangents[3].crossesBetween);
    EXPECT(isClose(tangents[0].onFirst, Vector2(0, -1)) && isClose(tangents[0].onSecond, Vector2(4, -1)));
    EXPECT(isClose(tangents[1].onFirst, Vector2(0, 1)) && isClose(tangents[1].onSecond, Vector2(4, 1)));
    float half = std::sqrt(3.0f) / 2;
    EXPECT(isClose(tangents[2].onFirst, Vector2(0.5f, -half)) && isClose(tangents[2].onSecond, Vector2(3.5f, half)));
    EXPECT(isClose(tangents[3].onFirst, Vector2(0.5f, half)) && isClose(tangents[3].onSecond, Vector2(3.5f, -half)));
}

TEST(commonTangentsCountFourThreeTwoOneNone) {
    Circle2 first{Vector2(0, 0), 2};
    EXPECT(commonTangents(first, Circle2{Vector2(6, 0), 1}).size() == 4);
    std::vector<CircleTangent> touching = commonTangents(first, Circle2{Vector2(3, 0), 1});
    EXPECT(touching.size() == 3);
    EXPECT(touching[2].crossesBetween && isClose(touching[2].onFirst, Vector2(2, 0)) && isClose(touching[2].onSecond, Vector2(2, 0)));
    EXPECT(commonTangents(first, Circle2{Vector2(2, 0), 1}).size() == 2);
    std::vector<CircleTangent> inside = commonTangents(first, Circle2{Vector2(1, 0), 1});
    EXPECT(inside.size() == 1 && isClose(inside[0].onFirst, Vector2(2, 0)) && isClose(inside[0].onSecond, Vector2(2, 0)));
    EXPECT(commonTangents(first, Circle2{Vector2(0.5f, 0), 1}).empty());
    EXPECT(commonTangents(first, first).empty());
}

TEST(commonTangentsTouchBothCirclesAtRightAngles) {
    SeededRandom random(41);
    for (int trial = 0; trial < 200; trial += 1) {
        Circle2 first{Vector2(random.nextFloat(-4, 4), random.nextFloat(-4, 4)), random.nextFloat(0.5f, 2.5f)};
        Circle2 second{Vector2(random.nextFloat(-4, 4), random.nextFloat(-4, 4)), random.nextFloat(0.5f, 2.5f)};
        for (const CircleTangent& tangent : commonTangents(first, second)) {
            EXPECT(isApproximatelyEqual(tangent.onFirst.distance(first.centre), first.radius, 1e-4f));
            EXPECT(isApproximatelyEqual(tangent.onSecond.distance(second.centre), second.radius, 1e-4f));
            Vector2 along = tangent.onSecond - tangent.onFirst;
            if (along.length() < 1e-3f) continue;
            Vector2 direction = along.normalized();
            EXPECT(std::fabs((tangent.onFirst - first.centre).normalized().dot(direction)) < 1e-3f);
            EXPECT(std::fabs((tangent.onSecond - second.centre).normalized().dot(direction)) < 1e-3f);
            // Outer: both centres on the same side of the line; inner: opposite sides.
            float firstSide = direction.cross(first.centre - tangent.onFirst);
            float secondSide = direction.cross(second.centre - tangent.onFirst);
            EXPECT((firstSide * secondSide < 0) == tangent.crossesBetween);
        }
    }
}

TEST(circleThroughThreePointsIsTheConstructionsCircle) {
    // The construction header points at Circle2::through: (0, 0), (4, 0), (0, 3) lie on the
    // circle with diameter (4, 0)–(0, 3), centre (2, 1.5), radius 2.5.
    std::optional<Circle2> circle = Circle2::through(Vector2(0, 0), Vector2(4, 0), Vector2(0, 3));
    EXPECT(circle && isClose(circle->centre, Vector2(2, 1.5f)) && isApproximatelyEqual(circle->radius, 2.5f));
}

// MARK: Ellipse2

TEST(ellipsePointRunsCounterclockwiseFromTheMajorAxis) {
    Ellipse2 ellipse{Vector2(1, 2), 3, 1, Angle::degrees(90)};
    EXPECT(isClose(ellipse.point(Angle::degrees(0)), Vector2(1, 5)));
    EXPECT(isClose(ellipse.point(Angle::degrees(90)), Vector2(0, 2)));
    EXPECT(isClose(ellipse.point(Angle::degrees(180)), Vector2(1, -1)));
}

TEST(ellipseNearestPointHandWorked) {
    Ellipse2 ellipse{Vector2(0, 0), 2, 1, Angle::degrees(0)};
    // On the axes outside: the axis ends.
    EXPECT(isClose(ellipse.nearestPoint(Vector2(5, 0)), Vector2(2, 0)));
    EXPECT(isClose(ellipse.nearestPoint(Vector2(0, -4)), Vector2(0, -1)));
    EXPECT(isApproximatelyEqual(ellipse.distance(Vector2(5, 0)), 3));
    // Inside on the major axis within the evolute's cusp (x < (a² − b²)/a = 1.5): x = 1 gives
    // share = a·x/(a² − b²) = 2/3, the point (4/3, √(1 − 4/9)) = (4/3, √5/3), at distance √(1/9 + 5/9).
    Vector2 inner = ellipse.nearestPoint(Vector2(1, 0));
    EXPECT(isClose(inner, Vector2(4.0f / 3, std::sqrt(5.0f) / 3)));
    EXPECT(isApproximatelyEqual(ellipse.distance(Vector2(1, 0)), std::sqrt(6.0f) / 3));
    // A circle projects radially.
    Ellipse2 circle{Vector2(1, 1), 2, 2, Angle::degrees(30)};
    EXPECT(isClose(circle.nearestPoint(Vector2(4, 5)), Vector2(1 + 2 * 0.6f, 1 + 2 * 0.8f)));
}

TEST(ellipseNearestPointIsOnTheEllipseAndAlongItsNormal) {
    SeededRandom random(53);
    for (int trial = 0; trial < 300; trial += 1) {
        Ellipse2 ellipse{Vector2(random.nextFloat(-2, 2), random.nextFloat(-2, 2)), random.nextFloat(1, 3), 0,
                         Angle::degrees(random.nextFloat(-90, 90))};
        ellipse.minorRadius = ellipse.majorRadius * random.nextFloat(0.2f, 1);
        Vector2 position(random.nextFloat(-6, 6), random.nextFloat(-6, 6));
        Vector2 nearest = ellipse.nearestPoint(position);
        // On the ellipse: (u/a)² + (v/b)² = 1 in its own frame.
        Vector2 offset = nearest - ellipse.centre;
        float along = offset.dot(Vector2(ellipse.rotation.cosine(), ellipse.rotation.sine()));
        float across = offset.dot(Vector2(-ellipse.rotation.sine(), ellipse.rotation.cosine()));
        float level = (along / ellipse.majorRadius) * (along / ellipse.majorRadius)
            + (across / ellipse.minorRadius) * (across / ellipse.minorRadius);
        EXPECT(isApproximatelyEqual(level, 1, 1e-4f));
        // No sampled point of the ellipse is nearer.
        float best = position.distance(nearest);
        for (int step = 0; step < 720; step += 1) {
            Vector2 sample = ellipse.point(Angle::degrees(static_cast<float>(step) / 2));
            EXPECT(position.distance(sample) >= best - 1e-4f);
        }
    }
}
