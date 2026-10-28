// Tests of the plane geometry added with the 2D curves port: Segment2::closestFoot / isNear / stitchedLoop,
// Line2::meets, Polyline2, Bounds2::overlaps / centre. Golden cases are from graphics_cpp
// Curves2D/tests/golden/geometry.json.
#include <vector>

#include "../Geometry/Bounds2.h"
#include "../Geometry/Line2.h"
#include "../Geometry/Polyline2.h"
#include "../Geometry/Segment2.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

Polyline2 squareCorner() { return Polyline2{{Vector2(0, 0), Vector2(10, 0), Vector2(10, 10), Vector2(0, 10)}}; }

} // namespace

// MARK: Segment2

TEST(segmentClosestFootGivesThePointAndHowFarAlong) {
    Segment2 segment{Vector2(0, 0), Vector2(10, 0)};
    SegmentFoot middle = segment.closestFoot(Vector2(5, 3));
    EXPECT(isClose(middle.point, Vector2(5, 0)));
    EXPECT(isApproximatelyEqual(middle.parameter, 0.5f));
    SegmentFoot past = segment.closestFoot(Vector2(12, 1));
    EXPECT(isClose(past.point, Vector2(10, 0)));
    EXPECT(isApproximatelyEqual(past.parameter, 1));
    EXPECT(isApproximatelyEqual(segment.closestFoot(Vector2(-4, 2)).parameter, 0));
    // A single point has parameter 0, and the point agrees with closestPoint everywhere.
    EXPECT(isApproximatelyEqual(Segment2{Vector2(1, 1), Vector2(1, 1)}.closestFoot(Vector2(5, 5)).parameter, 0));
    for (Vector2 probe : {Vector2(2, 7), Vector2(-3, -3), Vector2(9, -1)}) {
        EXPECT(isClose(segment.closestFoot(probe).point, segment.closestPoint(probe)));
    }
}

TEST(segmentIsNearWithinATolerance) {
    // golden: nearSegment([5, 1], [0, 0], [10, 0], 2) and ([12, 1], ...) are true.
    Segment2 segment{Vector2(0, 0), Vector2(10, 0)};
    EXPECT(segment.isNear(Vector2(5, 1), 2));
    EXPECT(segment.isNear(Vector2(12, 1), 2));  // 2 beyond the end, as far as the tolerance allows
    EXPECT(!segment.isNear(Vector2(13, 1), 2)); // 3 beyond
    EXPECT(!segment.isNear(Vector2(5, 3), 2));  // 3 off the line
    EXPECT(segment.isNear(Vector2(13, 1), 2, 3.5f)); // a wider overshoot
    EXPECT(!segment.isNear(Vector2(12, 1), 2, 0.5f));
    Segment2 point{Vector2(1, 1), Vector2(1, 1)};
    EXPECT(!point.isNear(Vector2(1, 1), 5)); // no line, nothing is near
}

TEST(segmentStitchedLoopStartsEachWhereTheLastEnded) {
    // golden: stitch moves each start onto the end before it, round the loop.
    std::vector<Segment2> loop = {{Vector2(0, 0), Vector2(10, 0)},
                                  {Vector2(10.000001f, 0), Vector2(10, 10)},
                                  {Vector2(10, 10), Vector2(0, 2e-6f)}};
    std::vector<Segment2> stitched = Segment2::stitchedLoop(loop);
    EXPECT(isClose(stitched[0].start, Vector2(0, 2e-6f), 1e-7f));
    EXPECT(isClose(stitched[1].start, Vector2(10, 0), 1e-7f));
    EXPECT(isClose(stitched[2].start, Vector2(10, 10), 1e-7f));
    EXPECT(isClose(stitched[1].end, loop[1].end, 1e-7f)); // the ends stay
    for (size_t index = 0; index < stitched.size(); index += 1) { // property: no gap remains
        EXPECT(stitched[index].end == stitched[(index + 1) % stitched.size()].start);
    }
    EXPECT(Segment2::stitchedLoop({}).empty());
}

// MARK: Line2

TEST(lineMeetsGivesTheDistanceAlongEach) {
    // golden: meet((0,0)+(1,0), (5,5)+(0,-1)) = (5, 5).
    Line2 horizontal{Vector2(0, 0), Vector2(1, 0)};
    Line2 vertical{Vector2(5, 5), Vector2(0, -1)};
    std::optional<LineMeeting> meeting = horizontal.meets(vertical);
    EXPECT(meeting.has_value());
    EXPECT(isApproximatelyEqual(meeting->alongFirst, 5));
    EXPECT(isApproximatelyEqual(meeting->alongSecond, 5));
}

TEST(lineMeetsParallelLinesIsNothing) {
    // golden: meet((0,0)+(1,0), (5,5)+(1,0)) = null.
    Line2 axis{Vector2(0, 0), Vector2(1, 0)};
    Line2 parallel{Vector2(5, 5), Vector2(1, 0)};
    Line2 nearlyParallel{Vector2(5, 5), Vector2(1, 1e-3f)};
    EXPECT(!axis.meets(parallel).has_value());
    EXPECT(axis.meets(nearlyParallel).has_value());
    // A wide `flat` makes nearly parallel lines count as parallel.
    EXPECT(!axis.meets(nearlyParallel, 0.01f).has_value());
}

TEST(lineMeetsInsideKeepsOnlyCrossingsStrictlyInsideBothSegments) {
    Line2 across{Vector2(0, 0), Vector2(10, 0)};
    Line2 tall{Vector2(5, -5), Vector2(0, 10)};
    EXPECT(across.meets(tall, 1e-6f, 0.1f).has_value());             // crossing at 0.5 and 0.5
    Line2 short_{Vector2(5, -5), Vector2(0, 4)};                     // the line crosses at 1.25: past its end
    EXPECT(!across.meets(short_, 1e-6f, 0.0f).has_value());
    EXPECT(short_.meets(across).has_value());                        // without `inside` it is still a crossing
    Line2 touching{Vector2(10, -5), Vector2(0, 10)};                 // meets across's end exactly
    EXPECT(!across.meets(touching, 1e-6f, 0.0f).has_value());        // strictly inside rejects the end point
}

TEST(lineMeetsIsSymmetricAndReproducesThePoint) {
    Line2 first{Vector2(1, 2), Vector2(3, 1)};
    Line2 second{Vector2(8, -1), Vector2(-1, 4)};
    std::optional<LineMeeting> forward = first.meets(second);
    std::optional<LineMeeting> backward = second.meets(first);
    EXPECT(forward.has_value() && backward.has_value());
    EXPECT(isApproximatelyEqual(forward->alongFirst, backward->alongSecond, 1e-5f));
    EXPECT(isApproximatelyEqual(forward->alongSecond, backward->alongFirst, 1e-5f));
    Vector2 onFirst = first.origin + first.direction * forward->alongFirst;
    Vector2 onSecond = second.origin + second.direction * forward->alongSecond;
    EXPECT(isClose(onFirst, onSecond, 1e-4f));
}

// MARK: Polyline2

TEST(polylineRunLengthsAccumulateTheDistance) {
    // golden: runLengths of the square's three sides = [0, 10, 20, 30].
    std::vector<float> runs = squareCorner().runLengths();
    EXPECT_EQUAL(runs.size(), size_t(4));
    EXPECT(isApproximatelyEqual(runs[0], 0));
    EXPECT(isApproximatelyEqual(runs[1], 10));
    EXPECT(isApproximatelyEqual(runs[2], 20));
    EXPECT(isApproximatelyEqual(runs[3], 30));
    EXPECT(Polyline2{}.runLengths().empty());
}

TEST(polylinePointAlongWalksTheChain) {
    // golden: pointAlong clamps -1 and 0 to the start; the rest follows the sides.
    Polyline2 chain = squareCorner();
    std::vector<float> runs = chain.runLengths();
    EXPECT(isClose(chain.pointAlong(runs, -1), Vector2(0, 0)));
    EXPECT(isClose(chain.pointAlong(runs, 0), Vector2(0, 0)));
    EXPECT(isClose(chain.pointAlong(runs, 15), Vector2(10, 5)));
    EXPECT(isClose(chain.pointAlong(runs, 30), Vector2(0, 10)));
    EXPECT(isClose(chain.pointAlong(runs, 99), Vector2(0, 10)));
    // Property: at each running length the point is the chain's own point.
    for (size_t index = 0; index < chain.points.size(); index += 1) {
        EXPECT(isClose(chain.pointAlong(runs, runs[index]), chain.points[index]));
    }
}

TEST(polylinePointAlongSurvivesRepeatedPointsAndBadInput) {
    Polyline2 repeated{{Vector2(0, 0), Vector2(0, 0), Vector2(4, 0)}};
    EXPECT(isClose(repeated.pointAlong(repeated.runLengths(), 2), Vector2(2, 0)));
    EXPECT(isClose(Polyline2{{Vector2(7, 7)}}.pointAlong({0}, 3), Vector2(7, 7)));
    EXPECT_PROBLEM(Polyline2{}.pointAlong({}, 1), "no points");
    EXPECT_PROBLEM(squareCorner().pointAlong({0, 1}, 1), "one number per point");
}

TEST(polylineLengthIsTheSumOfItsPieces) {
    // 3-4-5 then 6: 5 + 6 = 11. A control polygon is at least as long as the chord between its ends.
    Polyline2 chain{{Vector2(0, 0), Vector2(3, 4), Vector2(3, 10)}};
    EXPECT(isApproximatelyEqual(chain.length(), 11, 1e-5f));
    EXPECT(isApproximatelyEqual(squareCorner().length(), squareCorner().runLengths().back()));
    EXPECT(chain.length() >= chain.points.front().distance(chain.points.back()));
    EXPECT(isApproximatelyEqual(Polyline2{{Vector2(1, 1)}}.length(), 0));
}

// MARK: Bounds2

TEST(boundsOverlapsWithAMargin) {
    Bounds2 left = Bounds2::checked(Vector2(0, 0), Vector2(2, 2));
    Bounds2 right = Bounds2::checked(Vector2(3, 0), Vector2(5, 2));
    EXPECT(!left.overlaps(right));
    EXPECT(left.overlaps(right, 1)); // a gap of 1 is within a margin of 1
    EXPECT(!left.overlaps(right, 0.5f));
    EXPECT(left.overlaps(Bounds2::checked(Vector2(2, 2), Vector2(4, 4)))); // touching at a corner counts
    EXPECT(left.overlaps(Bounds2::checked(Vector2(1, 1), Vector2(1.5f, 1.5f)))); // one inside the other
    // Property: it does not matter which is asked.
    EXPECT(left.overlaps(right, 1) == right.overlaps(left, 1));
    EXPECT(left.overlaps(right) == right.overlaps(left));
}

TEST(boundsCentreIsTheMiddle) {
    EXPECT(isClose(Bounds2::checked(Vector2(0, 0), Vector2(2, 2)).centre(), Vector2(1, 1)));
    EXPECT(isClose(Bounds2::checked(Vector2(-3, 1), Vector2(5, 2)).centre(), Vector2(1, 1.5f)));
    EXPECT(isClose(Bounds2::enclosing({Vector2(4, 4)}).centre(), Vector2(4, 4)));
}
