// Tests of Curves/CurveCircleCrossings. Golden values are transcribed from graphics_cpp Curves2D/tests/golden/
// intersect.json (circleCrossings, curveCrossings), compared at 1e-4.
#include <cmath>

#include "../Curves/CurveCircleCrossings.h"
#include "../Curves/Path2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Curve2 = CubicBezier<Vector2>;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

} // namespace

TEST(circleCrossingsOfAStraightLineThroughACircle) {
    // A straight cubic along y = 0 from x = 0 to 30 (evenly paced) meets the circle of radius 5 round (10, 0) at x = 5 and 15.
    Curve2 line = Path2D::evenlyPacedCubic(Vector2(0, 0), Vector2(30, 0));
    std::vector<CircleCrossing> crossings = circleCrossings(line, Vector2(10, 0), 5);
    EXPECT(crossings.size() == 2);
    EXPECT(isApproximatelyEqual(crossings[0].parameter, 5.0f / 30, 1e-5f) && isClose(crossings[0].point, Vector2(5, 0)));
    EXPECT(isApproximatelyEqual(crossings[1].parameter, 15.0f / 30, 1e-5f) && isClose(crossings[1].point, Vector2(15, 0)));
}

TEST(everyCrossingLiesOnTheCircleAndOnTheCurve) {
    Curve2 curve(Vector2(0, 0), Vector2(50, 100), Vector2(50, -100), Vector2(100, 0));
    for (Vector2 centre : {Vector2(50, 50), Vector2(50, 0), Vector2(30, -10)}) {
        for (const CircleCrossing& crossing : circleCrossings(curve, centre, 30)) {
            EXPECT(isApproximatelyEqual(crossing.point.distance(centre), 30.0f, 1e-3f));
            EXPECT(isClose(curve.point(crossing.parameter), crossing.point, 1e-4f));
        }
    }
}

TEST(crossingsComeInOrderOfParameter) {
    Curve2 curve(Vector2(0, 0), Vector2(50, 100), Vector2(50, -100), Vector2(100, 0));
    std::vector<CircleCrossing> crossings = circleCrossings(curve, Vector2(50, 0), 30, 16);
    EXPECT(crossings.size() == 2);
    EXPECT(crossings[0].parameter < crossings[1].parameter);
}

TEST(circleCrossingsGoldenArch) {
    // boolean.json-style arch: (0,0) (0,100) (100,100) (100,0) against the circle (50, 50) radius 40.
    Curve2 arch(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0));
    std::vector<CircleCrossing> crossings = circleCrossings(arch, Vector2(50, 50), 40);
    EXPECT(crossings.size() == 2);
    EXPECT(isApproximatelyEqual(crossings[0].parameter, 0.19669291f, 1e-4f));
    EXPECT(isClose(crossings[0].point, Vector2(10.084495f, 47.401443f)));
    EXPECT(isApproximatelyEqual(crossings[1].parameter, 0.80330709f, 1e-4f));
    EXPECT(isClose(crossings[1].point, Vector2(89.915505f, 47.401443f)));
}

TEST(circleCrossingsGoldenSCurve) {
    Curve2 curve(Vector2(0, 0), Vector2(50, 100), Vector2(50, -100), Vector2(100, 0));
    std::vector<CircleCrossing> first = circleCrossings(curve, Vector2(50, 50), 40);
    EXPECT(first.size() == 2);
    EXPECT(isApproximatelyEqual(first[0].parameter, 0.13809206f, 1e-4f) && isClose(first[0].point, Vector2(18.11673f, 25.845144f)));
    EXPECT(isApproximatelyEqual(first[1].parameter, 0.42956926f, 1e-4f) && isClose(first[1].point, Vector2(44.682757f, 10.354989f)));
    std::vector<CircleCrossing> second = circleCrossings(curve, Vector2(50, 0), 30, 16);
    EXPECT(second.size() == 2);
    EXPECT(isApproximatelyEqual(second[0].parameter, 0.29770455f, 1e-4f) && isClose(second[0].point, Vector2(33.999978f, 25.377141f)));
    EXPECT(isApproximatelyEqual(second[1].parameter, 0.70229545f, 1e-4f) && isClose(second[1].point, Vector2(66.00002f, -25.377141f)));
}

TEST(circleCrossingsGoldenDiagonalAndFreeCurve) {
    Curve2 diagonal(Vector2(0, 0), Vector2(10, 10), Vector2(20, 20), Vector2(30, 30));
    std::vector<CircleCrossing> hit = circleCrossings(diagonal, Vector2(50, 50), 40);
    EXPECT(hit.size() == 1);
    EXPECT(isApproximatelyEqual(hit[0].parameter, 0.72385763f, 1e-4f) && isClose(hit[0].point, Vector2(21.715729f, 21.715729f)));
    Curve2 free(Vector2(12.5f, -3), Vector2(40, 80), Vector2(95, 60), Vector2(130, 10));
    std::vector<CircleCrossing> both = circleCrossings(free, Vector2(50, 50), 40);
    EXPECT(both.size() == 2);
    EXPECT(isApproximatelyEqual(both[0].parameter, 0.11070381f, 1e-4f) && isClose(both[0].point, Vector2(22.579686f, 20.877391f)));
    EXPECT(isApproximatelyEqual(both[1].parameter, 0.66528829f, 1e-4f) && isClose(both[1].point, Vector2(89.91453f, 47.3865f)));
}

TEST(aCircleThatMissesTheCurveOrSwallowsItHasNoCrossings) {
    Curve2 arch(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0));
    EXPECT(circleCrossings(arch, Vector2(50, 50), 1000).empty());
    EXPECT(circleCrossings(arch, Vector2(50, 0), 30, 16).empty());
}

TEST(aCurveEndOnTheCircleCountsOnce) {
    Curve2 line = Path2D::evenlyPacedCubic(Vector2(0, 0), Vector2(10, 0));
    std::vector<CircleCrossing> crossings = circleCrossings(line, Vector2(0, 0), 5);
    EXPECT(crossings.size() == 1 && isApproximatelyEqual(crossings[0].parameter, 0.5f, 1e-5f));
    // The start lies on a circle round it of radius 0 only; a circle through the start is found at t = 0.
    std::vector<CircleCrossing> atStart = circleCrossings(line, Vector2(0, 5), 5);
    EXPECT(!atStart.empty() && atStart.front().parameter == 0);
}

TEST(circleCrossingsNeedAPositiveRadius) {
    Curve2 line = Path2D::evenlyPacedCubic(Vector2(0, 0), Vector2(10, 0));
    EXPECT_PROBLEM(circleCrossings(line, Vector2(0, 0), 0), "radius");
    EXPECT_PROBLEM(circleCrossings(line, Vector2(0, 0), -1), "radius");
}

TEST(curveCrossingsGolden) {
    Curve2 arch(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0));
    Curve2 across(Vector2(0, 50), Vector2(40, 50), Vector2(60, 50), Vector2(100, 50));
    std::optional<std::vector<CurveCrossing>> found = curveCrossings(arch, across);
    EXPECT(found.has_value() && found->size() == 2);
    if (found && found->size() == 2) {
        EXPECT(isApproximatelyEqual((*found)[0].firstParameter, 0.2113247f, 1e-4f));
        EXPECT(isApproximatelyEqual((*found)[0].secondParameter, 0.1006374f, 1e-4f));
        EXPECT(isApproximatelyEqual((*found)[1].firstParameter, 0.7886753f, 1e-4f));
        EXPECT(isApproximatelyEqual((*found)[1].secondParameter, 0.8993587f, 1e-4f));
    }
    // The same curve against itself runs along itself: nothing.
    EXPECT(!curveCrossings(arch, arch).has_value());
}

TEST(curveCrossingsAreSymmetric) {
    Curve2 arch(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0));
    Curve2 s(Vector2(0, 0), Vector2(50, 100), Vector2(50, -100), Vector2(100, 0));
    std::optional<std::vector<CurveCrossing>> forward = curveCrossings(arch, s);
    std::optional<std::vector<CurveCrossing>> backward = curveCrossings(s, arch);
    EXPECT(forward.has_value() && backward.has_value());
    if (forward && backward) EXPECT(forward->size() == backward->size());
}
