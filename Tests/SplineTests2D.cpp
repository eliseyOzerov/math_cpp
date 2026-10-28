// Tests of the flat splines of slice S5: the Catmull-Rom spline with alpha, tension and closing, and
// the periodic (closed) B-spline. Golden values are from graphics_cpp's spline.json.
#include <cmath>
#include <vector>

#include "../Curves/BSpline.h"
#include "../Curves/CatmullRomSpline.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

std::vector<Vector2> hill() { return {Vector2(0, 0), Vector2(100, 50), Vector2(200, 0)}; }

} // namespace

TEST(catmullRomFlatOpenSplineMatchesTheGoldenHandles) {
    // Golden "catmullRom" with alpha 0.5 on the hill: two cubics.
    std::vector<CubicBezier<Vector2>> cubics = CatmullRomSpline::cubicsThrough(hill());
    EXPECT(cubics.size() == 2);
    EXPECT(isClose(cubics[0].startHandle, Vector2(33.3333f, 16.6667f), 0.02f));
    EXPECT(isClose(cubics[0].endHandle, Vector2(66.6667f, 50), 0.02f));
    EXPECT(isClose(cubics[1].startHandle, Vector2(133.3333f, 50), 0.02f));
    EXPECT(isClose(cubics[1].endHandle, Vector2(166.6667f, 16.6667f), 0.02f));
}

TEST(catmullRomFlatAlphaChangesNothingForSymmetricPointsButTensionZeroIsThePolyline) {
    // By the symmetry of the hill the handles are the same for alpha 0, 0.5 and 1 (golden).
    for (float alpha : {0.0f, 0.5f, 1.0f}) {
        CatmullRomOptions options;
        options.alpha = alpha;
        std::vector<CubicBezier<Vector2>> cubics = CatmullRomSpline::cubicsThrough(hill(), options);
        EXPECT(isClose(cubics[0].startHandle, Vector2(33.3333f, 16.6667f), 0.02f));
    }
    CatmullRomOptions flat;
    flat.tension = 0;
    std::vector<CubicBezier<Vector2>> cubics = CatmullRomSpline::cubicsThrough(hill(), flat);
    EXPECT(cubics[0].startHandle == cubics[0].start);
    EXPECT(cubics[0].endHandle == cubics[0].end);
}

TEST(catmullRomFlatAlphaMattersForUnevenPoints) {
    // Uneven gaps: the uniform and the centripetal splines differ at the middle handle.
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(1, 0), Vector2(1, 4), Vector2(10, 4)};
    CatmullRomOptions uniform;
    uniform.alpha = 0;
    CubicBezier<Vector2> centripetal = CatmullRomSpline::cubicsThrough(points)[1];
    CubicBezier<Vector2> flat = CatmullRomSpline::cubicsThrough(points, uniform)[1];
    EXPECT(!isClose(centripetal.startHandle, flat.startHandle, 1e-2f));
    // Hand-worked centripetal handles for these points, as in the 3D spline's test: (13/9, 4/9), (0.2, 3.2).
    EXPECT(isClose(centripetal.startHandle, Vector2(13.0f / 9, 4.0f / 9), 1e-4f));
    EXPECT(isClose(centripetal.endHandle, Vector2(0.2f, 3.2f), 1e-4f));
}

TEST(catmullRomFlatClosedSplineMatchesTheGoldenAndMeetsSmoothlyAtEveryJoint) {
    CatmullRomOptions closed;
    closed.closed = true;
    std::vector<CubicBezier<Vector2>> cubics = CatmullRomSpline::cubicsThrough(hill(), closed);
    EXPECT(cubics.size() == 3);
    EXPECT(isClose(cubics[0].startHandle, Vector2(-2.25126f, 9.53648f), 0.02f));
    EXPECT(isClose(cubics[0].endHandle, Vector2(66.6667f, 50), 0.02f));
    EXPECT(isClose(cubics[2].startHandle, Vector2(196.98899f, -12.75486f), 0.03f));
    EXPECT(isClose(cubics[2].endHandle, Vector2(3.01101f, -12.75486f), 0.03f));
    EXPECT(cubics[2].end == cubics[0].start);
    // Direction continuity at the three joints (including the closing one).
    for (size_t index = 0; index < 3; index += 1) {
        const CubicBezier<Vector2>& before = cubics[index];
        const CubicBezier<Vector2>& after = cubics[(index + 1) % 3];
        Vector2 arriving = (before.end - before.endHandle).normalized();
        Vector2 leaving = (after.startHandle - after.start).normalized();
        EXPECT(isClose(arriving, leaving, 1e-4f));
    }
}

TEST(catmullRomFlatPassesThroughEveryPointAndMergesRepeats) {
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(0, 0), Vector2(3, 4), Vector2(3, 4), Vector2(9, 1)};
    std::vector<CubicBezier<Vector2>> cubics = CatmullRomSpline::cubicsThrough(points);
    EXPECT(cubics.size() == 2);
    EXPECT(cubics[0].start == Vector2(0, 0));
    EXPECT(cubics[0].end == Vector2(3, 4));
    EXPECT(cubics[1].start == Vector2(3, 4));
    EXPECT(cubics[1].end == Vector2(9, 1));
    EXPECT(CatmullRomSpline::cubicsThrough({Vector2(1, 1), Vector2(1, 1)}).empty());
    EXPECT(CatmullRomSpline::cubicsThrough({}).empty());
}

TEST(catmullRomFlatClosedListThatEndsOnItsStartDropsTheRepeat) {
    CatmullRomOptions closed;
    closed.closed = true;
    std::vector<Vector2> ring = {Vector2(0, 0), Vector2(10, 0), Vector2(10, 10), Vector2(0, 10)};
    std::vector<Vector2> ringRepeated = ring;
    ringRepeated.push_back(Vector2(0, 0));
    EXPECT(CatmullRomSpline::cubicsThrough(ring, closed) == CatmullRomSpline::cubicsThrough(ringRepeated, closed));
    EXPECT(CatmullRomSpline::cubicsThrough(ring, closed).size() == 4);
}

TEST(catmullRomPathIsClosedOrOpenAsAsked) {
    std::optional<Path2D> open = CatmullRomSpline::pathThrough(hill());
    EXPECT(open.has_value());
    EXPECT(!open->isClosed());
    EXPECT(open->segments().size() == 2);
    EXPECT(open->start == Vector2(0, 0));
    EXPECT(open->currentPoint() == Vector2(200, 0));
    CatmullRomOptions closed;
    closed.closed = true;
    std::optional<Path2D> loop = CatmullRomSpline::pathThrough(hill(), closed);
    EXPECT(loop.has_value());
    EXPECT(loop->isClosed());
    EXPECT(loop->segments().size() == 3);
    EXPECT(loop->currentPoint() == loop->start);
    EXPECT(!CatmullRomSpline::pathThrough({Vector2(1, 1)}).has_value());
}

TEST(periodicBSplineMeetsItsStartAndPassesThroughTheUniformJoints) {
    // A uniform cubic B-spline passes (P₀ + 4P₁ + P₂)/6 at its first joint: (100, 133.33) for the
    // golden ring, whose second control point is (100, 200).
    std::vector<Vector2> ring = {Vector2(0, 0), Vector2(100, 200), Vector2(200, 0), Vector2(300, 200), Vector2(400, 0)};
    std::optional<BSpline<Vector2>> spline = BSpline<Vector2>::periodic(3, ring);
    EXPECT(spline.has_value());
    ParameterRange range = spline->parameterRange();
    EXPECT(range.lowerBound == 3 && range.upperBound == 8);
    EXPECT(isClose(spline->point(range.lowerBound), Vector2(100, 133.3333f), 1e-3f));
    EXPECT(isClose(spline->point(range.lowerBound), spline->point(range.upperBound), 1e-2f));
    // The first derivative closes too (C¹ and above).
    EXPECT(isClose(spline->firstDerivative(range.lowerBound), spline->firstDerivative(range.upperBound), 5e-2f));
}

TEST(periodicQuadraticBSplineJointsAreTheMidpointsOfTheRing) {
    // A uniform quadratic B-spline passes through the midpoints of its control polygon's sides.
    std::vector<Vector2> square = {Vector2(0, 0), Vector2(10, 0), Vector2(10, 10), Vector2(0, 10)};
    std::optional<BSpline<Vector2>> spline = BSpline<Vector2>::periodic(2, square);
    EXPECT(spline.has_value());
    EXPECT(isClose(spline->point(2), Vector2(5, 0), 1e-4f));
    EXPECT(isClose(spline->point(3), Vector2(10, 5), 1e-4f));
    EXPECT(isClose(spline->point(4), Vector2(5, 10), 1e-4f));
    EXPECT(isClose(spline->point(5), Vector2(0, 5), 1e-4f));
    EXPECT(isClose(spline->point(6), Vector2(5, 0), 1e-4f));
}

TEST(periodicBSplineNeedsMoreControlPointsThanTheDegree) {
    EXPECT(!BSpline<Vector2>::periodic(3, {Vector2(0, 0), Vector2(1, 0), Vector2(1, 1)}).has_value());
    EXPECT(!BSpline<Vector2>::periodic(0, {Vector2(0, 0), Vector2(1, 0), Vector2(1, 1)}).has_value());
    EXPECT(!BSpline<Vector2>::closedPath({Vector2(0, 0), Vector2(1, 0), Vector2(1, 1)}).has_value());
}

TEST(closedBSplinePathMatchesTheGoldenCubicsAndCloses) {
    std::vector<Vector2> ring = {Vector2(0, 0), Vector2(100, 200), Vector2(200, 0), Vector2(300, 200), Vector2(400, 0)};
    std::optional<Path2D> path = BSpline<Vector2>::closedPath(ring);
    EXPECT(path.has_value());
    EXPECT(path->isClosed());
    EXPECT(path->segments().size() == 5);
    EXPECT(path->currentPoint() == path->start);
    // Golden piece 0: (100,133.33) (133.33,133.33) (166.67,66.67) (200,66.67).
    EXPECT(isClose(path->start, Vector2(100, 133.3333f), 0.02f));
    EXPECT(isClose(path->segments()[0].startHandle, Vector2(133.3333f, 133.3333f), 0.02f));
    EXPECT(isClose(path->segments()[0].endHandle, Vector2(166.6667f, 66.6667f), 0.02f));
    EXPECT(isClose(path->segments()[0].end(), Vector2(200, 66.6667f), 0.02f));
    // Golden piece 2 ends at (316.67, 33.33) with handles (333.33,133.33), (366.67,66.67).
    EXPECT(isClose(path->segments()[2].startHandle, Vector2(333.3333f, 133.3333f), 0.03f));
    EXPECT(isClose(path->segments()[2].endHandle, Vector2(366.6667f, 66.6667f), 0.03f));
    EXPECT(isClose(path->segments()[2].end(), Vector2(316.6667f, 33.3333f), 0.03f));
    // Smooth at every joint: the handles either side of a node are in line with it.
    size_t count = path->segments().size();
    for (size_t index = 0; index < count; index += 1) {
        const Path2D::Segment& before = path->segments()[index];
        const Path2D::Segment& after = path->segments()[(index + 1) % count];
        Vector2 node = before.end();
        EXPECT(isApproximatelyEqual((node - before.endHandle).normalized().cross((after.startHandle - node).normalized()), 0.0f, 1e-3f));
    }
}
