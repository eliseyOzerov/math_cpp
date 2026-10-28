// Tests of the fitting, hull, clipping and offset functions ported from graphics_cpp curves/fit and
// curves/offset (slice S2): single-piece fits, fits with corners, quadratic approximation, convex hulls,
// half-plane clipping, circles through three points, offsets with a distance function, circular arcs
// and stroke outlines. Golden values are from graphics_cpp tests/golden/{fit,offset}.json (a left-positive
// distance there is a right-positive one here, so its sign is flipped).
#include <algorithm>
#include <cmath>
#include <vector>

#include "../Curves/CubicBezier.h"
#include "../Curves/Path2D.h"
#include "../Curves/Polygon2D.h"
#include "../Curves/QuadraticApproximation.h"
#include "../Geometry/Circle2.h"
#include "../Geometry/ConvexHull2D.h"
#include "../Geometry/HalfPlane2.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Curve2 = CubicBezier<Vector2>;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

bool arePointsClose(const std::vector<Vector2>& first, const std::vector<Vector2>& second, float tolerance = 1e-3f) {
    if (first.size() != second.size()) return false;
    for (size_t index = 0; index < first.size(); index += 1) {
        if (!isClose(first[index], second[index], tolerance)) return false;
    }
    return true;
}

Curve2 semicircleLike() { return Curve2(Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(100, 0)); }

/// The quarter of the circle of radius 100 about the origin, counterclockwise from (100, 0).
Curve2 quarterCircle() {
    constexpr float handle = 55.22847f;
    return Curve2(Vector2(100, 0), Vector2(100, handle), Vector2(handle, 100), Vector2(0, 100));
}

} // namespace

// MARK: Single-piece fit and fits with corners

TEST(fittedSinglePieceUsesTheGivenDirections) {
    // Golden fitCubic: two points, leaving along +x and arriving along -(0,1)·: handles a third of the chord (50).
    FittedCubic twoPoints = Curve2::fittedSinglePiece({Vector2(0, 0), Vector2(30, 40)}, Vector2(1, 0), Vector2(0, 1));
    EXPECT(isClose(twoPoints.cubic.startHandle, Vector2(16.666666f, 0)) && isClose(twoPoints.cubic.endHandle, Vector2(30, 56.666667f)));
    EXPECT(twoPoints.largestError == 0 && twoPoints.cubic.start == Vector2(0, 0) && twoPoints.cubic.end == Vector2(30, 40));
    // Golden fitCubic: nine points of an arch, up at both ends (end direction points back into the curve, so (0,1)).
    std::vector<Vector2> arch = {Vector2(0, 0), Vector2(4.296875f, 32.8125f), Vector2(15.625f, 56.25f), Vector2(31.640625f, 70.3125f),
                                 Vector2(50, 75), Vector2(68.359375f, 70.3125f), Vector2(84.375f, 56.25f), Vector2(95.703125f, 32.8125f),
                                 Vector2(100, 0)};
    FittedCubic fit = Curve2::fittedSinglePiece(arch, Vector2(0, 1), Vector2(0, 1));
    EXPECT(isClose(fit.cubic.startHandle, Vector2(0, 99.99725f), 0.05f) && isClose(fit.cubic.endHandle, Vector2(100, 99.99725f), 0.05f));
    EXPECT(isApproximatelyEqual(fit.largestError, 0.00206f, 2e-3f));
    // Property: the handles leave along the given directions, and every point is within the reported error.
    EXPECT(fit.cubic.startHandle.x == 0 && fit.cubic.endHandle.x == 100);
    for (Vector2 point : arch) EXPECT(fit.cubic.nearestPoint(point).distance <= fit.largestError + 1e-3f);
    // Edge cases: no points, one point, repeated points.
    EXPECT(Curve2::fittedSinglePiece({}, Vector2(1, 0), Vector2(1, 0)).cubic == Curve2());
    FittedCubic single = Curve2::fittedSinglePiece({Vector2(3, 4)}, Vector2(1, 0), Vector2(1, 0));
    EXPECT(single.cubic.start == Vector2(3, 4) && single.cubic.end == Vector2(3, 4) && single.largestError == 0);
    FittedCubic repeated = Curve2::fittedSinglePiece({Vector2(0, 0), Vector2(0, 0), Vector2(30, 40)}, Vector2(1, 0), Vector2(0, 1));
    EXPECT(isClose(repeated.cubic.startHandle, twoPoints.cubic.startHandle));
}

TEST(fittedWithCornersKeepsEachCornerAsAPieceEnd) {
    // An L of dense points: the corner at (10, 0) is a 90 degree turn, over the 60 degree default, so it is a piece end.
    std::vector<Vector2> corner;
    for (int index = 0; index <= 10; index += 1) corner.push_back(Vector2(static_cast<float>(index), 0));
    for (int index = 1; index <= 10; index += 1) corner.push_back(Vector2(10, static_cast<float>(index)));
    std::vector<Curve2> pieces = Curve2::fittedWithCorners(corner, 0.05f);
    bool hasCornerEnd = false;
    for (const Curve2& piece : pieces) hasCornerEnd = hasCornerEnd || piece.end == Vector2(10, 0);
    EXPECT(pieces.size() == 2 && hasCornerEnd);
    EXPECT(pieces.front().start == corner.front() && pieces.back().end == corner.back());
    // Property: pieces join end to start, and every point is within the tolerance.
    EXPECT(pieces[0].end == pieces[1].start);
    for (Vector2 point : corner) EXPECT(Curve2::nearest(pieces, point).distance <= 0.05f + 1e-4f);
    // A 180 degree limit never cuts: one smooth run, fitted by the general fitter.
    std::vector<Curve2> smooth = Curve2::fittedWithCorners(corner, 0.05f, Angle::degrees(180));
    EXPECT(smooth.size() == Curve2::fitted(corner, 0.05f, std::nullopt, std::nullopt, false).size());
    EXPECT(Curve2::fittedWithCorners({Vector2(1, 1)}, 0.1f).empty());
    EXPECT(Curve2::fittedWithCorners({Vector2(1, 1), Vector2(1, 1)}, 0.1f).empty());
}

// MARK: Quadratic approximation

namespace {

/// How far the cubic is from the quadratic spline with these controls: the largest, over samples of
/// the cubic, of the distance to the densely sampled spline.
float distanceToSpline(const Curve2& cubic, const std::vector<Vector2>& controls) {
    std::vector<Vector2> joins{cubic.start};
    for (size_t index = 0; index + 1 < controls.size(); index += 1) joins.push_back((controls[index] + controls[index + 1]) * 0.5f);
    joins.push_back(cubic.end);
    std::vector<Vector2> dense;
    for (size_t piece = 0; piece < controls.size(); piece += 1) {
        for (int step = 0; step <= 200; step += 1) {
            float parameter = static_cast<float>(step) / 200;
            Vector2 first = Vector2::linearBlend(joins[piece], controls[piece], parameter);
            Vector2 second = Vector2::linearBlend(controls[piece], joins[piece + 1], parameter);
            dense.push_back(Vector2::linearBlend(first, second, parameter));
        }
    }
    float largest = 0;
    for (int step = 0; step <= 50; step += 1) {
        Vector2 sample = cubic.point(static_cast<float>(step) / 50);
        float nearest = 1e9f;
        for (Vector2 point : dense) nearest = std::min(nearest, sample.distance(point));
        largest = std::max(largest, nearest);
    }
    return largest;
}

} // namespace

TEST(quadraticApproximationFollowsTheCubic) {
    // Golden cubicToQuads: the semicircle-like cubic within 5 is two quadratics, controls (0, 75) and (100, 75).
    std::vector<Vector2> coarse = QuadraticApproximation::controlPoints(semicircleLike(), 5);
    EXPECT(arePointsClose(coarse, {Vector2(0, 75), Vector2(100, 75)}, 1e-2f));
    // Golden: a cubic with a handle on its end needs controls (0, 0) and (100, 62.5) within 5.
    Curve2 doubleHandle(Vector2(0, 0), Vector2(0, 0), Vector2(100, 50), Vector2(100, 100));
    EXPECT(arePointsClose(QuadraticApproximation::controlPoints(doubleHandle, 5), {Vector2(0, 0), Vector2(100, 62.5f)}, 1e-2f));
    // A quadratic raised to a cubic comes back as its single control (the hand-worked case: (50, 100)).
    Curve2 raised(Vector2(0, 0), Vector2(100.0f / 3, 200.0f / 3), Vector2(200.0f / 3, 200.0f / 3), Vector2(100, 0));
    EXPECT(arePointsClose(QuadraticApproximation::controlPoints(raised, 0.01f), {Vector2(50, 100)}, 1e-2f));
    // Golden: a looping cubic at 0.1 starts with the control (17.3077, 11.5385).
    Curve2 loopy(Vector2(0, 0), Vector2(150, 100), Vector2(-50, 100), Vector2(100, 0));
    std::vector<Vector2> tight = QuadraticApproximation::controlPoints(loopy, 0.1f);
    EXPECT(isClose(tight.front(), Vector2(17.307692f, 11.538462f), 1e-2f));
    // Property: the spline stays within the tolerance of the cubic (plus a hair for float sampling), and a
    // smaller tolerance never needs fewer pieces.
    for (float tolerance : {0.05f, 0.5f, 5.0f}) {
        std::vector<Vector2> controls = QuadraticApproximation::controlPoints(semicircleLike(), tolerance);
        EXPECT(distanceToSpline(semicircleLike(), controls) <= tolerance + 0.02f);
    }
    EXPECT(QuadraticApproximation::controlPoints(semicircleLike(), 0.05f).size() >= QuadraticApproximation::controlPoints(semicircleLike(), 5).size());
}

TEST(staysWithinTestsTheDifferenceCubic) {
    Vector2 zero;
    EXPECT(QuadraticApproximation::staysWithin(zero, zero, zero, zero, 0.1f));
    // Controls (0, 0), (3, 0), (3, 0), (0, 0): the middle is (0 + 9 + 9 + 0)/8 = 2.25 away, outside 2, inside 3.
    EXPECT(!QuadraticApproximation::staysWithin(zero, Vector2(3, 0), Vector2(3, 0), zero, 2));
    EXPECT(QuadraticApproximation::staysWithin(zero, Vector2(3, 0), Vector2(3, 0), zero, 3));
    // Inner controls inside the tolerance are enough.
    EXPECT(QuadraticApproximation::staysWithin(zero, Vector2(0.5f, 0), Vector2(-0.5f, 0.2f), zero, 1));
}

// MARK: Convex hulls, half-planes, circles

TEST(convexHullIsTheCounterclockwiseWrap) {
    // Golden hull: repeated and inner points drop out; the hull starts at the leftmost point.
    std::vector<Vector2> points = {Vector2(3, 4), Vector2(1, 1), Vector2(8, 2), Vector2(5, 5), Vector2(2, 9), Vector2(9, 9), Vector2(5, 5),
                                   Vector2(7, 3), Vector2(4, 4), Vector2(0, 6), Vector2(6, 0), Vector2(1, 1), Vector2(10, 5)};
    std::vector<Vector2> hull = ConvexHull2D::of(points);
    EXPECT(hull == (std::vector<Vector2>{Vector2(0, 6), Vector2(1, 1), Vector2(6, 0), Vector2(8, 2), Vector2(10, 5), Vector2(9, 9), Vector2(2, 9)}));
    // Property: every turn is a left turn, every input point is inside or on it, and sorting first changes nothing.
    for (size_t index = 0; index < hull.size(); index += 1) {
        Vector2 a = hull[index], b = hull[(index + 1) % hull.size()], c = hull[(index + 2) % hull.size()];
        EXPECT((b - a).cross(c - b) > 0);
    }
    for (Vector2 point : points) {
        for (size_t index = 0; index < hull.size(); index += 1) {
            EXPECT((hull[(index + 1) % hull.size()] - hull[index]).cross(point - hull[index]) >= 0);
        }
    }
    std::vector<Vector2> sorted = points;
    std::sort(sorted.begin(), sorted.end(), [](Vector2 first, Vector2 second) { return first.x < second.x || (first.x == second.x && first.y < second.y); });
    EXPECT(ConvexHull2D::of(sorted, true) == hull);
    EXPECT(ConvexHull2D::of({}).empty() && ConvexHull2D::of({Vector2(1, 2)}).size() == 1);
}

TEST(mergedHullIsTheHullOfBothPolygons) {
    // Golden mergeHulls: two overlapping hexagons.
    std::vector<Vector2> right = {Vector2(6, 2), Vector2(10, -1), Vector2(16, -1), Vector2(20, 2), Vector2(16, 5), Vector2(10, 5)};
    std::vector<Vector2> left = {Vector2(0, 0), Vector2(4, -3), Vector2(10, -3), Vector2(14, 0), Vector2(10, 3), Vector2(4, 3)};
    std::vector<Vector2> merged = ConvexHull2D::merged(right, left);
    EXPECT(merged == (std::vector<Vector2>{Vector2(0, 0), Vector2(4, -3), Vector2(10, -3), Vector2(16, -1), Vector2(20, 2), Vector2(16, 5), Vector2(10, 5), Vector2(4, 3)}));
    // Property: equal to the hull of all the points; two squares (golden) give their joint outline.
    std::vector<Vector2> all = right;
    all.insert(all.end(), left.begin(), left.end());
    EXPECT(merged == ConvexHull2D::of(all));
    std::vector<Vector2> squares = ConvexHull2D::merged({Vector2(0, 0), Vector2(10, 0), Vector2(10, 10), Vector2(0, 10)},
                                                         {Vector2(5, 5), Vector2(15, 5), Vector2(15, 15), Vector2(5, 15)});
    EXPECT(squares == (std::vector<Vector2>{Vector2(0, 0), Vector2(10, 0), Vector2(15, 5), Vector2(15, 15), Vector2(5, 15), Vector2(0, 10)}));
}

TEST(halfPlaneKeepsTheSideTheNormalPointsTo) {
    HalfPlane2 plane{Vector2(5, 0), Vector2(1, 0)};
    EXPECT(plane.signedDistance(Vector2(8, 3)) == 3 && plane.signedDistance(Vector2(2, 3)) == -3);
    EXPECT(plane.contains(Vector2(5, 7)) && plane.contains(Vector2(6, 0)) && !plane.contains(Vector2(4, 0)));
    // Golden clipPolygon: a 10 by 10 square cut at x = 5 keeps its right half, in order.
    std::vector<Vector2> square = {Vector2(0, 0), Vector2(10, 0), Vector2(10, 10), Vector2(0, 10)};
    EXPECT(plane.clipped(square) == (std::vector<Vector2>{Vector2(5, 0), Vector2(10, 0), Vector2(10, 10), Vector2(5, 10)}));
    // All kept stays whole (golden: plane at x = -5); all cut leaves nothing.
    EXPECT((HalfPlane2{Vector2(-5, 0), Vector2(1, 0)}).clipped(square) == square);
    EXPECT((HalfPlane2{Vector2(15, 0), Vector2(1, 0)}).clipped(square).empty());
    // The normal's length scales distances only: the same cut.
    EXPECT((HalfPlane2{Vector2(5, 0), Vector2(4, 0)}).clipped(square) == plane.clipped(square));
    // Property: the two halves of a cut have areas that add up to the square's.
    Polygon2D whole(square);
    std::optional<Polygon2D> kept = whole.clippedBy(HalfPlane2{Vector2(3, 0), Vector2(1, 0)});
    std::optional<Polygon2D> other = whole.clippedBy(HalfPlane2{Vector2(3, 0), Vector2(-1, 0)});
    EXPECT(kept && other);
    EXPECT(isApproximatelyEqual(kept->signedArea(), 70.0f, 1e-3f) && isApproximatelyEqual(other->signedArea(), 30.0f, 1e-3f));
}

TEST(polygonClippedByReturnsNothingWhenLessThanATriangleRemains) {
    Polygon2D square({Vector2(0, 0), Vector2(10, 0), Vector2(10, 10), Vector2(0, 10)});
    std::optional<Polygon2D> half = square.clippedBy(HalfPlane2{Vector2(5, 0), Vector2(1, 0)});
    EXPECT(half && half->corners().size() == 4 && isApproximatelyEqual(half->signedArea(), 50.0f, 1e-3f));
    EXPECT(!square.clippedBy(HalfPlane2{Vector2(20, 0), Vector2(1, 0)}));
    // Touching along an edge keeps nothing with an area (the corners come back doubled up, or not at all).
    std::optional<Polygon2D> touching = square.clippedBy(HalfPlane2{Vector2(10, 0), Vector2(1, 0)});
    EXPECT(!touching || isApproximatelyEqual(touching->signedArea(), 0.0f, 1e-4f));
}

TEST(circleThroughThreePointsIsTheCircumcircle) {
    // (1, 0), (0, 1), (-1, 0) lie on the unit circle.
    std::optional<Circle2> unit = Circle2::through(Vector2(1, 0), Vector2(0, 1), Vector2(-1, 0));
    EXPECT(unit && isClose(unit->centre, Vector2(0, 0), 1e-5f) && isApproximatelyEqual(unit->radius, 1.0f, 1e-5f));
    // The right triangle (0,0), (6,0), (0,8) has the hypotenuse as diameter: centre (3, 4), radius 5.
    std::optional<Circle2> right = Circle2::through(Vector2(0, 0), Vector2(6, 0), Vector2(0, 8));
    EXPECT(right && isClose(right->centre, Vector2(3, 4), 1e-4f) && isApproximatelyEqual(right->radius, 5.0f, 1e-4f));
    // Property: all three points are equally far from the centre, whichever order, and far from the origin too.
    Vector2 shift(1000, -2000);
    std::optional<Circle2> moved = Circle2::through(Vector2(0, 8) + shift, Vector2(0, 0) + shift, Vector2(6, 0) + shift);
    EXPECT(moved && isClose(moved->centre, Vector2(3, 4) + shift, 0.05f) && isApproximatelyEqual(moved->radius, 5.0f, 0.05f));
    // On a line (including a repeated point) there is no circle.
    EXPECT(!Circle2::through(Vector2(0, 0), Vector2(1, 1), Vector2(2, 2)));
    EXPECT(!Circle2::through(Vector2(1, 1), Vector2(1, 1), Vector2(5, 3)));
}

// MARK: Offsets with a distance function

TEST(scaledOutMovesAPieceAlongItsNormals) {
    // Golden scale (left-positive 10 = right-positive -10) of a curve whose end normals are parallel.
    Curve2 moved = semicircleLike().scaledOut(-10.0f);
    EXPECT(isClose(moved.start, Vector2(-10, 0)) && isClose(moved.startHandle, Vector2(-10, 100)));
    EXPECT(isClose(moved.endHandle, Vector2(110, 100)) && isClose(moved.end, Vector2(110, 0)));
    // Golden: a straight piece moves along its normal; left +10 on a 45 degree line is (-7.07, 7.07).
    Curve2 line(Vector2(0, 0), Vector2(10, 10), Vector2(20, 20), Vector2(30, 30));
    Curve2 movedLine = line.scaledOut(-10.0f);
    EXPECT(isClose(movedLine.start, Vector2(-7.071068f, 7.071068f)) && isClose(movedLine.startHandle, Vector2(2.928932f, 17.071068f)));
    EXPECT(isClose(movedLine.end, Vector2(22.928932f, 37.071068f)));
    // Golden: an S piece, 7 to its right.
    Curve2 snake(Vector2(0, 0), Vector2(50, 100), Vector2(50, -100), Vector2(100, 0));
    Curve2 movedSnake = snake.scaledOut(7.0f);
    EXPECT(isClose(movedSnake.start, Vector2(6.260990f, -3.130495f), 2e-3f) && isClose(movedSnake.startHandle, Vector2(56.260990f, 96.869505f), 2e-3f));
    EXPECT(isClose(movedSnake.endHandle, Vector2(56.260990f, -103.130495f), 2e-3f) && isClose(movedSnake.end, Vector2(106.260990f, -3.130495f), 2e-3f));
    // Property: a circular arc moved out by d is the concentric arc: radius 100 + d everywhere (to the cubic's own 3e-4).
    Curve2 outer = quarterCircle().scaledOut(10.0f);
    for (float parameter : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) EXPECT(isApproximatelyEqual(outer.point(parameter).length(), 110.0f, 0.1f));
    Curve2 inner = quarterCircle().scaledOut(-30.0f);
    for (float parameter : {0.0f, 0.5f, 1.0f}) EXPECT(isApproximatelyEqual(inner.point(parameter).length(), 70.0f, 0.1f));
}

TEST(scaledOutByAFunctionGradesTheDistance) {
    // Golden scaleRamp: left-positive -3 growing to 8 is right-positive 3 falling to -8, i.e. 3 - 11u.
    Curve2 snake(Vector2(0, 0), Vector2(50, 100), Vector2(50, -100), Vector2(100, 0));
    Curve2 ramp = snake.scaledOut(OffsetReach([](float parameter) { return 3 - 11 * parameter; }));
    EXPECT(isClose(ramp.start, Vector2(2.683282f, -1.341641f), 2e-3f) && isClose(ramp.startHandle, Vector2(49.403715f, 100.298142f), 2e-3f));
    EXPECT(isClose(ramp.endHandle, Vector2(46.124149f, -98.062074f), 2e-3f) && isClose(ramp.end, Vector2(92.844582f, 3.577709f), 2e-3f));
    // Property: ends are exactly reach(0) and reach(1) out along the normals.
    Curve2 arc = quarterCircle().scaledOut(OffsetReach([](float parameter) { return 10 + 20 * parameter; }));
    EXPECT(isClose(arc.start, Vector2(110, 0), 1e-3f) && isClose(arc.end, Vector2(0, 130), 1e-3f));
    // A constant function on an arc stays near the concentric arc (within 1.5% of the radius: the graded
    // handles move by the distance along their ray, not in proportion to it, which is exact only for `scaledOut(distance)`).
    Curve2 steady = quarterCircle().scaledOut(OffsetReach([](float) { return 10.0f; }));
    EXPECT(isApproximatelyEqual(steady.point(0.5f).length(), 110.0f, 1.5f));
}

TEST(pointOffsetByMovesAlongTheRightNormal) {
    // Golden offsetAt: left 6 at t = 0 of a curve that starts towards (150, 100) is (-3.33, 4.99); right is the mirror.
    Curve2 loopy(Vector2(0, 0), Vector2(150, 100), Vector2(-50, 100), Vector2(100, 0));
    EXPECT(isClose(loopy.pointOffsetBy(0, -6), Vector2(-3.328201f, 4.992302f), 1e-3f));
    EXPECT(isClose(loopy.pointOffsetBy(0, 6), Vector2(3.328201f, -4.992302f), 1e-3f));
    // Property: the offset point is exactly |distance| from the curve point, at a right angle to the tangent.
    for (float parameter : {0.2f, 0.5f, 0.9f}) {
        Vector2 point = loopy.point(parameter);
        Vector2 moved = loopy.pointOffsetBy(parameter, 4);
        EXPECT(isApproximatelyEqual(moved.distance(point), 4.0f, 1e-4f));
        EXPECT(std::fabs((moved - point).dot(loopy.tangentDirection(parameter))) < 1e-4f);
    }
    EXPECT(loopy.pointOffsetBy(0.3f, 0) == loopy.point(0.3f));
}

TEST(offsetByReachFollowsTheTrueOffsetAndJoinsUp) {
    Curve2 curve = semicircleLike();
    std::vector<Curve2> pieces = curve.offsetByReach([](float) { return 10.0f; });
    EXPECT(pieces.size() >= 2);
    // Ends: exactly the true offset points at 0 and 1; consecutive pieces meet.
    EXPECT(isClose(pieces.front().start, curve.pointOffsetBy(0, 10), 1e-3f) && isClose(pieces.back().end, curve.pointOffsetBy(1, 10), 1e-3f));
    for (size_t index = 0; index + 1 < pieces.size(); index += 1) EXPECT(isClose(pieces[index].end, pieces[index + 1].start, 0.05f));
    // Property: every sample of every piece is about 10 from the original curve.
    for (const Curve2& piece : pieces) {
        for (float parameter : {0.0f, 0.3f, 0.5f, 0.8f, 1.0f}) {
            EXPECT(std::fabs(curve.nearestPoint(piece.point(parameter), 256).distance - 10.0f) < 1.0f);
        }
    }
    // A graded reach: 2 at the start, 12 at the end, and the ends are exactly there.
    std::vector<Curve2> graded = curve.offsetByReach([](float parameter) { return 2 + 10 * parameter; });
    EXPECT(isClose(graded.front().start, curve.pointOffsetBy(0, 2), 1e-3f) && isClose(graded.back().end, curve.pointOffsetBy(1, 12), 1e-3f));
    // A straight curve is one piece.
    EXPECT(Curve2(Vector2(0, 0), Vector2(10, 0), Vector2(20, 0), Vector2(30, 0)).offsetByReach([](float) { return 2.0f; }).size() == 1);
}

// MARK: Circular arcs and outlines

TEST(circularArcsStayWithinTheThreshold) {
    std::vector<CircularArcPiece> arcs = semicircleLike().circularArcs(5);
    // Golden arcs: two arcs, the first ending near t = 0.653 at (72.2, 68.0), both clockwise.
    EXPECT(arcs.size() == 2 && !arcs[0].isCounterclockwise && !arcs[1].isCounterclockwise);
    EXPECT(isApproximatelyEqual(arcs[0].endParameter, 0.652998f, 5e-3f) && isClose(arcs[0].to, Vector2(72.2334f, 67.9775f), 0.5f));
    EXPECT(arcs[0].centre && isClose(*arcs[0].centre, Vector2(50.688f, 18.505f), 0.5f) && isApproximatelyEqual(arcs[0].radius, 53.96f, 0.5f));
    EXPECT(arcs[1].centre && isClose(*arcs[1].centre, Vector2(21.874f, 7.748f), 0.5f) && isApproximatelyEqual(arcs[1].radius, 78.51f, 0.5f));
    // Property: arcs run end to end over 0…1, start and end on the curve, and keep close to it.
    EXPECT(arcs.front().startParameter == 0 && arcs.back().endParameter == 1);
    for (size_t index = 0; index + 1 < arcs.size(); index += 1) {
        EXPECT(arcs[index].to == arcs[index + 1].from && arcs[index].endParameter == arcs[index + 1].startParameter);
    }
    for (const CircularArcPiece& arc : arcs) {
        for (float share : {0.2f, 0.5f, 0.8f}) {
            Vector2 sample = semicircleLike().point(arc.startParameter + (arc.endParameter - arc.startParameter) * share);
            EXPECT(std::fabs(sample.distance(*arc.centre) - arc.radius) < 5.5f);
        }
    }
    // A tighter threshold needs more arcs.
    EXPECT(semicircleLike().circularArcs(0.2f).size() > arcs.size());
}

TEST(circularArcsOfACircleAndOfALine) {
    // A quarter circle is one arc, centre at the origin, counterclockwise from angle 0 to 90 degrees.
    std::vector<CircularArcPiece> arcs = quarterCircle().circularArcs(0.5f);
    EXPECT(arcs.size() == 1 && arcs[0].isCounterclockwise && arcs[0].centre);
    EXPECT(isClose(*arcs[0].centre, Vector2(0, 0), 0.1f) && isApproximatelyEqual(arcs[0].radius, 100.0f, 0.1f));
    EXPECT(isApproximatelyEqual(arcs[0].startAngle.radians, 0.0f, 1e-3f) && isApproximatelyEqual(arcs[0].endAngle.radians, Angle::degrees(90).radians, 1e-3f));
    // A straight cubic is one straight part: no centre, an infinite radius.
    std::vector<CircularArcPiece> straight = Curve2(Vector2(0, 0), Vector2(10, 0), Vector2(20, 0), Vector2(30, 0)).circularArcs();
    EXPECT(straight.size() == 1 && !straight[0].centre && std::isinf(straight[0].radius));
    EXPECT(straight[0].from == Vector2(0, 0) && straight[0].to == Vector2(30, 0));
}

TEST(strokeOutlineSurroundsTheSpine) {
    Curve2 spine(Vector2(0, 0), Vector2(10, 0), Vector2(20, 0), Vector2(30, 0));
    // Two on each side of a straight spine of length 30: left side along y = 2, a cap, right side back along y = -2.
    Path2D outline = Path2D::strokeOutline(spine, 2);
    EXPECT(outline.isClosed() && outline.start == Vector2(0, 2) && outline.segments().size() == 3);
    EXPECT(outline.segments()[0].endPoint == Vector2(30, 2));
    EXPECT(outline.segments()[1].isLine() && outline.segments()[1].endPoint == Vector2(30, -2));
    EXPECT(outline.segments()[2].endPoint == Vector2(0, -2));
    // The path runs the left side forwards, so it is clockwise: its area is -(30 · 4).
    EXPECT(isApproximatelyEqual(outline.signedArea(), -120.0f, 1.0f));
    // Different sides: 1 on the left and 3 on the right gives a strip 4 wide, between y = 1 and y = -3.
    Path2D uneven = Path2D::strokeOutline(spine, 1, 3.0f);
    EXPECT(uneven.start == Vector2(0, 1) && uneven.segments()[2].endPoint == Vector2(0, -3));
    // Tapering both sides to 0 at the end gives a triangle of area 30 · 4 / 2 (the taper is linear in length).
    Path2D tapered = Path2D::strokeOutline(spine, 2, std::nullopt, 0.0f, 0.0f);
    EXPECT(isApproximatelyEqual(tapered.signedArea(), -60.0f, 1.0f));
    EXPECT(tapered.segments()[1].endPoint == Vector2(30, 0));
    EXPECT_PROBLEM(Path2D::strokeOutline(spine, std::numeric_limits<float>::infinity()), "finite");
}

TEST(strokeOutlinePiecesAreClosedAndAddUpToTheOutline) {
    Curve2 spine = semicircleLike();
    std::vector<Path2D> pieces = Path2D::strokeOutlinePieces(spine, 4);
    EXPECT(pieces.size() == spine.simplePiecesForOffset().size() && pieces.size() >= 2);
    for (const Path2D& piece : pieces) EXPECT(piece.isClosed() && piece.segments().size() == 3);
    // Each piece's left side begins where the previous one ended, to the float.
    for (size_t index = 0; index + 1 < pieces.size(); index += 1) {
        EXPECT(isClose(pieces[index].segments()[0].endPoint, pieces[index + 1].start, 0.1f));
    }
    // On a straight spine the pieces' areas add up to the whole strip.
    Curve2 straight(Vector2(0, 0), Vector2(10, 0), Vector2(20, 0), Vector2(30, 0));
    std::vector<Path2D> strips = Path2D::strokeOutlinePieces(straight, 2);
    EXPECT(strips.size() == 1 && isApproximatelyEqual(strips[0].signedArea(), -120.0f, 1.0f));
}
