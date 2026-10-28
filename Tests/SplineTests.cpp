// Ports of EditorTests/Math/Curves/ for the Catmull–Rom spline, the rational quadratic Bézier
// and the B-spline (CatmullRomSplineTests, RationalQuadraticBezierTests, BSplineTests), plus
// bit-for-bit checks against values printed by the Swift engine. `BSpline.path` is not ported
// (the core has no Path2D yet), so its two checks are left out.
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "../Curves/BSpline.h"
#include "../Curves/CatmullRomSpline.h"
#include "../Curves/RationalQuadraticBezier.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
constexpr float swiftPi = 0x1.921fb4p1f;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

bool isClose(Vector3 first, Vector3 second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }

bool sameBits(float value, std::uint32_t bits) { return std::bit_cast<std::uint32_t>(value) == bits; }

bool sameBits(Vector2 value, std::uint32_t x, std::uint32_t y) { return sameBits(value.x, x) && sameBits(value.y, y); }

bool sameBits(Vector3 value, std::uint32_t x, std::uint32_t y, std::uint32_t z) {
    return sameBits(value.x, x) && sameBits(value.y, y) && sameBits(value.z, z);
}

float ulp(float value) { return std::nextafter(std::fabs(value), infinity) - std::fabs(value); }

// MARK: Catmull–Rom

CatmullRomSpline evenLine() {
    return CatmullRomSpline({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0), Vector3(3, 0, 0)});
}

std::vector<Vector3> bendyPoints() {
    return {Vector3(0, 0, 0), Vector3(1, 2, 0), Vector3(3, 3, 1), Vector3(6, 1, 1), Vector3(7, -2, 0)};
}

} // namespace

TEST(catmullRomEvenlySpacedCollinearPointsGiveAStraightLineAtUnitSpeed) {
    CatmullRomSpline line = evenLine();
    for (float parameter : {0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.75f, 3.0f}) {
        EXPECT(isClose(line.point(parameter), Vector3(parameter, 0, 0)));
        EXPECT(isClose(line.firstDerivative(parameter), Vector3(1, 0, 0)));
    }
}

TEST(catmullRomTwoPointsGiveTheStraightSegmentBetweenThem) {
    CatmullRomSpline spline({Vector3(1, 1, 1), Vector3(3, 5, 1)});
    EXPECT((spline.parameterRange() == ParameterRange{0, 1}));
    EXPECT(isClose(spline.point(0.5f), Vector3(2, 3, 1)));
    EXPECT(isClose(spline.firstDerivative(0.5f), Vector3(2, 4, 0)));
}

TEST(catmullRomSymmetricArchHasTheValuesWorkedOutByHand) {
    CatmullRomSpline spline({Vector3(0, 0, 0), Vector3(1, 1, 0), Vector3(3, 1, 0), Vector3(4, 0, 0)});
    EXPECT(isClose(spline.segments[1].startHandle, Vector3(1.519856f, 1.215331f, 0), 1e-5f));
    EXPECT(isClose(spline.segments[1].endHandle, Vector3(2.480144f, 1.215331f, 0), 1e-5f));
    EXPECT(isClose(spline.point(1.5f), Vector3(2, 1.161498f, 0), 1e-5f));
}

TEST(catmullRomUnevenGapsGiveTheHandlesWorkedOutByHand) {
    CatmullRomSpline spline({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 4, 0), Vector3(10, 4, 0)});
    EXPECT(isClose(spline.segments[1].startHandle, Vector3(13.0f / 9, 4.0f / 9, 0), 1e-5f));
    EXPECT(isClose(spline.segments[1].endHandle, Vector3(0.2f, 3.2f, 0), 1e-5f));
}

TEST(catmullRomUnevenGapsMatchTheBarryGoldmanPyramid) {
    std::vector<Vector3> points = {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 4, 0), Vector3(10, 4, 0)};
    float knots[] = {0, 1, 3, 6};
    auto blend = [](Vector3 first, Vector3 second, float start, float end, float time) {
        return Vector3::linearBlend(first, second, (time - start) / (end - start));
    };
    CatmullRomSpline spline(points);
    for (float time : {1.0f, 1.5f, 2.0f, 2.5f, 3.0f}) {
        Vector3 firstBlend = blend(points[0], points[1], knots[0], knots[1], time);
        Vector3 secondBlend = blend(points[1], points[2], knots[1], knots[2], time);
        Vector3 thirdBlend = blend(points[2], points[3], knots[2], knots[3], time);
        Vector3 leftBlend = blend(firstBlend, secondBlend, knots[0], knots[2], time);
        Vector3 rightBlend = blend(secondBlend, thirdBlend, knots[1], knots[3], time);
        Vector3 expected = blend(leftBlend, rightBlend, knots[1], knots[2], time);
        float parameter = 1 + (time - 1) / 2;
        EXPECT(isClose(spline.point(parameter), expected, 1e-5f));
    }
}

TEST(catmullRomPassesThroughEveryInputPoint) {
    std::vector<Vector3> points = bendyPoints();
    CatmullRomSpline spline(points);
    EXPECT((spline.parameterRange() == ParameterRange{0, 4}));
    for (size_t index = 0; index < points.size(); index += 1) {
        EXPECT(isClose(spline.point(static_cast<float>(index)), points[index]));
    }
}

TEST(catmullRomPiecesJoinWithMatchingDirection) {
    CatmullRomSpline spline(bendyPoints());
    for (size_t index = 1; index < 4; index += 1) {
        Vector3 endOfPrevious = spline.segments[index - 1].firstDerivative(1);
        Vector3 startOfNext = spline.segments[index].firstDerivative(0);
        EXPECT(isClose(endOfPrevious.normalized(), startOfNext.normalized(), 1e-4f));
    }
}

TEST(catmullRomDerivativeMatchesFiniteDifferences) {
    CatmullRomSpline spline(bendyPoints());
    float step = 1e-3f;
    for (int index = 0; 0.1f + static_cast<float>(index) * 0.4f <= 3.9f; index += 1) {
        float parameter = 0.1f + static_cast<float>(index) * 0.4f;
        Vector3 slope = (spline.point(parameter + step) - spline.point(parameter - step)) / (2 * step);
        EXPECT(isClose(slope, spline.firstDerivative(parameter), 5e-3f));
    }
}

TEST(catmullRomCentripetalSplineDoesNotOvershootASharpTurn) {
    CatmullRomSpline spline({Vector3(0, 0, 0), Vector3(10, 0, 0), Vector3(10, 0.5f, 0), Vector3(0, 0.5f, 0)});
    for (int index = 0; index <= 30; index += 1) {
        Vector3 point = spline.point(static_cast<float>(index) / 10);
        EXPECT(point.x <= 10.5f && point.x >= -0.5f);
        EXPECT(point.y <= 1 && point.y >= -0.5f);
    }
}

TEST(catmullRomCoincidentNeighboursAreMerged) {
    CatmullRomSpline withDuplicate({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 0, 0), Vector3(2, 1, 0)});
    CatmullRomSpline without({Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(2, 1, 0)});
    EXPECT(withDuplicate.throughPoints.size() == 3);
    EXPECT((withDuplicate.parameterRange() == ParameterRange{0, 2}));
    for (int index = 0; index <= 8; index += 1) {
        float parameter = static_cast<float>(index) / 4;
        EXPECT(isClose(withDuplicate.point(parameter), without.point(parameter)));
    }
}

TEST(catmullRomAllPointsCoincidentGivesAMotionlessSpline) {
    Vector3 point(2, 2, 2);
    CatmullRomSpline spline({point, point, point});
    EXPECT((spline.parameterRange() == ParameterRange{0, 1}));
    EXPECT(spline.point(0.5f) == point);
    EXPECT(spline.firstDerivative(0.5f) == Vector3(0, 0, 0));
}

TEST(catmullRomOutsideTheRangeTheEndPiecesContinue) {
    CatmullRomSpline line = evenLine();
    EXPECT(isClose(line.point(-1), Vector3(-1, 0, 0)));
    EXPECT(isClose(line.point(4), Vector3(4, 0, 0)));
}

TEST(catmullRomRefusesFewerThanTwoPointsAndANonFiniteParameter) {
    EXPECT_PROBLEM(CatmullRomSpline({Vector3(0, 0, 0)}), "A spline needs at least two points.");
    EXPECT_PROBLEM(CatmullRomSpline(std::vector<Vector3>{}), "A spline needs at least two points.");
    EXPECT_PROBLEM(evenLine().point(notANumber), "finite");
    EXPECT_PROBLEM(evenLine().firstDerivative(infinity), "finite");
}

TEST(catmullRomMatchesSwiftBitForBit) {
    CatmullRomSpline spline(bendyPoints());
    EXPECT(sameBits(spline.point(2.3f), 0x407aabb0, 0x402c7e2a, 0x3f90ce40));
    EXPECT(sameBits(spline.firstDerivative(2.3f), 0x4053b158, 0xbfd7e5ec, 0x3e638422));
    EXPECT(sameBits(spline.totalLength(), 0x413cfae2));
    EXPECT(sameBits(spline.length(0.5f, 3.2f), 0x40fe6957));
    EXPECT(sameBits(spline.parameterAtDistance(4), 0x3fd719b4));
}

// MARK: Rational quadratic Bézier

namespace {

const Vector2 conicStart(0, 0), conicControl(1, 2), conicEnd(2, 0);

RationalQuadraticBezier<Vector2> conic(float weight) {
    return *RationalQuadraticBezier<Vector2>::make(conicStart, conicControl, conicEnd, weight);
}

std::optional<RationalQuadraticBezier<Vector2>> arc(float sweepDegrees, float radius = 1) {
    return RationalQuadraticBezier<Vector2>::circularArc(Vector2(0, 0), radius, Angle::degrees(0), Angle::degrees(sweepDegrees));
}

} // namespace

TEST(rationalWeightOneIsThePolynomialQuadratic) {
    auto curve = conic(1);
    EXPECT(isClose(curve.point(0.5f), Vector2(1, 1)));
    EXPECT(isClose(curve.point(0.25f), Vector2(0.5f, 0.75f)));
    for (int step = 0; step <= 100; step += 1) {
        float t = static_cast<float>(step) / 100;
        float u = 1 - t;
        Vector2 polynomial = conicStart * (u * u) + conicControl * (2 * u * t) + conicEnd * (t * t);
        EXPECT(isClose(curve.point(t), polynomial, 1e-6f));
    }
}

TEST(rationalWeightTwoPullsTowardTheControlPoint) {
    EXPECT(isClose(conic(2).point(0.5f), Vector2(1, 4.0f / 3)));
    EXPECT(isClose(conic(0.5f).point(0.5f), Vector2(1, 2.0f / 3)));
}

TEST(rationalEndPointsAreExactAndTheDerivativeIsTheTangent) {
    auto curve = conic(0.7f);
    EXPECT(curve.point(0) == conicStart);
    EXPECT(curve.point(1) == conicEnd);
    EXPECT((curve.parameterRange() == ParameterRange{0, 1}));
    EXPECT(isClose(curve.firstDerivative(0), Vector2(1.4f, 2.8f), 4e-7f));
    EXPECT(isClose(curve.firstDerivative(1), Vector2(1.4f, -2.8f), 4e-7f));
    for (float parameter : {0.1f, 0.4f, 0.5f, 0.8f}) {
        float step = 1e-3f;
        Vector2 difference = (curve.point(parameter + step) - curve.point(parameter - step)) / (2 * step);
        EXPECT(isClose(curve.firstDerivative(parameter), difference, 1.5e-4f));
    }
}

TEST(rationalConicTypeFollowsTheWeight) {
    EXPECT(conic(0.5f).conicType() == ConicType::ellipse);
    EXPECT(conic(0.999f).conicType() == ConicType::ellipse);
    EXPECT(conic(1).conicType() == ConicType::parabola);
    EXPECT(conic(1.001f).conicType() == ConicType::hyperbola);
    EXPECT(conic(5).conicType() == ConicType::hyperbola);
}

TEST(rationalParabolaSatisfiesItsImplicitEquation) {
    auto parabola = conic(1);
    for (int step = 0; step <= 20; step += 1) {
        Vector2 point = parabola.point(static_cast<float>(step) / 20);
        EXPECT(std::fabs(point.y - point.x * (2 - point.x)) < 4e-7f);
    }
}

TEST(rationalACircleArcIsExact) {
    Vector2 centre(1, -2);
    auto quarter = RationalQuadraticBezier<Vector2>::circularArc(centre, 2, Angle::degrees(0), Angle::degrees(90));
    EXPECT(quarter.has_value());
    EXPECT(isClose(quarter->start, centre + Vector2(2, 0)));
    EXPECT(isClose(quarter->control, centre + Vector2(2, 2), 4e-7f));
    EXPECT(isClose(quarter->end, centre + Vector2(0, 2), 4e-7f));
    EXPECT(isApproximatelyEqual(quarter->controlWeight, 0.70710678f, 1e-7f));
    EXPECT(quarter->conicType() == ConicType::ellipse);
    EXPECT(isClose(quarter->point(0.5f), centre + Vector2(1.4142135f, 1.4142135f), 8e-7f));
    for (float degrees : {30.0f, 90.0f, 150.0f, -90.0f, -120.0f}) {
        auto piece = RationalQuadraticBezier<Vector2>::circularArc(centre, 2, Angle::degrees(20), Angle::degrees(degrees));
        EXPECT(piece.has_value());
        float largestError = 0;
        for (int step = 0; step <= 500; step += 1) {
            largestError = largerOf(largestError, std::fabs(piece->point(static_cast<float>(step) / 500).distance(centre) - 2));
        }
        EXPECT(largestError < 2.2e-6f);
    }
}

TEST(rationalCircleArcsRefuseBadInput) {
    EXPECT(!arc(0).has_value());
    EXPECT(!arc(180).has_value());
    EXPECT(!arc(-180).has_value());
    EXPECT(!arc(270).has_value());
    EXPECT(!arc(notANumber).has_value());
    EXPECT(!arc(90, 0).has_value());
    EXPECT(!arc(90, -1).has_value());
    EXPECT(!arc(90, notANumber).has_value());
    EXPECT(arc(179).has_value());
    using Conic = RationalQuadraticBezier<Vector2>;
    EXPECT(!Conic::make(Vector2(0, 0), Vector2(1, 1), Vector2(2, 0), 0).has_value());
    EXPECT(!Conic::make(Vector2(0, 0), Vector2(1, 1), Vector2(2, 0), -1).has_value());
    EXPECT(!Conic::make(Vector2(0, 0), Vector2(1, 1), Vector2(2, 0), notANumber).has_value());
    EXPECT(!Conic::make(Vector2(0, 0), Vector2(1, notANumber), Vector2(2, 0), 1).has_value());
}

TEST(rationalArcLengthOfAQuarterCircle) {
    auto quarter = arc(90, 3);
    EXPECT(quarter.has_value());
    EXPECT(isApproximatelyEqual(quarter->totalLength(), 3 * swiftPi / 2, 3e-6f));
}

TEST(rationalScaleAndPositionDoNotMatter) {
    struct Case {
        float scale;
        Vector2 offset;
    };
    for (Case testCase : {Case{1e-3f, Vector2(0, 0)}, Case{1e3f, Vector2(0, 0)}, Case{1, Vector2(1000, 1000)}}) {
        auto piece = RationalQuadraticBezier<Vector2>::circularArc(
            testCase.offset, testCase.scale, Angle::degrees(10), Angle::degrees(100));
        EXPECT(piece.has_value());
        float largestError = 0;
        for (int step = 0; step <= 200; step += 1) {
            Vector2 point = piece->point(static_cast<float>(step) / 200);
            largestError = largerOf(largestError, std::fabs((point - testCase.offset).length() - testCase.scale));
        }
        EXPECT(largestError <= 1e-6f * testCase.scale + (ulp(testCase.offset.x) + ulp(testCase.offset.y)) * 2);
    }
}

TEST(rationalWorksInThreeDimensions) {
    auto curve = RationalQuadraticBezier<Vector3>::make(Vector3(0, 0, 0), Vector3(1, 2, 3), Vector3(2, 0, 0), 1);
    EXPECT(curve.has_value());
    EXPECT(isClose(curve->point(0.5f), Vector3(1, 1, 1.5f)));
}

TEST(rationalAnArcFarFromTheOriginIsAccurateToTheRounding) {
    Vector2 centre(1e6f, 1e6f);
    auto piece = RationalQuadraticBezier<Vector2>::circularArc(centre, 1, Angle::degrees(10), Angle::degrees(100));
    EXPECT(piece.has_value());
    double largestError = 0;
    for (int step = 0; step <= 500; step += 1) {
        Vector2 point = piece->point(static_cast<float>(step) / 500);
        double offsetX = static_cast<double>(point.x) - 1e6;
        double offsetY = static_cast<double>(point.y) - 1e6;
        largestError = std::max(largestError, std::fabs(std::sqrt(offsetX * offsetX + offsetY * offsetY) - 1));
    }
    EXPECT(largestError < 0.0625);
}

TEST(rationalATinySweepIsAParabolaThatStaysOnTheCircle) {
    auto tiny = RationalQuadraticBezier<Vector2>::circularArc(Vector2(0, 0), 1, Angle::degrees(10), Angle::degrees(0.01f));
    EXPECT(tiny.has_value());
    EXPECT(tiny->controlWeight == 1);
    EXPECT(tiny->conicType() == ConicType::parabola);
    float largestError = 0;
    for (int step = 0; step <= 20; step += 1) {
        largestError = largerOf(largestError, std::fabs(tiny->point(static_cast<float>(step) / 20).length() - 1));
    }
    EXPECT(largestError < 2e-7f);
    auto small = RationalQuadraticBezier<Vector2>::circularArc(Vector2(0, 0), 1, Angle::degrees(10), Angle::degrees(1));
    EXPECT(small.has_value());
    EXPECT(small->conicType() == ConicType::ellipse);
}

TEST(rationalAWeightedControlThatOverflowsIsRefused) {
    using Conic = RationalQuadraticBezier<Vector2>;
    EXPECT(!Conic::make(Vector2(0, 0), Vector2(1e20f, 1e20f), Vector2(2, 0), 1e30f).has_value());
    EXPECT(!Conic::make(Vector2(-3e38f, 0), Vector2(0, 1), Vector2(3e38f, 0), 1).has_value());
    EXPECT(Conic::make(Vector2(0, 0), Vector2(1e20f, 1e20f), Vector2(2, 0), 1e15f).has_value());
}

TEST(rationalMatchesSwiftBitForBit) {
    auto piece = RationalQuadraticBezier<Vector2>::circularArc(Vector2(1, -2), 2, Angle::degrees(20), Angle::degrees(100));
    EXPECT(piece.has_value());
    EXPECT(sameBits(piece->start, 0x403847d9, 0xbfa8715e));
    EXPECT(sameBits(piece->control, 0x40041b7d, 0x3f6c7e70));
    EXPECT(sameBits(piece->end, 0xb4000000, 0xbe8930a4));
    EXPECT(sameBits(piece->controlWeight, 0x3f248dbb));
    EXPECT(sameBits(piece->point(0.37f), 0x400740dc, 0xbead5398));
    EXPECT(sameBits(piece->firstDerivative(0.37f), 0xc04376f1, 0x4002fa8a));
    EXPECT(sameBits(piece->totalLength(), 0x405f66f3));
    auto spatial = RationalQuadraticBezier<Vector3>::make(Vector3(0, 0, 0), Vector3(1, 2, 3), Vector3(2, 0, 0), 1.7f);
    EXPECT(spatial.has_value());
    EXPECT(sameBits(spatial->point(0.61f), 0x3f951fd4, 0x3f9b553d, 0x3fe8ffdd));
    EXPECT(sameBits(spatial->totalLength(), 0x40a2e258));
}

// MARK: B-spline

namespace {

using Spline = BSpline<Vector2>;

const std::vector<Vector2> bezierControls = {Vector2(0, 0), Vector2(1, 3), Vector2(3, 3), Vector2(4, 0)};
const std::vector<float> bezierKnots = {0, 0, 0, 0, 1, 1, 1, 1};

/// Six control points, clamped uniform cubic knots [0,0,0,0,1,2,3,3,3,3].
Spline sixPointCubic(float scale = 1, Vector2 offset = Vector2(0, 0)) {
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(1, 2), Vector2(2.5f, 2.2f), Vector2(3.5f, 0.3f), Vector2(5, 0.5f), Vector2(6, 2.5f)};
    std::vector<Vector2> moved;
    for (Vector2 point : points) moved.push_back(point * scale + offset);
    return *Spline::make(3, moved, *Spline::clampedUniformKnots(3, 6));
}

std::optional<std::string> problem(
    int degree = 3, std::vector<Vector2> points = bezierControls, std::vector<float> knots = bezierKnots,
    std::optional<std::vector<float>> weights = std::nullopt) {
    return Spline::problem(degree, points, knots, weights);
}

int count(const std::vector<float>& values, float wanted) {
    int total = 0;
    for (float value : values) total += value == wanted ? 1 : 0;
    return total;
}

} // namespace

TEST(bSplineDegreeOneIsThePolyline) {
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(2, 0), Vector2(2, 2)};
    auto knots = Spline::clampedUniformKnots(1, 3);
    EXPECT(knots.has_value());
    EXPECT((*knots == std::vector<float>{0, 0, 1, 2, 2}));
    auto spline = Spline::make(1, points, *knots);
    EXPECT(spline.has_value());
    EXPECT((spline->parameterRange() == ParameterRange{0, 2}));
    EXPECT(spline->point(0) == Vector2(0, 0));
    EXPECT(isClose(spline->point(0.5f), Vector2(1, 0)));
    EXPECT(isClose(spline->point(1), Vector2(2, 0)));
    EXPECT(isClose(spline->point(1.5f), Vector2(2, 1)));
    EXPECT(spline->point(2) == Vector2(2, 2));
    EXPECT(isClose(spline->firstDerivative(0.5f), Vector2(2, 0)));
    EXPECT(isClose(spline->firstDerivative(1.5f), Vector2(0, 2)));
}

TEST(bSplineClampedKnotsHelper) {
    EXPECT((*Spline::clampedUniformKnots(3, 4) == std::vector<float>{0, 0, 0, 0, 1, 1, 1, 1}));
    EXPECT((*Spline::clampedUniformKnots(3, 6) == std::vector<float>{0, 0, 0, 0, 1, 2, 3, 3, 3, 3}));
    EXPECT((*Spline::clampedUniformKnots(2, 5) == std::vector<float>{0, 0, 0, 1, 2, 3, 3, 3}));
    EXPECT(!Spline::clampedUniformKnots(3, 3).has_value());
    EXPECT(!Spline::clampedUniformKnots(0, 3).has_value());
    EXPECT(!Spline::clampedUniformKnots(99, 300).has_value());
}

TEST(bSplineAClampedCubicWithFourPointsIsTheBezierCurve) {
    auto spline = Spline::make(3, bezierControls, bezierKnots);
    EXPECT(spline.has_value());
    CubicBezier<Vector2> bezier(bezierControls[0], bezierControls[1], bezierControls[2], bezierControls[3]);
    for (int step = 0; step <= 100; step += 1) {
        float parameter = static_cast<float>(step) / 100;
        EXPECT(isClose(spline->point(parameter), bezier.point(parameter), 1e-6f));
        EXPECT(isClose(spline->firstDerivative(parameter), bezier.firstDerivative(parameter), 4e-6f));
    }
    EXPECT(isClose(spline->point(1.25f), bezier.point(1.25f), 2e-6f));
    EXPECT(isClose(spline->point(-0.25f), bezier.point(-0.25f), 2e-6f));
}

TEST(bSplineBasisFunctionsOfAUniformQuadraticAndCubic) {
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(1, 4), Vector2(2, 0)};
    auto quadratic = Spline::make(2, points, {0, 1, 2, 3, 4, 5});
    EXPECT(quadratic.has_value());
    BasisFunctionValues basis = quadratic->basisFunctions(2.5f);
    EXPECT(basis.firstIndex == 0);
    EXPECT(basis.values.size() == 3);
    EXPECT(isApproximatelyEqual(basis.values[0], 0.125f) && isApproximatelyEqual(basis.values[1], 0.75f)
           && isApproximatelyEqual(basis.values[2], 0.125f));
    EXPECT(isClose(quadratic->point(2.5f), Vector2(1, 3)));

    auto cubic = Spline::make(3, bezierControls, {0, 1, 2, 3, 4, 5, 6, 7});
    EXPECT(cubic.has_value());
    BasisFunctionValues cubicBasis = cubic->basisFunctions(3.5f);
    EXPECT(cubicBasis.values.size() == 4);
    float expected[] = {1.0f / 48, 23.0f / 48, 23.0f / 48, 1.0f / 48};
    for (size_t index = 0; index < 4; index += 1) EXPECT(isApproximatelyEqual(cubicBasis.values[index], expected[index]));
}

TEST(bSplineBasisFunctionsAddUpToOne) {
    Spline spline = sixPointCubic();
    ParameterRange range = spline.parameterRange();
    for (int step = 0; step <= 300; step += 1) {
        float parameter = linearBlend(range.lowerBound, range.upperBound, static_cast<float>(step) / 300);
        BasisFunctionValues basis = spline.basisFunctions(parameter);
        float total = 0;
        Vector2 sum(0, 0);
        for (size_t offset = 0; offset < basis.values.size(); offset += 1) {
            EXPECT(basis.values[offset] >= -1e-7f);
            total += basis.values[offset];
            sum += spline.controlPoints[static_cast<size_t>(basis.firstIndex) + offset] * basis.values[offset];
        }
        EXPECT(isApproximatelyEqual(total, 1, 3e-7f));
        EXPECT(isClose(sum, spline.point(parameter), 3e-6f));
    }
}

TEST(bSplineSpansAreFoundByBinarySearch) {
    Spline spline = sixPointCubic();
    EXPECT(spline.span(0) == 3);
    EXPECT(spline.span(0.99f) == 3);
    EXPECT(spline.span(1) == 4);
    EXPECT(spline.span(2.5f) == 5);
    EXPECT(spline.span(3) == 5);
    EXPECT(spline.span(-1) == 3);
    EXPECT(spline.span(7) == 5);
}

TEST(bSplineClampedEndPointsAreExact) {
    std::vector<Vector2> points = {Vector2(0.1f, 0.7f), Vector2(1.3f, 2.9f), Vector2(2.7f, -1.1f), Vector2(4.9f, 0.3f), Vector2(5.2f, 1.7f)};
    auto spline = Spline::make(3, points, *Spline::clampedUniformKnots(3, 5));
    EXPECT(spline.has_value());
    EXPECT(spline->point(spline->parameterRange().lowerBound) == points[0]);
    EXPECT(spline->point(spline->parameterRange().upperBound) == points[4]);
    auto rational = Spline::make(3, points, spline->knots, std::vector<float>{1, 0.3f, 2.2f, 0.9f, 1.7f});
    EXPECT(rational.has_value());
    EXPECT(rational->point(0) == points[0]);
    EXPECT(rational->point(2) == points[4]);
}

TEST(bSplineInsertingAKnotKeepsTheCurve) {
    Spline spline = sixPointCubic();
    Spline refined = spline;
    for (int expectedMultiplicity = 1; expectedMultiplicity <= 3; expectedMultiplicity += 1) {
        auto next = refined.insertingKnot(1.37f);
        EXPECT(next.has_value());
        if (!next) return;
        refined = *next;
        EXPECT(count(refined.knots, 1.37f) == expectedMultiplicity);
        EXPECT(refined.controlPoints.size() == spline.controlPoints.size() + static_cast<size_t>(expectedMultiplicity));
        EXPECT(refined.knots.size() == refined.controlPoints.size() + 4);
        for (int step = 0; step <= 200; step += 1) {
            float parameter = static_cast<float>(step) * 3 / 200;
            EXPECT(isClose(refined.point(parameter), spline.point(parameter), 1.5e-6f));
        }
    }
    EXPECT(!refined.insertingKnot(1.37f).has_value());
    EXPECT(!spline.insertingKnot(0).has_value());
    EXPECT(!spline.insertingKnot(3).has_value());
    EXPECT(!spline.insertingKnot(-1).has_value());
    EXPECT(!spline.insertingKnot(notANumber).has_value());
    auto doubled = spline.insertingKnot(2);
    EXPECT(doubled.has_value());
    EXPECT(count(doubled->knots, 2) == 2);
    EXPECT(isClose(doubled->point(2.3f), spline.point(2.3f), 3e-6f));
}

TEST(bSplineInsertingAKnotKeepsARationalCurve) {
    auto circle = Spline::circle(Vector2(1, 2), 3);
    EXPECT(circle.has_value());
    auto once = circle->insertingKnot(0.1f);
    EXPECT(once.has_value());
    auto refined = once->insertingKnot(0.6f);
    EXPECT(refined.has_value());
    EXPECT(refined->controlPoints.size() == 11);
    EXPECT(refined->weights.has_value() && refined->weights->size() == 11);
    for (int step = 0; step <= 200; step += 1) {
        float parameter = static_cast<float>(step) / 200;
        EXPECT(isClose(refined->point(parameter), circle->point(parameter), 3e-6f));
    }
}

TEST(bSplineBezierPiecesEqualTheSpline) {
    Spline spline = sixPointCubic();
    auto pieces = spline.cubicBezierPieces();
    EXPECT(pieces.has_value());
    if (!pieces) return;
    EXPECT(pieces->size() == 3);
    EXPECT((*pieces)[0].start == spline.controlPoints[0]);
    EXPECT((*pieces)[2].end == spline.controlPoints[5]);
    for (size_t index = 1; index < pieces->size(); index += 1) EXPECT((*pieces)[index - 1].end == (*pieces)[index].start);
    for (size_t index = 0; index < pieces->size(); index += 1) {
        for (int step = 0; step <= 50; step += 1) {
            float local = static_cast<float>(step) / 50;
            EXPECT(isClose((*pieces)[index].point(local), spline.point(static_cast<float>(index) + local), 3e-6f));
        }
    }
    // The same control points as repeated knot insertion (Boehm), read in groups of three.
    Spline inserted = spline;
    for (float knot : {1.0f, 2.0f}) {
        while (count(inserted.knots, knot) < 3) inserted = *inserted.insertingKnot(knot);
    }
    EXPECT(inserted.controlPoints.size() == 10);
    for (size_t index = 0; index < pieces->size(); index += 1) {
        const CubicBezier<Vector2>& piece = (*pieces)[index];
        Vector2 controls[] = {piece.start, piece.startHandle, piece.endHandle, piece.end};
        for (size_t offset = 0; offset < 4; offset += 1) {
            EXPECT(isClose(controls[offset], inserted.controlPoints[3 * index + offset], 1e-6f));
        }
    }
}

TEST(bSplineBezierPiecesOfASingleSpanAndWithRepeatedKnots) {
    auto single = Spline::make(3, bezierControls, bezierKnots);
    EXPECT(single.has_value());
    auto pieces = single->cubicBezierPieces();
    EXPECT(pieces.has_value() && pieces->size() == 1);
    EXPECT(isClose((*pieces)[0].startHandle, bezierControls[1]));
    EXPECT(isClose((*pieces)[0].endHandle, bezierControls[2]));
    std::vector<Vector2> points;
    for (int index = 0; index < 7; index += 1) points.push_back(Vector2(static_cast<float>(index), static_cast<float>(index * index % 5)));
    auto cornered = Spline::make(3, points, {0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2});
    EXPECT(cornered.has_value());
    EXPECT(cornered->cubicBezierPieces().has_value() && cornered->cubicBezierPieces()->size() == 2);
    EXPECT(isClose(cornered->point(1), points[3], 1e-5f));
}

TEST(bSplineBezierPiecesNeedAClampedNonRationalCubic) {
    auto quadratic = Spline::make(2, {bezierControls[0], bezierControls[1], bezierControls[2]}, {0, 0, 0, 1, 1, 1});
    EXPECT(quadratic.has_value() && !quadratic->cubicBezierPieces().has_value());
    auto rational = Spline::make(3, bezierControls, bezierKnots, std::vector<float>{1, 2, 1, 1});
    EXPECT(rational.has_value() && !rational->cubicBezierPieces().has_value());
    auto unclamped = Spline::make(3, bezierControls, {0, 1, 2, 3, 4, 5, 6, 7});
    EXPECT(unclamped.has_value() && !unclamped->cubicBezierPieces().has_value());
}

TEST(bSplineDerivativeMatchesDifferences) {
    Spline spline = sixPointCubic();
    for (float parameter : {0.2f, 0.9f, 1.0f, 1.6f, 2.4f, 2.9f}) {
        float step = 1e-2f;
        Vector2 difference = (spline.point(parameter + step) - spline.point(parameter - step)) / (2 * step);
        EXPECT(isClose(spline.firstDerivative(parameter), difference, 5e-4f));
    }
}

TEST(bSplineWeightsPullTheCurveTowardTheirControlPoint) {
    float previous = infinity;
    for (float weight : {0.25f, 1.0f, 4.0f, 16.0f}) {
        auto spline = Spline::make(3, bezierControls, bezierKnots, std::vector<float>{1, weight, 1, 1});
        EXPECT(spline.has_value());
        float distance = spline->point(0.3f).distance(bezierControls[1]);
        EXPECT(distance < previous);
        previous = distance;
    }
    Spline ordinary = sixPointCubic();
    auto ones = Spline::make(3, ordinary.controlPoints, ordinary.knots, std::vector<float>{1, 1, 1, 1, 1, 1});
    EXPECT(ones.has_value());
    for (int step = 0; step <= 60; step += 1) {
        float parameter = static_cast<float>(step) / 20;
        EXPECT(ones->point(parameter) == ordinary.point(parameter));
    }
    std::vector<Vector2> triple = {Vector2(0, 0), Vector2(1, 2), Vector2(2, 0)};
    auto nurbs = Spline::make(2, triple, {0, 0, 0, 1, 1, 1}, std::vector<float>{1, 0.6f, 1});
    auto piece = RationalQuadraticBezier<Vector2>::make(triple[0], triple[1], triple[2], 0.6f);
    EXPECT(nurbs.has_value() && piece.has_value());
    for (int step = 0; step <= 50; step += 1) {
        float parameter = static_cast<float>(step) / 50;
        EXPECT(isClose(nurbs->point(parameter), piece->point(parameter), 1e-6f));
    }
}

TEST(bSplineTheNurbsCircleHasUnitDistanceEverywhere) {
    Vector2 centre(3, -2);
    float radius = 2.5f;
    auto circle = Spline::circle(centre, radius);
    EXPECT(circle.has_value());
    EXPECT(circle->degree == 2 && circle->controlPoints.size() == 9);
    EXPECT(circle->point(0) == circle->point(1));
    EXPECT(circle->point(0) == centre + Vector2(radius, 0));
    float largestError = 0;
    for (int step = 0; step <= 2000; step += 1) {
        Vector2 point = circle->point(static_cast<float>(step) / 2000);
        largestError = largerOf(largestError, std::fabs(point.distance(centre) - radius));
    }
    EXPECT(largestError < 3e-6f);
    EXPECT(isClose(circle->point(0.25f), centre + Vector2(0, radius), 1e-6f));
    EXPECT(isClose(circle->point(0.5f), centre + Vector2(-radius, 0), 1e-6f));
    for (int step = 1; step < 200; step += 1) {
        float parameter = static_cast<float>(step) / 200;
        Vector2 radial = circle->point(parameter) - centre;
        Vector2 tangent = circle->firstDerivative(parameter);
        EXPECT(tangent.length() > 1);
        EXPECT(std::fabs(tangent.normalized().dot(radial.normalized())) < 1.2e-6f);
    }
}

TEST(bSplineTheCircleRefusesBadInput) {
    EXPECT(!Spline::circle(Vector2(0, 0), 0).has_value());
    EXPECT(!Spline::circle(Vector2(0, 0), -1).has_value());
    EXPECT(!Spline::circle(Vector2(0, 0), notANumber).has_value());
    EXPECT(!Spline::circle(Vector2(infinity, 0), 1).has_value());
}

TEST(bSplineBadInputsAreReportedNotTrapped) {
    EXPECT(!problem().has_value());
    EXPECT(problem(0).has_value());
    EXPECT(problem(17).has_value());
    EXPECT(problem(3, {bezierControls[0], bezierControls[1], bezierControls[2]}).has_value());
    EXPECT(problem(3, {Vector2(0, 0), Vector2(notANumber, 1), Vector2(2, 2), Vector2(3, 3)}).has_value());
    EXPECT(problem(3, {Vector2(0, 0), Vector2(infinity, 1), Vector2(2, 2), Vector2(3, 3)}).has_value());
    EXPECT(problem(3, bezierControls, {0, 0, 0, 1, 1, 1, 1}).has_value());
    EXPECT(problem(3, bezierControls, {0, 0, 0, 0, 1, 1, 1, notANumber}).has_value());
    EXPECT(problem(3, bezierControls, {0, 0, 0, 0, 1, 1, 1, infinity}).has_value());
    EXPECT(problem(3, bezierControls, {0, 0, 0, 1, 0, 1, 1, 1}).has_value());
    EXPECT(problem(3, bezierControls, {0, 0, 0, 0, 0, 1, 1, 1}).has_value());
    EXPECT(problem(3, bezierControls, {1, 1, 1, 1, 1, 1, 1, 1}).has_value());
    EXPECT(problem(3, bezierControls, bezierKnots, std::vector<float>{1, 1, 1}).has_value());
    EXPECT(problem(3, bezierControls, bezierKnots, std::vector<float>{1, 0, 1, 1}).has_value());
    EXPECT(problem(3, bezierControls, bezierKnots, std::vector<float>{1, -1, 1, 1}).has_value());
    EXPECT(problem(3, bezierControls, bezierKnots, std::vector<float>{1, notANumber, 1, 1}).has_value());
    EXPECT(!problem(3, bezierControls, bezierKnots, std::vector<float>{1, 2, 1, 1}).has_value());
    std::vector<Vector2> eight;
    for (int index = 0; index < 8; index += 1) eight.push_back(Vector2(static_cast<float>(index), 0));
    EXPECT(Spline::problem(3, eight, {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2}, std::nullopt).has_value());
    EXPECT(Spline::problem(3, eight, {0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2}, std::nullopt).has_value());
    EXPECT(!Spline::make(3, bezierControls, {0, 0, 0, 1, 1, 1, 1, 1}).has_value());
    auto spline = Spline::make(3, bezierControls, bezierKnots);
    EXPECT(spline.has_value());
    EXPECT(std::isnan(spline->point(notANumber).x));
    EXPECT(std::isnan(spline->firstDerivative(notANumber).y));
}

TEST(bSplineProblemSentencesAreSwiftsWords) {
    EXPECT_EQUAL(*problem(0), std::string("The degree must be between 1 and 16, not 0."));
    EXPECT_EQUAL(*problem(3, {bezierControls[0], bezierControls[1]}),
                 std::string("A spline of degree 3 needs at least 4 control points, not 2."));
    EXPECT_EQUAL(*problem(3, bezierControls, {0, 0, 0, 1, 1, 1, 1}),
                 std::string("The knot vector needs control points + degree + 1 = 8 knots, not 7."));
    EXPECT_EQUAL(*problem(3, bezierControls, {0, 0, 0, 1, 0, 1, 1, 1}),
                 std::string("The knots must not decrease, but knot 4 is smaller than knot 3."));
    EXPECT_EQUAL(*problem(3, bezierControls, {1, 1, 1, 1, 1, 1, 1, 1}),
                 std::string("The domain is empty: knot 3 must be smaller than knot 4."));
    EXPECT_EQUAL(
        *problem(1, {Vector2(0, 0), Vector2(1, 0), Vector2(2, 0), Vector2(3, 0), Vector2(4, 0)}, {0, 0, 1, 1, 1, 2, 2}),
        std::string("The knot 1.0 is repeated 3 times; the most allowed is 1."));
    EXPECT_EQUAL(*problem(3, bezierControls, bezierKnots, std::vector<float>{1, 1, 1}),
                 std::string("There must be one weight per control point (4), not 3."));
}

TEST(bSplineScaleAndPositionOfTheCurveAndOfTheKnotsDoNotMatter) {
    Spline reference = sixPointCubic();
    struct Space {
        float scale;
        Vector2 offset;
    };
    struct Knots {
        float scale;
        float offset;
    };
    for (Space space : {Space{1e-3f, Vector2(0, 0)}, Space{1e3f, Vector2(0, 0)}, Space{1, Vector2(1000, 1000)}}) {
        for (Knots knotMap : {Knots{1, 0}, Knots{1e-3f, 0}, Knots{1e3f, 1000}, Knots{1, -5000}}) {
            Spline moved = sixPointCubic(space.scale, space.offset);
            std::vector<float> newKnots;
            for (float knot : moved.knots) newKnots.push_back(knot * knotMap.scale + knotMap.offset);
            auto spline = Spline::make(3, moved.controlPoints, newKnots);
            EXPECT(spline.has_value());
            if (!spline) continue;
            float slack = (ulp(space.offset.x) + ulp(space.offset.y)) * 2;
            for (int step = 0; step <= 100; step += 1) {
                float parameter = static_cast<float>(step) * 3 / 100 * knotMap.scale + knotMap.offset;
                float referenceParameter = static_cast<float>((static_cast<double>(parameter) - knotMap.offset) / knotMap.scale);
                Vector2 expected = reference.point(referenceParameter) * space.scale + space.offset;
                EXPECT(isClose(spline->point(parameter), expected, 4e-6f * space.scale + slack));
            }
        }
    }
}

TEST(bSplineWorksInThreeDimensionsToo) {
    std::vector<Vector3> points = {Vector3(0, 0, 0), Vector3(1, 2, 3), Vector3(3, 2, 1), Vector3(4, 0, 4)};
    auto spline = BSpline<Vector3>::make(3, points, bezierKnots);
    EXPECT(spline.has_value());
    CubicBezier<Vector3> bezier(points[0], points[1], points[2], points[3]);
    for (int step = 0; step <= 20; step += 1) {
        float parameter = static_cast<float>(step) / 20;
        EXPECT(isClose(spline->point(parameter), bezier.point(parameter), 3e-6f));
    }
}

TEST(bSplineAnEmptyFirstSpanIsSkippedBelowTheDomain) {
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(1, 1), Vector2(2, 0), Vector2(3, 1), Vector2(4, 0)};
    auto spline = Spline::make(2, points, {0, 0, 1, 1, 2, 3, 3, 3});
    EXPECT(spline.has_value());
    EXPECT((spline->parameterRange() == ParameterRange{1, 3}));
    EXPECT(spline->span(0.5f) == 3);
    EXPECT(spline->basisFunctions(0.5f).firstIndex == 1);
    Vector2 extrapolated = spline->point(1) * 3 - spline->point(1.5f) * 3 + spline->point(2);
    EXPECT(spline->point(0.5f).isFinite());
    EXPECT(isClose(spline->point(0.5f), extrapolated, 1e-5f));
    EXPECT(spline->firstDerivative(0.5f).isFinite());
}

TEST(bSplineKnotsWhoseRangeOverflowsAreRefused) {
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(1, 1)};
    EXPECT(!Spline::make(1, points, {-3e38f, -3e38f, 3e38f, 3e38f}).has_value());
    auto half = Spline::make(1, points, {-1.5e38f, -1.5e38f, 1.5e38f, 1.5e38f});
    EXPECT(half.has_value());
    EXPECT(isClose(half->point(0), Vector2(0.5f, 0.5f)));
}

TEST(bSplineAWeightThatOverflowsAPointIsRefused) {
    std::vector<Vector2> points = {Vector2(0, 0), Vector2(1e20f, 1e20f), Vector2(2, 0)};
    std::vector<float> knots = {0, 0, 0, 1, 1, 1};
    EXPECT(!Spline::make(2, points, knots, std::vector<float>{1, 1e30f, 1}).has_value());
    EXPECT(Spline::problem(2, points, knots, std::vector<float>{1, 1e30f, 1}).has_value());
    auto heavy = Spline::make(2, points, knots, std::vector<float>{1, 1e15f, 1});
    EXPECT(heavy.has_value());
    EXPECT(heavy->point(0.5f).isFinite());
}

TEST(bSplineACircleFarFromTheOriginIsAccurateToTheRounding) {
    auto circle = Spline::circle(Vector2(1e6f, 1e6f), 1);
    EXPECT(circle.has_value());
    double largestError = 0;
    for (int step = 0; step <= 2000; step += 1) {
        Vector2 point = circle->point(static_cast<float>(step) / 2000);
        double offsetX = static_cast<double>(point.x) - 1e6;
        double offsetY = static_cast<double>(point.y) - 1e6;
        largestError = std::max(largestError, std::fabs(std::sqrt(offsetX * offsetX + offsetY * offsetY) - 1));
    }
    EXPECT(largestError < 0.0625);
}

TEST(bSplineMatchesSwiftBitForBit) {
    Spline spline = sixPointCubic();
    EXPECT(sameBits(spline.point(1.7f), 0x4051036a, 0x3f7c61c0));
    EXPECT(sameBits(spline.firstDerivative(1.7f), 0x3fae6669, 0xbfa18938));
    EXPECT(sameBits(spline.totalLength(), 0x4105a11d));
    auto circle = Spline::circle(Vector2(3, -2), 2.5f);
    EXPECT(circle.has_value());
    EXPECT(sameBits(circle->point(0.3f), 0x4010fd76, 0x3ec78143));
    EXPECT(sameBits(circle->firstDerivative(0.3f), 0xc16ea7c3, 0xc092b713));
    EXPECT(sameBits(circle->totalLength(), 0x417b53d1));
    auto inserted = spline.insertingKnot(1.37f);
    EXPECT(inserted.has_value());
    EXPECT(sameBits(inserted->controlPoints[2], 0x4001c290, 0x4008c49c));
    EXPECT(sameBits(inserted->controlPoints[3], 0x403d3a07, 0x3faa89e6));
    auto pieces = spline.cubicBezierPieces();
    EXPECT(pieces.has_value());
    EXPECT(sameBits((*pieces)[1].startHandle, 0x40355556, 0x3fc88888));
    EXPECT(sameBits((*pieces)[1].endHandle, 0x404aaaab, 0x3f6eeef0));
    EXPECT(sameBits((*pieces)[1].end, 0x406d5556, 0x3f2aaaaa));
    auto spatial = BSpline<Vector3>::make(
        3, {Vector3(0, 0, 0), Vector3(1, 2, 3), Vector3(3, 2, 1), Vector3(4, 0, 4), Vector3(5, 1, 2)},
        *BSpline<Vector3>::clampedUniformKnots(3, 5), std::vector<float>{1, 0.5f, 2, 1, 1.5f});
    EXPECT(spatial.has_value());
    EXPECT(sameBits(spatial->point(0.8f), 0x402fe593, 0x3fe3ec2e, 0x3fd1c13e));
    EXPECT(sameBits(spatial->totalLength(), 0x40e6dfd2));
}
