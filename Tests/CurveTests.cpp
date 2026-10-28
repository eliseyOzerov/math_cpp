// Ports of EditorTests/Math/Curves/ for the cubic Bézier, arc length and curve frames:
// CubicBezierTests, CubicBezier2DTests, CubicBezierAnalysisTests, CubicBezierArcTests,
// ArcLengthTests and CurveFrameTests (the Catmull-Rom, B-spline and rational cases come with
// those curves), plus bit-for-bit checks against values printed by the Swift engine.
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "../Curves/CubicBezier.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
// Swift's `Float.pi`: rounded toward zero, one step below the nearest float.
constexpr float swiftPi = 0x1.921fb4p1f;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// Whether the float has exactly these bits (the Swift reference values below were printed as bits).
bool sameBits(float value, std::uint32_t bits) { return std::bit_cast<std::uint32_t>(value) == bits; }

bool sameBits(Vector3 value, std::uint32_t x, std::uint32_t y, std::uint32_t z) {
    return sameBits(value.x, x) && sameBits(value.y, y) && sameBits(value.z, z);
}

using Curve3 = CubicBezier<Vector3>;
using Curve2 = CubicBezier<Vector2>;

/// An arch: up from the origin, across, and down to (3, 0, 0).
Curve3 arch3() { return Curve3(Vector3(0, 0, 0), Vector3(0, 3, 0), Vector3(3, 3, 0), Vector3(3, 0, 0)); }

/// A straight line from the origin to (3, 0, 0), with evenly spaced control points.
Curve3 line3() { return Curve3(Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0), Vector3(3, 0, 0)); }

/// A quarter circle of radius 1 from (1,0,0) to (0,1,0), using the handle length 0.5523.
Curve3 quarterCircle3() {
    return Curve3(Vector3(1, 0, 0), Vector3(1, 0.5523f, 0), Vector3(0.5523f, 1, 0), Vector3(0, 1, 0));
}

/// A lopsided arch: up from the origin, across, and down to (6, 0). Not symmetric, so a swapped
/// handle would show.
Curve2 arch2() { return Curve2(Vector2(0, 0), Vector2(0, 4), Vector2(4, 4), Vector2(6, 0)); }

/// The symmetric arch: x = 3t² − 2t³, y = 3t(1 − t).
Curve2 symmetricArch() { return Curve2(Vector2(0, 0), Vector2(0, 1), Vector2(1, 1), Vector2(1, 0)); }

/// An S: x = 4t³ − 6t² + 3t, y = 3t² − 2t³.
Curve2 sCurve() { return Curve2(Vector2(0, 0), Vector2(1, 0), Vector2(0, 1), Vector2(1, 1)); }

/// The parabola y = x², as a cubic: x = t, y = t².
Curve2 parabola() {
    return Curve2(Vector2(0, 0), Vector2(1.0f / 3, 0), Vector2(2.0f / 3, 1.0f / 3), Vector2(1, 1));
}

/// A quarter of the unit circle, from (1, 0) to (0, 1), counterclockwise.
Curve2 quarterCircle2() {
    return Curve2(Vector2(1, 0), Vector2(1, 0.5522848f), Vector2(0.5522848f, 1), Vector2(0, 1));
}

Curve2 translatedBy(const Curve2& curve, Vector2 offset) { return curve.translated(offset); }

Curve2 reversed(const Curve2& curve) { return Curve2(curve.end, curve.endHandle, curve.startHandle, curve.start); }

/// A curve whose four control points are drawn uniformly from the square ±`spread`.
Curve2 randomCurve(SeededRandom& generator, float spread) {
    auto point = [&] {
        float x = generator.nextFloat(-spread, spread);
        float y = generator.nextFloat(-spread, spread);
        return Vector2(x, y);
    };
    Vector2 start = point();
    Vector2 startHandle = point();
    Vector2 endHandle = point();
    Vector2 end = point();
    return Curve2(start, startHandle, endHandle, end);
}

/// A curve whose control points are on the grid of 1/1024 inside ±3, so adding a whole-number
/// offset below 8192 to them is exact in Float.
Curve2 randomGridCurve(SeededRandom& generator) {
    auto point = [&] {
        float x = static_cast<float>(generator.nextInt(-3072, 3072)) / 1024;
        float y = static_cast<float>(generator.nextInt(-3072, 3072)) / 1024;
        return Vector2(x, y);
    };
    Vector2 start = point();
    Vector2 startHandle = point();
    Vector2 endHandle = point();
    Vector2 end = point();
    return Curve2(start, startHandle, endHandle, end);
}

/// The inflections of the stored curve in Double: the sign changes in (0, 1) of
/// `−3(a×b)t² + 3(c×a)t + (c×b)`. With a margin, `false` when a root lies within that distance of
/// 0 or 1, or the discriminant is within it of zero (cases where the count depends on rounding).
bool exactInflections(const Curve2& curve, double margin, std::vector<double>& result) {
    struct Pair {
        double x;
        double y;
    };
    auto pair = [](Vector2 point) { return Pair{double(point.x), double(point.y)}; };
    auto minus = [](Pair first, Pair second) { return Pair{first.x - second.x, first.y - second.y}; };
    auto times = [](double factor, Pair value) { return Pair{factor * value.x, factor * value.y}; };
    auto plus = [](Pair first, Pair second) { return Pair{first.x + second.x, first.y + second.y}; };
    auto cross = [](Pair first, Pair second) { return first.x * second.y - first.y * second.x; };
    Pair toStartHandle = minus(pair(curve.startHandle), pair(curve.start));
    Pair toEndHandle = minus(pair(curve.endHandle), pair(curve.start));
    Pair toEnd = minus(pair(curve.end), pair(curve.start));
    Pair a = plus(minus(toEnd, times(3, toEndHandle)), times(3, toStartHandle));
    Pair b = minus(times(3, toEndHandle), times(6, toStartHandle));
    Pair c = times(3, toStartHandle);
    double quadratic = -3 * cross(a, b);
    double linear = 3 * cross(c, a);
    double constant = cross(c, b);
    std::vector<double> roots;
    if (quadratic == 0) {
        if (linear != 0) roots = {-constant / linear};
    } else {
        double discriminant = linear * linear - 4 * quadratic * constant;
        if (margin > 0 && std::fabs(discriminant) <= margin) return false;
        if (discriminant > 0) {
            double first = (-linear - std::sqrt(discriminant)) / (2 * quadratic);
            double second = (-linear + std::sqrt(discriminant)) / (2 * quadratic);
            roots = first < second ? std::vector<double>{first, second} : std::vector<double>{second, first};
        }
    }
    if (margin > 0) {
        for (double root : roots) {
            if (std::fabs(root) <= margin || std::fabs(root - 1) <= margin) return false;
        }
    }
    result.clear();
    for (double root : roots) {
        if (root > 0 && root < 1) result.push_back(root);
    }
    return true;
}

/// The fan area of the stored curve about `pivot` in Double: ½∫(B − pivot) × B′ dt by 3-point
/// Gauss–Legendre, exact for this degree-5 integrand.
double exactAreaSwept(const Curve2& curve, Vector2 pivot) {
    Vector2 controls[4] = {curve.start, curve.startHandle, curve.endHandle, curve.end};
    double px[4];
    double py[4];
    for (int index = 0; index < 4; index += 1) {
        px[index] = double(controls[index].x) - double(pivot.x);
        py[index] = double(controls[index].y) - double(pivot.y);
    }
    double root = std::sqrt(0.6);
    double parameters[3] = {0.5 - 0.5 * root, 0.5, 0.5 + 0.5 * root};
    double weights[3] = {5.0 / 18, 8.0 / 18, 5.0 / 18};
    double total = 0;
    for (int node = 0; node < 3; node += 1) {
        double t = parameters[node];
        double u = 1 - t;
        double x = u * u * u * px[0] + 3 * u * u * t * px[1] + 3 * u * t * t * px[2] + t * t * t * px[3];
        double y = u * u * u * py[0] + 3 * u * u * t * py[1] + 3 * u * t * t * py[2] + t * t * t * py[3];
        double vx = 3 * (u * u * (px[1] - px[0]) + 2 * u * t * (px[2] - px[1]) + t * t * (px[3] - px[2]));
        double vy = 3 * (u * u * (py[1] - py[0]) + 2 * u * t * (py[2] - py[1]) + t * t * (py[3] - py[2]));
        total += weights[node] * (x * vy - y * vx) / 2;
    }
    return total;
}

void expectOrthonormalAndRightHanded(const CurveFrame& frame) {
    EXPECT(isApproximatelyEqual(frame.tangent.length(), 1));
    EXPECT(isApproximatelyEqual(frame.normal.length(), 1));
    EXPECT(isApproximatelyEqual(frame.binormal.length(), 1));
    EXPECT(isApproximatelyEqual(frame.tangent.dot(frame.normal), 0));
    EXPECT(isApproximatelyEqual(frame.tangent.dot(frame.binormal), 0));
    EXPECT(isApproximatelyEqual(frame.normal.dot(frame.binormal), 0));
    EXPECT(isApproximatelyEqual(frame.tangent.cross(frame.normal), frame.binormal));
}

std::vector<float> evenGrid(int steps, float divisor) {
    std::vector<float> grid;
    for (int step = 0; step <= steps; step += 1) grid.push_back(static_cast<float>(step) / divisor);
    return grid;
}

} // namespace

// MARK: CubicBezier (3D)

TEST(bezier3DPointAtTheMiddleOfTheArch) {
    // Bernstein weights at 0.5: 1/8, 3/8, 3/8, 1/8.
    // x = 0 + 3/8·0 + 3/8·3 + 1/8·3 = 1.125 + 0.375 = 1.5
    // y = 0 + 3/8·3 + 3/8·3 + 0   = 2.25
    EXPECT(isApproximatelyEqual(arch3().point(0.5f), Vector3(1.5f, 2.25f, 0)));
}

TEST(bezier3DPointAtAQuarterOfTheArch) {
    // Weights at 0.25 (u = 0.75): u³ = 27/64, 3u²t = 27/64, 3ut² = 9/64, t³ = 1/64.
    // x = 27/64·0 + 9/64·3 + 1/64·3 = 30/64 = 0.46875
    // y = 27/64·3 + 9/64·3 = 108/64 = 1.6875
    EXPECT(isApproximatelyEqual(arch3().point(0.25f), Vector3(0.46875f, 1.6875f, 0)));
}

TEST(bezier3DFirstDerivativeOfTheArch) {
    // Steps: (0,3,0), (3,0,0), (0,−3,0). At 0.5 the quadratic weights are 1/4, 1/2, 1/4:
    // (0.25·0 + 0.5·3 + 0.25·0, 0.25·3 + 0 − 0.25·3) = (1.5, 0); times 3 = (4.5, 0).
    EXPECT(isApproximatelyEqual(arch3().firstDerivative(0.5f), Vector3(4.5f, 0, 0)));
    // At the ends it is 3 × the first and last step.
    EXPECT(isApproximatelyEqual(arch3().firstDerivative(0), Vector3(0, 9, 0)));
    EXPECT(isApproximatelyEqual(arch3().firstDerivative(1), Vector3(0, -9, 0)));
}

TEST(bezier3DSecondDerivativeOfTheArch) {
    // bend₀ = p2 − 2·p1 + p0 = (3,3,0) − (0,6,0) = (3,−3,0)
    // bend₁ = p3 − 2·p2 + p1 = (3,0,0) − (6,6,0) + (0,3,0) = (−3,−3,0)
    // At 0.5: 6·(½·bend₀ + ½·bend₁) = 3·(0,−6,0) = (0,−18,0). At 0 it is 6·bend₀ = (18, −18, 0).
    EXPECT(isApproximatelyEqual(arch3().secondDerivative(0.5f), Vector3(0, -18, 0)));
    EXPECT(isApproximatelyEqual(arch3().secondDerivative(0), Vector3(18, -18, 0)));
}

TEST(bezier3DEvenlySpacedCollinearPointsMoveAtConstantSpeed) {
    // x(t) = 3t, so the point is (3t, 0, 0), the velocity (3, 0, 0), the acceleration 0.
    for (float parameter : {0.0f, 0.1f, 0.5f, 0.9f, 1.0f}) {
        EXPECT(isApproximatelyEqual(line3().point(parameter), Vector3(3 * parameter, 0, 0)));
        EXPECT(isApproximatelyEqual(line3().firstDerivative(parameter), Vector3(3, 0, 0)));
        EXPECT(isApproximatelyEqual(line3().secondDerivative(parameter), Vector3()));
    }
}

TEST(bezier3DSplittingTheArchAtTheMiddle) {
    // a = mid(p0,p1) = (0,1.5), b = mid(p1,p2) = (1.5,3), c = mid(p2,p3) = (3,1.5)
    // d = mid(a,b) = (0.75,2.25), e = mid(b,c) = (2.25,2.25), f = mid(d,e) = (1.5,2.25)
    CubicBezierSplit<Vector3> halves = arch3().split(0.5f);
    EXPECT(halves.first.start == Vector3(0, 0, 0));
    EXPECT(isApproximatelyEqual(halves.first.startHandle, Vector3(0, 1.5f, 0)));
    EXPECT(isApproximatelyEqual(halves.first.endHandle, Vector3(0.75f, 2.25f, 0)));
    EXPECT(isApproximatelyEqual(halves.first.end, Vector3(1.5f, 2.25f, 0)));
    EXPECT(isApproximatelyEqual(halves.second.start, Vector3(1.5f, 2.25f, 0)));
    EXPECT(isApproximatelyEqual(halves.second.startHandle, Vector3(2.25f, 2.25f, 0)));
    EXPECT(isApproximatelyEqual(halves.second.endHandle, Vector3(3, 1.5f, 0)));
    EXPECT(halves.second.end == Vector3(3, 0, 0));
}

TEST(bezier3DPassesThroughStartAndEnd) {
    EXPECT(arch3().point(0) == arch3().start);
    EXPECT(arch3().point(1) == arch3().end);
}

TEST(bezier3DSplitHalvesReproduceTheOriginalCurve) {
    for (float splitParameter : {0.2f, 0.5f, 0.75f}) {
        CubicBezierSplit<Vector3> halves = arch3().split(splitParameter);
        for (float localParameter : evenGrid(10, 10)) {
            EXPECT(isApproximatelyEqual(
                halves.first.point(localParameter), arch3().point(localParameter * splitParameter)));
            float secondParameter = linearBlend(splitParameter, 1, localParameter);
            EXPECT(isApproximatelyEqual(halves.second.point(localParameter), arch3().point(secondParameter)));
        }
    }
}

TEST(bezier3DDerivativesMatchFiniteDifferencesOfTheCurve) {
    // A central difference (f(t+h) − f(t−h))/2h approximates the derivative with error ~h².
    float step = 1e-2f;
    for (float parameter : evenGrid(10, 10)) {
        if (parameter == 0 || parameter == 1) continue;
        Vector3 slope = (arch3().point(parameter + step) - arch3().point(parameter - step)) / (2 * step);
        EXPECT(isApproximatelyEqual(slope, arch3().firstDerivative(parameter), 2e-3f));
        Vector3 bend = (arch3().firstDerivative(parameter + step) - arch3().firstDerivative(parameter - step))
            / (2 * step);
        EXPECT(isApproximatelyEqual(bend, arch3().secondDerivative(parameter), 2e-3f));
    }
}

TEST(bezier3DOutsideTheRangeThePolynomialContinues) {
    // For the straight line, x(t) = 3t continues: t = 2 gives 6 and t = −1 gives −3.
    EXPECT(isApproximatelyEqual(line3().point(2), Vector3(6, 0, 0)));
    EXPECT(isApproximatelyEqual(line3().point(-1), Vector3(-3, 0, 0)));
}

TEST(bezier3DACurveWithAllPointsEqualStaysPut) {
    Vector3 point(1, 2, 3);
    Curve3 still(point, point, point, point);
    EXPECT(still.point(0.37f) == point);
    EXPECT(still.firstDerivative(0.37f) == Vector3());
}

// MARK: CubicBezier (2D)

TEST(bezier2DPointAtTheMiddleOfTheArch) {
    // x = 0 + 0 + 3/8·4 + 1/8·6 = 2.25      y = 0 + 3/8·4 + 3/8·4 + 0 = 3
    EXPECT(isClose(arch2().point(0.5f), Vector2(2.25f, 3)));
    // Weights at 0.25: x = 9/64·4 + 1/64·6 = 0.65625      y = 27/64·4 + 9/64·4 = 2.25
    EXPECT(isClose(arch2().point(0.25f), Vector2(0.65625f, 2.25f)));
}

TEST(bezier2DTheCurveStartsAndEndsAtTheEndPoints) {
    EXPECT(arch2().point(0) == Vector2(0, 0));
    EXPECT(arch2().point(1) == Vector2(6, 0));
    EXPECT(arch2().parameterRange() == (ParameterRange{0, 1}));
}

TEST(bezier2DDerivativesOfTheArch) {
    // Steps: (0,4), (4,0), (2,−4). At 0.5: (0 + 2 + 0.5, 1 + 0 − 1) = (2.5, 0); times 3 = (7.5, 0).
    EXPECT(isClose(arch2().firstDerivative(0.5f), Vector2(7.5f, 0)));
    EXPECT(isClose(arch2().firstDerivative(0), Vector2(0, 12)));
    EXPECT(isClose(arch2().firstDerivative(1), Vector2(6, -12)));
    // bend₀ = (4,−4), bend₁ = (−2,−4): at 0.5 it is 3·(2,−8) = (6,−24); at 0, 6·bend₀ = (24,−24).
    EXPECT(isClose(arch2().secondDerivative(0.5f), Vector2(6, -24)));
    EXPECT(isClose(arch2().secondDerivative(0), Vector2(24, -24)));
}

TEST(bezier2DSplittingTheArchAtTheMiddle) {
    // a = (0,2), b = (2,4), c = (5,2); d = (1,3), e = (3.5,3), f = (2.25,3)
    CubicBezierSplit<Vector2> halves = arch2().split(0.5f);
    EXPECT(halves.first.start == Vector2(0, 0));
    EXPECT(isClose(halves.first.startHandle, Vector2(0, 2)));
    EXPECT(isClose(halves.first.endHandle, Vector2(1, 3)));
    EXPECT(isClose(halves.first.end, Vector2(2.25f, 3)));
    EXPECT(isClose(halves.second.start, Vector2(2.25f, 3)));
    EXPECT(isClose(halves.second.startHandle, Vector2(3.5f, 3)));
    EXPECT(isClose(halves.second.endHandle, Vector2(5, 2)));
    EXPECT(halves.second.end == Vector2(6, 0));
}

TEST(bezier2DSplitHalvesReproduceTheOriginalCurve) {
    for (float cut : {0.2f, 0.5f, 0.75f}) {
        CubicBezierSplit<Vector2> halves = arch2().split(cut);
        for (float local : evenGrid(10, 10)) {
            EXPECT(isClose(halves.first.point(local), arch2().point(local * cut), 2e-6f));
            EXPECT(isClose(halves.second.point(local), arch2().point(cut + local * (1 - cut)), 2e-6f));
        }
    }
}

TEST(bezier2DIsTheFlatCaseOfThe3DCurve) {
    // The same four points in the z = 0 plane give the same curve: the code is shared.
    Curve3 curve3D(Vector3(0, 0, 0), Vector3(0, 4, 0), Vector3(4, 4, 0), Vector3(6, 0, 0));
    for (float parameter : evenGrid(10, 10)) {
        Vector2 flat = arch2().point(parameter);
        Vector3 space = curve3D.point(parameter);
        EXPECT(flat.x == space.x && flat.y == space.y && space.z == 0);
        Vector2 flatVelocity = arch2().firstDerivative(parameter);
        Vector3 spaceVelocity = curve3D.firstDerivative(parameter);
        EXPECT(flatVelocity.x == spaceVelocity.x && flatVelocity.y == spaceVelocity.y);
    }
}

TEST(bezier2DAStraightCubicHasTheLengthOfItsLine) {
    // Evenly spaced collinear points (0,0), (3,4), (6,8), (9,12): speed 3·|(3,4)| = 15.
    Curve2 line(Vector2(0, 0), Vector2(3, 4), Vector2(6, 8), Vector2(9, 12));
    EXPECT(isApproximatelyEqual(line.totalLength(), 15, 2e-6f));
    EXPECT(isApproximatelyEqual(line.length(0.25f, 0.75f), 7.5f, 2e-6f));
    EXPECT(isApproximatelyEqual(line.parameterAtDistance(7.5f), 0.5f, 2e-6f));
}

TEST(bezier2DAQuarterCircleApproximationHasTheArcLengthOfAQuarterCircle) {
    // The true arc has length π/2; the cubic's length is 2.2e-4 above it (measured).
    EXPECT(isApproximatelyEqual(quarterCircle2().totalLength(), swiftPi / 2, 7e-4f));
}

TEST(bezier2DEvenlySpacedPointsOnAQuarterCircleAreEvenlySpacedInAngle) {
    std::vector<Vector2> points = quarterCircle2().evenlySpacedPoints(5);
    EXPECT_EQUAL(points.size(), size_t(5));
    for (size_t index = 0; index < points.size(); index += 1) {
        EXPECT(isApproximatelyEqual(points[index].length(), 1, 8e-4f));
        float degrees = Angle::fromRadians(std::atan2(points[index].y, points[index].x)).degrees();
        EXPECT(isApproximatelyEqual(degrees, static_cast<float>(index) * 22.5f, 2e-3f));
    }
}

TEST(bezier2DTheLengthsOfTheHalvesAddUp) {
    CubicBezierSplit<Vector2> halves = arch2().split(0.3f);
    float sum = halves.first.totalLength() + halves.second.totalLength();
    EXPECT(isApproximatelyEqual(sum, arch2().totalLength(), 3e-7f * arch2().totalLength()));
}

// MARK: Analysis

TEST(analysisExtremaOfTheSymmetricArch) {
    // y′ = 3(1 − 2t) is zero at t = 1/2. x′ = 6t(1 − t) is zero at the two ends only.
    EXPECT_EQUAL(symmetricArch().verticalExtremaParameters().size(), size_t(1));
    EXPECT(isApproximatelyEqual(symmetricArch().verticalExtremaParameters()[0], 0.5f));
    EXPECT(symmetricArch().horizontalExtremaParameters() == (std::vector<float>{0, 1}));
    EXPECT(symmetricArch().extremaParameters() == (std::vector<float>{0, 0.5f, 1}));
}

TEST(analysisBoundsOfTheSymmetricArch) {
    // y(1/2) = 3·½·½ = 0.75. The control points reach y = 1, so the tight box is lower.
    Bounds2 bounds = symmetricArch().bounds();
    EXPECT(isClose(bounds.minimum, Vector2(0, 0)));
    EXPECT(isClose(bounds.maximum, Vector2(1, 0.75f)));
}

TEST(analysisExtremaAndBoundsOfACurveThatOvershootsInX) {
    // P = (0,0), (3,1), (−2,1), (1,0): x′ = 3(4t − 1)(4t − 3), so t = 1/4 and 3/4; x(1/4) = 1,
    // x(3/4) = 0. y′ = −6t + 3: t = 1/2, y(1/2) = 0.75.
    Curve2 curve(Vector2(0, 0), Vector2(3, 1), Vector2(-2, 1), Vector2(1, 0));
    std::vector<float> horizontal = curve.horizontalExtremaParameters();
    EXPECT_EQUAL(horizontal.size(), size_t(2));
    EXPECT(isApproximatelyEqual(horizontal[0], 0.25f));
    EXPECT(isApproximatelyEqual(horizontal[1], 0.75f));
    EXPECT(isClose(curve.bounds().minimum, Vector2(0, 0)));
    EXPECT(isClose(curve.bounds().maximum, Vector2(1, 0.75f)));
}

TEST(analysisExtremaOfALopsidedArch) {
    // x′ = 6t(4 − 3t): roots 0 and 4/3; only 0 is in range. y = 12t(1 − t): maximum 3.
    EXPECT(arch2().horizontalExtremaParameters() == (std::vector<float>{0}));
    EXPECT(isClose(arch2().bounds().maximum, Vector2(6, 3)));
}

TEST(analysisAStraightCurveHasNoInteriorExtrema) {
    Curve2 line(Vector2(0, 0), Vector2(1, 1), Vector2(2, 2), Vector2(3, 3));
    EXPECT(line.extremaParameters().empty());
    EXPECT(line.bounds() == (Bounds2{Vector2(0, 0), Vector2(3, 3)}));
}

TEST(analysisACurveWithSteadyDerivativeTouchingZeroHasNoExtremum) {
    // x = t³ has x′ = 3t², a double root at 0: a flat spot, not an extreme.
    Curve2 curve(Vector2(0, 0), Vector2(0, 1), Vector2(0, 2), Vector2(1, 3));
    EXPECT(curve.horizontalExtremaParameters().empty());
}

TEST(analysisBoundsContainTheCurveAndTouchIt) {
    SeededRandom generator(7);
    for (int trial = 0; trial < 200; trial += 1) {
        Curve2 curve = randomCurve(generator, 5);
        Bounds2 bounds = curve.bounds();
        std::vector<Vector2> samples;
        for (int step = 0; step <= 2000; step += 1) samples.push_back(curve.point(static_cast<float>(step) / 2000));
        Bounds2 sampled = Bounds2::enclosing(samples);
        for (Vector2 sample : samples) {
            EXPECT(sample.x >= bounds.minimum.x - 1e-5f && sample.x <= bounds.maximum.x + 1e-5f);
            EXPECT(sample.y >= bounds.minimum.y - 1e-5f && sample.y <= bounds.maximum.y + 1e-5f);
        }
        EXPECT(isClose(sampled.minimum, bounds.minimum, 5e-6f));
        EXPECT(isClose(sampled.maximum, bounds.maximum, 5e-6f));
    }
}

TEST(analysisDerivativeIsZeroAtEveryReportedExtremum) {
    SeededRandom generator(11);
    for (int trial = 0; trial < 200; trial += 1) {
        Curve2 curve = randomCurve(generator, 5);
        for (float parameter : curve.horizontalExtremaParameters()) {
            EXPECT(std::fabs(curve.firstDerivative(parameter).x) <= 3e-5f);
        }
        for (float parameter : curve.verticalExtremaParameters()) {
            EXPECT(std::fabs(curve.firstDerivative(parameter).y) <= 3e-5f);
        }
    }
}

TEST(analysisInflectionOfTheSCurve) {
    // a×b = 0, c×a = −6, c×b = 9, so −18t + 9 = 0 and t = 1/2: the middle of the S.
    std::vector<float> inflections = sCurve().inflectionParameters();
    EXPECT_EQUAL(inflections.size(), size_t(1));
    EXPECT(isApproximatelyEqual(inflections[0], 0.5f));
    EXPECT(isApproximatelyEqual(sCurve().signedCurvature(0.5f), 0));
    EXPECT(sCurve().signedCurvature(0.25f) * sCurve().signedCurvature(0.75f) < 0);
}

TEST(analysisCurvesWithoutAnInflection) {
    EXPECT(symmetricArch().inflectionParameters().empty());
    EXPECT(quarterCircle2().inflectionParameters().empty());
    Curve2 line(Vector2(0, 0), Vector2(1, 1), Vector2(2, 2), Vector2(3, 3));
    EXPECT(line.inflectionParameters().empty());
}

TEST(analysisAStraightCurveWithNonBinaryPointsHasNoInflection) {
    // These points lie on y = 0.7x + 0.23 but none is exact in binary: the cross products are noise.
    Curve2 line(Vector2(0.1f, 0.3f), Vector2(0.4f, 0.51f), Vector2(0.8f, 0.79f), Vector2(1.1f, 1.0f));
    EXPECT(line.inflectionParameters().empty());
}

TEST(analysisACuspHasNoInflectionWhereverItIs) {
    // The quadratic is 9(2t − 1)²: a double root at 1/2, where B′(1/2) = 0 (a cusp). Shifted by
    // (0.1, 0.2) the points are no longer exact in binary; the answer must not change.
    Curve2 cusp(Vector2(0, 0), Vector2(1, 1), Vector2(0, 1), Vector2(1, 0));
    for (Curve2 curve : {cusp, translatedBy(cusp, Vector2(0.1f, 0.2f))}) {
        EXPECT(curve.inflectionParameters().empty());
        EXPECT(curve.horizontalExtremaParameters().empty());
        EXPECT_EQUAL(curve.verticalExtremaParameters().size(), size_t(1));
        EXPECT(isApproximatelyEqual(curve.verticalExtremaParameters()[0], 0.5f));
    }
}

TEST(analysisStraightCurvesHaveNoInflections) {
    SeededRandom generator(66);
    auto gridValue = [&](int lower, int upper, float denominator) {
        return static_cast<float>(generator.nextInt(lower, upper)) / denominator;
    };
    for (int trial = 0; trial < 2000; trial += 1) {
        float startX = gridValue(-64, 64, 64);
        float startY = gridValue(-64, 64, 64);
        Vector2 start(startX, startY);
        float directionX = gridValue(-64, 64, 64);
        float directionY = gridValue(-64, 64, 64);
        Vector2 direction(directionX, directionY);
        float firstFactor = gridValue(-8, 16, 8);
        float secondFactor = gridValue(-8, 16, 8);
        Curve2 line(start, start + direction * firstFactor, start + direction * secondFactor, start + direction);
        EXPECT(line.inflectionParameters().empty());
    }
    for (int trial = 0; trial < 2000; trial += 1) {
        float startX = generator.nextFloat(-1, 1);
        float startY = generator.nextFloat(-1, 1);
        Vector2 start(startX, startY);
        float directionX = generator.nextFloat(-1, 1);
        float directionY = generator.nextFloat(-1, 1);
        Vector2 direction(directionX, directionY);
        float firstFactor = generator.nextFloat(-1, 2);
        float secondFactor = generator.nextFloat(-1, 2);
        Curve2 line(start, start + direction * firstFactor, start + direction * secondFactor, start + direction);
        // The stored points are slightly off the line, so on rare curves an inflection is real;
        // then exact arithmetic must find as many.
        std::vector<float> reported = line.inflectionParameters();
        if (!reported.empty()) {
            std::vector<double> exact;
            exactInflections(line, 0, exact);
            EXPECT_EQUAL(reported.size(), exact.size());
        }
    }
}

TEST(analysisInflectionsMatchExactArithmetic) {
    SeededRandom generator(67);
    for (int trial = 0; trial < 2000; trial += 1) {
        Curve2 curve = randomCurve(generator, 1);
        std::vector<double> exact;
        if (!exactInflections(curve, 1e-3, exact)) continue;
        EXPECT_EQUAL(curve.inflectionParameters().size(), exact.size());
    }
}

TEST(analysisACurveWithTwoInflections) {
    // The quadratic is −27(11t² − 7t + 1): t = (7 ± √5)/22 = 0.21654 and 0.41982.
    Curve2 curve(Vector2(0, 0), Vector2(-3, -3), Vector2(-1, 0), Vector2(4, 0));
    std::vector<float> inflections = curve.inflectionParameters();
    EXPECT_EQUAL(inflections.size(), size_t(2));
    EXPECT(isApproximatelyEqual(inflections[0], 0.2165424f, 1e-5f));
    EXPECT(isApproximatelyEqual(inflections[1], 0.4198213f, 1e-5f));
    for (float parameter : inflections) {
        EXPECT(std::fabs(curve.signedCurvature(parameter)) <= 2e-3f);
        EXPECT(curve.signedCurvature(parameter - 0.02f) * curve.signedCurvature(parameter + 0.02f) < 0);
    }
}

TEST(analysisCurvatureOfTheParabola) {
    // x = t, y = t²: κ = 2 / (1 + 4t²)^(3/2): 2, 0.70710678, 0.17888544 at t = 0, 1/2, 1.
    EXPECT(isApproximatelyEqual(parabola().signedCurvature(0), 2));
    EXPECT(isApproximatelyEqual(parabola().signedCurvature(0.5f), 0.70710678f));
    EXPECT(isApproximatelyEqual(parabola().signedCurvature(1), 0.17888544f));
}

TEST(analysisCurvatureOfTheQuarterCircleIsOneOverTheRadius) {
    for (float parameter : evenGrid(20, 20)) {
        EXPECT(isApproximatelyEqual(quarterCircle2().signedCurvature(parameter), 1, 0.065f));
    }
    Curve2 backwards = reversed(quarterCircle2());
    EXPECT(isApproximatelyEqual(backwards.signedCurvature(0.5f), -quarterCircle2().signedCurvature(0.5f)));
    EXPECT(backwards.signedCurvature(0.5f) < 0);
}

TEST(analysisCurvatureAtTinyAndHugeScales) {
    // The parabola scaled by s has κ(0) = 2/s.
    for (float scale : {1e-15f, 1e-3f, 1e3f, 1e13f}) {
        Curve2 base = parabola();
        Curve2 curve(base.start * scale, base.startHandle * scale, base.endHandle * scale, base.end * scale);
        float relative = curve.signedCurvature(0) * scale / 2;
        EXPECT(isApproximatelyEqual(relative, 1, 3.6e-7f));
    }
}

TEST(analysisCurvatureOfACurveWithANonFinitePointIsNaN) {
    for (float bad : {notANumber, infinity}) {
        Curve2 curve(Vector2(0, 0), Vector2(bad, 0), Vector2(1, 1), Vector2(2, 0));
        EXPECT(std::isnan(curve.signedCurvature(0.5f)));
    }
}

TEST(analysisCurvatureIsZeroWhereTheVelocityIsZero) {
    Curve2 cusp(Vector2(0, 0), Vector2(0, 0), Vector2(1, 1), Vector2(2, 0));
    EXPECT(cusp.signedCurvature(0) == 0);
}

TEST(analysisAreaSweptByTheArch) {
    // ∫y dx = 18·B(3,3) = 0.6; about the origin the fan is that region traversed clockwise: −0.6.
    EXPECT(isApproximatelyEqual(symmetricArch().signedAreaSwept(), -0.6f));
}

TEST(analysisAreaSweptByAStraightCurve) {
    Curve2 line(Vector2(1, 0), Vector2(2.0f / 3, 1.0f / 3), Vector2(1.0f / 3, 2.0f / 3), Vector2(0, 1));
    EXPECT(isApproximatelyEqual(line.signedAreaSwept(), 0.5f));
    EXPECT(isApproximatelyEqual(line.signedAreaSwept(Vector2(0.5f, 0.5f)), 0));
}

TEST(analysisAreaSweptByTheQuarterCircle) {
    // The true quarter disc is π/4; the cubic bulges outward and adds +2.2e-4 (measured).
    EXPECT(isApproximatelyEqual(quarterCircle2().signedAreaSwept(), swiftPi / 4, 7e-4f));
}

TEST(analysisAreaMatchesNumericalIntegration) {
    SeededRandom generator(21);
    for (int trial = 0; trial < 100; trial += 1) {
        Curve2 curve = randomCurve(generator, 3);
        float pivotX = generator.nextFloat(-2, 2);
        float pivotY = generator.nextFloat(-2, 2);
        Vector2 pivot(pivotX, pivotY);
        int steps = 200;
        double sum = 0;
        for (int step = 0; step <= steps; step += 1) {
            float parameter = static_cast<float>(step) / static_cast<float>(steps);
            double weight = step == 0 || step == steps ? 1 : (step % 2 == 1 ? 4 : 2);
            sum += weight * double((curve.point(parameter) - pivot).cross(curve.firstDerivative(parameter)));
        }
        float simpson = static_cast<float>(sum / double(3 * steps) / 2);
        EXPECT(isApproximatelyEqual(curve.signedAreaSwept(pivot), simpson, 1.5e-5f));
    }
}

TEST(analysisAreaIsInvariantUnderMovingTheCurveAndThePivot) {
    SeededRandom generator(33);
    for (int trial = 0; trial < 100; trial += 1) {
        Curve2 curve = randomGridCurve(generator);
        Vector2 pivot(1, -1);
        for (Vector2 offset : {Vector2(512, -256), Vector2(8192, 8192)}) {
            EXPECT(translatedBy(curve, offset).signedAreaSwept(pivot + offset) == curve.signedAreaSwept(pivot));
        }
    }
}

TEST(analysisAreaFarFromTheOriginMatchesExactArithmetic) {
    SeededRandom generator(34);
    for (int trial = 0; trial < 100; trial += 1) {
        Curve2 curve = translatedBy(randomCurve(generator, 1), Vector2(1e5f, 1e5f));
        Vector2 pivot = curve.start + Vector2(0.5f, -0.25f);
        float reference = static_cast<float>(exactAreaSwept(curve, pivot));
        EXPECT(isApproximatelyEqual(curve.signedAreaSwept(pivot), reference, 3.6e-7f));
    }
}

TEST(analysisAreaScalesWithTheSquareOfTheSize) {
    SeededRandom generator(44);
    for (float scale : {1e-3f, 1e3f}) {
        for (int trial = 0; trial < 50; trial += 1) {
            Curve2 curve = randomCurve(generator, 3);
            Curve2 big(curve.start * scale, curve.startHandle * scale, curve.endHandle * scale, curve.end * scale);
            float expected = curve.signedAreaSwept() * scale * scale;
            float ratio = std::fabs(big.signedAreaSwept() - expected) / (scale * scale);
            EXPECT(ratio <= 4e-5f);
        }
    }
}

TEST(analysisReversingTheCurveNegatesTheArea) {
    SeededRandom generator(55);
    for (int trial = 0; trial < 50; trial += 1) {
        Curve2 curve = randomCurve(generator, 3);
        EXPECT(isApproximatelyEqual(reversed(curve).signedAreaSwept(), -curve.signedAreaSwept(), 3e-5f));
    }
}

TEST(analysisHasFiniteControlPoints) {
    EXPECT(arch2().hasFiniteControlPoints());
    EXPECT(!Curve2(Vector2(0, 0), Vector2(infinity, 0), Vector2(1, 1), Vector2(2, 0)).hasFiniteControlPoints());
}

// MARK: Tangents

TEST(tangentsDirectionsAtTheEnds) {
    EXPECT(isClose(arch2().startDirection(), Vector2(0, 1)));
    EXPECT(isClose(arch2().endDirection(), Vector2(0.4472136f, -0.8944272f)));
    // A handle on the start: the curve leaves along the next handle; all on the start: a point.
    Curve2 handleOnStart(Vector2(1, 1), Vector2(1, 1), Vector2(1, 3), Vector2(2, 3));
    EXPECT(isClose(handleOnStart.startDirection(), Vector2(0, 1)));
    Curve2 point(Vector2(1, 1), Vector2(1, 1), Vector2(1, 1), Vector2(1, 1));
    EXPECT(point.startDirection() == Vector2());
    EXPECT(point.endDirection() == Vector2());
}

TEST(tangentsPortionOfACurveIsTheCurveBetweenTheParameters) {
    Curve2 curve = arch2();
    Curve2 piece = curve.portion(0.2f, 0.7f);
    for (float local : evenGrid(10, 10)) {
        EXPECT(isClose(piece.point(local), curve.point(0.2f + local * 0.5f), 1e-5f));
    }
    EXPECT(curve.portion(0, 1) == curve);
    Curve2 empty = curve.portion(0.4f, 0.4f);
    EXPECT(empty.start == empty.end);
    EXPECT_PROBLEM(curve.portion(0.7f, 0.2f), "A portion needs 0 ≤ lower ≤ upper ≤ 1.");
}

TEST(tangentsTranslatedMovesEveryControlPoint) {
    Curve2 moved = arch2().translated(Vector2(1, 2));
    EXPECT(moved.start == Vector2(1, 2));
    EXPECT(moved.end == Vector2(7, 2));
}

// MARK: Flattening

TEST(flatteningAStraightLineIsOnePiece) {
    Curve2 line(Vector2(0, 0), Vector2(1, 1), Vector2(2, 2), Vector2(3, 3));
    EXPECT_EQUAL(line.flatteningPieceCount(0.01f), 1);
    EXPECT_EQUAL(line.flattened(0.01f).size(), size_t(2));
}

TEST(flatteningStaysWithinTheTolerance) {
    // n ≥ √(0.75·M / tolerance), M the larger bend |P0 − 2P1 + P2| or |P1 − 2P2 + P3|. For the
    // arch both bends are |(4,−4)| = 5.657 and |(−2,−4)| = 4.472, so n = ⌈√(0.75·5.657/0.01)⌉ = 21.
    EXPECT_EQUAL(arch2().flatteningPieceCount(0.01f), 21);
    std::vector<Vector2> polyline = arch2().flattened(0.01f);
    EXPECT_EQUAL(polyline.size(), size_t(22));
    EXPECT(polyline.front() == arch2().start);
    EXPECT(polyline.back() == arch2().end);
}

TEST(flatteningLimitsAndBadInput) {
    EXPECT_EQUAL(arch2().flatteningPieceCount(1e-20f), Curve2::mostFlatteningPieces);
    EXPECT_PROBLEM(arch2().flatteningPieceCount(0), "Flattening needs a positive tolerance.");
    EXPECT_PROBLEM(arch2().flatteningPieceCount(notANumber), "Flattening needs a positive tolerance.");
    Curve2 broken(Vector2(0, 0), Vector2(notANumber, 0), Vector2(1, 1), Vector2(2, 0));
    EXPECT_EQUAL(broken.flatteningPieceCount(0.01f), 1);
}

// MARK: Circular arcs

TEST(arcHandleFactorOfAQuarterTurn) {
    // (4/3)·tan(90°/4) = (4/3)(√2 − 1) = 0.5522848; at 45°, 0.2652165; the mirror image clockwise.
    EXPECT(isApproximatelyEqual(Curve2::circularArcHandleFactor(Angle::degrees(90)), 0.5522848f));
    EXPECT(isApproximatelyEqual(Curve2::circularArcHandleFactor(Angle::degrees(45)), 0.2652165f));
    EXPECT(isApproximatelyEqual(Curve2::circularArcHandleFactor(Angle::degrees(-90)), -0.5522848f));
    EXPECT(Curve2::circularArcHandleFactor(Angle::degrees(0)) == 0);
}

TEST(arcAQuarterArcHasTheKnownControlPoints) {
    std::vector<Curve2> arcs = Curve2::circularArc(Vector2(2, 3), 2, Angle::degrees(0), Angle::degrees(90));
    EXPECT_EQUAL(arcs.size(), size_t(1));
    EXPECT(isClose(arcs[0].start, Vector2(4, 3)));
    EXPECT(isClose(arcs[0].startHandle, Vector2(4, 4.1045695f)));
    EXPECT(isClose(arcs[0].endHandle, Vector2(3.1045695f, 5)));
    EXPECT(isClose(arcs[0].end, Vector2(2, 5), 2e-5f));
}

TEST(arcAClockwiseArcMirrorsTheCounterclockwiseOne) {
    std::vector<Curve2> arcs = Curve2::circularArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(-90));
    EXPECT_EQUAL(arcs.size(), size_t(1));
    EXPECT(isClose(arcs[0].start, Vector2(1, 0)));
    EXPECT(isClose(arcs[0].startHandle, Vector2(1, -0.5522848f)));
    EXPECT(isClose(arcs[0].end, Vector2(0, -1), 2e-5f));
}

TEST(arcThePieceCountIsTheFewestOfAtMostNinetyDegrees) {
    auto count = [](float degrees) {
        return Curve2::circularArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(degrees)).size();
    };
    EXPECT_EQUAL(count(0), size_t(0));
    EXPECT_EQUAL(count(1), size_t(1));
    EXPECT_EQUAL(count(90), size_t(1));
    EXPECT_EQUAL(count(91), size_t(2));
    EXPECT_EQUAL(count(180), size_t(2));
    EXPECT_EQUAL(count(270), size_t(3));
    EXPECT_EQUAL(count(360), size_t(4));
    EXPECT_EQUAL(count(450), size_t(5));
    EXPECT_EQUAL(count(-360), size_t(4));
    EXPECT_EQUAL(count(-180), size_t(2));
    EXPECT(Curve2::circularArc(Vector2(), 0, Angle::degrees(0), Angle::degrees(90)).empty());
}

TEST(arcInvalidRadiusOrSweepGivesNoPieces) {
    auto count = [](float radius, Angle sweep) {
        return Curve2::circularArc(Vector2(), radius, Angle::degrees(0), sweep).size();
    };
    EXPECT_EQUAL(count(1, Angle::fromRadians(notANumber)), size_t(0));
    EXPECT_EQUAL(count(1, Angle::fromRadians(infinity)), size_t(0));
    EXPECT_EQUAL(count(1, Angle::fromRadians(-infinity)), size_t(0));
    EXPECT_EQUAL(count(1, Angle::fromRadians(1e9f)), size_t(0));
    EXPECT_EQUAL(count(notANumber, Angle::degrees(90)), size_t(0));
    EXPECT_EQUAL(count(infinity, Angle::degrees(90)), size_t(0));
    EXPECT_EQUAL(count(-1, Angle::degrees(90)), size_t(0));
    // 100 turns are 400 quarter turns: allowed. 3000 turns would be 12,000 pieces, over the cap.
    EXPECT_EQUAL(count(1, Angle::degrees(36000)), size_t(400));
    EXPECT_EQUAL(count(1, Angle::degrees(3000 * 360)), size_t(0));
}

TEST(arcAFullCircleJoinsAtTheQuarterPoints) {
    std::vector<Curve2> arcs = Curve2::circularArc(Vector2(), 1, Angle::degrees(0), Angle::degrees(360));
    Vector2 corners[5] = {Vector2(1, 0), Vector2(0, 1), Vector2(-1, 0), Vector2(0, -1), Vector2(1, 0)};
    for (size_t index = 0; index < arcs.size(); index += 1) {
        EXPECT(isClose(arcs[index].start, corners[index], 2e-5f));
        EXPECT(isClose(arcs[index].end, corners[index + 1], 2e-5f));
    }
}

TEST(arcConsecutivePiecesJoinSmoothly) {
    SeededRandom generator(3);
    for (int trial = 0; trial < 50; trial += 1) {
        float centreX = generator.nextFloat(-3, 3);
        float radius = generator.nextFloat(0.5f, 3);
        float startDegrees = generator.nextFloat(0, 360);
        float sweepDegrees = generator.nextFloat(100, 700);
        std::vector<Curve2> arcs = Curve2::circularArc(
            Vector2(centreX, 1), radius, Angle::degrees(startDegrees), Angle::degrees(sweepDegrees));
        for (size_t index = 1; index < arcs.size(); index += 1) {
            Vector2 incoming = arcs[index - 1].firstDerivative(1).normalized();
            Vector2 outgoing = arcs[index].firstDerivative(0).normalized();
            EXPECT(isApproximatelyEqual(incoming.cross(outgoing), 0, 2e-5f));
            EXPECT(incoming.dot(outgoing) > 0.99f);
            // The boundary points are computed once and shared, so the join is exact.
            EXPECT(arcs[index - 1].end == arcs[index].start);
        }
    }
}

namespace {

/// The largest distance of the arc's points from the circle, as a fraction of the radius.
float largestRadialError(Vector2 centre, float radius, Angle sweep) {
    std::vector<Curve2> arcs = Curve2::circularArc(centre, radius, Angle::degrees(20), sweep);
    float worst = 0;
    for (const Curve2& arc : arcs) {
        for (int step = 0; step <= 1000; step += 1) {
            Vector2 offset = arc.point(static_cast<float>(step) / 1000) - centre;
            worst = largerOf(worst, std::fabs(offset.length() - radius) / radius);
        }
    }
    return worst;
}

} // namespace

TEST(arcTheRadialErrorOfAQuarterTurnIsAboutTwoPointSevenTenThousandths) {
    float error = largestRadialError(Vector2(), 1, Angle::degrees(90));
    EXPECT(error > 2.6e-4f && error < 2.8e-4f);
}

TEST(arcHalvingTheAngleShrinksTheErrorByAboutSixtyFour) {
    float quarter = largestRadialError(Vector2(), 1, Angle::degrees(90));
    float twoQuarters = largestRadialError(Vector2(), 1, Angle::degrees(180));
    EXPECT(std::fabs(twoQuarters - quarter) < 1e-6f);
    float finer = largestRadialError(Vector2(), 1, Angle::degrees(45));
    EXPECT(finer < quarter / 55 && finer > quarter / 75);
}

TEST(arcTheErrorIsRelativeToTheRadiusAndSurvivesAFarCentre) {
    EXPECT(largestRadialError(Vector2(), 1e-3f, Angle::degrees(90)) < 2.8e-4f);
    EXPECT(largestRadialError(Vector2(), 1e3f, Angle::degrees(90)) < 2.8e-4f);
    EXPECT(largestRadialError(Vector2(1024, -2048), 100, Angle::degrees(270)) < 2.8e-4f);
}

// MARK: Arc length

TEST(arcLengthOfAnEvenlySpacedLineIsTheDistanceBetweenItsEnds) {
    // Speed is a constant 3 over parameter 0…1, so the length is 3; half the range is half.
    EXPECT(isApproximatelyEqual(line3().totalLength(), 3));
    EXPECT(isApproximatelyEqual(line3().length(0, 0.5f), 1.5f));
    EXPECT(isApproximatelyEqual(line3().length(0.25f, 0.75f), 1.5f));
}

TEST(arcLengthOfALineWithUnevenSpeedStillEqualsTheDistance) {
    Curve3 uneven(Vector3(0, 0, 0), Vector3(0.5f, 0, 0), Vector3(1, 0, 0), Vector3(5, 0, 0));
    EXPECT(isApproximatelyEqual(uneven.totalLength(), 5));
}

TEST(arcLengthQuarterCircleLengthIsCloseToAQuarterOfTwoPi) {
    EXPECT(isApproximatelyEqual(quarterCircle3().totalLength(), swiftPi / 2, 5e-4f));
}

TEST(arcLengthOfSplitHalvesAddsUpToTheWhole) {
    CubicBezierSplit<Vector3> halves = quarterCircle3().split(0.3f);
    float sum = halves.first.totalLength() + halves.second.totalLength();
    EXPECT(isApproximatelyEqual(sum, quarterCircle3().totalLength(), 2e-5f));
}

TEST(arcLengthReversedOrEmptyRangesHaveNoLength) {
    EXPECT(line3().length(0.5f, 0.5f) == 0);
    EXPECT(line3().length(0.8f, 0.2f) == 0);
}

TEST(arcLengthParameterAtDistanceOnAnEvenLineIsLinear) {
    EXPECT(isApproximatelyEqual(line3().parameterAtDistance(1.5f), 0.5f));
    EXPECT(isApproximatelyEqual(line3().parameterAtDistance(0.3f), 0.1f));
}

TEST(arcLengthParameterAtDistanceInvertsLength) {
    std::vector<Curve3> curves = {
        quarterCircle3(),
        Curve3(Vector3(0, 0, 0), Vector3(0.5f, 0, 0), Vector3(1, 0, 0), Vector3(5, 2, 1))};
    for (const Curve3& curve : curves) {
        for (float fraction : {0.05f, 0.2f, 0.5f, 0.77f, 0.95f}) {
            float parameter = curve.parameterRange().lowerBound
                + fraction * (curve.parameterRange().upperBound - curve.parameterRange().lowerBound);
            float distance = curve.length(curve.parameterRange().lowerBound, parameter);
            EXPECT(isApproximatelyEqual(curve.parameterAtDistance(distance), parameter, 1e-4f));
        }
    }
}

TEST(arcLengthDistancesOutsideTheCurveAreClamped) {
    EXPECT(line3().parameterAtDistance(0) == 0);
    EXPECT(line3().parameterAtDistance(-5) == 0);
    EXPECT(line3().parameterAtDistance(100) == 1);
}

TEST(arcLengthAStartPastTheEndIsReturnedUnchanged) {
    EXPECT(line3().parameterAtDistance(1, 2) == 2);
}

TEST(arcLengthInversionStaysAccurateOnALongCurveFarFromTheOrigin) {
    // An evenly paced line 3000 long, starting 10 km out: parameter = distance / 3000.
    Vector3 origin(10000, 0, 0);
    Curve3 line(origin, origin + Vector3(1000, 0, 0), origin + Vector3(2000, 0, 0), origin + Vector3(3000, 0, 0));
    for (float distance : {1.0f, 750.0f, 1500.0f, 2999.0f}) {
        EXPECT(isApproximatelyEqual(line.parameterAtDistance(distance), distance / 3000, 1e-5f));
    }
}

TEST(arcLengthEvenlySpacedPointsOnALineAreEvenlySpaced) {
    // Even though the speed varies, five samples of the 0…5 segment sit at 0, 1.25, …, 5.
    Curve3 uneven(Vector3(0, 0, 0), Vector3(0.5f, 0, 0), Vector3(1, 0, 0), Vector3(5, 0, 0));
    std::vector<Vector3> points = uneven.evenlySpacedPoints(5);
    EXPECT_EQUAL(points.size(), size_t(5));
    for (size_t index = 0; index < points.size(); index += 1) {
        EXPECT(isApproximatelyEqual(points[index], Vector3(1.25f * static_cast<float>(index), 0, 0), 1e-4f));
    }
}

TEST(arcLengthEvenlySpacedSamplesHaveEqualArcLengthBetweenThem) {
    Curve3 curve(Vector3(0, 0, 0), Vector3(4, 0, 3), Vector3(-2, 5, 4), Vector3(3, 6, -2));
    std::vector<float> parameters = curve.evenlySpacedParameters(12);
    EXPECT(parameters.front() == 0);
    EXPECT(parameters.back() == 1);
    float expectedStep = curve.totalLength() / 11;
    for (size_t index = 1; index < parameters.size(); index += 1) {
        EXPECT(isApproximatelyEqual(curve.length(parameters[index - 1], parameters[index]), expectedStep, 1e-4f));
    }
}

TEST(arcLengthTwoSamplesAreJustTheEnds) {
    EXPECT(line3().evenlySpacedPoints(2) == (std::vector<Vector3>{Vector3(0, 0, 0), Vector3(3, 0, 0)}));
    EXPECT_PROBLEM(line3().evenlySpacedParameters(1), "Evenly spaced samples need at least a start and an end.");
}

// MARK: Curve frames

namespace {

Curve3 twistyCurve() { return Curve3(Vector3(0, 0, 0), Vector3(4, 0, 3), Vector3(-2, 5, 4), Vector3(3, 6, -2)); }

} // namespace

TEST(framesFirstFrameOfAStraightLineAlongYWithDefaultUp) {
    // Tangent +y; up = +z is already perpendicular, so the normal is +z. The binormal is y × z = +x.
    Curve3 line(Vector3(0, 0, 0), Vector3(0, 1, 0), Vector3(0, 2, 0), Vector3(0, 3, 0));
    CurveFrame frame = line.frame(0.5f);
    EXPECT(isApproximatelyEqual(frame.position, Vector3(0, 1.5f, 0)));
    EXPECT(isApproximatelyEqual(frame.tangent, Vector3(0, 1, 0)));
    EXPECT(isApproximatelyEqual(frame.normal, Vector3(0, 0, 1)));
    EXPECT(isApproximatelyEqual(frame.binormal, Vector3(1, 0, 0)));
}

TEST(framesUpVectorIsProjectedPerpendicularToTheTangent) {
    // Tangent (1,0,1)/√2 and up +z: up − (up·t)t = (−0.5, 0, 0.5), normalized (−0.7071, 0, 0.7071).
    Curve3 slope(Vector3(0, 0, 0), Vector3(1, 0, 1), Vector3(2, 0, 2), Vector3(3, 0, 3));
    CurveFrame frame = slope.frame(0.3f);
    EXPECT(isApproximatelyEqual(frame.normal, Vector3(-0.70710678f, 0, 0.70710678f)));
    expectOrthonormalAndRightHanded(frame);
}

TEST(framesATangentParallelToUpPicksAWorldAxis) {
    // Straight up: +z is no use, so the fallback is +x.
    Curve3 climb(Vector3(0, 0, 0), Vector3(0, 0, 1), Vector3(0, 0, 2), Vector3(0, 0, 3));
    CurveFrame frame = climb.frame(0.5f);
    EXPECT(isApproximatelyEqual(frame.normal, Vector3(1, 0, 0)));
    expectOrthonormalAndRightHanded(frame);
    // A custom up equal to the tangent direction along x falls back to +y.
    EXPECT(isApproximatelyEqual(line3().frame(0.5f, Vector3(5, 0, 0)).normal, Vector3(0, 1, 0)));
}

TEST(framesSamplesAtTheSamePositionStillRotateTheFrame) {
    // A closed loop sampled only at its two ends, which coincide at the origin. The tangent turns
    // from +x to +y; with up +y the first frame is n = +y, b = +z, and the smallest rotation taking
    // +x to +y is 90° about +z, which sends n to −x and keeps b at +z.
    Curve3 loop(Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, -1, 0), Vector3(0, 0, 0));
    std::vector<CurveFrame> frames = loop.rotationMinimizingFrames({0, 1}, Vector3::yAxis());
    EXPECT(isApproximatelyEqual(frames[1].tangent, Vector3(0, 1, 0)));
    EXPECT(isApproximatelyEqual(frames[1].normal, Vector3(-1, 0, 0)));
    EXPECT(isApproximatelyEqual(frames[1].binormal, Vector3(0, 0, 1)));
}

TEST(framesAreOrthonormalAndRightHandedAlongATwistyCurve) {
    for (const CurveFrame& frame : twistyCurve().rotationMinimizingFrames(evenGrid(40, 40))) {
        expectOrthonormalAndRightHanded(frame);
    }
}

TEST(framesFollowTheCurvePositionsAndTangents) {
    std::vector<float> parameters = evenGrid(10, 10);
    std::vector<CurveFrame> frames = twistyCurve().rotationMinimizingFrames(parameters);
    EXPECT_EQUAL(frames.size(), parameters.size());
    for (size_t index = 0; index < frames.size(); index += 1) {
        EXPECT(isApproximatelyEqual(frames[index].position, twistyCurve().point(parameters[index])));
        EXPECT(isApproximatelyEqual(
            frames[index].tangent, twistyCurve().firstDerivative(parameters[index]).normalized()));
    }
}

TEST(framesAPlanarCurveInTheGroundPlaneKeepsItsNormalUp) {
    Curve3 flat(Vector3(0, 0, 0), Vector3(5, 0, 0), Vector3(0, 5, 0), Vector3(5, 5, 0));
    for (const CurveFrame& frame : flat.rotationMinimizingFrames(evenGrid(30, 30))) {
        EXPECT(isApproximatelyEqual(frame.normal, Vector3(0, 0, 1)));
    }
}

TEST(framesAPlanarCurveInAVerticalPlaneKeepsItsBinormalConstant) {
    Curve3 climbingArc(Vector3(0, 0, 0), Vector3(3, 0, 0), Vector3(5, 0, 2), Vector3(5, 0, 6));
    std::vector<CurveFrame> frames = climbingArc.rotationMinimizingFrames(evenGrid(30, 30));
    Vector3 firstBinormal = frames[0].binormal;
    EXPECT(isApproximatelyEqual(std::fabs(firstBinormal.y), 1));
    for (const CurveFrame& frame : frames) EXPECT(isApproximatelyEqual(frame.binormal, firstBinormal, 1e-4f));
}

TEST(framesTheNormalDoesNotSpinAroundTheTangentOnATwistyCurve) {
    std::vector<CurveFrame> frames = twistyCurve().rotationMinimizingFrames(evenGrid(200, 200));
    for (size_t index = 1; index < frames.size(); index += 1) {
        Vector3 normalChange = frames[index].normal - frames[index - 1].normal;
        Vector3 averageBinormal = frames[index].binormal + frames[index - 1].binormal;
        EXPECT(std::fabs(normalChange.dot(averageBinormal)) < 1e-4f);
    }
}

TEST(framesNoParametersGiveNoFrames) { EXPECT(twistyCurve().rotationMinimizingFrames({}).empty()); }

TEST(framesSweepingAlongEvenlySpacedSamplesWorks) {
    std::vector<float> parameters = twistyCurve().evenlySpacedParameters(25);
    std::vector<CurveFrame> frames = twistyCurve().rotationMinimizingFrames(parameters);
    EXPECT_EQUAL(frames.size(), size_t(25));
    for (const CurveFrame& frame : frames) expectOrthonormalAndRightHanded(frame);
}

TEST(framesAHandleOnItsEndPointStillGivesTheTangent) {
    // B′(0) = 0 when the start handle sits on the start; just beside it the curve moves along +X.
    Curve3 startStops(Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 2, 0));
    EXPECT(isApproximatelyEqual(startStops.unitTangent(0), Vector3::xAxis(), 1e-3f));
    // The same at the end: P3 = P2, so the curve arrives along P3 − P1 = (0, 2, 0).
    Curve3 endStops(Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(1, 2, 0), Vector3(1, 2, 0));
    EXPECT(isApproximatelyEqual(endStops.unitTangent(1), Vector3::yAxis(), 1e-3f));
    Curve3 bothStop(Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(1, 2, 0), Vector3(1, 2, 0));
    for (const CurveFrame& frame : bothStop.rotationMinimizingFrames(evenGrid(8, 8))) {
        expectOrthonormalAndRightHanded(frame);
    }
    EXPECT(isApproximatelyEqual(bothStop.frame(0).tangent, Vector3(1, 2, 0).normalized(), 1e-3f));
}

TEST(framesACurveAtRestHasNoTangent) {
    Vector3 point(1, 2, 3);
    Curve3 still(point, point, point, point);
    EXPECT_PROBLEM(still.unitTangent(0.5f), "The curve does not move near parameter 0.5, so it has no tangent.");
}

// MARK: Bit for bit against the Swift engine
//
// The expected bits were printed by running the Swift sources (Engine/Math/Curves/) with swiftc on
// the same inputs; the core is built without fused multiply-adds so that it matches them exactly.

namespace {

Curve3 parityCurve3() {
    return Curve3(Vector3(0.1f, 0.2f, 0.3f), Vector3(1.3f, 0.1f, -0.7f), Vector3(2.1f, 1.9f, 0.6f), Vector3(3.3f, 0.4f, 1.7f));
}

Curve2 parityCurve2() {
    return Curve2(Vector2(0.1f, 0.2f), Vector2(1.3f, 2.1f), Vector2(2.1f, -1.9f), Vector2(3.3f, 0.4f));
}

} // namespace

TEST(parityEvaluationMatchesSwiftBitForBit) {
    EXPECT(sameBits(parityCurve3().point(0.37f), 0x3fa7747d, 0x3f1b1e85, 0x3c02b410));
    EXPECT(sameBits(parityCurve3().firstDerivative(0.37f), 0x40429888, 0x3fe4245e, 0x3f8a24de));
    EXPECT(sameBits(parityCurve3().secondDerivative(0.37f), 0xbf1fbe74, 0xbe1374a0, 0x41040000));
}

TEST(parityArcLengthMatchesSwiftBitForBit) {
    EXPECT(sameBits(parityCurve3().totalLength(), 0x40885e4c));
    EXPECT(sameBits(parityCurve3().length(0.15f, 0.8f), 0x4022f9da));
    EXPECT(sameBits(parityCurve3().parameterAtDistance(1.7f), 0x3ee7122b));
    std::vector<float> parameters = parityCurve3().evenlySpacedParameters(6);
    std::uint32_t expected[6] = {0x0, 0x3e5fb1b7, 0x3ee7af7c, 0x3f2a2ddf, 0x3f5a25ce, 0x3f800000};
    EXPECT_EQUAL(parameters.size(), size_t(6));
    for (size_t index = 0; index < parameters.size() && index < 6; index += 1) {
        EXPECT(sameBits(parameters[index], expected[index]));
    }
}

TEST(parityCurveFramesMatchSwiftBitForBit) {
    CurveFrame frame = parityCurve3().frame(0.37f);
    EXPECT(sameBits(frame.position, 0x3fa7747d, 0x3f1b1e85, 0x3c02b410));
    EXPECT(sameBits(frame.tangent, 0x3f532c45, 0x3ef793ae, 0x3e95e982));
    EXPECT(sameBits(frame.normal, 0xbe815460, 0xbe179fe7, 0x3f74c7dc));
    // The binormal is tangent × normal. Swift's cross is `simd_cross`, which fuses the multiply and
    // the subtraction; here (no fused operations) its z, a difference of two nearly equal products,
    // lands 1.4e-9 away (0x32000000 against 0x3182657a). So x and y are checked bit for bit and z
    // to within that.
    EXPECT(sameBits(frame.binormal.x, 0x3f017651) && sameBits(frame.binormal.y, 0xbf5cda14));
    EXPECT(isApproximatelyEqual(frame.binormal.z, std::bit_cast<float>(std::uint32_t(0x3182657a)), 1e-8f));

    std::vector<CurveFrame> frames = parityCurve3().rotationMinimizingFrames({0, 0.2f, 0.45f, 0.7f, 1});
    std::uint32_t normals[5][3] = {
        {0x3f22fcc9, 0xbd59510c, 0x3f44f174}, {0x3e6308db, 0xbe11082f, 0x3f76fb3e},
        {0xbe88cd88, 0xbecf2155, 0x3f5fe693}, {0xbf174599, 0xbf046909, 0x3f1e7e75},
        {0xbf569cc2, 0xbeca83dc, 0x3ec01692}};
    std::uint32_t tangents[5][3] = {
        {0x3f44434c, 0xbd82d788, 0xbf238d6a}, {0x3f691d80, 0x3ec4950f, 0xbe1c91e8},
        {0x3f48c9d6, 0x3edf3b5b, 0x3ee1efe3}, {0x3f384954, 0x3c09249b, 0x3f31ade4},
        {0x3f0ac760, 0xbf2d7938, 0x3efe6d86}};
    EXPECT_EQUAL(frames.size(), size_t(5));
    for (size_t index = 0; index < frames.size() && index < 5; index += 1) {
        EXPECT(sameBits(frames[index].normal, normals[index][0], normals[index][1], normals[index][2]));
        EXPECT(sameBits(frames[index].tangent, tangents[index][0], tangents[index][1], tangents[index][2]));
    }
}

TEST(parityFlatCurveAnalysisMatchesSwiftBitForBit) {
    Curve2 curve = parityCurve2();
    EXPECT(sameBits(curve.signedCurvature(0.3f), 0xbf813709));
    EXPECT(sameBits(curve.signedAreaSwept(Vector2(0.5f, -0.25f)), 0xbee56038));
    std::vector<float> inflections = curve.inflectionParameters();
    EXPECT_EQUAL(inflections.size(), size_t(1));
    if (inflections.size() == 1) EXPECT(sameBits(inflections[0], 0x3ef819ac));
    std::vector<float> extrema = curve.extremaParameters();
    EXPECT_EQUAL(extrema.size(), size_t(2));
    if (extrema.size() == 2) {
        EXPECT(sameBits(extrema[0], 0x3e50f92e));
        EXPECT(sameBits(extrema[1], 0x3f435cfb));
    }
    Bounds2 bounds = curve.bounds();
    EXPECT(sameBits(bounds.minimum.x, 0x3dcccccd) && sameBits(bounds.minimum.y, 0xbeac1707));
    EXPECT(sameBits(bounds.maximum.x, 0x40533333) && sameBits(bounds.maximum.y, 0x3f3ad241));
    std::vector<Vector2> polyline = curve.flattened(0.01f);
    EXPECT_EQUAL(polyline.size(), size_t(23));
    if (polyline.size() > 3) EXPECT(sameBits(polyline[3].x, 0x3f121463) && sameBits(polyline[3].y, 0x3f2dd7ea));
    Curve2 piece = curve.portion(0.2f, 0.7f);
    EXPECT(sameBits(piece.start.x, 0x3f474539) && sameBits(piece.startHandle.y, 0x3f3e5c91));
    EXPECT(sameBits(piece.endHandle.x, 0x3fe52bd3) && sameBits(piece.end.y, 0xbe98c7e4));
}

TEST(parityCircularArcMatchesSwiftBitForBit) {
    std::vector<Curve2> arcs = Curve2::circularArc(Vector2(1, 2), 3, Angle::degrees(20), Angle::degrees(200));
    EXPECT_EQUAL(arcs.size(), size_t(3));
    if (arcs.size() == 3) {
        EXPECT(sameBits(arcs[1].startHandle.x, 0xbcac88c0) && sameBits(arcs[1].startHandle.y, 0x40a210d4));
        EXPECT(sameBits(arcs[2].end.x, 0xbfa62938) && sameBits(arcs[2].end.y, 0x3d92b640));
    }
}
