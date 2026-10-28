// Tests of Numerics/LinearSystem and Curves/BezierCurveN (graphics_cpp curves/bezn): hand-worked
// values, the golden cases of Curves2D/tests/golden/bezn.json, and the properties that must always hold.
#include <cmath>
#include <vector>

#include "../Curves/BezierCurveN.h"
#include "../Numerics/LinearSystem.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

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

const std::vector<Vector2> cubicPoints = {Vector2(0, 0), Vector2(0, 100), Vector2(100, 100), Vector2(0, 75)};
const std::vector<Vector2> quarticPoints = {Vector2(0, 0), Vector2(20, 80), Vector2(50, -30), Vector2(80, 90), Vector2(100, 10)};

} // namespace

TEST(linearSystemSolvesSmallSystems) {
    // 2x + y = 3, x + 3y = 5 has x = 0.8, y = 1.4 (golden bezn solve).
    std::vector<double> solution = LinearSystem::solved({{2, 1}, {1, 3}}, {3, 5});
    EXPECT(std::fabs(solution[0] - 0.8) < 1e-12 && std::fabs(solution[1] - 1.4) < 1e-12);
    // A row that needs a swap: the first pivot is 0.
    std::vector<double> swapped = LinearSystem::solved({{0, 1}, {1, 0}}, {2, 3});
    EXPECT(swapped[0] == 3 && swapped[1] == 2);
    // The singular system [[1, 2], [2, 4]] x = [3, 6] leaves its second unknown at 0 (golden).
    std::vector<double> singular = LinearSystem::solved({{1, 2}, {2, 4}}, {3, 6});
    EXPECT(std::fabs(singular[0] - 3) < 1e-12 && singular[1] == 0);
}

TEST(linearSystemSolutionSatisfiesTheEquations) {
    LinearSystem::Matrix matrix = {{4, -2, 1, 0}, {-2, 5, 3, 1}, {1, 3, 6, -2}, {0, 1, -2, 7}};
    LinearSystem::Row constants = {3, -1, 4, 2};
    std::vector<double> solution = LinearSystem::solved(matrix, constants);
    for (size_t row = 0; row < 4; row += 1) {
        double sum = 0;
        for (size_t column = 0; column < 4; column += 1) sum += matrix[row][column] * solution[column];
        EXPECT(std::fabs(sum - constants[row]) < 1e-10);
    }
    EXPECT_PROBLEM(LinearSystem::solved({{1, 2}, {3, 4}}, {1, 2, 3}), "as many equations");
    EXPECT_PROBLEM(LinearSystem::solved({{1, 2, 3}, {3, 4, 5}}, {1, 2}), "square matrix");
}

TEST(linearSystemLeastSquaresFitsALine) {
    // The line y = 2x + 1 through (0, 1), (1, 3), (2, 5), (3, 7): rows [1, x], constants y.
    std::vector<double> exact = LinearSystem::leastSquares({{1, 0}, {1, 1}, {1, 2}, {1, 3}}, {1, 3, 5, 7});
    EXPECT(std::fabs(exact[0] - 1) < 1e-10 && std::fabs(exact[1] - 2) < 1e-10);
    // With a noisy point the residual is orthogonal to every column: Aᵀ(A x − b) = 0.
    LinearSystem::Matrix rows = {{1, 0}, {1, 1}, {1, 2}, {1, 3}};
    LinearSystem::Row noisy = {1, 3.5, 5, 6.5};
    std::vector<double> fit = LinearSystem::leastSquares(rows, noisy);
    for (size_t column = 0; column < 2; column += 1) {
        double dotProduct = 0;
        for (size_t row = 0; row < 4; row += 1) dotProduct += rows[row][column] * (rows[row][0] * fit[0] + rows[row][1] * fit[1] - noisy[row]);
        EXPECT(std::fabs(dotProduct) < 1e-10);
    }
    EXPECT_PROBLEM(LinearSystem::leastSquares({{1, 2}, {3}}, {1, 2}), "same number of entries");
}

TEST(bernsteinWeightsMatchPascalAndSumToOne) {
    // Order 2 at ½ is (1/4, 1/2, 1/4) (golden); order 3 at ⅓ is (8, 12, 6, 1)/27.
    std::vector<float> weights = BezierCurveN::bernsteinWeights(2, 0.5f);
    EXPECT(isApproximatelyEqual(weights[0], 0.25f) && isApproximatelyEqual(weights[1], 0.5f) && isApproximatelyEqual(weights[2], 0.25f));
    std::vector<float> cubicWeights = BezierCurveN::bernsteinWeights(3, 1.0f / 3);
    EXPECT(isApproximatelyEqual(cubicWeights[0], 8.0f / 27) && isApproximatelyEqual(cubicWeights[1], 12.0f / 27)
           && isApproximatelyEqual(cubicWeights[2], 6.0f / 27) && isApproximatelyEqual(cubicWeights[3], 1.0f / 27));
    EXPECT(BezierCurveN::bernsteinWeights(0, 0.3f) == std::vector<float>{1.0f});
    for (int order = 0; order <= 8; order += 1) {
        float sum = 0;
        for (float weight : BezierCurveN::bernsteinWeights(order, 0.37f)) sum += weight;
        EXPECT(isApproximatelyEqual(sum, 1.0f, 1e-5f));
    }
    EXPECT_PROBLEM(BezierCurveN::bernsteinWeights(-1, 0.5f), "cannot be negative");
}

TEST(pointOfOrderNMatchesTheCubicAndHandWorkedValues) {
    BezierCurveN curve(cubicPoints);
    // At ½ a cubic is (p0 + 3p1 + 3p2 + p3)/8: x = 300/8 = 37.5, y = (300 + 300 + 75)/8 = 84.375.
    EXPECT(isClose(curve.point(0.5f), Vector2(37.5f, 84.375f)));
    EXPECT(curve.point(0) == cubicPoints.front() && isClose(curve.point(1), cubicPoints.back()));
    CubicBezier<Vector2> cubic(cubicPoints[0], cubicPoints[1], cubicPoints[2], cubicPoints[3]);
    for (float parameter : {0.1f, 0.33f, 0.8f}) EXPECT(isClose(curve.point(parameter), cubic.point(parameter), 1e-3f));
    // A single point is a constant curve; no points is refused.
    EXPECT(BezierCurveN({Vector2(4, 5)}).point(0.7f) == Vector2(4, 5));
    EXPECT_PROBLEM(BezierCurveN(std::vector<Vector2>{}), "at least one control point");
    EXPECT(BezierCurveN::fromCubic(cubic).controlPoints == cubicPoints);
}

TEST(deCasteljauLevelsEndInThePointAndSplitSharesIt) {
    BezierCurveN curve(quarticPoints);
    std::vector<std::vector<Vector2>> levels = curve.deCasteljauLevels(0.4f);
    EXPECT(levels.size() == 5 && levels[0] == quarticPoints && levels[4].size() == 1);
    EXPECT(isClose(levels[1][0], Vector2::linearBlend(quarticPoints[0], quarticPoints[1], 0.4f)));
    EXPECT(isClose(levels.back().front(), curve.point(0.4f)));
    BezierCurveNSplit halves = curve.split(0.4f);
    EXPECT(halves.first.order() == 4 && halves.second.order() == 4);
    EXPECT(isClose(halves.first.point(1), curve.point(0.4f)) && isClose(halves.second.point(0), curve.point(0.4f)));
    // The first part at u is the whole at 0.4u; the second at u is the whole at 0.4 + 0.6u.
    for (float parameter : {0.25f, 0.5f, 0.9f}) {
        EXPECT(isClose(halves.first.point(parameter), curve.point(0.4f * parameter)));
        EXPECT(isClose(halves.second.point(parameter), curve.point(0.4f + 0.6f * parameter)));
    }
    // Splitting at 0 gives a point curve and the whole curve (golden splitN).
    BezierCurveNSplit atStart = BezierCurveN({Vector2(0, 0), Vector2(10, 10), Vector2(20, 20), Vector2(30, 30)}).split(0);
    EXPECT(atStart.first.controlPoints == std::vector<Vector2>(4, Vector2(0, 0)));
    EXPECT(atStart.second.controlPoints[3] == Vector2(30, 30));
}

TEST(derivativeIsTheHodograph) {
    // Golden: order · differences of [[0,0],[0,100],[100,100],[0,75]].
    BezierCurveN derivative = BezierCurveN(cubicPoints).derivative();
    EXPECT(derivative.controlPoints == (std::vector<Vector2>{Vector2(0, 300), Vector2(300, 0), Vector2(-300, -75)}));
    EXPECT(BezierCurveN({Vector2(0, 0), Vector2(10, 20)}).derivative().controlPoints == std::vector<Vector2>{Vector2(10, 20)});
    // The hodograph's point is the slope: it matches a central difference of the curve.
    BezierCurveN curve(quarticPoints);
    BezierCurveN velocity = curve.derivative();
    float step = 1e-3f;
    for (float parameter : {0.2f, 0.5f, 0.7f}) {
        Vector2 slope = (curve.point(parameter + step) - curve.point(parameter - step)) / (2 * step);
        EXPECT(isClose(velocity.point(parameter), slope, 0.05f));
    }
    EXPECT_PROBLEM(BezierCurveN({Vector2(1, 1)}).derivative(), "no derivative");
}

TEST(raisedKeepsTheCurve) {
    // Golden: the cubic [[0,0],[0,100],[100,100],[0,75]] raised to a quartic.
    BezierCurveN raised = BezierCurveN(cubicPoints).raised();
    std::vector<Vector2> expected = {Vector2(0, 0), Vector2(0, 75), Vector2(50, 100), Vector2(75, 93.75f), Vector2(0, 75)};
    EXPECT(arePointsClose(raised.controlPoints, expected));
    // A line becomes three points with the middle at the midpoint (golden).
    EXPECT(BezierCurveN({Vector2(0, 0), Vector2(10, 20)}).raised().controlPoints
           == (std::vector<Vector2>{Vector2(0, 0), Vector2(5, 10), Vector2(10, 20)}));
    for (float parameter : {0.0f, 0.15f, 0.5f, 0.95f, 1.0f}) {
        EXPECT(isClose(raised.point(parameter), BezierCurveN(cubicPoints).point(parameter), 1e-3f));
        EXPECT(isClose(BezierCurveN(quarticPoints).raised().point(parameter), BezierCurveN(quarticPoints).point(parameter), 1e-3f));
    }
}

TEST(loweredIsTheLeastSquaresReduction) {
    // Golden: the quartic [[0,0],[20,80],[50,-30],[80,90],[100,10]] reduced to a cubic.
    std::vector<Vector2> expected = {Vector2(0, 12.142857f), Vector2(26.666667f, 37.857143f), Vector2(73.333333f, 47.857143f), Vector2(100, 22.142857f)};
    EXPECT(arePointsClose(BezierCurveN(quarticPoints).lowered().controlPoints, expected, 2e-3f));
    // Golden: the cubic [[0,0],[0,100],[100,100],[0,75]] reduced to a quadratic.
    std::vector<Vector2> quadratic = {Vector2(-15, 3.75f), Vector2(75, 131.25f), Vector2(15, 71.25f)};
    EXPECT(arePointsClose(BezierCurveN(cubicPoints).lowered().controlPoints, quadratic, 2e-3f));
    // Exact when the curve was raised from the lower order: lowered(raised(c)) == c.
    BezierCurveN round = BezierCurveN(quarticPoints).raised().lowered();
    EXPECT(arePointsClose(round.controlPoints, quarticPoints, 2e-3f));
    // A line is unchanged.
    EXPECT(BezierCurveN({Vector2(0, 0), Vector2(10, 20)}).lowered().controlPoints.size() == 2);
}

TEST(rationalPointPullsTowardsHeavyControls) {
    // Golden: weights (1, 0.2, 1) at 0.25 on the quadratic [[0,0],[50,100],[100,0]].
    BezierCurveN quadratic({Vector2(0, 0), Vector2(50, 100), Vector2(100, 0)});
    EXPECT(isClose(quadratic.rationalPoint({1, 0.2f, 1}, 0.25f), Vector2(14.285714f, 10.714286f), 2e-3f));
    // Golden: a quartic with weights (1, 2, 3, 2, 1) at ½.
    EXPECT(isClose(BezierCurveN(quarticPoints).rationalPoint({1, 2, 3, 2, 1}, 0.5f), Vector2(50, 23.055556f), 2e-3f));
    // All weights 1 (or none) is the plain curve.
    EXPECT(isClose(quadratic.rationalPoint({1, 1, 1}, 0.3f), quadratic.point(0.3f), 1e-4f));
    EXPECT(isClose(quadratic.rationalPoint({}, 0.3f), quadratic.point(0.3f), 1e-4f));
    // A conic: weights (1, √2/2, 1) on (1,0), (1,1), (0,1) trace the unit quarter circle.
    BezierCurveN quarterCircle({Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)});
    for (float parameter : {0.1f, 0.4f, 0.5f, 0.77f}) {
        EXPECT(isApproximatelyEqual(quarterCircle.rationalPoint({1, std::sqrt(0.5f), 1}, parameter).length(), 1.0f, 1e-5f));
    }
}

TEST(fittedFindsTheCurveOfTheGivenOrder) {
    // Golden bezn fitBezier: 13 points of a symmetric arch, fitted by a cubic.
    std::vector<Vector2> arch = {
        Vector2(0, 0), Vector2(1.9675926f, 22.916667f), Vector2(7.4074074f, 41.666667f), Vector2(15.625f, 56.25f),
        Vector2(25.925926f, 66.666667f), Vector2(37.615741f, 72.916667f), Vector2(50, 75), Vector2(62.384259f, 72.916667f),
        Vector2(74.074074f, 66.666667f), Vector2(84.375f, 56.25f), Vector2(92.592593f, 41.666667f),
        Vector2(98.032407f, 22.916667f), Vector2(100, 0)};
    BezierCurveN cubicFit = BezierCurveN::fitted(arch, 3);
    std::vector<Vector2> expectedCubic = {Vector2(0, 0), Vector2(-3.149063f, 98.821736f), Vector2(103.149063f, 98.821736f), Vector2(100, 0)};
    EXPECT(arePointsClose(cubicFit.controlPoints, expectedCubic, 0.05f));
    // The same arch by a quadratic (golden: the apex control at 161.24), a symmetric result, ends exact.
    BezierCurveN quadraticFit = BezierCurveN::fitted(arch, 2);
    EXPECT(quadraticFit.order() == 2 && quadraticFit.controlPoints.front() == arch.front() && quadraticFit.controlPoints.back() == arch.back());
    EXPECT(isClose(quadraticFit.controlPoints[1], Vector2(50, 161.24268f), 0.05f));
    // Symmetric points give symmetric handles: the two inner x's add up to the width.
    EXPECT(isApproximatelyEqual(cubicFit.controlPoints[1].x + cubicFit.controlPoints[2].x, 100.0f, 1e-2f));
    EXPECT(isApproximatelyEqual(cubicFit.controlPoints[1].y, cubicFit.controlPoints[2].y, 1e-2f));
    // Golden: five points zig-zag fitted by a quadratic.
    BezierCurveN zigzag = BezierCurveN::fitted({Vector2(0, 0), Vector2(10, 5), Vector2(20, 0), Vector2(30, 5), Vector2(40, 0)}, 2);
    EXPECT(arePointsClose(zigzag.controlPoints, {Vector2(0, 0), Vector2(20, 7.046235f), Vector2(40, 0)}, 0.05f));
    // Points on a line fitted by order 1 are the line; too few points give an even straight curve.
    BezierCurveN line = BezierCurveN::fitted({Vector2(0, 0), Vector2(1, 1), Vector2(2, 2), Vector2(3, 3)}, 1);
    EXPECT(line.controlPoints == (std::vector<Vector2>{Vector2(0, 0), Vector2(3, 3)}));
    BezierCurveN few = BezierCurveN::fitted({Vector2(0, 0), Vector2(10, 5)}, 3);
    EXPECT(arePointsClose(few.controlPoints, {Vector2(0, 0), Vector2(10.0f / 3, 5.0f / 3), Vector2(20.0f / 3, 10.0f / 3), Vector2(10, 5)}));
    EXPECT_PROBLEM(BezierCurveN::fitted({Vector2(0, 0)}, 3), "at least 2 points");
    EXPECT_PROBLEM(BezierCurveN::fitted({Vector2(0, 0), Vector2(1, 1)}, 0), "order of at least 1");
}

TEST(asCubicsIsExactForLowOrdersAndFittedOtherwise) {
    // A cubic stays as it is (golden toCubics); a line is raised to the thirds.
    BezierCurveN cubic({Vector2(0, 0), Vector2(150, 100), Vector2(-50, 100), Vector2(100, 0)});
    std::vector<CubicBezier<Vector2>> same = cubic.asCubics();
    EXPECT(same.size() == 1 && same[0].startHandle == Vector2(150, 100) && same[0].endHandle == Vector2(-50, 100));
    std::vector<CubicBezier<Vector2>> line = BezierCurveN({Vector2(0, 0), Vector2(10, 20)}).asCubics();
    EXPECT(isClose(line[0].startHandle, Vector2(10.0f / 3, 20.0f / 3)) && isClose(line[0].endHandle, Vector2(20.0f / 3, 40.0f / 3)));
    // A quadratic raised to a cubic traces the same curve.
    BezierCurveN quadratic({Vector2(0, 0), Vector2(50, 100), Vector2(100, 0)});
    std::vector<CubicBezier<Vector2>> raised = quadratic.asCubics();
    EXPECT(raised.size() == 1 && isClose(raised[0].point(0.3f), quadratic.point(0.3f), 1e-3f));
    // A quartic is fitted: every sample of it lies within the tolerance of the cubics (golden: tol 0.05).
    BezierCurveN quartic(quarticPoints);
    std::vector<CubicBezier<Vector2>> fitted = quartic.asCubics(std::nullopt, 0.05f, 64);
    EXPECT(fitted.size() >= 2);
    EXPECT(fitted.front().start == quarticPoints.front() && isClose(fitted.back().end, quarticPoints.back()));
    for (int index = 0; index <= 64; index += 1) {
        Vector2 sample = quartic.point(static_cast<float>(index) / 64);
        EXPECT(CubicBezier<Vector2>::nearest(fitted, sample).distance < 0.06f);
    }
    // A conic with weights (1, √2/2, 1) on a quarter circle of radius 100 comes out within a tenth of it.
    BezierCurveN conic({Vector2(100, 0), Vector2(100, 100), Vector2(0, 100)});
    std::vector<CubicBezier<Vector2>> arc = conic.asCubics(std::vector<float>{1, std::sqrt(0.5f), 1}, 0.05f, 48);
    for (const CubicBezier<Vector2>& piece : arc) {
        for (float parameter : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) EXPECT(isApproximatelyEqual(piece.point(parameter).length(), 100.0f, 0.1f));
    }
    EXPECT_PROBLEM(conic.asCubics(std::nullopt, 0.1f, 1), "at least 2 samples");
}
