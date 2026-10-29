// Tests of the line, circle and ellipse fits (Curves/ShapeFits.h): exact shapes come back to 1e-4,
// noisy ones within the noise, degenerate input gives nothing. Also the step-2 checks of the
// stroke fitter (CubicBezier::fitted) the vector brush relies on.
#include <cmath>
#include <numbers>
#include <vector>

#include "../Curves/CubicBezier.h"
#include "../Curves/ShapeFits.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Curve2 = CubicBezier<Vector2>;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// `count` points of the circle from `startDegrees` through `sweepDegrees`, both ends included.
std::vector<Vector2> circlePoints(Circle2 circle, int count, float startDegrees = 0, float sweepDegrees = 360) {
    std::vector<Vector2> points;
    bool closed = sweepDegrees >= 360;
    int steps = closed ? count : count - 1;
    for (int index = 0; index < count; index += 1) {
        Angle angle = Angle::degrees(startDegrees + sweepDegrees * static_cast<float>(index) / static_cast<float>(steps));
        points.push_back(circle.centre + Vector2(angle.cosine(), angle.sine()) * circle.radius);
    }
    return points;
}

std::vector<Vector2> ellipsePoints(const Ellipse2& ellipse, int count) {
    std::vector<Vector2> points;
    for (int index = 0; index < count; index += 1) {
        points.push_back(ellipse.point(Angle::degrees(360.0f * static_cast<float>(index) / static_cast<float>(count))));
    }
    return points;
}

/// Each point moved by an even random amount up to `amount` in x and in y.
std::vector<Vector2> jittered(std::vector<Vector2> points, float amount, std::uint64_t seed) {
    SeededRandom random(seed);
    for (Vector2& point : points) point = point + Vector2(random.nextFloat(-amount, amount), random.nextFloat(-amount, amount));
    return points;
}

/// The farthest any point is from the fitted pieces (each point's nearest point on them).
float farthestFrom(const std::vector<Vector2>& points, const std::vector<Curve2>& pieces) {
    float farthest = 0;
    for (Vector2 point : points) farthest = std::fmax(farthest, Curve2::nearest(pieces, point, 256).distance);
    return farthest;
}

/// Points along the pieces, `perPiece` from each, ends shared.
std::vector<Vector2> sampled(const std::vector<Curve2>& pieces, int perPiece) {
    std::vector<Vector2> points;
    for (const Curve2& piece : pieces) {
        for (int step = points.empty() ? 0 : 1; step <= perPiece; step += 1) points.push_back(piece.point(static_cast<float>(step) / static_cast<float>(perPiece)));
    }
    return points;
}

} // namespace

// MARK: Line

TEST(fittedLineOfPointsOnALine) {
    // Points on y = 2x + 1: direction (1, 2)/√5, every residual zero.
    std::vector<Vector2> points = {Vector2(0, 1), Vector2(1, 3), Vector2(2, 5), Vector2(4, 9)};
    std::optional<FittedLine> line = fittedLine(points);
    EXPECT(line.has_value());
    EXPECT(isClose(line->direction, Vector2(1, 2) / std::sqrt(5.0f), 1e-4f));
    EXPECT(isClose(line->centroid, Vector2(1.75f, 4.5f), 1e-4f));
    EXPECT(isClose(line->span.start, Vector2(0, 1), 1e-4f) && isClose(line->span.end, Vector2(4, 9), 1e-4f));
    EXPECT(line->rmsResidual < 1e-4f);
}

TEST(fittedLineTreatsVerticalLinesAndKnownScatterAlike) {
    std::optional<FittedLine> vertical = fittedLine({Vector2(3, 5), Vector2(3, 1), Vector2(3, -2)});
    EXPECT(vertical && isClose(vertical->direction, Vector2(0, -1), 1e-5f) && vertical->rmsResidual < 1e-5f);
    // (0, 1), (1, −1), (2, 1), (3, −1): Sxx = 5, Sxy = −2, Syy = 4; the scatter's smaller eigenvalue
    // is (9 − √17)/2, so the residual is √(that/4).
    std::optional<FittedLine> zigzag = fittedLine({Vector2(0, 1), Vector2(1, -1), Vector2(2, 1), Vector2(3, -1)});
    EXPECT(zigzag && isApproximatelyEqual(zigzag->rmsResidual, std::sqrt((9 - std::sqrt(17.0f)) / 8), 1e-5f));
    EXPECT(!fittedLine({Vector2(1, 1)}));
    EXPECT(!fittedLine({Vector2(1, 1), Vector2(1, 1)}));
    EXPECT(!fittedLine({Vector2(1, 1), Vector2(NAN, 1)}));
}

TEST(fittedLineResidualIsTheSmallestOfAnyLine) {
    std::vector<Vector2> points = jittered({Vector2(0, 0), Vector2(1, 0.2f), Vector2(2, 0.5f), Vector2(3, 0.4f), Vector2(4, 1)}, 0.3f, 5);
    std::optional<FittedLine> line = fittedLine(points);
    EXPECT(line.has_value());
    for (int degrees = 0; degrees < 180; degrees += 3) {
        Vector2 direction(Angle::degrees(static_cast<float>(degrees)).cosine(), Angle::degrees(static_cast<float>(degrees)).sine());
        float sum = 0;
        for (Vector2 point : points) sum += std::pow((point - line->centroid).cross(direction), 2.0f);
        EXPECT(std::sqrt(sum / 5) >= line->rmsResidual - 1e-5f);
    }
}

// MARK: Circle

TEST(fittedCircleRecoversAnExactCircle) {
    Circle2 truth{Vector2(3, -2), 5};
    for (std::vector<Vector2> points : {circlePoints(truth, 24), circlePoints(truth, 9, 20, 90)}) {
        std::optional<FittedCircle> circle = fittedCircle(points);
        EXPECT(circle && isClose(circle->circle.centre, truth.centre, 1e-4f));
        EXPECT(circle && isApproximatelyEqual(circle->circle.radius, truth.radius, 1e-4f));
        EXPECT(circle && circle->rmsResidual < 1e-4f);
        std::optional<FittedCircle> algebraic = fittedCircleAlgebraic(points);
        EXPECT(algebraic && isApproximatelyEqual(algebraic->circle.radius, truth.radius, 1e-4f));
    }
    // Three points: the circumcircle, (0, 0), (4, 0), (0, 3) round (2, 1.5) of radius 2.5.
    std::optional<FittedCircle> three = fittedCircle({Vector2(0, 0), Vector2(4, 0), Vector2(0, 3)});
    EXPECT(three && isClose(three->circle.centre, Vector2(2, 1.5f), 1e-4f) && isApproximatelyEqual(three->circle.radius, 2.5f, 1e-4f));
}

TEST(fittedCircleOfNoisyPointsStaysWithinTheNoise) {
    Circle2 truth{Vector2(-1, 4), 10};
    std::vector<Vector2> points = jittered(circlePoints(truth, 200), 0.1f, 11);
    std::optional<FittedCircle> circle = fittedCircle(points);
    EXPECT(circle && circle->circle.centre.distance(truth.centre) < 0.05f);
    EXPECT(circle && std::fabs(circle->circle.radius - truth.radius) < 0.05f);
    // Uniform jitter of ±0.1 in x and y has a radial RMS of 0.1/√3 ≈ 0.058.
    EXPECT(circle && circle->rmsResidual < 0.08f);
}

TEST(fittedCircleGeometricBeatsKasaOnAShortNoisyArc) {
    Circle2 truth{Vector2(0, 0), 10};
    std::vector<Vector2> points = jittered(circlePoints(truth, 60, 60, 50), 0.3f, 19);
    std::optional<FittedCircle> algebraic = fittedCircleAlgebraic(points);
    std::optional<FittedCircle> geometric = fittedCircle(points);
    EXPECT(algebraic && geometric);
    // The geometric fit has the smallest sum of squared distances, so never more than Kåsa's.
    EXPECT(geometric->rmsResidual <= algebraic->rmsResidual + 1e-6f);
}

TEST(fittedCircleRefusesTooFewAndCollinearPoints) {
    EXPECT(!fittedCircle({Vector2(0, 0), Vector2(1, 1)}));
    EXPECT(!fittedCircle({Vector2(0, 0), Vector2(1, 1), Vector2(2, 2), Vector2(3, 3)}));
    EXPECT(!fittedCircleAlgebraic({Vector2(0, 0), Vector2(1, 0), Vector2(2, 0)}));
}

// MARK: Ellipse

TEST(fittedEllipseRecoversAnExactEllipse) {
    Ellipse2 truth{Vector2(2, 1), 4, 1.5f, Angle::degrees(30)};
    std::optional<FittedEllipse> ellipse = fittedEllipse(ellipsePoints(truth, 40));
    EXPECT(ellipse.has_value());
    EXPECT(isClose(ellipse->ellipse.centre, truth.centre, 1e-4f));
    EXPECT(isApproximatelyEqual(ellipse->ellipse.majorRadius, 4, 1e-4f));
    EXPECT(isApproximatelyEqual(ellipse->ellipse.minorRadius, 1.5f, 1e-4f));
    EXPECT(isApproximatelyEqual(ellipse->ellipse.rotation.degrees(), 30, 1e-2f));
    EXPECT(ellipse->rmsResidual < 1e-4f);
    // A steep one, whose rotation comes back in −90°…90°.
    Ellipse2 steep{Vector2(-3, 0), 2, 0.5f, Angle::degrees(-75)};
    std::optional<FittedEllipse> steepFit = fittedEllipse(ellipsePoints(steep, 30));
    EXPECT(steepFit && isApproximatelyEqual(steepFit->ellipse.rotation.degrees(), -75, 1e-2f));
    EXPECT(steepFit && isApproximatelyEqual(steepFit->ellipse.majorRadius, 2, 1e-4f));
}

TEST(fittedEllipseOfACircleHasEqualAxes) {
    std::optional<FittedEllipse> ellipse = fittedEllipse(circlePoints(Circle2{Vector2(1, 1), 3}, 30));
    EXPECT(ellipse && isApproximatelyEqual(ellipse->ellipse.majorRadius, 3, 1e-4f));
    EXPECT(ellipse && isApproximatelyEqual(ellipse->ellipse.minorRadius, 3, 1e-4f));
    EXPECT(ellipse && isClose(ellipse->ellipse.centre, Vector2(1, 1), 1e-4f));
}

TEST(fittedEllipseOfNoisyPointsStaysWithinTheNoise) {
    Ellipse2 truth{Vector2(0, 0), 20, 8, Angle::degrees(-20)};
    std::optional<FittedEllipse> ellipse = fittedEllipse(jittered(ellipsePoints(truth, 200), 0.2f, 29));
    EXPECT(ellipse && ellipse->ellipse.centre.distance(truth.centre) < 0.1f);
    EXPECT(ellipse && std::fabs(ellipse->ellipse.majorRadius - 20) < 0.2f && std::fabs(ellipse->ellipse.minorRadius - 8) < 0.2f);
    EXPECT(ellipse && ellipse->rmsResidual < 0.16f);
}

TEST(fittedEllipseRefusesTooFewAndLinePoints) {
    EXPECT(!fittedEllipse({Vector2(0, 0), Vector2(1, 0), Vector2(0, 1), Vector2(1, 1)}));
    EXPECT(!fittedEllipse({Vector2(0, 0), Vector2(1, 1), Vector2(2, 2), Vector2(3, 3), Vector2(4, 4), Vector2(5, 5)}));
}

// MARK: Stroke fitting (step 2 of the vector features)

TEST(strokeFitOfAStraightStrokeIsOnePiece) {
    std::vector<Vector2> points;
    for (int index = 0; index <= 50; index += 1) points.push_back(Vector2(0.2f * static_cast<float>(index), 0.1f * static_cast<float>(index)));
    std::vector<Curve2> pieces = Curve2::fitted(points, 0.01f);
    EXPECT(pieces.size() == 1);
    EXPECT(farthestFrom(points, pieces) <= 0.01f);
}

TEST(strokeFitStaysWithinTheToleranceOfAHandBuiltStroke) {
    // An S: a half circle up, then a half circle down, sampled every 6°.
    std::vector<Vector2> points = circlePoints(Circle2{Vector2(0, 0), 2}, 31, 180, -180);
    std::vector<Vector2> second = circlePoints(Circle2{Vector2(4, 0), 2}, 31, 180, 180);
    points.insert(points.end(), second.begin() + 1, second.end());
    for (float tolerance : {0.05f, 0.01f, 0.002f}) {
        std::vector<Curve2> pieces = Curve2::fitted(points, tolerance);
        EXPECT(!pieces.empty());
        EXPECT(farthestFrom(points, pieces) <= tolerance * 1.01f);
    }
}

TEST(strokeFitOfASampledCircleStaysWithinThreeHundredthsOfAPercent) {
    // One cubic per quarter circle is off the circle by at most 0.027% of the radius (the classic
    // k = 0.5523 quarter), so four pieces could meet a tolerance of 0.03% of the radius. The
    // fitter splits where its error is largest rather than at the quarters, and its end tangents
    // come from the samples, so it reaches that tolerance with eight pieces (measured: 8 for 72
    // and 360 samples); the check holds it to the tolerance and to at most eight.
    Circle2 circle{Vector2(5, 5), 10};
    for (int count : {72, 360}) {
        std::vector<Vector2> points = circlePoints(circle, count);
        std::vector<Curve2> pieces = Curve2::fitted(points, 0.0003f * circle.radius, std::nullopt, std::nullopt, true);
        EXPECT(!pieces.empty() && pieces.size() <= 8);
        EXPECT(pieces.front().start == pieces.back().end);
        for (const Curve2& piece : pieces) {
            for (int step = 0; step <= 200; step += 1) {
                float radialError = std::fabs(piece.point(static_cast<float>(step) / 200).distance(circle.centre) - circle.radius);
                EXPECT(radialError <= 0.0003f * circle.radius);
            }
        }
    }
}

TEST(strokeRefitOfAFittedCurveDoesNotDrift) {
    std::vector<Vector2> points = circlePoints(Circle2{Vector2(0, 0), 3}, 40, 0, 200);
    float tolerance = 0.01f;
    std::vector<Curve2> first = Curve2::fitted(points, tolerance);
    // Refitting the whole stroke's samples again and again keeps the number of pieces, and the
    // shape stays within twice the tolerance of the drawn points (the joints may slide along it).
    std::vector<Curve2> again = first;
    for (int round = 0; round < 5; round += 1) {
        again = Curve2::fitted(sampled(again, 20), tolerance);
        EXPECT(again.size() == first.size());
        EXPECT(farthestFrom(points, again) <= 2 * tolerance);
    }
    // Refitting each piece's own samples keeps every joint where it was: one piece each, with
    // the same ends, and the same curve to within the tolerance.
    for (const Curve2& piece : first) {
        std::vector<Curve2> refitted = Curve2::fitted(sampled({piece}, 20), tolerance);
        EXPECT(refitted.size() == 1);
        if (refitted.size() != 1) continue;
        EXPECT(isClose(refitted[0].start, piece.start) && isClose(refitted[0].end, piece.end));
        EXPECT(farthestFrom(sampled({piece}, 50), refitted) <= tolerance);
    }
}
