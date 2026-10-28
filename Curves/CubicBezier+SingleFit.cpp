// Fitting one cubic to points between known end directions, and fitting a whole stroke with its
// corners kept (graphics_cpp curves/fit `fitCubic` and `fitCubics`). The many-piece fitter with
// its own tangent estimates is `CubicBezier::fitted` in CubicBezier+Fitting.cpp; these two are
// the pieces around it: the single least-squares piece, and the cut at sharp turns.
#include <algorithm>
#include <cmath>
#include <optional>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "CubicBezier.h"

namespace editor {

namespace {

constexpr int newtonRounds = 8;
/// A handle shorter than this share of the chord is replaced by chord/3.
constexpr double shortestHandleShare = 1e-3;
/// Consecutive points closer than this are one point.
constexpr double duplicateDistance = 1e-9;

struct Point2D {
    double x = 0;
    double y = 0;
    Point2D operator+(Point2D other) const { return {x + other.x, y + other.y}; }
    Point2D operator-(Point2D other) const { return {x - other.x, y - other.y}; }
    Point2D operator*(double scale) const { return {x * scale, y * scale}; }
};

double dotOf(Point2D first, Point2D second) { return first.x * second.x + first.y * second.y; }
double lengthOf(Point2D vector) { return std::sqrt(dotOf(vector, vector)); }
Point2D asDouble(Vector2 vector) { return {static_cast<double>(vector.x), static_cast<double>(vector.y)}; }

/// A cubic in Double: point, first and second derivative at a parameter.
struct DoubleCubic {
    Point2D start, startHandle, endHandle, end;

    Point2D point(double parameter) const {
        double rest = 1 - parameter;
        return start * (rest * rest * rest) + startHandle * (3 * rest * rest * parameter)
            + endHandle * (3 * rest * parameter * parameter) + end * (parameter * parameter * parameter);
    }
    Point2D velocity(double parameter) const {
        double rest = 1 - parameter;
        return (startHandle - start) * (3 * rest * rest) + (endHandle - startHandle) * (6 * rest * parameter)
            + (end - endHandle) * (3 * parameter * parameter);
    }
    Point2D acceleration(double parameter) const {
        double rest = 1 - parameter;
        return (endHandle - startHandle * 2 + start) * (6 * rest) + (end - endHandle * 2 + startHandle) * (6 * parameter);
    }
    /// One Newton step on (B(t) − p)·B′(t) = 0, the parameter of the nearest place; the parameter
    /// stays when the slope vanishes.
    double nearerParameter(double parameter, Point2D target) const {
        Point2D offset = point(parameter) - target;
        Point2D slope = velocity(parameter);
        double denominator = dotOf(slope, slope) + dotOf(offset, acceleration(parameter));
        if (!(denominator > 1e-12)) return parameter;
        return std::min(1.0, std::max(0.0, parameter - dotOf(offset, slope) / denominator));
    }
};

} // namespace

/// One cubic through `points`, from the first to the last, leaving along `startDirection` and
/// arriving with its end handle along `endDirection` (which points back into the curve, from the
/// last point towards the first). Both are unit vectors.
///
/// How: Schneider's least squares. Each point gets a parameter by its share of the polyline's
/// length. With the end points fixed and the handles `A + tA·αₐ`, `B + tB·α_b` along the given
/// directions, the squared distances are quadratic in the two lengths, so they come from a 2×2
/// system (the normal equations). Then every parameter takes a Newton step towards its nearest
/// place on the curve, and the solve repeats, eight times; the round with the least error wins.
/// A handle that comes out shorter than a thousandth of the chord (or negative) is replaced by
/// a third of the chord for both.
///
///     A ●══ tA →╮                ╭← tB ══● B     the handle lengths are the unknowns
///               ╰─── points ─────╯
///
/// Conventions: y-up plane; the error is the largest distance from a point to the curve after
/// the last Newton step. Repeated points (closer than 1e-9) count once. No points give a
/// zero cubic; one point a cubic standing there; two points a straight cubic with handles at
/// the thirds along the directions. Throws nothing.
///
/// Source: Schneider, "An algorithm for automatically fitting digitized curves", *Graphics
/// Gems* (1990); graphics_cpp curves/fit `fitCubic`.
///
/// Use when: fitting one smooth piece to a run of samples whose end directions are known (a
///   stem's rail between two joints); checking how well one cubic can follow measured points
/// Tags: fit cubic, Schneider, least squares Bezier, single piece fit, handle lengths
/// Node: planned (fitCubicPiece)
template <CurvePoint PointType>
FittedCubic CubicBezier<PointType>::fittedSinglePiece(
    const std::vector<Vector2>& points, Vector2 startDirection, Vector2 endDirection)
    requires std::same_as<PointType, Vector2>
{
    if (points.empty()) return FittedCubic{CubicBezier<Vector2>(), 0};
    std::vector<Point2D> distinct;
    for (Vector2 point : points) {
        Point2D candidate = asDouble(point);
        if (distinct.empty() || lengthOf(candidate - distinct.back()) > duplicateDistance) distinct.push_back(candidate);
    }
    Vector2 first = points.front();
    Vector2 last = points.back();
    if (distinct.size() < 2) return FittedCubic{CubicBezier<Vector2>(first, first, last, last), 0};
    distinct.back() = asDouble(last);
    Point2D startPoint = distinct.front();
    Point2D endPoint = distinct.back();
    double chord = lengthOf(endPoint - startPoint);
    Point2D tangentA = asDouble(startDirection);
    Point2D tangentB = asDouble(endDirection);
    auto floatCubic = [&](Point2D startHandle, Point2D endHandle) {
        return CubicBezier<Vector2>(
            first, Vector2(static_cast<float>(startHandle.x), static_cast<float>(startHandle.y)),
            Vector2(static_cast<float>(endHandle.x), static_cast<float>(endHandle.y)), last);
    };
    if (distinct.size() == 2) {
        return FittedCubic{floatCubic(startPoint + tangentA * (chord / 3), endPoint + tangentB * (chord / 3)), 0};
    }
    std::vector<double> parameters{0.0};
    for (size_t index = 1; index < distinct.size(); index += 1) {
        parameters.push_back(parameters.back() + lengthOf(distinct[index] - distinct[index - 1]));
    }
    double total = parameters.back();
    for (double& parameter : parameters) parameter /= total;

    std::optional<DoubleCubic> bestCubic;
    double bestError = 0;
    for (int round = 0; round < newtonRounds; round += 1) {
        double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
        for (size_t index = 0; index < distinct.size(); index += 1) {
            double parameter = parameters[index];
            double rest = 1 - parameter;
            double blendStart = rest * rest * rest;
            double blendFirst = 3 * rest * rest * parameter;
            double blendSecond = 3 * rest * parameter * parameter;
            double blendEnd = parameter * parameter * parameter;
            Point2D startTerm = tangentA * blendFirst;
            Point2D endTerm = tangentB * blendSecond;
            Point2D remainder = distinct[index] - startPoint * (blendStart + blendFirst) - endPoint * (blendSecond + blendEnd);
            a11 += dotOf(startTerm, startTerm);
            a12 += dotOf(startTerm, endTerm);
            a22 += dotOf(endTerm, endTerm);
            b1 += dotOf(remainder, startTerm);
            b2 += dotOf(remainder, endTerm);
        }
        double determinant = a11 * a22 - a12 * a12;
        double startLength = determinant != 0 ? (b1 * a22 - b2 * a12) / determinant : 0;
        double endLength = determinant != 0 ? (a11 * b2 - a12 * b1) / determinant : 0;
        if (!(startLength > chord * shortestHandleShare) || !(endLength > chord * shortestHandleShare)) {
            startLength = endLength = chord / 3;
        }
        startLength = std::min(startLength, chord);
        endLength = std::min(endLength, chord);
        DoubleCubic cubic{startPoint, startPoint + tangentA * startLength, endPoint + tangentB * endLength, endPoint};
        double error = 0;
        for (size_t index = 0; index < distinct.size(); index += 1) {
            parameters[index] = cubic.nearerParameter(parameters[index], distinct[index]);
            error = std::max(error, lengthOf(cubic.point(parameters[index]) - distinct[index]));
        }
        if (!bestCubic || error < bestError) {
            bestCubic = cubic;
            bestError = error;
        }
    }
    return FittedCubic{floatCubic(bestCubic->startHandle, bestCubic->endHandle), static_cast<float>(bestError)};
}

/// Cubics through a stroke to within `tolerance`, cut wherever the stroke turns by more than
/// `cornerAngle`, so a corner stays a corner instead of being rounded off.
///
/// How: repeated points are dropped; at every inner point the turn between the step in and the
/// step out is `acos(clamp(a·b, −1, 1))` of the unit steps `a`, `b`; where it exceeds the
/// limit the run is cut. Each run between cuts is fitted by `fitted` (Schneider with split at
/// the worst point), as an open stroke, and the pieces are joined in order. Fewer than two
/// distinct points give no pieces.
///
///     smooth run ──────╮ turn > cornerAngle: cut here, the corner stays sharp
///                      ╰──── next run
///
/// Conventions: y-up plane; `cornerAngle` of 180° or more never cuts. Each run keeps the
/// tolerance of `fitted`; the run ends are input points. Deviation from graphics_cpp
/// `fitCubics`: its end tangents come from the first two points of a run, here `fitted`
/// estimates them from a parabola through nearby points, which is steadier on noisy strokes.
///
/// Source: Schneider (1990), as in `fitted`; the corner cut is graphics_cpp curves/fit
/// `fitCubics` (60° by default).
///
/// Use when: turning a hand-drawn outline with real corners (a star, a letter) into cubics
///   without rounding the corners; fitting a flattened path back
/// Tags: fit cubics, corner detection, split at corners, Schneider, stroke to curves
/// Node: planned (fitCubicsWithCorners)
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::fittedWithCorners(
    const std::vector<Vector2>& points, float tolerance, Angle cornerAngle)
    requires std::same_as<PointType, Vector2>
{
    std::vector<Vector2> distinct = withoutDuplicates(points);
    if (distinct.size() < 2) return {};
    std::vector<size_t> cuts{0};
    for (size_t index = 1; index + 1 < distinct.size(); index += 1) {
        Vector2 stepIn = (distinct[index] - distinct[index - 1]).normalized();
        Vector2 stepOut = (distinct[index + 1] - distinct[index]).normalized();
        if (Angle::arcCosine(stepIn.dot(stepOut)).radians > cornerAngle.radians) cuts.push_back(index);
    }
    cuts.push_back(distinct.size() - 1);
    std::vector<CubicBezier<Vector2>> result;
    for (size_t run = 0; run + 1 < cuts.size(); run += 1) {
        std::vector<Vector2> stretch(distinct.begin() + static_cast<long>(cuts[run]), distinct.begin() + static_cast<long>(cuts[run + 1]) + 1);
        std::vector<CubicBezier<Vector2>> pieces = fitted(stretch, tolerance, std::nullopt, std::nullopt, false);
        result.insert(result.end(), pieces.begin(), pieces.end());
    }
    return result;
}

template class CubicBezier<Vector2>;

} // namespace editor
