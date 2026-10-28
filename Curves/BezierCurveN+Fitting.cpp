// Lowering, fitting and converting Béziers of any order (graphics_cpp curves/bezn `lower`,
// `fitBezier` and `toCubics`). The matrices are small and solved in Double by
// `LinearSystem::leastSquares`.
#include <algorithm>
#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Numerics/LinearSystem.h"
#include "BezierCurveN.h"

namespace editor {

namespace {

/// The points r (x and y separately) that minimise |rows · r − points|².
std::vector<Vector2> leastSquaresPoints(const LinearSystem::Matrix& rows, const std::vector<Vector2>& points) {
    LinearSystem::Row xs;
    LinearSystem::Row ys;
    for (Vector2 point : points) {
        xs.push_back(point.x);
        ys.push_back(point.y);
    }
    std::vector<double> solvedX = LinearSystem::leastSquares(rows, xs);
    std::vector<double> solvedY = LinearSystem::leastSquares(rows, ys);
    std::vector<Vector2> result;
    for (size_t index = 0; index < solvedX.size(); index += 1) {
        result.push_back(Vector2(static_cast<float>(solvedX[index]), static_cast<float>(solvedY[index])));
    }
    return result;
}

std::vector<double> bernsteinInDouble(int order, double parameter) {
    std::vector<double> weights{1.0};
    for (int level = 1; level <= order; level += 1) {
        weights.push_back(0);
        for (int index = level; index >= 0; index -= 1) {
            weights[index] = (index < level ? weights[index] * (1 - parameter) : 0.0) + (index > 0 ? weights[index - 1] * parameter : 0.0);
        }
    }
    return weights;
}

/// The parameter near `start` where a Bézier comes nearest `target`: Newton's steps on
/// (B − p)·B′ = 0, four of them, kept in 0…1.
float nearestParameterNear(const BezierCurveN& curve, Vector2 target, float start) {
    BezierCurveN velocity = curve.derivative();
    BezierCurveN acceleration = velocity.order() > 0 ? velocity.derivative() : BezierCurveN({Vector2()});
    float parameter = start;
    for (int step = 0; step < 4; step += 1) {
        Vector2 offset = curve.point(parameter) - target;
        Vector2 slope = velocity.point(parameter);
        Vector2 bend = acceleration.point(parameter);
        float numerator = offset.dot(slope);
        float denominator = slope.dot(slope) + offset.dot(bend);
        if (std::fabs(denominator) < 1e-14f) break;
        parameter = std::min(1.0f, std::max(0.0f, parameter - numerator / denominator));
    }
    return parameter;
}

} // namespace

/// The closest curve with one control point fewer (degree reduction by least squares).
///
/// How: raising a curve of order k − 1 to order k is a matrix `M` with k + 1 rows and k columns
/// on the control points, with `M₀₀ = 1`, `M_k,k−1 = 1` and for 0 < i < k: `Mᵢᵢ = 1 − i/k`,
/// `Mᵢ,ᵢ₋₁ = i/k`. The lowered points `r` minimise `|M r − p|²`: the normal equations
/// `MᵀM r = Mᵀp`, solved for x and y (`LinearSystem::leastSquares`). Exact when the curve was
/// raised from the lower order.
///
/// Conventions: a line (order 1) or a point comes back unchanged; the ends are not forced to stay.
///
/// Source: Farin, *Curves and Surfaces for CAGD*, §5.4 (degree reduction by least squares).
///
/// Use when: bringing a quartic from a fit or a raise back to a cubic when an approximation is fine
/// Tags: degree reduction, lower order, least squares reduction, quartic to cubic
/// Node: planned (lowerBezier)
BezierCurveN BezierCurveN::lowered() const {
    int k = order();
    if (k < 2) return *this;
    LinearSystem::Matrix elevation(k + 1, LinearSystem::Row(k, 0.0));
    elevation[0][0] = 1;
    elevation[k][k - 1] = 1;
    for (int index = 1; index < k; index += 1) {
        elevation[index][index] = 1 - static_cast<double>(index) / k;
        elevation[index][index - 1] = static_cast<double>(index) / k;
    }
    return BezierCurveN(leastSquaresPoints(elevation, controlPoints));
}

/// One Bézier of the given order fitted to `points` by least squares, with its ends on the first
/// and last points.
///
/// How: each point starts with the parameter of its share of the polyline's length; the inner
/// control points solve the normal equations of `Σ |B(tⱼ) − pⱼ|²` with the end control points fixed
/// (the Bernstein matrix; `LinearSystem::leastSquares`); then every inner point's parameter moves to
/// its nearest place on the curve (Newton steps on `(B − p)·B′ = 0`) and the solve repeats,
/// `fittingRounds` (5) times. Chord-length parameters alone put an arch's handles 17% of its
/// width off; five rounds bring that to 3%.
///
/// Conventions: y-up plane; `order >= 1`, else throws a `MathProblem`. With `order + 1` points or
/// fewer there are too few points to fit, and the straight curve between the ends comes back, its
/// control points evenly spaced. Needs at least 2 points (throws otherwise). All points equal gives
/// a curve at that point.
///
/// Source: Piegl & Tiller, *The NURBS Book* (1997), §9.4 (least-squares curve fitting) with
/// Schneider's reparameterisation; graphics_cpp curves/bezn `fitBezier`.
///
/// Use when: one smooth piece through many points (a stem's gentle swell, a profile through measured
///   widths) where one curve matters more than matching every point
/// Tags: fit Bezier of order n, least squares Bezier, regression curve, fitBezier, single curve fit
/// Node: planned (fitBezierCurve)
BezierCurveN BezierCurveN::fitted(const std::vector<Vector2>& points, int order) {
    if (order < 1) throw MathProblem("A fitted Bézier needs an order of at least 1, but got " + std::to_string(order) + ".");
    if (points.size() < 2) throw MathProblem("A Bézier fit needs at least 2 points, but got " + std::to_string(points.size()) + ".");
    Vector2 first = points.front();
    Vector2 last = points.back();
    if (static_cast<int>(points.size()) <= order + 1) {
        std::vector<Vector2> straight;
        for (int index = 0; index <= order; index += 1) {
            straight.push_back(Vector2::linearBlend(first, last, static_cast<float>(index) / static_cast<float>(order)));
        }
        return BezierCurveN(straight);
    }
    std::vector<double> runs{0.0};
    for (size_t index = 1; index < points.size(); index += 1) runs.push_back(runs.back() + (points[index] - points[index - 1]).length());
    double total = runs.back() > 0 ? runs.back() : 1.0;
    std::vector<float> parameters;
    for (double run : runs) parameters.push_back(static_cast<float>(run / total));

    std::vector<Vector2> controls;
    for (int round = 0; round < fittingRounds; round += 1) {
        LinearSystem::Matrix inner;
        std::vector<Vector2> remainders;
        for (size_t index = 0; index < points.size(); index += 1) {
            std::vector<double> weights = bernsteinInDouble(order, parameters[index]);
            inner.push_back(order > 1 ? LinearSystem::Row(weights.begin() + 1, weights.begin() + order) : LinearSystem::Row{});
            remainders.push_back(points[index] - first * static_cast<float>(weights[0]) - last * static_cast<float>(weights[order]));
        }
        controls.clear();
        controls.push_back(first);
        if (order > 1) {
            for (Vector2 control : leastSquaresPoints(inner, remainders)) controls.push_back(control);
        }
        controls.push_back(last);
        BezierCurveN curve(controls);
        for (size_t index = 1; index + 1 < points.size(); index += 1) {
            parameters[index] = nearestParameterNear(curve, points[index], parameters[index]);
        }
    }
    return BezierCurveN(controls);
}

/// The curve as cubics: exact when it can be, fitted to `tolerance` otherwise.
///
/// How: a plain curve (no weights, or all equal) of up to four control points is raised to four
/// (`raised`) and returned as one cubic, exactly. Anything of a higher order or rational is
/// sampled at `samples + 1` evenly spaced parameters and fitted with `CubicBezier::fitted` to
/// `tolerance`, as an open stroke. A rational quadratic is a conic: weights `[1, cos(θ/2), 1]` make
/// a circular arc of angle θ.
///
/// Conventions: y-up plane; ends are exact; the error is at the sample points, not between them,
/// so use enough samples for the curve's wiggles. `tolerance` should be positive; `samples` at
/// least 2 (throws a `MathProblem` otherwise).
///
/// Source: graphics_cpp curves/bezn `toCubics`. Deviation: it fits with the Schneider fitter
/// `fitted`, whose tangents are estimated from the samples, not read from the curve.
///
/// Use when: bringing a quartic, a quadratic or a conic into the engine's cubics
/// Tags: to cubics, convert Bezier, rational to cubic, conic to cubic, approximate curve
/// Node: planned (bezierToCubics)
std::vector<CubicBezier<Vector2>> BezierCurveN::asCubics(
    const std::optional<std::vector<float>>& weights, float tolerance, int samples) const {
    if (samples < 2) throw MathProblem("Converting a Bézier to cubics needs at least 2 samples, but got " + std::to_string(samples) + ".");
    bool isPlain = true;
    if (weights) {
        for (float weight : *weights) {
            if (!(std::fabs(weight - (*weights)[0]) < 1e-6f)) {
                isPlain = false;
                break;
            }
        }
    }
    if (isPlain && controlPoints.size() <= 4) {
        BezierCurveN lifted = *this;
        while (lifted.controlPoints.size() < 4) lifted = lifted.raised();
        const std::vector<Vector2>& points = lifted.controlPoints;
        return {CubicBezier<Vector2>(points[0], points[1], points[2], points[3])};
    }
    std::vector<Vector2> sampled;
    for (int index = 0; index <= samples; index += 1) {
        float parameter = static_cast<float>(index) / static_cast<float>(samples);
        sampled.push_back(isPlain ? point(parameter) : rationalPoint(*weights, parameter));
    }
    return CubicBezier<Vector2>::fitted(sampled, tolerance, std::nullopt, std::nullopt, false);
}

} // namespace editor
