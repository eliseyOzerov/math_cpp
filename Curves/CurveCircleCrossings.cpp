// Circle crossings of a cubic, and the free-function name for two cubics' crossings.
#include "CurveCircleCrossings.h"

#include <cmath>

#include "../Linear/MathProblem.h"

namespace editor {

namespace {

/// The squared distance from `centre` to the curve at `parameter`, less the squared radius: zero on
/// the circle, negative inside it, positive outside.
float heightAboveCircle(const CubicBezier<Vector2>& curve, Vector2 centre, float radius, float parameter) {
    return (curve.point(parameter) - centre).lengthSquared() - radius * radius;
}

/// How many halvings pin a crossing down: more than a float has bits.
constexpr int bisectionSteps = 40;

} // namespace

/// Where `curve` crosses the circle of `radius` round `centre`: each crossing as its parameter on the
/// curve and its point, in order of parameter.
///
/// How: the height f(t) = |B(t) − centre|² − r² is a degree 6 polynomial in t that is zero exactly on
/// the circle. It is sampled at `samples` + 1 equal steps of t; between two samples where its sign
/// changes there is a root, found by bisecting the step 40 times. A sample that lands on the circle
/// (within 1e-6·r² of zero) is a crossing itself.
///
///          ╭────╮         the curve in and out of the circle
///     ─────┼──●─┼───●──   each ● is a sign change of f
///          ╰────╯
///
/// Conventions: y-up plane; `radius` must be greater than zero or the call throws a `MathProblem`.
/// A curve that only grazes the circle between two samples without a sign change on either side is
/// missed, so raise `samples` for a curve that runs nearly along the circle; the ends of the curve
/// count when they lie on the circle. `samples` less than 1 is taken as 1.
///
/// Source: graphics_cpp curves/intersect.cpp (sampling and bisecting the squared distance).
///
/// Use when: where a dot's clearance circle meets a stem; where a serif of a given reach lands on a
///   bowl; the edge of a brush circle on a stroke
/// Tags: circle intersection, curve circle crossing, ray circle, trim at radius, brush reach
/// Node: planned (circleCrossings)
std::vector<CircleCrossing> circleCrossings(
    const CubicBezier<Vector2>& curve, Vector2 centre, float radius, int samples) {
    if (!(radius > 0)) throw MathProblem("A circle needs a radius greater than zero.");
    if (samples < 1) samples = 1;
    float onTheCircle = 1e-6f * radius * radius;
    std::vector<CircleCrossing> crossings;
    float previousParameter = 0;
    float previousHeight = heightAboveCircle(curve, centre, radius, 0);
    if (std::fabs(previousHeight) < onTheCircle) crossings.push_back({0, curve.start});
    for (int step = 1; step <= samples; step += 1) {
        float parameter = static_cast<float>(step) / static_cast<float>(samples);
        float height = heightAboveCircle(curve, centre, radius, parameter);
        if (std::fabs(height) < onTheCircle) {
            crossings.push_back({parameter, step == samples ? curve.end : curve.point(parameter)});
        } else if (previousHeight * height < 0 && std::fabs(previousHeight) >= onTheCircle) {
            float low = previousParameter;
            float high = parameter;
            float lowHeight = previousHeight;
            for (int bisection = 0; bisection < bisectionSteps; bisection += 1) {
                float middle = (low + high) / 2;
                float middleHeight = heightAboveCircle(curve, centre, radius, middle);
                if (lowHeight * middleHeight <= 0) {
                    high = middle;
                } else {
                    low = middle;
                    lowHeight = middleHeight;
                }
            }
            float root = (low + high) / 2;
            crossings.push_back({root, curve.point(root)});
        }
        previousParameter = parameter;
        previousHeight = height;
    }
    return crossings;
}

/// Where two cubics cross, as pairs of parameters and the point, or nothing where they run along each
/// other (more crossings than two cubics can have, see `CubicBezier::mostCrossingsOfTwoCubics`).
/// The same search as `CubicBezier::crossings(other)`, under the name every intersection is found by.
///
/// Use when: cutting two outlines where they meet; marking where a road crosses a river
/// Tags: curve intersection, bezier bezier crossing, intersect curves, curve curve
/// Node: inside profileCrossings
std::optional<std::vector<CurveCrossing>> curveCrossings(
    const CubicBezier<Vector2>& first, const CubicBezier<Vector2>& second) {
    return first.crossings(second);
}

} // namespace editor
