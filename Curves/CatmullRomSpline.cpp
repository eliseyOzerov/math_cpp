// The centripetal Catmull–Rom spline (Engine/Math/Curves/CatmullRomSpline.swift).
#include "CatmullRomSpline.h"

#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

CatmullRomSpline::CatmullRomSpline(const std::vector<Vector3>& points) {
    if (!(points.size() >= 2)) throw MathProblem("A spline needs at least two points.");
    std::vector<Vector3> distinctPoints = {points[0]};
    for (size_t index = 1; index < points.size(); index += 1) {
        if (points[index].distance(distinctPoints[distinctPoints.size() - 1]) > Tolerance::geometric) {
            distinctPoints.push_back(points[index]);
        }
    }
    throughPoints = distinctPoints;

    if (!(distinctPoints.size() >= 2)) {
        Vector3 point = distinctPoints[0];
        segments = {CubicBezier<Vector3>(point, point, point, point)};
        return;
    }

    size_t last = distinctPoints.size() - 1;
    Vector3 phantomBefore = phantomPoint(distinctPoints[0], distinctPoints[1]);
    Vector3 phantomAfter = phantomPoint(distinctPoints[last], distinctPoints[last - 1]);
    std::vector<Vector3> paddedPoints = {phantomBefore};
    paddedPoints.insert(paddedPoints.end(), distinctPoints.begin(), distinctPoints.end());
    paddedPoints.push_back(phantomAfter);
    for (size_t index = 0; index < last; index += 1) {
        segments.push_back(segment(paddedPoints[index], paddedPoints[index + 1], paddedPoints[index + 2], paddedPoints[index + 3]));
    }
}

/// The point on the spline at `parameter`, found from the Catmull–Rom piece that
/// contains it.
///
/// Use when: placing a lamp post, car or camera on a smooth path through points
/// Tags: point on spline, evaluate spline, spline position, sample curve
/// Node: inside curveToMesh, sweep
Vector3 CatmullRomSpline::point(float parameter) const {
    SegmentLocation location = segmentLocation(parameter);
    return segments[location.index].point(location.localParameter);
}

/// The velocity per unit of spline parameter. Every piece spans exactly one unit of
/// parameter, so this is the piece's own derivative, with no rescaling.
///
/// Use when: finding which way a road or camera path points; scaling a road width
///   cross-section along a spline
/// Tags: spline tangent, derivative, velocity, direction of travel, spline direction
/// Node: inside curveToMesh, curveFrames
Vector3 CatmullRomSpline::firstDerivative(float parameter) const {
    SegmentLocation location = segmentLocation(parameter);
    return segments[location.index].firstDerivative(location.localParameter);
}

/// Which piece a parameter falls in, and where inside that piece (0…1 for parameters
/// inside the range). Out-of-range parameters use the nearest end piece.
/// The parameter must be finite: Swift traps on NaN or infinity when converting to a piece
/// index; here a `MathProblem` is thrown.
CatmullRomSpline::SegmentLocation CatmullRomSpline::segmentLocation(float parameter) const {
    if (!std::isfinite(parameter)) throw MathProblem("The spline parameter must be finite.");
    // Clamp as a float before converting, so a huge parameter cannot overflow the integer.
    float wholePart = std::floor(parameter);
    float highestIndex = static_cast<float>(segments.size() - 1);
    int index = static_cast<int>(smallerOf(largerOf(wholePart, 0), highestIndex));
    return SegmentLocation{index, parameter - static_cast<float>(index)};
}

/// An invented point that continues the line from `neighbour` through `endPoint` by the
/// same distance again, so the end pieces have a point on each side like the others.
///
/// Formula: `endPoint + (endPoint − neighbour) = 2·endPoint − neighbour`, the reflection
/// of `neighbour` through `endPoint`:
///
///     neighbour ●───────● endPoint ───────○ phantom
Vector3 CatmullRomSpline::phantomPoint(Vector3 endPoint, Vector3 neighbour) {
    return endPoint * 2 - neighbour;
}

/// How long the centripetal parameterization "spends" going from one point to the next:
/// `distance^alpha` with alpha = 0.5, which is the square root of the distance.
///
/// Longer gaps get more time, but less than in proportion, which is what keeps the
/// curve from overshooting. Equal to 0 only for coincident points, which the
/// constructor has already merged.
float CatmullRomSpline::centripetalInterval(Vector3 start, Vector3 end) {
    return std::sqrt(start.distance(end));
}

/// The Bézier piece between `current` and `next`, given the points around them.
///
/// How: give the four points knot times `t₀ = 0, t₁, t₂, t₃`, each gap being the
/// centripetal interval. The Catmull–Rom curve is the one cubic through `current` at
/// `t₁` and `next` at `t₂` whose velocity at those two knots is (Barry–Goldman, written
/// in terms of the points):
///
///     velocity at t₁ = (P₁−P₀)/(t₁−t₀) − (P₂−P₀)/(t₂−t₀) + (P₂−P₁)/(t₂−t₁)
///     velocity at t₂ = (P₂−P₁)/(t₂−t₁) − (P₃−P₁)/(t₃−t₁) + (P₃−P₂)/(t₃−t₂)
///
/// Rescaling time so the piece takes 1 unit instead of `t₂−t₁` multiplies both
/// velocities by `t₂−t₁`. A Bézier curve whose end velocities are `v₁` and `v₂` has
/// handles one third of a velocity away from its ends:
///
///     startHandle = P₁ + v₁/3        endHandle = P₂ − v₂/3
///
/// Check: for evenly spaced collinear points the velocity is the gap vector, and the
/// four Bézier points are evenly spaced on the line.
///
/// Sources: Yuksel, Schaefer & Keyser (2011); Farin (Hermite → Bézier handles).
CubicBezier<Vector3> CatmullRomSpline::segment(Vector3 previous, Vector3 current, Vector3 next, Vector3 following) {
    float intervalBefore = centripetalInterval(previous, current);
    float intervalMiddle = centripetalInterval(current, next);
    float intervalAfter = centripetalInterval(next, following);

    Vector3 velocityAtCurrent = (current - previous) / intervalBefore
        - (next - previous) / (intervalBefore + intervalMiddle)
        + (next - current) / intervalMiddle;
    Vector3 velocityAtNext = (next - current) / intervalMiddle
        - (following - current) / (intervalMiddle + intervalAfter)
        + (following - next) / intervalAfter;

    return CubicBezier<Vector3>(
        current,
        current + velocityAtCurrent * (intervalMiddle / 3),
        next - velocityAtNext * (intervalMiddle / 3),
        next);
}

} // namespace editor
