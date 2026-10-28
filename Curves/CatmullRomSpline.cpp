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

// MARK: - Flat splines

/// Cubics through every point of a flat list in turn: a Catmull–Rom spline of any `alpha`, with a
/// `tension` and the option to close.
///
/// How: consecutive points closer than `Tolerance.geometric` are merged (a closed list that ends
/// on its first point drops that last one). For each pair `P₁ → P₂` with neighbours `P₀`, `P₃`,
/// knot gaps are `tᵢ = |Pᵢ − Pᵢ₋₁|^alpha` (at least 1e-9), and Barry and Goldman's end velocities
/// (see `segment`) give the Bézier handles
///
///     startHandle = P₁ + v₁·(t₂−t₁)/3·tension        endHandle = P₂ − v₂·(t₂−t₁)/3·tension
///
/// An open spline's missing neighbours are mirrored (`2·end − neighbour`), so it leaves the ends
/// straight on; a closed one wraps round, one more cubic joining the last point to the first.
///
/// Conventions: y-up plane; alpha 0 gives the uniform spline, 0.5 the centripetal (no loops), 1 the
/// chordal; tension 1 is the spline itself and 0 the polyline. Empty for fewer than two distinct
/// points.
///
/// Source: Barry & Goldman (1988); Yuksel, Schaefer & Keyser (2011); graphics_cpp curves/spline.cpp
/// `catmullRom`.
///
/// Use when: a stroke that must pass through hand-placed points, open or closed; a wobble's keys
/// Tags: Catmull-Rom, centripetal, chordal, uniform, spline through points, closed spline, tension
/// Node: planned (splineThrough)
std::vector<CubicBezier<Vector2>> CatmullRomSpline::cubicsThrough(
    const std::vector<Vector2>& points, const CatmullRomOptions& options) {
    std::vector<Vector2> distinct;
    for (Vector2 point : points) {
        if (distinct.empty() || point.distance(distinct.back()) > Tolerance::geometric) distinct.push_back(point);
    }
    if (options.closed && distinct.size() > 2 && distinct.front().distance(distinct.back()) < Tolerance::geometric) {
        distinct.pop_back();
    }
    int count = static_cast<int>(distinct.size());
    if (count < 2) return {};
    auto neighbour = [&](int index) {
        if (options.closed) return distinct[static_cast<size_t>(((index % count) + count) % count)];
        if (index < 0) return distinct[0] * 2 - distinct[1];
        if (index >= count) return distinct[static_cast<size_t>(count - 1)] * 2 - distinct[static_cast<size_t>(count - 2)];
        return distinct[static_cast<size_t>(index)];
    };
    auto knotGap = [&](Vector2 from, Vector2 to) { return largerOf(std::pow(from.distance(to), options.alpha), 1e-9f); };
    // The Barry–Goldman velocity at the middle point of three, for knots `ta < tb < tc`.
    auto velocity = [](Vector2 a, Vector2 b, Vector2 c, float ta, float tb, float tc) {
        return (b - a) / (tb - ta) - (c - a) / (tc - ta) + (c - b) / (tc - tb);
    };
    std::vector<CubicBezier<Vector2>> cubics;
    int pieceCount = options.closed ? count : count - 1;
    for (int index = 0; index < pieceCount; index += 1) {
        Vector2 before = neighbour(index - 1);
        Vector2 current = neighbour(index);
        Vector2 next = neighbour(index + 1);
        Vector2 after = neighbour(index + 2);
        float firstKnot = knotGap(before, current);
        float secondKnot = firstKnot + knotGap(current, next);
        float thirdKnot = secondKnot + knotGap(next, after);
        Vector2 velocityAtCurrent = velocity(before, current, next, 0, firstKnot, secondKnot);
        Vector2 velocityAtNext = velocity(current, next, after, firstKnot, secondKnot, thirdKnot);
        float reach = (secondKnot - firstKnot) / 3 * options.tension;
        cubics.push_back(CubicBezier<Vector2>(current, current + velocityAtCurrent * reach, next - velocityAtNext * reach, next));
    }
    return cubics;
}

/// The Catmull–Rom spline through the points as a path (closed when the options say so), or
/// nothing for fewer than two distinct points.
///
/// How: `cubicsThrough`, one cubic piece per cubic; a closed spline's last cubic ends on the first
/// point, then `close()`.
///
/// Use when: a stroke through points that will be offset, filled or measured as a path
/// Tags: Catmull-Rom path, spline path, curve through points, smooth outline
/// Node: planned (splineThrough)
std::optional<Path2D> CatmullRomSpline::pathThrough(const std::vector<Vector2>& points, const CatmullRomOptions& options) {
    std::vector<CubicBezier<Vector2>> cubics = cubicsThrough(points, options);
    if (cubics.empty()) return std::nullopt;
    Path2D path(cubics.front().start);
    for (const CubicBezier<Vector2>& cubic : cubics) path.addCubic(cubic.startHandle, cubic.endHandle, cubic.end);
    if (options.closed) path.close();
    return path;
}

// MARK: - Spatial splines

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
