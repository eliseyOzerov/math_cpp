// The point of a flat cubic (or of a list of them) nearest a given point. Ported from
// graphics_cpp's (geometry.cpp `nearest`, bezier.cpp `project`).
#include <cmath>
#include <limits>

#include "../Linear/MathProblem.h"
#include "CubicBezier.h"

namespace editor {

/// The point of the cubics `curves` nearest `target`: which curve, at what parameter, the point
/// and its distance.
///
/// How: every curve is sampled at `samples` even steps of its parameter and the closest sample
/// taken; then Newton's method is run on f(t) = (B(t) − target)·B′(t), which is zero where the
/// squared distance is smallest, using f′(t) = |B′|² + (B − target)·B″. A step is kept only while
/// it does not move further from the target; up to 8 steps, each clamped to 0…1.
///
///     target ○
///             ╲ nearest sample, then refined along the curve
///        ──────●──────
///
/// Conventions: the first curve wins a tie. The sample search can miss a nearer point between
/// samples on a curve with a sharp turn; raise `samples` for those. An empty list or fewer
/// than 1 sample throws a `MathProblem`.
///
/// Source: Pomax, "A Primer on Bézier Curves", projecting a point onto a curve.
///
/// Use when: picking the object under the cursor; snapping a point onto a path
/// Tags: closest point on curve, project point, nearest point Bezier, snapping, pick
/// Node: planned (closestPointOnCurve)
template <CurvePoint PointType>
PathPoint2 CubicBezier<PointType>::nearest(
    const std::vector<CubicBezier<Vector2>>& curves, Vector2 target, int samples)
    requires std::same_as<PointType, Vector2>
{
    if (curves.empty()) throw MathProblem("There is no curve to find the nearest point on.");
    if (!(samples >= 1)) throw MathProblem("The nearest point needs at least one sample.");
    PathPoint2 best{0, 0, curves[0].start, std::numeric_limits<float>::infinity()};
    for (size_t index = 0; index < curves.size(); index += 1) {
        for (int sample = 0; sample <= samples; sample += 1) {
            float parameter = static_cast<float>(sample) / static_cast<float>(samples);
            Vector2 candidate = curves[index].point(parameter);
            float distance = candidate.distance(target);
            if (distance < best.distance) best = PathPoint2{static_cast<int>(index), parameter, candidate, distance};
        }
    }
    const CubicBezier<Vector2>& curve = curves[best.pieceIndex];
    float parameter = best.parameter;
    float distanceNow = best.distance;
    for (int step = 0; step < 8; step += 1) {
        Vector2 offset = curve.point(parameter) - target;
        Vector2 velocity = curve.firstDerivative(parameter);
        float slope = offset.dot(velocity);
        float slopeChange = velocity.lengthSquared() + offset.dot(curve.secondDerivative(parameter));
        if (std::fabs(slopeChange) < 1e-12f) break;
        float next = std::fmin(1.0f, std::fmax(0.0f, parameter - slope / slopeChange));
        float distanceNext = curve.point(next).distance(target);
        if (distanceNext > distanceNow) break;
        parameter = next;
        distanceNow = distanceNext;
    }
    Vector2 refined = curve.point(parameter);
    float refinedDistance = refined.distance(target);
    if (refinedDistance <= best.distance) return PathPoint2{best.pieceIndex, parameter, refined, refinedDistance};
    return best;
}

/// The point of this curve nearest `target`: its parameter, the point and the distance
/// (`pieceIndex` is 0).
///
/// How: `nearest` over this one curve, with `samples` samples (64 by default).
///
/// Use when: dragging a point along a curve; the distance from a point to a curve
/// Tags: project point onto curve, closest point, nearest point Bezier, distance to curve
/// Node: planned (closestPointOnCurve)
template <CurvePoint PointType>
PathPoint2 CubicBezier<PointType>::nearestPoint(Vector2 target, int samples) const
    requires std::same_as<PointType, Vector2>
{
    return nearest({*this}, target, samples);
}

template class CubicBezier<Vector2>;

} // namespace editor
