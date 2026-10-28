// A curve as circular arcs (graphics_cpp curves/offset `arcs`, Bezier.js `arcs`): the longest arc
// from each end that stays near the curve, found by halving.
#include <algorithm>
#include <cmath>
#include <limits>

#include "../Geometry/Circle2.h"
#include "../Linear/Scalar.h"
#include "CubicBezier.h"

namespace editor {

namespace {

/// An arc piece and the largest gap between it and the curve it follows.
struct ArcFit {
    CircularArcPiece arc;
    float error = 0;
};

/// The arc through `curve` at `startParameter`, the middle and `endParameter`, and how far the
/// curve strays from it, taken as the largest gap at six more places (an eighth to seven eighths
/// of the way, not the middle). Where the three points are on a line the arc is a straight
/// part (infinite radius) and the gap is the distance to the chord.
ArcFit arcOver(const CubicBezier<Vector2>& curve, float startParameter, float endParameter) {
    Vector2 from = curve.point(startParameter);
    Vector2 middle = curve.point((startParameter + endParameter) / 2);
    Vector2 to = curve.point(endParameter);
    std::optional<Circle2> circle = Circle2::through(from, middle, to);
    constexpr float shares[] = {0.125f, 0.25f, 0.375f, 0.625f, 0.75f, 0.875f};
    float largestGap = 0;
    if (!circle) {
        Vector2 chord = to - from;
        float chordLength = chord.length() > 0 ? chord.length() : 1;
        for (float share : shares) {
            Vector2 sample = curve.point(startParameter + (endParameter - startParameter) * share);
            largestGap = std::max(largestGap, std::fabs(chord.cross(sample - from)) / chordLength);
        }
        CircularArcPiece straight;
        straight.from = from;
        straight.to = to;
        straight.startParameter = startParameter;
        straight.endParameter = endParameter;
        return ArcFit{straight, largestGap};
    }
    for (float share : shares) {
        Vector2 sample = curve.point(startParameter + (endParameter - startParameter) * share);
        largestGap = std::max(largestGap, std::fabs(sample.distance(circle->centre) - circle->radius));
    }
    auto angleOf = [&](Vector2 point) {
        return Angle::fromRadians(std::atan2(point.y - circle->centre.y, point.x - circle->centre.x));
    };
    CircularArcPiece arc;
    arc.centre = circle->centre;
    arc.radius = circle->radius;
    arc.startAngle = angleOf(from);
    arc.endAngle = angleOf(to);
    arc.isCounterclockwise = (middle - from).cross(to - middle) > 0;
    arc.from = from;
    arc.to = to;
    arc.startParameter = startParameter;
    arc.endParameter = endParameter;
    return ArcFit{arc, largestGap};
}

} // namespace

/// The curve as circular arcs that each stay within `threshold` of it, end to end.
///
/// How: from the parameter where the last arc ended, try an arc to the end of the curve
/// (`Circle2::through` the start, middle and end of that stretch); if the curve strays from it
/// by more than `threshold` at six checked places, bisect for the longest stretch (30 halvings,
/// down to 1e-6 of the parameter) whose arc still fits, and continue from there. The
/// stretch is never shorter than 1e-3 of the parameter, and its allowance is at least 1e-4 of the
/// box diagonal, so the loop always advances. At most 1000 arcs.
///
///     curve ─╮__        arc 1 ╮     arc 2 ╮
///            ╰──╮   →   ╭─────╯    ╰──────╯   each arc ends where the next begins
///
/// Conventions: y-up plane; angles counterclockwise from +x in radians; a straight stretch has
/// no centre, an infinite radius and zero angles. The error is measured at six places, not
/// everywhere, as in Bezier.js.
///
/// Source: graphics_cpp curves/offset `arcs`; Pomax, *A Primer on Bézier Curves*, "Circle
/// approximation".
///
/// Use when: making an arc-only path for a CNC machine, a road with constant-radius bends, or
///   a dashed border that follows a curve
/// Tags: arcs, biarc, circular arc approximation, curve to arcs, radius of curvature, Bezier.js arcs
/// Node: planned (circularArcs)
template <CurvePoint PointType>
std::vector<CircularArcPiece> CubicBezier<PointType>::circularArcs(float threshold) const
    requires std::same_as<PointType, Vector2>
{
    constexpr int mostArcs = 1000;
    Bounds2 box = bounds();
    float allowanceFloor = 1e-4f * std::max(1e-12f, (box.maximum - box.minimum).length());
    std::vector<CircularArcPiece> arcs;
    float startParameter = 0;
    while (startParameter < 1 - 1e-6f && static_cast<int>(arcs.size()) < mostArcs) {
        ArcFit good = arcOver(*this, startParameter, 1);
        if (good.error > threshold) {
            float lower = startParameter;
            float upper = 1;
            good = arcOver(*this, startParameter, std::min(1.0f, startParameter + 1e-3f));
            for (int step = 0; step < 30 && upper - lower > 1e-6f; step += 1) {
                float middle = (lower + upper) / 2;
                ArcFit trial = arcOver(*this, startParameter, middle);
                if (trial.error <= std::max(threshold, allowanceFloor)) {
                    lower = middle;
                    good = trial;
                } else {
                    upper = middle;
                }
            }
        }
        arcs.push_back(good.arc);
        startParameter = good.arc.endParameter;
    }
    return arcs;
}

template class CubicBezier<Vector2>;

} // namespace editor
