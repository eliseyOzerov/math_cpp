// Circular arcs given by their end points: a single cubic from two end points and the directions
// there, and the cubics of an arc from its end points and the angle it leaves the chord at.
// Ported from graphics_cpp's geometry.cpp (arcCubic, arcCubics); the arcs themselves are
// made by the existing `circularArc`.
#include <cmath>

#include "../Linear/MathProblem.h"
#include "CubicBezier.h"

namespace editor {

/// A cubic from `start` (heading along `startDirection`) to `end` (heading along `endDirection`)
/// that bends like a circular arc.
///
/// How: the turn θ between the two directions fixes a circular arc with chord c = |start→end|;
/// its radius is c/(2 sin(θ/2)) and its handles are (4/3)·tan(θ/4)·radius long, so each handle is
/// c·(4/3)·tan(θ/4)/(2 sin(θ/2)), along the unit directions: P1 = start + k·u, P2 = end − k·v.
/// For a turn under 1e-6 radians the handles are a third of the chord (a straight line).
///
/// Conventions: the directions need not be unit length (they are normalised); a zero direction
/// counts as no turn.
///
/// Source: Riškus (2006) and Goldapp (1991), as for `circularArcHandleFactor`.
///
/// Use when: joining two points with a curve that leaves and arrives in set directions and looks
///   like a circle
/// Tags: arc from tangents, circular arc cubic, biarc half, tangent arc, round join
/// Node: planned (arcFromTangents)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::arcCubic(
    Vector2 start, Vector2 startDirection, Vector2 end, Vector2 endDirection)
    requires std::same_as<PointType, Vector2>
{
    Vector2 leaving = startDirection.normalized();
    Vector2 arriving = endDirection.normalized();
    float turn = std::fabs(Angle::signedTurn(leaving, arriving).radians);
    float chord = start.distance(end);
    float handleLength =
        turn < 1e-6f ? chord / 3 : chord * (4.0f / 3) * std::tan(turn / 4) / (2 * std::sin(turn / 2));
    return CubicBezier<Vector2>(start, start + leaving * handleLength, end - arriving * handleLength, end);
}

/// The circular arc from `start` to `end` that leaves its chord `chordAngle` counter-clockwise of
/// it (and so turns by −2·chordAngle), as cubics of at most a quarter turn each.
///
/// How: the radius is R = chord / (2·sin|chordAngle|); the start direction is the chord direction
/// turned by `chordAngle`; the centre lies R from the start, to the right of that direction when
/// the arc turns clockwise (positive angle) and to the left otherwise; `circularArc` then makes
/// the pieces from the centre, the start's angle round it and the sweep −2·chordAngle. The last
/// piece's end is set to `end` exactly.
///
///      end ●───────────● start      chordAngle α between the chord and the arc's start tangent
///           ╲   α  ╱                the arc turns by 2α the other way round
///             ╰──╯
///
/// Conventions: y up, counter-clockwise positive. `chordAngle` must be non-zero and less than a
/// half turn in size, and the chord not zero, or the call throws a `MathProblem` (a zero angle
/// is a straight line; a half turn has no finite radius).
///
/// Source: Riškus (2006); Goldapp (1991).
///
/// Use when: an arc between two clicked points with a set bulge; round corners given by angle
/// Tags: arc from end points, arc by sweep, SVG arc, circular arc to Bezier, chord angle, bulge
/// Node: planned (arcBetweenPoints)
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::arcBetween(Vector2 start, Vector2 end, Angle chordAngle)
    requires std::same_as<PointType, Vector2>
{
    float chord = start.distance(end);
    float halfSweep = chordAngle.radians;
    if (!(chord > 0 && std::isfinite(chord))) throw MathProblem("An arc needs two different end points.");
    if (!(std::fabs(halfSweep) > 0 && std::fabs(halfSweep) < Angle::pi)) {
        throw MathProblem("An arc's angle to its chord must be non-zero and less than a half turn.");
    }
    float radius = chord / (2 * std::sin(std::fabs(halfSweep)));
    Vector2 chordDirection = (end - start) / chord;
    Vector2 startDirection(
        chordDirection.x * std::cos(halfSweep) - chordDirection.y * std::sin(halfSweep),
        chordDirection.x * std::sin(halfSweep) + chordDirection.y * std::cos(halfSweep));
    Vector2 towardsCentre = halfSweep > 0 ? startDirection.rightNormal() : startDirection.perpendicular();
    Vector2 centre = start + towardsCentre * radius;
    Vector2 fromCentre = start - centre;
    Angle startAngle = Angle::fromRadians(std::atan2(fromCentre.y, fromCentre.x));
    std::vector<CubicBezier<Vector2>> pieces = circularArc(centre, radius, startAngle, Angle::fromRadians(-2 * halfSweep));
    if (!pieces.empty()) pieces.back().end = end;
    return pieces;
}

template class CubicBezier<Vector2>;

} // namespace editor
