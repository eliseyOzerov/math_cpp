// A flat cubic seen in its own frame: where one coordinate reaches a value, the curve laid along
// a line, the tight box round it in that frame, whether it is straight, and which of the six
// shape classes it belongs to. Ported from graphics_cpp's (bezier.cpp: coordRoots,
// align, tightBox, canonical, name; geometry.cpp: isLine), after Pomax, "A Primer on Bézier
// Curves" (https://pomax.github.io/bezierinfo), and Stone and DeRose, "A Geometric
// Characterization of Parametric Cubic Curves" (ACM Transactions on Graphics, 1989).
//
// Conventions: y up, counter-clockwise angles positive, parameters 0…1.
#include <algorithm>
#include <cmath>

#include "../Linear/Angle.h"
#include "../Numerics/CubicRoots.h"
#include "CubicBezier.h"

namespace editor {

namespace {

/// `vector` turned by `angle` counter-clockwise about the origin.
Vector2 turnedBy(Vector2 vector, Angle angle) { return vector.rotated(angle); }

/// The direction of the line from `lineStart` to `lineEnd` as an angle from +x (0 for a point).
Angle directionAngle(Vector2 lineStart, Vector2 lineEnd) { return Angle::fromRadians(std::atan2((lineEnd - lineStart).y, (lineEnd - lineStart).x)); }

} // namespace

/// The parameters in 0…1 where a one-dimensional cubic Bézier equals `value`, ascending.
///
/// How: the control values p₀…p₃ minus `value` are q₀…q₃, and the Bernstein form
/// (1−t)³q₀ + 3(1−t)²t·q₁ + 3(1−t)t²·q₂ + t³q₃ expands to the power basis
///
///     (−q₀ + 3q₁ − 3q₂ + q₃)·t³ + (3q₀ − 6q₁ + 3q₂)·t² + (−3q₀ + 3q₁)·t + q₀
///
/// whose roots in 0…1 `CubicRoots` finds.
///
/// Conventions: a root a hair outside 0…1 counts as the end; a constant function that equals
/// `value` everywhere has no roots listed (the equation is zero, not crossed).
///
/// Source: Pomax, "A Primer on Bézier Curves", the section on finding roots.
///
/// Use when: finding where a curve is at a given height or distance across; the extremes of a
///   curve from the roots of its derivative values
/// Tags: bezier roots, solve bezier = value, coordinate roots, x of t, find t for y
/// Node: planned (parametersAtCoordinate)
template <CurvePoint PointType>
std::vector<float> CubicBezier<PointType>::coordinateRoots(
    float startValue, float startHandleValue, float endHandleValue, float endValue, float value)
    requires std::same_as<PointType, Vector2>
{
    float first = startValue - value;
    float second = startHandleValue - value;
    float third = endHandleValue - value;
    float fourth = endValue - value;
    return CubicRoots::solve(
        -first + 3 * second - 3 * third + fourth, 3 * first - 6 * second + 3 * third, -3 * first + 3 * second, first,
        0, 1);
}

/// The parameters in 0…1 where the curve's x (or y) equals `value`, ascending.
///
/// How: `coordinateRoots` of the four control x (or y) values.
///
/// Use when: where a road reaches a given x; where a profile crosses a height
/// Tags: curve x equals, solve for t, horizontal line crossing, level crossing
/// Node: planned (parametersAtCoordinate)
template <CurvePoint PointType>
std::vector<float> CubicBezier<PointType>::parametersAtCoordinate(CurveAxis axis, float value) const
    requires std::same_as<PointType, Vector2>
{
    if (axis == CurveAxis::x) return coordinateRoots(start.x, startHandle.x, endHandle.x, end.x, value);
    return coordinateRoots(start.y, startHandle.y, endHandle.y, end.y, value);
}

/// The curve moved and turned so the line from `lineStart` to `lineEnd` lies along the x axis,
/// starting at the origin.
///
/// How: every control point minus `lineStart`, turned by −θ where θ is the angle of the line
/// from +x. The map is rigid, so lengths and curvatures are unchanged.
///
///     before:   ●P3                 after:  P0●────────● (line)
///              /  line at θ                  curve now relative to the x axis
///         P0 ●
///
/// Conventions: a zero-length line has angle 0, so the curve is only moved.
///
/// Source: Pomax, "A Primer on Bézier Curves", aligning a curve (used for inflections and
/// bounding boxes).
///
/// Use when: finding the tight box of a curve; testing where a curve crosses a line by roots
///   of its y
/// Tags: align curve, rotate to x axis, curve in line frame, canonical placement
/// Node: planned (alignCurve)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::alignedTo(Vector2 lineStart, Vector2 lineEnd) const
    requires std::same_as<PointType, Vector2>
{
    Angle undo = -directionAngle(lineStart, lineEnd);
    return CubicBezier<Vector2>(
        turnedBy(start - lineStart, undo), turnedBy(startHandle - lineStart, undo),
        turnedBy(endHandle - lineStart, undo), turnedBy(end - lineStart, undo));
}

/// The curve aligned to its own chord (`alignedTo(start, end)`): the start at the origin, the
/// end on the positive x axis.
///
/// Use when: reading a curve's shape independent of where it sits and how it is turned
/// Tags: align to chord, normalise curve, chord frame
/// Node: planned (alignCurve)
template <CurvePoint PointType>
CubicBezier<Vector2> CubicBezier<PointType>::alignedToChord() const
    requires std::same_as<PointType, Vector2>
{
    return alignedTo(start, end);
}

/// The smallest rectangle round the curve that is lined up with its chord: four corners,
/// counter-clockwise from the corner at the low-x, low-y side of the chord frame.
///
/// How: align the curve to its chord, take the exact bounds of that (`bounds`), and turn the
/// four corners back by the chord's angle and add the start.
///
///        c3 ┌─────────────┐ c2        the box hugs the curve in the frame of
///           │   ╭────╮    │           the chord P0→P3, not in the x and y axes
///      P0 ● ┼──╯      ╰───┼─● P3
///        c0 └─────────────┘ c1
///
/// Conventions: y up; a straight curve gives a box of zero height.
///
/// Source: Pomax, "A Primer on Bézier Curves", tight bounding boxes.
///
/// Use when: a tighter collision box for a curved road; sizing a label along a curve
/// Tags: oriented bounding box, OBB, tight box, rotated bounds, minimum box
/// Node: planned (tightBox)
template <CurvePoint PointType>
std::array<Vector2, 4> CubicBezier<PointType>::tightBox() const
    requires std::same_as<PointType, Vector2>
{
    Bounds2 box = alignedToChord().bounds();
    Angle back = directionAngle(start, end);
    auto cornerAt = [&](float x, float y) { return start + turnedBy(Vector2(x, y), back); };
    return {cornerAt(box.minimum.x, box.minimum.y), cornerAt(box.maximum.x, box.minimum.y),
            cornerAt(box.maximum.x, box.maximum.y), cornerAt(box.minimum.x, box.maximum.y)};
}

/// Whether both handles lie on the chord from the start to the end, so the curve is a straight
/// line (possibly traversed unevenly).
///
/// How: each handle's distance from the chord, |(handle − start) × chord| / |chord|, must be at
/// most 1e-6 of the chord's length, i.e. |(handle − start) × chord| ≤ 1e-6·|chord|². A curve whose
/// ends coincide (|chord|² ≤ 1e-18) is not a line.
///
/// Conventions: the tolerance is relative to the chord, so it holds at any scale; it is looser
/// than the double-precision 1e-9 of graphics_cpp because points are floats.
///
/// Use when: skipping curve work for a straight piece; drawing a line instead of a curve
/// Tags: is line, is straight, collinear handles, degenerate cubic, flat curve
/// Node: planned (isStraightCurve)
template <CurvePoint PointType>
bool CubicBezier<PointType>::isStraight() const
    requires std::same_as<PointType, Vector2>
{
    Vector2 chord = end - start;
    float chordSquared = chord.lengthSquared();
    if (!(chordSquared > 1e-18f)) return false;
    for (Vector2 handle : {startHandle, endHandle}) {
        if (!(std::fabs((handle - start).cross(chord)) <= 1e-6f * chordSquared)) return false;
    }
    return true;
}

/// The shape class of the cubic and where its canonical form puts the fourth point.
///
/// How (Stone and DeRose; Pomax's Primer): with u = P1 − P0, v = P2 − P0, w = P3 − P0, the affine
/// map taking P0, P1, P2 to (0, 0), (0, 1), (1, 1) puts P3 at (x, y) with w = s·u + r·v,
/// x = r, y = s + r (s, r by Cramer's rule). The class is read from the plane's regions:
///
///     cusp curve   y = (−x² + 2x + 3)/4        (x ≤ 1)
///     y > 1                                  → one inflection
///     x > 1                                  → arch (no inflection, no loop)
///     above the cusp curve, y ≤ 1            → two inflections
///     below it, at or above the loop edge    → loop
///     otherwise                              → arch
///
/// where the loop edge is (−x² + 3x)/3 for x ≤ 0 and (√(3(4x − x²)) − x)/2 for x > 0.
/// On the cusp curve (within `tolerance`) the curve has a cusp.
///
/// Conventions: if the first three points are collinear (u × v within `tolerance` of zero,
/// relative) no map exists: the kind is `line` when all four points are collinear, else `arch`,
/// and `fourthPoint` is empty. Computed in double, so the float input does not blur the
/// boundaries.
///
/// Use when: deciding whether a curve needs splitting at a loop or cusp before offsetting; telling
///   how many inflections it has without solving for them
/// Tags: canonical form, Stone DeRose, cubic classification, loop cusp inflection, curve type
/// Node: planned (classifyCurve)
template <CurvePoint PointType>
CanonicalForm CubicBezier<PointType>::canonicalForm(float tolerance) const
    requires std::same_as<PointType, Vector2>
{
    auto cross = [](double firstX, double firstY, double secondX, double secondY) {
        return firstX * secondY - firstY * secondX;
    };
    double ux = double(startHandle.x) - start.x, uy = double(startHandle.y) - start.y;
    double vx = double(endHandle.x) - start.x, vy = double(endHandle.y) - start.y;
    double wx = double(end.x) - start.x, wy = double(end.y) - start.y;
    double scaleOfFirstTwo = ux * ux + uy * uy + vx * vx + vy * vy;
    double scaleOfChord = wx * wx + wy * wy;
    double determinant = cross(ux, uy, vx, vy);
    // A zero scale would make the tolerance zero: use 1, as a unit-sized figure.
    auto orOne = [](double value) { return value != 0 && !std::isnan(value) ? value : 1.0; };
    if (std::fabs(determinant) < tolerance * orOne(scaleOfFirstTwo)) {
        double handlesOffChord = std::fabs(cross(wx, wy, double(endHandle.x) - startHandle.x,
                                                 double(endHandle.y) - startHandle.y));
        bool straight = handlesOffChord < tolerance * orOne(scaleOfChord)
            && std::fabs(cross(wx, wy, ux, uy)) < tolerance * orOne(scaleOfChord);
        return CanonicalForm{straight ? CubicShapeKind::line : CubicShapeKind::arch, std::nullopt};
    }
    double s = cross(wx, wy, vx, vy) / determinant;
    double r = cross(ux, uy, wx, wy) / determinant;
    double x = r;
    double y = s + r;
    Vector2 fourth(static_cast<float>(x), static_cast<float>(y));
    double cuspCurve = (-x * x + 2 * x + 3) / 4;
    if (std::fabs(y - cuspCurve) < tolerance && x <= 1) return CanonicalForm{CubicShapeKind::cusp, fourth};
    if (y > 1) return CanonicalForm{CubicShapeKind::singleInflection, fourth};
    if (x > 1) return CanonicalForm{CubicShapeKind::arch, fourth};
    if (y > cuspCurve) return CanonicalForm{CubicShapeKind::doubleInflection, fourth};
    double loopEdge = x <= 0 ? (-x * x + 3 * x) / 3 : (std::sqrt(3 * (4 * x - x * x)) - x) / 2;
    return CanonicalForm{y >= loopEdge ? CubicShapeKind::loop : CubicShapeKind::arch, fourth};
}

/// The kind's name as a short phrase: "line", "arch", "single inflection", "double inflection",
/// "loop" or "cusp".
///
/// Use when: showing the class of a curve in a panel or a log
/// Tags: curve kind name, classification label, cubic type name
/// Node: none (label)
std::string_view cubicShapeKindName(CubicShapeKind kind) {
    switch (kind) {
    case CubicShapeKind::line: return "line";
    case CubicShapeKind::arch: return "arch";
    case CubicShapeKind::singleInflection: return "single inflection";
    case CubicShapeKind::doubleInflection: return "double inflection";
    case CubicShapeKind::loop: return "loop";
    case CubicShapeKind::cusp: return "cusp";
    }
    return "";
}

template class CubicBezier<Vector2>;

} // namespace editor
