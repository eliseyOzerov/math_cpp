// A circle is not a polynomial, so no cubic Bézier is exactly a circle. But a short arc is
// matched so closely that the eye cannot see the difference, and then everything that works for
// cubics (bounds, area, offsetting, sweeping) works for circles too
// (Engine/Math/Curves/CubicBezier+Arc.swift).
#include <cmath>

#include "../Linear/Scalar.h"
#include "CubicBezier.h"

namespace editor {

/// The handle length, as a fraction of the radius, of the cubic that approximates a
/// circular arc of `sweep`: `(4/3)·tan(sweep/4)`. Negative for a clockwise sweep.
///
/// How: take the arc symmetric about the x axis, from angle −h to +h with h = sweep/2.
/// Its cubic has end points `r·(cos h, ∓sin h)` and handles along the end tangents,
/// each of length `k·r`. The curve's middle point is `(P0 + 3P1 + 3P2 + P3)/8`; its x
/// coordinate is `r·(cos h + ¾·k·sin h)`. Demanding that the middle lies on the circle,
/// x = r, gives `k = (4/3)(1 − cos h)/sin h = (4/3)·tan(h/2) = (4/3)·tan(sweep/4)`.
/// For a quarter turn that is 0.5523.
///
///        P1 ●─ ─ ─ ─ ● P2        the handles are tangent to the circle at the ends,
///          ╱  ╭───╮   ╲          each k·r long
///     P0 ●───╯     ╰───● P3
///
/// Conventions: `sweep` is in radians inside `Angle`; for 0 the factor is 0 (a point).
/// The sweep should be at most 90° for the error stated at `circularArc`.
///
/// Source: Riškus, "Approximation of a cubic Bézier curve by circular arcs and vice
/// versa" (2006); Goldapp, "Approximation of circular arcs by cubic polynomials" (1991).
///
/// Use when: writing a circle arc as a Bézier handle; checking the famous 0.5523 constant
/// Tags: kappa, 0.5523, circle handle length, magic number, 4/3 tan, circle bezier
///   constant
/// Node: none (solver)
template <CurvePoint PointType>
float CubicBezier<PointType>::circularArcHandleFactor(Angle sweep)
    requires std::same_as<PointType, Vector2>
{
    return (sweep / 4).tangent() * (4.0f / 3.0f);
}

/// The cubic Béziers, in order, that approximate the circle arc around `centre` with
/// `radius`, starting at `startAngle` and turning by `sweep`. Counterclockwise for a
/// positive sweep, clockwise for a negative one. A sweep of 360° is a full circle in four
/// pieces; more than 360° winds around again.
///
/// How: the sweep is cut into the fewest equal pieces of at most 90° (a tiny excess of
/// `arcPieceCountSlack` of a quarter turn is forgiven so that exactly 90° or 360° do not
/// split in two). For a piece from angle α to β with unit vectors `u(α) = (cos α, sin α)`
/// and its tangent `u(α).perpendicular`:
///
///     P0 = centre + r·u(α)         P1 = P0 + k·r·perpendicular(u(α))
///     P3 = centre + r·u(β)         P2 = P3 − k·r·perpendicular(u(β))
///     k  = (4/3)·tan((β − α)/4)
///
/// The boundary angles `startAngle + pieceSweep·i` and their points are computed once,
/// so each piece ends exactly where the next begins.
///
/// Accuracy: the curve is not on the circle exactly. For a 90° piece the largest radial
/// error is about 2.7e-4·radius (0.027%), and it falls with the sixth power of the
/// angle: a 45° piece is about 64 times better. The end points and end tangents are
/// exact, so consecutive pieces join smoothly.
///
/// Conventions: y-up plane, angles measured from +x counterclockwise. No pieces (an empty
/// array) when the radius is not positive and finite, the sweep is zero or not finite
/// (NaN, infinity), or the sweep would need more than `mostArcPieces` pieces.
///
/// Source: Riškus (2006); Goldapp (1991).
///
/// Use when: drawing or sweeping a circle or rounded corner with the Bézier tools;
///   adding an arc to a path; building a pipe cross-section
/// Tags: circle arc to Bezier, arc as cubic, circle approximation, rounded arc, SVG arc,
///   kappa, ellipse arc, quarter circle
/// Node: profileArc
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::circularArc(
    Vector2 centre, float radius, Angle startAngle, Angle sweep)
    requires std::same_as<PointType, Vector2>
{
    // Written so that NaN fails every test: NaN > 0 and NaN.isFinite are both false.
    if (!(radius > 0 && std::isfinite(radius) && std::isfinite(sweep.radians) && sweep.radians != 0)) return {};
    float quarterTurns = std::fabs(sweep.radians) / longestArcPerCubic().radians;
    float neededPieces = std::ceil(quarterTurns - arcPieceCountSlack);
    if (!(neededPieces <= static_cast<float>(mostArcPieces))) return {};
    int pieceCount = static_cast<int>(largerOf(1, neededPieces));
    Angle pieceSweep = sweep / static_cast<float>(pieceCount);
    float handleLength = circularArcHandleFactor(pieceSweep) * radius;

    std::vector<Vector2> directions;
    for (int index = 0; index <= pieceCount; index += 1) {
        Angle angle = startAngle + pieceSweep * static_cast<float>(index);
        directions.push_back(Vector2(angle.cosine(), angle.sine()));
    }
    std::vector<Vector2> points;
    for (Vector2 direction : directions) points.push_back(centre + direction * radius);
    std::vector<CubicBezier<Vector2>> pieces;
    for (int index = 0; index < pieceCount; index += 1) {
        pieces.push_back(CubicBezier<Vector2>(
            points[index],
            points[index] + directions[index].perpendicular() * handleLength,
            points[index + 1] - directions[index + 1].perpendicular() * handleLength,
            points[index + 1]));
    }
    return pieces;
}

template class CubicBezier<Vector2>;

} // namespace editor
