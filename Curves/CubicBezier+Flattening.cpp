// Screens and meshes draw straight lines, so a curve is turned into a polyline before it is drawn
// or filled. The only question is how many pieces: too few and the curve looks like a polygon,
// too many and the work is wasted (Engine/Math/Curves/CubicBezier+Flattening.swift).
#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "CubicBezier.h"

namespace editor {

/// How many equal steps of the parameter are enough to keep a polyline within
/// `tolerance` of the curve.
///
/// How: joining B(t₀) and B(t₁) by a straight line is linear interpolation, whose error
/// at the same parameter is at most `h²/8 · max|B″|` with `h = t₁ − t₀`. For a cubic,
/// `B″ = 6·((1−t)·(P0 − 2P1 + P2) + t·(P1 − 2P2 + P3))`, so `max|B″| ≤ 6M` where
///
///     M = max( |P0 − 2·P1 + P2|, |P1 − 2·P2 + P3| )     (the largest "bend" of the polygon)
///
/// With `n` equal steps, `h = 1/n` and the error is at most `0.75·M/n²`. It is at most
/// `tolerance` when `n ≥ √(0.75·M / tolerance)`. The polyline point at parameter t is a
/// curve-to-polyline distance, so the true distance to the polyline is no larger.
///
/// Conventions: at least 1 (a straight line needs one piece), at most
/// `mostFlatteningPieces`. `tolerance` is in the curve's own units and must be positive
/// (a NaN tolerance stops the program). A curve with a control point that is not finite
/// gets 1 piece, so the NaN or infinity shows in the two points of the polyline instead
/// of being spread over `mostFlatteningPieces` of them.
///
/// Source: Wang's formula: Wang (1984); Filip, Magedson & Markot, "Surface algorithms
/// using bounds on derivatives", *CAGD* 3 (1986). For degree d it reads
/// `n ≥ √(d(d−1)/8 · M / tolerance)`, which is 0.75 for d = 3.
///
/// Use when: deciding how many segments a curve needs on screen; budgeting points of a
///   profile
/// Tags: flatness, subdivision count, tessellation count, curve segments, polyline
///   approximation, flatten Bezier
/// Node: none (solver)
template <CurvePoint PointType>
int CubicBezier<PointType>::flatteningPieceCount(float tolerance) const
    requires std::same_as<PointType, Vector2>
{
    if (!(tolerance > 0)) throw MathProblem("Flattening needs a positive tolerance.");
    if (!hasFiniteControlPoints()) return 1;
    float firstBend = (start - startHandle * 2 + endHandle).length();
    float secondBend = (startHandle - endHandle * 2 + end).length();
    float largestBend = largerOf(firstBend, secondBend);
    float needed = std::ceil(std::sqrt(0.75f * largestBend / tolerance));
    if (!std::isfinite(needed)) return mostFlatteningPieces;
    // Clamped as a float first, so a huge `needed` cannot overflow the integer.
    return static_cast<int>(smallerOf(largerOf(1, needed), static_cast<float>(mostFlatteningPieces)));
}

/// Points of a polyline that follows the curve to within `tolerance`: the start, the
/// end, and enough points between at equal steps of the parameter.
///
/// How: `flatteningPieceCount(tolerance:)` pieces, the points `B(i/n)` for
/// `i = 0…n`. Every point of the curve is then within `tolerance` of the polyline.
///
/// Conventions: the first point is exactly `start` and the last exactly `end`; a
/// straight curve gives 2 points.
///
/// Use when: drawing a curve with lines; filling a profile; measuring distance to a
///   curve approximately
/// Tags: flatten Bezier, polyline, tessellate curve, curve to lines, discretize curve
/// Node: inside roundCorners, chamferCorners, offsetProfile, profileArc
template <CurvePoint PointType>
std::vector<Vector2> CubicBezier<PointType>::flattened(float tolerance) const
    requires std::same_as<PointType, Vector2>
{
    int pieceCount = flatteningPieceCount(tolerance);
    // The ends are taken from the control points themselves: evaluating at 1 can miss
    // `end` by a rounding step.
    std::vector<Vector2> points = {start};
    for (int step = 1; step < pieceCount; step += 1) {
        points.push_back(point(static_cast<float>(step) / static_cast<float>(pieceCount)));
    }
    points.push_back(end);
    return points;
}

template class CubicBezier<Vector2>;

} // namespace editor
