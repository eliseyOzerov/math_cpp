// The offset of a flat cubic: the curve that runs alongside it at a set distance, like the
// edge of a road drawn from its centre line, or one side of a stroke
// (Engine/Math/Curves/CubicBezier+Offset.swift).
//
//     offset  ╭───────────╮          every point moved by d along the normal there
//            ╱  ╭───────╮  ╲
//           ╱  ╱ curve   ╲  ╲
//
// The exact offset of a cubic is not a cubic (its formula has a square root in it), so it is
// approximated by cubics: the curve is cut into "simple" pieces that bend gently, each piece
// is replaced by the cubic that matches the true offset's end points and end derivatives
// (a Hermite fit), and a piece whose fit strays from the true offset by more than the
// tolerance is halved and fitted again.
//
// Sign: a positive distance moves to the RIGHT of the direction of travel (`rightNormal`).
// Walking a counterclockwise closed outline the right side is the outside, so a positive
// distance grows such an outline and a negative one shrinks it.
//
// Swift's `ClosedRange<Float>` is `ParameterRange`; `precondition` is a thrown `MathProblem`.
#include <cmath>
#include <limits>
#include <optional>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "ArcLength.h"
#include "CubicBezier.h"

namespace editor {

namespace {

/// The largest of `max(|x|, |y|)` over the points (Swift's `.map { max(abs($0.x), abs($0.y)) }.max()
/// ?? 0`): `Array.max` keeps the first of equal values and, like the comparison here, passes over
/// a NaN that is not first.
float largestCoordinateOf(std::initializer_list<Vector2> points) {
    bool isFirst = true;
    float largest = 0;
    for (Vector2 point : points) {
        float size = largerOf(std::fabs(point.x), std::fabs(point.y));
        if (isFirst || largest < size) largest = size;
        isFirst = false;
    }
    return largest;
}

} // namespace

/// The largest turn between the start and end directions of a piece that counts as simple
/// enough to offset in one fit: 60°.
///
/// Use when: splitting a curve into pieces gentle enough to offset
/// Tags: offset split, turn limit, simple curve, reduce
/// Node: none (validation)
template <CurvePoint PointType>
Angle CubicBezier<PointType>::largestSimpleTurn()
    requires std::same_as<PointType, Vector2>
{
    return Angle::degrees(60);
}

/// Whether the curve bends gently enough to offset in one fit.
///
/// Two tests, both from Fountain's `simple`:
/// 1. Both handles lie on the same side of the chord from start to end (or on it): the
///    cross products `(P3 − P0) × (P1 − P0)` and `(P3 − P0) × (P2 − P0)` do not have opposite
///    signs. This rules out S-shapes, whose offset bulges both ways.
/// 2. The direction turns by less than `largestSimpleTurn` (60°) from start to end.
///
///     simple:   ╭──╮          not simple:  ╭╮     ╭─╮
///              ╱    ╲                     ╱ ╰╮   ╱  ╰─╮
///                                              ╲╱       (an S, or a hairpin)
///
/// The cross products are taken of the control points measured from the start and divided
/// by their largest coordinate, and their signs are compared rather than their product: at
/// a scale of 1e-20 the products of raw coordinates underflow to 0 and an S would pass.
///
/// A curve that is a single point, or has a non-finite control point, is not simple.
///
/// Source: Fountain `curves/offset.ts` (the user's own library), after Pomax, *A Primer on
/// Bézier Curves*, "Curve offsetting".
///
/// Use when: deciding whether a piece needs cutting before it is offset
/// Tags: simple curve, offsettable, gentle curve, Bezier reduce, control polygon side test
/// Node: none (solver)
template <CurvePoint PointType>
bool CubicBezier<PointType>::isSimpleForOffset() const
    requires std::same_as<PointType, Vector2>
{
    Vector2 offsets[3] = {startHandle - start, endHandle - start, end - start};
    for (Vector2 offset : offsets) {
        if (!offset.isFinite()) return false;
    }
    float largestCoordinate = largestCoordinateOf({offsets[0], offsets[1], offsets[2]});
    if (!(largestCoordinate > 0)) return false;
    Vector2 scaled[3] = {offsets[0] / largestCoordinate, offsets[1] / largestCoordinate, offsets[2] / largestCoordinate};
    float startHandleSide = scaled[2].cross(scaled[0]);
    float endHandleSide = scaled[2].cross(scaled[1]);
    bool handlesOnOppositeSides = (startHandleSide > 0 && endHandleSide < 0) || (startHandleSide < 0 && endHandleSide > 0);
    if (handlesOnOppositeSides) return false;
    Vector2 startTangent = startDirection();
    Vector2 endTangent = endDirection();
    if (startTangent == Vector2() || endTangent == Vector2()) return false;
    return std::fabs(Angle::signedTurn(startTangent, endTangent).radians) < largestSimpleTurn().radians;
}

/// The curve cut into pieces that are each `isSimpleForOffset`, in order, the first starting
/// at `start` and the last ending at `end` exactly. See `simplePieceParameters`.
///
/// Use when: preparing a curve for offsetting or stroking
/// Tags: reduce, split into simple curves, offset preparation, Bezier subdivision
/// Node: none (solver)
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::simplePiecesForOffset() const
    requires std::same_as<PointType, Vector2>
{
    std::vector<CubicBezier<Vector2>> pieces;
    for (ParameterRange range : simplePieceParameters()) pieces.push_back(portion(range.lowerBound, range.upperBound));
    return pieces;
}

/// The parameter ranges of the simple pieces (`simplePiecesForOffset`), in order, from 0 to
/// 1 without gaps.
///
/// How: cut at the inflections first (a piece that bends both ways is never simple), then
/// halve every range whose piece is still not simple, at most `mostOffsetHalvings` times; a
/// piece that is still not simple then (around a cusp, where the direction flips in an
/// instant) is kept as it is, and its fit will then miss the tolerance.
///
/// Fountain grows each piece in steps of 0.01 while it stays simple; halving gives
/// fewer, evenly sized pieces and needs no step size.
///
/// Use when: cutting a curve into simple pieces while keeping where each lies on it, for a
///   distance that changes along the curve
/// Tags: reduce, simple ranges, parameter intervals, offset preparation
/// Node: none (solver)
template <CurvePoint PointType>
std::vector<ParameterRange> CubicBezier<PointType>::simplePieceParameters() const
    requires std::same_as<PointType, Vector2>
{
    std::vector<float> cuts;
    cuts.push_back(0);
    for (float parameter : inflectionParameters()) {
        if (parameter > 0 && parameter < 1) cuts.push_back(parameter);
    }
    cuts.push_back(1);
    std::vector<ParameterRange> ranges;
    for (size_t index = 0; index + 1 < cuts.size(); index += 1) {
        for (ParameterRange range : halvedUntilSimple(ParameterRange{cuts[index], cuts[index + 1]}, 0)) {
            ranges.push_back(range);
        }
    }
    return ranges;
}

/// `range` halved until the piece of the curve over each part is simple (see
/// `simplePieceParameters`).
template <CurvePoint PointType>
std::vector<ParameterRange> CubicBezier<PointType>::halvedUntilSimple(ParameterRange range, int halvings) const
    requires std::same_as<PointType, Vector2>
{
    CubicBezier<Vector2> piece = portion(range.lowerBound, range.upperBound);
    if (piece.isSimpleForOffset() || halvings >= mostOffsetHalvings) return {range};
    float middle = linearBlend(range.lowerBound, range.upperBound, 0.5f);
    std::vector<ParameterRange> result = halvedUntilSimple(ParameterRange{range.lowerBound, middle}, halvings + 1);
    for (ParameterRange second : halvedUntilSimple(ParameterRange{middle, range.upperBound}, halvings + 1)) {
        result.push_back(second);
    }
    return result;
}

/// The offset of the curve by `distance` to the right of its direction of travel, as cubics
/// within `tolerance` of the true offset, or nil. See
/// `offset(startDistance:endDistance:tolerance:)`.
///
/// Use when: drawing the edge of a road or a wall from its centre line; one side of a
///   stroke; growing or shrinking a curved outline
/// Tags: offset curve, parallel curve, inset, outset, Bezier offset, stroke side
/// Node: inside offsetProfile
template <CurvePoint PointType>
std::optional<std::vector<CubicBezier<Vector2>>> CubicBezier<PointType>::offset(float distance, float tolerance) const
    requires std::same_as<PointType, Vector2>
{
    return offset(distance, distance, tolerance);
}

/// The curve offset to the right of its direction of travel by a distance that changes
/// linearly in the parameter from `startDistance` (at t = 0) to `endDistance` (at t = 1), as
/// cubics within `tolerance` of the true offset `O(t) = B(t) + d(t)·N(t)` (N the right unit
/// normal). The pieces join end to end exactly; the first starts at `start + d0·N(0)`, the
/// last ends at `end + d1·N(1)`. nil when no such list could be made (see the edge cases):
/// a result is never returned with a piece measured to be off by more than the tolerance.
///
/// How:
/// 1. Cut the parameter range into simple pieces (`simplePieceParameters`).
/// 2. Fit each piece with the cubic that has the true offset's end points and end
///    derivatives (`offsetFit(over:startDistance:endDistance:)`, Hermite matching).
/// 3. Measure the fit at the `offsetErrorSteps − 1` points inside it against the true offset
///    at the same parameter (`largestOffsetError`). A piece whose largest error is over
///    `measuredErrorShare` (½) of the tolerance is halved and both halves go back to step 2,
///    up to `mostOffsetHalvings` times; still over then, the offset is refused (nil).
///
/// Every point, direction and derivative is evaluated on the whole curve at the piece's
/// parameters, never on a split-off copy: a tiny piece's control points are a few ulps
/// apart, and directions taken from their differences would be noise. Neighbouring fits
/// therefore share their end point exactly (the same evaluation at the same parameter).
///
/// The tolerance contract: the error is checked at those points only, against half the
/// tolerance, and between them it stays within the whole tolerance on the curves tested
/// (smooth random curves measured densely, 41 points per piece, peaking at about 0.5 of the
/// tolerance; see the tests). It is a measured guarantee, not a proven bound.
///
/// The work is done with the curve moved so its start is at the origin, so far from the
/// origin nothing large cancels; the tolerance used is at least 4 ulps of the curve's
/// largest coordinate, which is how precisely the result can be stored out there.
///
/// Conventions: positive distance = right of travel = outward on a counterclockwise
/// outline. The distance is linear in the curve's *parameter*, not in arc length; on an
/// evenly paced curve the two agree.
///
/// Edge cases:
/// - Where the curvature radius on the offset side is smaller than the distance the true
///   offset itself folds over and makes a small loop (a "swallowtail"). The loop is kept:
///   removing it needs the booleans of batch 5c.
/// - At a cusp (the velocity is zero inside the curve) the direction flips, so the true
///   offset jumps by 2|d| and no list of joined cubics follows it: nil. Near a cusp the
///   offset swings round a tight half circle; when that needs more than `mostOffsetHalvings`
///   halvings it is nil as well. A handle sitting on its end point is not a cusp.
/// - A curve that is a single point, or has a non-finite control point or distance: nil.
/// - The tolerance must be positive and finite (a programming error otherwise).
///
/// Source: the cut into simple pieces follows Fountain `curves/offset.ts` and Pomax, *A
/// Primer on Bézier Curves*, "Curve offsetting"; the end-derivative fit is derived in
/// `offsetDerivative(at:startDistance:endDistance:)`.
///
/// Use when: one side of a stroke that gets thinner towards its end; a tapering road or
///   river bank; the outline of a brush stroke
/// Tags: variable offset, variable width stroke, tapered offset, graduated offset, Bezier
///   offset, parallel curve
/// Node: inside offsetProfile
template <CurvePoint PointType>
std::optional<std::vector<CubicBezier<Vector2>>> CubicBezier<PointType>::offset(
    float startDistance, float endDistance, float tolerance) const
    requires std::same_as<PointType, Vector2>
{
    if (!(tolerance > 0 && std::isfinite(tolerance))) throw MathProblem("An offset needs a positive, finite tolerance.");
    if (!(hasFiniteControlPoints() && std::isfinite(startDistance) && std::isfinite(endDistance)
          && startDirection() != Vector2())) {
        return std::nullopt;
    }
    Vector2 origin = start;
    CubicBezier<Vector2> local = translated(-origin);
    float largestCoordinate = largestCoordinateOf({start, startHandle, endHandle, end});
    float effectiveTolerance = largerOf(tolerance, 4 * curve_detail::unitInTheLastPlace(largestCoordinate));

    std::vector<CubicBezier<Vector2>> fitted;
    for (ParameterRange range : local.simplePieceParameters()) {
        std::optional<std::vector<CubicBezier<Vector2>>> pieces =
            local.offsetWithinTolerance(range, startDistance, endDistance, effectiveTolerance, 0);
        if (!pieces) return std::nullopt;
        fitted.insert(fitted.end(), pieces->begin(), pieces->end());
    }
    std::vector<CubicBezier<Vector2>> moved;
    for (const CubicBezier<Vector2>& piece : fitted) moved.push_back(piece.translated(origin));
    return moved;
}

/// Steps 2 and 3 of `offset(startDistance:endDistance:tolerance:)` for the piece over
/// `range`: fit, measure, halve while the error is over the tolerance. nil at a cusp, or
/// when the halving limit is reached with the error still over it.
template <CurvePoint PointType>
std::optional<std::vector<CubicBezier<Vector2>>> CubicBezier<PointType>::offsetWithinTolerance(
    ParameterRange range, float startDistance, float endDistance, float tolerance, int halvings) const
    requires std::same_as<PointType, Vector2>
{
    std::optional<CubicBezier<Vector2>> fit = offsetFit(range, startDistance, endDistance);
    if (!fit) return std::nullopt;
    float error = largestOffsetError(*fit, range, startDistance, endDistance);
    if (error <= tolerance * measuredErrorShare) return std::vector<CubicBezier<Vector2>>{*fit};
    if (!(halvings < mostOffsetHalvings)) return std::nullopt;
    float middle = linearBlend(range.lowerBound, range.upperBound, 0.5f);
    std::optional<std::vector<CubicBezier<Vector2>>> first =
        offsetWithinTolerance(ParameterRange{range.lowerBound, middle}, startDistance, endDistance, tolerance, halvings + 1);
    if (!first) return std::nullopt;
    std::optional<std::vector<CubicBezier<Vector2>>> second =
        offsetWithinTolerance(ParameterRange{middle, range.upperBound}, startDistance, endDistance, tolerance, halvings + 1);
    if (!second) return std::nullopt;
    first->insert(first->end(), second->begin(), second->end());
    return first;
}

/// The cubic that follows the offset of this curve over the parameters `range`: it starts
/// and ends where the offset does, and leaves and arrives with the offset's own
/// derivatives (a Hermite fit). The distance is linear in the parameter from
/// `startDistance` at t = 0 to `endDistance` at t = 1 of the whole curve. nil where the
/// curve has no direction at an end of the range (a cusp).
///
/// How: over the range a…b the piece's own parameter is `s = (t − a)/(b − a)`, so its
/// derivative is `(b − a)·O′(t)`, and a cubic Bézier's derivative at its ends is
/// `3(Q1 − Q0)` and `3(Q3 − Q2)`:
///
///     Q0 = O(a)                      Q3 = O(b)
///     Q1 = Q0 + (b − a)·O′(a) / 3    Q2 = Q3 − (b − a)·O′(b) / 3
///
/// with `O(t) = B(t) + d(t)·N(t)` and `O′` from `offsetDerivative`. For a constant distance on
/// a circular arc this is the concentric arc, its handles scaled by `1 + d·κ`.
///
/// Source: the Hermite-to-Bézier handle relation is Farin, *Curves and Surfaces for CAGD*
/// (Bézier end derivatives); applying it to the offset's derivative is derived here.
///
/// Use when: a quick offset of a piece that bends gently; one step of an accurate offset
/// Tags: Hermite fit, offset approximation, end derivative matching, offset handles
/// Node: none (solver)
template <CurvePoint PointType>
std::optional<CubicBezier<Vector2>> CubicBezier<PointType>::offsetFit(
    ParameterRange range, float startDistance, float endDistance) const
    requires std::same_as<PointType, Vector2>
{
    float width = range.upperBound - range.lowerBound;
    std::optional<Vector2> fitStart = offsetPoint(range.lowerBound, startDistance, endDistance);
    if (!fitStart) return std::nullopt;
    std::optional<Vector2> fitEnd = offsetPoint(range.upperBound, startDistance, endDistance);
    if (!fitEnd) return std::nullopt;
    std::optional<Vector2> startVelocity = offsetDerivative(range.lowerBound, startDistance, endDistance);
    if (!startVelocity) return std::nullopt;
    std::optional<Vector2> endVelocity = offsetDerivative(range.upperBound, startDistance, endDistance);
    if (!endVelocity) return std::nullopt;
    return CubicBezier<Vector2>(
        *fitStart, *fitStart + *startVelocity * (width / 3), *fitEnd - *endVelocity * (width / 3), *fitEnd);
}

/// The point of the true offset at `parameter`: `B(t) + d(t)·N(t)`, with d linear in t from
/// `startDistance` to `endDistance` and N the right unit normal; nil where the curve has no
/// direction (a cusp inside it, or a curve that is a single point).
///
/// Use when: one exact point of an offset or stroke edge, to fit or to check against
/// Tags: offset point, parallel curve point, normal offset, stroke edge point
/// Node: none (solver)
template <CurvePoint PointType>
std::optional<Vector2> CubicBezier<PointType>::offsetPoint(float parameter, float startDistance, float endDistance) const
    requires std::same_as<PointType, Vector2>
{
    std::optional<Vector2> direction = offsetTangent(parameter);
    if (!direction) return std::nullopt;
    float distance = linearBlend(startDistance, endDistance, parameter);
    return point(parameter) + direction->rightNormal() * distance;
}

/// The derivative, with respect to the parameter, of the offset `O(t) = B(t) + d(t)·N(t)`,
/// with d linear in t from `startDistance` to `endDistance` (so d′ = endDistance −
/// startDistance); nil where the curve has no direction.
///
/// How: N is the right unit normal (`rightNormal` of the unit tangent T = B′/|B′|). Turning
/// T by a right angle is linear, and T′ = κ·|B′|·(left normal) with κ the signed curvature
/// (counterclockwise positive), so N′ = κ·B′. Then
///
///     O′ = B′ + d·N′ + d′·N = B′·(1 + d·κ) + d′·N
///
/// and `κ·B′ = (B′ × B″) / |B′|² · T`. The ratio is taken of B′ and B″ divided by the control
/// polygon's largest extent (it is unchanged by that scale), so it neither overflows nor
/// underflows.
///
/// Where a handle sits on its end point, B′ = 0 at that end and the ratio is 0/0. Its
/// limit: near t = 0, B′ ≈ B″(0)·t + B‴·t²/2 and B″ ≈ B″(0) + B‴·t, so
/// B′ × B″ ≈ (t²/2)·B″(0) × B‴ and |B′|² ≈ t²·|B″(0)|², giving
/// `κ·B′ → (B″ × B‴) / (2|B″|²) · T` (the same at t = 1, from the other side). T is then
/// `startDirection` / `endDirection`, the limit direction. If B″ is 0 as well the curve is
/// straight there and the term is 0.
///
/// Conventions: y up, positive distance to the right of travel; `B‴ = 6(P3 − 3P2 + 3P1 − P0)`
/// is the same everywhere.
///
/// Use when: fitting a cubic to an offset by its end derivatives; the speed and direction
///   of a stroke's edge at a point
/// Tags: offset derivative, offset tangent, variable width tangent, Hermite data, curvature
/// Node: none (solver)
template <CurvePoint PointType>
std::optional<Vector2> CubicBezier<PointType>::offsetDerivative(
    float parameter, float startDistance, float endDistance) const
    requires std::same_as<PointType, Vector2>
{
    std::optional<Vector2> direction = offsetTangent(parameter);
    if (!direction) return std::nullopt;
    float distance = linearBlend(startDistance, endDistance, parameter);
    float extent = largestCoordinateOf({startHandle - start, endHandle - start, end - start});
    Vector2 velocity = firstDerivative(parameter);
    Vector2 scaledVelocity = velocity / extent;
    Vector2 scaledAcceleration = secondDerivative(parameter) / extent;
    float bendRate;
    if (scaledVelocity.lengthSquared() > 0) {
        bendRate = scaledVelocity.cross(scaledAcceleration) / scaledVelocity.lengthSquared();
    } else if (scaledAcceleration.lengthSquared() > 0) {
        bendRate = scaledAcceleration.cross(scaledThirdDerivative(extent)) / (2 * scaledAcceleration.lengthSquared());
    } else {
        bendRate = 0;
    }
    return velocity + *direction * (distance * bendRate) + direction->rightNormal() * (endDistance - startDistance);
}

/// The unit direction of travel at `parameter`, for offsetting: at the ends the limit
/// direction (`startDirection`, `endDirection`, right even when a handle sits on its end
/// point), inside the curve the normalized velocity; nil where there is none (a cusp, or a
/// curve that is a single point).
template <CurvePoint PointType>
std::optional<Vector2> CubicBezier<PointType>::offsetTangent(float parameter) const
    requires std::same_as<PointType, Vector2>
{
    std::optional<Vector2> direction;
    if (parameter == 0) {
        direction = startDirection();
    } else if (parameter == 1) {
        direction = endDirection();
    } else {
        direction = firstDerivative(parameter).unitDirection();
    }
    if (!direction || *direction == Vector2()) return std::nullopt;
    return direction;
}

/// The third derivative `B‴ = 6(P3 − 3P2 + 3P1 − P0)` divided by `extent`, the same at every
/// parameter. The control points are measured from the start before the sum, so a curve far
/// from the origin loses nothing to cancellation.
template <CurvePoint PointType>
Vector2 CubicBezier<PointType>::scaledThirdDerivative(float extent) const
    requires std::same_as<PointType, Vector2>
{
    Vector2 fromStart[3] = {(startHandle - start) / extent, (endHandle - start) / extent, (end - start) / extent};
    return (fromStart[2] - fromStart[1] * 3 + fromStart[0] * 3) * 6;
}

/// The largest distance, over the `offsetErrorSteps − 1` points inside `offsetPiece`, between
/// the piece and the true offset at the same parameter, where the piece fits the offset over
/// the parameters `range` (step 3 of `offset(startDistance:endDistance:tolerance:)`).
/// Infinity when a measurement is not a number or the curve has no direction at a sample,
/// so a broken fit always fails.
///
/// How: the piece's parameter s stands for `t = a + s·(b − a)` on the curve. At each sample
/// the error is `|Q(s) − O(t)|`, with `O(t) = B(t) + d(t)·N(t)` (`offsetPoint`). The fit shares
/// the offset's end points and end derivatives, so the two run at the same pace and the gap
/// at the same parameter is small exactly when the curves are close. It is never less than
/// the distance from Q(s) to the offset curve, so it is a safe measure, and unlike
/// projecting Q(s) onto the original it stays right inside a swallowtail, where points of
/// the true offset are nearer to some other part of the original than the distance.
///
/// Use when: checking an approximate offset against the true one
/// Tags: offset error, parametric distance, Hausdorff estimate, fit check
/// Node: none (solver)
template <CurvePoint PointType>
float CubicBezier<PointType>::largestOffsetError(
    const CubicBezier<Vector2>& offsetPiece, ParameterRange range, float startDistance, float endDistance) const
    requires std::same_as<PointType, Vector2>
{
    float largest = 0;
    for (int index = 1; index < offsetErrorSteps; index += 1) {
        float pieceParameter = static_cast<float>(index) / static_cast<float>(offsetErrorSteps);
        float parameter = linearBlend(range.lowerBound, range.upperBound, pieceParameter);
        std::optional<Vector2> trueOffset = offsetPoint(parameter, startDistance, endDistance);
        if (!trueOffset) return std::numeric_limits<float>::infinity();
        float error = offsetPiece.point(pieceParameter).distance(*trueOffset);
        // `Swift.max` would drop a NaN; a fit that cannot be measured is not within anything.
        if (!std::isfinite(error)) return std::numeric_limits<float>::infinity();
        largest = largerOf(largest, error);
    }
    return largest;
}

template class CubicBezier<Vector2>;

} // namespace editor
