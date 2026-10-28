// Where flat cubic Bézier curves cross a line, each other, or themselves.
//
// - A line: turn the picture so the line is the x axis; the crossings are the roots of the
//   curve's height above it, a cubic in t, solved exactly (`CubicRoots`).
// - Another cubic: no closed form (the equations have degree 9). Cut both curves in halves
//   again and again, keeping only the pairs of pieces whose boxes overlap, until the pieces
//   are tiny; then polish each candidate with Newton's method on both parameters at once.
// - Itself: a single cubic crosses itself at most once (its loop), and the two parameters come
//   out of a quadratic in closed form.
//
// Conventions: y up; parameters run 0…1 on each curve. A crossing at a parameter of exactly
// 0 or 1 reports the stored end point, never `point(parameter)`, so pieces that share an end agree on
// it to the bit.
//
// (Engine/Math/Curves/CubicBezier+Crossings.swift. `CurveCrossing` and `SelfCrossingParameters`
// are declared in CubicBezier.h; Swift's `nil` is an empty `std::optional`.)
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

#include "../Numerics/CubicRoots.h"
#include "../Numerics/QuadraticRoots.h"
#include "../Linear/Scalar.h"
#include "CubicBezier.h"

namespace editor {

namespace {

/// Swift's `Float.ulpOfOne`.
constexpr float ulpOfOne = std::numeric_limits<float>::epsilon();

using FlatCubic = CubicBezier<Vector2>;

/// Two pieces of the curves being crossed, with the parameter ranges they cover on the whole
/// curves and how many halvings made them (see `CubicBezier::crossings(other)`).
struct CrossingSearchPair {
    FlatCubic first;
    float firstLower;
    float firstUpper;
    FlatCubic second;
    float secondLower;
    float secondUpper;
    int depth;
};

/// The middle of a parameter range.
float middle(float lower, float upper) { return (lower + upper) / 2; }

/// The largest absolute coordinate of any control point of `curves`.
float largestCoordinate(const std::vector<FlatCubic>& curves) {
    float largest = 0;
    bool isFirst = true;
    for (const FlatCubic& curve : curves) {
        for (Vector2 control : {curve.start, curve.startHandle, curve.endHandle, curve.end}) {
            float size = largerOf(std::fabs(control.x), std::fabs(control.y));
            if (isFirst || largest < size) largest = size;
            isFirst = false;
        }
    }
    return largest;
}

/// Whether the boxes around the control points of two curves overlap (touching counts).
/// Each curve lies inside its box, so curves whose boxes are apart cannot meet. The box
/// corners are the exact minimum and maximum of the stored coordinates, so this test does
/// no rounding.
bool controlBoxesOverlap(const FlatCubic& first, const FlatCubic& second) {
    Bounds2 firstBox = Bounds2::enclosing({first.start, first.startHandle, first.endHandle, first.end});
    Bounds2 secondBox = Bounds2::enclosing({second.start, second.startHandle, second.endHandle, second.end});
    return firstBox.minimum.x <= secondBox.maximum.x && secondBox.minimum.x <= firstBox.maximum.x
        && firstBox.minimum.y <= secondBox.maximum.y && secondBox.minimum.y <= firstBox.maximum.y;
}

/// Step 1 of `crossings(other)`: the middles (t, u) of the tiny piece pairs whose boxes
/// still overlap after `crossingSubdivisionDepth` halvings; nothing past
/// `mostCrossingCandidates` of them (the curves overlap along a stretch).
std::optional<std::vector<Vector2>> candidatePairs(const FlatCubic& first, const FlatCubic& second) {
    std::vector<CrossingSearchPair> stack;
    stack.push_back(CrossingSearchPair{first, 0, 1, second, 0, 1, 0});
    std::vector<Vector2> candidates;
    while (!stack.empty()) {
        CrossingSearchPair pair = stack.back();
        stack.pop_back();
        if (!controlBoxesOverlap(pair.first, pair.second)) continue;
        if (pair.depth == FlatCubic::crossingSubdivisionDepth) {
            if (!(static_cast<int>(candidates.size()) < FlatCubic::mostCrossingCandidates)) return std::nullopt;
            candidates.push_back(
                Vector2(middle(pair.firstLower, pair.firstUpper), middle(pair.secondLower, pair.secondUpper)));
            continue;
        }
        CubicBezierSplit<Vector2> firstHalves = pair.first.split(0.5f);
        CubicBezierSplit<Vector2> secondHalves = pair.second.split(0.5f);
        float firstMiddle = middle(pair.firstLower, pair.firstUpper);
        float secondMiddle = middle(pair.secondLower, pair.secondUpper);
        struct Half {
            FlatCubic piece;
            float lower;
            float upper;
        };
        std::array<Half, 2> firstPieces{
            Half{firstHalves.first, pair.firstLower, firstMiddle}, Half{firstHalves.second, firstMiddle, pair.firstUpper}};
        std::array<Half, 2> secondPieces{
            Half{secondHalves.first, pair.secondLower, secondMiddle},
            Half{secondHalves.second, secondMiddle, pair.secondUpper}};
        for (const Half& firstPiece : firstPieces) {
            for (const Half& secondPiece : secondPieces) {
                stack.push_back(CrossingSearchPair{
                    firstPiece.piece, firstPiece.lower, firstPiece.upper, secondPiece.piece, secondPiece.lower,
                    secondPiece.upper, pair.depth + 1});
            }
        }
    }
    return candidates;
}

/// Step 2 of `crossings(other)`: Newton's method from the candidate (t, u), or nothing when it
/// does not end within `acceptance`.
std::optional<CurveCrossing> polishedCrossing(
    const FlatCubic& first, const FlatCubic& second, Vector2 candidate, float acceptance) {
    float firstParameter = candidate.x;
    float secondParameter = candidate.y;
    Vector2 gap = first.point(firstParameter) - second.point(secondParameter);
    for (int step = 0; step < FlatCubic::crossingPolishSteps; step += 1) {
        Vector2 firstVelocity = first.firstDerivative(firstParameter);
        Vector2 secondVelocity = second.firstDerivative(secondParameter);
        float determinant = firstVelocity.cross(secondVelocity);
        if (!(determinant != 0 && std::isfinite(determinant))) break;
        float nextFirst = clamped(firstParameter - gap.cross(secondVelocity) / determinant, 0, 1);
        float nextSecond = clamped(secondParameter - gap.cross(firstVelocity) / determinant, 0, 1);
        Vector2 nextGap = first.point(nextFirst) - second.point(nextSecond);
        // Keep only steps that bring the curves closer: near a tangent touch Newton can
        // overshoot, and the best point seen is the answer.
        if (!(nextGap.length() < gap.length())) break;
        firstParameter = nextFirst;
        secondParameter = nextSecond;
        gap = nextGap;
    }
    if (!(gap.length() <= acceptance)) return std::nullopt;
    return CurveCrossing{firstParameter, secondParameter, first.point(firstParameter)};
}

/// The parameter moved onto 0 or 1 when the crossing is that stored end of `curve` up to the
/// acceptance.
float snappedParameter(float parameter, const FlatCubic& curve, const CurveCrossing& crossing, float acceptance) {
    if (parameter <= FlatCubic::crossingMergeParameterDistance && curve.start.distance(crossing.point) <= acceptance) {
        return 0;
    }
    if (parameter >= 1 - FlatCubic::crossingMergeParameterDistance && curve.end.distance(crossing.point) <= acceptance) {
        return 1;
    }
    return parameter;
}

/// Moves a polished crossing onto a stored end point of either curve when it is that end
/// up to the acceptance (see "Ends exactly" on `crossings(other)`).
CurveCrossing snappedOntoEnds(
    const CurveCrossing& crossing, const FlatCubic& first, const FlatCubic& second, float acceptance) {
    float firstParameter = snappedParameter(crossing.firstParameter, first, crossing, acceptance);
    return CurveCrossing{
        firstParameter, snappedParameter(crossing.secondParameter, second, crossing, acceptance),
        first.storedEndOrPoint(firstParameter)};
}

/// Whether two polished crossings are the same one (step 3 of `crossings(other)`).
bool isSameCrossing(const CurveCrossing& first, const CurveCrossing& second, float acceptance, float mergeDistance) {
    float distance = first.point.distance(second.point);
    bool sameParameters =
        std::fabs(first.firstParameter - second.firstParameter) <= FlatCubic::crossingMergeParameterDistance
        && std::fabs(first.secondParameter - second.secondParameter) <= FlatCubic::crossingMergeParameterDistance;
    return (sameParameters && distance <= 2 * acceptance) || distance <= mergeDistance;
}

/// The roots in 0…1 of `q0·(1−t)² + q1·t(1−t) + q2·t²` (see `rootsOfBernsteinCubic`).
std::vector<float> rootsOfBernsteinQuadratic(float q0, float q1, float q2) {
    return CubicRoots::solve(0, q0 - q1 + q2, q1 - 2 * q0, q0, 0, 1);
}

std::vector<float> followedBy(std::vector<float> first, const std::vector<float>& second) {
    first.insert(first.end(), second.begin(), second.end());
    return first;
}

Vector2 magnitude(Vector2 vector) { return Vector2(std::fabs(vector.x), std::fabs(vector.y)); }

/// An upper bound on the rounding error of `a × b` in `selfCrossingParameters`, from the
/// control points measured from the start (and divided by their largest coordinate).
///
/// How: per coordinate (x shown), `a = d3 − 3d2 + 3d1` and `b = 3d2 − 6d1` are bounded by
/// the sums of their terms, `Aₓ = |d3ₓ| + 3|d2ₓ| + 3|d1ₓ|` and `Bₓ = 3|d2ₓ| + 6|d1ₓ|`. Each
/// difference dᵢ is rounded once when taken and once when divided, and each coefficient
/// takes up to three more roundings, so each component errs by at most `5·ε/2 ≤ 3ε` of
/// its bound (as `QuadraticWithRoundingBounds` in CubicBezier+Analysis, plus the
/// division). The cross product `aₓb_y − a_ybₓ` then carries `3ε·2·(AₓB_y + A_yBₓ)` from
/// its inputs, and its two products and one subtraction add under `ε·(AₓB_y + A_yBₓ)`:
///
///     bound = 8ε·(Aₓ·B_y + A_y·Bₓ)
///
/// Taken per coordinate rather than from lengths, the bound shrinks with a curve that is
/// flat along an axis, exactly as `a × b` does.
///
/// Use when: deciding whether `a × b` of a cubic is zero up to rounding
/// Tags: rounding bound, error bound, cross product error, loop test, floating point
float loopDenominatorBound(Vector2 toStartHandle, Vector2 toEndHandle, Vector2 toEnd) {
    Vector2 cubicSize = magnitude(toEnd) + magnitude(toEndHandle) * 3 + magnitude(toStartHandle) * 3;
    Vector2 quadraticSize = magnitude(toEndHandle) * 3 + magnitude(toStartHandle) * 6;
    return 8 * ulpOfOne * (cubicSize.x * quadraticSize.y + cubicSize.y * quadraticSize.x);
}

} // namespace

// MARK: - Crossings with a line

/// Where the curve meets the line through `lineStart` and `lineEnd`, ordered along the
/// curve. `secondParameter` is the line parameter s, with the point at
/// `lineStart + s·(lineEnd − lineStart)`; with `withinSegment` only crossings with
/// 0 ≤ s ≤ 1 are kept.
///
/// How: measure every control point's signed height above the line (subtracting
/// `lineStart` first, so far from the origin nothing large cancels):
///
///     hᵢ = u × (Pᵢ − lineStart)          u = the line's unit direction
///
/// Heights are linear, so the curve's height is the cubic with these Bernstein
/// coefficients, `h(t) = h0·(1−t)³ + 3h1·t(1−t)² + 3h2·t²(1−t) + h3·t³`, and the crossings
/// are its roots in 0…1 (`CubicRoots::solve`). The line parameter of each crossing is the
/// shadow of the point on the line, `s = (P − lineStart)·u / |lineEnd − lineStart|`.
///
///              ╭───╮
///     ────────●─────●───────●──── line (height 0)
///            ╱       ╲     ╱
///     ──────╯         ╰───╯         the curve's height changes sign at each ●
///
/// Ends exactly: a height within its rounding bound of zero counts as zero, and then the
/// end is a root exactly, *deflated* out of the cubic without rounding. With h0 = 0,
/// `h(t) = t·[3h1·(1−t)² + 3h2·t(1−t) + h3·t²]`; with h3 = 0,
/// `h(t) = (1−t)·[h0·(1−t)² + 3h1·t(1−t) + 3h2·t²]`; with both, `h(t) = t(1−t)·[3h1·(1−t) +
/// 3h2·t]`. The bracketed quadratic (or line) gives the other roots. A crossing at
/// t = 0 or 1 reports the stored end point.
///
/// Rounding bound of a height: the difference `Pᵢ − lineStart` is exact up to ε/2 of
/// itself, the unit direction is off by at most 2ε per component, and the cross product
/// adds three roundings, so a height errs by less than `8ε·(|dₓ| + |d_y|)`, d the
/// difference, ε = `Float.ulpOfOne`.
///
/// Edge cases:
/// - A curve lying along the line (every height within its rounding bound) has no
///   *isolated* crossings: every point is common. It returns no crossings; check
///   `isOnLine` style cases separately if they matter.
/// - A curve that only touches the line (a double root) reports the touching point once.
/// - A line through two equal points, or any non-finite input, gives no crossings.
/// - `withinSegment`: s within a rounding slack of 0 or 1 counts and is clamped onto it.
///
/// Source: the alignment approach is the one in Pomax, *A Primer on Bézier Curves*,
/// "Intersections: line–curve" (align the line with the x axis, find the roots).
///
/// Use when: cutting a profile by a straight line; finding where a path crosses a grid
///   line or the edge of a window
/// Tags: line curve intersection, Bezier line intersection, ray Bezier, cubic roots,
///   clip curve by line, segment intersection
/// Node: profileCrossings
template <CurvePoint PointType>
std::vector<CurveCrossing> CubicBezier<PointType>::crossings(Vector2 lineStart, Vector2 lineEnd, bool withinSegment) const
    requires std::same_as<PointType, Vector2>
{
    Vector2 lineVector = lineEnd - lineStart;
    std::optional<Vector2> lineDirectionIfAny = lineVector.unitDirection();
    if (!lineDirectionIfAny || !hasFiniteControlPoints() || !std::isfinite(lineStart.x) || !std::isfinite(lineStart.y)) {
        return {};
    }
    Vector2 lineDirection = *lineDirectionIfAny;
    float largestComponent = largerOf(std::fabs(lineVector.x), std::fabs(lineVector.y));
    float lineLength = largestComponent * (lineVector / largestComponent).length();

    std::array<Vector2, 4> offsets{start - lineStart, startHandle - lineStart, endHandle - lineStart, end - lineStart};
    std::vector<float> heights;
    std::array<bool, 4> isZero{};
    for (int index = 0; index < 4; index += 1) {
        heights.push_back(lineDirection.cross(offsets[index]));
        isZero[index] = std::fabs(heights[index]) <= heightRoundingBound(offsets[index]);
    }
    if (isZero[0] && isZero[1] && isZero[2] && isZero[3]) return {};
    if (isZero[0]) heights[0] = 0;
    if (isZero[3]) heights[3] = 0;

    std::vector<float> parameters = rootsOfBernsteinCubic(heights);
    std::vector<CurveCrossing> found;
    for (float parameter : parameters) {
        Vector2 crossingPoint = storedEndOrPoint(parameter);
        float along = (crossingPoint - lineStart).dot(lineDirection) / lineLength;
        if (!withinSegment) {
            found.push_back(CurveCrossing{parameter, along, crossingPoint});
            continue;
        }
        float slack = lineParameterSlack(crossingPoint - lineStart, lineLength);
        if (!(along >= -slack && along <= 1 + slack)) continue;
        found.push_back(CurveCrossing{parameter, clamped(along, 0, 1), crossingPoint});
    }
    return found;
}

/// The rounding bound `8ε·(|dₓ| + |d_y|)` of a height `u × d` above a line (derivation on
/// `crossings(lineStart, lineEnd, withinSegment)`).
///
/// Use when: deciding whether a control point lies on a line up to rounding
/// Tags: rounding bound, height above line, point on line test, orientation error
/// Node: none (solver)
template <CurvePoint PointType>
float CubicBezier<PointType>::heightRoundingBound(Vector2 offset) requires std::same_as<PointType, Vector2> {
    return 8 * ulpOfOne * (std::fabs(offset.x) + std::fabs(offset.y));
}

/// How far outside 0…1 a computed line parameter may fall and still count as on the
/// segment: its own rounding, about `8ε·(|dₓ| + |d_y|) / length`, plus 4 ulps of 1.
///
/// Use when: keeping a crossing that lands exactly on a segment's end point
/// Tags: segment end slack, parameter tolerance, rounding slack
/// Node: none (solver)
template <CurvePoint PointType>
float CubicBezier<PointType>::lineParameterSlack(Vector2 offset, float lineLength)
    requires std::same_as<PointType, Vector2>
{
    return heightRoundingBound(offset) / lineLength + 4 * ulpOfOne;
}

/// The roots in 0…1 of the cubic with Bernstein coefficients `heights` (h0…h3),
/// ascending and without repeats. An exactly zero h0 or h3 makes 0 or 1 a root exactly,
/// and it is divided out (deflated) before solving the rest.
///
/// How: with neither end zero, the power basis `h0 + 3(h1−h0)·t + 3(h2 − 2h1 + h0)·t² +
/// (h3 − 3h2 + 3h1 − h0)·t³` goes to `CubicRoots::solve`. With an end zero, the quadratic
/// left after dividing out t or (1 − t) (formulas on `crossings(lineStart, lineEnd, withinSegment)`) has
/// Bernstein coefficients q0, q1, q2 for `q0·(1−t)² + q1·t(1−t) + q2·t²`, whose power basis
/// is `q0 + (q1 − 2q0)·t + (q0 − q1 + q2)·t²`. Dividing in the Bernstein basis is exact:
/// it only drops a factor, it rounds nothing.
///
/// Use when: crossings of a Bézier curve with a line or an axis
/// Tags: Bernstein roots, Bezier polynomial roots, deflation, roots in unit interval
/// Node: none (solver)
template <CurvePoint PointType>
std::vector<float> CubicBezier<PointType>::rootsOfBernsteinCubic(const std::vector<float>& heights)
    requires std::same_as<PointType, Vector2>
{
    float h0 = heights[0];
    float h1 = heights[1];
    float h2 = heights[2];
    float h3 = heights[3];
    std::vector<float> roots;
    if (h0 != 0 && h3 != 0) {
        roots = CubicRoots::solve(h3 - 3 * h2 + 3 * h1 - h0, 3 * (h2 - 2 * h1 + h0), 3 * (h1 - h0), h0, 0, 1);
    } else if (h0 == 0 && h3 != 0) {
        roots = followedBy({0}, rootsOfBernsteinQuadratic(3 * h1, 3 * h2, h3));
    } else if (h0 != 0 && h3 == 0) {
        roots = followedBy(rootsOfBernsteinQuadratic(h0, 3 * h1, 3 * h2), {1});
    } else {
        // 3h1·(1−t) + 3h2·t: a line in Bernstein form, written as a quadratic with q0 = q2
        // matching it: (1−t) = (1−t)² + t(1−t), t = t(1−t) + t².
        roots = followedBy(
            followedBy({0}, rootsOfBernsteinQuadratic(3 * h1, 3 * h1 + 3 * h2, 3 * h2)), {1});
    }
    std::sort(roots.begin(), roots.end());
    std::vector<float> withoutRepeats;
    for (size_t index = 0; index < roots.size(); index += 1) {
        if (index == 0 || roots[index - 1] != roots[index]) withoutRepeats.push_back(roots[index]);
    }
    return withoutRepeats;
}

/// The point at `parameter`, but the stored end point when the parameter is exactly 0 or 1.
///
/// Use when: reporting where a crossing or a cut lies, so that pieces sharing an end agree
///   on it exactly
/// Tags: exact end point, point at parameter, stored end
/// Node: none (solver)
template <CurvePoint PointType>
Vector2 CubicBezier<PointType>::storedEndOrPoint(float parameter) const requires std::same_as<PointType, Vector2> {
    if (parameter == 0) return start;
    if (parameter == 1) return end;
    return point(parameter);
}

// MARK: - Crossings with another cubic

/// Two polished crossings closer than this in space are one crossing: 1/256 of the
/// curves' size (their largest coordinate after moving the first start to the origin).
///
/// Why: where the curves touch or cross at a shallow angle they stay within the
/// acceptance of each other along a stretch, and candidates there polish to points spread
/// along it, a little apart in both parameters. The tangent touch of `(2t − 1, (2t − 1)²)`
/// and its mirror spreads them over about 0.003 of its size; 1/256 is 0.0039.
///
/// Use when: merging the many points a tangent touch polishes to into one crossing
/// Tags: duplicate crossing, cluster, merge radius, tangent intersection, deduplicate
/// Node: none (solver)
template <CurvePoint PointType>
float CubicBezier<PointType>::crossingMergeDistance(float extent) requires std::same_as<PointType, Vector2> {
    return extent / 256;
}

/// How close the two curves must come at a polished crossing, `|A(t) − B(u)|`: 64 ε times
/// the curves' size (their largest coordinate after moving the first start to the origin).
/// Newton's method ends a few roundings of that size away; the margin covers the rounding
/// of `point(parameter)` itself and tangent crossings, which converge more slowly.
///
/// Use when: deciding that two curves really meet at a polished candidate
/// Tags: intersection tolerance, crossing acceptance, convergence tolerance
/// Node: none (solver)
template <CurvePoint PointType>
float CubicBezier<PointType>::crossingAcceptance(float extent) requires std::same_as<PointType, Vector2> {
    return 64 * ulpOfOne * extent;
}

/// Where this curve crosses (or touches) `other`, ordered along this curve.
/// `firstParameter` is on this curve, `secondParameter` on `other`.
///
/// How, in three steps:
/// 1. **Subdivide.** Both curves are moved so this curve's start is at the origin (nothing
///    large cancels far out). A curve lies inside the box around its four control points
///    (the convex hull property). If the boxes of two pieces do not overlap, the pieces
///    cannot meet; otherwise both are cut in half (de Casteljau) and the four pairs are
///    tried again. After `crossingSubdivisionDepth` halvings the surviving pairs are
///    candidates, at the middle of their parameter ranges.
///
///        ┌──┐┌──┐           boxes that overlap are halved again,
///        │ ╲││ ╱│           the others dropped, until the pieces are tiny
///        └──┘└──┘
///
/// 2. **Polish.** Newton's method on `F(t, u) = A(t) − B(u) = 0`, two equations in two
///    unknowns. With the velocities A′ and B′, the step solves `A′·Δt − B′·Δu = −F` by
///    Cramer's rule: `Δt = −(F × B′) / (A′ × B′)`, `Δu = −(F × A′) / (A′ × B′)`. Steps are
///    clamped to 0…1, and stop when the curves are parallel there (`A′ × B′ = 0`).
///    A candidate is kept when it ends within `crossingAcceptance` (`|F|`).
/// 3. **Merge.** Neighbouring candidates polish to the same crossing; two that agree in
///    both parameters within `crossingMergeParameterDistance` and in position within twice
///    the acceptance are one, and so are two closer in space than
///    `crossingMergeDistance(extent:)` (the points a tangent touch polishes to).
///
/// Ends exactly: a crossing within the acceptance of a stored end point, with its
/// parameter within the merge distance of that end, is moved onto it (parameter 0 or 1,
/// the stored point), so two pieces of a path that share an end report it exactly.
///
/// Nil means the answer cannot be trusted: the curves **overlap** along a stretch (the
/// same curve, one a piece of the other, or two that stay within the acceptance of each
/// other), so they meet at every point of it. It is detected two ways: more than
/// `mostCrossingCandidates` candidates, or more than `mostCrossingsOfTwoCubics` separate
/// crossings after merging. Any non-finite control point also gives nil, and so do two
/// curves that are both the same single point.
///
/// Limits: crossings closer together than 1/256 of the curves' size come back as one.
///
/// Source: subdivision on bounding boxes is Lane & Riesenfeld (1980), "A theoretical
/// development for the computer generation and display of piecewise polynomial
/// surfaces"; the Newton polish is the standard two-variable Newton's method.
///
/// Use when: cutting two outlines where they cross; finding where a road crosses a
///   river; the corners of a shape made by overlapping curves
/// Tags: curve curve intersection, Bezier intersection, subdivision, bounding box
///   intersection, Newton polish, clipping
/// Node: inside profileCrossings
template <CurvePoint PointType>
std::optional<std::vector<CurveCrossing>> CubicBezier<PointType>::crossings(const CubicBezier<Vector2>& other) const
    requires std::same_as<PointType, Vector2>
{
    if (!hasFiniteControlPoints() || !other.hasFiniteControlPoints()) return std::nullopt;
    Vector2 origin = start;
    FlatCubic first = translated(-origin);
    FlatCubic second = other.translated(-origin);
    float extent = largestCoordinate({first, second});
    if (!(extent > 0)) return std::nullopt;

    std::optional<std::vector<Vector2>> candidates = candidatePairs(first, second);
    if (!candidates) return std::nullopt;
    float acceptance = crossingAcceptance(extent);
    float mergeDistance = crossingMergeDistance(extent);
    std::vector<CurveCrossing> found;
    for (Vector2 candidate : *candidates) {
        std::optional<CurveCrossing> polished = polishedCrossing(first, second, candidate, acceptance);
        if (!polished) continue;
        CurveCrossing snapped = snappedOntoEnds(*polished, first, second, acceptance);
        bool isKnown = std::any_of(found.begin(), found.end(), [&](const CurveCrossing& known) {
            return isSameCrossing(known, snapped, acceptance, mergeDistance);
        });
        if (isKnown) continue;
        found.push_back(snapped);
        if (!(static_cast<int>(found.size()) <= mostCrossingsOfTwoCubics)) return std::nullopt;
    }
    std::stable_sort(found.begin(), found.end(), [](const CurveCrossing& left, const CurveCrossing& right) {
        return left.firstParameter < right.firstParameter;
    });
    std::vector<CurveCrossing> result;
    for (const CurveCrossing& crossing : found) {
        result.push_back(CurveCrossing{
            crossing.firstParameter, crossing.secondParameter, storedEndOrPoint(crossing.firstParameter)});
    }
    return result;
}

// MARK: - Crossing itself

/// The two parameters (smaller first) at which the curve passes through the same point
/// twice, the knot of its loop; nil when it has no loop.
///
/// How: in the power basis measured from the start (see `CubicBezier+Analysis`),
/// `B(t) = P0 + a·t³ + b·t² + c·t`. Two parameters s ≠ t land on the same point when
/// `B(s) − B(t) = 0`; dividing by `s − t`:
///
///     a·(s² + s·t + t²) + b·(s + t) + c = 0
///
/// With σ = s + t and π = s·t, `s² + s·t + t² = σ² − π`, so `a·(σ² − π) + b·σ + c = 0`, two
/// equations (x and y) in σ and π. Crossing them with a removes the `σ² − π` term, and
/// crossing with b removes σ:
///
///     σ = (c × a) / (a × b)          σ² − π = (b × c) / (a × b)
///
/// s and t are then the roots of `x² − σ·x + π = 0`. The curve has a loop when both are
/// real, different and inside 0…1.
///
///          ╭─╮
///          ╰─●─╮           ● is reached at t and again at s
///           ╱  ╰──
///
/// Scale: the control points are measured from the start and divided by their largest
/// coordinate first, so the cross products neither overflow nor underflow at any size.
///
/// Edge cases: `a × b` within its rounding bound of zero (`loopDenominatorBound`: the
/// curve is a parabola, a line or has a cusp) has no loop and gives nil. A double root
/// (σ² = 4π, a cusp) is not a loop. Roots a few ulps outside 0…1 do not count. A curve
/// whose start and end are the same point gives nil: (0, 1) is its only double point.
/// Non-finite control points give nil.
///
/// Thin loops: the rounding bound is taken per coordinate, so a loop flattened along x or
/// y is found at any thinness (a loop 1e-6 of its width tall is). A thin loop lying
/// at a slant mixes the long direction into both coordinates, and the bound with it.
/// Measured on the loop (0, 0), (3, h), (−2, h), (1, 0) turned by 0.3, 0.79 and 1 radian:
/// h = 1e-5 is found (its parameters off by up to 0.03, as the rounding of a × b is then a
/// good share of it), h = 1e-6 is missed. The turn itself rounds the control points by
/// about 1e-7, so such a loop is barely stored at all.
///
/// Source: derived here (the symmetric-function elimination written out above).
///
/// Use when: finding where a single curve crosses itself, to cut its loop off or to warn
///   about a twisted handle; self-crossings of a path
/// Tags: self intersection, loop, double point, knot, self-crossing cubic, crunode
/// Node: inside profileCrossings
template <CurvePoint PointType>
std::optional<SelfCrossingParameters> CubicBezier<PointType>::selfCrossingParameters() const
    requires std::same_as<PointType, Vector2>
{
    if (!hasFiniteControlPoints()) return std::nullopt;
    std::array<Vector2, 3> differences{startHandle - start, endHandle - start, end - start};
    float largest = 0;
    bool isFirst = true;
    for (Vector2 difference : differences) {
        float size = largerOf(std::fabs(difference.x), std::fabs(difference.y));
        if (isFirst || largest < size) largest = size;
        isFirst = false;
    }
    if (!(largest > 0)) return std::nullopt;
    Vector2 toStartHandle = differences[0] / largest;
    Vector2 toEndHandle = differences[1] / largest;
    Vector2 toEnd = differences[2] / largest;
    Vector2 cubic = toEnd - toEndHandle * 3 + toStartHandle * 3;
    Vector2 quadratic = toEndHandle * 3 - toStartHandle * 6;
    Vector2 linear = toStartHandle * 3;

    float denominator = cubic.cross(quadratic);
    if (!(std::fabs(denominator) > loopDenominatorBound(toStartHandle, toEndHandle, toEnd))) return std::nullopt;
    float sum = linear.cross(cubic) / denominator;
    float product = sum * sum - quadratic.cross(linear) / denominator;

    // A curve that ends where it starts (a teardrop) meets itself at (0, 1). The pair
    // (σ, π) solving the two equations is unique, so that is its only double point: its
    // closing corner, not a loop.
    if (!(start != end)) return std::nullopt;
    QuadraticRoots roots = QuadraticRoots::solve(1, -sum, product);
    if (roots.kind != QuadraticRoots::Kind::two) return std::nullopt;
    if (!(roots.smaller >= 0 && roots.larger <= 1 && roots.smaller < roots.larger)) return std::nullopt;
    return SelfCrossingParameters{roots.smaller, roots.larger};
}

template class CubicBezier<Vector2>;

} // namespace editor
