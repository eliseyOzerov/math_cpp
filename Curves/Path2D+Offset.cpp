// The offset of a whole path: every piece offset on its own (lines exactly, cubics by
// `CubicBezier::offset`), and the gaps that open at the corners closed with a join
// (Engine/Math/Paths/Path2D+Offset.swift).
//
//     ┌───────────┐ ← offset         miter: the offset edges extended until they meet
//     │ ┌───────┐ │                  round: an arc around the corner
//     │ │ path  │ │                  bevel: a straight cut across the gap
//
// Sign, as for `CubicBezier::offset`: a positive distance moves to the RIGHT of the direction of
// travel, which is outward on a counterclockwise closed path (one with positive
// `signedArea`). A negative distance shrinks such a path.
//
// Swift's `private static` helpers `miterTip` and `joinSegments` are file-local functions here.
#include <cmath>
#include <optional>

#include "ArcLength.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "Path2D.h"

namespace editor {

namespace {

/// One piece of a path, offset: its segments, where it starts and ends, the directions there,
/// and the corner of the original path it ends at.
struct OffsetPiece {
    Vector2 start;
    std::vector<Path2D::Segment> segments;
    Vector2 end;
    Vector2 startDirection;
    Vector2 endDirection;
    float startDistance = 0;
    float endDistance = 0;
    Vector2 originalEnd;

    /// Offsets `edge` (step 2 of `Path2D::offset(startDistance:endDistance:join:tolerance:)`);
    /// nothing when its curve cannot be offset within the tolerance.
    static std::optional<OffsetPiece> of(const Path2D::Edge& edge, float startDistance, float endDistance, float tolerance) {
        OffsetPiece piece;
        piece.startDistance = startDistance;
        piece.endDistance = endDistance;
        piece.originalEnd = edge.segment.end();
        if (std::optional<CubicBezier<Vector2>> cubic = edge.cubic()) {
            piece.startDirection = cubic->startDirection();
            piece.endDirection = cubic->endDirection();
            std::optional<std::vector<CubicBezier<Vector2>>> pieces = cubic->offset(startDistance, endDistance, tolerance);
            if (!pieces) return std::nullopt;
            piece.start = pieces->empty() ? edge.start + piece.startDirection.rightNormal() * startDistance : pieces->front().start;
            piece.end = pieces->empty() ? edge.segment.end() + piece.endDirection.rightNormal() * endDistance : pieces->back().end;
            for (const CubicBezier<Vector2>& offsetCubic : *pieces) {
                piece.segments.push_back(
                    Path2D::Segment::cubic(offsetCubic.startHandle, offsetCubic.endHandle, offsetCubic.end));
            }
        } else {
            std::optional<Vector2> found = (edge.segment.end() - edge.start).unitDirection();
            Vector2 direction = found ? *found : Vector2();
            piece.startDirection = direction;
            piece.endDirection = direction;
            piece.start = edge.start + direction.rightNormal() * startDistance;
            piece.end = edge.segment.end() + direction.rightNormal() * endDistance;
            piece.segments = {Path2D::Segment::line(piece.end)};
        }
        return piece;
    }
};

/// Where the offset edges meet when carried on past the corner: the miter tip; nil when
/// they do not meet ahead of the corner.
///
/// How: with the same distance d on both sides and unit right normals nₐ (incoming) and
/// n_b (outgoing), the tip is `V + d·(nₐ + n_b) / (1 + nₐ·n_b)`: it lies on both offset
/// lines, since `(nₐ + n_b)·nₐ = 1 + nₐ·n_b`. This form stays accurate when the turn is
/// tiny (the tip is then the midpoint of the two ends), where intersecting the two lines
/// would divide one rounding error by another. When the distances differ (only at the
/// seam of a closed path whose start and end distances differ), the lines through the
/// ends are intersected: `E₁ + a·λ` with `λ = ((E₂ − E₁) × b) / (a × b)`.
std::optional<Vector2> miterTip(const OffsetPiece& first, const OffsetPiece& second) {
    Vector2 incomingNormal = first.endDirection.rightNormal();
    Vector2 outgoingNormal = second.startDirection.rightNormal();
    if (first.endDistance == second.startDistance) {
        float denominator = 1 + incomingNormal.dot(outgoingNormal);
        if (!(denominator > 0)) return std::nullopt;
        return first.originalEnd + (incomingNormal + outgoingNormal) * (first.endDistance / denominator);
    }
    float turn = first.endDirection.cross(second.startDirection);
    float along = (second.start - first.end).cross(second.startDirection) / turn;
    if (!(along >= 0 && std::isfinite(along))) return std::nullopt;
    return first.end + first.endDirection * along;
}

/// The segments that join the end of `first` to the start of `second` (step 3 of
/// `offset(startDistance:endDistance:join:tolerance:)`); none when they already meet.
std::vector<Path2D::Segment> joinSegments(const OffsetPiece& first, const OffsetPiece& second, Path2D::OffsetJoin join) {
    using Segment = Path2D::Segment;
    Vector2 from = first.end;
    Vector2 to = second.start;
    if (from == to) return {};
    Vector2 corner = first.originalEnd;
    Vector2 incoming = first.endDirection;
    Vector2 outgoing = second.startDirection;
    float turn = incoming.cross(outgoing);
    float distance = first.endDistance;
    // An inner corner, a reversal or no turn at all: the ends are joined straight. The signs
    // are compared, not multiplied: the product of a tiny turn and a tiny distance can
    // underflow to 0.
    bool turnsAwayFromOffsetSide = (turn > 0 && distance > 0) || (turn < 0 && distance < 0);
    if (!turnsAwayFromOffsetSide) return {Segment::line(to)};

    switch (join.kind) {
    case Path2D::OffsetJoin::Kind::bevel:
        return {Segment::line(to)};
    case Path2D::OffsetJoin::Kind::miter: {
        std::optional<Vector2> tip = miterTip(first, second);
        if (!tip) return {Segment::line(to)};
        float reach = largerOf(std::fabs(first.endDistance), std::fabs(second.startDistance));
        if (!(tip->distance(corner) <= join.limit * reach)) return {Segment::line(to)};
        return {Segment::line(*tip), Segment::line(to)};
    }
    case Path2D::OffsetJoin::Kind::round: {
        float radius = from.distance(corner);
        Angle startAngle = Angle::signedTurn(Vector2::xAxis(), from - corner);
        Angle sweep = Angle::signedTurn(from - corner, to - corner);
        std::vector<CubicBezier<Vector2>> arcs = CubicBezier<Vector2>::circularArc(corner, radius, startAngle, sweep);
        if (arcs.empty()) return {Segment::line(to)};
        // The arc's own end points are only rounding away from the offset ends; the ends
        // themselves are used, so the pieces meet exactly.
        std::vector<Segment> segments;
        for (size_t index = 0; index < arcs.size(); index += 1) {
            Vector2 end = index == arcs.size() - 1 ? to : arcs[index].end;
            segments.push_back(Segment::cubic(arcs[index].startHandle, arcs[index].endHandle, end));
        }
        return segments;
    }
    }
    return {Segment::line(to)};
}

} // namespace

/// The path offset by `distance` to the right of its direction of travel (outward on a
/// counterclockwise closed path), with corners joined by `join`, and curves within
/// `tolerance` of their true offset. See `offset(startDistance:endDistance:join:tolerance:)`.
///
/// Use when: growing or shrinking an outline (a margin around a shape, the wall around a
///   room); the edge of a road from its centre line
/// Tags: offset path, inset, outset, grow shape, shrink shape, parallel path, buffer,
///   Clipper offset, Illustrator offset path
/// Node: offsetProfile
std::optional<Path2D> Path2D::offset(float distance, OffsetJoin join, float tolerance) const {
    return offset(distance, distance, join, tolerance);
}

/// The path offset to the right of its direction of travel by a distance that changes
/// linearly with the length walked along the path, from `startDistance` at its start to
/// `endDistance` at its end; nil when the path has no piece of non-zero length, or when a
/// curve of it cannot be offset within `tolerance` (`CubicBezier.offset` gives nil: a cusp).
///
/// How:
/// 1. The pieces are the path's `edges`, plus the closing line when the path is closed
///    (`edgesWithClosingLine`). Pieces of zero length are skipped. Each piece gets the
///    distance at its ends from the length walked so far, so the distance is continuous
///    from piece to piece.
/// 2. A line is offset exactly: both ends move along its right normal. A cubic is offset
///    by `CubicBezier.offset(startDistance:endDistance:tolerance:)`; if that gives nil, so
///    does this.
/// 3. Where one piece ends and the next starts, the original corner V has the incoming
///    direction a and the outgoing direction b. If the offset ends already meet, nothing
///    is added. If the path turns *towards* the offset side (`a × b` and d of opposite
///    signs, an inner corner), the two offsets overlap and are joined by a straight line, which leaves a
///    small loop behind the corner. Otherwise a gap opens and `join` closes it:
///    - **miter**: the point where the line on through the first end (along a) meets the
///      line back through the second end (along −b), at `λ = ((E₂ − E₁) × b) / (a × b)`
///      along a; used when it is within `limit·|d|` of V, else a bevel;
///    - **round**: arcs around V (`CubicBezier.circularArc`) from the first end to the
///      second, sweeping the signed turn between them;
///    - **bevel**: a line from the first end to the second.
///    A closed path also joins its last piece to its first and is closed.
///
///         a ──────▶ V                 miter tip ●
///      E₁ ─────────●┐                 ╱       │
///                   │ b             E₁ ────── ● E₂ (bevel cuts across)
///                   ▼
///
/// Join points are always the stored ends of the offset pieces (an arc's first and last
/// points are replaced by them), so the pieces meet exactly and a closed result ends where
/// it starts.
///
/// Conventions: positive distance = right of travel = outward on a counterclockwise
/// closed path. The distance changes linearly in arc length from piece to piece, and
/// linearly in the parameter inside a cubic. On a closed path whose two distances differ,
/// the last-to-first join meets the start's distance from the end's.
///
/// Edge cases:
/// - Loops are not removed: behind inner corners and where a curve bends tighter than the
///   distance, the result folds over itself. Insetting the unit square by 0.1 with a miter
///   gives an outline that crosses itself 4 times and encloses 0.62 rather than 0.64.
///   Removing the loops (or routing inner joins through the corner) needs the booleans of
///   batch 5c; until then, check `selfCrossings` before trusting the area of an inset.
/// - A path that turns right round (180°) at a corner is joined straight across: the turn
///   there is ±180° only up to rounding, so neither side is reliably the outside.
/// - A miter limit below 1 or NaN never allows a miter: such a join is always a bevel.
/// - A non-finite distance gives nil. The tolerance must be positive and finite.
///
/// Source: the joins are the stroke joins of PostScript and SVG (`stroke-linejoin`,
/// `stroke-miterlimit`); the curve offsets are fitted by their end derivatives, see
/// `CubicBezier.offset`.
///
/// Use when: a tapering outline (a brush stroke, a river widening towards its mouth); one
///   side of a stroke whose width changes along the path
/// Tags: variable offset, variable width, tapered stroke, graduated offset, offset path
/// Node: offsetProfile
std::optional<Path2D> Path2D::offset(float startDistance, float endDistance, OffsetJoin join, float tolerance) const {
    if (!(tolerance > 0 && std::isfinite(tolerance))) throw MathProblem("An offset needs a positive, finite tolerance.");
    if (!(std::isfinite(startDistance) && std::isfinite(endDistance))) return std::nullopt;
    std::vector<Edge> pieces;
    for (const Edge& edge : isClosed() ? edgesWithClosingLine() : edges()) {
        if (edge.hasLength()) pieces.push_back(edge);
    }
    if (pieces.empty()) return std::nullopt;

    std::vector<float> lengths;
    for (const Edge& piece : pieces) lengths.push_back(piece.length());
    float totalLength = 0;
    for (float length : lengths) totalLength += length;
    float walked = 0;
    std::vector<OffsetPiece> offsetPieces;
    for (size_t index = 0; index < pieces.size(); index += 1) {
        auto distanceAt = [&](float length) -> float {
            if (!(totalLength > 0 && std::isfinite(totalLength))) return startDistance;
            return linearBlend(startDistance, endDistance, length / totalLength);
        };
        float before = walked;
        walked += lengths[index];
        bool isLast = index == pieces.size() - 1;
        std::optional<OffsetPiece> offsetPiece = OffsetPiece::of(
            pieces[index], distanceAt(before), isLast ? endDistance : distanceAt(walked), tolerance);
        if (!offsetPiece) return std::nullopt;
        offsetPieces.push_back(*offsetPiece);
    }

    Path2D path(offsetPieces[0].start);
    for (size_t index = 0; index < offsetPieces.size(); index += 1) {
        for (const Segment& segment : offsetPieces[index].segments) path.append(segment);
        bool isLast = index == offsetPieces.size() - 1;
        if (isLast && !isClosed()) break;
        const OffsetPiece& next = offsetPieces[isLast ? 0 : index + 1];
        for (const Segment& segment : joinSegments(offsetPieces[index], next, join)) path.append(segment);
    }
    if (isClosed()) path.close();
    return path;
}

/// Adds `segment` to the end of the path: a line or a cubic from the current point.
///
/// Use when: copying pieces from one path into another; building a path from segments
///   made elsewhere
/// Tags: append segment, add segment, extend path
/// Node: none (solver)
void Path2D::append(const Segment& segment) {
    switch (segment.kind) {
    case Segment::Kind::line:
        addLine(segment.endPoint);
        break;
    case Segment::Kind::cubic:
        addCubic(segment.startHandle, segment.endHandle, segment.endPoint);
        break;
    }
}

/// Whether the piece goes anywhere: false when all its points are the same to within
/// rounding (a line to its own start, a cubic collapsed to a point, or the hair-thin
/// closing line left when an arc ends a few ulps from where the path started), or when a
/// point is not finite.
///
/// How: every point is compared with the start. A gap counts only when one of its
/// components is larger than `4·ulp` of the largest coordinate among the points: below
/// that the gap is rounding noise, and the direction it points in is meaningless, so a
/// corner or an offset taken along it would be arbitrary.
///
/// Use when: skipping empty pieces before measuring directions or offsetting
/// Tags: degenerate segment, zero length edge, empty piece, rounding noise
/// Node: none (validation)
bool Path2D::Edge::hasLength() const {
    std::vector<Vector2> points{start, segment.endPoint};
    if (std::optional<CubicBezier<Vector2>> curve = cubic()) {
        points = {curve->start, curve->startHandle, curve->endHandle, curve->end};
    }
    // Checked first and explicitly: `Swift.max` and `Array.max` can drop a NaN, which
    // would leave a finite bound that a gap of NaN or infinity might still pass.
    for (Vector2 point : points) {
        if (!point.isFinite()) return false;
    }
    float largestCoordinate = 0;
    bool isFirst = true;
    for (Vector2 point : points) {
        float size = largerOf(std::fabs(point.x), std::fabs(point.y));
        if (isFirst || largestCoordinate < size) largestCoordinate = size;
        isFirst = false;
    }
    float roundingBound = 4 * curve_detail::unitInTheLastPlace(largestCoordinate);
    for (size_t index = 1; index < points.size(); index += 1) {
        Vector2 gap = points[index] - start;
        if (largerOf(std::fabs(gap.x), std::fabs(gap.y)) > roundingBound) return true;
    }
    return false;
}

/// The length of the piece: the distance between its ends for a line, the arc length
/// (`totalLength`) for a cubic.
///
/// Use when: sharing a width or a dash pattern along a path by length
/// Tags: edge length, segment length, arc length of piece
/// Node: inside offsetProfile
float Path2D::Edge::length() const {
    if (std::optional<CubicBezier<Vector2>> curve = cubic()) return curve->totalLength();
    return start.distance(segment.endPoint);
}

namespace {

/// One simple piece of a spine with the lengths along the whole spine where it starts and ends.
struct MeasuredPiece {
    CubicBezier<Vector2> curve;
    float startLength = 0;
    float endLength = 0;
};

/// The simple pieces of `spine` with the length where each starts and ends.
std::vector<MeasuredPiece> measuredSimplePieces(const CubicBezier<Vector2>& spine) {
    std::vector<MeasuredPiece> pieces;
    float run = 0;
    for (const CubicBezier<Vector2>& curve : spine.simplePiecesForOffset()) {
        float length = curve.totalLength();
        pieces.push_back(MeasuredPiece{curve, run, run + length});
        run += length;
    }
    return pieces;
}

/// The two sides of the outline of a piece: the left side forwards and the right side backwards
/// (each reversed to run from the piece's end to its start), with the distances tapering
/// linearly by length from `leftStart`/`rightStart` at the spine's start to `leftEnd`/`rightEnd`
/// at its end.
struct PieceSides {
    CubicBezier<Vector2> left;
    CubicBezier<Vector2> rightBackwards;
};

PieceSides sidesOf(
    const MeasuredPiece& piece, float totalLength, float leftStart, float leftEnd, float rightStart, float rightEnd) {
    auto taper = [&](float from, float to) {
        return [=, &piece](float parameter) {
            float length = piece.startLength + (piece.endLength - piece.startLength) * parameter;
            return from + (to - from) * length / totalLength;
        };
    };
    // A positive reach is to the right of travel, so the left side has a negative reach.
    OffsetReach leftReach = [leftTaper = taper(leftStart, leftEnd)](float parameter) { return -leftTaper(parameter); };
    OffsetReach rightReach = taper(rightStart, rightEnd);
    return PieceSides{piece.curve.scaledOut(leftReach), piece.curve.scaledOut(rightReach).reversed()};
}

} // namespace

/// A closed outline round a curve: `leftDistance` out to its left, `rightDistance` out to its
/// right (default the same), tapering to `leftEndDistance` and `rightEndDistance` at the curve's
/// end when given (by length along the curve).
///
/// How: the curve is cut into simple pieces (`CubicBezier::simplePiecesForOffset`), each side
/// is the pieces moved out with `CubicBezier::scaledOut(reach)` where the reach changes linearly
/// with the length along the whole curve. The path runs the left side forwards, a straight cap
/// across the end, the right side backwards, and a straight cap back to the start.
///
///     left side  ╭───────────────╮
///     start cap │ ●═════════════● │ end cap       the spine is ●═●; caps are straight
///     right side ╰───────────────╯
///
/// Conventions: y-up plane; the path runs the left side forwards, so a spine running left to right
/// gives a clockwise outline. Distances
/// are widths on that side, not signed offsets; a negative one pulls that side across the spine.
/// The sides are Bezier.js's handle-scaling fits, with no tolerance (see `CubicBezier::offsetByReach`).
/// Throws a `MathProblem` for a non-finite curve or distance.
///
/// Source: graphics_cpp curves/offset `outline` (Bezier.js `outline`).
///
/// Use when: drawing a thick, tapering stroke from a centre line (a leaf vein, a brush stroke, a road)
/// Tags: outline, stroke outline, variable width stroke, both sides, caps, Bezier.js outline
/// Node: planned (strokeOutline)
Path2D Path2D::strokeOutline(
    const CubicBezier<Vector2>& spine, float leftDistance, std::optional<float> rightDistance,
    std::optional<float> leftEndDistance, std::optional<float> rightEndDistance) {
    if (!spine.hasFiniteControlPoints() || !std::isfinite(leftDistance)) {
        throw MathProblem("An outline needs a curve with finite control points and a finite distance.");
    }
    float rightStart = rightDistance.value_or(leftDistance);
    float leftEnd = leftEndDistance.value_or(leftDistance);
    float rightEnd = rightEndDistance.value_or(rightStart);
    std::vector<MeasuredPiece> pieces = measuredSimplePieces(spine);
    float total = pieces.back().endLength > 0 ? pieces.back().endLength : 1;
    std::vector<CubicBezier<Vector2>> left;
    std::vector<CubicBezier<Vector2>> rightBackwards;
    for (const MeasuredPiece& piece : pieces) {
        PieceSides sides = sidesOf(piece, total, leftDistance, leftEnd, rightStart, rightEnd);
        left.push_back(sides.left);
        rightBackwards.insert(rightBackwards.begin(), sides.rightBackwards);
    }
    Path2D outline(left.front().start);
    for (const CubicBezier<Vector2>& piece : left) outline.addCubic(piece.startHandle, piece.endHandle, piece.end);
    outline.addLine(rightBackwards.front().start);
    for (const CubicBezier<Vector2>& piece : rightBackwards) outline.addCubic(piece.startHandle, piece.endHandle, piece.end);
    outline.close();
    return outline;
}

/// The outline of a curve (`strokeOutline`) as one closed path per simple piece: each its left
/// side, an end, its right side backwards and a start.
///
/// Conventions: the same distances and taper as `strokeOutline`, measured along the whole curve,
/// so the pieces join without a step. The paths are separate and may overlap at their caps.
///
/// Source: graphics_cpp curves/offset `outlineShapes`.
///
/// Use when: filling a variable-width stroke piece by piece, or colouring each piece differently
/// Tags: outline shapes, stroke pieces, outline per segment, tapered stroke polygons
/// Node: planned (strokeOutline)
std::vector<Path2D> Path2D::strokeOutlinePieces(
    const CubicBezier<Vector2>& spine, float leftDistance, std::optional<float> rightDistance,
    std::optional<float> leftEndDistance, std::optional<float> rightEndDistance) {
    if (!spine.hasFiniteControlPoints() || !std::isfinite(leftDistance)) {
        throw MathProblem("An outline needs a curve with finite control points and a finite distance.");
    }
    float rightStart = rightDistance.value_or(leftDistance);
    float leftEnd = leftEndDistance.value_or(leftDistance);
    float rightEnd = rightEndDistance.value_or(rightStart);
    std::vector<MeasuredPiece> pieces = measuredSimplePieces(spine);
    float total = pieces.back().endLength > 0 ? pieces.back().endLength : 1;
    std::vector<Path2D> shapes;
    for (const MeasuredPiece& piece : pieces) {
        PieceSides sides = sidesOf(piece, total, leftDistance, leftEnd, rightStart, rightEnd);
        Path2D shape(sides.left.start);
        shape.addCubic(sides.left.startHandle, sides.left.endHandle, sides.left.end);
        shape.addLine(sides.rightBackwards.start);
        shape.addCubic(sides.rightBackwards.startHandle, sides.rightBackwards.endHandle, sides.rightBackwards.end);
        shape.close();
        shapes.push_back(shape);
    }
    return shapes;
}

} // namespace editor
