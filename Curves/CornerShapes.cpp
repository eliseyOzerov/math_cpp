// Corners of shapes and their replacement: finding them, cutting them back, drawing something else
// between the cuts. See CornerShapes.h. Ported from graphics_cpp curves/corners.cpp.
#include "CornerShapes.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

namespace {

/// The closed copy of an outline: an open outline is read as closed by a straight line, as a fill reads it.
Path2D closedCopy(const Path2D& outline) {
    Path2D copy = outline;
    if (!copy.isClosed()) copy.close();
    return copy;
}

/// The pieces corners are found between, as `OutlinePiece`s, in the order of `Path2D::cornerPieces`.
std::vector<OutlinePiece> cornerPieceList(const Path2D& closedOutline) {
    std::vector<OutlinePiece> pieces;
    for (const Path2D::Edge& edge : closedOutline.cornerPieces()) {
        if (std::optional<CubicBezier<Vector2>> cubic = edge.cubic()) {
            pieces.push_back(OutlinePiece::curve(*cubic));
        } else {
            pieces.push_back(OutlinePiece::line(edge.start, edge.segment.end()));
        }
    }
    return pieces;
}

std::vector<ShapeCorner> cornersOfOutline(const Path2D& closedOutline, int outlineIndex, Angle minimumTurn) {
    std::vector<ShapeCorner> corners;
    for (const PathCorner& corner : closedOutline.corners(minimumTurn)) {
        corners.push_back(ShapeCorner{
            outlineIndex, corner.outgoingPieceIndex, corner.point, corner.incomingDirection, corner.outgoingDirection,
            corner.turn()});
    }
    return corners;
}

/// Math.sign: -1, 0 or 1.
float signOf(float value) { return value > 0 ? 1.0f : (value < 0 ? -1.0f : 0.0f); }

/// How far a fillet of `radius` reaches along the sides of a corner that turns by `turn`:
/// r·tan(|turn|/2), the turn held under 0.999π so a hairpin stays finite.
float filletReach(float radius, Angle turn) {
    return radius * std::tan(smallerOf(std::fabs(turn.radians), Angle::pi * 0.999f) / 2);
}

/// The piece with `cutFromStart` of its length cut off its start and `cutFromEnd` off its end.
OutlinePiece cutBack(const OutlinePiece& piece, float length, float cutFromStart, float cutFromEnd) {
    return piece.portion(
        piece.parameterAtDistance(cutFromStart, length), piece.parameterAtDistance(length - cutFromEnd, length));
}

} // namespace

/// The angle of the wedge outside the outline at the corner: π + turn. A square's convex corner has
/// 270° of outside round it, a concave right angle 90°, a straight run 180°.
///
/// Use when: telling how sharp a pit is (the narrower the wedge, the more ink pools there)
/// Tags: outside angle, wedge, reflex angle, corner opening
/// Node: planned (shapeCorners)
Angle ShapeCorner::outsideAngle() const { return Angle::fromRadians(Angle::pi + turn.radians); }

/// Every corner of every outline of the shape that turns by at least `minimumTurn`, in order along each
/// outline (`Path2D::corners` of the outline closed by its last line).
///
/// Conventions: the inside is taken on the left of each outline (outer counterclockwise).
///
/// Use when: listing the vertices to round, trap or shape; marking the sharp points of a glyph
/// Tags: shape corners, find corners, vertices, kinks, sharp points, peaks and pits
/// Node: planned (shapeCorners)
std::vector<ShapeCorner> cornersOfShape(const Shape2D& shape, Angle minimumTurn) {
    std::vector<ShapeCorner> corners;
    for (size_t index = 0; index < shape.outlines.size(); index += 1) {
        std::vector<ShapeCorner> found =
            cornersOfOutline(closedCopy(shape.outlines[index]), static_cast<int>(index), minimumTurn);
        corners.insert(corners.end(), found.begin(), found.end());
    }
    return corners;
}

/// The pits of the shape: concave corners (a turn to the right) whose outside wedge is narrower than
/// `largestOutsideAngle`, straight or curved sides alike. Where strokes meet the ink pools, and the
/// sharper the wedge the more; these are the spots for ink traps and ink wells.
///
/// Use when: placing ink traps where a bar meets a stem (Z, E, the bowl of b)
/// Tags: pit, concave corner, ink trap, ink well, inner corner, reflex vertex
/// Node: planned (shapeCorners)
std::vector<ShapeCorner> pitsOfShape(const Shape2D& shape, Angle largestOutsideAngle) {
    std::vector<ShapeCorner> pits;
    for (const ShapeCorner& corner : cornersOfShape(shape)) {
        if (!corner.isConvex() && corner.outsideAngle().radians < largestOutsideAngle.radians) pits.push_back(corner);
    }
    return pits;
}

/// The peaks of the shape: convex corners (a turn to the left) turning by more than `smallestTurn`: the
/// corners of square ends, the apex of an A, a flat end cut at a slant. The other collection beside
/// the pits.
///
/// Use when: rounding or bevelling only the outer corners of a shape
/// Tags: peak, convex corner, outer corner, apex, tip
/// Node: planned (shapeCorners)
std::vector<ShapeCorner> peaksOfShape(const Shape2D& shape, Angle smallestTurn) {
    std::vector<ShapeCorner> peaks;
    for (const ShapeCorner& corner : cornersOfShape(shape)) {
        if (corner.isConvex() && corner.turn.radians > smallestTurn.radians) peaks.push_back(corner);
    }
    return peaks;
}

/// The shape with corners replaced: each corner of at least `options.minimumTurn` that `reach` gives
/// a length (0 or less leaves it) has both its sides cut back that far, but at most `options.share` of
/// each side, and `replacement` draws what goes between the cut points. Rounding, bevels, notches and
/// ink traps are all this.
///
/// How: a corner's sides run along the outline to the corners either side of it, over the pieces between
/// them that run on smoothly (a boolean splits an outline wherever it likes, and capping the cut at the
/// piece next to the corner made a well as big as one join's and a sliver at the next, by where the
/// split fell). The cut `d = min(reach, share·run to the next corner, share·run to the previous corner)`
/// is taken off whole pieces going out from the corner (dropping those it swallows) and the rest off
/// the piece it ends in, by arc length. The tangents given to `replacement` come from each side cut
/// back at this corner only: cut at both ends, a side as long as the two cuts would be left a point
/// with no direction.
///
///      ───────────┐    cut d back both ways   ─────────╮
///                 │  ─────────────────────▶            │   (replacement draws the ╮)
///
/// Conventions: y-up plane, inside on the left of every outline; outlines without corners are kept
/// as they are.
///
/// Source: graphics_cpp curves/corners.cpp `replaceCorners`.
///
/// Use when: any corner treatment the fillet and chamfer do not cover
/// Tags: replace corners, corner treatment, fillet, bevel, notch, cut corners, round corners
/// Node: planned (shapeCorners)
Shape2D replacedCorners(
    const Shape2D& shape, const std::function<float(const ShapeCorner&)>& reach, const CornerReplacement& replacement,
    CornerOptions options) {
    Shape2D result;
    result.fillRule = shape.fillRule;
    for (size_t outlineIndex = 0; outlineIndex < shape.outlines.size(); outlineIndex += 1) {
        Path2D outline = closedCopy(shape.outlines[outlineIndex]);
        std::vector<OutlinePiece> pieces = cornerPieceList(outline);
        std::vector<ShapeCorner> corners = cornersOfOutline(outline, static_cast<int>(outlineIndex), options.minimumTurn);
        if (corners.empty() || pieces.empty()) {
            result.outlines.push_back(shape.outlines[outlineIndex]);
            continue;
        }
        int count = static_cast<int>(pieces.size());
        std::vector<float> lengths;
        for (const OutlinePiece& piece : pieces) lengths.push_back(piece.length());
        std::set<int> uniqueStops;
        for (const ShapeCorner& corner : corners) uniqueStops.insert(corner.pieceIndex);
        std::vector<int> stops(uniqueStops.begin(), uniqueStops.end());
        auto runBetween = [&](int from, int to) {
            float total = 0;
            int index = from;
            do {
                total += lengths[static_cast<size_t>(index)];
                index = (index + 1) % count;
            } while (index != to);
            return total;
        };
        // What each piece loses at its start and end, the pieces cut away whole, and what goes in at each corner.
        std::vector<float> cutFromStart(static_cast<size_t>(count), 0);
        std::vector<float> cutFromEnd(static_cast<size_t>(count), 0);
        std::vector<char> isGone(static_cast<size_t>(count), 0);
        std::map<int, std::vector<OutlinePiece>> fills;
        for (const ShapeCorner& corner : corners) {
            float wanted = reach(corner);
            if (!(wanted > 0)) continue;
            int cornerPiece = corner.pieceIndex;
            size_t stopIndex = static_cast<size_t>(std::find(stops.begin(), stops.end(), cornerPiece) - stops.begin());
            int next = stops[(stopIndex + 1) % stops.size()];
            int previous = stops[(stopIndex + stops.size() - 1) % stops.size()];
            float distance = smallerOf(
                smallerOf(wanted, options.share * runBetween(cornerPiece, next)),
                options.share * runBetween(previous, cornerPiece));
            if (!(distance > 1e-9f)) continue;
            // Forward over the pieces from the corner, then back over those before it.
            int forward = cornerPiece;
            float remaining = distance;
            for (int step = 0; step < count && remaining > lengths[static_cast<size_t>(forward)] - 1e-9f && remaining > 1e-9f; step += 1) {
                isGone[static_cast<size_t>(forward)] = 1;
                remaining -= lengths[static_cast<size_t>(forward)];
                forward = (forward + 1) % count;
            }
            cutFromStart[static_cast<size_t>(forward)] = largerOf(0, remaining);
            int backward = (cornerPiece + count - 1) % count;
            float back = distance;
            for (int step = 0; step < count && back > lengths[static_cast<size_t>(backward)] - 1e-9f && back > 1e-9f; step += 1) {
                isGone[static_cast<size_t>(backward)] = 1;
                back -= lengths[static_cast<size_t>(backward)];
                backward = (backward + count - 1) % count;
            }
            cutFromEnd[static_cast<size_t>(backward)] = largerOf(0, back);
            OutlinePiece before = cutBack(
                pieces[static_cast<size_t>(backward)], lengths[static_cast<size_t>(backward)], 0, cutFromEnd[static_cast<size_t>(backward)]);
            OutlinePiece after = cutBack(
                pieces[static_cast<size_t>(forward)], lengths[static_cast<size_t>(forward)], cutFromStart[static_cast<size_t>(forward)], 0);
            fills[cornerPiece] = replacement(before.end(), before.endDirection(), after.start(), after.startDirection(), corner);
        }
        if (fills.empty()) {
            result.outlines.push_back(shape.outlines[outlineIndex]);
            continue;
        }
        std::vector<OutlinePiece> rebuilt;
        for (int index = 0; index < count; index += 1) {
            size_t position = static_cast<size_t>(index);
            auto fill = fills.find(index);
            if (fill != fills.end()) rebuilt.insert(rebuilt.end(), fill->second.begin(), fill->second.end());
            if (!isGone[position] && lengths[position] - cutFromStart[position] - cutFromEnd[position] > 1e-6f) {
                rebuilt.push_back(cutBack(pieces[position], lengths[position], cutFromStart[position], cutFromEnd[position]));
            }
        }
        result.outlines.push_back(pathOfPieces(stitchedPieces(rebuilt), true));
    }
    return result;
}

/// The frame a corner's replacement is drawn in: x from −1 at the first cut `firstCut` to 1 at the
/// second, and y from 0 on the chord between them to 1 at `corner` itself.
///
/// How: middle = (a + b)/2, halfChord = (b − a)/2, height = corner − middle. A corner on its chord
/// (no turn) has no height, so a unit square to the chord, `height = perpendicular(halfChord)`,
/// stands in. The map to the plane is affine, so a curve drawn in the frame keeps its shape class.
///
/// Conventions: y-up plane. `firstCut` and `secondCut` must differ or the call throws a `MathProblem`.
///
/// Use when: drawing an ink trap or a cove once and fitting it to every corner
/// Tags: corner frame, local frame, affine basis, notch coordinates, corner coordinates
/// Node: planned (shapeCorners)
CornerFrame cornerFrame(Vector2 firstCut, Vector2 secondCut, Vector2 corner) {
    if (firstCut == secondCut) throw MathProblem("A corner frame needs two different cut points.");
    Vector2 middle = Vector2::linearBlend(firstCut, secondCut, 0.5f);
    Vector2 halfChord = (secondCut - firstCut) * 0.5f;
    Vector2 height = corner - middle;
    if (std::fabs(halfChord.cross(height)) <= 1e-6f * halfChord.length() * height.length()) height = halfChord.perpendicular();
    return CornerFrame{middle, halfChord, height, halfChord.cross(height)};
}

/// The point of the plane at frame coordinates `local`: middle + halfChord·x + height·y.
///
/// Use when: placing a curve drawn in the corner's frame
/// Tags: frame to plane, local to world, affine map
/// Node: planned (shapeCorners)
Vector2 CornerFrame::toPlane(Vector2 local) const { return middle + halfChord * local.x + height * local.y; }

/// The frame coordinates of a point of the plane: the inverse of `toPlane`, by Cramer's rule on
/// the two columns halfChord and height (their determinant is `determinant`).
///
/// Use when: reading where a point lies relative to a corner's cuts
/// Tags: plane to frame, world to local, inverse affine map
/// Node: planned (shapeCorners)
Vector2 CornerFrame::fromPlane(Vector2 plane) const {
    Vector2 offset = plane - middle;
    return Vector2(offset.cross(height) / determinant, halfChord.cross(offset) / determinant);
}

/// The shape with each corner that `size` gives a radius (0 or less leaves it) replaced by a shaped
/// piece, cut back as a fillet of that radius would be (`radius·tan(turn/2)`).
///
/// How: with no tilt and no profile the replacement is the circular fillet arc (`arcCubic`). With a
/// `tilt` s in −1…1 the arc from cut to cut is made to leave the chord at the angle
/// `α = f + sign(f)·t·π/2` (held within ±0.98π), where `f` is the angle at which the fillet leaves
/// the chord and `t = s` at a peak and `−s` at a pit: the arc bulges further towards the corner
/// (adding ink at a peak, taking ink at a pit) as s grows; at s = ±1 it leaves the sides square. A
/// `profile` is drawn across a box from x −1 at the first cut to 1 at the second, its top as far off
/// the chord as that arc bulges, `(chord/2)·tan(α/2)`. A `curve` is drawn in the corner's own frame
/// (`cornerFrame`) with its ends pinned to the cuts. `seen` is told each corner's cuts as they are made.
///
/// Conventions: y-up plane; inside on the left of every outline; mirror image of graphics_cpp (which
/// keeps the inside on the right), so a peak is a left turn here.
///
/// Source: graphics_cpp curves/corners.cpp `shapeCorners`; arcs by `CubicBezier::arcBetween`.
///
/// Use when: ink traps, ink wells, soft or sharp corners that vary by corner; one corner shape across a font
/// Tags: shape corners, ink trap, corner profile, soften corners, tilted fillet, notch
/// Node: planned (shapeCorners)
Shape2D shapedCorners(
    const Shape2D& shape, const std::function<float(const ShapeCorner&)>& size,
    const std::function<CornerShape(const ShapeCorner&)>& shapeOfCorner,
    const std::function<void(const ShapeCorner&, Vector2, Vector2)>& seen) {
    return replacedCorners(
        shape, [&](const ShapeCorner& corner) { return filletReach(size(corner), corner.turn); },
        [&](Vector2 cutBefore, Vector2 directionBefore, Vector2 cutAfter, Vector2 directionAfter,
            const ShapeCorner& corner) -> std::vector<OutlinePiece> {
            CornerShape cornerShape = shapeOfCorner(corner);
            float tilt = cornerShape.tilt;
            if (seen) seen(corner, cutBefore, cutAfter);
            float chord = cutBefore.distance(cutAfter);
            if (!(chord > 1e-9f)) return {OutlinePiece::line(cutBefore, cutAfter)};
            if (cornerShape.curve) {
                std::vector<OutlinePiece> drawn = drawnPieces(*cornerShape.curve);
                if (!drawn.empty()) {
                    CornerFrame frame = cornerFrame(cutBefore, cutAfter, corner.point);
                    return pinnedPieces(mappedPieces(drawn, [&](Vector2 point) { return frame.toPlane(point); }), cutBefore, cutAfter);
                }
            }
            if (tilt == 0 && !cornerShape.profile) {
                return {OutlinePiece::curve(CubicBezier<Vector2>::arcCubic(cutBefore, directionBefore, cutAfter, directionAfter))};
            }
            Vector2 chordDirection = (cutAfter - cutBefore) / chord;
            // The fillet leaves at `leaving` off its chord, bulging towards the corner; a larger tilt bulges
            // further that way, which is away from the ink at a peak and into it at a pit.
            float leaving = Angle::signedTurn(chordDirection, directionBefore).radians;
            float towards = corner.isConvex() ? tilt : -tilt;
            float angle = clamped(leaving + signOf(leaving) * towards * Angle::pi / 2, -Angle::pi * 0.98f, Angle::pi * 0.98f);
            if (std::fabs(angle) < 1e-6f) return {OutlinePiece::line(cutBefore, cutAfter)};
            if (cornerShape.profile) {
                std::vector<OutlinePiece> drawn = drawnPieces(*cornerShape.profile);
                if (!drawn.empty()) {
                    // The arc leaving at α bulges (chord/2)·tan(α/2) off its chord, to the left of it for α > 0.
                    float bulge = chord / 2 * std::tan(angle / 2);
                    Vector2 left = chordDirection.perpendicular();
                    float top = cornerShape.profile->bounds().maximum.y;
                    auto place = [&](Vector2 point) {
                        float along = (point.x + 1) / 2 * chord;
                        float across = top > 1e-9f ? point.y / top * bulge : 0;
                        return cutBefore + chordDirection * along + left * across;
                    };
                    return pinnedPieces(mappedPieces(drawn, place), cutBefore, cutAfter);
                }
            }
            std::vector<OutlinePiece> arcs;
            for (const CubicBezier<Vector2>& arc : CubicBezier<Vector2>::arcBetween(cutBefore, cutAfter, Angle::fromRadians(angle))) {
                arcs.push_back(OutlinePiece::curve(arc));
            }
            return arcs;
        });
}

/// A single path's corners shaped (see the shape version): the path read as one outline.
///
/// Use when: shaping the corners of one outline without wrapping it in a shape
/// Tags: shape path corners, ink trap path, round path corners with profile
/// Node: planned (shapeCorners)
Path2D shapedCorners(
    const Path2D& path, const std::function<float(const ShapeCorner&)>& size,
    const std::function<CornerShape(const ShapeCorner&)>& shapeOfCorner) {
    Shape2D single;
    single.outlines.push_back(path);
    return shapedCorners(single, size, shapeOfCorner).outlines.front();
}

} // namespace editor
