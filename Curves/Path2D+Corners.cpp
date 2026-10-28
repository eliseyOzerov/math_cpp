// Corners of a path, and rounding them off (fillet) or cutting them flat (chamfer)
// (Engine/Math/Paths/Path2D+Corners.swift).
//
//     ───────────┐            ─────────╮              ─────────╲
//                │   fillet →          │   chamfer →           │
//                │                     │                       │
//
// Both trim the two pieces that meet at a corner back by the same length and bridge the gap:
// with an arc of a circle tangent to both (fillet) or a straight line (chamfer).
//
// Swift's closures are `std::function`; its private `replacingCorners`, `filletArc` and
// `TrimmedPiece` are file-local. `PathCorner` is declared in Path2D.h.
#include <cmath>
#include <optional>

#include "../Linear/Angle.h"
#include "ArcLength.h"
#include "../Linear/Scalar.h"
#include "Path2D.h"

namespace editor {

// MARK: - PathCorner

/// How far the direction turns at the corner, signed: positive for a left
/// (counterclockwise) turn, in −180°…180°. 0 means straight on.
///
/// Use when: telling convex corners of a counterclockwise outline (positive) from
///   concave ones; choosing a radius by how sharp a corner is
/// Tags: deflection angle, exterior angle, turn, bend angle
/// Node: inside roundCorners, chamferCorners
Angle PathCorner::turn() const { return Angle::signedTurn(incomingDirection, outgoingDirection); }

/// The angle inside the corner between the two pieces, 180° − |turn|: 90° for a square's
/// corner, 180° for no corner at all, near 0 for a hairpin.
///
/// Use when: sizing a fillet by the opening of a corner (the tangent length is
///   r / tan(interior / 2))
/// Tags: interior angle, opening angle, corner angle, included angle
/// Node: inside roundCorners, chamferCorners
Angle PathCorner::interiorAngle() const {
    return Angle::degrees(180) - Angle::fromRadians(std::fabs(turn().radians));
}

// MARK: - Corners

/// The corners found and filleted: those whose turn is at least this, by default. Smaller
/// kinks are left alone.
///
/// Use when: ignoring tiny kinks when finding, rounding or cutting corners
/// Tags: corner threshold, kink, minimum angle, smooth join
/// Node: none (validation)
Angle Path2D::defaultMinimumCornerTurn() { return Angle::degrees(1); }

/// The pieces the corners are found between: the edges, plus the closing line when the
/// path is closed (`edgesWithClosingLine`), without pieces of zero length.
///
/// Use when: indexing the pieces a `PathCorner` refers to
/// Tags: corner pieces, edges without empty ones
/// Node: inside roundCorners, chamferCorners
std::vector<Path2D::Edge> Path2D::cornerPieces() const {
    std::vector<Edge> result;
    for (const Edge& edge : closed ? edgesWithClosingLine() : edges()) {
        if (edge.hasLength()) result.push_back(edge);
    }
    return result;
}

/// The corners of the path whose turn is at least `minimumTurn`, in order along the path.
///
/// How: at every point where one of `cornerPieces` ends and the next starts (and, for a
/// closed path, where the last ends and the first starts), the incoming direction is the
/// end direction of the piece before and the outgoing direction the start direction of the
/// piece after (`CubicBezier.endDirection`, `startDirection`; a line's own direction).
/// The turn is the signed angle between them (`Angle.signedTurn`).
///
/// Conventions: the ends of an open path are not corners. Pieces of zero length are
/// skipped, so a doubled point is not a corner of its own.
///
/// Use when: finding the sharp corners of an outline before rounding them; listing the
///   vertices of a polygon with their angles
/// Tags: find corners, detect corners, vertices, kinks, sharp points, angle at vertex
/// Node: inside roundCorners, chamferCorners
std::vector<PathCorner> Path2D::corners(Angle minimumTurn) const {
    std::vector<Edge> pieces = cornerPieces();
    if (!(pieces.size() > 1 || (isClosed() && pieces.size() == 1))) return {};
    size_t firstOutgoing = isClosed() ? 0 : 1;
    std::vector<PathCorner> result;
    for (size_t outgoingIndex = firstOutgoing; outgoingIndex < pieces.size(); outgoingIndex += 1) {
        size_t incomingIndex = outgoingIndex == 0 ? pieces.size() - 1 : outgoingIndex - 1;
        PathCorner corner{
            static_cast<int>(outgoingIndex), pieces[outgoingIndex].start, pieces[incomingIndex].endDirection(),
            pieces[outgoingIndex].startDirection()};
        if (std::fabs(corner.turn().radians) >= minimumTurn.radians) result.push_back(corner);
    }
    return result;
}

namespace {

/// A piece of a path with a length trimmed off its start and its end (by arc length on a
/// cubic), and the directions at its new ends.
struct TrimmedPiece {
    Vector2 start;
    std::vector<Path2D::Segment> segments;
    Vector2 end;
    Vector2 startDirection;
    Vector2 endDirection;

    /// The piece `edge` with `startTrim` cut off its start and `endTrim` off its end.
    TrimmedPiece(const Path2D::Edge& edge, float startTrim, float endTrim) {
        if (std::optional<CubicBezier<Vector2>> cubic = edge.cubic()) {
            float length = cubic->totalLength();
            float lower = startTrim > 0 ? cubic->parameterAtDistance(startTrim) : 0;
            float upper = endTrim > 0 ? cubic->parameterAtDistance(length - endTrim) : 1;
            CubicBezier<Vector2> kept = cubic->portion(lower, largerOf(lower, upper));
            start = kept.start;
            end = kept.end;
            startDirection = kept.startDirection();
            endDirection = kept.endDirection();
            segments = {Path2D::Segment::cubic(kept.startHandle, kept.endHandle, kept.end)};
        } else {
            std::optional<Vector2> found = (edge.segment.end() - edge.start).unitDirection();
            Vector2 direction = found ? *found : Vector2();
            start = startTrim > 0 ? edge.start + direction * startTrim : edge.start;
            end = endTrim > 0 ? edge.segment.end() - direction * endTrim : edge.segment.end();
            startDirection = direction;
            endDirection = direction;
            segments = {Path2D::Segment::line(end)};
        }
    }
};

using Segment = Path2D::Segment;


/// Makes the segments that bridge a corner: the corner, the two cut points, the unit directions there.
using CornerBridge = std::function<std::vector<Segment>(const PathCorner&, Vector2, Vector2, Vector2, Vector2)>;

/// The arc that rounds `corner` from the cut `from` (where the path runs along `incoming`)
/// to the cut `to` (running along `outgoing`); see `filleted(minimumTurn:radius:)`.
std::vector<Segment> filletArc(const PathCorner& corner, Vector2 from, Vector2 to, Vector2 incoming, Vector2 outgoing) {
    Angle sweep = Angle::signedTurn(incoming, outgoing);
    if (sweep.radians == 0) return {Segment::line(to)};
    float halfSweepSine = std::fabs((sweep / 2).sine());
    float radius = from.distance(to) / (2 * halfSweepSine);
    if (!(radius > 0 && std::isfinite(radius))) return {Segment::line(to)};
    // The centre is on the inside of the turn: left of the direction for a left turn.
    Vector2 inward = sweep.radians > 0 ? incoming.perpendicular() : incoming.rightNormal();
    Vector2 centre = from + inward * radius;
    std::vector<CubicBezier<Vector2>> arcs;
    if (incoming == corner.incomingDirection && outgoing == corner.outgoingDirection) {
        // Both neighbours are straight here: the exact circle, in pieces of up to 90°.
        arcs = CubicBezier<Vector2>::circularArc(centre, radius, Angle::signedTurn(Vector2::xAxis(), from - centre), sweep);
    } else {
        float handleLength =
            CubicBezier<Vector2>::circularArcHandleFactor(Angle::fromRadians(std::fabs(sweep.radians))) * radius;
        arcs = {CubicBezier<Vector2>(from, from + incoming * handleLength, to - outgoing * handleLength, to)};
    }
    if (arcs.empty()) return {Segment::line(to)};
    std::vector<Segment> segments;
    for (size_t index = 0; index < arcs.size(); index += 1) {
        Vector2 end = index == arcs.size() - 1 ? to : arcs[index].end;
        segments.push_back(Segment::cubic(arcs[index].startHandle, arcs[index].endHandle, end));
    }
    return segments;
}

/// The shared work of fillets and chamfers: trim both pieces at each corner by the length
/// `trimLength(corner)` asks for (nil: leave it sharp), clamped to `largestCornerShare` of
/// each piece, and bridge the gap with the segments `bridge` makes. `bridge` gets the
/// corner, the two cut points, the unit directions there, and the trim actually used is
/// reflected in the cut points.
Path2D replacingCorners(
    const Path2D& path, Angle minimumTurn, const std::function<std::optional<float>(const PathCorner&)>& trimLength,
    const CornerBridge& bridge) {
    std::vector<Path2D::Edge> pieces = path.cornerPieces();
    if (pieces.empty()) return path;
    std::vector<float> lengths;
    for (const Path2D::Edge& piece : pieces) lengths.push_back(piece.length());
    // The trim at the start and at the end of every piece; zero where there is no corner.
    std::vector<float> startTrims(pieces.size(), 0);
    std::vector<float> endTrims(pieces.size(), 0);
    std::vector<std::optional<PathCorner>> cornerAtStart(pieces.size());
    for (const PathCorner& corner : path.corners(minimumTurn)) {
        // A full reversal has no tangent circle. `atan2` gives it as ±`Float.pi` (3.1415925),
        // which is below `Angle.degrees(180).radians` (3.1415927, rounded up), so the test
        // is against `Float.pi` itself.
        if (!(std::fabs(corner.turn().radians) < Angle::pi)) continue;
        std::optional<float> wanted = trimLength(corner);
        if (!wanted || !std::isfinite(*wanted)) continue;
        size_t outgoing = static_cast<size_t>(corner.outgoingPieceIndex);
        size_t incoming = outgoing == 0 ? pieces.size() - 1 : outgoing - 1;
        float trim = smallerOf(
            smallerOf(*wanted, Path2D::largestCornerShare * lengths[incoming]),
            Path2D::largestCornerShare * lengths[outgoing]);
        if (!(trim > 0)) continue;
        endTrims[incoming] = trim;
        startTrims[outgoing] = trim;
        cornerAtStart[outgoing] = corner;
    }
    // Rebuilding drops pieces of zero length and spells out the closing line; with nothing
    // to cut, the path is returned as it was.
    bool anyCorner = false;
    for (const std::optional<PathCorner>& corner : cornerAtStart) {
        if (corner) anyCorner = true;
    }
    if (!anyCorner) return path;

    std::vector<TrimmedPiece> trimmed;
    for (size_t index = 0; index < pieces.size(); index += 1) {
        trimmed.push_back(TrimmedPiece(pieces[index], startTrims[index], endTrims[index]));
    }
    // A closed path whose first corner is cut starts at the cut, so the bridge across that
    // corner is the last thing drawn and the path closes exactly at its start.
    Path2D result(trimmed[0].start);
    for (size_t index = 0; index < trimmed.size(); index += 1) {
        if (index > 0 && cornerAtStart[index]) {
            const TrimmedPiece& before = trimmed[index - 1];
            for (const Segment& segment :
                 bridge(*cornerAtStart[index], before.end, trimmed[index].start, before.endDirection, trimmed[index].startDirection)) {
                result.append(segment);
            }
        }
        for (const Segment& segment : trimmed[index].segments) result.append(segment);
    }
    if (path.isClosed()) {
        if (cornerAtStart[0]) {
            const TrimmedPiece& last = trimmed.back();
            for (const Segment& segment :
                 bridge(*cornerAtStart[0], last.end, trimmed[0].start, last.endDirection, trimmed[0].startDirection)) {
                result.append(segment);
            }
        }
        result.close();
    }
    return result;
}

} // namespace

/// The path with every corner (turn at least `minimumTurn`) rounded off by a circle arc of
/// `radius`. See `filleted(minimumTurn:radius:)`.
///
/// Use when: rounding the corners of a rectangle, a polygon or a profile by one radius
/// Tags: fillet, round corners, rounded rectangle, corner radius, border radius
/// Node: roundCorners
Path2D Path2D::filleted(float radius, Angle minimumTurn) const {
    return filleted(minimumTurn, [radius](const PathCorner&) { return radius; });
}

/// The path with each corner (turn at least `minimumTurn`) rounded off by a circle arc of
/// the radius `radius(corner)` gives it.
///
/// How: a circle of radius r tangent to both pieces touches each at the tangent length
/// from the corner,
///
///     tangent length = r / tan(θ/2) = r · tan(|turn|/2)      θ = interior angle
///
///          corner ●──── tangent length ──● cut
///                 │                    ╱
///                 │      r          ╱
///                 ● cut ────── ● centre
///
/// (the right triangle corner–cut–centre has the angle θ/2 at the corner and the side r
/// opposite it). Both pieces are trimmed back by that length, measured along the piece:
/// exactly for a line, by arc length for a cubic (`parameter(atDistance:)`, then
/// `portion`). The gap is bridged by the arc:
/// - between two lines: the exact circle arc (`CubicBezier.circularArc`, at most 90° per
///   cubic), centred `r` along the inner normal from the first cut;
/// - next to a curve: one cubic from cut to cut, its handles along the two directions at
///   the cuts with the circle-arc length `(4/3)·tan(sweep/4)·ρ`, ρ the radius of the circle
///   through both cuts with that sweep (`chord / (2·sin(sweep/2))`). It is the circle arc
///   exactly when the cuts are symmetric about the corner, as they are on lines, and
///   close to it otherwise.
///
/// Clamping: a corner may trim at most `largestCornerShare` (half) of each piece next to
/// it, so neighbouring corners never overlap. When the tangent length is more than that,
/// both pieces are trimmed by the smaller allowed length and the arc gets the radius that
/// fits, `trim / tan(|turn|/2)`, so it still meets both pieces tangentially (as Fountain's
/// `share`).
///
/// Edge cases: a radius that is not positive and finite (including NaN) leaves that corner
/// sharp, and so does a full reversal (turn ±180°), which no circle can round. A path with
/// no corner rounded (none sharp enough, or only reversals) comes back unchanged. The
/// arc's ends are the stored ends of the trimmed pieces, so the path stays joined exactly.
///
/// Source: the tangent length is textbook trigonometry; the clamping follows Fountain
/// `curves/corners.ts` (the user's own library).
///
/// Use when: rounding each corner by its own radius (sharper corners less, a chosen
///   corner more); fillets that depend on the corner's angle
/// Tags: variable fillet, per-corner radius, round corners, fillet, corner rounding
/// Node: roundCorners
Path2D Path2D::filleted(Angle minimumTurn, const std::function<float(const PathCorner&)>& radius) const {
    return replacingCorners(
        *this, minimumTurn,
        [&radius](const PathCorner& corner) -> std::optional<float> {
            float size = radius(corner);
            if (!(size > 0 && std::isfinite(size))) return std::nullopt;
            return size * (Angle::fromRadians(std::fabs(corner.turn().radians)) / 2).tangent();
        },
        [](const PathCorner& corner, Vector2 cutIn, Vector2 cutOut, Vector2 incomingCut, Vector2 outgoingCut) {
            return filletArc(corner, cutIn, cutOut, incomingCut, outgoingCut);
        });
}

/// `filleted(minimumTurn:radius:)` with the default minimum turn (Swift's trailing-closure form).
Path2D Path2D::filleted(const std::function<float(const PathCorner&)>& radius) const {
    return filleted(defaultMinimumCornerTurn(), radius);
}

/// The path with every corner (turn at least `minimumTurn`) cut flat, trimming `cut` off
/// each of the two pieces that meet there. See `chamfered(minimumTurn:cut:)`.
///
/// Use when: bevelling the corners of a box outline or a plate by one size
/// Tags: chamfer, bevel corners, cut corners, corner cut, flat corner
/// Node: chamferCorners
Path2D Path2D::chamfered(float cut, Angle minimumTurn) const {
    return chamfered(minimumTurn, [cut](const PathCorner&) { return cut; });
}

/// The path with each corner (turn at least `minimumTurn`) cut flat: both pieces that meet
/// there are trimmed back by `cut(corner)`, measured along each piece (by arc length on a
/// cubic), and the cut points are joined by a straight line.
///
/// On a square's corner a cut of c removes a right triangle with legs c, area c²/2.
///
/// Clamping and edge cases as for `filleted(minimumTurn:radius:)`: at most half of each
/// piece is trimmed; a cut that is not positive and finite leaves the corner sharp.
///
/// Use when: chamfering each corner by its own size; cutting only the sharpest corners
/// Tags: variable chamfer, per-corner chamfer, bevel, corner cut
/// Node: chamferCorners
Path2D Path2D::chamfered(Angle minimumTurn, const std::function<float(const PathCorner&)>& cut) const {
    return replacingCorners(
        *this, minimumTurn,
        [&cut](const PathCorner& corner) -> std::optional<float> {
            float size = cut(corner);
            if (!(size > 0 && std::isfinite(size))) return std::nullopt;
            return size;
        },
        [](const PathCorner&, Vector2, Vector2 cutOut, Vector2, Vector2) { return std::vector<Segment>{Segment::line(cutOut)}; });
}

/// `chamfered(minimumTurn:cut:)` with the default minimum turn (Swift's trailing-closure form).
Path2D Path2D::chamfered(const std::function<float(const PathCorner&)>& cut) const {
    return chamfered(defaultMinimumCornerTurn(), cut);
}

/// The unit direction the piece leaves its start in (see `CubicBezier.startDirection`).
///
/// Use when: the outgoing direction at a corner of a path
/// Tags: edge start tangent, outgoing direction
/// Node: inside roundCorners, chamferCorners
Vector2 Path2D::Edge::startDirection() const {
    if (std::optional<CubicBezier<Vector2>> curve = cubic()) return curve->startDirection();
    std::optional<Vector2> found = (segment.end() - start).unitDirection();
    return found ? *found : Vector2();
}

/// The unit direction the piece arrives at its end in (see `CubicBezier.endDirection`).
///
/// Use when: the incoming direction at a corner of a path
/// Tags: edge end tangent, incoming direction
/// Node: inside roundCorners, chamferCorners
Vector2 Path2D::Edge::endDirection() const {
    if (std::optional<CubicBezier<Vector2>> curve = cubic()) return curve->endDirection();
    std::optional<Vector2> found = (segment.end() - start).unitDirection();
    return found ? *found : Vector2();
}

} // namespace editor
