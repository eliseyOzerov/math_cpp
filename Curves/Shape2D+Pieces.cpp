// Outline pieces: lines and cubics held as cubics (see Shape2D+Pieces.h).
#include "Shape2D+Pieces.h"

#include <cmath>

#include "../Geometry/Segment2.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

/// The straight line from `start` to `end`, held as a cubic with its handles a third and two
/// thirds of the way along.
///
/// Use when: putting a line among curved pieces; building a cut piece of a line
/// Tags: line as cubic, straight piece, degree elevation
/// Node: none (internal piece)
OutlinePiece OutlinePiece::line(Vector2 start, Vector2 end) {
    return OutlinePiece{Path2D::evenlyPacedCubic(start, end), true};
}

/// The same piece with its ends moved to `newStart` and `newEnd`: a line is rebuilt between them, a
/// curve keeps its handles and only its end points move.
///
/// Use when: snapping the ends of consecutive pieces together; welding a loop shut
/// Tags: move ends, snap piece, weld
/// Node: none (internal piece)
OutlinePiece OutlinePiece::withEnds(Vector2 newStart, Vector2 newEnd) const {
    if (isLine) return line(newStart, newEnd);
    CubicBezier<Vector2> moved = cubic;
    moved.start = newStart;
    moved.end = newEnd;
    return curve(moved);
}

/// The piece run backwards.
///
/// Use when: turning an outline round; making fragments run the way their loop runs
/// Tags: reverse piece, flip direction
/// Node: none (internal piece)
OutlinePiece OutlinePiece::reversed() const { return OutlinePiece{cubic.reversed(), isLine}; }

/// The part of the piece from parameter `lower` to `upper`: a line is cut exactly, a curve by
/// `CubicBezier::portion`.
///
/// Use when: taking the stretch of a piece between two crossings
/// Tags: sub piece, trim, cut out, fragment
/// Node: none (internal piece)
OutlinePiece OutlinePiece::portion(float lower, float upper) const {
    if (isLine) {
        return line(Vector2::linearBlend(start(), end(), lower), Vector2::linearBlend(start(), end(), upper));
    }
    return curve(cubic.portion(lower, upper));
}

/// The length of the piece: the distance between the ends of a line, the arc length of a curve.
///
/// Use when: dropping pieces too short to matter; cutting a corner back by a length
/// Tags: piece length, arc length
/// Node: none (internal piece)
float OutlinePiece::length() const { return isLine ? start().distance(end()) : cubic.totalLength(); }

/// The point at `parameter` (0 at the start, 1 at the end).
///
/// Use when: probing the middle of a fragment
/// Tags: point on piece, evaluate
/// Node: none (internal piece)
Vector2 OutlinePiece::pointAt(float parameter) const {
    return isLine ? Vector2::linearBlend(start(), end(), parameter) : cubic.point(parameter);
}

/// The unit direction of travel at `parameter`; the zero vector where the piece has no direction.
///
/// Use when: stepping off a piece sideways to see which side is inside
/// Tags: piece tangent, direction of travel
/// Node: none (internal piece)
Vector2 OutlinePiece::directionAt(float parameter) const {
    if (isLine) return (end() - start()).normalized();
    return cubic.tangentDirection(parameter);
}

/// The parameter at which `distance` has been travelled along the piece, `distance` clamped into
/// 0…`knownLength`; a line is linear in length, a curve is searched (`parameterAtDistance`).
///
/// Use when: cutting a corner back by a length
/// Tags: parameter at length, arc length parameter
/// Node: none (internal piece)
float OutlinePiece::parameterAtDistance(float distance, float knownLength) const {
    if (distance <= 0) return 0;
    if (distance >= knownLength) return 1;
    return isLine ? distance / knownLength : cubic.parameterAtDistance(distance);
}

/// Whether the piece is a line or lies within `flatness` of its chord: both handles within
/// `flatness` of the segment between the ends.
///
/// Use when: treating nearly straight curves as lines before a boolean, so they can be cut exactly
/// Tags: nearly straight, flat curve, straightness test
/// Node: none (internal piece)
bool OutlinePiece::isStraightWithin(float flatness) const {
    if (isLine) return true;
    Segment2 chord{start(), end()};
    return chord.isNear(cubic.startHandle, flatness) && chord.isNear(cubic.endHandle, flatness);
}

/// The place on the piece nearest `target`: exact for a line, the curve's own search for a curve.
///
/// Use when: finding where one piece's end lies on another
/// Tags: nearest point on piece, project
/// Node: none (internal piece)
PathPoint2 OutlinePiece::nearestTo(Vector2 target) const {
    if (isLine) {
        SegmentFoot foot = Segment2{start(), end()}.closestFoot(target);
        return PathPoint2{0, foot.parameter, foot.point, foot.point.distance(target)};
    }
    return cubic.nearestPoint(target);
}

/// The pieces of the path as `OutlinePiece`s, closing line included, zero-length lines left out.
///
/// Use when: starting a boolean or a corner pass on a path
/// Tags: path to pieces, edges as cubics
/// Node: none (internal piece)
std::vector<OutlinePiece> outlinePieces(const Path2D& path) {
    std::vector<OutlinePiece> pieces;
    for (const Path2D::Edge& edge : path.edgesWithClosingLine()) {
        if (std::optional<CubicBezier<Vector2>> cubic = edge.cubic()) {
            pieces.push_back(OutlinePiece::curve(*cubic));
        } else if (!(edge.start == edge.segment.end())) {
            pieces.push_back(OutlinePiece::line(edge.start, edge.segment.end()));
        }
    }
    return pieces;
}

/// The pieces of the path as drawn: its edges, with no closing line.
///
/// Use when: reading an open profile or cap curve piece by piece
/// Tags: open path pieces, edges as cubics
/// Node: none (internal piece)
std::vector<OutlinePiece> drawnPieces(const Path2D& path) {
    std::vector<OutlinePiece> pieces;
    for (const Path2D::Edge& edge : path.edges()) {
        if (std::optional<CubicBezier<Vector2>> cubic = edge.cubic()) {
            pieces.push_back(OutlinePiece::curve(*cubic));
        } else {
            pieces.push_back(OutlinePiece::line(edge.start, edge.segment.end()));
        }
    }
    return pieces;
}

/// The pieces with every control point moved by `map`; a line is rebuilt between its mapped ends, so
/// it stays exactly straight.
///
/// Use when: placing a curve drawn in a local frame into the plane
/// Tags: transform pieces, map control points, place curve
/// Node: none (internal piece)
std::vector<OutlinePiece> mappedPieces(const std::vector<OutlinePiece>& pieces, const std::function<Vector2(Vector2)>& map) {
    std::vector<OutlinePiece> result;
    for (const OutlinePiece& piece : pieces) {
        if (piece.isLine) {
            result.push_back(OutlinePiece::line(map(piece.start()), map(piece.end())));
        } else {
            result.push_back(OutlinePiece::curve(CubicBezier<Vector2>(
                map(piece.cubic.start), map(piece.cubic.startHandle), map(piece.cubic.endHandle), map(piece.cubic.end))));
        }
    }
    return result;
}

/// The pieces with the first one's start moved to `first` and the last one's end to `last`. Needs at
/// least one piece.
///
/// Use when: pinning the ends of a placed curve exactly onto the cut points
/// Tags: pin ends, snap curve ends
/// Node: none (internal piece)
std::vector<OutlinePiece> pinnedPieces(std::vector<OutlinePiece> pieces, Vector2 first, Vector2 last) {
    pieces.front() = pieces.front().withEnds(first, pieces.front().end());
    pieces.back() = pieces.back().withEnds(pieces.back().start(), last);
    return pieces;
}

/// Each piece moved to start where the one before ends, the first where the last ends.
///
/// Use when: closing the gaps that cutting and snapping leave in a loop
/// Tags: stitch, weld loop, close gaps
/// Node: none (internal piece)
std::vector<OutlinePiece> stitchedPieces(const std::vector<OutlinePiece>& pieces) {
    std::vector<OutlinePiece> result;
    size_t count = pieces.size();
    for (size_t index = 0; index < count; index += 1) {
        result.push_back(pieces[index].withEnds(pieces[(index + count - 1) % count].end(), pieces[index].end()));
    }
    return result;
}

/// A path along `pieces`, closed if asked. An empty list is a problem: a path needs a start.
///
/// Use when: turning the result of a boolean or a corner pass back into a path
/// Tags: pieces to path, build path
/// Node: none (internal piece)
Path2D pathOfPieces(const std::vector<OutlinePiece>& pieces, bool closed) {
    if (pieces.empty()) throw MathProblem("A path needs at least one piece.");
    Path2D path(pieces.front().start());
    for (const OutlinePiece& piece : pieces) {
        if (piece.isLine) {
            path.addLine(piece.end());
        } else {
            path.addCubic(piece.cubic.startHandle, piece.cubic.endHandle, piece.end());
        }
    }
    if (closed) path.close();
    return path;
}

} // namespace editor
