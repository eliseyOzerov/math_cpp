// The pieces of an outline as the booleans and the corner shapes work on them: every piece a cubic,
// with a flag for the ones that are straight lines. Path2D keeps lines and cubics apart, which is
// right for storing a path but means every step of a boolean would branch on the kind; here a line
// is a cubic with its handles at a third and two thirds (the same shape `Path2D::evenlyPacedCubic`
// makes), so one piece type can be cut, reversed and measured. Ported from graphics_cpp's `Piece`
// (Curves2D).
#pragma once

#include <functional>
#include <vector>

#include "CubicBezier.h"
#include "Path2D.h"

namespace editor {

/// One piece of an outline: a straight line or a cubic Bézier, held as a cubic.
///
///     line   ●────────────●          held as a cubic with its handles on the line, a third and
///                                    two thirds of the way along, so parameter t is the fraction
///                                    of the way along the line
///
/// Conventions: y-up plane. `isLine` is true for a piece that came from a line (or was judged
/// straight); the handles of a line are always rebuilt from its ends, never trusted.
///
/// Use when: cutting, reversing or measuring the pieces of an outline without caring whether they
///   are lines or curves
/// Tags: outline piece, segment as cubic, line or curve, path piece
/// Node: none (internal piece)
struct OutlinePiece {
    CubicBezier<Vector2> cubic;
    bool isLine = false;

    bool operator==(const OutlinePiece&) const = default;

    /// The straight line from `start` to `end`, held as a cubic.
    static OutlinePiece line(Vector2 start, Vector2 end);

    /// A curved piece.
    static OutlinePiece curve(const CubicBezier<Vector2>& cubic) { return OutlinePiece{cubic, false}; }

    Vector2 start() const { return cubic.start; }
    Vector2 end() const { return cubic.end; }

    OutlinePiece withEnds(Vector2 newStart, Vector2 newEnd) const;
    OutlinePiece reversed() const;
    OutlinePiece portion(float lower, float upper) const;
    float length() const;
    Vector2 pointAt(float parameter) const;
    Vector2 directionAt(float parameter) const;
    float parameterAtDistance(float distance, float knownLength) const;
    bool isStraightWithin(float flatness) const;
    PathPoint2 nearestTo(Vector2 target) const;
    Vector2 startDirection() const { return directionAt(0); }
    Vector2 endDirection() const { return directionAt(1); }
};

/// The pieces of `path` in drawing order, with the closing line added when the pen does not end
/// where it began (see `Path2D::edgesWithClosingLine`), leaving out lines of zero length.
std::vector<OutlinePiece> outlinePieces(const Path2D& path);

/// The pieces with each one's start moved onto the end of the one before, round the loop.
std::vector<OutlinePiece> stitchedPieces(const std::vector<OutlinePiece>& pieces);

/// The pieces of `path` as drawn, without the closing line of a closed path: for open curves such
/// as a corner profile or a cap.
std::vector<OutlinePiece> drawnPieces(const Path2D& path);

/// The pieces with every control point sent through `map` (a line stays a line between the mapped ends).
std::vector<OutlinePiece> mappedPieces(const std::vector<OutlinePiece>& pieces, const std::function<Vector2(Vector2)>& map);

/// The pieces with the first one's start moved to `first` and the last one's end to `last`.
std::vector<OutlinePiece> pinnedPieces(std::vector<OutlinePiece> pieces, Vector2 first, Vector2 last);

/// A path through `pieces` in order, closed when `closed`. Each piece is taken to start where the
/// one before ends; the first start is the path's start.
Path2D pathOfPieces(const std::vector<OutlinePiece>& pieces, bool closed);

} // namespace editor
