// SVG path data as paths and back: the `d` of a <path> read into `Path2D`s (lines and cubics), and
// paths written as `d`, so a drawing made elsewhere (a logo, a glyph exported from an editor) can be
// measured, deconstructed into strokes, or shown.
//
// Every command of SVG 1.1 is read, absolute and relative: M L H V C S Q T Z, and A as cubics (each
// arc split into quarter turns at most, with the usual 4/3·tan(θ/4) handles). Quadratics are raised
// exactly to cubics. A subpath that is not closed is closed with a line, as a fill closes it, so
// every path read is closed (its closing line is the path's implicit one). y stays as SVG has it
// (down); flip with an affine map for y-up units.
//
// Ported from graphics_cpp's Curves2D svg.hpp/svg.cpp (fountain-ts/src/curves/svg.ts).
// Differences: float; a path read has no explicit closing piece; numbers are written fixed to
// `digits` decimals with trailing zeros dropped (JS number printing is not reproduced).
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "Path2D.h"

namespace editor {

/// Reading and writing SVG path data.
///
/// Use when: importing a logo or glyph as paths; exporting outlines to an SVG file
/// Tags: svg, path data, d attribute, import, export, parse svg path
/// Node: planned (svgPath)
struct PathSVG {
    /// The paths of an SVG path's `d`, one per subpath (each M starts one), all closed. A subpath
    /// without a piece is left out. Malformed data (an unknown command, a missing number or flag,
    /// a number before any command) throws a `MathProblem` naming the position.
    ///
    /// How: a one-pass reader of the grammar in SVG 1.1 §8.3; S and T reflect the last control point
    /// of a preceding C/S or Q/T about the current point; an arc is turned from its end-point to its
    /// centre form (SVG 1.1 implementation notes F.6.5), radii grown when too small to reach, and cut
    /// into quarter turns at most. A zero radius gives a line, equal end points no piece.
    ///
    /// Conventions: whitespace is ASCII whitespace; commas separate numbers.
    ///
    /// Use when: reading a wordmark or a glyph from an SVG file
    /// Tags: parse svg path, path data reader, d attribute, import svg
    /// Node: planned (svgPath)
    static std::vector<Path2D> parsed(std::string_view data);

    /// `paths` as one SVG path's `d`, numbers fixed to `digits` decimals (0…9) with trailing zeros
    /// dropped: "M0 0L10 0L10 10Z". Each path starts with M, lines are L, cubics are C, and a closed
    /// path ends with Z. A path with no piece is skipped. A non-finite coordinate or a `digits`
    /// outside 0…9 throws a `MathProblem`.
    ///
    /// Use when: exporting outlines to an SVG file or a panel
    /// Tags: svg path, write svg, d attribute, export path
    /// Node: planned (svgPath)
    static std::string text(const std::vector<Path2D>& paths, int digits = 2);
};

} // namespace editor
