// SVG path data as contours and back: the `d` of a <path> read into the engine's pieces (lines and cubics), and
// contours written as `d`, so a drawing made elsewhere (a logo, a glyph exported from an editor) can be measured,
// deconstructed into strokes, or shown.
//
// Use-cases: reading a wordmark or a glyph from an SVG file to measure it or find its centerlines (src/image/
// skeleton.ts); writing outlines into an SVG panel or file.
//
// Ported from fountain-ts/src/curves/svg.ts.
//
// Decisions:
// - Every command of SVG 1.1 is read, absolute and relative: M L H V C S Q T Z, and A as cubics (each arc split into
//   quarter turns at most, the usual 4/3·tan(θ/4) handles). Quadratics are raised exactly to cubics. A subpath that
//   isn't closed is closed with a line, as a fill closes it. y stays as SVG has it (down); flip with an affine map
//   for font units.
// - The TS throws Error on malformed data; here std::invalid_argument with the same message.
// - Numbers are written as JS prints them (shortest round-trip digits, JS's switch to exponent form), after
//   toFixed's rounding (halves away from zero on the exact value), so the strings equal the TS's.
// - Input is read byte by byte: whitespace is ASCII whitespace (JS's \s also takes Unicode spaces).
#pragma once

#include "curves/geometry.hpp"

#include <string>
#include <vector>

namespace curves {

/** The contours of an SVG path's `d`. */
std::vector<Contour> parseSvgPath(const std::string& d);

/** Contours as an SVG path's `d`, to `digits` decimals. */
std::string svgPath(const std::vector<Contour>& cs, int digits = 2);

}  // namespace curves
