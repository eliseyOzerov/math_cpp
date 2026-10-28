// The shader library's patterns, ported from graphics_metal's Patterns.metal: antialiased masks
// from 0 to 1 (checkerboard, stripes, grid, dots, bricks, hexagons, Truchet tiles, ripples,
// spirals, sunbursts, zigzags, herringbone, basket weave, moire, halftone), the Bayer dither
// threshold and a gradient ramp. A mask is antialiased from the size of a pixel in the units of
// the pattern's coordinates, so lines stay crisp at any scale; `softness` widens the edge on
// purpose.
//
// The pixel footprint is a parameter. graphics_metal reads it with `fwidth`, a derivative that
// exists only in a fragment shader and has no meaning on the CPU or in a compute kernel. Every
// function that used `fwidth` (or `patternPixel`) takes the footprint as its last parameter
// instead, named `pixelSize`. A fragment shader passes what graphics_metal measured, a compute
// kernel or the CPU any footprint it likes (0 gives the sharpest edge the 0.0001 floor allows).
// Given the same footprint the result is graphics_metal's, to the rounding of the arithmetic.
// Each function's comment says exactly what to pass. Where graphics_metal took `fwidth` of a value
// the function derives from `p` (the stripe coordinate, the zigzag phase, the halftone screen's
// coordinate, the herringbone band), a small public function returns that value, so the caller
// writes `fwidth(stripeCoordinate(p, count, angle))` and the footprint is exact.
//
// Name map from graphics_metal (each function's Tags line repeats its old name):
//   patternCoverage → antialiasedCoverage, patternChecker → checkerboardPattern,
//   patternStripes → stripesPattern, patternGrid → gridPattern, patternDots → dotsPattern,
//   patternBricks → bricksPattern, patternHex → hexagonTilesPattern,
//   patternTruchet → truchetPattern, patternRipples → ripplesPattern,
//   patternSpiral → spiralPattern, patternSunburst → sunburstPattern,
//   patternZigzag → zigzagPattern, patternHerringbone → herringbonePattern,
//   patternBasketweave → basketweavePattern, patternMoire → moirePattern,
//   patternHalftone → halftonePattern, bayerThreshold → bayerMatrixThreshold,
//   ditherLevels → ditheredLevels, patternGradient → gradientRamp.
//   New: stripeCoordinate, zigzagPhase, herringboneBand, halftoneCoordinate (see above).
//
// Stays in the Metal backend: `patternPixel(p)` (max(length(fwidth(p)), 0.0001)), because it is
// only `fwidth`. In a fragment shader pass `length(fwidth(p))` (or `patternPixel(p)`) as the
// footprint.
//
// Differences from graphics_metal, all on purpose: `mix`, `smoothstep`, `saturate`, `fract`,
// `floor` and the others are the prelude's names (`linearBlend`, `safeSmoothStep`, `clamped`,
// `fractionalPart`, `roundedDown`), `glslMod` is `flooredModulo`, the constants are
// `tau<float>()`, and `bayerMatrixThreshold` clamps a negative pixel position to 0 before
// converting it to an unsigned integer (that conversion is undefined for negatives on the CPU).
#pragma once

#include "Hashes.h"
#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderCoordinates.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

/// 1 inside a shape, 0 outside, from the signed distance inside it and the size of a pixel in the
/// same units: a smooth step across one pixel, so the edge is antialiased.
///
/// Source: the standard screen-space antialiasing of a signed distance field (for instance Inigo
/// Quilez's articles on distance functions), as graphics_metal has it.
///
/// How: width = max(pixelSize, 0.0001) + softness; the result is the smooth step of
/// distanceInside from −width/2 to +width/2, 3t² − 2t³ with t = clamp((d + width/2) / width, 0, 1).
///
/// Conventions: distanceInside is positive inside the shape; at distance 0 the result is 0.5;
/// `softness` ≥ 0 widens the edge (a negative one that makes the width ≤ 0 is the caller's mistake).
///
/// Use when: turning any distance into an antialiased mask; every pattern below ends with it
/// Tags: patternCoverage, antialiasing, coverage, smooth edge, SDF mask, smoothstep edge
/// Node: none (shader library)
inline float antialiasedCoverage(float distanceInside, float pixelSize, float softness) {
    float width = largerOf(pixelSize, 0.0001f) + softness;
    return safeSmoothStep(-0.5f * width, 0.5f * width, distanceInside);
}

/// A checkerboard of unit squares: 0 and 1 alternate, with antialiased edges.
///
/// How: with w = max(pixelSize + softness, 0.0001) per axis, the integral of a square wave
/// filtered by a box of width w gives i = 2 (|fract((p − w/2)/2) − 0.5| − |fract((p + w/2)/2) − 0.5|) / w
/// per axis (a triangle-wave difference); the mask is 0.5 − 0.5 i.x i.y. (Inigo Quilez, "Filtering
/// the checkerboard", as graphics_metal has it.)
///
/// Conventions: inside a square far from its edges i is +1 or −1 on each axis (+1 for ⌊p⌋ even, −1
/// for odd), so the mask is 0 where ⌊p.x⌋ + ⌊p.y⌋ is even and 1 where it is odd; the square
/// [0,1)×[0,1) is 0. `pixelSize` is `fwidth(p)` per axis (x and y separately), in the units of `p`.
///
/// Use when: a checker texture; a floor or transparency grid
/// Tags: patternChecker, checkerboard, chess, tiles
/// Node: none (shader library)
inline float checkerboardPattern(Vector2Of<float> p, float softness, Vector2Of<float> pixelSize) {
    Vector2Of<float> w = largerOf(pixelSize + softness, Vector2Of<float>(0.0001f));
    Vector2Of<float> low = absolute(fractionalPart((p - 0.5f * w) * 0.5f) - 0.5f);
    Vector2Of<float> high = absolute(fractionalPart((p + 0.5f * w) * 0.5f) - 0.5f);
    Vector2Of<float> i = 2.0f * (low - high) / w;
    return 0.5f - 0.5f * i.x * i.y;
}

/// The coordinate across the stripes of `stripesPattern`: p − 0.5 turned by `angle` (radians,
/// counter-clockwise), its x scaled by `count`, plus 0.5. Pass `fwidth` of it as the stripes'
/// footprint.
///
/// How: (rotate(p − 0.5, angle)).x × count + 0.5.
///
/// Use when: measuring the pixel footprint for `stripesPattern` in a fragment shader
/// Tags: stripe coordinate, rotated axis, stripes q
/// Node: none (shader library)
inline float stripeCoordinate(Vector2Of<float> p, float count, float angle) {
    return rotatedCounterClockwise(p - 0.5f, angle).x * count + 0.5f;
}

/// Stripes across `p` (0 to 1) turned by `angle`: x is the mask, y the stripe's number.
///
/// How: q = stripeCoordinate(p, count, angle); f = fract(q); the mask is antialiasedCoverage(width/2
/// − |f − 0.5|, pixelSize, softness), so a stripe is centred at f = 0.5 and `width` is a fraction
/// of the stripe period; y is ⌊q⌋.
///
/// Conventions: `pixelSize` is `fwidth(stripeCoordinate(p, count, angle))`, the footprint in
/// stripe periods; angle in radians.
///
/// Use when: stripes, bars, hatching
/// Tags: patternStripes, stripes, lines, bars, hatching
/// Node: none (shader library)
inline Vector2Of<float> stripesPattern(Vector2Of<float> p, float count, float angle, float width, float softness, float pixelSize) {
    float q = stripeCoordinate(p, count, angle);
    float f = fractionalPart(q);
    return Vector2Of<float>(antialiasedCoverage(width * 0.5f - absolute(f - 0.5f), pixelSize, softness), roundedDown(q));
}

/// Lines between the cells of a grid, `width` a fraction of a cell: x is the mask, y the cell's
/// random number.
///
/// How: q = p × count; the distance to the nearest line on each axis is 0.5 − |fract(q) − 0.5|;
/// each axis's mask is antialiasedCoverage(width/2 − thatDistance, footprint, softness) and the
/// result is the larger of the two. y is hash21(⌊q⌋).
///
/// Conventions: `pixelSize` is `fwidth(p)` per axis, in the units of `p` (the footprint in cells
/// is that times `count`, as `fwidth(p × count)` is).
///
/// Use when: grid lines, graph paper, tiles with a random value each
/// Tags: patternGrid, grid, graph paper, cell lines
/// Node: none (shader library)
inline Vector2Of<float> gridPattern(Vector2Of<float> p, Vector2Of<float> count, float width, float softness, Vector2Of<float> pixelSize) {
    Vector2Of<float> q = p * count;
    Vector2Of<float> f = fractionalPart(q);
    Vector2Of<float> toLine = 0.5f - absolute(f - 0.5f);
    Vector2Of<float> pixel = pixelSize * absolute(count);
    float x = antialiasedCoverage(width * 0.5f - toLine.x, pixel.x, softness);
    float y = antialiasedCoverage(width * 0.5f - toLine.y, pixel.y, softness);
    return Vector2Of<float>(largerOf(x, y), hash21(roundedDown(q)));
}

/// Round dots on a grid, optionally with every other row slid half a cell: x is the mask, y the
/// dot's random number.
///
/// How: q = p × scale (with `stagger`: q.x += 0.5 × (⌊q.y⌋ mod 2)); f = fract(q) − 0.5; the mask is
/// antialiasedCoverage(radius − |f|, footprint, softness); y is hash21(⌊q⌋).
///
/// Conventions: `radius` is in cells (0.5 touches the neighbours); `pixelSize` is
/// `length(fwidth(p))` in the units of `p`; the footprint in cells is max(pixelSize × |scale|, 0.0001).
///
/// Use when: polka dots, perforation, halftone-like texture
/// Tags: patternDots, polka dots, dot grid, stagger
/// Node: none (shader library)
inline Vector2Of<float> dotsPattern(Vector2Of<float> p, float scale, float radius, float softness, bool stagger, float pixelSize) {
    Vector2Of<float> q = p * scale;
    if (stagger) q.x += 0.5f * flooredModulo(roundedDown(q.y), 2.0f);
    Vector2Of<float> f = fractionalPart(q) - 0.5f;
    float cellPixel = largerOf(pixelSize * absolute(scale), 0.0001f);
    return Vector2Of<float>(antialiasedCoverage(radius - lengthOf(f), cellPixel, softness), hash21(roundedDown(q)));
}

/// Bricks in `count` columns and rows, rows slid by `offset`; `mortar` is a fraction of a brick's
/// height. x is the brick mask, y the brick's random number.
///
/// How: q = rows of p × count slid by `offset` (rowOffsetGrid); toEdge = min(fract(q), 1 −
/// fract(q)); halfMortar = mortar/2 × (count.x / max(count.y, 1), 1) so the mortar is as thick
/// along both axes on the brick; the mask is the smaller of the two axes' antialiasedCoverage(
/// toEdge − halfMortar, footprint, softness). y is hash21(⌊q⌋).
///
/// Conventions: `pixelSize` is `fwidth(p)` per axis (the footprint in bricks is that times
/// `count`; the row slide is constant within a row, so it does not change the footprint).
///
/// Use when: brick walls, tiles with grout
/// Tags: patternBricks, bricks, masonry, tiles, mortar
/// Node: none (shader library)
inline Vector2Of<float> bricksPattern(Vector2Of<float> p, Vector2Of<float> count, float offset, float mortar, float softness, Vector2Of<float> pixelSize) {
    Vector2Of<float> q = rowOffsetGrid(p, count, offset, 0);
    Vector2Of<float> f = fractionalPart(q);
    Vector2Of<float> toEdge = smallerOf(f, 1.0f - f);
    Vector2Of<float> halfMortar = 0.5f * mortar * Vector2Of<float>(count.x / largerOf(count.y, 1.0f), 1.0f);
    Vector2Of<float> pixel = pixelSize * absolute(count);
    float brick = smallerOf(antialiasedCoverage(toEdge.x - halfMortar.x, pixel.x, softness),
                            antialiasedCoverage(toEdge.y - halfMortar.y, pixel.y, softness));
    return Vector2Of<float>(brick, hash21(roundedDown(q)));
}

/// Hexagonal tiles: x the tile mask, y the distance to the tile's edge, z and w two random
/// numbers for the tile.
///
/// How: q = p × scale; cell = the hexagonal cell of q (hexagonalGrid); edge = the distance
/// to the cell's edge (0.5 at the centre); x = antialiasedCoverage(edge − gap/2, footprint,
/// softness); z = hash21(cell centre), w = hash21(cell centre + 41).
///
/// Conventions: `pixelSize` is `length(fwidth(p))`; the footprint in cells is max(pixelSize ×
/// |scale|, 0.0001).
///
/// Use when: hexagonal tiles, honeycombs, sci-fi panels
/// Tags: patternHex, hexagon tiles, honeycomb, hex tiles
/// Node: none (shader library)
inline Vector4Of<float> hexagonTilesPattern(Vector2Of<float> p, float scale, float gap, float softness, float pixelSize) {
    Vector2Of<float> q = p * scale;
    Vector4Of<float> cell = hexagonalGrid(q);
    float edge = hexagonalEdgeDistance(Vector2Of<float>(cell.x, cell.y));
    Vector2Of<float> centre(cell.z, cell.w);
    float cellPixel = largerOf(pixelSize * absolute(scale), 0.0001f);
    return Vector4Of<float>(antialiasedCoverage(edge - 0.5f * gap, cellPixel, softness), edge, hash21(centre),
                            hash21(centre + 41.0f));
}

/// Truchet tiles: random quarter-circle arcs (variant 0) or diagonals (variant 1) that join up
/// across cells.
///
/// Source: Sébastien Truchet, 1704 (as used in procedural textures).
///
/// How: q = p × scale; f = fract(q); a cell whose hash21(⌊q⌋) > 0.5 is mirrored (f.x = 1 − f.x).
/// Variant 1: d = |f.x − f.y| × 0.70710678 (distance to the diagonal); otherwise d = the smaller of
/// ||f| − 0.5| and ||f − (1,1)| − 0.5| (distance to the two quarter circles). The mask is
/// antialiasedCoverage(width/2 − d, footprint, softness).
///
/// Conventions: `width` is in cells; `pixelSize` is `length(fwidth(p))`; the footprint in cells is
/// max(pixelSize × |scale|, 0.0001).
///
/// Use when: maze-like and woven line patterns
/// Tags: patternTruchet, truchet tiles, quarter circles, maze, random tiles
/// Node: none (shader library)
inline float truchetPattern(Vector2Of<float> p, float scale, float width, float softness, int variant, float pixelSize) {
    Vector2Of<float> q = p * scale;
    Vector2Of<float> f = fractionalPart(q);
    if (hash21(roundedDown(q)) > 0.5f) f.x = 1.0f - f.x;
    float d = variant == 1 ? absolute(f.x - f.y) * 0.70710678f
                           : smallerOf(absolute(lengthOf(f) - 0.5f), absolute(lengthOf(f - 1.0f) - 0.5f));
    float cellPixel = largerOf(pixelSize * absolute(scale), 0.0001f);
    return antialiasedCoverage(width * 0.5f - d, cellPixel, softness);
}

/// Rings out from the origin in a wave shape (0 sine, 1 triangle, 2 square, otherwise sawtooth),
/// fading with distance. No antialiasing: it needs no pixel footprint.
///
/// How: r = |p|; 0.5 + 0.5 × waveShape(r × frequency − time, shape) × e^(−r × decay).
///
/// Conventions: the origin gives 0.5 + 0.5 × waveShape(−time); `decay` ≥ 0 fades outwards; for
/// decay ≥ 0 the result is in 0…1.
///
/// Use when: water ripples, radar pings, sound waves
/// Tags: patternRipples, ripples, concentric rings, waves, radial
/// Node: none (shader library)
inline float ripplesPattern(Vector2Of<float> p, float frequency, float time, float decay, int shape) {
    float r = lengthOf(p);
    return 0.5f + 0.5f * waveShape(r * frequency - time, shape) * exponential(-r * decay);
}

/// Spiral arms around the origin, straight (Archimedean) or logarithmic.
///
/// How: r = max(|p|, 0.0001); angle = atan2(p.y, p.x); radial = r × frequency (logarithmic: ln r ×
/// frequency); phase = radial − arms × angle / τ − spin; the mask is antialiasedCoverage(width/2 −
/// |fract(phase) − 0.5|, footprint, softness). The footprint is the pixel size times the rate of
/// change of the phase per unit length: pixel × √(rate² + (arms/(τ r))²), the radial rate being
/// `frequency` (logarithmic: frequency / r).
///
/// Conventions: `logarithmic` is 1 for the logarithmic spiral, any other value for the straight
/// one; `pixelSize` is `length(fwidth(p))`, floored at 0.0001 as patternPixel does.
///
/// Use when: spiral galaxies, swirls, hypnotic backgrounds
/// Tags: patternSpiral, spiral, Archimedean spiral, logarithmic spiral, swirl
/// Node: none (shader library)
inline float spiralPattern(Vector2Of<float> p, float arms, float frequency, float spin, float width, float softness, int logarithmic, float pixelSize) {
    float r = largerOf(lengthOf(p), 0.0001f);
    float angle = arcTangent(p.y, p.x);
    float radial = logarithmic == 1 ? naturalLogarithm(r) * frequency : r * frequency;
    float phase = radial - arms * angle / tau<float>() - spin;
    float rate = logarithmic == 1 ? frequency / r : frequency;
    float around = arms / (tau<float>() * r);
    float pixel = largerOf(pixelSize, 0.0001f) * squareRoot(rate * rate + around * around);
    return antialiasedCoverage(width * 0.5f - absolute(fractionalPart(phase) - 0.5f), pixel, softness);
}

/// Rays out from the origin, like a sunburst or a pie of equal wedges and gaps.
///
/// How: r = max(|p|, 0.0001); phase = rays × (atan2(p.y, p.x) + rotation) / τ; the footprint is
/// min(pixel × rays / (τ r), 0.5) (the angle per pixel, capped near the centre); the mask is
/// antialiasedCoverage(width/2 − |fract(phase) − 0.5|, footprint, softness).
///
/// Conventions: `rotation` in radians; `pixelSize` is `length(fwidth(p))`, floored at 0.0001.
///
/// Use when: sunbursts, speed lines, radial stripes
/// Tags: patternSunburst, sunburst, rays, radial stripes, pie wedges
/// Node: none (shader library)
inline float sunburstPattern(Vector2Of<float> p, float rays, float rotation, float width, float softness, float pixelSize) {
    float r = largerOf(lengthOf(p), 0.0001f);
    float phase = rays * (arcTangent(p.y, p.x) + rotation) / tau<float>();
    float pixel = smallerOf(largerOf(pixelSize, 0.0001f) * rays / (tau<float>() * r), 0.5f);
    return antialiasedCoverage(width * 0.5f - absolute(fractionalPart(phase) - 0.5f), pixel, softness);
}

/// The phase of `zigzagPattern`: p.y × rows plus the bend. Pass `fwidth` of it as the zigzag's
/// footprint.
///
/// How: waveShape(p.x × frequency, shape) × amplitude / 2 + p.y × rows, the wave being a sine for
/// `smoothing` 1 and a triangle otherwise.
///
/// Use when: measuring the pixel footprint for `zigzagPattern` in a fragment shader
/// Tags: zigzag phase, wavy stripes coordinate
/// Node: none (shader library)
inline float zigzagPhase(Vector2Of<float> p, float frequency, float rows, float amplitude, int smoothing) {
    float bend = waveShape(p.x * frequency, smoothing == 1 ? 0 : 1) * amplitude * 0.5f;
    return p.y * rows + bend;
}

/// Stripes bent into a zigzag (`smoothing` 0) or a wave (1).
///
/// How: phase = zigzagPhase(…); the mask is antialiasedCoverage(width/2 − |fract(phase) − 0.5|,
/// pixelSize, softness).
///
/// Conventions: `pixelSize` is `fwidth(zigzagPhase(p, frequency, rows, amplitude, smoothing))`.
///
/// Use when: chevron and wave stripes, fabric and water patterns
/// Tags: patternZigzag, zigzag, chevron, wavy stripes, sine stripes
/// Node: none (shader library)
inline float zigzagPattern(Vector2Of<float> p, float frequency, float rows, float amplitude, float width, float softness, int smoothing, float pixelSize) {
    float phase = zigzagPhase(p, frequency, rows, amplitude, smoothing);
    return antialiasedCoverage(width * 0.5f - absolute(fractionalPart(phase) - 0.5f), pixelSize, softness);
}

/// The band coordinate of `herringbonePattern`: p.y × rows plus a lean that alternates between
/// columns. Pass `fwidth` of it as the second component of the herringbone's footprint.
///
/// How: lean = +1 in even columns (⌊p.x × columns⌋ mod 2 < 1), −1 in odd ones; p.y × rows + lean ×
/// (fract(p.x × columns) − 0.5) × slant.
///
/// Use when: measuring the pixel footprint for `herringbonePattern` in a fragment shader
/// Tags: herringbone band, slanted rows coordinate
/// Node: none (shader library)
inline float herringboneBand(Vector2Of<float> p, float columns, float rows, float slant) {
    float x = p.x * columns;
    float f = fractionalPart(x);
    float lean = flooredModulo(roundedDown(x), 2.0f) < 1.0f ? 1.0f : -1.0f;
    return p.y * rows + lean * (f - 0.5f) * slant;
}

/// Bricks in columns whose rows slant one way then the other, which reads as herringbone: x is the
/// brick mask, y the brick's random number.
///
/// How: x = p.x × columns, f = fract(x); band = herringboneBand(…), fy = fract(band); the mask is
/// the smaller of antialiasedCoverage(min(f, 1 − f) − mortar/2, pixelSize.x, softness) and
/// antialiasedCoverage(min(fy, 1 − fy) − mortar/2, pixelSize.y, softness); y is hash21(⌊x⌋, ⌊band⌋).
///
/// Conventions: `pixelSize` is (`fwidth(p.x × columns)`, `fwidth(herringboneBand(p, columns, rows,
/// slant))`).
///
/// Use when: herringbone floors, slanted brick bonds
/// Tags: patternHerringbone, herringbone, parquet, zigzag bricks
/// Node: none (shader library)
inline Vector2Of<float> herringbonePattern(Vector2Of<float> p, float columns, float rows, float slant, float mortar, float softness, Vector2Of<float> pixelSize) {
    float x = p.x * columns;
    float column = roundedDown(x);
    float f = fractionalPart(x);
    float band = herringboneBand(p, columns, rows, slant);
    float fy = fractionalPart(band);
    float brick = smallerOf(antialiasedCoverage(smallerOf(f, 1.0f - f) - 0.5f * mortar, pixelSize.x, softness),
                            antialiasedCoverage(smallerOf(fy, 1.0f - fy) - 0.5f * mortar, pixelSize.y, softness));
    return Vector2Of<float>(brick, hash21(Vector2Of<float>(column, roundedDown(band))));
}

/// Woven bars, alternating direction by cell: x is the bar mask, y a shading that rounds each bar.
///
/// How: q = p × scale; cells with (⌊q.x⌋ + ⌊q.y⌋) even run their bars along x, the others along y;
/// `along` is the position along the bar, `across` × bars the position across it (fb = its
/// fract). side = antialiasedCoverage(min(fb, 1 − fb) − gap/2, footprint × bars, softness); end =
/// antialiasedCoverage(min(along, 1 − along) − gap/2, footprint, softness); mask = side × end;
/// shade = 1 − 0.5 (2 fb − 1)²; the result is (mask, mask × shade).
///
/// Conventions: `pixelSize` is `length(fwidth(p))`; the footprint in cells is max(pixelSize ×
/// |scale|, 0.0001).
///
/// Use when: wicker, cloth and basket textures
/// Tags: patternBasketweave, basket weave, woven, wicker, plaid weave
/// Node: none (shader library)
inline Vector2Of<float> basketweavePattern(Vector2Of<float> p, float scale, float bars, float gap, float softness, float pixelSize) {
    Vector2Of<float> q = p * scale;
    Vector2Of<float> cell = roundedDown(q);
    Vector2Of<float> f = fractionalPart(q);
    bool horizontal = flooredModulo(cell.x + cell.y, 2.0f) < 1.0f;
    float along = horizontal ? f.x : f.y;
    float across = (horizontal ? f.y : f.x) * bars;
    float fb = fractionalPart(across);
    float cellPixel = largerOf(pixelSize * absolute(scale), 0.0001f);
    float side = antialiasedCoverage(smallerOf(fb, 1.0f - fb) - 0.5f * gap, cellPixel * bars, softness);
    float end = antialiasedCoverage(smallerOf(along, 1.0f - along) - 0.5f * gap, cellPixel, softness);
    float shadeBase = 2.0f * fb - 1.0f;
    float shade = 1.0f - 0.5f * shadeBase * shadeBase;
    float mask = side * end;
    return Vector2Of<float>(mask, mask * shade);
}

/// Two gratings multiplied, which makes moire fringes: lines (`rings` 0) turned apart by `angle`,
/// or rings (`rings` 1) around two centres `spread` apart. No antialiasing.
///
/// How: lines: a = p.x × frequency, b = rotate(p, angle).x × frequency; rings: a and b the
/// distances to (±spread/2, 0) times frequency. Each grating is the smooth step from 0.3 to 0.7 of
/// 0.5 + 0.5 sin(τ a); the result is their product.
///
/// Conventions: `angle` in radians; the result is in 0…1.
///
/// Use when: moire, interference and optical-illusion textures
/// Tags: patternMoire, moire, interference, gratings, fringes
/// Node: none (shader library)
inline float moirePattern(Vector2Of<float> p, float frequency, float angle, float spread, int rings) {
    float a;
    float b;
    if (rings == 1) {
        a = lengthOf(p - Vector2Of<float>(spread * 0.5f, 0.0f)) * frequency;
        b = lengthOf(p + Vector2Of<float>(spread * 0.5f, 0.0f)) * frequency;
    } else {
        a = p.x * frequency;
        b = rotatedCounterClockwise(p, angle).x * frequency;
    }
    float firstGrating = safeSmoothStep(0.3f, 0.7f, 0.5f + 0.5f * sine(tau<float>() * a));
    float secondGrating = safeSmoothStep(0.3f, 0.7f, 0.5f + 0.5f * sine(tau<float>() * b));
    return firstGrating * secondGrating;
}

/// The coordinate of the halftone screen: (p − 0.5) turned by `angle` and scaled by `scale`. Pass
/// `length(fwidth(...))` of it as the halftone's footprint.
///
/// How: rotate(p − 0.5, angle) × scale.
///
/// Use when: measuring the pixel footprint for `halftonePattern` in a fragment shader
/// Tags: halftone coordinate, rotated screen, halftone q
/// Node: none (shader library)
inline Vector2Of<float> halftoneCoordinate(Vector2Of<float> p, float scale, float angle) {
    return rotatedCounterClockwise(p - 0.5f, angle) * scale;
}

/// Halftone dots (shape 0), squares (1) or lines (2) on a screen turned by `angle`, covering
/// `value` of each cell.
///
/// How: q = halftoneCoordinate(p, scale, angle); f = fract(q) − 0.5; v = clamp(value, 0, 1). Dots:
/// inside = (√2/2) √v − |f| (the dot that fills the cell's area fraction v at v = 1 reaches the
/// corners); squares: 0.5 √v − max(|f.x|, |f.y|); lines: 0.5 v − |f.y|. The mask is
/// antialiasedCoverage(inside, footprint, 0).
///
/// Conventions: `pixelSize` is `length(fwidth(halftoneCoordinate(p, scale, angle)))`, floored at
/// 0.0001, in cells of the screen.
///
/// Use when: comic and print halftones; turning a grey value into dots
/// Tags: patternHalftone, halftone, dot screen, comic dots, print screen
/// Node: none (shader library)
inline float halftonePattern(Vector2Of<float> p, float value, float scale, float angle, int shape, float pixelSize) {
    Vector2Of<float> q = halftoneCoordinate(p, scale, angle);
    Vector2Of<float> f = fractionalPart(q) - 0.5f;
    float v = clamped(value, 0.0f, 1.0f);
    float inside;
    if (shape == 1) inside = 0.5f * squareRoot(v) - largerOf(absolute(f.x), absolute(f.y));
    else if (shape == 2) inside = 0.5f * v - absolute(f.y);
    else inside = 0.70710678f * squareRoot(v) - lengthOf(f);
    return antialiasedCoverage(inside, largerOf(pixelSize, 0.0001f), 0.0f);
}

/// The Bayer (ordered dither) matrix's threshold at a pixel, for a matrix of 2^bits × 2^bits
/// cells: between 0 and 1, never exactly 0 or 1.
///
/// Source: Bryce Bayer, "An optimum method for two-level rendition of continuous-tone pictures",
/// 1973; the recursive bit-interleaved construction.
///
/// How: pixel = ⌊position / max(cellSize, 1)⌋ as unsigned integers; for each bit i from the lowest,
/// x and y are bit i of the pixel's coordinates and index = index × 4 + (2x + 3y − 4xy), which is
/// the Bayer digit for the bit pair: (0,0)→0, (1,0)→2, (0,1)→3, (1,1)→1. The result is
/// (index + 0.5) / 4^bits. For one bit the 2×2 thresholds are 0.125, 0.625 / 0.875, 0.375.
///
/// Conventions: `position` in pixels, a negative coordinate is taken as 0; `bits` ≥ 0 (0 gives
/// 0.5); `cellSize` ≥ 1 makes each matrix cell several pixels wide.
///
/// Use when: ordered dithering, retro and banding-free gradients
/// Tags: bayerThreshold, Bayer matrix, ordered dither, dither pattern, threshold map
/// Node: none (shader library)
inline float bayerMatrixThreshold(Vector2Of<float> position, float cellSize, int bits) {
    Vector2Of<float> cellPosition = largerOf(roundedDown(position / largerOf(cellSize, 1.0f)), Vector2Of<float>(0.0f));
    unsigned int pixelX = static_cast<unsigned int>(cellPosition.x);
    unsigned int pixelY = static_cast<unsigned int>(cellPosition.y);
    unsigned int index = 0u;
    for (int bit = 0; bit < bits; bit++) {
        unsigned int x = (pixelX >> bit) & 1u;
        unsigned int y = (pixelY >> bit) & 1u;
        index = index * 4u + (2u * x + 3u * y - 4u * x * y);
    }
    return (static_cast<float>(index) + 0.5f) / static_cast<float>(1u << (2 * bits));
}

/// A value reduced to `levels` steps, the rounding decided by a threshold.
///
/// How: steps = max(levels − 1, 1); clamp(⌊clamp(value, 0, 1) × steps + threshold⌋ / steps, 0, 1).
///
/// Conventions: a threshold of 0.5 rounds to the nearest level; thresholds from
/// `bayerMatrixThreshold` dither. The result is one of 0, 1/steps, …, 1.
///
/// Use when: ordered dithering, posterising to a palette of few levels
/// Tags: ditherLevels, dither, posterize, quantize, color levels
/// Node: none (shader library)
inline float ditheredLevels(float value, float levels, float threshold) {
    float steps = largerOf(levels - 1.0f, 1.0f);
    return clamped(roundedDown(clamped(value, 0.0f, 1.0f) * steps + threshold) / steps, 0.0f, 1.0f);
}

/// A 0-to-1 ramp of a shape: linear (0), radial (1), angular or conic (2), diamond (3) or square
/// (4). `size` is how much of the canvas the ramp spans.
///
/// How: d = rotate(p − centre, angle); reach = max(size/2, 0.0001); linear 0.5 + d.x / max(size,
/// 0.0001); radial |d| / reach; angular fract(atan2(d.y, d.x) / τ + 1); diamond (|d.x| + |d.y|) /
/// reach; square max(|d.x|, |d.y|) / reach; clamped to 0…1.
///
/// Conventions: `angle` in radians; the angular ramp starts at the +x axis of the turned frame and
/// grows counter-clockwise; any other `shape` than 1 to 4 is linear.
///
/// Use when: gradients, vignettes, masks, conic sweeps
/// Tags: patternGradient, gradient, linear gradient, radial gradient, conic gradient, ramp
/// Node: none (shader library)
inline float gradientRamp(Vector2Of<float> p, Vector2Of<float> centre, float angle, float size, int shape) {
    Vector2Of<float> d = rotatedCounterClockwise(p - centre, angle);
    float reach = largerOf(size * 0.5f, 0.0001f);
    float t;
    if (shape == 1) t = lengthOf(d) / reach;
    else if (shape == 2) t = fractionalPart(arcTangent(d.y, d.x) / tau<float>() + 1.0f);
    else if (shape == 3) t = (absolute(d.x) + absolute(d.y)) / reach;
    else if (shape == 4) t = largerOf(absolute(d.x), absolute(d.y)) / reach;
    else t = 0.5f + d.x / largerOf(size, 0.0001f);
    return clamped(t, 0.0f, 1.0f);
}

} // namespace kernels
} // namespace editor
