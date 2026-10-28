// The shader library's coordinate transforms, ported from graphics_metal's Coordinates.metal:
// mirroring and flipping UV, kaleidoscope folding, twirl and spherize warps, brick-style row
// offsets, hexagonal and triangular grids, pixelating, and a 3×3 matrix transform of a 2D point.
// They reshape a texture coordinate before a pattern reads it. All are written once for the
// kernel vectors (KernelVectors.h) and run the same on the CPU and the GPU.
//
// Differences from graphics_metal, all on purpose:
// - `triplanar` was not ported: it samples a texture (texture2d, sampler), which shared kernels
//   cannot do; it stays a Metal-only function in graphics_metal.
// - `matrixTransform3` took a `float3x3`; no kernel matrices exist, so `transformedByMatrix3`
//   takes the matrix as its three columns.
// - `triangularGrid` calls the float-only `hash21` of Hashes.h, so it is written for `float`.
// - Names are spelt out (`kaleidoscope` → `kaleidoscopeFolded`); the graphics_metal name is in
//   each Tags line.
#pragma once

#include "Hashes.h"
#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

/// `point` with its sign dropped on one axis or both, so one half or one quarter of the plane is
/// repeated as a mirror image.
///
/// How: (mode == 1 ? x : |x|, mode == 0 ? y : |y|). Mode 0 mirrors across the vertical axis
/// (left copies right), mode 1 across the horizontal axis, any other mode across both.
///
/// Conventions: no range is assumed; the mirror axes pass through the origin.
///
/// Use when: making a pattern symmetric about the centre of the canvas
/// Tags: mirrorUv, mirror, symmetry, reflect, fold, abs
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> mirroredUv(Vector2Of<Real> point, int mode) {
    return Vector2Of<Real>(mode == 1 ? point.x : absolute(point.x), mode == 0 ? point.y : absolute(point.y));
}

/// `uv` turned around the middle of the unit square on one axis or both.
///
/// How: (mode == 1 ? u : 1 − u, mode == 0 ? v : 1 − v). Mode 0 flips left to right, mode 1 flips
/// top to bottom, any other mode flips both.
///
/// Conventions: `uv` runs 0 to 1 on each axis; values outside that range are flipped the same way.
///
/// Use when: mirroring a texture coordinate, as a flip control on an image or pattern
/// Tags: flipUv, flip, invert uv, one minus
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> flippedUv(Vector2Of<Real> uv, int mode) {
    return Vector2Of<Real>(mode == 1 ? uv.x : Real(1.0f) - uv.x, mode == 0 ? uv.y : Real(1.0f) - uv.y);
}

/// `point` folded into one of `segments` mirrored wedges around the origin, a kaleidoscope.
///
/// How: n = max(⌊segments + 0.5⌋, 1); sector = τ / n; a = atan2(y, x) − angle; a = a mod sector
/// (floored); a = |a − sector / 2|; the result is |point| (cos a, sin a). Every wedge shows the
/// same picture, each neighbour a mirror image of the next.
///
///        ╲ wedge ╱         the whole plane is folded into
///          ╲   ╱           the wedge 0 … sector / 2 above
///    ───────╳───────       the x axis rotated by `angle`
///
/// Conventions: radians; `segments` is rounded to a whole number, at least 1; the length of
/// `point` is kept, so a point at the origin stays there. The result's angle lies in
/// 0 … sector / 2.
///
/// Use when: kaleidoscope and snowflake patterns, radial symmetry
/// Tags: kaleidoscope, mirror wedges, radial symmetry, polar fold, mandala
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> kaleidoscopeFolded(Vector2Of<Real> point, Real segments, Real angle) {
    Real count = largerOf(roundedDown(segments + Real(0.5f)), Real(1.0f));
    Real sector = tau<Real>() / count;
    Real wedgeAngle = arcTangent(point.y, point.x) - angle;
    wedgeAngle = flooredModulo(wedgeAngle, sector);
    wedgeAngle = absolute(wedgeAngle - sector * Real(0.5f));
    return lengthOf(point) * Vector2Of<Real>(cosine(wedgeAngle), sine(wedgeAngle));
}

/// `point` twisted around the origin: turned most at the centre and not at all from `radius` out.
///
/// How: r = |point|; falloff = max(0, 1 − r / max(radius, 0.000001)); the point is rotated
/// counter-clockwise by strength × falloff² radians (rotatedCounterClockwise).
///
/// Conventions: `strength` in radians (the turn at the centre); a negative strength turns
/// clockwise; the length of `point` is kept, so a swirl never moves a point to another radius.
///
/// Use when: swirl and whirlpool distortions
/// Tags: twirl, swirl, twist, whirlpool, vortex, distort
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> twirled(Vector2Of<Real> point, Real strength, Real radius) {
    Real distanceFromOrigin = lengthOf(point);
    Real falloff = largerOf(Real(0.0f), Real(1.0f) - distanceFromOrigin / largerOf(radius, Real(0.000001f)));
    return rotatedCounterClockwise(point, strength * falloff * falloff);
}

/// `point` pushed outwards (bulge) or pulled inwards (pinch) inside the disc of `radius` around the origin.
///
/// How: r = |point| / max(radius, 0.000001); from r ≥ 1 the point is unchanged; otherwise
/// bent = r^(1 + strength) and the result is point × bent / max(r, 0.000001), which puts the point
/// at relative radius `bent`.
///
/// Conventions: a positive `strength` shrinks r (r < 1 raised to a power above 1), pulling
/// points toward the centre, which magnifies the picture sampled there (a bulge); a negative one
/// does the opposite (a pinch). Strength 0 changes nothing; the edge of the disc stays fixed.
///
/// Use when: lens, bulge and pinch distortions
/// Tags: spherize, bulge, pinch, lens, fisheye, distort
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> spherized(Vector2Of<Real> point, Real strength, Real radius) {
    Real relativeRadius = lengthOf(point) / largerOf(radius, Real(0.000001f));
    if (relativeRadius >= Real(1.0f)) {
        return point;
    }
    Real bent = power(relativeRadius, Real(1.0f) + strength);
    return point * (bent / largerOf(relativeRadius, Real(0.000001f)));
}

/// `uv` scaled into a grid of `count` cells, with every other row (or column) slid by `offset` cells: bricks.
///
/// How: p = uv × count; direction 0 gives p.x += offset × (⌊p.y⌋ mod 2); any other direction
/// gives p.y += offset × (⌊p.x⌋ mod 2). The result is in cell units: its integer part is the
/// cell, its fractional part the place inside it.
///
/// Conventions: `offset` in cells (0.5 is a running bond); the mod is the floored kind
/// (flooredModulo), so negative rows alternate the same way as positive ones.
///
/// Use when: brick walls, tiles laid in a stagger, woven rows
/// Tags: offsetRows, brick, stagger, running bond, row offset
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> rowOffsetGrid(Vector2Of<Real> uv, Vector2Of<Real> count, Real offset, int direction) {
    Vector2Of<Real> point = uv * count;
    if (direction == 0) {
        point.x += offset * flooredModulo(roundedDown(point.y), Real(2.0f));
    } else {
        point.y += offset * flooredModulo(roundedDown(point.x), Real(2.0f));
    }
    return point;
}

/// A hexagonal grid with pointed tops, as a position inside a cell and the cell's centre.
///
/// How: two interleaved rectangular grids with cells of size s = (1, √3) hold the hexagon
/// centres: the first at (⌊p / s⌋ + 0.5) s, the second, looked up from the point shifted by
/// (0.5, 1), at (⌊(p − (0.5, 1)) / s⌋ + 1) s. Each gives the offset of `point` from its
/// centre, and the nearer of the two centres wins (the standard two-grid hex tiling).
/// Centres form a triangular lattice with neighbours 1 apart.
///
///        ⬡ ⬡ ⬡            neighbouring centres are 1 apart, so a point is at most
///         ⬡ ⬡ ⬡           1 / √3 = 0.577 from the centre it belongs to
///
/// Conventions: returns (x, y) the position of `point` relative to its cell centre (the cell
/// radius is about 0.5 along x), and (z, w) the centre itself, which identifies the cell.
/// Points equidistant from two centres go to the first grid's.
///
/// Use when: honeycomb patterns, hexagonal tiles, per-cell randomness on a hex grid
/// Tags: hexGrid, hexagon grid, honeycomb, hex tiling, cell id
/// Node: none (shader library)
template <typename Real>
inline Vector4Of<Real> hexagonalGrid(Vector2Of<Real> point) {
    Vector2Of<Real> cellSize(Real(1.0f), Real(1.7320508f));
    Vector2Of<Real> firstCentre = roundedDown(point / cellSize) + Real(0.5f);
    Vector2Of<Real> secondCentre = roundedDown((point - Vector2Of<Real>(Real(0.5f), Real(1.0f))) / cellSize) + Real(0.5f);
    Vector2Of<Real> firstOffset = point - firstCentre * cellSize;
    Vector2Of<Real> secondOffset = point - (secondCentre + Real(0.5f)) * cellSize;
    if (dotProduct(firstOffset, firstOffset) < dotProduct(secondOffset, secondOffset)) {
        Vector2Of<Real> centre = firstCentre * cellSize;
        return Vector4Of<Real>(firstOffset.x, firstOffset.y, centre.x, centre.y);
    }
    Vector2Of<Real> centre = (secondCentre + Real(0.5f)) * cellSize;
    return Vector4Of<Real>(secondOffset.x, secondOffset.y, centre.x, centre.y);
}

/// How far a point inside a hexagonal cell is from the cell's edge: 0.5 at the centre, 0 at the edge.
///
/// How: q = |local|; 0.5 − max(q · n, q.x) with n = normalize(1, √3), the unit normal of the
/// slanted edges (30° from the x axis). The vertical edges are at x = 0.5, the slanted ones at q · n = 0.5.
///
/// Conventions: `local` is the first two components of `hexagonalGrid`; the result is negative
/// outside the cell.
///
/// Use when: outlines and bevels on a hex grid
/// Tags: hexEdgeDistance, hex edge, honeycomb border, cell edge distance
/// Node: none (shader library)
template <typename Real>
inline Real hexagonalEdgeDistance(Vector2Of<Real> local) {
    Vector2Of<Real> magnitude = absolute(local);
    Vector2Of<Real> slantedNormal = normalized(Vector2Of<Real>(Real(1.0f), Real(1.7320508f)));
    return Real(0.5f) - largerOf(dotProduct(magnitude, slantedNormal), magnitude.x);
}

/// An equilateral triangle grid: the position inside the triangle, the distance to its nearest
/// edge and a random number for the cell.
///
/// How: skew the plane so the triangles become halves of unit squares: skewed = (x − y / √3,
/// 2y / √3); cell = ⌊skewed⌋, local = fract(skewed). The square splits along its diagonal: when
/// local.x + local.y > 1 the triangle is the upper one and its barycentric coordinates are
/// (1 − local.x, 1 − local.y, local.x + local.y − 1), otherwise (local.x, local.y, 1 − local.x − local.y).
/// edge = the smallest of the three; id = hash21(2 × cell + (upper ? 1 : 0)).
///
/// Conventions: returns (x, y) the first two barycentric coordinates, z the barycentric
/// distance to the nearest edge (0 on an edge, 1/3 at the centre), w a hash from 0 to 1 that is
/// the same everywhere in one triangle. Triangles have side 2 / √3.
///
/// Use when: triangle tilings, low-poly looks, per-triangle colour
/// Tags: triangleGrid, triangular tiling, barycentric, delaunay look, cell id
/// Node: none (shader library)
inline Vector4Of<float> triangularGrid(Vector2Of<float> point) {
    const float root = 1.7320508f;
    Vector2Of<float> skewed(point.x - point.y / root, point.y * 2.0f / root);
    Vector2Of<float> cell = roundedDown(skewed);
    Vector2Of<float> local = fractionalPart(skewed);
    bool upper = local.x + local.y > 1.0f;
    Vector3Of<float> barycentric = upper ? Vector3Of<float>(1.0f - local.x, 1.0f - local.y, local.x + local.y - 1.0f)
                                         : Vector3Of<float>(local.x, local.y, 1.0f - local.x - local.y);
    float edge = smallerOf(barycentric.x, smallerOf(barycentric.y, barycentric.z));
    float identifier = hash21(cell * 2.0f + (upper ? 1.0f : 0.0f));
    return Vector4Of<float>(barycentric.x, barycentric.y, edge, identifier);
}

/// `uv` snapped to the centre of a coarse cell, so the picture shows in blocks.
///
/// How: count = max((cells × aspect, cells), (1, 1)); result = (⌊uv × count⌋ + 0.5) / count.
///
/// Conventions: `uv` runs 0 to 1; `cells` is the number of blocks along the vertical, `aspect`
/// (width / height) widens the count along the horizontal so blocks are square; fewer than one
/// block per axis becomes one.
///
/// Use when: mosaic and low-resolution looks
/// Tags: pixelateUv, pixelate, mosaic, blocks, low res, snap
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> pixelatedUv(Vector2Of<Real> uv, Real cells, Real aspect) {
    Vector2Of<Real> count = largerOf(Vector2Of<Real>(cells * aspect, cells), Vector2Of<Real>(Real(1.0f)));
    return (roundedDown(uv * count) + Real(0.5f)) / count;
}

/// A 2D point moved by a 3×3 matrix in homogeneous coordinates: translation, rotation, scale and perspective.
///
/// How: h = M · (x, y, 1) = column0 × x + column1 × y + column2; the result is (h.x, h.y) / h.z,
/// or (h.x, h.y) unchanged when |h.z| < 0.000001. The matrix is given by its columns, graphics_metal's
/// `float3x3` layout, so a translation by (tx, ty) has column2 = (tx, ty, 1).
///
/// Conventions: columns, not rows; no matrix type exists in kernels.
///
/// Use when: an arbitrary affine or projective transform of texture coordinates
/// Tags: matrixTransform3, matrix transform, affine, homography, 3x3 matrix
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> transformedByMatrix3(Vector2Of<Real> point, Vector3Of<Real> firstColumn,
    Vector3Of<Real> secondColumn, Vector3Of<Real> thirdColumn) {
    Vector3Of<Real> homogeneous = firstColumn * point.x + secondColumn * point.y + thirdColumn;
    Real divisor = absolute(homogeneous.z) < Real(0.000001f) ? Real(1.0f) : homogeneous.z;
    return Vector2Of<Real>(homogeneous.x / divisor, homogeneous.y / divisor);
}

} // namespace kernels
} // namespace editor
