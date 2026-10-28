// The shader library's signed distance fields, ported from graphics_metal's SDF.metal: masks from
// a distance (fill, stroke, glow, shadow), smooth boolean operations, repetition, and the
// distances of three-dimensional shapes with the transforms that place them. A distance is
// negative inside a shape, zero on its surface and positive outside; Inigo Quilez's distance
// function catalogue (iquilezles.org/articles/distfunctions) is the source of the shapes.
//
// Differences from graphics_metal, all on purpose:
// - Metal's `smoothstep(-edge, edge, x)` became `safeSmoothStep` (ShaderMath.h): the same curve,
//   but a zero-width edge gives a hard step instead of an undefined result.
// - Metal's `sign` is the prelude's `signOf`.
// - Names are spelt out (`sdf3Box` → `boxDistance`); the graphics_metal name is in each Tags line.
// - No function of SDF.metal calls Noise or samples a texture, so all were ported.
#pragma once

#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

// ---------------------------------------------------------------------------------------------
// Masks from a distance

/// A mask that is 1 inside the shape and 0 outside, with a soft edge about a pixel wide.
///
/// How: edge = max(feather, 1) × pixelHeight; mask = 1 − smoothstep(−edge, edge, distanceValue),
/// where smoothstep(a, b, x) = t² (3 − 2t), t = clamp((x − a) / (b − a), 0, 1). On the boundary
/// (distance 0) the mask is 0.5; one `edge` inside it is 1.
///
/// Conventions: `distanceValue` negative inside; `pixelHeight` is the height of one pixel in the
/// same units as the distance (1 / canvas height when the canvas is one unit high), so the edge
/// stays about a pixel wide whatever the canvas size; `feather` in pixels, at least 1. A zero
/// edge gives a hard step (safeSmoothStep) where Metal's smoothstep is undefined.
///
/// Use when: turning a shape's distance into an antialiased mask
/// Tags: sdfFill, fill, antialiased edge, mask from distance, smoothstep edge
/// Node: none (shader library)
template <typename Real>
inline Real distanceFillMask(Real distanceValue, Real pixelHeight, Real feather) {
    Real edge = largerOf(feather, Real(1.0f)) * pixelHeight;
    return Real(1.0f) - safeSmoothStep(-edge, edge, distanceValue);
}

/// A mask of the band `width` wide along a shape's boundary: centred on it, inside it or outside it.
///
/// How: halfWidth = max(width, 0) / 2. The distance is shifted so the band sits where the
/// alignment says: centred (0) none, inside (1) + halfWidth, outside (2) − halfWidth; then the
/// mask is distanceFillMask(|shifted| − halfWidth, …), the fill of a band around zero.
///
///     inside ◄── alignment 1 ──┤boundary├── alignment 2 ──► outside
///                           centred (0): half each side
///
/// Conventions: `width` in the distance's units; `alignment` 0, 1 or 2 (any other number acts as
/// centred); `pixelHeight` and `feather` as in `distanceFillMask`.
///
/// Use when: outlines and borders of any shape
/// Tags: sdfStroke, stroke, outline, border, ring mask
/// Node: none (shader library)
template <typename Real>
inline Real distanceStrokeMask(Real distanceValue, Real width, int alignment, Real pixelHeight, Real feather) {
    Real halfWidth = largerOf(width, Real(0.0f)) * Real(0.5f);
    Real shifted = alignment == 1 ? distanceValue + halfWidth : (alignment == 2 ? distanceValue - halfWidth : distanceValue);
    return distanceFillMask(absolute(shifted) - halfWidth, pixelHeight, feather);
}

/// Light that is full strength inside a shape and fades away outside it.
///
/// How: intensity × e^(−max(distance, 0) / max(radius, 0.0001)). One `radius` outside the
/// boundary the light is intensity / e (0.368 of it).
///
/// Conventions: `radius` in the distance's units, at least 0.0001.
///
/// Use when: a glow or halo around a shape
/// Tags: sdfGlow, glow, halo, bloom, exponential falloff
/// Node: none (shader library)
template <typename Real>
inline Real distanceGlow(Real distanceValue, Real radius, Real intensity) {
    return intensity * exponential(-largerOf(distanceValue, Real(0.0f)) / largerOf(radius, Real(0.0001f)));
}

/// The strength of a soft shadow of a shape, moved by `offset`.
///
/// How: direction = gradient / max(|gradient|, 0.000001); moved = distance − direction · offset;
/// soft = max(blur, pixelHeight); shadow = opacity × (1 − smoothstep(−soft, soft, moved)). The
/// distance of the moved shape is approximated by the distance minus the offset's part along the
/// field's slope: exact for a straight edge, close elsewhere.
///
/// Conventions: `gradient` is the slope of the distance on screen (its derivative, such as from
/// `dfdx`/`dfdy`), pointing away from the shape; a zero gradient gives a zero direction, so the
/// shadow is not moved. `blur` and `offset` in the distance's units.
///
/// Use when: drop shadows under shapes and text
/// Tags: sdfShadow, drop shadow, soft shadow, offset shadow, blur
/// Node: none (shader library)
template <typename Real>
inline Real distanceShadow(Real distanceValue, Vector2Of<Real> gradient, Vector2Of<Real> offset, Real blur, Real opacity,
    Real pixelHeight) {
    Vector2Of<Real> direction = gradient / largerOf(lengthOf(gradient), Real(0.000001f));
    Real moved = distanceValue - dotProduct(direction, offset);
    Real soft = largerOf(blur, pixelHeight);
    return opacity * (Real(1.0f) - safeSmoothStep(-soft, soft, moved));
}

// ---------------------------------------------------------------------------------------------
// Smooth boolean operations

/// The union of two shapes with a rounded crease where they meet: the polynomial smooth minimum
/// of their distances.
///
/// How: Inigo Quilez, "smooth minimum" (iquilezles.org/articles/smin): r = max(k, 0.000001);
/// h = clamp(0.5 + 0.5 (b − a) / r, 0, 1); result = mix(b, a, h) − r h (1 − h). a = b = 1 and
/// r = 1: h = 0.5, 1 − 0.25 = 0.75. The result is never above min(a, b).
///
/// Conventions: `a`, `b` signed distances; `smoothness` in the same units, the width of the
/// blend; as it goes to 0 the result is the plain minimum. The result is not an exact distance
/// within the blend.
///
/// Use when: blobby unions, metaballs, fillets between shapes
/// Tags: sdfSmoothUnion, smooth union, smin, blend shapes, fillet, opSmoothUnion
/// Node: none (shader library)
template <typename Real>
inline Real smoothUnion(Real a, Real b, Real smoothness) {
    Real radius = largerOf(smoothness, Real(0.000001f));
    Real blend = clamped(Real(0.5f) + Real(0.5f) * (b - a) / radius, Real(0.0f), Real(1.0f));
    return linearBlend(b, a, blend) - radius * blend * (Real(1.0f) - blend);
}

/// Shape `a` with shape `b` carved out of it, the cut edge rounded.
///
/// How: Inigo Quilez's opSmoothSubtraction: r = max(k, 0.000001); h = clamp(0.5 − 0.5 (a + b) / r,
/// 0, 1); result = mix(a, −b, h) + r h (1 − h). a = b = 0 and r = 1: h = 0.5, 0.25. The result is
/// never below max(a, −b).
///
/// Conventions: `a` the shape that is kept, `b` the cutter; as `smoothness` goes to 0 the result
/// is max(a, −b).
///
/// Use when: holes and notches with soft edges
/// Tags: sdfSmoothSubtraction, smooth subtraction, carve, cut out, opSmoothSubtraction
/// Node: none (shader library)
template <typename Real>
inline Real smoothSubtraction(Real a, Real b, Real smoothness) {
    Real radius = largerOf(smoothness, Real(0.000001f));
    Real blend = clamped(Real(0.5f) - Real(0.5f) * (a + b) / radius, Real(0.0f), Real(1.0f));
    return linearBlend(a, -b, blend) + radius * blend * (Real(1.0f) - blend);
}

/// The overlap of two shapes with the crease rounded.
///
/// How: Inigo Quilez's opSmoothIntersection: r = max(k, 0.000001); h = clamp(0.5 − 0.5 (b − a) / r,
/// 0, 1); result = mix(b, a, h) + r h (1 − h). a = b = 0 and r = 1: 0.25. The result is never
/// below max(a, b).
///
/// Conventions: as `smoothUnion`; as `smoothness` goes to 0 the result is max(a, b).
///
/// Use when: soft intersections; rounding the inside corner of two shapes
/// Tags: sdfSmoothIntersection, smooth intersection, smax, opSmoothIntersection
/// Node: none (shader library)
template <typename Real>
inline Real smoothIntersection(Real a, Real b, Real smoothness) {
    Real radius = largerOf(smoothness, Real(0.000001f));
    Real blend = clamped(Real(0.5f) - Real(0.5f) * (b - a) / radius, Real(0.0f), Real(1.0f));
    return linearBlend(b, a, blend) + radius * blend * (Real(1.0f) - blend);
}

// ---------------------------------------------------------------------------------------------
// Repetition

/// A 2D point folded into the cell of a grid `spacing` apart that it lies in, with the cell's index.
///
/// How: s = max(|spacing|, 0.000001); cell = ⌊point / s + 0.5⌋ (cells are centred on multiples of
/// s); when limit ≥ 0.5 the cell is clamped to ±⌊limit + 0.5⌋; result = (point − s × cell, cell).
/// (2.3, −0.4) with spacing 1: cell (2, 0), folded point (0.3, −0.4).
///
/// Conventions: returns (x, y) the folded point in −s/2 … s/2 (beyond that only past `limit`),
/// (z, w) the integer cell index; `limit` 0 means no limit, otherwise the number of repeats
/// either side of the middle (a finite row or grid).
///
/// Use when: tiling a shape across the plane; finite arrays of a shape
/// Tags: sdfRepeat, repeat, tile, domain repetition, array, grid cell
/// Node: none (shader library)
template <typename Real>
inline Vector4Of<Real> repeatedCell2(Vector2Of<Real> point, Real spacing, Real limit) {
    Real cellSize = largerOf(absolute(spacing), Real(0.000001f));
    Vector2Of<Real> cell = roundedDown(point / cellSize + Real(0.5f));
    if (limit >= Real(0.5f)) {
        Real largestCell = roundedDown(limit + Real(0.5f));
        cell = clamped(cell, -largestCell, largestCell);
    }
    Vector2Of<Real> folded = point - cellSize * cell;
    return Vector4Of<Real>(folded.x, folded.y, cell.x, cell.y);
}

// ---------------------------------------------------------------------------------------------
// Three-dimensional shapes: the signed distance of a point, negative inside.

/// The distance from `point` to a sphere of `radius` centred on the origin.
///
/// How: |point| − radius (Inigo Quilez, distance functions: sdSphere). A sphere of radius 1 and
/// a point (2, 0, 0): 2 − 1 = 1.
///
/// Conventions: exact; −radius at the centre.
///
/// Use when: balls, planets, round blobs; the basis of metaball unions
/// Tags: sdf3Sphere, sphere, ball, round, sdSphere
/// Node: none (shader library)
template <typename Real>
inline Real sphereDistance(Vector3Of<Real> point, Real radius) {
    return lengthOf(point) - radius;
}

/// The distance from `point` to a box centred on the origin, with optionally rounded edges.
///
/// How: Inigo Quilez, distance functions: sdRoundBox. r = clamp(rounding, 0, min of the half
/// sizes); q = |point| − (halfSize − r); result = |max(q, 0)| + min(max(q.x, q.y, q.z), 0) − r.
/// At the centre of a 1 × 2 × 3 half-size box: q = (−1, −2, −3) → −1, minus the smallest half size.
///
/// Conventions: `halfSize` is half the width, height and depth; the box has its sharp corners
/// shrunk by `rounding` (clamped to the smallest half size, where a cube becomes a sphere);
/// exact for a sharp box and for the rounded box both inside and outside.
///
/// Use when: crates, walls, blocks, rounded cubes
/// Tags: sdf3Box, box, cube, round box, cuboid, sdBox, sdRoundBox
/// Node: none (shader library)
template <typename Real>
inline Real boxDistance(Vector3Of<Real> point, Vector3Of<Real> halfSize, Real rounding) {
    Real corner = clamped(rounding, Real(0.0f), smallerOf(halfSize.x, smallerOf(halfSize.y, halfSize.z)));
    Vector3Of<Real> offsetFromCorner = absolute(point) - (halfSize - corner);
    return lengthOf(largerOf(offsetFromCorner, Vector3Of<Real>(Real(0.0f))))
        + smallerOf(largerOf(offsetFromCorner.x, largerOf(offsetFromCorner.y, offsetFromCorner.z)), Real(0.0f)) - corner;
}

/// The distance from `point` to a torus lying in the horizontal (x, z) plane around the y axis.
///
/// How: Inigo Quilez, distance functions: sdTorus. |(|(x, z)| − majorRadius, y)| − minorRadius:
/// the distance to the ring of radius `majorRadius`, less the tube's radius.
///
/// Conventions: y is up (the torus's axis); exact; the tube's centre line is the circle of
/// radius `majorRadius` at y = 0.
///
/// Use when: rings, donuts, tyres, pipes bent into circles
/// Tags: sdf3Torus, torus, donut, ring, tire, sdTorus
/// Node: none (shader library)
template <typename Real>
inline Real torusDistance(Vector3Of<Real> point, Real majorRadius, Real minorRadius) {
    Vector2Of<Real> ringOffset(lengthOf(Vector2Of<Real>(point.x, point.z)) - majorRadius, point.y);
    return lengthOf(ringOffset) - minorRadius;
}

/// The distance from `point` to a capped cylinder around the y axis, centred on the origin.
///
/// How: Inigo Quilez, distance functions: sdCappedCylinder. d = |(|(x, z)|, y)| − (radius,
/// height / 2); result = min(max(d.x, d.y), 0) + |max(d, 0)|. Radius 1, height 4: centre → −1;
/// (3, 0, 0) → 2.
///
/// Conventions: y is up; `height` is the full height; exact.
///
/// Use when: cans, pillars, wheels, pegs
/// Tags: sdf3Cylinder, cylinder, can, pillar, capped cylinder, sdCappedCylinder
/// Node: none (shader library)
template <typename Real>
inline Real cylinderDistance(Vector3Of<Real> point, Real radius, Real height) {
    Vector2Of<Real> profile = absolute(Vector2Of<Real>(lengthOf(Vector2Of<Real>(point.x, point.z)), point.y))
        - Vector2Of<Real>(radius, height * Real(0.5f));
    return smallerOf(largerOf(profile.x, profile.y), Real(0.0f))
        + lengthOf(largerOf(profile, Vector2Of<Real>(Real(0.0f))));
}

/// The distance from `point` to a cone with its base at the bottom and its tip at the top, centred on the origin.
///
/// How: Inigo Quilez, distance functions: sdCone (the exact version). The cone is revolved
/// about y; in the plane (ρ, y) with ρ = |(x, z)| it is the triangle from the tip (0, h/2) to the
/// base corner (radius, −h/2). With q = (radius, −h) the tip-to-corner edge and w = (ρ, y − h/2)
/// the point relative to the tip: a = w minus its projection on the slanted edge; b = w minus its
/// projection on the base; the distance is √min(a·a, b·b), signed by which side of both lines the
/// point is on.
///
/// Conventions: y is up; `height` and `radius` are clamped to at least 0.000001; the base is at
/// y = −height / 2, the tip at y = +height / 2; exact. A radius 1, height 2 cone at its centre: −1/√5 = −0.447.
///
/// Use when: cones, spikes, party hats, tree tops
/// Tags: sdf3Cone, cone, spike, pyramid-like, sdCone
/// Node: none (shader library)
template <typename Real>
inline Real coneDistance(Vector3Of<Real> point, Real radius, Real height) {
    Real coneHeight = largerOf(height, Real(0.000001f));
    Vector2Of<Real> edge(largerOf(radius, Real(0.000001f)), -coneHeight);
    Vector2Of<Real> fromTip(lengthOf(Vector2Of<Real>(point.x, point.z)), point.y - coneHeight * Real(0.5f));
    Vector2Of<Real> toSlant = fromTip - edge * clamped(dotProduct(fromTip, edge) / dotProduct(edge, edge), Real(0.0f), Real(1.0f));
    Vector2Of<Real> toBase = fromTip - edge * Vector2Of<Real>(clamped(fromTip.x / edge.x, Real(0.0f), Real(1.0f)), Real(1.0f));
    Real orientation = signOf(edge.y);
    Real squaredDistance = smallerOf(dotProduct(toSlant, toSlant), dotProduct(toBase, toBase));
    Real side = largerOf(orientation * (fromTip.x * edge.y - fromTip.y * edge.x), orientation * (fromTip.y - edge.y));
    return squareRoot(squaredDistance) * signOf(side);
}

/// The signed distance from `point` to an infinite plane.
///
/// How: point · n + offset, where n = normalize(normalDirection + (0, 0.000001, 0)). The tiny
/// push on y keeps a zero `normalDirection` from dividing by zero; it makes that case a
/// horizontal plane facing up. Normal (0, 1, 0) and offset −2: the plane y = 2, so (0, 5, 0) → 3.
///
/// Conventions: positive on the side the normal points to; `normalDirection` need not be unit
/// length (it is normalised, to within the 0.000001 push); the plane is at distance −offset from
/// the origin along n.
///
/// Use when: floors, walls, cutting planes
/// Tags: sdf3Plane, plane, floor, half space, sdPlane
/// Node: none (shader library)
template <typename Real>
inline Real planeDistance(Vector3Of<Real> point, Vector3Of<Real> normalDirection, Real offset) {
    Vector3Of<Real> unitNormal = normalized(normalDirection + Vector3Of<Real>(Real(0.0f), Real(0.000001f), Real(0.0f)));
    return dotProduct(point, unitNormal) + offset;
}

/// The distance from `point` to a capsule: a segment along the y axis, centred on the origin, swollen by `radius`.
///
/// How: Inigo Quilez, distance functions: sdVerticalCapsule. y is slid toward the segment by
/// clamp(y, −height / 2, height / 2); result = |(x, y − clamp, z)| − radius. Radius 0.5, height 2:
/// (2, 0, 0) → 1.5; (0, 3, 0) → |2| − 0.5 = 1.5.
///
/// Conventions: y is up; `height` is the length of the core segment, not of the whole capsule
/// (which is height + 2 radius); exact.
///
/// Use when: limbs, pills, rounded bars
/// Tags: sdf3Capsule, capsule, pill, stadium, vertical capsule, sdVerticalCapsule
/// Node: none (shader library)
template <typename Real>
inline Real capsuleDistance(Vector3Of<Real> point, Real radius, Real height) {
    Vector3Of<Real> toSegment = point;
    toSegment.y -= clamped(toSegment.y, -height * Real(0.5f), height * Real(0.5f));
    return lengthOf(toSegment) - radius;
}

// ---------------------------------------------------------------------------------------------
// Placing a shape: these change the point before a distance function reads it.

/// The point to give a distance function so that its shape appears moved by `offset`.
///
/// How: point − offset. The shape at the origin, asked about point − offset, answers as if it
/// stood at `offset`.
///
/// Use when: moving a shape in a distance-field scene
/// Tags: sdf3Translated, translate, move, offset, domain transform
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> translatedPoint(Vector3Of<Real> point, Vector3Of<Real> offset) {
    return point - offset;
}

/// The point to give a distance function so that its shape appears turned by `degrees` about x, y and z.
///
/// How: the point is turned by the opposite angles, so the shape turns by the angles given: first
/// about z by −degrees.z, then about y by −degrees.y, then about x by −degrees.x (each a
/// right-handed rotation, written out in the formulas below). (1, 0, 0) with degrees (0, 0, 90)
/// → (0, −1, 0): the shape's +y axis ends on world +x.
///
///     about z by a: (c x − s y, s x + c y, z)       c = cos a, s = sin a
///     about y by a: (c x + s z, y, −s x + c z)
///     about x by a: (x, c y − s z, s y + c z)
///
/// Conventions: degrees (converted with degreesToRadians); right-handed, y up as the other 3D
/// shapes here; the order is z, then y, then x, applied to the point.
///
/// Use when: turning a shape in a distance-field scene
/// Tags: sdf3Rotated, rotate, euler angles, orientation, domain transform
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> rotatedPoint(Vector3Of<Real> point, Vector3Of<Real> degrees) {
    Vector3Of<Real> angles = -Vector3Of<Real>(degreesToRadians(degrees.x), degreesToRadians(degrees.y), degreesToRadians(degrees.z));
    Vector3Of<Real> turned = point;
    Real sineOfAngle = sine(angles.z);
    Real cosineOfAngle = cosine(angles.z);
    turned = Vector3Of<Real>(cosineOfAngle * turned.x - sineOfAngle * turned.y, sineOfAngle * turned.x + cosineOfAngle * turned.y, turned.z);
    sineOfAngle = sine(angles.y);
    cosineOfAngle = cosine(angles.y);
    turned = Vector3Of<Real>(cosineOfAngle * turned.x + sineOfAngle * turned.z, turned.y, -sineOfAngle * turned.x + cosineOfAngle * turned.z);
    sineOfAngle = sine(angles.x);
    cosineOfAngle = cosine(angles.x);
    turned = Vector3Of<Real>(turned.x, cosineOfAngle * turned.y - sineOfAngle * turned.z, sineOfAngle * turned.y + cosineOfAngle * turned.z);
    return turned;
}

/// The point folded into the grid cell of size `spacing` it lies in, so a shape repeats in x, y and z.
///
/// How: s = max(|spacing|, 0.000001); cell = ⌊point / s + 0.5⌋; when limit ≥ 0.5 the cell is
/// clamped to ±⌊limit + 0.5⌋; result = point − s × cell. Spacing 2, (3.5, 0, −5.2): cell
/// (2, 0, −3) → (−0.5, 0, 0.8).
///
/// Conventions: `limit` 0 means unlimited, otherwise the number of repeats either side of the
/// middle. The distance of a repeated shape is exact only if the shape fits within one cell.
///
/// Use when: arrays of pillars, bolts, tiles in 3D
/// Tags: sdf3Repeated, repeat, domain repetition, array, tile 3D
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> repeatedPoint(Vector3Of<Real> point, Real spacing, Real limit) {
    Real cellSize = largerOf(absolute(spacing), Real(0.000001f));
    Vector3Of<Real> cell = roundedDown(point / cellSize + Real(0.5f));
    if (limit >= Real(0.5f)) {
        Real largestCell = roundedDown(limit + Real(0.5f));
        cell = clamped(cell, -largestCell, largestCell);
    }
    return point - cellSize * cell;
}

/// The integer index of the grid cell that `repeatedPoint` folds the point into.
///
/// How: the `cell` of `repeatedPoint`: ⌊point / s + 0.5⌋, clamped to ±⌊limit + 0.5⌋ when limit ≥ 0.5.
///
/// Use when: giving each repeat of a shape its own colour or size
/// Tags: sdf3Cell, cell index, repeat id, domain repetition id
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> repeatedCell3(Vector3Of<Real> point, Real spacing, Real limit) {
    Real cellSize = largerOf(absolute(spacing), Real(0.000001f));
    Vector3Of<Real> cell = roundedDown(point / cellSize + Real(0.5f));
    if (limit >= Real(0.5f)) {
        Real largestCell = roundedDown(limit + Real(0.5f));
        cell = clamped(cell, -largestCell, largestCell);
    }
    return cell;
}

/// The point twisted about the y axis, more the higher it is, so a shape twists along its height.
///
/// How: angle = degreesToRadians(degreesPerUnit) × y; the point is turned about y by that angle:
/// (c x − s z, y, s x + c z) with c = cos angle, s = sin angle. At y = 0 nothing changes; 90°
/// per unit at (1, 1, 0) gives (0, 1, 1).
///
/// Conventions: degrees per unit of height; the result is a twisted domain, not an exact
/// distance: a distance read from it can exceed the true distance, so step rays shorter.
///
/// Use when: twisted columns, screws, rope
/// Tags: sdf3Twisted, twist, screw, helix, domain deformation
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> twistedPoint(Vector3Of<Real> point, Real degreesPerUnit) {
    Real angle = degreesToRadians(degreesPerUnit) * point.y;
    Real sineOfAngle = sine(angle);
    Real cosineOfAngle = cosine(angle);
    return Vector3Of<Real>(cosineOfAngle * point.x - sineOfAngle * point.z, point.y, sineOfAngle * point.x + cosineOfAngle * point.z);
}

} // namespace kernels
} // namespace editor
