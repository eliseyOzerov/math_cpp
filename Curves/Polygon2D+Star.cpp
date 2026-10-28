// The outlines a ball's motifs are drawn with, and the crossing of a ray from the motif's centre
// with its outline, which is where a seam leaving the motif starts
// (Engine/Math/Paths/Polygon2D+Star.swift). Swift's preconditions are `MathProblem`s here.
#include <cmath>
#include <string>

#include "../Linear/Angle.h"
#include "../Linear/MathProblem.h"
#include "../Linear/NumberText.h"
#include "Polygon2D.h"

namespace editor {

namespace {
} // namespace

/// A **regular star** of `points` tips around the origin: `2 · points` corners at even angle
/// steps, alternately a tip at `outerRadius` and a notch at `innerRatio · outerRadius`, the
/// first tip at angle `turn` from +x.
///
/// Source: construction (the star polygon {k/1} with its notches pulled in).
///
/// How: corner j sits at angle `turn + j · π / points`, at the outer radius for even j and
/// the inner one for odd j:
///
///              tip 0 at `turn`
///                ●
///          ●   ╱   ╲   ●       points = 5: 10 corners, 36° apart
///           ╲ ●     ● ╱
///            ●   ·   ●         · the origin
///             ╲     ╱
///
/// `innerRatio = 1` gives a regular polygon of `2 · points` sides; `cos(π / points)` puts
/// each notch on the line between its two tips, so the outline is a regular polygon of
/// `points` sides with a corner in the middle of every side.
///
/// Conventions: counterclockwise; y up. `points` at least 2, `outerRadius` positive,
/// `innerRatio` above 0 and at most 1 (anything else throws a `MathProblem`). The star is
/// star-shaped about the origin: every ray from the origin crosses its outline once.
///
/// Use when: drawing the star or pentagon a ball's panel carries; a badge or a rosette
/// Tags: star polygon, regular star, five-point star, pentagram outline, star shape,
///   rosette, regular polygon with midpoints
/// Node: inside tileSphere
Polygon2D Polygon2D::star(int points, float outerRadius, float innerRatio, Angle turn) {
    if (!(points >= 2)) throw MathProblem("A star needs at least 2 points, but got " + std::to_string(points) + ".");
    if (!(outerRadius > 0 && std::isfinite(outerRadius))) throw MathProblem("A star needs a positive outer radius.");
    if (!(innerRatio > 0 && innerRatio <= 1)) {
        throw MathProblem("A star's inner ratio must be above 0 and at most 1, but got " + numberText(innerRatio) + ".");
    }
    Angle step = Angle::fromRadians(Angle::pi) / static_cast<float>(points);
    std::vector<Vector2> corners;
    for (int index = 0; index < 2 * points; ++index) {
        Angle angle = turn + step * static_cast<float>(index);
        float radius = index % 2 == 0 ? outerRadius : outerRadius * innerRatio;
        corners.push_back(Vector2(angle.cosine(), angle.sine()) * radius);
    }
    return Polygon2D(corners);
}

/// The first point where the ray from `origin` along `direction` meets the outline, or nothing
/// if it meets none of the sides.
///
/// Source: Goldman, "Intersection of two lines in three-space", *Graphics Gems* (1990), in
/// its 2D cross-product form; Preparata & Shamos, *Computational Geometry* (1985), for
/// star-shaped polygons.
///
/// How: a side from `a` with offset `e = b − a` meets the ray `origin + t·d` where
///
///     t = ((a − origin) × e) / (d × e)      u = ((a − origin) × d) / (d × e)
///
/// (2D cross products), with `0 ≤ u ≤ 1` on the side. Of all sides met with `t > 0`, the
/// smallest `t` wins. A side parallel to the ray (`d × e = 0`) is skipped:
///
///     origin ●──────●────▶ d
///                  ╱ crossing at the least t
///               a ●──── b
///
/// For a ray from a point the outline is star-shaped about (a star's centre) there is
/// exactly one crossing. A ray through a corner of the outline, such as a star tip lying on
/// a seam, meets the two sides at that corner at the same `t`, and returns the corner.
///
/// Conventions: `direction` need not have length 1, but must not be zero (a `MathProblem`
/// otherwise). `u` is allowed a tolerance of 1e-6 at both ends, so a ray through a corner is
/// not lost to rounding.
///
/// Use when: where a seam leaving a motif's centre crosses its outline; the edge of a shape
///   seen from inside it along a direction
/// Tags: ray polygon intersection, first hit, ray cast 2D, line segment intersection,
///   star-shaped polygon, crossing point
/// Node: inside tileSphere
std::optional<Vector2> Polygon2D::firstCrossing(Vector2 origin, Vector2 direction) const {
    if (direction == Vector2()) throw MathProblem("A ray needs a direction that is not zero.");
    const float sideTolerance = 1e-6f;
    std::optional<float> nearest;
    for (const Segment2& side : sides()) {
        Vector2 offset = side.end - side.start;
        float denominator = direction.cross(offset);
        if (denominator == 0) continue;
        Vector2 toStart = side.start - origin;
        float rayParameter = toStart.cross(offset) / denominator;
        float sideParameter = toStart.cross(direction) / denominator;
        if (!(rayParameter > 0 && sideParameter >= -sideTolerance && sideParameter <= 1 + sideTolerance)) continue;
        if (!nearest || rayParameter < *nearest) nearest = rayParameter;
    }
    if (!nearest) return std::nullopt;
    return origin + direction * *nearest;
}

} // namespace editor
