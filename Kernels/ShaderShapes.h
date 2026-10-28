// The shader library's two-dimensional shapes, ported from graphics_metal's Shapes.metal: a
// regular polygon with its antialiased edge, and the signed distances of the shape nodes (circle,
// box, segment, ellipse, triangle, ring, arc, star, heart, cross, moon, vesica, egg, Bézier
// curve, polygon) after Inigo Quilez's 2D distance functions
// (iquilezles.org/articles/distfunctions2d). Units are the canvas's: one is its height.
//
// Signs: every distance here is negative inside the shape, zero on its edge and positive
// outside, except `regularPolygonDistance`, which graphics_metal defines as the depth inside
// (positive inside), and which `polygonEdgeMask` consumes.
//
// Differences from graphics_metal, all on purpose:
// - Metal's `smoothstep` became `safeSmoothStep` (ShaderMath.h): the same curve, but a zero-width
//   edge gives a hard step instead of an undefined result.
// - Names are spelt out (`shapeCircle` → `circleDistance`); the graphics_metal name is in each
//   Tags line.
// - No function of Shapes.metal calls Noise or samples a texture, so all were ported.
#pragma once

#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderDistances.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

/// How far inside a regular polygon `point` is, in the polygon's own units: positive inside,
/// negative outside, 0 on the edge.
///
/// How: sides = max(3, ⌊|sidesInput| + 0.5⌋); sector = τ / sides; the angle of the point from
/// the x axis, less `rotation`, is wrapped to the nearest edge direction:
/// localAngle = ((angle + sector / 2) mod sector) − sector / 2. The edge lies at the apothem
/// radius cos(π / sides) from the centre, so along that direction it is at
/// apothem / cos(localAngle); the result is that boundary distance minus |point|.
///
/// Conventions: `radius` is the circumradius (corner distance), at least 0.000001 in size; with
/// `rotation` 0 an edge's middle faces +x; radians. The value is the radial depth, not the
/// perpendicular distance to the edge, so it is not Lipschitz 1 near the corners. At the centre
/// it is the apothem.
///
/// Use when: polygon masks, stars-with-flat-sides, badges
/// Tags: regularPolygonDistance, regular polygon, ngon, hexagon, octagon, polygon radial depth
/// Node: none (shader library)
template <typename Real>
inline Real regularPolygonDistance(Vector2Of<Real> point, Real sidesInput, Real radiusInput, Real rotation) {
    Real sides = largerOf(Real(3.0f), roundedDown(absolute(sidesInput) + Real(0.5f)));
    Real sector = tau<Real>() / sides;
    Real angle = arcTangent(point.y, point.x) - rotation;
    Real localAngle = flooredModulo(angle + sector * Real(0.5f), sector) - sector * Real(0.5f);
    Real radius = largerOf(absolute(radiusInput), Real(0.000001f));
    Real apothem = radius * cosine(pi<Real>() / sides);
    Real boundary = apothem / largerOf(cosine(localAngle), Real(0.000001f));
    return boundary - lengthOf(point);
}

/// A mask from the inside-depth of `regularPolygonDistance`: 0 outside, 1 inside, a soft edge between.
///
/// How: antialiasing = max(max(feather, 0), pixelWidth); mask = smoothstep(−antialiasing,
/// antialiasing, distanceInside). Halfway (depth 0) the mask is 0.5.
///
/// Conventions: `distanceInside` positive inside (the opposite sign to the other distances in
/// this file); `pixelWidth` the width of a pixel in the same units; a zero antialiasing gives a hard step.
///
/// Use when: turning a polygon's depth into an antialiased mask
/// Tags: polygonEdge, polygon mask, antialiased edge, smoothstep mask
/// Node: none (shader library)
template <typename Real>
inline Real polygonEdgeMask(Real distanceInside, Real pixelWidth, Real feather) {
    Real antialiasing = largerOf(largerOf(feather, Real(0.0f)), pixelWidth);
    return safeSmoothStep(-antialiasing, antialiasing, distanceInside);
}

/// The point to give a shape's distance function so that the shape appears turned by `degrees`.
///
/// How: the point is turned by −degrees: (c x − s y, s x + c y) with the angle a = −degrees in
/// radians, c = cos a, s = sin a. The point (1, 0) with degrees 90 gives (0, −1): the shape turns
/// counter-clockwise, so the shape's +y axis ends on world +x.
///
/// Conventions: degrees; counter-clockwise shape rotation seen with x to the right and y up.
///
/// Use when: turning any 2D shape before measuring its distance
/// Tags: shapeRotated, rotate shape, rotate point, 2D rotation
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> rotatedShapePoint(Vector2Of<Real> point, Real degrees) {
    Real angle = -degreesToRadians(degrees);
    Real sineOfAngle = sine(angle);
    Real cosineOfAngle = cosine(angle);
    return Vector2Of<Real>(cosineOfAngle * point.x - sineOfAngle * point.y, sineOfAngle * point.x + cosineOfAngle * point.y);
}

/// The distance from `point` to a circle of `radius` centred on the origin.
///
/// How: |point| − radius (Inigo Quilez, 2D distance functions: sdCircle). Exact.
///
/// Use when: discs, dots, round buttons
/// Tags: shapeCircle, circle, disc, dot, sdCircle
/// Node: none (shader library)
template <typename Real>
inline Real circleDistance(Vector2Of<Real> point, Real radius) {
    return lengthOf(point) - radius;
}

/// The distance from `point` to an axis-aligned rectangle centred on the origin.
///
/// How: q = |point| − halfSize; |max(q, 0)| + min(max(q.x, q.y), 0) (Inigo Quilez: sdBox). At the
/// centre of a rectangle with half size (1, 2): q = (−1, −2) → −1. Exact.
///
/// Conventions: `halfSize` is half the width and height.
///
/// Use when: squares, panels, bars
/// Tags: shapeBox, box, rectangle, square, sdBox
/// Node: none (shader library)
template <typename Real>
inline Real rectangleDistance(Vector2Of<Real> point, Vector2Of<Real> halfSize) {
    Vector2Of<Real> offsetFromCorner = absolute(point) - halfSize;
    return lengthOf(largerOf(offsetFromCorner, Vector2Of<Real>(Real(0.0f))))
        + smallerOf(largerOf(offsetFromCorner.x, offsetFromCorner.y), Real(0.0f));
}

/// The distance from `point` to a rectangle with rounded corners, centred on the origin.
///
/// How: r = clamp(radius, 0, min(halfSize.x, halfSize.y)); the distance to the rectangle shrunk
/// by r on every side (`rectangleDistance`), less r (Inigo Quilez: sdRoundedBox). Exact.
///
/// Conventions: a `radius` above the smaller half size is limited to it (a stadium or a circle).
///
/// Use when: buttons, cards, rounded panels
/// Tags: shapeRoundedBox, rounded rectangle, rounded box, pill, sdRoundedBox
/// Node: none (shader library)
template <typename Real>
inline Real roundedRectangleDistance(Vector2Of<Real> point, Vector2Of<Real> halfSize, Real radius) {
    Real cornerRadius = clamped(radius, Real(0.0f), smallerOf(halfSize.x, halfSize.y));
    return rectangleDistance(point, halfSize - cornerRadius) - cornerRadius;
}

/// The distance from `point` to the line segment from `start` to `end`.
///
/// How: h = clamp((point − start) · (end − start) / max(|end − start|², 0.000001), 0, 1); result
/// = |point − start − (end − start) h| (Inigo Quilez: sdSegment). Unsigned (never negative).
/// Segment (0,0)–(2,0) and point (1, 1): h = 0.5 → 1.
///
/// Conventions: a zero-length segment is the distance to `start`.
///
/// Use when: lines, strokes, bones; the building block of polylines
/// Tags: shapeSegment, line segment, distance to segment, sdSegment, capsule core
/// Node: none (shader library)
template <typename Real>
inline Real segmentDistance(Vector2Of<Real> point, Vector2Of<Real> start, Vector2Of<Real> end) {
    Vector2Of<Real> fromStart = point - start;
    Vector2Of<Real> alongSegment = end - start;
    Real fraction = clamped(dotProduct(fromStart, alongSegment) / largerOf(dotProduct(alongSegment, alongSegment), Real(0.000001f)),
        Real(0.0f), Real(1.0f));
    return lengthOf(fromStart - alongSegment * fraction);
}

/// An approximate distance from `point` to an ellipse with the given radii, centred on the origin.
///
/// How: r = max(|radii|, 0.000001); k0 = |point / r|; k1 = |point / r²|; result = k0 (k0 − 1) / k1,
/// or −min(r.x, r.y) at the centre where k1 would be 0 (Inigo Quilez, "ellipse - distance
/// estimation", iquilezles.org/articles/ellipsedist: the cheap approximation, not the exact root).
/// Radii (2, 1) and the point (2, 0) on the edge: k0 = 1 → 0; (4, 0): k0 = 2, k1 = 1 → 2.
///
/// Conventions: exact on the axes, a close estimate elsewhere and not Lipschitz 1 far away; the
/// sign is right everywhere.
///
/// Use when: ovals and eyes where a mask edge needs only be close
/// Tags: shapeEllipse, ellipse, oval, distance estimate, sdEllipse
/// Node: none (shader library)
template <typename Real>
inline Real ellipseDistance(Vector2Of<Real> point, Vector2Of<Real> radii) {
    Vector2Of<Real> semiAxes = largerOf(absolute(radii), Vector2Of<Real>(Real(0.000001f)));
    Real normalised = lengthOf(point / semiAxes);
    Real gradientLength = lengthOf(point / (semiAxes * semiAxes));
    return gradientLength < Real(0.000001f) ? -smallerOf(semiAxes.x, semiAxes.y)
                                            : normalised * (normalised - Real(1.0f)) / gradientLength;
}

/// The distance from `point` to an equilateral triangle with a flat bottom and a point at the top, centred on its middle.
///
/// How: Inigo Quilez, 2D distance functions: sdEquilateralTriangle, with `radius` half the side
/// length. With k = √3: p = (|x| − radius, y + radius / k); if p.x + k p.y > 0 then p is reflected
/// across the slanted edge, p = (p.x − k p.y, −k p.x − p.y) / 2; then p.x is clamped back onto the
/// edge, p.x −= clamp(p.x, −2 radius, 0); the result is −|p| × sign(p.y). The centre is at
/// −radius / √3 (the inradius); the base is at y = −radius / √3.
///
/// Conventions: side length 2 × radius; exact.
///
/// Use when: arrows, warning signs, play buttons
/// Tags: shapeTriangle, triangle, equilateral triangle, sdEquilateralTriangle
/// Node: none (shader library)
template <typename Real>
inline Real triangleDistance(Vector2Of<Real> point, Real radius) {
    const Real root = Real(1.7320508f);
    Vector2Of<Real> folded(absolute(point.x) - radius, point.y + radius / root);
    if (folded.x + root * folded.y > Real(0.0f)) {
        folded = Vector2Of<Real>(folded.x - root * folded.y, -root * folded.x - folded.y) * Real(0.5f);
    }
    folded.x -= clamped(folded.x, Real(-2.0f) * radius, Real(0.0f));
    return -lengthOf(folded) * signOf(folded.y);
}

/// The distance from `point` to a ring (an annulus) of the given centre-line `radius` and `thickness`.
///
/// How: |(|point| − radius)| − thickness / 2 (Inigo Quilez: sdRing in its simple form). Radius 2,
/// thickness 0.5: on the centre line → −0.25; the origin → 1.75. Exact.
///
/// Conventions: `radius` is to the middle of the band; `thickness` is the band's full width.
///
/// Use when: outlines of circles, rings, dials
/// Tags: shapeRing, ring, annulus, donut 2D, circle outline
/// Node: none (shader library)
template <typename Real>
inline Real ringDistance(Vector2Of<Real> point, Real radius, Real thickness) {
    return absolute(lengthOf(point) - radius) - thickness * Real(0.5f);
}

/// A rounded arc (`filled` false) or a pie slice (`filled` true), opening upwards over ±`aperture` degrees.
///
/// How: Inigo Quilez, 2D distance functions: sdArc and sdPie. sc = (sin aperture, cos aperture);
/// p = (|x|, y) folds the left half onto the right.
/// Pie: max(|p| − radius, |p − sc × clamp(p · sc, 0, radius)| × sign(sc.y p.x − sc.x p.y)).
/// Arc: d = |p − sc × radius| when sc.y p.x > sc.x p.y (beyond the aperture, nearest the arc's
/// end), else ||p| − radius|; result d − thickness / 2.
/// Aperture 45°, radius 1, thickness 0.2, arc: (0, 1) → −0.1; (0, 2) → 0.9.
///
/// Conventions: degrees, 0 to 180; the arc is centred on the +y axis and spans aperture either
/// side of it; `thickness` is used by the arc only; exact.
///
/// Use when: gauges, pie charts, smiles, progress arcs
/// Tags: shapeArc, arc, pie, pie slice, sector, gauge, sdArc, sdPie
/// Node: none (shader library)
template <typename Real>
inline Real arcDistance(Vector2Of<Real> point, Real aperture, Real radius, Real thickness, bool filled) {
    Vector2Of<Real> opening(sine(degreesToRadians(aperture)), cosine(degreesToRadians(aperture)));
    Vector2Of<Real> folded(absolute(point.x), point.y);
    if (filled) {
        Real outerEdge = lengthOf(folded) - radius;
        Real toRadialEdge = lengthOf(folded - opening * clamped(dotProduct(folded, opening), Real(0.0f), radius));
        return largerOf(outerEdge, toRadialEdge * signOf(opening.y * folded.x - opening.x * folded.y));
    }
    Real toBand = (opening.y * folded.x > opening.x * folded.y) ? lengthOf(folded - opening * radius)
                                                                 : absolute(lengthOf(folded) - radius);
    return toBand - thickness * Real(0.5f);
}

/// The distance from `point` to a star with `pointsInput` points, centred on the origin, one point up.
///
/// How: Inigo Quilez, 2D distance functions: sdStar. n = max(3, ⌊points + 0.5⌋); m = mix(n, 2,
/// clamp(pointiness, 0, 1)); an = π / n, en = π / m; acs = (cos an, sin an), ecs = (cos en, sin en).
/// The point is folded into one half-wedge with bn = ((atan2(x, y)) mod 2 an) − an,
/// p = |point| (cos bn, |sin bn|); then p −= radius acs and
/// p += ecs × clamp(−p · ecs, 0, radius acs.y / ecs.y); the result is |p| × sign(p.x).
///
/// Conventions: `radius` is the tip distance; angles in radians. Following the formula as
/// graphics_metal has it, `pointiness` 1 (m = 2) makes the edges meet the tips at right angles to
/// the wedge axis, giving the regular polygon with `points` corners; as `pointiness` falls
/// toward 0 (m → n) the notches between the tips deepen until the star has no width. The
/// comment in graphics_metal ("0 is a regular polygon") describes it the other way round; the
/// formula is ported unchanged.
///
/// Use when: stars, badges, sparkles, polygons by corner count
/// Tags: shapeStar, star, polygon, pentagram, sparkle, sdStar
/// Node: none (shader library)
template <typename Real>
inline Real starDistance(Vector2Of<Real> point, Real radius, Real pointsInput, Real pointiness) {
    Real count = largerOf(Real(3.0f), roundedDown(pointsInput + Real(0.5f)));
    Real notchedCount = linearBlend(count, Real(2.0f), clamped(pointiness, Real(0.0f), Real(1.0f)));
    Real cornerAngle = pi<Real>() / count;
    Real edgeAngle = pi<Real>() / notchedCount;
    Vector2Of<Real> cornerDirection(cosine(cornerAngle), sine(cornerAngle));
    Vector2Of<Real> edgeDirection(cosine(edgeAngle), sine(edgeAngle));
    Real wedgeAngle = flooredModulo(arcTangent(point.x, point.y), Real(2.0f) * cornerAngle) - cornerAngle;
    Vector2Of<Real> folded = lengthOf(point) * Vector2Of<Real>(cosine(wedgeAngle), absolute(sine(wedgeAngle)));
    folded -= radius * cornerDirection;
    folded += edgeDirection * clamped(-dotProduct(folded, edgeDirection), Real(0.0f), radius * cornerDirection.y / edgeDirection.y);
    return lengthOf(folded) * signOf(folded.x);
}

/// The distance from `point` to a heart of the given `size`, centred about the origin.
///
/// How: Inigo Quilez, 2D distance functions: sdHeart, scaled. s = max(size, 0.000001); p =
/// (|x|, y) / s + (0, 0.5). If p.y + p.x > 1 the nearest part is the round lobe: |p − (0.25, 0.75)|
/// − 0.35355339 (√2 / 4). Otherwise it is the nearer of the point (0, 1) and the diagonal
/// line: √min(|p − (0, 1)|², |p − 0.5 max(p.x + p.y, 0)|²) × sign(p.x − p.y). The answer is
/// multiplied back by s. At the origin with size 1: −0.35355339.
///
/// Conventions: the heart is about `size` wide and tall, point down at the bottom; the shape
/// scales exactly with `size` (distance(k point, k size) = k distance(point, size)).
///
/// Use when: hearts, like buttons, decorations
/// Tags: shapeHeart, heart, love, sdHeart
/// Node: none (shader library)
template <typename Real>
inline Real heartDistance(Vector2Of<Real> point, Real size) {
    Real scale = largerOf(size, Real(0.000001f));
    Vector2Of<Real> folded = Vector2Of<Real>(absolute(point.x), point.y) / scale + Vector2Of<Real>(Real(0.0f), Real(0.5f));
    Real distanceValue;
    if (folded.y + folded.x > Real(1.0f)) {
        Vector2Of<Real> fromLobe = folded - Vector2Of<Real>(Real(0.25f), Real(0.75f));
        distanceValue = squareRoot(dotProduct(fromLobe, fromLobe)) - Real(0.35355339f);
    } else {
        Vector2Of<Real> fromNotch = folded - Vector2Of<Real>(Real(0.0f), Real(1.0f));
        Vector2Of<Real> fromDiagonal = folded - Real(0.5f) * largerOf(folded.x + folded.y, Real(0.0f));
        distanceValue = squareRoot(smallerOf(dotProduct(fromNotch, fromNotch), dotProduct(fromDiagonal, fromDiagonal)))
            * signOf(folded.x - folded.y);
    }
    return distanceValue * scale;
}

/// The distance from `point` to a plus sign (two crossing bars), centred on the origin.
///
/// How: Inigo Quilez, 2D distance functions: sdCross. b = (size, thickness / 2); p = |point| with
/// its larger component first; q = p − b; k = max(q.y, q.x); w = k > 0 ? q : (b.y − p.x, −k);
/// result = sign(k) × |max(w, 0)|. Size 1, thickness 0.2: origin → −0.1414 (the nearest edge is
/// the inner corner at (0.1, 0.1)); (2, 0) → 1.
///
/// Conventions: `size` is the half length of each arm; `thickness` the full width of an arm;
/// exact.
///
/// Use when: plus signs, crosshairs, medical symbols
/// Tags: shapeCross, cross, plus, crosshair, sdCross
/// Node: none (shader library)
template <typename Real>
inline Real crossDistance(Vector2Of<Real> point, Real size, Real thickness) {
    Vector2Of<Real> armSize(size, thickness * Real(0.5f));
    Vector2Of<Real> folded = absolute(point);
    folded = (folded.y > folded.x) ? Vector2Of<Real>(folded.y, folded.x) : folded;
    Vector2Of<Real> fromCorner = folded - armSize;
    Real farthest = largerOf(fromCorner.y, fromCorner.x);
    Vector2Of<Real> toNearest = (farthest > Real(0.0f)) ? fromCorner : Vector2Of<Real>(armSize.y - folded.x, -farthest);
    return signOf(farthest) * lengthOf(largerOf(toNearest, Vector2Of<Real>(Real(0.0f))));
}

/// The distance from `point` to a crescent moon: a disc of radius `outer` with a disc of radius
/// `inner`, whose centre is `offset` to the right, cut out of it.
///
/// How: Inigo Quilez, 2D distance functions: sdMoon. d = max(|offset|, 0.000001); p = (x, |y|);
/// a = (outer² − inner² + d²) / (2d); b = √max(outer² − a², 0) (the horn tip is (a, b)). If
/// d (p.x b − p.y a) > d² max(b − p.y, 0) the nearest point is the horn tip: |p − (a, b)|;
/// otherwise max(|p| − outer, −(|p − (d, 0)| − inner)). Outer 1, inner 1, offset 1: (−1, 0) → 0,
/// (−0.5, 0) → −0.5.
///
/// Conventions: the crescent opens to the right (+x), its horn tips at (a, ±b); exact.
///
/// Use when: crescent moons, lens flares, eyelids
/// Tags: shapeMoon, moon, crescent, sdMoon
/// Node: none (shader library)
template <typename Real>
inline Real moonDistance(Vector2Of<Real> point, Real offset, Real outer, Real inner) {
    Real separation = largerOf(absolute(offset), Real(0.000001f));
    Vector2Of<Real> folded(point.x, absolute(point.y));
    Real tipX = (outer * outer - inner * inner + separation * separation) / (Real(2.0f) * separation);
    Real tipY = squareRoot(largerOf(outer * outer - tipX * tipX, Real(0.0f)));
    if (separation * (folded.x * tipY - folded.y * tipX) > separation * separation * largerOf(tipY - folded.y, Real(0.0f))) {
        return lengthOf(folded - Vector2Of<Real>(tipX, tipY));
    }
    return largerOf(lengthOf(folded) - outer, -(lengthOf(folded - Vector2Of<Real>(separation, Real(0.0f))) - inner));
}

/// The distance from `point` to a vesica (the lens where two overlapping discs of radius
/// `radius` meet), centred on the origin with its points up and down.
///
/// How: Inigo Quilez, 2D distance functions: sdVesica. p = |point|; d = min(|spread|, 0.999 radius)
/// is the distance of each disc's centre from the middle; b = √max(radius² − d², 0) is the half
/// height. If (p.y − b) d > p.x b the nearest point is a tip: |p − (0, b)|; otherwise
/// |p − (−d, 0)| − radius. Radius 1, spread 0.6: the origin → d − radius = −0.4; (0, 2) → 2 − 0.8 = 1.2.
///
/// Conventions: `spread` is how far apart the centres lie, at most 0.999 radius; exact.
///
/// Use when: eyes, leaves, lens shapes, fish
/// Tags: shapeVesica, vesica, lens, almond, leaf, sdVesica
/// Node: none (shader library)
template <typename Real>
inline Real vesicaDistance(Vector2Of<Real> point, Real radius, Real spread) {
    Vector2Of<Real> folded = absolute(point);
    Real centreOffset = smallerOf(absolute(spread), radius * Real(0.999f));
    Real halfHeight = squareRoot(largerOf(radius * radius - centreOffset * centreOffset, Real(0.0f)));
    return ((folded.y - halfHeight) * centreOffset > folded.x * halfHeight)
        ? lengthOf(folded - Vector2Of<Real>(Real(0.0f), halfHeight))
        : lengthOf(folded - Vector2Of<Real>(-centreOffset, Real(0.0f))) - radius;
}

/// The distance from `point` to an egg: a big circle of radius `bottom` below, a small one of
/// radius `top` above, joined by tangent arcs, pointing up.
///
/// How: Inigo Quilez, 2D distance functions: sdEgg. k = √3; p = (|x|, y); r = bottom − top;
/// d = p.y < 0 ? |p| − r : (k (p.x + r) < p.y ? |(p.x, p.y − k r)| : |(p.x + r, p.y)| − 2 r);
/// result = d − top. Bottom 1, top 0.5: the origin → −1; (0, −2) → 1.
///
/// Conventions: `bottom` should be at least `top`; the big circle is centred on the origin;
/// exact for that case.
///
/// Use when: eggs, drops, pears
/// Tags: shapeEgg, egg, drop, oval 2D, sdEgg
/// Node: none (shader library)
template <typename Real>
inline Real eggDistance(Vector2Of<Real> point, Real bottom, Real top) {
    const Real root = Real(1.7320508f);
    Vector2Of<Real> folded(absolute(point.x), point.y);
    Real difference = bottom - top;
    Real distanceValue = (folded.y < Real(0.0f)) ? lengthOf(folded) - difference
        : (root * (folded.x + difference) < folded.y) ? lengthOf(Vector2Of<Real>(folded.x, folded.y - root * difference))
        : lengthOf(Vector2Of<Real>(folded.x + difference, folded.y)) - Real(2.0f) * difference;
    return distanceValue - top;
}

/// The distance from `point` to a quadratic Bézier curve through control points `a`, `b`, `c`.
///
/// How: Inigo Quilez, "distance to a quadratic Bézier" (iquilezles.org/articles/distfunctions2d,
/// sdBezier): the closest parameter t solves a cubic, found in closed form (Cardano's method
/// when one real root, the trigonometric method when three), clamped to 0…1; the result is the
/// distance to B(t) = (1−t)² a + 2 (1−t) t b + t² c. If the curve is a straight line
/// (|a − 2b + c|² < 0.00001) it is the distance to the segment a–c.
///
/// Conventions: unsigned (never negative); exact apart from float rounding of the cubic's
/// root, which grows near a straight curve and where the point is on the curve's axis of symmetry.
///
/// Use when: smooth curves, brush strokes, path outlines
/// Tags: shapeBezier, bezier, quadratic bezier, curve distance, sdBezier
/// Node: none (shader library)
template <typename Real>
inline Real bezierDistance(Vector2Of<Real> point, Vector2Of<Real> a, Vector2Of<Real> b, Vector2Of<Real> c) {
    Vector2Of<Real> firstDifference = b - a;
    Vector2Of<Real> secondDifference = a - Real(2.0f) * b + c;
    if (dotProduct(secondDifference, secondDifference) < Real(0.00001f)) {
        return segmentDistance(point, a, c);
    }
    Vector2Of<Real> fromPoint = a - point;
    Real inverseSquared = Real(1.0f) / dotProduct(secondDifference, secondDifference);
    Real kx = inverseSquared * dotProduct(firstDifference, secondDifference);
    Real ky = inverseSquared * (Real(2.0f) * dotProduct(firstDifference, firstDifference) + dotProduct(fromPoint, secondDifference)) / Real(3.0f);
    Real kz = inverseSquared * dotProduct(fromPoint, firstDifference);
    Real depressed = ky - kx * kx;
    Real depressedCubed = depressed * depressed * depressed;
    Real q = kx * (Real(2.0f) * kx * kx - Real(3.0f) * ky) + kz;
    Real discriminant = q * q + Real(4.0f) * depressedCubed;
    Vector2Of<Real> doubledFirst = firstDifference * Real(2.0f);
    Real squaredDistance;
    if (discriminant >= Real(0.0f)) {
        discriminant = squareRoot(discriminant);
        Vector2Of<Real> cubeArguments = (Vector2Of<Real>(discriminant, -discriminant) - q) * Real(0.5f);
        Vector2Of<Real> cubeRoots = signOf(cubeArguments)
            * power(absolute(cubeArguments), Vector2Of<Real>(Real(1.0f) / Real(3.0f)));
        Real t = clamped(cubeRoots.x + cubeRoots.y - kx, Real(0.0f), Real(1.0f));
        Vector2Of<Real> residual = fromPoint + (doubledFirst + secondDifference * t) * t;
        squaredDistance = dotProduct(residual, residual);
    } else {
        Real root = squareRoot(-depressed);
        Real angle = arcCosine(clamped(q / (depressed * root * Real(2.0f)), Real(-1.0f), Real(1.0f))) / Real(3.0f);
        Real cosineOfAngle = cosine(angle);
        Real sineOfRoot = sine(angle) * Real(1.7320508f);
        Vector3Of<Real> t = clamped(Vector3Of<Real>(cosineOfAngle + cosineOfAngle, -sineOfRoot - cosineOfAngle, sineOfRoot - cosineOfAngle) * root - kx,
            Real(0.0f), Real(1.0f));
        Vector2Of<Real> firstResidual = fromPoint + (doubledFirst + secondDifference * t.x) * t.x;
        Vector2Of<Real> secondResidual = fromPoint + (doubledFirst + secondDifference * t.y) * t.y;
        squaredDistance = smallerOf(dotProduct(firstResidual, firstResidual), dotProduct(secondResidual, secondResidual));
    }
    return squareRoot(squaredDistance);
}

/// The signed distance from `point` to a polygon through up to six points, negative inside.
///
/// How: Inigo Quilez, 2D distance functions: sdPolygon. The squared distance to every edge
/// (point `j` to point `i`, the last to the first) is found as for a segment and the smallest is
/// kept; the sign flips for each edge a ray going right from the point crosses (the even-odd
/// rule): the test is point.y ≥ pi.y, point.y < pj.y, and which side of the edge the point is on
/// (e.x w.y > e.y w.x); the sign flips when all three hold or all three fail.
///
/// Conventions: `count` (rounded, from 3 to 6) is how many of the six points are used, in order
/// around the outline, either winding; the polygon may be concave but should not cross itself;
/// exact.
///
/// Use when: arbitrary outlines of up to six corners
/// Tags: shapePolygon, polygon, convex polygon, concave polygon, even-odd, sdPolygon
/// Node: none (shader library)
template <typename Real>
inline Real polygonDistance(Vector2Of<Real> point, Vector2Of<Real> p0, Vector2Of<Real> p1, Vector2Of<Real> p2,
    Vector2Of<Real> p3, Vector2Of<Real> p4, Vector2Of<Real> p5, Real count) {
    Vector2Of<Real> corners[6] = {p0, p1, p2, p3, p4, p5};
    int used = clamped(int(roundedDown(count + Real(0.5f))), 3, 6);
    Real squaredDistance = dotProduct(point - corners[0], point - corners[0]);
    Real winding = Real(1.0f);
    for (int index = 0, previous = used - 1; index < used; previous = index, index++) {
        Vector2Of<Real> edge = corners[previous] - corners[index];
        Vector2Of<Real> fromCorner = point - corners[index];
        Vector2Of<Real> toEdge = fromCorner - edge * clamped(dotProduct(fromCorner, edge) / largerOf(dotProduct(edge, edge), Real(0.000001f)),
            Real(0.0f), Real(1.0f));
        squaredDistance = smallerOf(squaredDistance, dotProduct(toEdge, toEdge));
        bool aboveStart = point.y >= corners[index].y;
        bool belowEnd = point.y < corners[previous].y;
        bool leftOfEdge = edge.x * fromCorner.y > edge.y * fromCorner.x;
        if ((aboveStart && belowEnd && leftOfEdge) || (!aboveStart && !belowEnd && !leftOfEdge)) {
            winding = -winding;
        }
    }
    return winding * squareRoot(squaredDistance);
}

} // namespace kernels
} // namespace editor
