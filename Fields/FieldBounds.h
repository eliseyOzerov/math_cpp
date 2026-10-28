// Boxes around fields. Every field says which box holds its inside, so a model made of many
// parts can skip the parts that are far from the point being evaluated, and a surface
// extractor knows where to look. The box rules for each field are proven in its `bounds`
// comment; the shared box arithmetic lives here (Engine/Math/Fields/FieldBounds.swift).
//
// Swift adds these as members of `Bounds`, `Sphere` and `Plane`. Those types live in
// Primitives/, so here they are free functions that take the box or the shape first. The
// capsule's and the round cone's `bounds` are members (SignedDistances.h).
#pragma once

#include <optional>

#include "../Geometry/Bounds.h"
#include "../Geometry/Plane.h"
#include "../Geometry/Sphere.h"
#include "../Linear/Transform.h"
#include "SignedDistances.h"

namespace editor {

/// The box from `centre − radius` to `centre + radius` on every axis: the smallest box
/// around the ball.
///
/// Use when: culling a ball; sizing the region to mesh it in
/// Tags: sphere bounds, sphere AABB, bounding box of ball
/// Node: inside boundingBox, fieldToMesh
std::optional<Bounds> boundsOf(const Sphere& sphere);

/// Nil: a half space is unbounded, so there is no box around its inside.
///
/// Use when: checking whether a field can be culled by a box (a plane never can)
/// Tags: plane bounds, unbounded, half space, infinite extent
/// Node: inside boundingBox, fieldToMesh
std::optional<Bounds> boundsOf(const Plane& plane);

/// The box itself: a box's inside is the box.
///
/// Use when: treating a box as a field in a combination
/// Tags: box bounds, AABB of box
/// Node: inside boundingBox, fieldToMesh
std::optional<Bounds> boundsOf(const Bounds& box);

/// The box with every face moved outward by `margin`: from `minimum − margin` to
/// `maximum + margin` on each axis.
///
/// How: it holds every point within `margin` of the original box (such a point is within
/// `margin` of the box on each axis separately). So for a point outside the grown box,
/// the distance to the grown box is at most the distance to the original box minus
/// `margin`. The field `bounds` rules use this to grow a box by exactly what a blend or an
/// offset grows a shape.
///
/// Conventions: `margin` must be 0 or more (NaN is rejected, with a `MathProblem`); `+∞` gives
/// the whole space.
///
/// Use when: padding a box by a blend radius, an offset, a shell's half thickness
/// Tags: expand bounds, grow box, inflate AABB, pad bounds, dilate box, Expand
/// Node: inside fieldToMesh
Bounds expanded(const Bounds& box, float margin);

/// The box of the points inside both boxes, or nil when they do not overlap.
///
/// How: on each axis the overlap runs from the larger minimum to the smaller maximum; if
/// that is empty on any axis, the boxes share no point. Boxes that only touch give a flat
/// box.
///
/// Use when: bounding the intersection of two shapes; clipping a region to a box
/// Tags: intersect bounds, box overlap, AABB intersection, common box, clip box
/// Node: inside intersectFields
std::optional<Bounds> intersection(const Bounds& first, const Bounds& second);

/// The part of the box on the front side of an axis-aligned `plane` (the side its normal
/// points to, including the plane), or the box itself when that cannot be cut exactly.
///
/// How: when the plane's normal is `±` one axis, the front side is `axis ≥ d` (normal
/// `+`) or `axis ≤ −d` (normal `−`), with `d = plane.distance`, so the cut just moves one
/// face of the box. For a tilted plane the cut would be a many-sided shape; the whole box
/// is returned, which still holds everything, only looser. When the whole box is behind
/// the plane there is no front part, and the whole box is returned too.
///
/// Conventions: the result always holds every point of the box on the plane's front side.
///
/// Use when: bounding what a mirror keeps; clipping a box to a cut
/// Tags: clip box to half space, cut AABB, crop bounds, front part of box
/// Node: inside mirrorField
Bounds frontPart(const Bounds& box, const Plane& plane);

/// The box around both boxes, or nil when either is nil (an unbounded part makes the
/// whole unbounded). (Swift `Bounds.enclosing(_:_:)`.)
///
/// Use when: bounding the union of two fields, each of which may be unbounded
/// Tags: optional union, merge optional bounds, union of boxes, unbounded
/// Node: inside unionFields
std::optional<Bounds> enclosing(const std::optional<Bounds>& first, const std::optional<Bounds>& second);

/// The axis-aligned box around `box` after `transform` (scale, then rotate, then move):
/// the smallest such box around the moved box's eight corners.
///
/// Source: Arvo, "Transforming axis-aligned bounding boxes", *Graphics Gems* (1990).
///
/// How: the box is its centre plus `±h` along each axis (`h` = half the size). The
/// transform moves the centre to `T(centre)` and turns the three half-edge vectors into
/// `M(hₓ·x̂)`, `M(h_y·ŷ)`, `M(h_z·ẑ)`, where `M` is the linear part (rotation of the
/// scaled vector). A corner is the centre plus a ± sum of the three; on each world axis the
/// farthest corner reaches the sum of their absolute values on that axis:
///
///     new half size = |M(hₓ·x̂)| + |M(h_y·ŷ)| + |M(h_z·ẑ)|     (componentwise absolute)
///
/// Conventions: `transform.rotation` must be a unit quaternion. Negative scales (mirrors)
/// are fine.
///
/// Use when: bounding a transformed field; finding the world box of a moved object
/// Tags: transform AABB, Arvo, world bounds, rotated box, bounding box after rotation
/// Node: inside transform
Bounds transformed(const Bounds& box, const Transform& transform);

/// The axis-aligned box around `box`'s mirror image in `plane`.
///
/// How: as `transformed` (Arvo, 1990), with the reflection `p ↦ p − 2·(n·p − d)·n`
/// as the map: the centre goes to `reflection(plane, centre)`, and its linear part
/// `v ↦ v − 2(n·v)n` (Householder) acts on the half-edge vectors.
///
/// Use when: bounding a mirrored field
/// Tags: mirror AABB, reflect box, symmetric bounds, bounding box of reflection
/// Node: inside mirrorField
Bounds reflected(const Bounds& box, const Plane& plane);

/// `value` raised to at least the signed distance from `point` to `box`:
/// `max(value, box distance)`, with NaN kept.
///
/// Why it is safe: if the box holds the whole inside of a shape and `value` is a
/// conservative distance to it, the box's distance is conservative too. Outside the box,
/// the true distance to the shape is at least the distance to the box. Inside the box the
/// box's distance is negative, and its size is at least the distance to the shape's
/// surface (on the way out of the box a point crosses the surface), so it never wins
/// over a value that is right in sign and size. The larger of two conservative values is
/// conservative, and the maximum of two 1-Lipschitz functions is 1-Lipschitz. This is how
/// fields that are only bounds keep the "at least the box's distance" part of the
/// `DistanceField` contract.
///
/// Edge cases: a NaN `value` or `point` gives NaN (a plain maximum would drop it).
///
/// Use when: improving a field that underestimates far away, such as an ellipsoid bound
///   or noise displacement, without losing conservativeness
/// Tags: max with box, clamp to bounds distance, floor sdf, improve bound, box lower bound
/// Node: inside fieldToMesh
float raising(const Bounds& box, float value, Vector3 point);

/// The mirror image of `point` in the plane: as far behind the plane as `point` is in
/// front, on the same normal line.
///
/// How: `point − 2 × signedDistance(point) × normal`.
///
///     point ●
///           │ s
///     ──────┼────── plane
///           │ s
///   mirror  ○
///
/// Conventions: `normal` is unit (the plane keeps it so). A point on the plane is its own
/// mirror image; applying it twice gives the point back (up to rounding).
///
/// Use when: mirroring a shape across a plane of symmetry; reflecting a point
/// Tags: reflect point, mirror point, reflection across plane, symmetry, Householder
/// Node: inside mirrorField
Vector3 reflection(const Plane& plane, Vector3 point);

} // namespace editor
