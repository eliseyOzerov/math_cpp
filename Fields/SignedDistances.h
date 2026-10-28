// Exact signed distances to the basic solids, from Inigo Quilez's "distance functions"
// (iquilezles.org/articles/distfunctions). Negative inside, positive outside
// (Engine/Math/Fields/SignedDistances.swift).
//
// Swift makes `Sphere`, `Plane` and `Bounds` fields by extending them. Those types live in
// Primitives/, so here the field is asked through `signedDistanceOf` (and `boundsOf`, see
// FieldBounds.h); `DistanceField` takes any type that has these.
#pragma once

#include <optional>

#include "../Linear/GeometryProblem.h"
#include "../Geometry/Bounds.h"
#include "../Geometry/Plane.h"
#include "../Geometry/Segment.h"
#include "../Geometry/Sphere.h"
#include "../Linear/Vector3.h"

namespace editor {

/// The signed distance from `point` to the sphere's surface.
///
/// Source: Quílez, "distance functions", `sdSphere`.
///
/// How: the nearest surface point lies on the line from the centre through `point`, so
///
///     distance = |point − centre| − radius
///
///     centre ●────── radius ──────●────── distance ──────○ point
///
/// Inside the sphere it is negative, and −radius at the centre. A radius of 0 is a single
/// point. Exact everywhere, not only a bound.
///
/// Use when: modelling a ball, a bead, a blob; one metaball of a smooth union
/// Tags: sdSphere, sphere sdf, distance to sphere, metaball, ball, implicit sphere
/// Node: sphereField
float signedDistanceOf(const Sphere& sphere, Vector3 point);

/// A plane's signed distance is the one `Plane` already has; the field is the half space
/// behind it (negative on the side opposite the normal).
///
/// Use when: using a plane as a field in a combination (a half space)
/// Tags: half space, plane sdf, sdPlane
/// Node: none (value type)
float signedDistanceOf(const Plane& plane, Vector3 point);

/// All points within `radius` of a segment: a cylinder with a half sphere on each end, as a
/// limb, a pipe or a sausage.
///
/// Use when: modelling bones, tubes, arms; the base shape of a smooth blend
/// Tags: capsule, pill, sausage, stadium 3D, sdCapsule, rounded cylinder, line with radius
/// Node: capsuleField
struct Capsule {
    /// The core line; the surface is `radius` away from it.
    Segment axis;
    float radius = 0;

    /// A capsule around `axis`, `radius` thick. A radius of 0 is the segment itself and a
    /// degenerate segment (both ends equal) makes a sphere.
    Capsule(Segment axis, float radius) : axis(axis), radius(radius) {
        if (!(radius >= 0)) throw GeometryProblem("A capsule cannot have a negative radius.");
    }

    bool operator==(const Capsule&) const = default;

    /// The signed distance from `point` to the capsule's surface.
    ///
    /// Source: Quílez, "distance functions", `sdCapsule`.
    ///
    /// How: the distance to the capsule is the distance to its core segment, less the radius:
    ///
    ///     distance = |point − axis.closestPoint(to: point)| − radius
    ///
    ///     start ●━━━━━━━━━━━━━● end       the shell is everything within `radius` of the line
    ///
    /// Exact everywhere. A degenerate axis (`start == end`) gives the sphere's distance, since
    /// `closestPoint` then returns the single point. On the core line it is −radius.
    ///
    /// Use when: modelling limbs, tubes, rails; measuring clearance around a path
    /// Tags: sdCapsule, capsule sdf, distance to capsule, pill distance, distance to line minus radius
    /// Node: capsuleField
    float signedDistance(Vector3 point) const;

    /// The box around the axis segment's two ends, grown by `radius` on every side: the
    /// smallest box around the capsule (its two end balls touch it). Defined in FieldBounds.cpp
    /// (Swift's FieldBounds.swift).
    ///
    /// Use when: culling a capsule; sizing the region to mesh it in
    /// Tags: capsule bounds, capsule AABB, bounding box of segment and radius
    /// Node: inside boundingBox, fieldToMesh
    std::optional<Bounds> bounds() const;
};

/// The signed distance from `point` to the box, exact both outside and inside.
///
/// Source: Quílez, "distance functions", `sdBox`.
///
/// How: fold the point into the corner of positive coordinates and measure it against the
/// half size `h = size / 2`:
///
///     q = |point − centre| − h                    (how far past each face, per axis)
///     outside part = |max(q, 0)|                  (length of the part that sticks out:
///                                                  distance to the face, edge or corner)
///     inside part  = min(max(q.x, q.y, q.z), 0)   (the least-deep axis: minus the
///                                                  distance to the nearest face)
///     distance     = outside part + inside part
///
///     ┌───────────┐      past the top face only: q.y, the distance to that face
///     │     ○     │      past two faces: √(q.x² + q.y²), the distance to the edge
///     └───────────┘      inside: minus the distance to the nearest face
///
/// A box with zero size on some axes (a rectangle, a segment, a point) works unchanged.
/// For a point exactly on a face the result is 0.
///
/// Use when: modelling crates, walls, bricks; clipping a field to a region
/// Tags: sdBox, box sdf, distance to box, AABB distance, cuboid, rectangle distance, distance to bounding box
/// Node: boxField
float signedDistanceOf(const Bounds& box, Vector3 point);

/// Two spheres of different radii joined by their tangent cone: a tapered limb, a finger, a
/// teardrop. The sphere at `start` has `startRadius`, the one at `end` has `endRadius`.
///
/// Use when: modelling a limb that thins from thigh to knee, a nose, a bone with swollen ends
/// Tags: round cone, sdRoundCone, tapered capsule, uneven capsule, cone with spheres, taper
/// Node: taperedCapsuleField
struct RoundCone {
    Vector3 start;
    Vector3 end;
    float startRadius = 0;
    float endRadius = 0;

    /// A round cone from the sphere of `startRadius` at `start` to the sphere of `endRadius` at
    /// `end`. Radii may be 0 (a pointed tip) but not negative.
    RoundCone(Vector3 start, Vector3 end, float startRadius, float endRadius)
        : start(start), end(end), startRadius(startRadius), endRadius(endRadius) {
        if (!(startRadius >= 0 && endRadius >= 0)) throw GeometryProblem("A round cone cannot have a negative radius.");
    }

    bool operator==(const RoundCone&) const = default;

    /// The signed distance from `point` to the round cone's surface, exact.
    ///
    /// Source: Quílez, "distance functions", `sdRoundCone` (exact version); the derivation
    /// below is our own reading of it.
    ///
    /// How: work in the plane through the axis and the point. Let `L = |end − start|`, `y` the
    /// point's distance along the axis from `start`, `x ≥ 0` its distance from the axis,
    /// `d = startRadius − endRadius`, `s = d / L` and `c = √(1 − s²)`. The cone's side is the
    /// common tangent line of the two end circles. Its outward normal is `(c, s)` (it tilts
    /// toward the thinner end by `s`), and it touches the start circle at `startRadius × (c, s)`.
    ///
    ///        ●start        ╱ side, normal (c, s)
    ///       ( )━━━━━━━━━━━●╲        regions, told apart by w = c·y − s·x:
    ///                      end         w < 0          nearest to the start sphere
    ///                                  0 ≤ w ≤ c·L    nearest to the side
    ///                                  w > c·L        nearest to the end sphere
    ///
    ///     start sphere: |point − start| − startRadius
    ///     end sphere:   |point − end| − endRadius
    ///     side:         c·x + s·y − startRadius       (distance to the tangent line)
    ///
    /// `w` is the point's position along the side, from where it touches the start sphere (0)
    /// to where it touches the end sphere (`c·L`). The three formulas agree on the region
    /// borders, so the field is continuous.
    ///
    /// Edge cases: if one sphere contains the other (`L ≤ |d|`, which includes both centres
    /// equal) the shape is just the bigger sphere, and that is returned. Equal radii give a
    /// capsule.
    ///
    /// Use when: modelling a limb or finger that tapers; the body parts of a creature
    /// Tags: sdRoundCone, sdUnevenCapsule, round cone sdf, tapered capsule, uneven capsule, distance to cone, hull of two spheres
    /// Node: taperedCapsuleField
    float signedDistance(Vector3 point) const;

    /// The box around the two end balls: the round cone is the convex hull of its two
    /// spheres, and the box around both spheres holds that hull (a box is convex). Defined in
    /// FieldBounds.cpp.
    ///
    /// Use when: culling one piece of a tapered limb; sizing the region to mesh it in
    /// Tags: round cone bounds, tapered capsule AABB, bounding box of two spheres
    /// Node: taperedCapsuleField
    std::optional<Bounds> bounds() const;

    /// The same box as `bounds`, never nil: the box around both end balls. Defined in
    /// FieldBounds.cpp.
    ///
    /// Use when: storing the boxes of many round cones, as a tapered tube does
    /// Tags: round cone box, non-optional bounds, AABB of two spheres
    /// Node: taperedCapsuleField
    Bounds enclosingBox() const;
};

} // namespace editor
