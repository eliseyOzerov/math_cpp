// A half-line (Engine/Math/Primitives/Ray.swift and RayIntersections.swift).
#pragma once

#include <optional>

#include "../Linear/Vector3.h"
#include "RayIntersections.h"

namespace editor {

struct Plane;
struct Sphere;
struct Bounds;
struct Triangle;

/// A starting point and a direction of travel. Picking in the viewport casts a ray from the
/// camera through the cursor and asks what it hits first.
///
/// Use when: picking what is under the cursor; shooting light rays for refraction
/// Tags: ray, origin and direction, picking ray, line of sight, ray cast
/// Node: inside raycast
struct Ray {
    /// Where the ray starts.
    Vector3 origin;
    /// The direction of travel, always length 1, so "distance along the ray" means world units.
    Vector3 direction;

    /// Builds a ray. The direction is normalized here, so callers can pass any non-zero vector.
    /// A zero-length direction has no heading, so it throws a `GeometryProblem` (Swift traps).
    /// Any non-zero length is accepted, because normalizing works at every scale.
    Ray(Vector3 origin, Vector3 direction);

    bool operator==(const Ray&) const = default;

    /// The point `distance` units along the ray from its origin.
    ///
    /// How: origin + direction × distance. Because `direction` has length 1, `distance` is
    /// measured in world units. Negative values give points behind the origin; the ray itself
    /// does not include them, but the formula still works.
    ///
    /// Use when: finding where a ray hits something once the distance is known
    /// Tags: point along ray, ray at distance, GetPoint, ray evaluate, ray march position
    /// Node: inside raycast
    Vector3 point(float distance) const;

    // The intersection queries (Swift's `Ray` extension in RayIntersections.swift) are defined in
    // RayIntersections.cpp.

    /// The distance along the ray to `plane`, or an empty optional if the ray never reaches it.
    /// See RayIntersections.cpp for the derivation.
    ///
    /// Use when: finding where a ray meets a floor or wall; picking a point on the ground under the cursor
    /// Tags: ray plane intersection, raycast plane, plane hit, ground picking, Plane.Raycast
    /// Node: none (optics simulation)
    std::optional<float> intersectionDistance(const Plane& plane) const;

    /// Where the ray crosses `triangle`, or an empty optional if it misses (Möller–Trumbore).
    ///
    /// Use when: picking a triangle of a mesh under the cursor; testing a ray against a mesh face
    /// Tags: ray triangle, Moller-Trumbore, triangle hit, mesh picking, raycast triangle, intersect triangle
    /// Node: inside raycast
    std::optional<TriangleHit> intersection(const Triangle& triangle) const;

    /// The distance along the ray to the surface of `sphere`, or an empty optional if it misses.
    ///
    /// Use when: picking a ball under the cursor; finding where a light ray enters a sphere
    /// Tags: ray sphere intersection, sphere raycast, ball picking, sphere hit, ray sphere test
    /// Node: none (ray against an analytic sphere; graphs raycast meshes)
    std::optional<float> intersectionDistance(const Sphere& sphere) const;

    /// The distance along the ray to the surface of `bounds`, or an empty optional if it misses
    /// (the slab method).
    ///
    /// Use when: quick picking test against an object's bounding box; skipping objects the ray misses
    /// Tags: ray box intersection, slab test, AABB raycast, bounds hit, ray AABB
    /// Node: inside raycast
    std::optional<float> intersectionDistance(const Bounds& bounds) const;
};

} // namespace editor
