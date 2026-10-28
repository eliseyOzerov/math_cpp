// The result type of a ray against a triangle (Engine/Math/Primitives/RayIntersections.swift).
//
// Where a ray meets a shape. Every query answers with the distance along the ray to the FIRST
// surface it meets in front of its origin, or an empty optional for no hit. The queries are
// members of `Ray` (Ray.h) and are written in RayIntersections.cpp.
//
// Shared rules:
// - Distances are in world units (the ray direction has length 1) and are never negative.
//   Hits behind the origin are not reported.
// - A hit exactly at the origin has distance 0 and counts.
// - Planes and triangles are two-sided: they are hit from either side.
// - Closed solids (sphere, box) are treated as surfaces. A ray that starts inside one
//   reports where it leaves (the only surface in front of it).
#pragma once

#include "Triangle.h"

namespace editor {

/// A ray–triangle hit: how far along the ray, and where in the triangle.
///
/// Use when: reading the result of a ray-triangle test, including where inside the triangle it landed
/// Tags: triangle hit, ray hit, barycentric hit, hit distance, intersection result
/// Node: inside raycast
struct TriangleHit {
    /// Distance from the ray origin to the hit point.
    float distance;
    /// The hit point as a blend of the triangle's corners.
    BarycentricCoordinates barycentric;

    bool operator==(const TriangleHit&) const = default;
};

} // namespace editor
