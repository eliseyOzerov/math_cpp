// A solid with flat faces and no dents as an exact signed distance field
// (Engine/Math/Fields/ConvexPolyhedron.swift).
#pragma once

#include <optional>
#include <vector>

#include "../Mesh/Mesh.h"
#include "../Geometry/Bounds.h"
#include "../Geometry/Plane.h"
#include "../Geometry/Triangle.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A solid bounded by flat faces with no dents (a cube, a gem, any Platonic solid), as an
/// exact signed distance field.
///
/// How: a convex solid is the space behind all of its face planes at once.
///
/// - **Inside**, the nearest surface point lies on the nearest face plane, so the distance is
///   the largest of the signed distances to the planes (all negative; the largest is the
///   nearest).
/// - **Outside**, the distance is to the nearest point of the surface, which lies on a face
///   whose plane the point is in front of: at that surface point, the direction to the point
///   is a mix of the normals of the faces that meet there, so it is in front of at least one
///   of them. Only those faces are measured, each split into triangles and measured with
///   `Triangle::closestPoint` (Ericson, *Real-Time Collision Detection*, §5.1.5).
///
/// Exact everywhere, not only a bound: zero on the faces, negative inside, positive outside,
/// and 1-Lipschitz. `bounds` is the box around the corners, which holds the whole solid.
///
/// Conventions: the mesh closed, its faces flat and counterclockwise from outside, and the
/// solid convex. Checked when the field is made (throws `GeometryProblem`, where Swift
/// traps): every corner must be behind or on every face plane (within a millionth of the
/// solid's size).
///
/// Use when: the distance field of a die, a gem or a Platonic solid; carving or blending a
///   flat-faced seed shape with other fields
/// Tags: convex polyhedron sdf, polytope distance, distance to convex mesh, platonic solid
///   sdf, sdOctahedron, sdTetrahedron, half-space intersection
/// Node: platonicSolidField
struct ConvexPolyhedron {
    /// The plane of each face, its normal pointing out of the solid.
    std::vector<Plane> facePlanes;
    /// The triangles of each face, in the order of `facePlanes`.
    std::vector<std::vector<Triangle>> faceTriangles;
    /// The box around the corners.
    Bounds box;

    /// The field of the convex solid `mesh` encloses.
    explicit ConvexPolyhedron(const Mesh& mesh);

    bool operator==(const ConvexPolyhedron&) const = default;

    /// The signed distance from `point` to the solid's surface: negative inside.
    ///
    /// Use when: sampling the solid
    /// Tags: sdf, convex polyhedron distance
    /// Node: platonicSolidField
    float signedDistance(Vector3 point) const;

    /// The box around the corners, which holds the whole solid.
    ///
    /// Use when: culling the solid; sizing the region to mesh it in
    /// Tags: convex polyhedron bounds, polyhedron AABB
    /// Node: platonicSolidField
    std::optional<Bounds> bounds() const { return box; }
};

} // namespace editor
