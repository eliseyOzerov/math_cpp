// Triangles, the surface of every mesh (Engine/Math/Primitives/Triangle.swift).
#pragma once

#include <optional>

#include "../Linear/Angle.h"
#include "../Linear/Vector3.h"

namespace editor {

/// How much of each corner a point in a triangle's plane is made of. For a point p:
/// p = first·cornerFirst + second·cornerSecond + third·cornerThird, with first + second + third = 1.
///
/// A point inside the triangle has all three weights between 0 and 1. A weight of 1 is that
/// corner itself; a negative weight means the point lies outside, across the opposite edge.
///
/// Use when: blending a vertex color or texture coordinate across a triangle; testing whether a hit lies inside a face
/// Tags: barycentric, barycentric weights, triangle weights, interpolation weights, uvw, hit coordinates
/// Node: none (value type)
struct BarycentricCoordinates {
    float first;
    float second;
    float third;

    bool operator==(const BarycentricCoordinates&) const = default;

    /// Whether the point is inside the triangle or on its edge: all weights ≥ −tolerance.
    ///
    /// Use when: accepting or rejecting a ray hit on a triangle
    /// Tags: point in triangle, inside test, barycentric test, triangle containment, on triangle
    /// Node: barycentric
    bool isInsideTriangle(float tolerance = Tolerance::geometric) const {
        return first >= -tolerance && second >= -tolerance && third >= -tolerance;
    }
};

/// Three corners, counterclockwise seen from the front, the same winding the renderer uses to
/// decide which side faces the camera.
///
/// Use when: describing one face of a mesh; testing picks and shadows against it
/// Tags: triangle, face, mesh triangle, three points, polygon, primitive
/// Node: none (value type)
struct Triangle {
    Vector3 first;
    Vector3 second;
    Vector3 third;

    bool operator==(const Triangle&) const = default;

    /// Whether the corners are (nearly) in a straight line, so the triangle has no area.
    ///
    /// How: |edge1 × edge2| ≤ tolerance × longestEdge². The cross product's length is twice the
    /// area, and twice the area is longestEdge × height (the height measured to the longest
    /// edge). So the test asks whether height / longestEdge, how flat the triangle is, is under
    /// `Tolerance::geometric`. Being a ratio of lengths, it does not depend on the triangle's size.
    ///
    /// Use when: skipping faces that have no area when building normals or picking
    /// Tags: degenerate triangle, zero area, collinear points, sliver, flat triangle
    /// Node: inside measureMesh, closestPoint, raycast
    bool isDegenerate() const;

    /// The direction the triangle faces: length 1, by the right-hand rule (corners
    /// counterclockwise seen from the front give a normal pointing toward the viewer).
    ///
    /// How: (second − first) × (third − first), normalized (Ericson §3.4). A degenerate triangle
    /// has no facing, so the result is the zero vector.
    ///
    /// Use when: shading a face; finding which side a triangle faces
    /// Tags: face normal, triangle normal, flat shading, cross product normal, surface direction
    /// Node: inside closestPoint, raycast
    Vector3 normal() const;

    /// The surface area: half the length of the cross product of two edges (a parallelogram has
    /// area |edge1 × edge2|, a triangle is half of it). Exactly 0 is not forced for degenerate
    /// triangles; their cross product is simply tiny.
    ///
    /// Use when: weighting faces by size; measuring the surface area of a mesh
    /// Tags: triangle area, surface area, face area, half cross product
    /// Node: inside measureMesh
    float area() const;

    /// The centre of mass: the average of the three corners, (first + second + third) / 3.
    ///
    /// Use when: placing a label or an origin at the middle of a face
    /// Tags: triangle centre, centroid, barycentre, face centre, average of corners
    /// Node: none (the triangle case of a face centre, which measureFaces offers)
    Vector3 centroid() const;

    /// The weights that express `point` as a blend of the corners, for a point in the triangle's
    /// plane.
    ///
    /// Source: Ericson §3.4, "Barycentric coordinates". With edge1 = second − first, edge2 =
    /// third − first and offset = point − first, dot both sides with each edge to get two linear
    /// equations in the two unknowns, and solve them by Cramer's rule:
    ///
    ///     denominator = (edge1·edge1)(edge2·edge2) − (edge1·edge2)²
    ///     second = ((edge2·edge2)(offset·edge1) − (edge1·edge2)(offset·edge2)) / denominator
    ///     third  = ((edge1·edge1)(offset·edge2) − (edge1·edge2)(offset·edge1)) / denominator
    ///     first  = 1 − second − third
    ///
    /// A point off the plane is first projected along the normal, so the result describes its
    /// shadow on the triangle. Returns an empty optional for a degenerate triangle.
    ///
    /// Use when: turning a ray hit into a weight for each corner, to blend colors or UVs
    /// Tags: barycentric coordinates, triangle weights, interpolation, uvw, point in triangle
    /// Node: barycentric
    std::optional<BarycentricCoordinates> barycentricCoordinates(Vector3 point) const;

    /// The point of the triangle (inside or on its border) nearest to `point`.
    ///
    /// Source: Ericson §5.1.5, "Closest point on a triangle to a point". The plane around the
    /// triangle is cut into seven regions: three around the corners, three around the edges and
    /// the face itself. Dot products with the edges tell which region `point` is in. In a corner
    /// region the answer is that corner; in an edge region it is the closest point on that edge;
    /// in the face region it is the projection onto the triangle's plane, found from barycentric
    /// weights. Degenerate triangles have no face, so the answer is the nearest point on the
    /// three edges.
    ///
    /// Use when: snapping a point onto a mesh face; finding how far a point is from a triangle
    /// Tags: closest point on triangle, nearest point, distance to triangle, snap to surface, project to triangle
    /// Node: inside closestPoint
    Vector3 closestPoint(Vector3 point) const;

private:
    /// The cross product of two edges: perpendicular to the triangle, with a length equal to
    /// twice the area. Shared by `normal`, `area` and the degeneracy test.
    Vector3 scaledNormal() const;
};

} // namespace editor
