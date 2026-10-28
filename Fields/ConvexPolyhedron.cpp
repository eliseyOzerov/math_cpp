// A convex polyhedron as an exact signed distance field
// (Engine/Math/Fields/ConvexPolyhedron.swift).
#include "ConvexPolyhedron.h"

#include <algorithm>
#include <limits>

#include "../Linear/GeometryProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

ConvexPolyhedron::ConvexPolyhedron(const Mesh& mesh) : box(Bounds{}) {
    if (!mesh.isClosed()) throw GeometryProblem("A convex polyhedron field needs a closed mesh.");
    for (const Mesh::Face& face : mesh.faces) {
        const std::vector<int>& corners = face.corners;
        facePlanes.push_back(Plane(mesh.positions[corners[0]], mesh.positions[corners[1]], mesh.positions[corners[2]]));
        // A fan from the first corner, as `Face.fanTriangleCorners` in Swift.
        std::vector<Triangle> triangles;
        for (size_t index = 1; index + 1 < corners.size(); ++index) {
            triangles.push_back(
                Triangle{mesh.positions[corners[0]], mesh.positions[corners[index]], mesh.positions[corners[index + 1]]});
        }
        faceTriangles.push_back(std::move(triangles));
    }
    box = Bounds::enclosing(mesh.positions);
    float tolerance = box.size().length() * Tolerance::geometric;
    for (const Plane& plane : facePlanes) {
        for (Vector3 position : mesh.positions) {
            if (!(plane.signedDistance(position) <= tolerance)) {
                throw GeometryProblem("A convex polyhedron field needs a convex mesh: a corner is in front of a face.");
            }
        }
    }
}

float ConvexPolyhedron::signedDistance(Vector3 point) const {
    float largestPlaneDistance = -std::numeric_limits<float>::infinity();
    for (const Plane& plane : facePlanes) largestPlaneDistance = std::max(largestPlaneDistance, plane.signedDistance(point));
    if (largestPlaneDistance <= 0) return largestPlaneDistance;

    float nearest = std::numeric_limits<float>::infinity();
    for (size_t face = 0; face < facePlanes.size(); ++face) {
        if (!(facePlanes[face].signedDistance(point) > 0)) continue;
        for (const Triangle& triangle : faceTriangles[face]) {
            nearest = std::min(nearest, triangle.closestPoint(point).distance(point));
        }
    }
    return nearest;
}

} // namespace editor
