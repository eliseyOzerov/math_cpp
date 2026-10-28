// Questions asked of a finished mesh: whether it is closed, its area, the point on it nearest to
// another point, and where a ray first meets it (Engine/Geometry/Mesh.swift and
// Mesh+Queries.swift). They work on the fan triangles of the faces (`Mesh::triangles`), one
// triangle at a time, in face order, so ties resolve as in Swift.

#include <cstdint>
#include <unordered_map>

#include "Mesh.h"

namespace editor {

namespace {

/// A side walked from `start` to `end`, packed into one number to key a hash map.
std::uint64_t directedSide(int start, int end) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(start)) << 32) | static_cast<std::uint32_t>(end);
}

} // namespace

bool Mesh::isClosed() const {
    if (!isValid() || faces.empty()) return false;
    std::unordered_map<std::uint64_t, int> uses;
    for (const Face& face : faces) {
        size_t count = face.corners.size();
        for (size_t index = 0; index < count; ++index) {
            uses[directedSide(face.corners[index], face.corners[(index + 1) % count])] += 1;
        }
    }
    for (const auto& [side, count] : uses) {
        int start = static_cast<int>(side >> 32);
        int end = static_cast<int>(side & 0xffffffffu);
        auto reverse = uses.find(directedSide(end, start));
        if (count != 1 || reverse == uses.end() || reverse->second != 1) return false;
    }
    return true;
}

float Mesh::surfaceArea() const {
    float total = 0;
    for (const Face& face : faces) total = total + polygon(face).area();
    return total;
}

std::optional<MeshSurfacePoint> Mesh::closestPoint(Vector3 point) const {
    std::optional<MeshSurfacePoint> nearest;
    for (const Triangle& triangle : triangles()) {
        Vector3 candidate = triangle.closestPoint(point);
        float distance = candidate.distance(point);
        if (!nearest || distance < nearest->distance) {
            nearest = MeshSurfacePoint{candidate, triangle.normal(), distance};
        }
    }
    return nearest;
}

std::optional<MeshSurfacePoint> Mesh::firstHit(const Ray& ray) const {
    std::optional<Bounds> box = bounds();
    if (!box || !ray.intersectionDistance(*box)) return std::nullopt;
    std::optional<MeshSurfacePoint> nearest;
    for (const Triangle& triangle : triangles()) {
        std::optional<TriangleHit> hit = ray.intersection(triangle);
        if (!hit) continue;
        if (!nearest || hit->distance < nearest->distance) {
            nearest = MeshSurfacePoint{ray.point(hit->distance), triangle.normal(), hit->distance};
        }
    }
    return nearest;
}

} // namespace editor
