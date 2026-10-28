// Mesh building, checking and the two face tools, inset and extrude (Engine/Geometry/Mesh.swift,
// Mesh+Primitives.swift and Mesh+Extrude.swift). The order of every loop and sum follows the
// Swift engine, so both produce the same points.

#include "Mesh.h"

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <unordered_map>

#include "HalfEdgeMesh.h"

namespace editor {

namespace {

/// A side walked from `start` to `end`, packed into one number to key a hash map.
std::uint64_t directedSide(int start, int end) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(start)) << 32) | static_cast<std::uint32_t>(end);
}

} // namespace

Mesh Mesh::box(Vector3 size) {
    Vector3 half = size / 2;
    Mesh mesh;
    mesh.positions = {
        Vector3(-half.x, -half.y, -half.z), Vector3(half.x, -half.y, -half.z),
        Vector3(half.x, half.y, -half.z), Vector3(-half.x, half.y, -half.z),
        Vector3(-half.x, -half.y, half.z), Vector3(half.x, -half.y, half.z),
        Vector3(half.x, half.y, half.z), Vector3(-half.x, half.y, half.z),
    };
    mesh.faces = {
        Face{{1, 2, 6, 5}}, // right,  +X
        Face{{3, 0, 4, 7}}, // left,   −X
        Face{{2, 3, 7, 6}}, // back,   +Y
        Face{{0, 1, 5, 4}}, // front,  −Y
        Face{{4, 5, 6, 7}}, // top,    +Z
        Face{{3, 2, 1, 0}}, // bottom, −Z
    };
    return mesh;
}

Polygon Mesh::polygon(const Face& face) const {
    Polygon result;
    result.corners.reserve(face.corners.size());
    for (int corner : face.corners) result.corners.push_back(positions[corner]);
    return result;
}

bool Mesh::isValid() const {
    int pointCount = static_cast<int>(positions.size());
    for (const Face& face : faces) {
        if (face.corners.size() < 3) return false;
        for (int corner : face.corners) {
            if (corner < 0 || corner >= pointCount) return false;
        }
    }
    return true;
}

bool Mesh::isWatertight() const {
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
        if (reverse == uses.end() || reverse->second != count) return false;
    }
    return true;
}

size_t Mesh::triangleCount() const {
    size_t total = 0;
    for (const Face& face : faces) {
        if (face.corners.size() >= 3) total += face.corners.size() - 2;
    }
    return total;
}

std::optional<Bounds> Mesh::bounds() const {
    if (positions.empty()) return std::nullopt;
    Bounds result{positions[0], positions[0]};
    for (Vector3 point : positions) {
        result.minimum = Vector3(std::min(result.minimum.x, point.x), std::min(result.minimum.y, point.y),
                                 std::min(result.minimum.z, point.z));
        result.maximum = Vector3(std::max(result.maximum.x, point.x), std::max(result.maximum.y, point.y),
                                 std::max(result.maximum.z, point.z));
    }
    return result;
}

InsetResult Mesh::inset(const std::vector<int>& selected, float thickness) const {
    std::vector<Vector3> newPositions = positions;
    std::vector<Face> newFaces = faces;
    std::vector<int> frame;
    for (int faceIndex : selected) {
        std::vector<int> corners = newFaces[faceIndex].corners;
        size_t count = corners.size();
        Vector3 normal = polygon(newFaces[faceIndex]).normal();
        std::vector<Vector3> inwardNormals;
        inwardNormals.reserve(count);
        for (size_t index = 0; index < count; ++index) {
            Vector3 side = newPositions[corners[(index + 1) % count]] - newPositions[corners[index]];
            inwardNormals.push_back(normal.cross(side).normalized());
        }
        int innerStart = static_cast<int>(newPositions.size());
        for (size_t index = 0; index < count; ++index) {
            Vector3 before = inwardNormals[(index + count - 1) % count];
            Vector3 after = inwardNormals[index];
            newPositions.push_back(newPositions[corners[index]] + insetCornerOffset(before, after, thickness));
        }
        std::vector<int> inner;
        for (size_t index = 0; index < count; ++index) inner.push_back(innerStart + static_cast<int>(index));
        newFaces[faceIndex] = Face{inner};
        for (size_t index = 0; index < count; ++index) {
            size_t next = (index + 1) % count;
            frame.push_back(static_cast<int>(newFaces.size()));
            newFaces.push_back(Face{{corners[index], corners[next], inner[next], inner[index]}});
        }
    }
    return InsetResult{Mesh{std::move(newPositions), std::move(newFaces)}, selected, frame};
}

Vector3 Mesh::insetCornerOffset(Vector3 first, Vector3 second, float thickness) {
    return (first + second) * (thickness / (1 + first.dot(second)));
}

ExtrudeResult Mesh::extruded(const std::vector<int>& selected, float distance) const {
    HalfEdgeMesh halfEdgeMesh(*this);
    std::vector<bool> isSelected(faces.size(), false);
    for (int face : selected) isSelected[face] = true;

    // The sides of the selection's outline: no face across, or a face that stays.
    std::vector<int> wallHalfEdges;
    for (int faceIndex : selected) {
        auto [first, end] = halfEdgeMesh.halfEdgesOfFace(faceIndex);
        for (int halfEdge = first; halfEdge < end; ++halfEdge) {
            std::optional<int> twin = halfEdgeMesh.halfEdges[halfEdge].twin;
            if (!twin || !isSelected[halfEdgeMesh.halfEdges[*twin].face]) wallHalfEdges.push_back(halfEdge);
        }
    }
    std::set<int> outlinePoints;
    for (int halfEdge : wallHalfEdges) outlinePoints.insert(halfEdgeMesh.halfEdges[halfEdge].origin);

    std::vector<Vector3> newPositions = positions;
    std::vector<Face> newFaces = faces;

    // A point inside the selection moves in place, along the mean normal of its faces.
    std::map<int, Vector3> normalSums;
    for (int faceIndex : selected) {
        Vector3 newell = polygon(faces[faceIndex]).newellVector();
        for (int corner : faces[faceIndex].corners) {
            if (!outlinePoints.count(corner)) normalSums[corner] += newell;
        }
    }
    for (const auto& [point, normalSum] : normalSums) {
        newPositions[point] += normalSum.normalized() * distance;
    }

    // A point on the outline gets one moved copy per run of selected faces around it, moved
    // along that run's mean normal. Two selected faces that touch only at a corner are two runs
    // there: with one copy, their four walls would share one edge.
    std::map<std::pair<int, int>, int> copies; // (point, face) → moved point
    for (int point : outlinePoints) { // std::set walks in sorted order, like Swift's `sorted()`
        for (const std::vector<int>& run : halfEdgeMesh.selectedRuns(point, isSelected)) {
            Vector3 normalSum;
            for (int face : run) normalSum = normalSum + polygon(faces[face]).newellVector();
            for (int face : run) copies[{point, face}] = static_cast<int>(newPositions.size());
            newPositions.push_back(positions[point] + normalSum.normalized() * distance);
        }
    }
    // The point a selected face uses for `point` after the move.
    auto moved = [&copies](int point, int face) {
        auto copy = copies.find({point, face});
        return copy == copies.end() ? point : copy->second;
    };

    std::vector<int> sides;
    for (int halfEdge : wallHalfEdges) {
        int start = halfEdgeMesh.halfEdges[halfEdge].origin;
        int end = halfEdgeMesh.destination(halfEdge);
        int face = halfEdgeMesh.halfEdges[halfEdge].face;
        sides.push_back(static_cast<int>(newFaces.size()));
        newFaces.push_back(Face{{start, end, moved(end, face), moved(start, face)}});
    }
    for (int faceIndex : selected) {
        for (int& corner : newFaces[faceIndex].corners) corner = moved(corner, faceIndex);
    }
    return ExtrudeResult{Mesh{std::move(newPositions), std::move(newFaces)}, selected, sides};
}

Mesh Mesh::subdivided(int levels, const EdgeCreases& creases) const {
    Mesh mesh = *this;
    EdgeCreases current = creases;
    for (int level = 0; level < levels; ++level) {
        auto [next, nextCreases] = HalfEdgeMesh(mesh).subdividedOnce(current);
        mesh = std::move(next);
        current = std::move(nextCreases);
    }
    return mesh;
}

} // namespace editor
