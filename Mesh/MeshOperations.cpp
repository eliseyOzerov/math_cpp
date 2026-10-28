// Moving and combining meshes, normals and displacement, smoothing, and measuring
// (Engine/Geometry/Mesh+Operations.swift, Mesh+Smoothing.swift, Mesh.swift and
// Triangle+Volume.swift). The order of every loop and sum follows the Swift engine, so both
// produce the same points.

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>

#include "../Linear/NumberText.h"
#include "../Linear/GeometryProblem.h"
#include "Mesh.h"

namespace editor {

namespace {

/// For every point of a mesh, the points joined to it by an edge, and whether it is on a
/// boundary. Built from the faces in one pass.
///
/// How: every face edge (consecutive corners) is stored once as an unordered pair, with a count
/// of the faces that use it; an edge used once is a boundary edge and both its ends are fixed.
/// The pairs are then laid out as a compressed list: `starts[v]..starts[v + 1]` indexes the
/// neighbours of `v` in `flat`, in increasing order, so sums over them are the same on every run.
///
/// Use when: smoothing a mesh, which moves each point toward the average of its neighbours
/// Tags: vertex neighbours, point adjacency, boundary points, umbrella neighbours
/// Node: inside smooth
class VertexNeighbours {
public:
    explicit VertexNeighbours(const Mesh& mesh) {
        std::map<std::uint64_t, int> uses; // sorted by key, like Swift's `uses.keys.sorted()`
        for (const Mesh::Face& face : mesh.faces) {
            for (size_t index = 0; index < face.corners.size(); ++index) {
                int first = face.corners[index];
                int second = face.corners[(index + 1) % face.corners.size()];
                if (first == second) continue;
                uses[key(first, second)] += 1;
            }
        }
        size_t pointCount = mesh.positions.size();
        std::vector<int> counts(pointCount, 0);
        fixed.assign(pointCount, false);
        for (const auto& [pairKey, count] : uses) {
            auto [low, high] = ends(pairKey);
            counts[low] += 1;
            counts[high] += 1;
            if (count == 1) {
                fixed[low] = true;
                fixed[high] = true;
            }
        }
        starts.assign(pointCount + 1, 0);
        for (size_t point = 0; point < pointCount; ++point) starts[point + 1] = starts[point] + counts[point];
        flat.assign(static_cast<size_t>(starts[pointCount]), 0);
        std::vector<int> filled = starts;
        for (const auto& [pairKey, count] : uses) {
            auto [low, high] = ends(pairKey);
            flat[filled[low]] = high;
            filled[low] += 1;
            flat[filled[high]] = low;
            filled[high] += 1;
        }
    }

    /// The points joined to `point` by an edge: from index `begin` up to (not including) `end`
    /// of `neighbours`.
    std::pair<int, int> rangeOf(int point) const { return {starts[point], starts[point + 1]}; }
    int neighbour(int position) const { return flat[position]; }

    /// Whether `point` must not move: it is on a boundary, or no face uses it.
    bool isFixed(int point) const { return fixed[point] || starts[point] == starts[point + 1]; }

private:
    std::vector<int> starts;
    std::vector<int> flat;
    std::vector<bool> fixed;

    /// The smaller point number in the high bits, the larger in the low bits: one number per
    /// unordered pair.
    static std::uint64_t key(int first, int second) {
        return (static_cast<std::uint64_t>(std::min(first, second)) << 32) | static_cast<std::uint64_t>(std::max(first, second));
    }
    static std::pair<int, int> ends(std::uint64_t pairKey) {
        return {static_cast<int>(pairKey >> 32), static_cast<int>(pairKey & 0xFFFFFFFFu)};
    }
};

/// The mesh after `count` rounds of the umbrella steps in `factors`, in order.
Mesh umbrellaSteps(const Mesh& mesh, int count, const std::vector<float>& factors) {
    VertexNeighbours neighbours(mesh);
    std::vector<Vector3> current = mesh.positions;
    std::vector<Vector3> next = mesh.positions;
    for (int round = 0; round < count; ++round) {
        for (float factor : factors) {
            for (size_t point = 0; point < current.size(); ++point) {
                if (neighbours.isFixed(static_cast<int>(point))) continue;
                auto [begin, end] = neighbours.rangeOf(static_cast<int>(point));
                Vector3 sum(0, 0, 0);
                for (int position = begin; position < end; ++position) sum += current[neighbours.neighbour(position)];
                Vector3 average = sum / static_cast<float>(end - begin);
                next[point] = current[point] + (average - current[point]) * factor;
            }
            std::swap(current, next);
        }
    }
    Mesh result = mesh;
    result.positions = std::move(current);
    return result;
}

void checkIterations(int iterations) {
    if (iterations < 0 || iterations > MeshSmoothing::maximumIterations) {
        throw GeometryProblem("Smoothing needs 0 to " + std::to_string(MeshSmoothing::maximumIterations)
                              + " iterations, but got " + std::to_string(iterations) + ".");
    }
}

} // namespace

Mesh Mesh::transformed(const Matrix4& matrix) const {
    bool flipsFaces = Matrix3::upperLeftOf(matrix).determinant() < 0;
    Mesh result;
    result.positions.reserve(positions.size());
    for (Vector3 position : positions) result.positions.push_back(matrix.transformPoint(position));
    result.faces = faces;
    if (flipsFaces) {
        for (Face& face : result.faces) std::reverse(face.corners.begin(), face.corners.end());
    }
    return result;
}

Mesh Mesh::transformed(const Transform& transform) const { return transformed(transform.matrix()); }

Mesh Mesh::joined(const std::vector<Mesh>& meshes) {
    Mesh result;
    for (const Mesh& mesh : meshes) {
        int firstIndex = static_cast<int>(result.positions.size());
        result.positions.insert(result.positions.end(), mesh.positions.begin(), mesh.positions.end());
        for (const Face& face : mesh.faces) {
            Face shifted;
            shifted.corners.reserve(face.corners.size());
            for (int corner : face.corners) shifted.corners.push_back(corner + firstIndex);
            result.faces.push_back(std::move(shifted));
        }
    }
    return result;
}

std::vector<Vector3> Mesh::vertexNormals() const {
    std::vector<Vector3> sums(positions.size(), Vector3(0, 0, 0));
    for (const Face& face : faces) {
        Vector3 newell = polygon(face).newellVector();
        for (int corner : face.corners) sums[corner] += newell;
    }
    for (Vector3& sum : sums) sum = sum.normalized();
    return sums;
}

Mesh Mesh::displaced(const std::function<float(Vector3)>& amount) const {
    std::vector<Vector3> normals = vertexNormals();
    Mesh result;
    result.positions.reserve(positions.size());
    for (size_t point = 0; point < positions.size(); ++point) {
        result.positions.push_back(positions[point] + normals[point] * amount(positions[point]));
    }
    result.faces = faces;
    return result;
}

Mesh Mesh::taubinSmoothed(int iterations, float shrinkFactor, float inflateFactor) const {
    checkIterations(iterations);
    if (!(shrinkFactor > 0 && shrinkFactor <= 1)) {
        throw GeometryProblem("The shrink factor must be in (0, 1], but is " + numberText(shrinkFactor) + ".");
    }
    if (!(inflateFactor < -shrinkFactor)) {
        throw GeometryProblem("The inflate factor must be below minus the shrink factor, but is " + numberText(inflateFactor) + ".");
    }
    return umbrellaSteps(*this, iterations, {shrinkFactor, inflateFactor});
}

Mesh Mesh::laplacianSmoothed(int iterations, float factor) const {
    checkIterations(iterations);
    if (!(factor > 0 && factor <= 1)) {
        throw GeometryProblem("The smoothing factor must be in (0, 1], but is " + numberText(factor) + ".");
    }
    return umbrellaSteps(*this, iterations, {factor});
}

std::vector<Triangle> Mesh::triangles() const {
    std::vector<Triangle> result;
    for (const Face& face : faces) {
        if (face.corners.size() < 3) continue;
        for (size_t index = 1; index + 1 < face.corners.size(); ++index) {
            result.push_back(Triangle{positions[face.corners[0]], positions[face.corners[index]], positions[face.corners[index + 1]]});
        }
    }
    return result;
}

float signedTetrahedronVolume(const Triangle& triangle, Vector3 apex) {
    return (triangle.first - apex).dot((triangle.second - apex).cross(triangle.third - apex)) / 6;
}

float Mesh::signedVolume() const {
    if (positions.empty()) return 0;
    Vector3 apex = positions[0];
    float total = 0;
    for (const Triangle& triangle : triangles()) total = total + signedTetrahedronVolume(triangle, apex);
    return total;
}

} // namespace editor
