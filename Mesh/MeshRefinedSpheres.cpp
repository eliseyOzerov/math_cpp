// Spheres refined from a base solid, with each face's base face
// (Engine/Geometry/Mesh+RefinedSpheres.swift).
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string>
#include <utility>

#include "../Linear/Angle.h"
#include "Mesh.h"

namespace editor {

namespace {

/// Throws unless `radius` and `divisions` are a sphere's; `name` is "geodesic sphere" or
/// "cube sphere".
void checkSphereSizes(const std::string& name, float radius, int divisions) {
    if (!(std::isfinite(radius) && radius > 0)) throw GeometryProblem("A " + name + " needs a positive radius.");
    if (!(divisions >= 1 && divisions <= MeshOperationLimits::maximumSphereDivisions)) {
        throw GeometryProblem("A " + name + " needs 1 to " + std::to_string(MeshOperationLimits::maximumSphereDivisions)
                              + " divisions, but got " + std::to_string(divisions) + ".");
    }
}

/// The tangent of the equiangular grid angle for cube coordinate `coordinate`:
/// `tan(45° · coordinate / divisions)`, exactly ±1 at the cube's faces, and minus the value of
/// its positive for a negative coordinate, so the grid is mirror-symmetric to the last bit.
float equiangularTangent(int coordinate, int divisions) {
    if (coordinate < 0) return -equiangularTangent(-coordinate, divisions);
    if (coordinate == divisions) return 1;
    return (Angle::degrees(45) * (static_cast<float>(coordinate) / static_cast<float>(divisions))).tangent();
}

/// A cube sphere grid point's coordinates.
using CubeCoordinates = std::array<int, 3>;

} // namespace

RefinedSphere Mesh::geodesicSphere(float radius, int divisions) {
    checkSphereSizes("geodesic sphere", radius, divisions);
    Mesh icosahedron = icosphere(1, 0);
    std::vector<Vector3> positions = icosahedron.positions;
    // A grid point's name: its corners with a weight above 0, in corner order, as
    // [corner, weight, corner, weight, …]. A corner of the icosahedron is [corner, n].
    std::map<std::vector<int>, int> pointNumbers;
    for (int corner = 0; corner < static_cast<int>(positions.size()); ++corner) pointNumbers[{corner, divisions}] = corner;
    auto pointNumber = [&](std::vector<std::pair<int, int>> weightedCorners) {
        std::erase_if(weightedCorners, [](const std::pair<int, int>& entry) { return entry.second <= 0; });
        std::stable_sort(weightedCorners.begin(), weightedCorners.end(),
                         [](const std::pair<int, int>& first, const std::pair<int, int>& second) { return first.first < second.first; });
        std::vector<int> name;
        for (const auto& [corner, weight] : weightedCorners) {
            name.push_back(corner);
            name.push_back(weight);
        }
        auto existing = pointNumbers.find(name);
        if (existing != pointNumbers.end()) return existing->second;
        Vector3 flatPoint(0, 0, 0);
        for (const auto& [corner, weight] : weightedCorners) flatPoint = flatPoint + icosahedron.positions[corner] * static_cast<float>(weight);
        positions.push_back(flatPoint.normalized());
        int number = static_cast<int>(positions.size()) - 1;
        pointNumbers[name] = number;
        return number;
    };

    std::vector<Face> faces;
    std::vector<int> baseFaces;
    faces.reserve(static_cast<size_t>(20 * divisions * divisions));
    for (int baseFace = 0; baseFace < static_cast<int>(icosahedron.faces.size()); ++baseFace) {
        const std::vector<int>& triangle = icosahedron.faces[baseFace].corners;
        int first = triangle[0], second = triangle[1], third = triangle[2];
        // P(i, j): weight i on the second corner, j on the third, the rest on the first.
        auto gridPoint = [&](int i, int j) { return pointNumber({{first, divisions - i - j}, {second, i}, {third, j}}); };
        for (int j = 0; j < divisions; ++j) {
            for (int i = 0; i < divisions - j; ++i) {
                faces.push_back(Face{{gridPoint(i, j), gridPoint(i + 1, j), gridPoint(i, j + 1)}});
                baseFaces.push_back(baseFace);
                if (i + j < divisions - 1) {
                    faces.push_back(Face{{gridPoint(i + 1, j), gridPoint(i + 1, j + 1), gridPoint(i, j + 1)}});
                    baseFaces.push_back(baseFace);
                }
            }
        }
    }
    Mesh mesh;
    for (Vector3 position : positions) mesh.positions.push_back(position * radius);
    mesh.faces = std::move(faces);
    return RefinedSphere{std::move(mesh), std::move(baseFaces)};
}

RefinedSphere Mesh::cubeSphere(float radius, int divisions) {
    checkSphereSizes("cube sphere", radius, divisions);
    Mesh cube = platonicSolid(PlatonicSolid::cube, 1);
    std::vector<Vector3> positions = cube.positions;
    // A cube corner's coordinates are ±n, the sign of each of its components.
    std::vector<CubeCoordinates> cornerCoordinates;
    for (Vector3 corner : cube.positions) {
        cornerCoordinates.push_back({corner.x > 0 ? divisions : -divisions, corner.y > 0 ? divisions : -divisions,
                                     corner.z > 0 ? divisions : -divisions});
    }
    std::map<CubeCoordinates, int> pointNumbers;
    for (int corner = 0; corner < static_cast<int>(cornerCoordinates.size()); ++corner) pointNumbers[cornerCoordinates[corner]] = corner;
    auto pointNumber = [&](const CubeCoordinates& coordinates) {
        auto existing = pointNumbers.find(coordinates);
        if (existing != pointNumbers.end()) return existing->second;
        Vector3 direction(equiangularTangent(coordinates[0], divisions), equiangularTangent(coordinates[1], divisions),
                          equiangularTangent(coordinates[2], divisions));
        positions.push_back(direction.normalized());
        int number = static_cast<int>(positions.size()) - 1;
        pointNumbers[coordinates] = number;
        return number;
    };

    std::vector<Face> faces;
    std::vector<int> baseFaces;
    faces.reserve(static_cast<size_t>(6 * divisions * divisions));
    for (int baseFace = 0; baseFace < static_cast<int>(cube.faces.size()); ++baseFace) {
        const std::vector<int>& square = cube.faces[baseFace].corners;
        // Walking from the first corner, one step towards the second corner (or the fourth)
        // changes one coordinate by 2: the corners' coordinates differ by 2n there.
        CubeCoordinates origin = cornerCoordinates[square[0]];
        CubeCoordinates along{}, across{};
        for (int axis = 0; axis < 3; ++axis) {
            along[axis] = (cornerCoordinates[square[1]][axis] - origin[axis]) / divisions;
            across[axis] = (cornerCoordinates[square[3]][axis] - origin[axis]) / divisions;
        }
        auto gridPoint = [&](int step, int row) {
            CubeCoordinates coordinates{};
            for (int axis = 0; axis < 3; ++axis) coordinates[axis] = origin[axis] + along[axis] * step + across[axis] * row;
            return pointNumber(coordinates);
        };
        for (int row = 0; row < divisions; ++row) {
            for (int step = 0; step < divisions; ++step) {
                faces.push_back(Face{{gridPoint(step, row), gridPoint(step + 1, row), gridPoint(step + 1, row + 1), gridPoint(step, row + 1)}});
                baseFaces.push_back(baseFace);
            }
        }
    }
    Mesh mesh;
    for (Vector3 position : positions) mesh.positions.push_back(position * radius);
    mesh.faces = std::move(faces);
    return RefinedSphere{std::move(mesh), std::move(baseFaces)};
}

} // namespace editor
