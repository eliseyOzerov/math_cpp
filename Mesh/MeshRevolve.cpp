// Revolve: a 2D outline spun around the Z axis (Engine/Geometry/Mesh+Revolve.swift). The doc
// comments are on the declarations in Mesh.h.
#include <string>

#include "Mesh.h"

namespace editor {

std::optional<Mesh::Face> Mesh::Face::droppingRepeatedCorners(const std::vector<int>& corners) {
    std::vector<int> kept;
    for (size_t index = 0; index < corners.size(); ++index) {
        size_t previous = index == 0 ? corners.size() - 1 : index - 1;
        if (corners[index] != corners[previous]) kept.push_back(corners[index]);
    }
    if (kept.size() < 3) return std::nullopt;
    return Face{kept};
}

Mesh Mesh::revolve(const std::vector<Vector2>& profile, int segments, Angle sweep) {
    int profileCount = static_cast<int>(profile.size());
    if (profileCount < MeshOperationLimits::minimumRevolveProfilePoints || profileCount > MeshOperationLimits::maximumRevolveProfilePoints) {
        throw GeometryProblem(
            "A revolved profile needs " + std::to_string(MeshOperationLimits::minimumRevolveProfilePoints) + "..."
            + std::to_string(MeshOperationLimits::maximumRevolveProfilePoints) + " points, but has " + std::to_string(profileCount) + ".");
    }
    for (Vector2 corner : profile) {
        if (!(std::isfinite(corner.x) && corner.x >= 0 && std::isfinite(corner.y))) {
            throw GeometryProblem("A revolved profile needs finite points with a radius of 0 or more.");
        }
    }
    if (!(sweep.radians > 0 && sweep.radians <= Angle::degrees(360).radians)) {
        throw GeometryProblem("A revolve sweeps more than 0° and at most 360°.");
    }
    bool isFullTurn = sweep.radians >= Angle::degrees(360).radians;
    if (segments < (isFullTurn ? MeshOperationLimits::minimumRevolveSegmentsForFullTurn : 1)) {
        throw GeometryProblem(
            "A full revolve needs at least " + std::to_string(MeshOperationLimits::minimumRevolveSegmentsForFullTurn)
            + " segments, a partial one at least 1.");
    }
    if (segments > MeshOperationLimits::maximumRevolveSegments) {
        throw GeometryProblem(
            "A revolve makes at most " + std::to_string(MeshOperationLimits::maximumRevolveSegments) + " segments, but got "
            + std::to_string(segments) + ".");
    }
    // A full turn reuses the first angle for the last; a partial turn needs one more column.
    int columnCount = isFullTurn ? segments : segments + 1;

    Mesh mesh;
    std::vector<int> firstPoint;
    for (Vector2 corner : profile) {
        firstPoint.push_back(static_cast<int>(mesh.positions.size()));
        if (corner.x == 0) {
            mesh.positions.push_back(Vector3(0, 0, corner.y));
        } else {
            for (int column = 0; column < columnCount; ++column) {
                Angle angle = sweep * (static_cast<float>(column) / static_cast<float>(segments));
                mesh.positions.push_back(Vector3(corner.x * angle.cosine(), corner.x * angle.sine(), corner.y));
            }
        }
    }
    // The point of profile point `row` at angle number `column`; an axis point has just one.
    auto pointNumber = [&](int row, int column) {
        return profile[row].x == 0 ? firstPoint[row] : firstPoint[row] + column % columnCount;
    };

    for (int row = 0; row < profileCount - 1; ++row) {
        for (int column = 0; column < segments; ++column) {
            std::vector<int> quad = {
                pointNumber(row, column), pointNumber(row, column + 1),
                pointNumber(row + 1, column + 1), pointNumber(row + 1, column),
            };
            // Next to the axis two corners coincide: the quad becomes a triangle; with both
            // rows on the axis nothing is left and there is no face.
            if (std::optional<Face> face = Face::droppingRepeatedCorners(quad)) mesh.faces.push_back(*face);
        }
    }
    return mesh;
}

} // namespace editor
