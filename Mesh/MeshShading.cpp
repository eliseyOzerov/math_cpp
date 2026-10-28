#include "MeshShading.h"

#include <algorithm>

namespace editor {

namespace {

/// How close two normals' dot products may be to count as the same direction or as within the
/// smoothing angle. Absorbs the rounding of normalizing.
constexpr float normalTolerance = 1e-5f;

/// One use of a point: the face that uses it and that face's corner angle there.
struct PointUse {
    size_t face;
    float weight;
};

} // namespace

ShadedMesh shaded(const Mesh& mesh, Angle smoothingAngle) {
    if (!mesh.isValid()) {
        throw GeometryProblem("Only a valid mesh can be shaded: every face needs 3 or more corners that name existing points.");
    }
    std::vector<Vector3> faceNormals;
    faceNormals.reserve(mesh.faces.size());
    for (const Mesh::Face& face : mesh.faces) faceNormals.push_back(mesh.polygon(face).normal());
    float smoothingCosine = smoothingAngle.cosine();

    std::vector<std::vector<PointUse>> usesOfPoint(mesh.positions.size());
    for (size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex) {
        const Mesh::Face& face = mesh.faces[faceIndex];
        Polygon polygon = mesh.polygon(face);
        for (size_t cornerIndex = 0; cornerIndex < face.corners.size(); ++cornerIndex) {
            usesOfPoint[face.corners[cornerIndex]].push_back({faceIndex, polygon.cornerAngle(static_cast<int>(cornerIndex)).radians});
        }
    }

    ShadedMesh result;
    // The render vertices already made for each point, to find a corner's twin.
    std::vector<std::vector<size_t>> vertexIndicesOfPoint(mesh.positions.size());

    for (size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex) {
        const Mesh::Face& face = mesh.faces[faceIndex];
        Vector3 ownNormal = faceNormals[faceIndex];
        // A degenerate face has no direction of its own to compare with, so it accepts all.
        bool ownFaceIsDegenerate = ownNormal.length() == 0;
        std::vector<size_t> renderIndices;
        for (int point : face.corners) {
            Vector3 summed(0, 0, 0);
            for (const PointUse& use : usesOfPoint[point]) {
                Vector3 otherNormal = faceNormals[use.face];
                if (ownFaceIsDegenerate || ownNormal.dot(otherNormal) >= smoothingCosine - normalTolerance) {
                    summed += otherNormal * use.weight;
                }
            }
            Vector3 normal;
            if (summed.length() > 0) normal = summed.normalized();
            else normal = ownFaceIsDegenerate ? Vector3::zAxis() : ownNormal;

            const std::vector<size_t>& made = vertexIndicesOfPoint[point];
            auto existing = std::find_if(made.begin(), made.end(), [&](size_t vertex) {
                return result.vertices[vertex].normal.dot(normal) > 1 - normalTolerance;
            });
            if (existing != made.end()) {
                renderIndices.push_back(*existing);
            } else {
                result.vertices.push_back({mesh.positions[point], normal});
                vertexIndicesOfPoint[point].push_back(result.vertices.size() - 1);
                renderIndices.push_back(result.vertices.size() - 1);
            }
        }
        // The face's corners renumbered as render vertices, cut into a fan from the first corner.
        for (size_t index = 1; index + 1 < renderIndices.size(); ++index) {
            for (size_t corner : {renderIndices[0], renderIndices[index], renderIndices[index + 1]}) {
                result.indices.push_back(static_cast<std::uint32_t>(corner));
            }
        }
    }
    return result;
}

} // namespace editor
