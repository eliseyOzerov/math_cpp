// The five Platonic solids as meshes (Engine/Geometry/Mesh+PlatonicSolids.swift).
#include <algorithm>
#include <cmath>
#include <map>

#include "Mesh.h"

namespace editor {

namespace {

/// The tetrahedron of `platonicSolid`: point 0 the apex, 1…3 the base counterclockwise
/// seen from above.
Mesh tetrahedron(float radius) {
    float baseHeight = -radius / 3;
    float baseRadius = radius * (std::sqrt(8.0f) / 3);
    Mesh mesh;
    mesh.positions.push_back(Vector3(0, 0, radius));
    for (int step = 0; step < 3; ++step) {
        Angle azimuth = Angle::degrees(120) * static_cast<float>(step);
        mesh.positions.push_back(Vector3(baseRadius * azimuth.cosine(), baseRadius * azimuth.sine(), baseHeight));
    }
    // The base is seen from below, so its corners run the other way round.
    mesh.faces = {Mesh::Face{{1, 3, 2}}, Mesh::Face{{1, 2, 0}}, Mesh::Face{{2, 3, 0}}, Mesh::Face{{3, 1, 0}}};
    return mesh;
}

/// The octahedron of `platonicSolid`: points 0…3 the equator (+X, +Y, −X, −Y), 4 and 5
/// the poles +Z and −Z.
Mesh octahedron(float radius) {
    Mesh mesh;
    mesh.positions = {
        Vector3(radius, 0, 0), Vector3(0, radius, 0), Vector3(-radius, 0, 0), Vector3(0, -radius, 0),
        Vector3(0, 0, radius), Vector3(0, 0, -radius),
    };
    for (int step = 0; step < 4; ++step) {
        int next = (step + 1) % 4;
        mesh.faces.push_back(Mesh::Face{{step, next, 4}});
        mesh.faces.push_back(Mesh::Face{{next, step, 5}});
    }
    return mesh;
}

/// One face around an old corner in `dualOnSphere`, and the point before the corner in it
/// (the key of the next face round).
struct FaceAroundCorner {
    int face;
    int previous;
};

/// The dual of a closed mesh whose corners lie on a sphere around the origin: a corner on
/// the ray through each face's centre, `radius` from the origin, and a face around each
/// old corner, through the faces that meet there.
///
/// How: face `f` becomes corner `f`. For an old corner, its faces are put in order going
/// counterclockwise around it seen from outside: the face that lists the corner followed by
/// `next` (and `previous` before it) is followed by the one that lists the corner followed by
/// `previous`. So the new face is counterclockwise from outside too.
///
/// Conventions: the mesh closed (every corner surrounded by a single fan of faces).
/// Pushing to the sphere keeps the new faces flat only for a regular solid.
Mesh dualOnSphere(const Mesh& mesh, float radius) {
    Mesh dual;
    for (const Mesh::Face& face : mesh.faces) dual.positions.push_back(mesh.polygon(face).centroid().normalized() * radius);
    // For each old corner: the faces around it, keyed by the point that follows the corner.
    std::vector<std::map<int, FaceAroundCorner>> facesAroundCorner(mesh.positions.size());
    for (size_t faceNumber = 0; faceNumber < mesh.faces.size(); ++faceNumber) {
        const std::vector<int>& corners = mesh.faces[faceNumber].corners;
        size_t count = corners.size();
        for (size_t place = 0; place < count; ++place) {
            int corner = corners[place];
            int next = corners[(place + 1) % count];
            int previous = corners[(place + count - 1) % count];
            facesAroundCorner[corner][next] = FaceAroundCorner{static_cast<int>(faceNumber), previous};
        }
    }
    for (const std::map<int, FaceAroundCorner>& around : facesAroundCorner) {
        // Start from the lowest-numbered face, as the Swift engine does.
        FaceAroundCorner step = std::min_element(around.begin(), around.end(), [](const auto& first, const auto& second) {
                                    return first.second.face < second.second.face;
                                })->second;
        std::vector<int> ring;
        while (ring.size() < around.size()) {
            ring.push_back(step.face);
            step = around.at(step.previous);
        }
        dual.faces.push_back(Mesh::Face{ring});
    }
    return dual;
}

} // namespace

Mesh Mesh::platonicSolid(PlatonicSolid solid, float radius) {
    if (!(std::isfinite(radius) && radius > 0)) throw GeometryProblem("A Platonic solid needs a positive radius.");
    switch (solid) {
    case PlatonicSolid::tetrahedron: return tetrahedron(radius);
    case PlatonicSolid::cube: {
        float side = 2 * radius / std::sqrt(3.0f);
        return box(Vector3(side, side, side));
    }
    case PlatonicSolid::octahedron: return octahedron(radius);
    case PlatonicSolid::icosahedron: return icosphere(radius, 0);
    case PlatonicSolid::dodecahedron: return dualOnSphere(icosphere(radius, 0), radius);
    }
    throw GeometryProblem("A Platonic solid needs a positive radius.");
}

} // namespace editor
