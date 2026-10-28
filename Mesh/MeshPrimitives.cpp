// The basic shapes (Engine/Geometry/Mesh+Primitives.swift and Mesh+Icosphere.swift). All are
// Z-up and centred on the origin (the grid lies in the xy plane), and every face is listed
// counterclockwise as seen from outside, so `Mesh::signedVolume` is positive. Loops and sums
// follow the Swift engine, so both produce the same points.

#include <cmath>
#include <cstdint>
#include <map>
#include <string>

#include "../Linear/GeometryProblem.h"
#include "Mesh.h"

namespace editor {

namespace {

/// The point on the unit sphere halfway along the shorter great-circle arc between the unit
/// vectors `start` and `end`.
///
/// How: `normalize(start + end)`. The sum lies in the plane of the two vectors and, as the
/// diagonal of a rhombus with equal sides, bisects the angle between them; normalising puts it
/// back on the sphere.
///
/// Conventions: both inputs of length 1 and not opposite (their sum would be zero).
///
/// Use when: splitting an edge of a mesh that lies on a sphere; subdividing an icosphere
/// Tags: arc midpoint, great circle midpoint, slerp halfway, spherical midpoint, bisect angle
/// Node: inside icosphere
Vector3 arcMidpoint(Vector3 start, Vector3 end) { return (start + end).normalized(); }

/// The 12 points of an icosahedron of radius 1 with its poles on the Z axis.
///
/// How: the rings sit at polar angle `arctan 2 ≈ 63.43°` from the poles, where all edges come
/// out equal: ring height `r cos = r/√5`, ring radius `r sin = 2r/√5`. Every point is at
/// distance `√(r²/5 + 4r²/5) = r`.
///
/// Use when: starting an icosphere
/// Tags: icosahedron points, unit icosahedron
/// Node: inside icosphere
std::vector<Vector3> unitIcosahedronPoints() {
    float radius = 1;
    float ringHeight = radius / std::sqrt(5.0f);
    float ringRadius = 2 * ringHeight;
    auto ringPoint = [&](int step, Angle offset, float height) {
        Angle azimuth = Angle::degrees(72) * static_cast<float>(step) + offset;
        return Vector3(ringRadius * azimuth.cosine(), ringRadius * azimuth.sine(), height);
    };
    std::vector<Vector3> points = {Vector3(0, 0, radius)};
    for (int step = 0; step < 5; ++step) points.push_back(ringPoint(step, Angle::zero(), ringHeight));
    for (int step = 0; step < 5; ++step) points.push_back(ringPoint(step, Angle::degrees(36), -ringHeight));
    points.push_back(Vector3(0, 0, -radius));
    return points;
}

/// The 20 triangles of the icosahedron of `unitIcosahedronPoints`, counterclockwise from
/// outside: 5 around the north pole, 10 in the band (5 pointing down, 5 pointing up), 5 around
/// the south pole. The upper ring is points 1…5, the lower ring 6…10, the poles 0 and 11.
///
/// Use when: starting an icosphere
/// Tags: icosahedron faces
/// Node: inside icosphere
std::vector<std::vector<int>> icosahedronTriangles() {
    auto upper = [](int step) { return 1 + step % 5; };
    auto lower = [](int step) { return 6 + step % 5; };
    std::vector<std::vector<int>> triangles;
    for (int step = 0; step < 5; ++step) {
        triangles.push_back({0, upper(step), upper(step + 1)});
        triangles.push_back({upper(step), lower(step), upper(step + 1)});
        triangles.push_back({lower(step), lower(step + 1), upper(step + 1)});
        triangles.push_back({11, lower(step + 1), lower(step)});
    }
    return triangles;
}

} // namespace

Angle Mesh::turn(int step, int count) {
    return Angle::degrees(360) * (static_cast<float>(step) / static_cast<float>(count));
}

Mesh Mesh::grid(float size, int divisions) {
    if (!(size > 0)) throw GeometryProblem("A grid needs a positive size.");
    if (divisions < PrimitiveLimits::minimumGridDivisions) {
        throw GeometryProblem("A grid needs at least " + std::to_string(PrimitiveLimits::minimumGridDivisions) + " division.");
    }
    int pointsPerSide = divisions + 1;
    Mesh mesh;
    for (int row = 0; row < pointsPerSide; ++row) {
        for (int column = 0; column < pointsPerSide; ++column) {
            float x = linearBlend(-size / 2, size / 2, static_cast<float>(column) / static_cast<float>(divisions));
            float y = linearBlend(-size / 2, size / 2, static_cast<float>(row) / static_cast<float>(divisions));
            mesh.positions.push_back(Vector3(x, y, 0));
        }
    }
    for (int row = 0; row < divisions; ++row) {
        for (int column = 0; column < divisions; ++column) {
            int lowerLeft = row * pointsPerSide + column;
            int upperLeft = lowerLeft + pointsPerSide;
            mesh.faces.push_back(Face{{lowerLeft, lowerLeft + 1, upperLeft + 1, upperLeft}});
        }
    }
    return mesh;
}

Mesh Mesh::uvSphere(float radius, int segments, int rings) {
    if (!(radius > 0)) throw GeometryProblem("A sphere needs a positive radius.");
    if (segments < PrimitiveLimits::minimumSphereSegments) {
        throw GeometryProblem("A sphere needs at least " + std::to_string(PrimitiveLimits::minimumSphereSegments) + " segments.");
    }
    if (rings < PrimitiveLimits::minimumSphereRings) {
        throw GeometryProblem("A sphere needs at least " + std::to_string(PrimitiveLimits::minimumSphereRings) + " rings.");
    }
    Mesh mesh;
    mesh.positions.push_back(Vector3(0, 0, radius));
    for (int ring = 1; ring < rings; ++ring) {
        Angle polarAngle = Angle::degrees(180) * (static_cast<float>(ring) / static_cast<float>(rings));
        float ringRadius = radius * polarAngle.sine();
        float height = radius * polarAngle.cosine();
        for (int segment = 0; segment < segments; ++segment) {
            Angle azimuth = turn(segment, segments);
            mesh.positions.push_back(Vector3(ringRadius * azimuth.cosine(), ringRadius * azimuth.sine(), height));
        }
    }
    mesh.positions.push_back(Vector3(0, 0, -radius));

    int northPole = 0;
    int southPole = static_cast<int>(mesh.positions.size()) - 1;
    // The point on ring `ring` (1 is just below the north pole) at slice `segment`.
    auto point = [segments](int ring, int segment) { return 1 + (ring - 1) * segments + segment % segments; };

    for (int segment = 0; segment < segments; ++segment) {
        mesh.faces.push_back(Face{{northPole, point(1, segment), point(1, segment + 1)}});
    }
    for (int ring = 1; ring < rings - 1; ++ring) {
        for (int segment = 0; segment < segments; ++segment) {
            mesh.faces.push_back(Face{{point(ring + 1, segment), point(ring + 1, segment + 1), point(ring, segment + 1),
                                       point(ring, segment)}});
        }
    }
    for (int segment = 0; segment < segments; ++segment) {
        mesh.faces.push_back(Face{{southPole, point(rings - 1, segment + 1), point(rings - 1, segment)}});
    }
    return mesh;
}

Mesh Mesh::icosphere(float radius, int subdivisions) {
    if (!(std::isfinite(radius) && radius > 0)) throw GeometryProblem("An icosphere needs a positive radius.");
    if (subdivisions < 0 || subdivisions > MeshOperationLimits::maximumIcosphereSubdivisions) {
        throw GeometryProblem("An icosphere needs 0 to " + std::to_string(MeshOperationLimits::maximumIcosphereSubdivisions)
                              + " subdivisions, but got " + std::to_string(subdivisions) + ".");
    }
    std::vector<Vector3> positions = unitIcosahedronPoints();
    std::vector<std::vector<int>> triangles = icosahedronTriangles();

    for (int round = 0; round < subdivisions; ++round) {
        std::map<std::uint64_t, int> midpoints;
        // The number of the point halfway along the edge from `start` to `end`, on the sphere;
        // made the first time either neighbour asks.
        auto midpointNumber = [&](int start, int end) {
            std::uint64_t edge = MeshEdge(start, end).key();
            auto existing = midpoints.find(edge);
            if (existing != midpoints.end()) return existing->second;
            positions.push_back(arcMidpoint(positions[start], positions[end]));
            int number = static_cast<int>(positions.size()) - 1;
            midpoints[edge] = number;
            return number;
        };
        std::vector<std::vector<int>> splitTriangles;
        splitTriangles.reserve(triangles.size() * 4);
        for (const std::vector<int>& triangle : triangles) {
            int first = triangle[0], second = triangle[1], third = triangle[2];
            int firstSecond = midpointNumber(first, second);
            int secondThird = midpointNumber(second, third);
            int thirdFirst = midpointNumber(third, first);
            splitTriangles.push_back({first, firstSecond, thirdFirst});
            splitTriangles.push_back({second, secondThird, firstSecond});
            splitTriangles.push_back({third, thirdFirst, secondThird});
            splitTriangles.push_back({firstSecond, secondThird, thirdFirst});
        }
        triangles = std::move(splitTriangles);
    }
    Mesh mesh;
    for (Vector3 position : positions) mesh.positions.push_back(position * radius);
    for (const std::vector<int>& triangle : triangles) mesh.faces.push_back(Face{triangle});
    return mesh;
}

Mesh Mesh::cylinder(float radius, float height, int segments) {
    if (!(radius > 0 && height > 0)) throw GeometryProblem("A cylinder needs a positive radius and height.");
    if (segments < PrimitiveLimits::minimumCylinderSegments) {
        throw GeometryProblem("A cylinder needs at least " + std::to_string(PrimitiveLimits::minimumCylinderSegments) + " segments.");
    }
    Mesh mesh;
    for (float level : {-height / 2, height / 2}) {
        for (int segment = 0; segment < segments; ++segment) {
            Angle azimuth = turn(segment, segments);
            mesh.positions.push_back(Vector3(radius * azimuth.cosine(), radius * azimuth.sine(), level));
        }
    }
    for (int segment = 0; segment < segments; ++segment) {
        int next = (segment + 1) % segments;
        mesh.faces.push_back(Face{{segment, next, segments + next, segments + segment}});
    }
    Face top;
    for (int segment = 0; segment < segments; ++segment) top.corners.push_back(segments + segment);
    mesh.faces.push_back(top);
    Face bottom;
    for (int segment = segments - 1; segment >= 0; --segment) bottom.corners.push_back(segment);
    mesh.faces.push_back(bottom);
    return mesh;
}

Mesh Mesh::cone(float radius, float height, int segments) {
    if (!(radius > 0 && height > 0)) throw GeometryProblem("A cone needs a positive radius and height.");
    if (segments < PrimitiveLimits::minimumConeSegments) {
        throw GeometryProblem("A cone needs at least " + std::to_string(PrimitiveLimits::minimumConeSegments) + " segments.");
    }
    Mesh mesh;
    for (int segment = 0; segment < segments; ++segment) {
        Angle azimuth = turn(segment, segments);
        mesh.positions.push_back(Vector3(radius * azimuth.cosine(), radius * azimuth.sine(), -height / 2));
    }
    mesh.positions.push_back(Vector3(0, 0, height / 2));
    int tip = segments;

    for (int segment = 0; segment < segments; ++segment) {
        mesh.faces.push_back(Face{{segment, (segment + 1) % segments, tip}});
    }
    Face base;
    for (int segment = segments - 1; segment >= 0; --segment) base.corners.push_back(segment);
    mesh.faces.push_back(base);
    return mesh;
}

Mesh Mesh::torus(float majorRadius, float minorRadius, int majorSegments, int minorSegments) {
    if (!(minorRadius > 0 && minorRadius < majorRadius)) throw GeometryProblem("A torus needs 0 < minorRadius < majorRadius.");
    if (majorSegments < PrimitiveLimits::minimumTorusSegments || minorSegments < PrimitiveLimits::minimumTorusSegments) {
        throw GeometryProblem("A torus needs at least " + std::to_string(PrimitiveLimits::minimumTorusSegments)
                              + " segments each way.");
    }
    Mesh mesh;
    for (int majorStep = 0; majorStep < majorSegments; ++majorStep) {
        Angle around = turn(majorStep, majorSegments);
        for (int minorStep = 0; minorStep < minorSegments; ++minorStep) {
            Angle aroundTube = turn(minorStep, minorSegments);
            float distanceFromAxis = majorRadius + minorRadius * aroundTube.cosine();
            mesh.positions.push_back(Vector3(distanceFromAxis * around.cosine(), distanceFromAxis * around.sine(),
                                             minorRadius * aroundTube.sine()));
        }
    }
    // The point at cross-section `majorStep` and tube position `minorStep`, wrapping around.
    auto point = [majorSegments, minorSegments](int majorStep, int minorStep) {
        return (majorStep % majorSegments) * minorSegments + minorStep % minorSegments;
    };
    for (int majorStep = 0; majorStep < majorSegments; ++majorStep) {
        for (int minorStep = 0; minorStep < minorSegments; ++minorStep) {
            mesh.faces.push_back(Face{{point(majorStep, minorStep), point(majorStep + 1, minorStep),
                                       point(majorStep + 1, minorStep + 1), point(majorStep, minorStep + 1)}});
        }
    }
    return mesh;
}

} // namespace editor
