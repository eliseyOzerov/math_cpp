// A mesh turned into what a renderer draws: vertices with a normal each, and triangles
// (Engine/Geometry/Mesh+Shading.swift). The core makes no pictures itself, but every host that
// draws a graph's mesh (the Unreal plugin, the Apple app) needs the same normals, so the rule that
// splits and blends them lives here once.
#pragma once

#include <cstdint>
#include <vector>

#include "../Linear/Angle.h"
#include "Mesh.h"

namespace editor {

/// One vertex as a renderer draws it: where it is and which way the surface faces there.
///
/// Use when: reading the vertices of a shaded mesh to fill a vertex buffer
/// Tags: render vertex, vertex with normal, GPU vertex, position and normal
/// Node: none (renderer shading)
struct MeshVertex {
    Vector3 position;
    /// Length 1.
    Vector3 normal;

    bool operator==(const MeshVertex&) const = default;
};

/// A mesh prepared for a renderer: render vertices (position and normal) and the triangles that
/// connect them, as three indices each.
///
/// Use when: handing a mesh to a renderer; checking what the GPU will receive
/// Tags: render mesh, index buffer data, vertex and index arrays, shaded mesh, GPU mesh data
/// Node: none (renderer shading)
struct ShadedMesh {
    std::vector<MeshVertex> vertices;
    /// Three indices per triangle, counterclockwise seen from outside.
    std::vector<std::uint32_t> indices;

    bool operator==(const ShadedMesh&) const = default;
};

/// The mesh as render vertices and triangle indices, with normals for shading.
///
/// A point of a mesh can need several normals: the corner of a box has three faces meeting at it
/// and wants a different normal for each (so the faces look flat), while a point on a sphere wants
/// one normal shared by all its faces (so it looks round). The **smoothing angle** chooses: a face
/// bends smoothly into a neighbor when the angle between their normals is at most the smoothing
/// angle.
///
/// For every corner of every face:
///
/// 1. Look at all faces that use the same point. Keep those whose normal is within
///    `smoothingAngle` of **this corner's own face** (`n · n′ ≥ cos(smoothingAngle)`).
/// 2. Add up their normals, each weighted by its own corner angle at the point, and normalize.
///    Weighting by angle makes the result independent of how the surface is cut up: a cube corner
///    with a quad on one side and two triangles on the other does not lean toward the side with
///    more faces (Thürmer & Wüthrich 1998).
/// 3. A corner whose normal equals one already made for the same point reuses that render vertex;
///    otherwise it gets a new one.
///
///     smoothingAngle 0°      30°, box          180°
///     flat everywhere        box stays flat    sphere fully round
///     (24 vertices on a box) (90° > 30°)
///
/// Each face is then split into triangles from its first corner, as `Mesh::triangles` does.
///
/// The rule compares every neighbor against the corner's **own face**. It does not follow edges
/// and it is not transitive: face A may accept B and B accept C while A rejects C. So where many
/// faces fan out around one point, each can end up with its own normal (a 32-segment cone at 30°
/// gets one render vertex per side face at the tip).
///
/// Conventions: `smoothingAngle` runs from 0° (flat) to 180° (fully smooth). The mesh must be
/// valid (see `Mesh::isValid`); an invalid one throws a `GeometryProblem`. A degenerate face (no
/// normal) contributes nothing to its neighbors, and its own corners accept every neighbor at the
/// point, so they get the neighbors' blended normal. If all faces at a point are degenerate the
/// normal is +Z, so a shader never normalizes a zero vector. A point whose faces cancel out falls
/// back to its own face's normal. Z up and right-handed like the mesh: a host with other axes
/// converts the result, not the mesh.
///
/// Use when: preparing a graph's mesh for drawing in a host; choosing flat or smooth shading per object
/// Tags: vertex normals, smooth shading, flat shading, smoothing angle, angle weighted normals,
///   shade smooth, split normals, index buffer, crease angle, auto smooth
/// Node: none (renderer shading)
ShadedMesh shaded(const Mesh& mesh, Angle smoothingAngle);

} // namespace editor
