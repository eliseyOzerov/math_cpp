// A mesh with its connections worked out: for every side of every face, the face on the
// other side (Engine/Geometry/HalfEdgeMesh.swift).
#pragma once

#include <optional>
#include <vector>

#include "../Linear/Angle.h"
#include "Mesh.h"

namespace editor {

/// A mesh as half-edges: each face is a loop of directed sides, and each side knows its twin,
/// the same side walked the other way by the face next door.
///
/// Conventions: built from a `Mesh` whose faces are wound consistently. The build throws a
/// `GeometryProblem` for a face with fewer than 3 corners, a corner that names no point, a
/// point listed twice in one face, a side walked the same way by two faces (a flipped face or
/// more than two faces on one edge), and a point whose faces do not form one fan (a pinch).
///
/// Use when: walking around a point or across an edge; subdivision and extrusion
/// Tags: half-edge, DCEL, winged edge, mesh topology, adjacency
/// Node: none (inside extrude, subdivide)
struct HalfEdgeMesh {
    /// One directed side of a face.
    struct HalfEdge {
        int origin;
        int next;
        int previous;
        std::optional<int> twin;
        int face;
    };

    std::vector<Vector3> positions;
    std::vector<HalfEdge> halfEdges;
    std::vector<int> faceStart;
    std::vector<std::optional<int>> outgoing;

    /// The half-edges of `mesh`; throws `GeometryProblem` if it is not a clean surface.
    explicit HalfEdgeMesh(const Mesh& mesh);

    /// How many faces the mesh has.
    int faceCount() const { return static_cast<int>(faceStart.size()); }

    /// The point a half-edge runs to: the origin of the next one.
    ///
    /// Use when: the far end of an edge
    /// Tags: destination, end point, head, target vertex
    /// Node: none (topology)
    int destination(int halfEdge) const { return halfEdges[halfEdges[halfEdge].next].origin; }

    /// Whether the half-edge has no twin: it lies on a hole's edge.
    ///
    /// Use when: telling a border of an open mesh from an inside edge
    /// Tags: border, boundary edge, open edge, naked edge
    /// Node: none (topology)
    bool isBorder(int halfEdge) const { return !halfEdges[halfEdge].twin.has_value(); }

    /// The undirected edge a half-edge lies on.
    MeshEdge edge(int halfEdge) const { return MeshEdge(halfEdges[halfEdge].origin, destination(halfEdge)); }

    /// The first half-edge of `face` and one past its last, in the face's corner order.
    ///
    /// Use when: walking the sides of one face
    /// Tags: face loop, face half-edges, sides of face
    /// Node: none (topology)
    std::pair<int, int> halfEdgesOfFace(int face) const;

    /// The corner points of `face`, in order.
    std::vector<int> corners(int face) const;

    /// The half-edges leaving `point`, turning around it from one face to the next; on a border
    /// the walk starts at the border half-edge so it covers the whole fan.
    ///
    /// How: start at `outgoing[point]`, then repeatedly step to the twin of the previous
    /// half-edge (the next side out of the same point, one face over), until back at the start
    /// or a border stops the walk.
    ///
    /// Use when: the faces and neighbours around a point
    /// Tags: one-ring, vertex fan, around vertex, outgoing edges, umbrella
    /// Node: none (topology)
    std::vector<int> outgoingHalfEdges(int point) const;

    /// One half-edge per edge: the border ones, and of each twin pair the one with the
    /// smaller index.
    ///
    /// Use when: visiting each edge once, as subdivision does to make edge points
    /// Tags: edges, unique edges, edge list
    /// Node: none (topology)
    std::vector<int> edgeHalfEdges() const;

    /// The unit normal of `face` (see `Polygon::normal`).
    Vector3 faceNormal(int face) const;

    /// How sharply the surface bends across the edge of `halfEdge`: the angle between the
    /// normals of its two faces, or nothing on a border.
    ///
    /// Use when: finding the sharp edges to crease
    /// Tags: dihedral angle, edge angle, bend, crease angle
    /// Node: inside subdivide
    std::optional<Angle> bendAngle(int halfEdge) const;

    /// The edges whose faces bend more than `angle` across them.
    ///
    /// Use when: creasing the hard edges of a model before rounding it
    /// Tags: sharp edges, feature edges, auto smooth, crease by angle
    /// Node: inside subdivide
    std::vector<MeshEdge> edgesBendingMoreThan(Angle angle) const;

    /// The runs of selected faces around `point`, each run a list of consecutive faces of its
    /// fan that are all selected.
    ///
    /// How: on a closed fan, the walk starts just after the first unselected face, so a run that
    /// wraps around the start is not cut in two.
    ///
    /// Use when: extruding faces that meet at a point from two separate regions
    /// Tags: face runs, fan segments, selected wedges
    /// Node: inside extrude
    std::vector<std::vector<int>> selectedRuns(int point, const std::vector<bool>& isSelected) const;

    /// One round of Catmull–Clark subdivision (Catmull and Clark, "Recursively generated
    /// B-spline surfaces on arbitrary topological meshes", 1978), with semi-sharp creases
    /// (DeRose, Kass and Truong, "Subdivision Surfaces in Character Animation", 1998), and the
    /// creases of the next round.
    ///
    /// How: every face gets a face point (its centroid) and every edge an edge point; every old
    /// point moves (see `movedOldPoint`). Each half-edge of each face becomes one quad [origin,
    /// its edge point, the face point, the edge point of the previous half-edge]. New points
    /// are numbered: moved old points, then edge points in `edgeHalfEdges` order, then face
    /// points.
    ///
    /// An edge point is the midpoint of its edge if the edge is sharp (crease ≥ 1, or a border),
    /// else the smooth point (start + end + both face points) / 4, blended toward the midpoint
    /// by the crease. Each half of a creased edge keeps crease − 1.
    ///
    /// Use when: one level of the `subdivide` node
    /// Tags: Catmull-Clark, subdivision step, semi-sharp creases, subsurf
    /// Node: inside subdivide
    std::pair<Mesh, EdgeCreases> subdividedOnce(const EdgeCreases& creases) const;

private:
    /// Where an old point moves in one round of subdivision.
    ///
    /// How: with n neighbours, F the mean of the face points around it and M the mean of the
    /// midpoints of its edges, the smooth position is (F + 2M + (n − 3)·old) / n. A point with
    /// two sharp edges slides along them to (a + 6·old + b) / 8; a point with more than two, or
    /// a lone border corner, stays put. A crease below 1 blends smooth toward sharp by the mean
    /// crease of its sharp edges.
    Vector3 movedOldPoint(int point, const std::vector<Vector3>& facePoints, const EdgeCreases& creases) const;

    /// How sharp the edge of `halfEdge` is: infinite on a border, else its crease (0 if none).
    float sharpness(int halfEdge, const EdgeCreases& creases) const;
};

} // namespace editor
