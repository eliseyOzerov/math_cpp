// A polygon mesh: points, and faces that list the points at their corners
// (Engine/Geometry/Mesh.swift and its extensions).
#pragma once

#include <cmath>
#include <concepts>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include "../Curves/Curve.h"
#include "../Geometry/Bounds.h"
#include "../Geometry/Ray.h"
#include "../Geometry/Triangle.h"
#include "../Linear/Transform.h"
#include "../Linear/Vector2.h"
#include "../Linear/GeometryProblem.h"
#include "MeshLimits.h"
#include "PlatonicSolid.h"
#include "../Linear/Vector3.h"
#include "Polygon.h"

namespace editor {

/// A point on a mesh's surface and the direction the surface faces there.
///
/// Use when: reading where a closest-point or raycast query landed on a mesh
/// Tags: surface point, hit point, point and normal, mesh sample
/// Node: none (value type)
struct MeshSurfacePoint {
    /// The point, on the surface.
    Vector3 position;
    /// The normal of the triangle the point is on: length 1, pointing out of the face's front.
    Vector3 normal;
    /// How far the point is from the query: from the given point, or along the ray.
    float distance;

    bool operator==(const MeshSurfacePoint&) const = default;
};

struct InsetResult;
struct ExtrudeResult;
struct RefinedSphere;

/// An edge between two points, the same whichever way it is walked: the smaller point index
/// is always `first`.
///
/// Use when: keying a crease by its edge; finding the edge two faces share
/// Tags: edge, undirected edge, point pair
/// Node: none (inside subdivide)
struct MeshEdge {
    int first;
    int second;

    /// The edge between points `one` and `other`, in either order.
    MeshEdge(int one, int other) : first(one < other ? one : other), second(one < other ? other : one) {}
    bool operator==(const MeshEdge&) const = default;

    /// The two points packed into one number, for hashing.
    std::uint64_t key() const {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(first)) << 32)
            | static_cast<std::uint32_t>(second);
    }
};

/// Hashes a `MeshEdge` by its packed points.
struct MeshEdgeHash {
    size_t operator()(const MeshEdge& edge) const { return std::hash<std::uint64_t>()(edge.key()); }
};

/// How sharp each creased edge stays under subdivision: the number of rounds it keeps its
/// line. Edges not listed are smooth (0).
using EdgeCreases = std::unordered_map<MeshEdge, float, MeshEdgeHash>;

/// A polygon mesh: a list of points and a list of faces. Each face lists the indices of its
/// corner points in order.
///
/// Conventions: Z up, right-handed, metres. Corners run counter-clockwise seen from outside,
/// so face normals point out. A face has 3 or more corners and may be any flat-ish polygon.
///
/// Use when: anything a modelling node makes or changes
/// Tags: mesh, polygon mesh, geometry, faces, vertices, polys
/// Node: none (the value the mesh nodes pass around)
struct Mesh {
    /// One face: the indices of its corner points, in order around the face.
    struct Face {
        std::vector<int> corners;
        bool operator==(const Face&) const = default;

        /// A face from `corners` with every corner that repeats the corner before it dropped, the
        /// list read as a ring (the first corner follows the last); nothing when fewer than 3 are left.
        ///
        /// Why: a sweep or revolve builds each face as a quad between two rings. A ring that
        /// collapses to one point (a tip on the curve, a pole on the axis) gives that point twice in
        /// a row. Dropping the repeat turns the quad into the triangle that meets there; a quad
        /// between two collapsed rings keeps 2 corners and is no face at all.
        ///
        ///     [7, 8, 3, 3]  ->  [7, 8, 3]        [3, 3, 9, 9]  ->  [3, 9]  ->  nothing
        ///
        /// How: keep corner `i` unless it equals corner `i - 1` (corner 0 is compared with the last).
        /// Only neighbours are compared: a corner that comes back later in the ring is kept.
        ///
        /// Use when: building faces between rings of points where a ring may collapse to a point
        /// Tags: degenerate face, collapse quad to triangle, remove duplicate corners, pole, tip,
        ///   fan at a point, cyclic duplicates
        /// Node: inside revolve, sweep
        static std::optional<Face> droppingRepeatedCorners(const std::vector<int>& corners);
    };

    std::vector<Vector3> positions;
    std::vector<Face> faces;

    bool operator==(const Mesh&) const = default;

    /// A box centred on the origin, `size` along x, y and z, with 6 quad faces.
    ///
    /// How: the 8 corners (±size/2 on each axis), bottom four then top four, counter-clockwise
    /// seen from above; the faces are +x, −x, +y, −y, top, bottom, each wound to face out.
    ///
    /// Conventions: every side of `size` must be above zero (the `box` node checks this).
    ///
    /// Use when: the starting block of a crate, a table top or a house
    /// Tags: box, cube, cuboid, primitive, rectangular prism
    /// Node: box
    static Mesh box(Vector3 size);

    /// The face as a polygon of its corner positions.
    ///
    /// Use when: measuring one face: its normal, centre or Newell vector
    /// Tags: face polygon, corners of face
    /// Node: none (inside the mesh nodes)
    Polygon polygon(const Face& face) const;

    /// Whether every face has 3 or more corners and every corner names an existing point.
    ///
    /// Use when: checking a mesh before an operation that indexes its points
    /// Tags: valid mesh, index check, sanity check
    /// Node: none (checking)
    bool isValid() const;

    /// Whether the mesh closes on itself with no holes: valid, not empty, and every side is
    /// walked as many times one way as the other.
    ///
    /// How: count each directed side (start → end) over all faces; the mesh is watertight when
    /// every side's count equals the count of its reverse.
    ///
    /// Use when: checking a model can be 3D-printed or filled with a volume
    /// Tags: watertight, closed, manifold, no holes, solid
    /// Node: planned (isWatertight)
    bool isWatertight() const;

    /// How many triangles the faces split into: corners − 2 for each face (a fan from its first
    /// corner).
    ///
    /// Use when: printing how heavy a model is to draw
    /// Tags: triangle count, tri count, polycount
    /// Node: none (reporting)
    size_t triangleCount() const;

    /// The box around all the points, or nothing for a mesh without points.
    ///
    /// Use when: printing a model's size; checking where it stands
    /// Tags: bounding box, AABB, extents, bounds
    /// Node: planned (bounds)
    std::optional<Bounds> bounds() const;

    /// Shrinks each selected face inside itself, leaving a frame of quads around it.
    ///
    /// How: for each selected face, in order, each side's inward direction is the face normal
    /// crossed with the side, normalised. Each corner moves by `insetCornerOffset` of the
    /// inward directions of the sides before and after it. The face is replaced by the inner
    /// loop (same face index), and one quad per side is appended, [corner, next corner, next
    /// inner, inner], in side order.
    ///
    /// Conventions: `selected` lists each face at most once (the `inset` node checks this);
    /// `thickness` is in metres and not negative. The inner faces keep their indices; the
    /// frame quads come after all existing faces.
    ///
    /// Use when: the frame around a crate panel; a window inset into a wall
    /// Tags: inset, inset faces, shrink face, frame, Blender inset
    /// Node: inset
    InsetResult inset(const std::vector<int>& selected, float thickness) const;

    /// How far to move a corner of an inset so both sides beside it move in by `thickness`.
    ///
    /// How: with unit inward directions a (side before) and b (side after), the corner moves
    /// along a + b, scaled by thickness / (1 + a · b). Along a, that offset reaches
    /// thickness·(1 + a·b)/(1 + a·b) = thickness, and the same along b, so both sides end up
    /// exactly `thickness` in. Sides that fold back on each other (a · b = −1) divide by zero.
    ///
    /// Use when: offsetting a polygon's corners inward by a fixed width
    /// Tags: miter offset, corner offset, polygon offset, inset corner
    /// Node: inside inset
    static Vector3 insetCornerOffset(Vector3 first, Vector3 second, float thickness);

    /// Pushes the selected faces out along their shared normal (in for a negative distance),
    /// adding a wall of quads along the outline of the selection.
    ///
    /// How: the walls stand on every side of a selected face whose other face is not selected
    /// (or that has none). Points inside the selection move by their summed face normals,
    /// normalised, times `distance`. Each outline point is copied once for every run of
    /// selected faces around it, moved along that run's summed Newell vectors; this keeps two
    /// selected regions that only touch at a corner apart. The walls are [start, end, moved
    /// end, moved start] for each outline side, and the selected faces are rewired to the
    /// moved points.
    ///
    /// Conventions: the mesh must build a `HalfEdgeMesh` (throws `GeometryProblem` if not);
    /// `selected` lists each face at most once. The top faces keep their indices.
    ///
    /// Use when: pushing a crate's panels in; raising a plate on a panel; a chimney from a roof
    /// Tags: extrude, extrude faces, push pull, Blender extrude region
    /// Node: extrude
    ExtrudeResult extruded(const std::vector<int>& selected, float distance) const;

    /// The mesh after `levels` rounds of Catmull–Clark subdivision, with creased edges kept
    /// sharp for as many rounds as their crease says.
    ///
    /// How: each round builds a `HalfEdgeMesh` and calls `HalfEdgeMesh::subdividedOnce`, which
    /// also hands down the creases of the halves of each creased edge.
    ///
    /// Conventions: `levels` is 0 or more; the mesh must be a clean surface (throws
    /// `GeometryProblem` if not).
    ///
    /// Use when: rounding a blocky model; softening every edge of a crate but keeping its lines
    /// Tags: Catmull-Clark, subdivision surface, subsurf, smooth, creases
    /// Node: subdivide
    Mesh subdivided(int levels, const EdgeCreases& creases = {}) const;

    // MARK: Primitives (Mesh+Primitives.swift, Mesh+Icosphere.swift)

    /// The turn of step `step` out of `count` equal steps around a full circle:
    /// `360° × step / count`. Used to place points evenly around an axis.
    ///
    /// Conventions: `count` > 0; step 0 is 0° and step `count` is a full 360°. Steps are
    /// counterclockwise seen from the tip of the axis.
    ///
    /// Use when: placing the corners of a circle profile, the slices of a cylinder or sphere
    /// Tags: equal steps around circle, angle of step, divide circle, regular polygon angle, segment angle
    /// Node: inside uvSphere, cylinder, cone, torus
    static Angle turn(int step, int count);

    /// A flat square in the xy plane, facing +Z, cut into a grid of squares. The only
    /// primitive that is not closed.
    ///
    ///     3 ── 4 ── 5          divisions = 2: 3 × 3 = 9 points, 4 quads
    ///     │    │    │          point (column, row) is number row × (divisions + 1) + column
    ///     0 ── 1 ── 2
    ///
    /// Counts: (divisions + 1)² points, divisions² quad faces. Area: `size²`.
    ///
    /// Conventions: `size` is the side length, the grid spans −size/2 … +size/2 in x and y at
    /// z = 0. `divisions` ≥ 1 squares per side. Throws `GeometryProblem` otherwise.
    ///
    /// Use when: a floor or ground plane; the starting sheet to bend into terrain or cloth
    /// Tags: grid, plane, floor, ground, quad grid, subdivided plane, primitive, sheet
    /// Node: grid
    static Mesh grid(float size, int divisions);

    /// A sphere cut like a globe: `segments` slices of longitude (around the Z axis) and
    /// `rings` bands of latitude (from the north pole at +Z to the south pole). Triangles at
    /// the two poles, quads in between.
    ///
    /// Counts: `segments × (rings − 1) + 2` points, `segments × rings` faces (`2 × segments`
    /// triangles and `segments × (rings − 2)` quads). With 4 segments and 2 rings it is an
    /// octahedron: area `4√3·r²`, volume `4/3·r³`. With more it approaches `4πr²` and
    /// `4/3·πr³` from below.
    ///
    /// Conventions: Z is the polar axis. `segments` ≥ 3, `rings` ≥ 2; throws `GeometryProblem`
    /// otherwise.
    ///
    /// Use when: a ball, a planet, a head blockout; a smooth-shading test shape
    /// Tags: UV sphere, sphere, ball, globe, latitude longitude, primitive, polar sphere
    /// Node: uvSphere
    static Mesh uvSphere(float radius, int segments, int rings);

    /// A sphere built from triangles of nearly equal size: an **icosphere** (also geodesic
    /// sphere). It starts as an icosahedron (20 triangles) and splits every triangle into 4 by
    /// joining the midpoints of its sides, `subdivisions` times, pushing each new point out onto
    /// the sphere.
    ///
    /// How: the icosahedron has a point at each pole (+Z and −Z) and two rings of 5 points at
    /// heights `±r/√5` and radius `2r/√5`, the lower ring turned by 36°. Each triangle's side
    /// midpoints are made **once**: the two triangles that share an edge look the midpoint up
    /// under the edge's two end numbers, so the result has no duplicate points and no cracks.
    /// The sphere is built with radius 1, where a new point is `normalize(a + b)` for the unit
    /// points `a` and `b`, and every point is multiplied by `radius` once at the end.
    ///
    /// Counts: `10·4ⁿ + 2` points and `20·4ⁿ` triangular faces for `n` subdivisions. `n = 0` is
    /// the icosahedron: 12 points, 20 faces, every edge `radius / sin 72° ≈ 1.0515 radius` long.
    ///
    /// Conventions: Z-up, centred on the origin, faces counterclockwise from outside. `radius`
    /// positive and finite; `subdivisions` from 0 to
    /// `MeshOperationLimits::maximumIcosphereSubdivisions` (throws `GeometryProblem` otherwise).
    ///
    /// Use when: a round ball with evenly sized faces (no pinched poles) to push into a rock, a planet or a crown of foliage
    /// Tags: icosphere, geodesic sphere, subdivided icosahedron, ico sphere, subdivision sphere, sphere without poles, Blender Ico Sphere
    /// Node: icosphere
    static Mesh icosphere(float radius, int subdivisions);

    /// A Platonic solid centred on the origin with every corner exactly `radius` from it (the
    /// solid's circumradius, so each one fits the same ball).
    ///
    /// How each one is placed (Z-up, `r` is `radius`):
    ///
    ///     tetrahedron   apex (0, 0, r); a face down at z = −r/3, its corners at radius
    ///                   r·√8/3 and azimuths 0°, 120°, 240° (r² = (r/3)² + 8r²/9)
    ///     cube          `box` with side 2r/√3: corners (±r/√3, ±r/√3, ±r/√3)
    ///     octahedron    the six points ±r on the axes
    ///     icosahedron   `icosphere` with 0 subdivisions: poles on ±Z, two rings of 5
    ///     dodecahedron  the icosahedron's dual: one corner on the ray through the centre of
    ///                   each of its 20 triangles, pushed out to r, and one pentagon around
    ///                   each of its 12 corners, through the triangles that meet there
    ///
    /// Sizes for circumradius `r` (Coxeter, *Regular Polytopes*, Table I), with edge `a`:
    ///
    ///     solid          points faces  edge a             inradius        volume
    ///     tetrahedron       4     4    r·√(8/3) ≈ 1.633r  r/3             a³/(6√2)
    ///     cube              8     6    2r/√3   ≈ 1.155r   r/√3            a³
    ///     octahedron        6     8    r·√2    ≈ 1.414r   r/√3            a³·√2/3
    ///     icosahedron      12    20    r/sin72° ≈ 1.051r  0.7947r         a³·5(3 + √5)/12
    ///     dodecahedron     20    12    r(√5 − 1)/√3 ≈ 0.714r 0.7947r      a³·(15 + 7√5)/4
    ///
    /// Conventions: faces counterclockwise from outside, every face flat; `radius` positive
    /// and finite (throws `GeometryProblem` otherwise, where Swift traps).
    ///
    /// Use when: a die, a gem, a crystal; the coarsest stand-in for a ball; the seed shape a
    ///   sphere is refined from
    /// Tags: platonic solid, tetrahedron, cube, octahedron, icosahedron, dodecahedron,
    ///   regular polyhedron, circumradius, d4 d6 d8 d12 d20, Blender Add Mesh Extra Objects
    /// Node: platonicSolid
    static Mesh platonicSolid(PlatonicSolid solid, float radius);

    /// A **geodesic sphere** of frequency `divisions`: an icosahedron whose every triangle is
    /// cut into `divisions²` triangles by an even grid, each grid point pushed out onto the
    /// sphere (Kenner's Class I breakdown, *Geodesic Math and How to Use It*, 1976).
    ///
    /// How: a point of the grid on triangle (A, B, C) has whole-number weights `(k, i, j)` with
    /// `k + i + j = n`, `n` the divisions; it is the central projection of the flat point,
    /// `normalize(k·A + i·B + j·C)`, for unit corners, times `radius`:
    ///
    ///              C                 n = 3: weights along each side 0…3,
    ///             ╱ ╲                9 small triangles, 6 pointing up
    ///            ●───●               (corners P(i, j), P(i+1, j), P(i, j+1))
    ///           ╱ ╲ ╱ ╲              and 3 pointing down
    ///          ●───●───●             (P(i+1, j), P(i+1, j+1), P(i, j+1))
    ///         ╱ ╲ ╱ ╲ ╱ ╲
    ///        A───●───●───B
    ///
    /// A point on an edge or corner of the icosahedron is made once, named by its corners and
    /// weights in corner order, and its position is summed in that order. Points 0…11 are the
    /// icosahedron's corners, as `icosphere` places them; `divisions = 1` gives the icosahedron
    /// and `divisions = 2` the points of `icosphere` with 1 subdivision.
    ///
    /// Counts: `10n² + 2` points, `20n²` triangles, `30n²` edges.
    ///
    /// Conventions: Z-up, triangles counterclockwise from outside. `baseFaces` numbers the
    /// icosahedron's faces in `icosphere` order. `radius` positive and finite, `divisions` from
    /// 1 to `MeshOperationLimits::maximumSphereDivisions` (throws `GeometryProblem` otherwise,
    /// where Swift traps).
    ///
    /// Use when: a ball of near-equal triangles at any resolution; a geodesic dome; a sphere
    ///   whose 20 icosahedron panels are kept for colouring or seams
    /// Tags: geodesic sphere, geodesic polyhedron, class I geodesic, frequency, icosphere,
    ///   subdivided icosahedron, Fuller dome, 10n²+2
    /// Node: geodesicSphere
    static RefinedSphere geodesicSphere(float radius, int divisions);

    /// An **equiangular cube sphere**: a cube whose every face is cut into `divisions × divisions`
    /// quads at equal angles seen from the centre, each grid point pushed out onto the sphere
    /// (the "cubed sphere" of Ronchi, Iacono & Paolucci, 1996).
    ///
    /// How: each grid point has whole-number coordinates `(a, b, c)` on the cube's surface, each
    /// from −n to n in steps of 2 (`n` the divisions). Each coordinate becomes an angle
    /// `θ = 45° · a / n`, and the point is `(tan θa, tan θb, tan θc)` normalized, times
    /// `radius`; `tan 45°` is taken as exactly 1 and `tan(−θ)` as `−tan θ`, so the sphere is
    /// symmetric to the last bit. A point on a cube edge is made once, named by its coordinates.
    /// Points 0…7 are the cube's corners, as `platonicSolid(cube)` places them; `divisions = 1`
    /// gives that cube.
    ///
    /// Counts: `6n² + 2` points, `6n²` quads, `12n²` edges. The quads are not flat for `n ≥ 2`.
    ///
    /// Conventions: Z-up, quads counterclockwise from outside. `baseFaces` numbers the cube's
    /// faces in `box` order (+X, −X, +Y, −Y, +Z, −Z). `radius` positive and finite, `divisions`
    /// from 1 to `MeshOperationLimits::maximumSphereDivisions` (throws `GeometryProblem`
    /// otherwise, where Swift traps).
    ///
    /// Use when: a ball of quads with no poles (for subdivision, UV panels, a six-panel ball);
    ///   a planet or a sky sphere mapped face by face
    /// Tags: cube sphere, cubed sphere, equiangular cubed sphere, spherified cube, quad sphere,
    ///   quadsphere, gnomonic cube, Blender Quad Sphere, 6n²+2
    /// Node: cubeSphere
    static RefinedSphere cubeSphere(float radius, int divisions);

    /// A cylinder standing on the xy plane's centre, axis along Z, with a flat n-gon cap on
    /// each end.
    ///
    /// Counts: `2 × segments` points (bottom ring, then top ring), `segments + 2` faces.
    /// Volume: `½·segments·r²·sin(360°/segments)·height`; 4 segments gives a square prism of
    /// volume `2r²·height`.
    ///
    /// Conventions: spans −height/2 … +height/2 in z. `segments` ≥ 3; throws `GeometryProblem`
    /// otherwise.
    ///
    /// Use when: a column, a can, a wheel, a pipe; a shape that is smooth around and flat on the ends
    /// Tags: cylinder, tube, column, can, prism, capped cylinder, primitive
    /// Node: cylinder
    static Mesh cylinder(float radius, float height, int segments);

    /// A cone with its base on the plane z = −height/2 and its tip at z = +height/2, above
    /// the base's centre.
    ///
    /// Counts: `segments + 1` points (the base ring, then the tip), `segments + 1` faces
    /// (`segments` triangles up the side and one n-gon base). Volume: `⅓ × base area × height`.
    ///
    /// Conventions: `segments` ≥ 3; throws `GeometryProblem` otherwise.
    ///
    /// Use when: a spike, a roof, a funnel, a tree top; an arrow head
    /// Tags: cone, pyramid, spike, tip, capped cone, primitive
    /// Node: cone
    static Mesh cone(float radius, float height, int segments);

    /// A doughnut around the Z axis: a circle of radius `minorRadius` swept around a bigger
    /// circle of radius `majorRadius`.
    ///
    /// How: the point at angles (φ around Z, ψ around the tube) is
    /// `((R + r·cos ψ)·cos φ, (R + r·cos ψ)·sin φ, r·sin ψ)` with `R = majorRadius`,
    /// `r = minorRadius`; ψ = 0 is the outer equator, ψ = 90° the top.
    ///
    /// Counts: `majorSegments × minorSegments` points and the same number of quads. Area:
    /// `4π²·R·r`; volume: `2π²·R·r²` (both approached from below as segments grow).
    ///
    /// Conventions: `minorRadius` < `majorRadius`; both segment counts ≥ 3 (throws
    /// `GeometryProblem` otherwise).
    ///
    /// Use when: a ring, a tire, a bagel; a test shape with a hole
    /// Tags: torus, doughnut, donut, ring, tire, annulus, primitive
    /// Node: torus
    static Mesh torus(float majorRadius, float minorRadius, int majorSegments, int minorSegments);

    // MARK: Moving and combining (Mesh+Operations.swift)

    /// The mesh with every point moved by `matrix`, and the faces re-ordered if the matrix
    /// mirrors space.
    ///
    /// How: each point becomes `matrix · (x, y, z, 1)` (see `Matrix4::transformPoint`). A
    /// transform that includes a mirror (a negative scale on one or three axes, so the 3×3 part
    /// has a **negative determinant**) turns the shape inside out: a face that was
    /// counterclockwise seen from outside becomes clockwise. So when the determinant of the
    /// upper 3×3 part is below 0 every face's corner list is reversed, and the outside stays
    /// outside.
    ///
    /// Conventions: points are transformed as positions (translation applies). A matrix with
    /// determinant exactly 0 flattens the mesh and is not reversed. The original is unchanged.
    ///
    /// Use when: placing a part of a model where it belongs; mirroring one half of a symmetric model into the other
    /// Tags: transform mesh, apply matrix, move mesh, mirror mesh, flip normals on mirror, Object.transform, bake transform
    /// Node: transform
    Mesh transformed(const Matrix4& matrix) const;

    /// The mesh with every point moved by `transform`: scaled, then rotated, then translated.
    /// A negative scale mirrors, and the faces are re-ordered to match.
    ///
    /// Use when: placing a part of a model; positioning a branch mesh at its place on the trunk
    /// Tags: transform mesh, apply transform, place mesh, move rotate scale, bake transform
    /// Node: transform
    Mesh transformed(const Transform& transform) const;

    /// One mesh made of all of `meshes`, side by side with no connection between them.
    ///
    /// How: concatenate the point lists; the faces of the k-th mesh have their corner numbers
    /// raised by the number of points in the meshes before it. Faces keep their order. Points
    /// are not merged, even when two meshes touch.
    ///
    /// Conventions: an empty list gives an empty mesh.
    ///
    /// Use when: building a model from parts (trunk plus branches plus leaves) before it is drawn as one object
    /// Tags: join meshes, merge meshes, combine meshes, concatenate, append mesh, union of parts (no boolean), Join
    /// Node: join
    static Mesh joined(const std::vector<Mesh>& meshes);

    // MARK: Normals and displacement (Mesh+Operations.swift)

    /// The unit normal at every point: the direction the surface faces there, from all the faces
    /// that meet at the point.
    ///
    /// How: for each point, add the **Newell vectors** of the faces that use it (each a normal
    /// with length twice the face's area, see `Polygon::newellVector`) and normalise the sum:
    /// `normal(p) = normalize( Σ over faces f at p of area(f) · normal(f) )`. Weighting by area
    /// makes large faces count more.
    ///
    /// Conventions: one entry per point, in the same order. A point that no face uses, or whose
    /// faces cancel out exactly, gets the zero vector.
    ///
    /// Use when: pushing points outward along the surface; placing something on the surface standing up from it
    /// Tags: vertex normals, area weighted normals, smooth normals, point normals, per-vertex normal, Mesh.normals
    /// Node: inside displace, smooth
    std::vector<Vector3> vertexNormals() const;

    /// The mesh with every point moved along its normal by an amount that depends on where the
    /// point **was**.
    ///
    /// How: `p′ = p + normal(p) · amount(p)`, with `normal` from `vertexNormals` computed on the
    /// original mesh, and `amount` your function of the original position. Positive moves
    /// outward (for an outward-facing mesh), negative inward. Faces are unchanged.
    ///
    /// Conventions: a point with a zero normal does not move. Large amounts relative to the size
    /// of the faces can fold the surface over itself; the operation does not check.
    ///
    /// Use when: making a rock, a planet or a bumpy fruit from a sphere; inflating or shrinking a shape
    /// Tags: displace, displacement, displace along normal, noise displacement, inflate, push pull, Displace modifier, height map on mesh
    /// Node: displace
    Mesh displaced(const std::function<float(Vector3)>& amount) const;

    // MARK: Smoothing (Mesh+Smoothing.swift)

    /// The mesh with its noise and stair-steps smoothed away but its size kept, by **Taubin
    /// smoothing**.
    ///
    /// Source: Taubin, "A signal processing approach to fair surface design", SIGGRAPH 1995.
    ///
    /// How: one *iteration* is two steps of the **umbrella operator**, the vector from a point to
    /// the average of its neighbours, `Δp = (1/n) · Σ (neighbour − p)` over the n points joined
    /// to p by an edge: first a shrinking step `p ← p + λ·Δp` (λ > 0) and then an inflating step
    /// `p ← p + μ·Δp` (μ < 0). Within a step every point is moved using the positions from
    /// before the step. An iteration multiplies the wave of frequency k by
    /// `(1 − λk)(1 − μk)`, which is 1 at the pass-band frequency `1/λ + 1/μ`, so the gentle
    /// waves (the shape) pass and the fine ones (the noise) are damped.
    ///
    /// Conventions: the neighbours are the points joined by an edge of a face (consecutive
    /// corners). A point on a **boundary** (an edge used by one face only) stays where it is, and
    /// so does a point that no face uses. `iterations` is from 0 to
    /// `MeshSmoothing::maximumIterations`, `shrinkFactor` is in (0, 1], `inflateFactor` is below
    /// `−shrinkFactor`; anything else throws `GeometryProblem`. Only the positions move.
    ///
    /// Use when: smoothing a mesh made from a distance field or a voxel grid; removing noise from a scanned or displaced surface without making it smaller
    /// Tags: taubin smoothing, lambda mu smoothing, laplacian smoothing, mesh fairing, denoise mesh, shrink-free smoothing, umbrella operator, smooth modifier, relax, Blender smooth
    /// Node: smooth
    Mesh taubinSmoothed(int iterations, float shrinkFactor = MeshSmoothing::shrinkFactor,
                        float inflateFactor = MeshSmoothing::inflateFactor) const;

    /// The mesh smoothed by plain **Laplacian smoothing**: `iterations` steps of
    /// `p ← p + factor·Δp` with the umbrella operator of `taubinSmoothed`. It smooths as
    /// `taubinSmoothed` does but also shrinks the shape.
    ///
    /// Source: the textbook Laplacian smoothing; Taubin 1995 shows its shrinking.
    ///
    /// Conventions: as `taubinSmoothed` (boundary points stay); `factor` in (0, 1].
    ///
    /// Use when: comparing a smoothing against Taubin's; deliberately shrinking a blob
    /// Tags: laplacian smoothing, umbrella smoothing, average neighbours, relax mesh, shrinking smooth
    /// Node: none (taubinSmoothed is the node)
    Mesh laplacianSmoothed(int iterations, float factor = MeshSmoothing::shrinkFactor) const;

    // MARK: Measuring (Mesh.swift)

    /// The triangles the faces split into, each face as a fan from its first corner.
    ///
    /// Use when: measuring or picking against a mesh with code that works on triangles
    /// Tags: triangles, triangulate mesh, fan triangles, mesh to triangles
    /// Node: inside measureMesh, closestPoint, raycast
    std::vector<Triangle> triangles() const;

    /// The volume enclosed by a closed mesh, positive when the faces point outward.
    ///
    /// How: the sum of `signedTetrahedronVolume` over the fan triangles of every face
    /// (divergence theorem; Eberly 2002, Zhang & Chen 2001). The apex is the mesh's first point:
    /// the answer does not depend on it for a closed mesh, but a point on the mesh keeps the
    /// intermediate numbers small.
    ///
    /// Conventions: meaningful only for a closed mesh. An inside-out mesh gives a negative
    /// volume; a mesh with no points gives 0.
    ///
    /// Use when: computing the volume or mass of a model; detecting inside-out faces
    /// Tags: mesh volume, signed volume, enclosed volume, divergence theorem, volume of closed mesh
    /// Node: measureMesh
    float signedVolume() const;

    // MARK: Queries (Mesh+Queries.swift, Mesh.swift)

    /// Whether every side of every face is walked exactly once, and its reverse exactly once by
    /// the neighbouring face: no holes, consistent winding and no pinches.
    ///
    /// How: count the directed sides (corner to next corner) of every face; the mesh is closed
    /// when every side is used once and its reverse is used once.
    ///
    /// Conventions: an invalid mesh (see `isValid`) or one with no faces is not closed.
    ///
    /// Use when: checking that a mesh has a well-defined inside before computing its volume
    /// Tags: closed mesh, watertight, edge-manifold, holes, boundary edges, consistent winding
    /// Node: measureMesh
    bool isClosed() const;

    /// The total area of the faces, each from `Polygon::area`, added in face order.
    ///
    /// Use when: pricing paint or material for a model; checking that subdivision kept the area
    /// Tags: surface area, mesh area, total area, face area sum
    /// Node: measureMesh
    float surfaceArea() const;

    /// The point of the mesh's surface nearest to `point`, with the normal of the triangle it is
    /// on, or nothing for a mesh with no faces.
    ///
    /// How: `Triangle::closestPoint` on every fan triangle, keeping the nearest (Ericson,
    /// *Real-Time Collision Detection*, section 5.1.5). When two triangles are equally near, the
    /// first one in face order wins.
    ///
    /// Conventions: the surface is two-sided; a point inside a closed mesh finds the nearest wall.
    /// A degenerate triangle gives a zero normal.
    ///
    /// Use when: snapping a point onto a model's surface; measuring how far a part floats above a body
    /// Tags: closest point on mesh, nearest surface point, snap to surface, project onto mesh, proximity
    /// Node: closestPoint
    std::optional<MeshSurfacePoint> closestPoint(Vector3 point) const;

    /// Where `ray` first meets the mesh's surface, or nothing if it misses.
    ///
    /// How: a miss of the mesh's bounding box ends early; otherwise Moller-Trumbore on every fan
    /// triangle, keeping the nearest hit.
    ///
    /// Conventions: two-sided, so back faces are hit too, and a ray starting inside a closed mesh
    /// hits its wall from inside. A ray starting exactly on the surface hits it at distance 0.
    /// The normal is the hit triangle's, whichever side the ray came from.
    ///
    /// Use when: dropping an object onto a model from above; finding where a line of sight meets a wall
    /// Tags: raycast, ray mesh intersection, first hit, line of sight, drop to surface, pick
    /// Node: raycast
    std::optional<MeshSurfacePoint> firstHit(const Ray& ray) const;

    // MARK: Revolve and sweep (Mesh+Revolve.swift, Mesh+Sweep.swift)

    /// A solid of revolution (a **lathe**): a 2D outline spun around the Z axis. Vases, mugs,
    /// bottles, bowls, columns, chess pieces, a ball on a stalk.
    ///
    /// The profile is a list of points `(radius, height)`: how far from the Z axis, and how high.
    /// Imagine it drawn in the half-plane `y = 0, x >= 0` (x is the radius, z the height):
    ///
    ///      height z                 spin 360 degrees      profile (0,0) (1,0) (1,1) (0,1)
    ///        | 3 --- 2                 -->                revolves into a cylinder of radius 1,
    ///        | |     |                                    height 1, with a flat bottom and top
    ///        | 0 --- 1  > radius
    ///
    /// Method: ring `i` is the profile point `(r, h)` copied to angles `sweep * c / segments` for
    /// `c = 0 ...`: the points `(r cos phi, r sin phi, h)`. Between ring `i` and ring `i + 1`, one face
    /// for each pair of neighbouring angles `c, c + 1`:
    /// `[ring i at c, ring i at c+1, ring i+1 at c+1, ring i+1 at c]`. A full turn wraps the last
    /// angle back to the first, so there is no seam and no duplicated points. A partial turn
    /// (`sweep` below 360 degrees) stops at the last angle and leaves the two flat sides open.
    ///
    /// **A profile point with radius exactly 0 is on the axis (a pole)**: it becomes one shared
    /// point instead of `segments` copies, and the two quads touching it become triangles. So a
    /// profile that starts and ends on the axis gives a closed mesh with no degenerate faces.
    /// (A profile point with a tiny non-zero radius would make a sliver instead; use exact 0.)
    ///
    /// **Which way the profile runs.** For the faces to point outward, walk the outline with the
    /// solid on your left, in the (radius, height) plane with the radius to the right and height up.
    /// That is counterclockwise around the solid's cross-section: from the bottom pole outward
    /// along the bottom, up the outer side, and back inward along the top, as in the picture.
    /// The opposite direction gives an inside-out mesh (negative volume).
    ///
    /// Counts: for a full turn, `segments` points per off-axis profile point plus one per
    /// on-axis point. The unit-square profile gives a regular `segments`-gon prism of volume
    /// `(segments/2)*sin(2 pi/segments)` (approaching pi).
    ///
    /// Conventions: Z-up, right-handed; the angle grows counterclockwise seen from +Z, starting
    /// at +X. Radii must be finite and not negative; heights are any finite number.
    /// `profile` has 2 to `MeshOperationLimits::maximumRevolveProfilePoints` points; `segments` is
    /// at least 3 for a full turn, 1 for a partial one, and at most
    /// `MeshOperationLimits::maximumRevolveSegments`. `sweep` is in (0 degrees, 360 degrees]. Where
    /// Swift's precondition stops the program, this throws a `GeometryProblem` with the same sentence.
    ///
    /// **The profile is an open polyline**: its last point is not joined back to its first. To
    /// close the solid, start and end the profile on the axis (radius 0), as in the picture. An
    /// outline that never touches the axis (a ring, a doughnut's cross-section) cannot be closed
    /// this way: repeating its first point at the end gives two coincident rings joined by
    /// nothing, a seam where the surface is open, not a closed mesh.
    ///
    /// Use when: anything round and symmetric about an axis, drawn as a side outline: a vase, a
    ///   bowl, a lamp, a ball (with a half-circle profile), a column
    /// Tags: revolve, lathe, surface of revolution, spin, turned object, rotate profile around
    ///   axis, Screw modifier, revolve (CAD), solid of revolution
    /// Node: revolve
    static Mesh revolve(const std::vector<Vector2>& profile, int segments, Angle sweep = Angle::degrees(360));

    /// A closed ring of `segments` points on a circle of the given radius, as a profile for
    /// `sweep`: point `k` is at angle `360 degrees * k / segments`, so the ring runs counterclockwise
    /// from `(radius, 0)`.
    ///
    /// Formula: corner `k` is `radius * (cos phi, sin phi)` with `phi = Mesh::turn(k, segments)`.
    ///
    /// Conventions: `segments` from 3 to `MeshOperationLimits::maximumSweepProfilePoints`,
    /// `radius` finite and > 0. The profile is a regular polygon inscribed in
    /// the circle, so a sweep of it has area slightly under the true circle's.
    ///
    /// Use when: the cross-section of a tube, trunk, branch or pipe
    /// Tags: circle profile, round cross section, regular polygon profile, ring, tube profile
    /// Node: profileCircle
    static std::vector<Vector2> circleProfile(float radius, int segments);

    /// A solid made by dragging a flat ring (the **profile**) along a 3D curve, growing or
    /// shrinking it on the way: a pipe, a trunk, a branch, a horn.
    ///
    /// Method:
    ///
    /// 1. Take `samples` parameters spread **evenly over the curve's `parameterRange`** (not by arc
    ///    length: a curve that runs faster in one place gets farther-apart rings there).
    /// 2. At each, take the curve's **rotation-minimizing frame** (`Curve::rotationMinimizingFrames`,
    ///    Wang et al. 2008): a position, a tangent `T`, and a normal `N` and binormal `B`, with
    ///    `N x B = T`, that twist as little as possible along the curve, so the tube does not
    ///    corkscrew.
    /// 3. Place the profile point `(x, y)` at `position + (N*x + B*y) * scale(fraction)`, where
    ///    `fraction` runs from 0 at the first sample to 1 at the last. So **profile x goes along
    ///    the frame's normal and y along its binormal**.
    /// 4. Join neighbouring rings with one quad per profile edge:
    ///    `[ring i point j, ring i point j+1, ring i+1 point j+1, ring i+1 point j]`.
    ///
    ///         ring 0     ring 1     ring 2      N x B = T, so a profile that goes counterclockwise
    ///        +--*--+    +--*--+    +-*-+        in (x, y) goes counterclockwise seen from in front
    ///        |     | ===|     |====|   |        of the ring (looking back along -T), which makes
    ///        +-----+    +-----+    +---+        the quads' normals point out of the tube
    ///          ---------- T ---------->
    ///
    /// **A profile must be counterclockwise in its (x, y) plane and convex** (caps are n-gons,
    /// which the mesh requires to be convex); it is given as a list of corners without repeating
    /// the first at the end. A clockwise profile makes an inside-out mesh.
    ///
    /// **Ends.** Where `scale` is exactly 0 at the first or last sample the ring collapses to one
    /// point on the curve, and the quads next to it become triangles meeting there: a pointed tip
    /// without degenerate faces. Where it is positive and `capped` is true, a flat n-gon cap closes
    /// the end (facing backwards along the curve at the start, forwards at the end). With `capped`
    /// false an end with positive scale stays open, so the mesh is closed only if both ends are
    /// tips or capped. (With only 2 samples and both ends tips there are no faces at all: the
    /// whole sweep is a line segment.)
    ///
    /// **Zero scale in the middle.** A ring with scale 0 between the ends also collapses to one
    /// point: the faces on both sides become triangles meeting there, a **pinch** where the
    /// surface touches itself at a single point. The mesh stays closed and has no degenerate
    /// faces, but the pinch point is shared by two cones of faces, so it is not a manifold there.
    ///
    /// Counts (profile of `n` points, `samples` = `s`, no tips): `n*s` points, `n*(s - 1)` quads plus
    /// 2 caps. A straight sweep of a regular polygon is a prism: for a circle profile of radius `r`
    /// over a curve of length `h` the volume is `(n/2)*r^2*sin(2 pi/n)*h`.
    ///
    /// Conventions: Z-up, right-handed. `samples` from 2 to `MeshOperationLimits::maximumSweepSamples`,
    /// a profile of 3 to `MeshOperationLimits::maximumSweepProfilePoints` points. `scale` must
    /// return finite non-negative numbers. `up` is the direction the first frame's normal leans
    /// toward (the frame is perpendicular to the curve, so it is the closest perpendicular to
    /// `up`). The curve may stop at a sample (zero velocity, as at the end of a Bezier whose
    /// handle sits on its end point): the frame there takes the direction the curve moves in
    /// just beside it (`Curve::unitTangent`). Where Swift's precondition stops the program, this
    /// throws a `GeometryProblem` with the same sentence.
    ///
    /// Use when: a trunk, a branch, a pipe, a handle, a rope, a road edge; anything that has a
    ///   cross-section and a path
    /// Tags: sweep, extrude along path, pipe, tube, loft along curve, rotation minimizing frame,
    ///   Skin modifier, Curve to Mesh, tapered sweep, Bevel object
    /// Node: sweep
    template <typename PathCurve>
    static Mesh sweep(
        const std::vector<Vector2>& profile, const PathCurve& curve, int samples, const std::function<float(float)>& scale,
        bool capped, Vector3 up = Vector3::zAxis())
        requires std::same_as<typename PathCurve::Point, Vector3>;

    /// A round tube along a 3D curve whose radius can change along the way: a branch that
    /// thins toward its tip, a pipe, a horn. A `sweep` of a circle profile.
    ///
    /// The ring at fraction `f` along the curve is a regular polygon of `segments` corners and
    /// radius `radius(f)` (the profile is the unit circle; the scale function is the radius). A
    /// radius of exactly 0 at an end makes a pointed tip, otherwise the end is closed with a flat
    /// cap. The tube is always closed.
    ///
    /// Conventions: see `sweep`. `segments` from 3 and `samples` from 2, up to the
    /// `MeshOperationLimits` maxima; `radius` finite and not negative.
    ///
    /// Use when: tree trunks and branches (`radius: { 0.2 * (1 - $0) }` is a cone-like branch
    ///   ending in a point), cables, pipes, tentacles
    /// Tags: tube, tapered tube, branch, pipe, cylinder along curve, trunk, tapering,
    ///   Curve bevel, tube geometry
    /// Node: curveToMesh
    template <typename PathCurve>
    static Mesh tube(const PathCurve& curve, const std::function<float(float)>& radius, int segments, int samples)
        requires std::same_as<typename PathCurve::Point, Vector3>;

    /// Throws the `GeometryProblem` of `sweep` if the profile has too few or too many corners or
    /// the sweep too few or too many samples.
    ///
    /// Use when: the first lines of a sweep, before it reads the curve
    /// Tags: sweep limits, profile size check, sample count check
    /// Node: inside sweep
    static void checkSweepSize(size_t profilePointCount, int samples);

    /// The solid of a sweep, once the frames and scales are known: the rings, the quads between
    /// them and the caps (steps 3 and 4 of `sweep`). Not generic, so the template above stays small.
    ///
    /// Use when: `sweep` has placed its frames and evaluated its scales
    /// Tags: sweep rings, tube faces, caps, tips
    /// Node: inside sweep
    static Mesh sweepAlongFrames(
        const std::vector<Vector2>& profile, const std::vector<CurveFrame>& frames, const std::vector<float>& scales,
        bool capped);
};

template <typename PathCurve>
Mesh Mesh::sweep(
    const std::vector<Vector2>& profile, const PathCurve& curve, int samples, const std::function<float(float)>& scale,
    bool capped, Vector3 up)
    requires std::same_as<typename PathCurve::Point, Vector3>
{
    checkSweepSize(profile.size(), samples);
    ParameterRange parameterRange = curve.parameterRange();
    std::vector<float> fractions;
    std::vector<float> parameters;
    for (int sample = 0; sample < samples; ++sample) {
        fractions.push_back(static_cast<float>(sample) / static_cast<float>(samples - 1));
        parameters.push_back(editor::linearBlend(parameterRange.lowerBound, parameterRange.upperBound, fractions.back()));
    }
    std::vector<CurveFrame> frames = curve.rotationMinimizingFrames(parameters, up);
    std::vector<float> scales;
    for (float fraction : fractions) scales.push_back(scale(fraction));
    for (float value : scales) {
        if (!(std::isfinite(value) && value >= 0)) throw GeometryProblem("A sweep's scale must be finite and not negative.");
    }
    return sweepAlongFrames(profile, frames, scales, capped);
}

template <typename PathCurve>
Mesh Mesh::tube(const PathCurve& curve, const std::function<float(float)>& radius, int segments, int samples)
    requires std::same_as<typename PathCurve::Point, Vector3>
{
    return sweep(circleProfile(1, segments), curve, samples, radius, true);
}

/// The signed volume of the tetrahedron formed by `triangle` and the point `apex`. Positive when
/// the triangle faces away from the apex (Engine/Geometry/Triangle+Volume.swift).
///
/// How: `(first − apex) · ((second − apex) × (third − apex)) / 6`. The triple product is the
/// volume of the parallelepiped spanned by three edges, and a tetrahedron is one sixth of it.
/// It is positive when the three edges, in order, are right-handed, which for counterclockwise
/// corners means the apex sees the triangle's back. Summed over every triangle of a closed,
/// outward-facing mesh it gives the volume it encloses, for any apex.
///
/// Conventions: the default apex is the origin.
///
/// Use when: computing the volume of a closed mesh; deciding whether a mesh is inside out
/// Tags: signed volume, tetrahedron volume, triple product, divergence theorem, mesh volume, enclosed volume
/// Node: inside measureMesh
float signedTetrahedronVolume(const Triangle& triangle, Vector3 apex = Vector3(0, 0, 0));

/// What an inset makes: the mesh, the inner faces (the selected indices) and the frame quads.
struct InsetResult {
    Mesh mesh;
    std::vector<int> inner;
    std::vector<int> frame;
};

/// What an extrude makes: the mesh, the moved faces (the selected indices) and the walls.
struct ExtrudeResult {
    Mesh mesh;
    std::vector<int> top;
    std::vector<int> sides;
};

/// A sphere refined from a base solid, with the base face every face came from: its
/// **region**. The faces of one region tile one face of the base solid pushed onto the sphere.
///
/// Use when: refining a seed solid into a ball while keeping its panels; colouring or
///   splitting a ball along its base faces
/// Tags: refined sphere, face regions, base face, panel id, face group, sphere panels
/// Node: geodesicSphere, cubeSphere
struct RefinedSphere {
    /// The sphere.
    Mesh mesh;
    /// For each face of `mesh`, in order, the number of the base solid's face it lies in.
    std::vector<int> baseFaces;

    bool operator==(const RefinedSphere&) const = default;
};

} // namespace editor
