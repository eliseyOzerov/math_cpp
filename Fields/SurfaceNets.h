// Surface extraction: turning a distance field back into a triangle mesh. A signed distance
// field only says "how far"; a mesh is what the GPU draws and what other tools read. Naive
// surface nets finds the places where the field crosses zero on a grid and joins them
// (Engine/Math/Fields/SurfaceNets.swift). `DistanceField::extractSurface` is defined in
// SurfaceNets.cpp.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

#include "../Mesh/Mesh.h"
#include "../Geometry/Triangle.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A triangle mesh: corner positions and, three at a time, the indices of the corners of each
/// triangle. Extracted triangles are counterclockwise seen from outside, so their
/// `Triangle::normal` points out of the shape.
///
/// Use when: the result of turning a field into geometry; handing a surface to the renderer
/// Tags: mesh, triangle mesh, indexed mesh, vertices and indices, index buffer
/// Node: inside fieldToMesh
struct TriangleMesh {
    std::vector<Vector3> positions;
    /// Three indices into `positions` per triangle.
    std::vector<std::uint32_t> triangleIndices;

    bool operator==(const TriangleMesh&) const = default;

    /// How many triangles the mesh has: `triangleIndices.count / 3`.
    ///
    /// Use when: looping over a mesh's faces; checking whether an extraction found a surface
    /// Tags: triangle count, face count, number of triangles, index count over three
    /// Node: inside fieldToMesh
    int triangleCount() const { return static_cast<int>(triangleIndices.size() / 3); }

    /// The triangle `index` (0 to `triangleCount − 1`) with its corners in stored order.
    ///
    /// Use when: reading a face of the mesh as a `Triangle` to get its normal or area
    /// Tags: get triangle, face of mesh, triangle at index
    /// Node: inside fieldToMesh
    Triangle triangle(int index) const;

    /// The volume enclosed by a closed mesh, positive when the triangles face outward.
    ///
    /// Source: the divergence theorem, as used for mesh volumes in Eberly, *Polyhedral Mass
    /// Properties* (2002).
    ///
    /// How: each triangle `(a, b, c)` makes a tetrahedron with the origin, whose signed volume
    /// is `a · (b × c) / 6`. Added over all triangles, the parts outside the mesh cancel and
    /// what remains is the enclosed volume:
    ///
    ///     volume = Σ a · (b × c) / 6
    ///
    /// Only meaningful for a closed mesh. The origin can be anywhere, but far from the mesh the
    /// terms are large and cancel, which costs Float precision (so translate the mesh near the
    /// origin first when it is a kilometre away).
    ///
    /// Use when: checking that an extracted mesh holds the volume of its shape; mass of a solid
    /// Tags: mesh volume, signed volume, divergence theorem, tetrahedra sum, enclosed volume
    /// Node: none (volume before conversion; measureMesh measures the mesh fieldToMesh makes)
    float enclosedVolume() const;

    /// The polygon mesh with the same points and the same triangles (Swift `Mesh.init(_:)` and
    /// `TriangleMesh.mesh`).
    ///
    /// How: the positions are copied unchanged, and every three indices of `triangleIndices`
    /// become one face with three corners, in the stored order. Nothing is merged or split, so
    /// the point count, the triangle count and the sharing of points between neighbouring
    /// triangles all stay as they were. Surface nets makes one point per grid cell and lets
    /// every triangle around it use that one point, so the result is already welded.
    ///
    /// Conventions: the triangles are counterclockwise seen from outside, which is the mesh's
    /// rule too, so the faces point out of the shape.
    ///
    /// Use when: drawing, shading, smoothing or exporting a surface that was extracted from a
    ///   distance field
    /// Tags: triangle mesh to mesh, extracted surface to mesh, surface nets result, convert,
    ///   indexed triangles to faces
    /// Node: inside fieldToMesh
    Mesh mesh() const;
};

/// The values of a narrow-band sampling and how many corners it evaluated.
///
/// Use when: reading the result of `SurfaceNetsGrid::narrowBandValues`
/// Tags: narrow band result, sampled corner count
/// Node: inside fieldToMesh
struct NarrowBandSample {
    /// The field at every lattice corner, x fastest.
    std::vector<float> values;
    /// How many corners the field was actually evaluated at.
    std::int64_t sampledCorners = 0;
};

/// The sampled lattice and the working state of one surface-nets run.
///
/// Use when: reading how `DistanceField::extractSurface` lays out its grid
/// Tags: surface nets grid, lattice, voxel grid, sampling grid
/// Node: inside fieldToMesh
class SurfaceNetsGrid {
public:
    using CellCounts = std::array<int, 3>;

    Vector3 origin;
    float cellSize;
    /// Cells along x, y and z. The lattice has one more corner than cells along each axis.
    CellCounts cellCounts;

    /// Samples `field` at every lattice corner of a grid that starts at `origin`, with
    /// `cellCounts` cubic cells of `cellSize` along x, y and z.
    SurfaceNetsGrid(Vector3 origin, float cellSize, CellCounts cellCounts,
                    const std::function<float(Vector3)>& field);

    /// A grid whose corner values were already sampled (`narrowBandValues`), x fastest.
    SurfaceNetsGrid(Vector3 origin, float cellSize, CellCounts cellCounts, std::vector<float> values);

    /// Where lattice corner (x, y, z) of a grid starting at `origin` is:
    /// `origin + (x, y, z) × cellSize`. Every sampler uses this one formula, so they all sample
    /// the same points to the last bit.
    ///
    /// Use when: the position of a grid corner while sampling a field for surface nets
    /// Tags: lattice point, grid corner position, voxel corner
    /// Node: inside fieldToMesh
    static Vector3 cornerPosition(int x, int y, int z, Vector3 origin, float cellSize);

    /// The largest number of cells a grid may have in total: `2²⁴`.
    static constexpr int maximumCellCount = 1 << 24;

    /// How many cells of `cellSize` cover each length in `sizes`: the ratio rounded up, or to the
    /// nearest whole number when within `10⁻⁵` (relative) of it; at least 1.
    ///
    /// Why: see step 1 of `DistanceField::extractSurface`. Float division of a whole multiple can
    /// give 30.000002, and rounding that up would add a cell that only exists by rounding luck.
    /// A ratio that really is a hair above a whole number (`3.00001`) loses `10⁻⁵` of a cell at
    /// the far end of the grid, which does not matter for meshing.
    ///
    /// Edge cases: a ratio that is NaN or infinite, or a total above `maximumCellCount`, throws
    /// a `MathProblem` with a message; a negative size counts as 0.
    ///
    /// Use when: working out how many cells a box needs at a given cell size
    /// Tags: grid size, cell count, voxel count, ceil of ratio, round to whole number
    /// Node: inside fieldToMesh
    static CellCounts cellCountsForSize(const std::array<float, 3>& sizes, float cellSize);

    /// The field at every lattice corner, the way the first constructor samples it, but
    /// evaluated only near the surface and spread over every core. The meshes made from the two
    /// are identical.
    ///
    /// How:
    ///
    /// 1. **Classify.** The grid is split into leaf blocks of `NarrowBand::leafCells` cells, grouped
    ///    into top blocks of 16. Each top block's centre is sampled; if
    ///    `NarrowBand::surfaceCannotReach` says the surface misses the block, every leaf in it is
    ///    marked far, with that centre value. Otherwise the block is split into its 8 halves,
    ///    and so on down to the leaves, which are marked near when they too may hold surface.
    ///    Blocks at the grid's far edges stick out past it; the test covers the whole cube, so
    ///    it covers the part inside the grid too. Top blocks run in parallel, each writing only
    ///    its own leaves.
    /// 2. **Sample.** A corner touched by a near leaf (a corner on a leaf's face or edge touches up
    ///    to 8) gets the field's value at `cornerPosition`, the same position the full grid uses.
    ///    Any other corner gets the centre value of a far block it lies in. Lattice planes run in
    ///    parallel, each writing only its own corners.
    ///
    ///     · · · · · · · ·     far blocks (·) keep one centre sample;
    ///     · · ▪ ▪ ▪ · · ·     near leaves (▪) are sampled at every corner
    ///     · ▪ ▪ · ▪ ▪ · ·
    ///     · · ▪ ▪ ▪ · · ·
    ///
    /// Why the mesh is identical: a far block's corners all have the sign of its centre value
    /// (proof in `NarrowBand::surfaceCannotReach`), so the stand-in has the right sign. A cell
    /// with a vertex, and a lattice edge with a quad, has corners of both signs; the leaf that
    /// holds such a cell (or a cell round such an edge) is therefore near, and every corner it
    /// touches carries the true value. So every value surface nets interpolates is the true one,
    /// and every sign it tests is the true sign.
    ///
    /// Conventions: the field must be 1-Lipschitz or conservative with the right sign (see
    /// `NarrowBand::surfaceCannotReach`); on any other field, parts of the surface may be missed.
    /// `field` is called from several threads at once, so it must be pure: it may not change
    /// or read shared mutable state. When the grid is so far from the origin that rounding
    /// could flip a sign (`NarrowBand::roundingIsSmall` is false), every corner is sampled,
    /// still on every core.
    ///
    /// Use when: sampling a field for surface nets fast, for live editing
    /// Tags: narrow band sampling, sparse grid, parallel sampling, concurrentPerform, all cores
    /// Node: inside fieldToMesh
    static NarrowBandSample narrowBandValues(Vector3 origin, float cellSize, CellCounts cellCounts,
                                             const std::function<float(Vector3)>& field);

    /// The surface-nets mesh of the sampled grid: places the vertices, then joins them (steps 2
    /// and 3 of `DistanceField::extractSurface`).
    ///
    /// Use when: turning the sampled grid into triangles, once, after sampling
    /// Tags: build mesh, surface nets pass, polygonize grid
    /// Node: inside fieldToMesh
    TriangleMesh mesh();

private:
    /// The field at every lattice corner, x fastest.
    std::vector<float> values;
    /// For every cell, the index of its vertex in the mesh, or −1 if it has none.
    std::vector<std::int32_t> vertexOfCell;

    Vector3 cornerPosition(int x, int y, int z) const;
    float cornerValue(int x, int y, int z) const;
    int cellIndex(int x, int y, int z) const;
    static bool isInside(float value);
    static bool surfaceCrosses(float valueA, float valueB);
    static bool cellIsOnOneSide(const std::array<float, 8>& corners);
    void placeVertices(TriangleMesh& mesh);
    void joinVertices(TriangleMesh& mesh) const;
    static void addQuad(const std::array<std::uint32_t, 4>& quad, TriangleMesh& mesh);
};

} // namespace editor
