// Maps of the plane that deform a drawing as a whole: a perspective (projective) map between two
// quadrilaterals, a free-form lattice, and puppet pins solved as-rigid-as-possible. Each is a
// `Warp2D::PlaneMap` in the end, so `Warp2D::warpedCubic` carries curves through it.
//
// Three pieces of maths, each with its source:
//
// - Perspective: the homography between quadrilaterals, from the closed form of Heckbert, "Fundamentals
//   of Texture Mapping and Image Warping", M.Sc. thesis, UC Berkeley, 1989, section 2.2.
// - Lattice: a Bézier free-form deformation, Sederberg and Parry, "Free-form deformation of solid
//   geometric models", SIGGRAPH 1986.
// - Puppet: the local/global as-rigid-as-possible solver of Sorkine and Alexa, "As-rigid-as-possible
//   surface modeling", Symposium on Geometry Processing 2007, on a triangle mesh of the plane. Igarashi,
//   Moscovich and Hughes, "As-rigid-as-possible shape manipulation", SIGGRAPH 2005, introduced the
//   pin interface used here.
//
// Conventions: y-up plane, float interface; the solvers work in double inside.
#pragma once

#include <array>
#include <memory>
#include <vector>

#include "../Geometry/Bounds2.h"
#include "../Linear/Vector2.h"
#include "Warp2D.h"

namespace editor {

/// A projective map of the plane, the 3×3 matrix [a b c; d e f; g h 1] taking (x, y) to
/// ((a·x + b·y + c) ÷ w, (d·x + e·y + f) ÷ w) with w = g·x + h·y + 1. Straight lines stay straight
/// and four points go to any four points (no three in a line): a perspective view of a flat thing.
///
/// Use when: tilting a drawing as if seen at an angle; fitting a logo onto a quadrilateral
/// Tags: homography, perspective, projective transform, corner pin, keystone, four point warp
/// Node: planned (perspectiveWarp)
struct Homography {
    /// a, b, c, d, e, f, g, h in this order; the last entry of the matrix is 1.
    std::array<float, 8> entries = {1, 0, 0, 0, 1, 0, 0, 0};

    bool operator==(const Homography&) const = default;

    /// The map taking the unit square (0,0), (1,0), (1,1), (0,1) to the quadrilateral `corners` in that order.
    ///
    /// How (Heckbert 1989): with Δx1 = x1−x2, Δx2 = x3−x2, Δy1 = y1−y2, Δy2 = y3−y2 and
    /// Σx = x0−x1+x2−x3, Σy = y0−y1+y2−y3: when both sums are zero the quadrilateral is a
    /// parallelogram and the map is affine (g = h = 0, a = x1−x0, b = x2−x1, d = y1−y0, e = y2−y1);
    /// otherwise g = |Σx Δx2; Σy Δy2| ÷ |Δx1 Δx2; Δy1 Δy2|, h = |Δx1 Σx; Δy1 Σy| ÷ the same
    /// determinant, a = x1−x0+g·x1, b = x3−x0+h·x3, d = y1−y0+g·y1, e = y3−y0+h·y3; c = x0, f = y0.
    ///
    /// Conventions: a quadrilateral with three corners in a line has no such map (a `MathProblem`).
    ///
    /// Use when: the first half of `fromQuadrilaterals`; mapping a texture square onto a quad
    /// Tags: square to quad, unit square homography, Heckbert
    /// Node: planned (perspectiveWarp)
    static Homography squareToQuadrilateral(const std::array<Vector2, 4>& corners);

    /// The map taking the quadrilateral `from` onto `to`, corner for corner (counterclockwise from any
    /// corner, the same corner first in both).
    ///
    /// How: `from` goes to the unit square by the inverse of its square-to-quad map, then the unit
    /// square goes to `to`: M = squareToQuad(to) · squareToQuad(from)⁻¹.
    ///
    /// Use when: perspective warp of a selection; keystone correction
    /// Tags: quad to quad, four corners to four corners, corner pin, perspective transform
    /// Node: planned (perspectiveWarp)
    static Homography fromQuadrilaterals(const std::array<Vector2, 4>& from, const std::array<Vector2, 4>& to);

    /// Where the map takes `point`. A point on the line where w = 0 goes to infinity (non-finite result).
    ///
    /// Use when: moving points through a perspective
    /// Tags: apply homography, project point
    /// Node: planned (perspectiveWarp)
    Vector2 applied(Vector2 point) const;

    /// The map that undoes this one (the adjugate of the matrix, rescaled so its last entry is 1);
    /// a `MathProblem` when the map flattens the plane.
    ///
    /// Use when: mapping back from the quadrilateral; the first half of `fromQuadrilaterals`
    /// Tags: inverse homography, adjugate
    /// Node: planned (perspectiveWarp)
    Homography inverse() const;

    /// This map as a `Warp2D::PlaneMap`.
    ///
    /// Use when: warping curves through the map
    /// Tags: homography plane map
    /// Node: planned (perspectiveWarp)
    Warp2D::PlaneMap planeMap() const;
};

/// A free-form lattice: a grid of `columns` × `rows` control points over a box, and the smooth map
/// the points define. Moved apart from their rest places the points bend the plane; at rest the map
/// is the identity.
///
/// How (Sederberg and Parry 1986, in two dimensions): with u, v in 0…1 across the box, the image of
/// a point is Σ B(i, columns−1, u) · B(j, rows−1, v) · P(i, j) where B is the Bernstein polynomial
/// C(n,i) · uⁱ(1−u)ⁿ⁻ⁱ, evaluated by de Casteljau's algorithm. Control points at the evenly spaced
/// rest places give the identity, because the Bernstein basis reproduces linear functions (linear
/// precision). Outside the box the same polynomial continues.
///
/// Conventions: points run row by row from the bottom row, x first in each row; at least 2 columns and 2 rows.
///
/// Use when: envelope distort; bulging or pinching a drawing with a few handles
/// Tags: lattice, FFD, free form deformation, envelope warp, control grid, Bézier lattice
/// Node: planned (latticeWarp)
struct Lattice2D {
    Vector2 lowerCorner;
    Vector2 upperCorner;
    int columns = 2;
    int rows = 2;
    std::vector<Vector2> points;

    bool operator==(const Lattice2D&) const = default;

    /// The lattice at rest over the box: points at lower + (i/(columns−1) · width, j/(rows−1) · height).
    /// A `MathProblem` for fewer than 2 columns or rows, or a box with no area.
    ///
    /// Use when: starting a lattice warp before any point is moved
    /// Tags: undeformed lattice, rest lattice, identity lattice
    /// Node: planned (latticeWarp)
    static Lattice2D undeformed(Vector2 lowerCorner, Vector2 upperCorner, int columns, int rows);

    /// The rest place of control point `index`.
    ///
    /// Use when: showing the lattice; moving one control point from its rest
    /// Tags: lattice rest point, control point rest
    /// Node: planned (latticeWarp)
    Vector2 restPoint(int index) const;

    /// Where the lattice takes `point`. A `MathProblem` when `points` has not columns × rows entries.
    ///
    /// Use when: moving points through the lattice
    /// Tags: apply lattice, FFD evaluate
    /// Node: planned (latticeWarp)
    Vector2 applied(Vector2 point) const;

    /// This lattice as a `Warp2D::PlaneMap`.
    ///
    /// Use when: warping curves through the lattice
    /// Tags: lattice plane map
    /// Node: planned (latticeWarp)
    Warp2D::PlaneMap planeMap() const;
};

/// A triangle mesh over a box with pins in it, solved as-rigid-as-possible: dragging the pins to new
/// places moves the plane between them with as little stretching as it can, and every point of the
/// plane follows its triangle. The mesh depends on where the pins rest, so it is built once for a
/// set of pins and solved again for every drag.
///
/// How: the box is cut into a grid of right triangles (diagonals alternating); each pin splits the
/// triangle it lies in into three, so a mesh vertex sits exactly on it. A pin within 0.02 of an edge
/// (barycentric) is moved just inside, and one within 0.02 of a corner snaps to that corner (an error
/// of a fraction of a cell, never more). The energy (Sorkine and Alexa 2007, eq. 5) is
///
///     E = Σ over vertices i, Σ over neighbours j of  wᵢⱼ · ‖(pᵢ' − pⱼ') − Rᵢ (pᵢ − pⱼ)‖²
///
/// with cotangent weights wᵢⱼ = ½(cot α + cot β) (angles opposite the edge), clamped to 0…100 so a
/// sliver triangle cannot dominate. The solve alternates two exact steps: with the new positions
/// fixed, each Rᵢ is the best rotation, the angle atan2(M₀₁ − M₁₀, M₀₀ + M₁₁) of M = Σ wᵢⱼ eᵢⱼ e'ᵢⱼᵀ
/// (eᵢⱼ the rest edge, e'ᵢⱼ the new one); with the rotations fixed, the new positions solve the
/// sparse symmetric system Σⱼ wᵢⱼ (pᵢ' − pⱼ') = Σⱼ wᵢⱼ/2 (Rᵢ + Rⱼ) eᵢⱼ, pinned vertices held at their
/// targets, by conjugate gradients. The start is the single rigid motion that best takes the pins to
/// their targets (Procrustes), so pins moved rigidly give that rigid motion at once, and pins left
/// where they were give the identity.
///
/// Use when: puppet pins; bending a limb of a drawing while the rest stays rigid
/// Tags: as rigid as possible, ARAP, puppet warp, pins, mesh deformation, Sorkine Alexa, Igarashi
/// Node: planned (puppetWarp)
class PuppetMesh {
public:
    /// The mesh over `box` with about `cellsAlongLongerSide` cells along its longer side (at least 1),
    /// and a vertex at each of `pinRestPositions`. A `MathProblem` for a pin outside the box.
    ///
    /// Use when: starting a puppet warp once the pins are placed
    /// Tags: build puppet mesh, pin mesh
    /// Node: planned (puppetWarp)
    PuppetMesh(Bounds2 box, int cellsAlongLongerSide, const std::vector<Vector2>& pinRestPositions);

    int vertexCount() const { return static_cast<int>(vertexList.size()); }
    int triangleCount() const { return static_cast<int>(triangleList.size()); }
    int pinCount() const { return static_cast<int>(pinVertices.size()); }

    /// The vertices at rest.
    ///
    /// Use when: showing the mesh; the rest mesh for tests
    /// Tags: puppet mesh vertices
    /// Node: none (inspection)
    const std::vector<Vector2>& restVertices() const { return vertexList; }

    /// The vertices after the pins are dragged to `pinTargets` (one per pin, in the order given at
    /// construction), after `iterations` rounds of local and global steps (default 8, at least 0).
    /// Two pins on one vertex pull it to the mean of their targets. A `MathProblem` for a wrong number of targets.
    ///
    /// Use when: dragging a pin
    /// Tags: solve puppet, ARAP iterate, deform mesh
    /// Node: planned (puppetWarp)
    std::vector<Vector2> solved(const std::vector<Vector2>& pinTargets, int iterations = 8) const;

    /// Where `restPoint` goes when the vertices are at `solvedVertices`: its barycentric place in its
    /// triangle carried to the triangle's new corners. A point outside the mesh moves as the nearest
    /// vertex does.
    ///
    /// Use when: moving the drawing's points after a solve
    /// Tags: map point by mesh, barycentric map
    /// Node: planned (puppetWarp)
    Vector2 mapped(Vector2 restPoint, const std::vector<Vector2>& solvedVertices) const;

    /// The whole puppet warp as a `Warp2D::PlaneMap`: solves for `pinTargets` once and maps every
    /// point through the result.
    ///
    /// Use when: warping curves through the puppet
    /// Tags: puppet plane map
    /// Node: planned (puppetWarp)
    Warp2D::PlaneMap planeMap(const std::vector<Vector2>& pinTargets, int iterations = 8) const;

private:
    struct Neighbour {
        int vertex;
        double weight;
    };

    std::vector<Vector2> vertexList;
    std::vector<std::array<int, 3>> triangleList;
    std::vector<int> pinVertices;
    std::vector<Vector2> pinRestList;
    std::vector<std::vector<Neighbour>> neighbours;
    Bounds2 box;
    int cellColumns = 1;
    int cellRows = 1;
    std::vector<std::vector<int>> cellTriangles;

    void buildGrid(int cellsAlongLongerSide);
    void insertPin(Vector2 restPosition);
    void buildNeighbours();
    void buildCells();
    int cellIndexOf(Vector2 point) const;
    bool locate(Vector2 point, int& triangle, std::array<double, 3>& coordinates) const;
};

} // namespace editor
