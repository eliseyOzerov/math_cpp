// Narrow-band sampling for surface nets. Most of a field's box is far outside or deep inside the
// shape, and no triangle comes from there. One sample at the centre of a block of cells can prove
// that the surface does not pass through the block, so only the corners near the surface are
// sampled, and they are sampled on every core (Engine/Math/Fields/NarrowBand.swift).
// `SurfaceNetsGrid::narrowBandValues` is declared in SurfaceNets.h and defined in NarrowBand.cpp.
#pragma once

namespace editor {

/// The test that lets surface nets skip a block of cells, and the block sizes it works with.
///
/// Use when: deciding which parts of a field's grid need sampling before meshing it
/// Tags: narrow band, sparse sampling, octree culling, skip empty space, lipschitz bound
/// Node: inside fieldToMesh
struct NarrowBand {
    /// The smallest block that is tested on its own, in cells along each axis. A block of this
    /// size that the surface may reach has all its corners sampled.
    static constexpr int leafCells = 4;
    /// How many times a top block is halved down to a leaf: top blocks are
    /// `leafCells × 2^levels = 16` cells along each axis.
    static constexpr int levels = 2;
    /// The factor on the half-diagonal that leaves room for rounding in the field's value and in
    /// the corner positions: a block is skipped only when its centre value is 1/16 further from
    /// zero than the proof needs.
    static constexpr float roundingAllowance = 17.0f / 16.0f;

    /// The distance from the centre of a cube to each of its corners: half its diagonal.
    ///
    /// How: the diagonal of a cube with sides `side` is `√(side² + side² + side²) = side × √3`,
    /// so the half-diagonal is `side × √3 / 2`. Every point of the cube, its faces and corners
    /// included, is at most this far from the centre.
    ///
    ///     ┌─────────●     the centre ○ is half a diagonal from every corner ●
    ///     │       ╱ │
    ///     │     ○   │
    ///     │         │
    ///     └─────────┘
    ///
    /// Conventions: `side` in any length unit, 0 or more; the result is in the same unit.
    ///
    /// Use when: the radius of the ball round a cubic cell or block; bounding a grid block by a sphere
    /// Tags: cube half diagonal, circumradius of cube, bounding sphere of cube, side times root three
    /// Node: inside fieldToMesh
    static float cubeHalfDiagonal(float side);

    /// Whether the surface (the zero level of a field) cannot pass through a cube, given the
    /// field's value at the cube's centre: true when that value is further from zero than the
    /// cube's half-diagonal, with `roundingAllowance` to spare. Then every point of the cube,
    /// faces and corners included, has a value of the same sign as the centre, and none is 0.
    ///
    /// Source: the Lipschitz bound of Hart, "Sphere tracing: a geometric method for the
    /// antialiased ray tracing of implicit surfaces" (1996), §2–3: a field that changes no faster
    /// than distance has no zero closer to a point than the point's own value.
    ///
    /// How: let `c` be the centre, `h` the half-diagonal and `v = f(c)` with `|v| > h`. Every
    /// point `p` of the cube has `|p − c| ≤ h` (`cubeHalfDiagonal`). Either of two facts about
    /// the field then gives `f(p)` the sign of `v`:
    ///
    /// 1. **The field is 1-Lipschitz**, `|f(p) − f(c)| ≤ |p − c|`. Then
    ///    `|f(p) − v| ≤ h < |v|`, so `f(p)` lies strictly between 0 and `2v`: the sign of `v`,
    ///    never 0. Every `FieldShape` is (`FieldLipschitzTests`).
    /// 2. **The field is conservative with the right sign**, `|f| ≤` the true distance to the
    ///    surface and `f < 0` exactly inside. Then the ball of radius `|v|` round `c` holds no
    ///    surface point. A ball is connected, so all of it is inside or all of it is outside,
    ///    like `c`, and the cube lies within it. This covers `BoundedField`, which is
    ///    conservative but not 1-Lipschitz at its box's faces.
    ///
    /// With the field evaluated in `Float`, the computed values carry rounding errors `e`. The
    /// test asks for `|ṽ| > (17/16)·h`; the step above then leaves every point of the cube with
    /// `|f(p)| > h/16 − |e(c)|`, so its computed value keeps the sign as long as
    /// `|e(c)| + |e(p)| < h/16`. For a leaf, `h/16` is about a fifth of a cell (0.22). The
    /// errors are absolute, though: positions and the field's differences of them round to
    /// about one unit in the last place of the largest coordinate `M`, `ulp(M) ≈ M·2⁻²³`. So
    /// the room holds only while the cell is large next to `M`; `roundingIsSmall` asks for
    /// `M ≤ 2¹⁶` cells, which leaves a fifth of a cell ≥ 28·ulp(M) for the rounding, and the
    /// mesher samples every corner otherwise. (An independent review reproduced wrong signs
    /// near `M / cellSize ≈ 10⁷`, 2⁷ times past that limit.)
    ///
    /// Route 2 gives no room of its own for rounding. `BoundedField` gets the same `|v| − h`
    /// margin as route 1 anyway: inside its box it is its inner field, which is 1-Lipschitz,
    /// and outside the box it is at least the box's distance, which is 1-Lipschitz too.
    ///
    /// Edge cases: a NaN centre value is never far (the comparison is false), so a block with a
    /// NaN centre is always refined and its corners sampled. An infinite value is far: a field
    /// that is infinite somewhere is either infinite everywhere (an empty union) or not Lipschitz.
    ///
    /// Use when: skipping a block of grid cells that holds no surface before meshing a field
    /// Tags: narrow band test, empty block, lipschitz culling, unbounding sphere, sphere tracing bound, skip cell
    /// Node: inside fieldToMesh
    static bool surfaceCannotReach(float centreValue, float side);

    /// The most cells the largest coordinate of a grid may be from the origin while the
    /// rounding of positions stays within `roundingAllowance`'s room: 2¹⁶.
    static constexpr float largestCoordinateInCells = 65536;

    /// Whether `Float` rounding of positions near `farthestCoordinate` is small enough, next to
    /// `cellSize`, for `surfaceCannotReach` to be trusted: true when
    /// `farthestCoordinate ≤ 2¹⁶ × cellSize`.
    ///
    /// How: positions round by about `ulp(M) ≈ M·2⁻²³` at the largest coordinate `M`. With
    /// `M ≤ 2¹⁶·cellSize` that is at most `cellSize·2⁻⁷`, and the room `surfaceCannotReach`
    /// leaves for a leaf, about `0.22·cellSize`, is then at least 28 of those units.
    ///
    /// Conventions: `farthestCoordinate` is the largest absolute coordinate of any grid corner;
    /// `cellSize` is positive. A NaN or infinite input gives false.
    ///
    /// Use when: deciding whether a grid far from the origin can skip blocks at all
    /// Tags: float precision, ulp, large coordinates, catastrophic cancellation, far from origin
    /// Node: inside fieldToMesh
    static bool roundingIsSmall(float farthestCoordinate, float cellSize);
};

} // namespace editor
