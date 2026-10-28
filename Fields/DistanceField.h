// A signed distance field is a function that says, for every point in space, how far that
// point is from a shape's surface, and on which side. Shapes described this way can be
// combined (union, smooth blend) just by combining their numbers, and turned back into a mesh
// by looking for where the number crosses zero (Engine/Math/Fields/DistanceField.swift).
//
// Swift has a protocol `DistanceField` with a generic struct per field, and `AnyField` to hold
// any of them. Here `DistanceField` is that erased handle: a value holding a shared, immutable
// field, cheap to copy, built from any type that has `signedDistance(point)` and `bounds()`
// (members), or `signedDistanceOf` and `boundsOf` (free functions, for `Sphere`, `Plane` and
// `Bounds`). The generic structs (`SmoothUnion<First, Second>`) become plain structs that take
// `DistanceField` children. Meshing (`extractSurface`, `meshedSurface`) is declared here and
// defined in SurfaceNets.cpp.
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>

#include "../Geometry/Bounds.h"
#include "../Linear/Vector3.h"
#include "FieldBounds.h"
#include "SignedDistances.h"

namespace editor {

struct Mesh;
struct TriangleMesh;

/// The contract every field in the engine keeps, which ray marching and sparse surface
/// extraction rely on. (In Swift this is the doc comment of `protocol DistanceField`; here a
/// field is any type with `float signedDistance(Vector3) const` and
/// `std::optional<Bounds> bounds() const`, held by the `DistanceField` handle.)
///
/// A shape described by its **signed distance**: the distance from any point to the surface,
/// negative inside the shape, positive outside, zero on the surface.
///
/// Conventions: Z-up, right-handed; distances are in whatever length unit the points use.
/// An *exact* field returns the true distance to the nearest surface point, so it changes by
/// at most 1 per unit moved (it is **1-Lipschitz**) and its gradient has length 1. A *bound*
/// (see `SmoothUnion`) only promises never to overestimate the distance, which is enough for
/// ray marching and surface extraction, but its gradient length can differ from 1.
///
/// 1. **Conservative:** `|f(p)|` never exceeds the true distance from `p` to the field's
///    surface, and the sign is right (negative exactly inside). Exact fields meet this with
///    equality. Every other field here is **1-Lipschitz**, `|f(p) − f(q)| ≤ |p − q|`, and that
///    is enough: for any surface point `s`, `f(s) = 0`, so `|f(p)| = |f(p) − f(s)| ≤ |p − s|`
///    (Hart, "Sphere tracing", 1996, which calls such functions distance bounds).
///    `BoundedField` is the one exception to Lipschitz (it jumps down at its box), but it is
///    still conservative. Because the modifiers and smooth blends prove their sign from
///    Lipschitz, a `BoundedField` may only be used at the top level or directly under a plain
///    `FieldUnion` (min/max), never inside an offset, shell, displacement or smooth blend.
/// 2. **Bounded:** `bounds` is a box holding the whole inside of the shape, or nil when the
///    shape is unbounded (a plane).
/// 3. **At least the box's distance:** outside `bounds`, `f(p)` is at least the signed
///    distance from `p` to the box. This is what lets an offset or a shell grow its box by
///    the amount it grows the shape. Exact fields keep it automatically (the true distance to
///    a shape is at least the distance to a box around it); fields that are only bounds raise
///    their value to the box's distance where needed (see `raising`).
/// 4. **Pure:** `signedDistance` depends only on the point and the field's own values, and
///    changes nothing. Surface extraction calls it from several threads at once.
///
/// Use when: composing shapes into one model; asking whether a point is inside a shape;
///   feeding a surface extractor
/// Tags: signed distance field, SDF, distance function, implicit surface, isosurface,
///   level set, field, Shadertoy map(p)
/// Node: none (value type)
class DistanceField {
public:
    /// A field holding `concrete`: any type with `signedDistance(point)` and `bounds()`, or
    /// that `signedDistanceOf` / `boundsOf` accept. The box is read once, here.
    template <typename Concrete>
        requires(!std::is_same_v<std::decay_t<Concrete>, DistanceField>)
    DistanceField(Concrete concrete)
        : implementation(std::make_shared<const Model<std::decay_t<Concrete>>>(std::move(concrete))),
          boundingBox(implementation->bounds()) {}

    /// The signed distance from `point` to the surface: negative inside, zero on the surface,
    /// positive outside.
    ///
    /// Use when: sampling the field at a point
    /// Tags: sdf, distance to surface, map, evaluate field, inside test
    /// Node: sampleField
    float signedDistance(Vector3 point) const { return implementation->signedDistance(point); }

    /// A box that holds the whole inside of the shape, or nil when the shape is unbounded.
    /// Outside the box the field's value is at least the box's own signed distance.
    ///
    /// Use when: skipping a part of a model that is far from the point being evaluated;
    ///   choosing the region to extract a surface from
    /// Tags: bounding box, AABB, bounds of sdf, extent, culling box, broad phase
    /// Node: none (value type)
    const std::optional<Bounds>& bounds() const { return boundingBox; }

    // MARK: Gradient and normal

    /// The gradient of the field at `point`: the direction in which the distance grows fastest,
    /// with a length equal to how fast. For an exact field away from kinks it points straight
    /// away from the surface and has length 1.
    ///
    /// Source: central differences, as in Quílez, "normals for an SDF"
    /// (iquilezles.org/articles/normalsSDF), after the textbook finite-difference formula.
    ///
    /// How: for each axis, sample the field a step `h` either side of the point and divide the
    /// difference by the distance between the samples:
    ///
    ///     ∂f/∂x ≈ (f(p + h·x̂) − f(p − h·x̂)) / 2h        (same for y and z)
    ///
    ///     f(p−h) ●─────────●─────────● f(p+h)       the slope of the line through the two
    ///                  point p                       samples approximates the slope at p
    ///
    /// The Taylor series of `f` around `p` give the error: `h²/6 × f‴`. The divisor is the spacing
    /// of the samples as stored (see `centralDifference`), not `2h`, so a point far from the origin
    /// does not add a step error. So the formula is exact for linear fields (planes) and for
    /// quadratic ones along each axis. See `FiniteDifference::step` for choosing `h`.
    ///
    /// Edge cases: `step` must be positive, and large enough that `point ± step` differ from `point`
    /// in `Float` (otherwise the separation is 0 and the result is NaN or infinite). At a kink of the
    /// field (the medial axis of a shape, a box corner) the result is an average of the slopes on the
    /// two sides and its length is below 1. If the field is not finite near the point, the result
    /// contains NaN or infinity.
    ///
    /// Use when: shading a surface found by ray marching; pushing points off a surface along
    ///   its normal; building vertex normals for an extracted mesh
    /// Tags: gradient, calcNormal, finite differences, central difference, sdf normal, nabla,
    ///   numerical derivative
    /// Node: inside sampleField
    Vector3 gradient(Vector3 point, float step) const;

    /// The gradient of the field at `point`, with the step chosen by `FiniteDifference::step`,
    /// so it stays accurate far from the origin.
    ///
    /// How: `gradient(point, step)` with `step = FiniteDifference::step(featureSize, point)`.
    ///
    /// Conventions: `featureSize` is the size of the detail to resolve, in the units of the
    /// points; it must be positive.
    ///
    /// Use when: you want a gradient and do not want to pick the step yourself
    /// Tags: gradient, calcNormal, default step, numerical derivative, nabla
    /// Node: none (value type)
    Vector3 gradientForFeatureSize(Vector3 point, float featureSize) const;

    /// The direction the surface faces at (or near) `point`: the gradient scaled to length 1.
    /// It points out of the shape, toward positive distances.
    ///
    /// How: `gradient(point, step)` divided by its length. A zero gradient (a flat spot of the
    /// field, such as the exact centre of a sphere, where all directions are equally close)
    /// has no direction and gives the zero vector, like `Vector3::normalized`.
    ///
    /// Use when: lighting a point on an implicit surface; reflecting a ray off it
    /// Tags: surface normal, sdf normal, calcNormal, normalized gradient, implicit normal
    /// Node: inside sampleField
    Vector3 surfaceNormal(Vector3 point, float step) const;

    /// The unit normal at `point`, with the step chosen by `FiniteDifference::step`, so it stays
    /// accurate far from the origin.
    ///
    /// How: `surfaceNormal(point, step)` with `step = FiniteDifference::step(featureSize, point)`.
    ///
    /// Use when: you want a surface normal and do not want to pick the step yourself
    /// Tags: surface normal, sdf normal, calcNormal, default step, implicit normal
    /// Node: sampleField
    Vector3 surfaceNormalForFeatureSize(Vector3 point, float featureSize) const;

    // MARK: Meshing (DistanceField+Meshing.swift and SurfaceNets.swift)

    /// The box to mesh this field in: `bounds` grown by two cells on every side, so no sign
    /// change reaches the grid's outer faces and the mesh closes.
    ///
    /// Conventions: the field must be bounded (throws a `MathProblem` otherwise).
    ///
    /// Use when: choosing the region for `extractSurface` of a bounded field
    /// Tags: meshing box, extraction region, padded bounds
    /// Node: inside fieldToMesh
    Bounds paddedMeshingBox(float cellSize) const;

    /// How many grid cells `meshedSurface` would sample: the padded box's size over `cellSize`,
    /// rounded up, multiplied across the three axes. Capped at `Int.max / 2` so a huge box
    /// still gives a number to compare against a budget.
    ///
    /// Use when: checking a field model's grid fits its budget before meshing it
    /// Tags: cell count, grid size, voxel count, meshing budget
    /// Node: inside fieldToMesh
    std::int64_t meshingCellCount(float cellSize) const;

    /// The field's surface: meshed by surface nets in `paddedMeshingBox` and smoothed by
    /// `smoothingIterations` rounds of Taubin smoothing. Closed, faces pointing out.
    ///
    /// Use when: the mesh of a model built from fields (a bone, a plate, a blob)
    /// Tags: mesh field, sdf to mesh, polygonize and smooth, field to mesh node
    /// Node: fieldToMesh
    Mesh meshedSurface(float cellSize, int smoothingIterations) const;

    /// The zero level of the field as a triangle mesh, by **naive surface nets**.
    ///
    /// Source: Gibson, "Constrained elastic surface nets" (1998), in the common naive form
    /// without the relaxation step; Lorensen & Cline, "Marching cubes" (1987) is the family
    /// it belongs to.
    ///
    /// How:
    ///
    /// 1. **Sample.** The field is evaluated at the corners of a regular grid of cubic cells
    ///    `cellSize` wide, starting at `bounds.minimum`. The cell count along an axis is
    ///    `size / cellSize` rounded up (at least 1), so the grid covers `bounds` and may stick
    ///    out past `bounds.maximum` by less than one cell. A ratio within `10⁻⁵` (relative) of
    ///    a whole number is rounded to that number instead, because `size / cellSize` for sizes
    ///    that are whole multiples of the cell (`3 / 0.1`, or `0.003 / 0.0001`) can land a hair
    ///    above the whole number in `Float` and would otherwise add a cell; so the same bounds
    ///    and cell size, both scaled, give the same grid at every scale. A corner is *inside* if
    ///    its value is below 0 and *outside* otherwise (exactly 0 counts as outside). A NaN
    ///    corner is neither: every lattice edge with a NaN end is ignored in steps 2 and 3, so
    ///    NaN regions leave gaps in the mesh rather than NaN vertices. Only corners near the
    ///    surface are actually evaluated (`narrowBandValues`, on every core): a block the field
    ///    proves the surface cannot reach gives its corners its centre's value, which has
    ///    their sign, so the mesh is the one sampling every corner would give.
    /// 2. **One vertex per cell the surface passes through**: a cell with both inside and
    ///    outside corners. On each of its 12 edges whose ends differ, the zero is found by
    ///    linear interpolation, `t = a / (a − b)` from the end with value `a` toward the end
    ///    with value `b`. The vertex is the mean of these crossing points.
    /// 3. **One quad per grid edge that crosses the surface.** The four cells around such an
    ///    edge each have a vertex, and the quad joins them, wound so that its normal points
    ///    toward the outside (positive) end of the edge. Each quad is split into two triangles
    ///    along its shorter diagonal.
    ///
    ///     + + + +     2D picture: grid corners (+ outside, − inside). Each cell that
    ///     + ● ● +     mixes signs gets a vertex ● at the average of its edge crossings;
    ///     + ● ● +     every crossing edge joins the vertices of the cells around it, and
    ///     + + + +     the joins make the outline.
    ///
    /// Conventions: Z-up, right-handed; the triangles are counterclockwise seen from outside
    /// (normals point toward positive values, away from the shape). A quad needs four cells around
    /// its edge, so lattice edges lying on the grid's outer faces get no quad. The mesh is
    /// therefore **closed exactly when no lattice edge on the grid's outer faces changes sign**
    /// (and no NaN corner sits next to an inside corner); typically that means the surface
    /// stays one cell away from the outer faces. Where the surface leaves the grid the mesh is
    /// cut open and no cap is added.
    ///
    /// Accuracy: every vertex lies inside a cell that holds a sign change, so it is within one
    /// cell diagonal (`cellSize × √3`) of the surface. Sharp edges and corners are rounded off
    /// because a cell has only one vertex to put on both faces.
    ///
    /// Edge cases: `cellSize` must be positive, and `bounds.size / cellSize` finite along every
    /// axis (a `MathProblem` otherwise, for example for an infinite box). The grid is capped at
    /// `2²⁴ = 16 777 216` cells in total (256 per axis for a cube), which keeps the sampled
    /// values (4 bytes per corner) and the cell table (4 bytes per cell) near 140 MB and every
    /// vertex index inside `Int32`/`UInt32`; ask for a coarser cell or smaller bounds beyond
    /// that. A field with no zero crossing in the grid gives an empty mesh.
    ///
    /// Use when: turning a modelled signed-distance shape into a mesh to render or export;
    ///   previewing a blend of shapes
    /// Tags: surface nets, marching cubes, isosurface, polygonize, meshing an sdf, dual
    ///   contouring, extract mesh, voxel to mesh, metaball mesh
    /// Node: inside fieldToMesh
    TriangleMesh extractSurface(const Bounds& bounds, float cellSize) const;

    /// The same mesh as `extractSurface`, made by sampling every lattice corner of the grid on
    /// one core instead of only the narrow band near the surface. Slow; it is the reference the
    /// narrow band is checked against.
    ///
    /// Use when: checking that narrow-band meshing matches the plain definition of surface nets
    /// Tags: full grid, dense sampling, reference mesher, every corner
    /// Node: none (the test reference for fieldToMesh's mesher)
    TriangleMesh extractSurfaceSamplingEveryCorner(const Bounds& bounds, float cellSize) const;

private:
    /// What every held field answers: the two questions of the contract.
    struct Concept {
        virtual ~Concept() = default;
        virtual float signedDistance(Vector3 point) const = 0;
        virtual std::optional<Bounds> bounds() const = 0;
    };

    /// A `Concrete` field behind the `Concept`.
    template <typename Concrete>
    struct Model final : Concept {
        explicit Model(Concrete concrete) : concrete(std::move(concrete)) {}
        float signedDistance(Vector3 point) const override { return signedDistanceOfAny(concrete, point); }
        std::optional<Bounds> bounds() const override { return boundsOfAny(concrete); }
        Concrete concrete;
    };

    // A type with members is asked through them; `Sphere`, `Plane` and `Bounds` are asked
    // through the free `signedDistanceOf` / `boundsOf`.
    template <typename Concrete>
    static float signedDistanceOfAny(const Concrete& concrete, Vector3 point) {
        if constexpr (requires { concrete.signedDistance(point); }) return concrete.signedDistance(point);
        else return signedDistanceOf(concrete, point);
    }
    template <typename Concrete>
    static std::optional<Bounds> boundsOfAny(const Concrete& concrete) {
        if constexpr (requires { concrete.bounds(); }) return concrete.bounds();
        else return boundsOf(concrete);
    }

    /// The centre of a central difference: the slope along the unit vector `axis`.
    float centralDifference(Vector3 point, Vector3 axis, float step) const;

    std::shared_ptr<const Concept> implementation;
    /// The box of the held field, read once.
    std::optional<Bounds> boundingBox;
};

/// Choosing the step of a finite difference.
///
/// Use when: picking `h` for a numerical gradient without hand-tuning it
/// Tags: finite difference step, epsilon for normal, gradient step, h, calcNormal epsilon
/// Node: none (value type)
struct FiniteDifference {
    /// A good step `h` for `DistanceField::gradient` at `point`, when the shape's features are
    /// about `featureSize` across: `h = ∛(ε · max(|point|, R) · R²)` with `R = featureSize`.
    ///
    /// Why: a central difference has two errors. The *truncation* error grows with the step:
    /// the Taylor series give `h²/6 · f‴`, which for a shape with features of size `R` is about
    /// `h² / R²` (relative to the slope). The *rounding* error grows as the step shrinks: the
    /// field value at `point` is only known to `ε · |point|` (the coordinates themselves are
    /// rounded to that), and the two nearby values are subtracted and divided by `2h`, so it is
    /// about `ε · |point| / h`. `ε ≈ 6·10⁻⁸` is the rounding unit of `Float`. Setting the two
    /// equal gives `h³ = ε · |point| · R²`. Near the origin the point's own size does not matter
    /// below `R`, so `|point|` is replaced by `max(|point|, R)`, which reduces the formula to
    /// `h = ∛ε · R ≈ 4·10⁻³ · R`, each error then about `2·10⁻⁵`. A kilometre from the origin
    /// with `R = 1` the step grows to about `4·10⁻²`, and the normal stays good to a few `10⁻⁴`
    /// where the near-origin step would be off by `3·10⁻³`.
    ///
    /// Conventions: `featureSize` is the size of the detail to resolve (a sphere's radius, a
    /// capsule's thickness) in the same unit as the points, and must be positive. `|point|` is
    /// the distance from the origin.
    ///
    /// Use when: choosing the step for a normal on a shape of known size, anywhere in space
    /// Tags: finite difference step, epsilon for normal, gradient step, h, calcNormal epsilon
    /// Node: none (value type)
    static float step(float featureSize, Vector3 point);

    /// The rounding unit of `Float`, `2⁻²⁴ ≈ 6·10⁻⁸`: the largest relative error of one rounded
    /// operation.
    static constexpr float unitRoundoff = 1.1920929e-7f / 2;
};

/// A field given by a closure, for quick shapes and tests: `DistanceFieldFunction(bounds, [](Vector3 point) { … })`.
/// The closure must be pure (rule 4 of the contract): meshing calls it from several threads.
///
/// Use when: writing a one-off field without a new type
/// Tags: closure field, lambda sdf, inline sdf, custom distance function
/// Node: none (value type)
class DistanceFieldFunction {
public:
    /// A field whose value at a point is `function(point)`. Pass `bounds` only if the closure's
    /// shape lies inside it; the value is then raised to at least the box's distance outside it.
    DistanceFieldFunction(std::optional<Bounds> bounds, std::function<float(Vector3)> function)
        : boundingBox(bounds), function(std::move(function)) {}

    /// An unbounded field whose value at a point is `function(point)`.
    explicit DistanceFieldFunction(std::function<float(Vector3)> function)
        : DistanceFieldFunction(std::nullopt, std::move(function)) {}

    /// `function(point)`, as given to the initializer, raised to at least the signed distance
    /// to `bounds` when there is one (`raising`), so the field keeps the contract. Without
    /// bounds it is `function(point)` unchanged.
    ///
    /// Use when: sampling the field at a point
    /// Tags: sdf, evaluate closure field
    /// Node: none (value type)
    float signedDistance(Vector3 point) const {
        float value = function(point);
        if (!boundingBox) return value;
        return raising(*boundingBox, value, point);
    }

    /// The box given to the initializer, or nil (unbounded).
    std::optional<Bounds> bounds() const { return boundingBox; }

private:
    std::optional<Bounds> boundingBox;
    std::function<float(Vector3)> function;
};

} // namespace editor
