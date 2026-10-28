// Fields that change another field: mirror it, move it, grow it, hollow it, roughen it.
// Each keeps the `DistanceField` contract (conservative, bounded, at least its box's
// distance outside the box); the proof is on each type (Engine/Math/Fields/FieldModifiers.swift).
#pragma once

#include <optional>

#include "../Geometry/Plane.h"
#include "../Linear/Transform.h"
#include "DistanceField.h"
#include "GradientNoise.h"

namespace editor {

/// A bound on how fast `GradientNoise::value` changes: 4, so `|noise(p) − noise(q)| ≤ 4 |p − q|`.
/// (Swift `GradientNoise.slopeBound`; GradientNoise.h is owned by another file, so it lives here.)
///
/// **Status: measured, not proven.** 4 rests on a numerical search, not on a derivation or
/// a certified grid; a certificate (grid maximum plus spacing times a bound on the
/// objective's own slope) is still to do. The catalog row says "measured" too.
///
/// How it was found: the gradient at a point depends only
/// on the point's place in its lattice cell and the 8 corner gradients. For a fixed place
/// and direction, its component along the direction is a sum of one term per corner, so
/// the worst choice of the 8 gradients (over every seed at once) is taken corner by
/// corner. Maximising that over the place in the cell and the direction, by a dense grid
/// and 300 random restarts of hill climbing in double precision, gives **3.75**, at the
/// cell's centre along an axis. 4 leaves a margin over 3.75. A crude proven bound is
/// `√3 × (15/8 × 4 + 1) ≈ 14.7` (fade slope at most 15/8, corner terms at most 2 apart
/// by 4, gradient components at most 1), too loose to use. `FieldModifierTests` samples
/// the slope of many seeds and checks it stays below this bound.
///
/// Use when: scaling a noise-displaced field so it stays a distance bound
/// Tags: Lipschitz constant, noise gradient bound, max slope, derivative bound
/// Node: none (constant)
inline constexpr float gradientNoiseSlopeBound = 4;

/// The largest size `GradientNoise::value` reaches, 1.0364 (the search behind
/// `gradientNoiseSlopeBound` gives 1.03635, rounded up): the farthest noise can push a
/// displaced surface per unit of amplitude. (Swift `GradientNoise.largestValue`.)
///
/// Use when: growing a displaced field's box by how far the noise can move the surface
/// Tags: noise range, largest noise value, displacement reach
/// Node: none (constant)
inline constexpr float gradientNoiseLargestValue = 1.0364f;

/// A shape mirrored across a plane: the part on the plane's positive side (where the normal
/// points) is kept and copied onto the other side, like the two halves of a face.
///
/// How: a point behind the plane is first folded onto the front side, then the inner field
/// is asked:
///
///     value(p) = inner(p)                  if plane distance of p ≥ 0
///              = inner(mirror image of p)  otherwise
///
///            front (kept) │ back (copy)
///               ╭──╮      │      ╭──╮
///               │  │ ●p   │  p'○ │  │      value(p') = value(p)
///               ╰──╯      │      ╰──╯
///
/// Exact when the inner field is exact and its shape lies wholly on the front side: from a
/// point in front, the mirrored copy is never nearer than the original (any front point is
/// at least as near to a front point as to its mirror image). If the inner shape crosses
/// the plane, its part behind the plane is dropped and the front part is mirrored instead,
/// so the result is cut flat at the plane and closed by its mirror image. The value stays
/// conservative in every case: folding moves points by a map that never lengthens
/// distances, so the result is 1-Lipschitz whenever the inner field is.
///
/// Conventions: the plane's normal points to the kept side.
///
/// Use when: building a symmetric face or body from one half; mirroring a limb to the other
///   side
/// Tags: mirror, symmetry, opSymmetry, reflect field, fold space, symmetric model, abs(p.x)
/// Node: mirrorField
struct MirroredField {
    DistanceField inner;
    /// The plane of symmetry; its normal points to the side that is kept.
    Plane plane;

    /// `inner`'s front part (the side `plane`'s normal points to), mirrored to the back.
    MirroredField(DistanceField inner, Plane plane);

    /// The inner field at `point`, or at its mirror image when `point` is behind the plane.
    ///
    /// Use when: sampling a symmetric shape
    /// Tags: mirrored sdf, symmetric field value, folded point
    /// Node: mirrorField
    float signedDistance(Vector3 point) const;

    /// The box around the inner box and its mirror image, or nil if the inner field is
    /// unbounded.
    ///
    /// How: the inside is the front part of the inner inside and its mirror image. The inner
    /// box is first cut to the plane's front side (`frontPart`, exact for an axis-aligned
    /// plane), because the back part is dropped; both pieces then lie in that cut box and its
    /// mirror image. A sphere of radius 1 at x = −0.5 mirrored across x = 0 gets
    /// x ∈ [−0.5, 0.5], not [−1.5, 1.5].
    ///
    /// Why the box still floors the value outside it: a point in front, outside the bigger
    /// box, has inner value at least its distance to the inner box. When the plane crosses
    /// the inner box, that distance equals the distance to the bigger box (along the cut axis
    /// a front point is measured from the same far face, `h`; the other axes are unchanged).
    /// When the cut changes nothing, the bigger box holds the inner box, so it is no farther.
    /// A point behind the plane is the mirror image of one in front, and the bigger box is
    /// symmetric.
    ///
    /// Use when: culling a mirrored shape
    /// Tags: mirror bounds, symmetric bounds, reflected box
    /// Node: mirrorField
    std::optional<Bounds> bounds() const { return mirroredBox; }

private:
    std::optional<Bounds> mirroredBox;
};

/// A shape moved, turned and scaled by a `Transform`.
///
/// How: a world point is taken back into the shape's own frame by the inverse transform,
/// and the distance there is scaled back to world units:
///
///     own point = rotation⁻¹(point − translation) / scale        (componentwise divide)
///     value     = s × inner(own point),      s = smallest |scale| component
///
/// raised to at least the distance to `bounds`.
///
/// With a **uniform** scale this is exact when the inner field is: distances scale with the
/// shape, so `value(T·p) = scale × inner(p)`. A **non-uniform** scale is handled by using the
/// smallest scale factor, which keeps the value conservative (but not exact): the inverse
/// transform stretches distances by at most `1/s`, so the own-frame distance is at most
/// `world distance / s`, and multiplying by `s` never overstates the world distance. The
/// result is 1-Lipschitz for the same reason. The raise to the box's distance restores the
/// contract's third part, which the shrink by `s` would otherwise lose far from the shape.
///
/// Conventions: `transform` maps the shape's own frame to the world: scale, then rotate,
/// then move. Every scale component must be finite and non-zero; a negative one mirrors.
///
/// Use when: placing a part of a model; posing a bone's shape; reusing one shape in many
///   places
/// Tags: transform field, translate sdf, rotate sdf, scale sdf, opTransform, instance,
///   local frame, non-uniform scale
/// Node: transform
struct TransformedField {
    DistanceField inner;
    /// Maps the shape's own frame to the world.
    Transform transform;

    /// `inner` moved by `transform`. Every scale component must be finite and non-zero.
    TransformedField(DistanceField inner, Transform transform);

    /// The point in the shape's own frame that `transform` moves to `point`:
    /// `rotation⁻¹(point − translation) / scale`.
    ///
    /// Use when: asking an inner field about a world point
    /// Tags: inverse transform point, world to local, to object space
    /// Node: transform
    Vector3 ownPoint(Vector3 point) const;

    /// The smallest scale factor times the inner value at `ownPoint(point)`, raised to at
    /// least the distance to `bounds` when there is one.
    ///
    /// Use when: sampling a placed or posed shape
    /// Tags: transformed sdf value, scaled distance, local sdf
    /// Node: transform
    float signedDistance(Vector3 point) const;

    /// The world box around the moved inner box, or nil.
    std::optional<Bounds> bounds() const { return worldBox; }

private:
    /// The smallest scale factor, which converts own-frame distances to world ones.
    float distanceScale;
    std::optional<Bounds> worldBox;
};

/// A shape grown (positive `amount`) or shrunk (negative) by a distance: every surface point
/// moves `amount` along its normal.
///
/// How: `value = inner − amount`. The surface is where the inner field equals `amount`.
///
/// For an exact inner field and a positive amount the value is exact outside the grown
/// shape, for any shape: the grown shape is all points within `amount` of the original, so
/// the distance to it is the distance to the original minus `amount`. Inside it, between the
/// old and the new surface, it is exact for convex shapes and a bound for others (in a
/// concave crease the grown surface folds onto itself). Shrinking is the same with inside
/// and outside swapped. For any inner field the value is 1-Lipschitz if the inner is, and
/// so conservative.
///
/// Use when: thickening a part; making a layer that wraps a shape at a distance; shrinking a
///   shape to make room
/// Tags: offset, dilate, erode, grow, inflate, opRound, rounding, expand shape
/// Node: offsetField
struct OffsetField {
    DistanceField inner;
    /// How far the surface moves outward; negative moves it inward. Finite.
    float amount;

    /// `inner` grown by `amount` (shrunk when negative). `amount` must be finite.
    OffsetField(DistanceField inner, float amount);

    /// `inner(point) − amount`.
    ///
    /// Use when: sampling the grown or shrunk shape
    /// Tags: offset sdf value, rounded distance, dilated field
    /// Node: offsetField
    float signedDistance(Vector3 point) const { return inner.signedDistance(point) - amount; }

    /// The inner box grown by `amount` when it is positive, the inner box itself when it is
    /// not (a shrunk shape stays inside its old box), or nil if the inner is unbounded.
    ///
    /// How: outside the grown box the inner value is at least the distance `D` to the inner
    /// box, so the value is at least `D − amount`, which is at least the distance to the box
    /// grown by `amount` (see `expanded`).
    ///
    /// Use when: culling a grown shape
    /// Tags: offset bounds, dilate bounds, grown box
    /// Node: offsetField
    std::optional<Bounds> bounds() const;
};

/// A hollow layer of thickness `thickness` centred on a shape's surface, such as skin, a fat
/// layer or a shell of a nut.
///
/// How: `value = |inner| − thickness/2`, negative within `thickness/2` of the surface on
/// either side.
///
///        outside      thickness       inside
///     ───────────────┃▓▓▓▓▓▓▓▓▓▓┃──────────────
///                       surface
///
/// Conservative: `|inner|` is 1-Lipschitz when `inner` is, and so is the shift. For an exact
/// inner field the value is exact outside the layer.
///
/// Use when: making a skin or fat layer of a given thickness; hollowing a shape
/// Tags: shell, onion, opOnion, hollow, layer, skin, thickness, rind
/// Node: shellField
struct ShellField {
    DistanceField inner;
    /// The total thickness of the layer, 0 or more.
    float thickness;

    /// A layer of `thickness` (finite, 0 or more) centred on `inner`'s surface.
    ShellField(DistanceField inner, float thickness);

    /// `|inner(point)| − thickness / 2`.
    ///
    /// Use when: sampling the layer
    /// Tags: shell sdf value, onion distance, hollow field
    /// Node: shellField
    float signedDistance(Vector3 point) const;

    /// The inner box grown by half the thickness, or nil if the inner is unbounded.
    ///
    /// How: as for `OffsetField` with `amount = thickness/2`: outside the inner box the inner
    /// value is positive, so `|inner| = inner`.
    ///
    /// Use when: culling a layer
    /// Tags: shell bounds, onion bounds, layer box
    /// Node: shellField
    std::optional<Bounds> bounds() const;
};

/// A shape whose surface is pushed in and out by gradient noise, for bumpy skin, rock or
/// bark. The value is scaled down so that it stays conservative.
///
/// How:
///
///     value = (inner(p) + amplitude × noise(p × frequency)) / (1 + |amplitude| × frequency × L)
///
/// raised to at least the distance to `bounds`. `noise` is `GradientNoise::value`, and
/// `L = gradientNoiseSlopeBound` bounds how fast it changes. Moving `p` by `δ` changes the
/// noise term by at most `|amplitude| × frequency × L × δ`, so the numerator changes by at most
/// `(1 + |amplitude| × frequency × L) × δ` when the inner field is 1-Lipschitz. Dividing by
/// that makes the value 1-Lipschitz, hence conservative (Hart, "Sphere tracing", 1996, which
/// treats displaced surfaces this way). The division does not move the surface: it is
/// still where `inner = −amplitude × noise`.
///
/// It is not a distance any more, only a bound: far from the surface it underestimates by
/// the divisor, which slows ray marching but never makes it overshoot.
///
/// Conventions: `frequency` is in cycles per unit length (the noise lattice has unit
/// spacing); the surface moves at most `|amplitude| × 1.0364` (the noise's range).
///
/// Use when: adding skin pores, wrinkles or lumps to a smooth body part; roughening rock
/// Tags: displacement, noise displacement, opDisplace, bumpy surface, perlin displacement,
///   Lipschitz bound, roughen
/// Node: roughenField
struct DisplacedField {
    DistanceField inner;
    /// The noise that pushes the surface.
    GradientNoise noise;
    /// How far the surface moves for a noise value of 1, in length units. Finite.
    float amplitude;
    /// How many noise cycles per unit length. Finite, 0 or more.
    float frequency;

    /// `inner` displaced by `amplitude × noise(p × frequency)`. `amplitude` must be finite and
    /// `frequency` finite and 0 or more.
    DisplacedField(DistanceField inner, GradientNoise noise, float amplitude, float frequency);

    /// The displaced, rescaled value at `point` (see the type), raised to at least the
    /// distance to `bounds` when there is one.
    ///
    /// Use when: sampling a bumpy shape
    /// Tags: displaced sdf value, noisy distance, Lipschitz-scaled field
    /// Node: roughenField
    float signedDistance(Vector3 point) const;

    /// The inner box grown by how far the noise can move the surface, or nil.
    std::optional<Bounds> bounds() const { return grownBox; }

private:
    /// `1 + |amplitude| × frequency × L`, the Lipschitz constant of the numerator.
    float slopeDivisor;
    std::optional<Bounds> grownBox;
};

} // namespace editor
