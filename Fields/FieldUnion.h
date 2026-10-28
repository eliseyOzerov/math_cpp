// Combining shapes (Engine/Math/Fields/FieldUnion.swift). The union of two solids is where
// either is: the smaller of the two distances. A smooth union rounds the crease where they meet,
// like two drops of water. Swift's `Float.smoothMinimum(_:_:blendRadius:)` and its relatives are
// free functions here.
#pragma once

#include <cmath>
#include <optional>

#include "../Linear/Scalar.h"
#include "DistanceField.h"

namespace editor {

/// The polynomial **smooth minimum** of two numbers: the smaller of the two, pulled down where
/// they are within `blendRadius` of each other, so the result has no kink.
///
/// Source: Quílez, "smooth minimum" (iquilezles.org/articles/smin), the quadratic form.
///
/// How:
///
///     h = max(k − |a − b|, 0) / k          (0 when the numbers are k or more apart,
///                                           1 when they are equal)
///     smoothMinimum = min(a, b) − h² × k / 4
///
/// The dip is the bump `(k − |a−b|)² / 4k`: it is 0 with zero slope at `|a − b| = k`, so the
/// join with the plain minimum is smooth, and it is `k/4` at most.
///
/// Applied to two distance fields this is a **bound**, not an exact distance: it is lower
/// than the plain minimum by up to `k/4` (at equal distances, on the bisector between the
/// two shapes), which fattens the blended shape in the crease by that much, and its
/// gradient length can differ from 1 there. Hence `min(a, b) − k/4 ≤ result ≤ min(a, b)`.
///
/// Edge cases: `blendRadius ≤ 0` gives the plain minimum. Symmetric in its two inputs. A NaN
/// input gives NaN in every case, also with `blendRadius ≤ 0`. Two equal infinities give that
/// infinity (`+∞` and `+∞` give `+∞`, so a union can be seeded with `+∞`), and the result is
/// `−∞` if either input is `−∞`.
///
/// Use when: blending two shapes' distances; joining a limb to a body; metaball-like merging
/// Tags: smin, smooth min, polynomial smin, smooth union, metaball, blend, soft minimum, opSmoothUnion
/// Node: smoothMinimum
inline float smoothMinimum(float first, float second, float blendRadius) {
    // `smallerOf` returns whichever argument is not NaN depending on their order, so the NaN
    // check comes first and makes the answer the same for (a, b) and (b, a).
    if (std::isnan(first) || std::isnan(second)) return NAN;
    float smaller = smallerOf(first, second);
    if (!(blendRadius > 0)) return smaller;
    // |a − b| is NaN for two equal infinities (∞ − ∞); they are not blended.
    if (!(std::isfinite(first) && std::isfinite(second))) return smaller;
    float overlap = largerOf(blendRadius - std::fabs(first - second), 0) / blendRadius;
    return smaller - overlap * overlap * blendRadius / 4;
}

/// The polynomial **smooth maximum** of two numbers: the larger of the two, pushed up where
/// they are within `blendRadius` of each other. The mirror image of `smoothMinimum`.
///
/// Source: Quílez, "smooth minimum" (iquilezles.org/articles/smin), which builds the smooth
/// maximum from the smooth minimum by negation.
///
/// How: `smoothMaximum(a, b) = −smoothMinimum(−a, −b)`, which works out to
///
///     h = max(k − |a − b|, 0) / k
///     smoothMaximum = max(a, b) + h² × k / 4
///
/// so `max(a, b) ≤ result ≤ max(a, b) + k/4`, with the k/4 reached at `a = b`.
///
/// Why it keeps fields conservative: where the blend is active and `a ≥ b`, the result is
/// `a + h²k/4` with `h = (k − a + b)/k`, so `∂/∂a = 1 − h/2` and `∂/∂b = h/2`. Both lie in
/// 0…1 and they add up to 1, and outside the blend the result is `a` or `b` itself. So
/// the result changes by at most the larger change of `a` and `b`: if both are
/// 1-Lipschitz, so is the result (the same argument holds for `smoothMinimum`). A
/// 1-Lipschitz field never overstates the distance to its own surface (see
/// `DistanceField`), so no correcting factor is needed.
///
/// Edge cases: `blendRadius ≤ 0` gives the plain maximum. Symmetric in its inputs. A NaN
/// input gives NaN, also with `blendRadius ≤ 0`. Two equal infinities give that infinity,
/// and the result is `+∞` if either input is `+∞`.
///
/// Use when: intersecting two shapes with a rounded crease; carving one shape out of another with a fillet
/// Tags: smax, smooth max, polynomial smax, smooth intersection, smooth subtraction, soft maximum, opSmoothIntersection, opSmoothSubtraction
/// Node: smoothMinimum
inline float smoothMaximum(float first, float second, float blendRadius) {
    return -smoothMinimum(-first, -second, blendRadius);
}

/// The most `smoothMinimum` can lower a value below the plain minimum (and
/// `smoothMaximum` raise it above the plain maximum): `blendRadius / 4`, reached when the
/// two inputs are equal; 0 when `blendRadius` is 0, negative or NaN (no blend happens).
///
/// How: the dip is `h² × k / 4` with `h` in 0…1 (see `smoothMinimum`), largest at `h = 1`.
///
/// Use when: growing a blended shape's bounding box by exactly as much as the blend can
///   grow the shape
/// Tags: smin dip, blend overshoot, k/4, smooth union growth, bounds padding
/// Node: inside unionFields, intersectFields
inline float smoothMinimumLargestDip(float blendRadius) { return blendRadius > 0 ? blendRadius / 4 : 0; }

/// A bound on how far a chain of `steps` smooth minimums, `smin(… smin(smin(f₀, f₁), f₂) …,
/// fₙ)`, can lower its value below the plain minimum of all its inputs:
/// `min(steps × k/4, k)`; 0 when there are no steps or `blendRadius` is 0, negative or NaN.
/// It is exact for one step and as the steps grow (two steps dip at most 0.39k, not 0.5k).
///
/// How: each step dips at most k/4 (`smoothMinimumLargestDip`), which gives `steps × k/4`.
/// The second bound, `k` for any number of steps, comes from one inequality about a single
/// step:
///
///     smoothMinimum(a, b) ≥ min(a, b − k)
///
/// If `b − a ≥ k` the blend is off and the result is `a`. Otherwise write `d = b − a`,
/// which is less than k. When `d ≥ 0` the result is `a − (k − d)²/4k = b − g(d)`, where
/// `g(d) = d + (k − d)²/4k` has slope `1 − (k − d)/2k > 0`, so `g(d) ≤ g(k) = k`; so the
/// result is at least `b − k`. When `d < 0` the result is at least `min(a, b) − k/4`,
/// which is at least `b − k`.
///
/// Then, by induction, the chain value `cⱼ` is at least `mⱼ − k`, where `mⱼ` is the
/// smallest of `f₀ … fⱼ`. At the start `c₀ = f₀ = m₀`. At each step,
/// `cⱼ = smin(cⱼ₋₁, fⱼ) ≥ min(cⱼ₋₁, fⱼ − k)`, and both terms are at least `mⱼ − k`.
///
///     value
///       0 ┤●──────── every input 0
///    −k/4 ┤ ●        c₁ = −k/4
///         ┤   ●  ●   c₂ ≈ −0.39k, c₃ ≈ −0.48k …
///      −k ┤┄┄┄┄┄┄┄┄┄ the chain tends to −k but never passes it
///
/// The bound is tight as the steps grow: with every input equal to 0, `cⱼ` falls towards −k (the fixed
/// point of `c = c − (k + c)²/4k`).
///
/// Use when: growing the box of a smooth union of many parts by as much as the blends can
///   grow the shape, without the padding growing with the number of parts
/// Tags: smin chain dip, n-ary smooth union bounds, bounds padding, blend overshoot
/// Node: inside unionFields
inline float smoothMinimumChainLargestDip(float blendRadius, int steps) {
    // No steps, no dip, even for an infinite blend radius (where 0 × ∞ would be NaN).
    if (!(steps > 0)) return 0;
    float perStep = smoothMinimumLargestDip(blendRadius);
    return smallerOf(static_cast<float>(steps) * perStep, 4 * perStep);
}

/// The union of two shapes: a point is inside if it is inside either, so the distance is the
/// smaller of the two distances.
///
/// For exact fields the result is exact outside both shapes. Inside the overlap it can
/// underestimate the depth, because the nearest point of one shape's surface may lie buried
/// inside the other. It never overestimates the distance and stays 1-Lipschitz.
///
/// Use when: merging parts of a model into one; combining the limbs of a creature
/// Tags: union, min, opUnion, combine shapes, CSG union, add shapes
/// Node: unionFields
struct FieldUnion {
    DistanceField first;
    DistanceField second;

    /// The union of `first` and `second`.
    FieldUnion(DistanceField first, DistanceField second) : first(first), second(second) {}

    /// `min(first, second)` at `point`.
    ///
    /// Use when: sampling the combined shape
    /// Tags: opUnion, min of distances, csg union distance
    /// Node: unionFields
    float signedDistance(Vector3 point) const {
        return smallerOf(first.signedDistance(point), second.signedDistance(point));
    }

    /// The box around both parts' boxes, or nil if either part is unbounded.
    ///
    /// How: `enclosing(first.bounds, second.bounds)`. Outside that box both values are
    /// at least their own box's distance, which is at least the distance to the bigger box,
    /// so their minimum is too.
    ///
    /// Use when: culling the combined shape; sizing the region to mesh it in
    /// Tags: union bounds, merge boxes, bounding box of union
    /// Node: unionFields
    std::optional<Bounds> bounds() const { return enclosing(first.bounds(), second.bounds()); }
};

/// The union of two shapes with the crease between them rounded off over a distance
/// `blendRadius`. A bound, not an exact distance: see `smoothMinimum`.
///
/// Use when: joining a limb to a body; making blobby, organic shapes from simple parts
/// Tags: smooth union, opSmoothUnion, smin, metaballs, blend shapes, organic blend
/// Node: unionFields
struct SmoothUnion {
    DistanceField first;
    DistanceField second;
    /// The distance over which the shapes blend; 0 or less is a plain union.
    float blendRadius;

    /// The union of `first` and `second`, blended over `blendRadius`.
    SmoothUnion(DistanceField first, DistanceField second, float blendRadius)
        : first(first), second(second), blendRadius(blendRadius) {}

    /// `smoothMinimum` of the two distances at `point`, with `blendRadius`.
    ///
    /// The surface moves outward by up to `blendRadius / 4` where the two shapes' surfaces
    /// are equally far, and not at all where one is `blendRadius` or more nearer than the
    /// other. The value is lower than the plain union's by at most `blendRadius / 4`.
    ///
    /// Use when: sampling the blended shape
    /// Tags: smooth min of fields, opSmoothUnion, metaball field, blended distance
    /// Node: unionFields
    float signedDistance(Vector3 point) const {
        return smoothMinimum(first.signedDistance(point), second.signedDistance(point), blendRadius);
    }

    /// The box around both parts' boxes, grown on every side by `blendRadius / 4`, or nil if
    /// either part is unbounded.
    ///
    /// How: the smooth minimum is at least `min(a, b) − k/4` (see `smoothMinimum`).
    /// Outside the grown box, `min(a, b)` is at least the distance `D` to the plain union box,
    /// and `D − k/4` is at least the distance to the box grown by `k/4` (that box holds every
    /// point within `k/4` of the smaller one). So the blended field keeps the contract, and its
    /// inside (where it is ≤ 0) lies in the grown box. A blend radius of 0 or less grows nothing.
    ///
    /// Use when: culling the blended shape; sizing the region to mesh it in
    /// Tags: smooth union bounds, blend bounds, bounding box of smin
    /// Node: unionFields
    std::optional<Bounds> bounds() const {
        std::optional<Bounds> both = enclosing(first.bounds(), second.bounds());
        if (!both) return std::nullopt;
        return expanded(*both, smoothMinimumLargestDip(blendRadius));
    }
};

} // namespace editor
