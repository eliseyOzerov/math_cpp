// Carving and trimming shapes with rounded creases, and blending many parts at once. The
// smooth maximum is the smooth minimum turned upside down, with the same polynomial, so one
// blend radius means the same fillet everywhere (Engine/Math/Fields/FieldCombinations.swift).
#pragma once

#include <optional>
#include <vector>

#include "DistanceField.h"

namespace editor {

/// One shape with another carved out of it, the crease rounded over `blendRadius`: the points
/// inside `kept` and outside `removed`.
///
/// How: the inside of "kept and not removed" is where both `kept` and `−removed` are
/// negative, so the value is their (smooth) maximum:
///
///     value = smoothMaximum(kept, −removed, blendRadius)
///
///      kept            removed           result
///     ╭─────╮         ╭───╮             ╭─────╮
///     │     │    −    │   │      =      │   ╭─╯   the fillet takes up to k/4 more
///     ╰─────╯         ╰───╯             ╰───╯       from the kept shape near the cut
///
/// Conservative: the smooth maximum of two 1-Lipschitz fields is 1-Lipschitz (see
/// `smoothMaximum`), so the value never overstates the distance to its own surface.
/// It is a bound, not the exact distance, even with radius 0 (as for `FieldUnion`, the
/// nearest surface point of one part can lie inside the other). The blend only raises the
/// value, by at most `blendRadius / 4`, so the result never reaches outside the hard
/// subtraction.
///
/// Use when: carving an eye socket, a nostril, the gap between fingers; cutting a shape with
///   a soft edge
/// Tags: smooth subtraction, opSmoothSubtraction, smooth difference, CSG difference, carve,
///   boolean subtract, fillet, smax
/// Node: subtractFields
struct SmoothSubtraction {
    DistanceField kept;
    DistanceField removed;
    /// The distance over which the crease is rounded; 0 or less is a hard subtraction.
    float blendRadius;

    /// `kept` with `removed` carved out, the crease rounded over `blendRadius`.
    SmoothSubtraction(DistanceField kept, DistanceField removed, float blendRadius)
        : kept(kept), removed(removed), blendRadius(blendRadius) {}

    /// `smoothMaximum(kept, −removed, blendRadius)` at `point`.
    ///
    /// Use when: sampling the carved shape
    /// Tags: smooth difference distance, opSmoothSubtraction, carved sdf
    /// Node: subtractFields
    float signedDistance(Vector3 point) const;

    /// The kept shape's box (nil if it is unbounded): carving only removes.
    ///
    /// How: the value is at least `kept`'s value (the maximum is at least each input), which
    /// is at least the box's distance outside the box.
    ///
    /// Use when: culling the carved shape
    /// Tags: subtraction bounds, difference bounds, keep first box
    /// Node: subtractFields
    std::optional<Bounds> bounds() const { return kept.bounds(); }
};

/// The part two shapes share, the crease rounded over `blendRadius`: the points inside both.
///
/// How: a point is inside both when both values are negative, so the value is their (smooth)
/// maximum, raised to at least the distance to `bounds`:
///
///     value = max(smoothMaximum(first, second, blendRadius), distance to bounds)
///
/// Conservative, for the reasons given on `SmoothSubtraction`: the smooth maximum is
/// 1-Lipschitz, and raising to the box's distance keeps it conservative (`raising`). The raise
/// is needed because the overlap of two boxes can be far smaller than either, so neither
/// part's own value reaches the overlap box's distance. The blend only raises the value, so
/// the result stays inside the hard intersection.
///
/// Use when: trimming a shape to a region; the rounded lens where two shapes overlap
/// Tags: smooth intersection, opSmoothIntersection, CSG intersection, boolean and, common
///   part, smax, trim
/// Node: intersectFields
struct SmoothIntersection {
    DistanceField first;
    DistanceField second;
    /// The distance over which the crease is rounded; 0 or less is a hard intersection.
    float blendRadius;

    /// The part `first` and `second` share, the crease rounded over `blendRadius`.
    SmoothIntersection(DistanceField first, DistanceField second, float blendRadius);

    /// The smooth maximum of the two values at `point`, raised to at least the distance to
    /// `bounds` when there is one.
    ///
    /// Use when: sampling the shared part of two shapes
    /// Tags: smooth intersection distance, opSmoothIntersection, intersected sdf
    /// Node: intersectFields
    float signedDistance(Vector3 point) const;

    /// The overlap of the two boxes; the one box when the other part is unbounded; nil when
    /// both are.
    ///
    /// How: the inside of the result lies inside both parts, so inside both boxes. When the
    /// boxes do not overlap the shape is empty (the value is positive everywhere), and the
    /// box is the single point at the first box's centre, which holds the empty inside.
    /// (Swift computes this on every call; here it is worked out once.)
    ///
    /// Use when: culling the shared part
    /// Tags: intersection bounds, box overlap, AABB of intersection
    /// Node: intersectFields
    std::optional<Bounds> bounds() const { return overlapBox; }

private:
    std::optional<Bounds> overlapBox;
};

/// The smooth union of any number of shapes with one blend radius, for shapes chosen while
/// the program runs (a model graph, a recipe).
///
/// How: the parts are blended pairwise from first to last:
///
///     value = smoothMinimum(… smoothMinimum(smoothMinimum(f₀, f₁), f₂) …, fₙ₋₁)
///
/// **The order matters.** The smooth minimum is not associative: blending `(a, b)` first and
/// then `c` differs slightly from blending `(b, c)` first where all three are within the
/// blend radius of each other. The parts are always taken in the stored order, so the
/// result is repeatable.
///
/// Conservative: each step is 1-Lipschitz (see `smoothMaximum`), so the whole chain
/// is. The chain lowers the plain union by at most `min((n − 1) × blendRadius / 4,
/// blendRadius)` (see `smoothMinimumChainLargestDip`): never more than one blend
/// radius, however many parts there are.
///
/// Speed: a part whose box is at least `blendRadius` farther than the value so far is skipped.
/// Its own value is at least its box's distance, so the smooth minimum with it would return
/// the value so far unchanged; the skip gives the very same result.
///
/// Use when: blending all the muscles of a limb into one; joining many parts built at run
///   time
/// Tags: n-ary smooth union, smooth union of many, smin of list, metaballs, blend list,
///   multi blend
/// Node: unionFields
struct SmoothUnionOfMany {
    /// The parts, blended in this order.
    std::vector<DistanceField> parts;
    /// The distance over which neighbouring parts blend; 0 or less is a plain union.
    float blendRadius;

    /// The smooth union of `parts` (at least one), blended in order over `blendRadius`.
    SmoothUnionOfMany(std::vector<DistanceField> parts, float blendRadius);

    /// The parts' values blended in order by `smoothMinimum`, skipping parts too far
    /// away to change the result.
    ///
    /// Use when: sampling the blend of many parts
    /// Tags: smooth union of list, n-ary smin, blended distance
    /// Node: unionFields
    float signedDistance(Vector3 point) const;

    /// The box around all parts, grown by the chain's largest dip, or nil.
    std::optional<Bounds> bounds() const { return chainBox; }

private:
    /// Each part's box, worked out once.
    std::vector<std::optional<Bounds>> partBoxes;
    std::optional<Bounds> chainBox;

    /// The box around `boxes` grown by the chain's largest dip, `min((n − 1)k/4, k)`, or nil
    /// if any is nil.
    ///
    /// How: outside the grown box every part is at least the distance `D` to the plain union
    /// box, so the chain is at least `D − dip` (`smoothMinimumChainLargestDip`), which
    /// is at least the distance to the grown box (see `expanded`).
    static std::optional<Bounds> chainBounds(const std::vector<std::optional<Bounds>>& boxes, float blendRadius);
};

} // namespace editor
