// A character is many parts, and a point near the hand does not need the foot's exact
// distance: the distance to the foot's box is enough to know the foot is far away
// (Engine/Math/Fields/BoundedField.swift).
#pragma once

#include <optional>

#include "DistanceField.h"

namespace editor {

/// A field that answers with the cheap distance to its box when the point is outside the
/// box, and with the inner field's own value inside it.
///
/// How:
///
///     value(p) = box distance(p)    if p is outside the box
///              = inner(p)           if p is inside or on the box
///
///     ┌──────── box ────────┐
///     │   ╭───────────╮     │   ● outside: one box distance, the inner field
///     │   │   inner   │     │     is not evaluated
///     │   ╰───────────╯     │
///     └─────────────────────┘
///
/// Conservative, with the right sign everywhere: outside, the box's distance is positive
/// and at most the true distance (the shape is in the box); inside, it is the inner field.
/// Inside the box the values are equal to the inner field's. It is **not** 1-Lipschitz: at the
/// box's faces the value can drop from the inner field's value to the box's 0. That keeps it
/// conservative but makes its gradient wrong right at the faces, so take normals from the
/// inner field.
///
/// **Where it may be used:** only as the top-level field handed to a ray marcher or surface
/// extractor, or directly under a plain `FieldUnion` (min) or maximum, which only compare
/// values and so keep the sign right. **Never** put it inside `OffsetField`, `ShellField`,
/// `DisplacedField`, `SmoothUnion`, `SmoothUnionOfMany`, `SmoothSubtraction` or
/// `SmoothIntersection`: those prove their sign from the input being 1-Lipschitz, and the
/// small positive box distance outside the box is pushed below 0 by an offset, a shell, a
/// displacement or a smooth minimum, which draws phantom blobs at the box corners. Example:
/// a unit ball with `OffsetField(by: 0.5)` at (1.2, 1.2, 1.2) is +0.578 (outside), but
/// `OffsetField(BoundedField(ball), by: 0.5)` gives −0.154 (inside). The blends and
/// `SmoothUnionOfMany` already skip far parts using the parts' own `bounds`; leave the
/// culling to them.
///
/// Conventions: the box is the inner field's `bounds`; an unbounded inner field (nil) is
/// always evaluated.
///
/// Use when: speeding up the final, top-level field of a model during ray marching or
///   meshing; culling far parts under a plain union
/// Tags: bounding volume, early out, box culling, lazy evaluation, BVH leaf, sdf
///   acceleration, bounded sdf
/// Node: none (value type)
struct BoundedField {
    DistanceField inner;
    /// The inner field's box, worked out once (Swift's stored `bounds`).
    std::optional<Bounds> boundingBox;

    /// `inner`, answered by its box's distance outside its box.
    explicit BoundedField(DistanceField inner) : inner(inner), boundingBox(inner.bounds()) {}

    /// The box's signed distance when `point` is outside the box, otherwise the inner field's
    /// value.
    ///
    /// Use when: sampling a part cheaply far from it
    /// Tags: bounded distance, culled sdf, early-out evaluation
    /// Node: none (an evaluation speed-up for a renderer; a graph samples a field with sampleField)
    float signedDistance(Vector3 point) const {
        if (boundingBox) {
            float boxDistance = signedDistanceOf(*boundingBox, point);
            if (boxDistance > 0) return boxDistance;
        }
        return inner.signedDistance(point);
    }

    /// The inner field's box.
    std::optional<Bounds> bounds() const { return boundingBox; }
};

} // namespace editor
