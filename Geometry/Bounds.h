// An axis-aligned bounding box (Engine/Math/Primitives/Bounds.swift).
#pragma once

#include <vector>

#include "../Linear/Vector3.h"

namespace editor {

/// The box between two opposite corners, with faces parallel to the axes. Its faces and edges
/// belong to the box.
///
/// It is a plain aggregate `{minimum, maximum}`; build one that is checked with `checked`,
/// `fromCentre` or `enclosing`. Swift's `Bounds` traps on an inside-out box; these throw a
/// `GeometryProblem` with Swift's sentence.
///
/// Use when: quickly rejecting objects that cannot be hit; framing the camera around a model; printing the size of a model; finding the lowest z, where the model stands
/// Tags: bounding box, AABB, axis aligned box, bounds, extents, Bounds, box
/// Node: boxField
struct Bounds {
    /// The corner with the smallest x, y and z.
    Vector3 minimum;
    /// The corner with the largest x, y and z.
    Vector3 maximum;

    bool operator==(const Bounds&) const = default;

    /// A box from its two corners (Swift `Bounds(minimum:maximum:)`). Every component of
    /// `minimum` must be ≤ the same component of `maximum`, otherwise the box would be inside
    /// out, and this throws.
    static Bounds checked(Vector3 minimum, Vector3 maximum);

    /// The box of `size` (width, depth, height) whose middle is `centre` (Swift
    /// `Bounds(centre:size:)`).
    ///
    /// How: half the size goes each way from the centre, minimum = centre − size / 2 and
    /// maximum = centre + size / 2. Every component of `size` must be 0 or more (throws
    /// otherwise, as `checked` does).
    static Bounds fromCentre(Vector3 centre, Vector3 size);

    /// The smallest box that holds every point of a non-empty list (Swift
    /// `Bounds(enclosing:)`).
    ///
    /// Source: Ericson §4.2, "Axis-aligned bounding boxes". How: `minimum` is the smallest x,
    /// the smallest y and the smallest z over all points; `maximum` the largest. An empty list
    /// has no box, so it throws.
    static Bounds enclosing(const std::vector<Vector3>& points);

    /// The middle of the box: halfway between the corners (`Vector3::linearBlend` at 0.5).
    ///
    /// Use when: aiming the camera at the middle of a model; placing a pivot
    /// Tags: bounds center, box centre, midpoint, centre of box, Bounds.center
    /// Node: boundingBox
    Vector3 centre() const { return Vector3::linearBlend(minimum, maximum, 0.5f); }

    /// The width, depth and height: `maximum − minimum`. A flat box has a 0 component.
    ///
    /// Use when: sizing a camera or a grid to fit a model
    /// Tags: box size, extents, dimensions, bounds size, width height depth
    /// Node: boundingBox
    Vector3 size() const { return maximum - minimum; }

    /// The smallest box that holds both this box and `other`: the smaller minimum and the larger
    /// maximum on each axis. (Swift `union`, a keyword in C++.)
    ///
    /// Use when: growing the bounds of a scene as objects are added; combining the boxes of a model's parts
    /// Tags: merge bounds, union of boxes, Encapsulate, expand bounds, combine AABB
    /// Node: boundingBox
    Bounds unionWith(const Bounds& other) const;

    /// Whether `point` is inside the box or on its surface: minimum ≤ point ≤ maximum on every
    /// axis (Ericson §4.2). No tolerance; a point 1e-9 outside is outside.
    ///
    /// Use when: testing whether a point lies inside a region, such as a selection box or a trigger zone
    /// Tags: point in box, inside bounds, AABB contains, containment, box test
    /// Node: pointInBox
    bool contains(Vector3 point) const;
};

} // namespace editor
