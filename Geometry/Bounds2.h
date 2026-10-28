// An axis-aligned rectangle on the plane (Engine/Math/Primitives/Bounds2.swift).
#pragma once

#include <vector>

#include "../Linear/Vector2.h"

namespace editor {

/// An axis-aligned rectangle on the plane, from its lower-left corner (`minimum`) to its
/// upper-right corner (`maximum`), in the y-up plane.
///
/// Conventions: `minimum ≤ maximum` on both axes (checked when it is made with `checked` or
/// `enclosing`). A rectangle with zero width or height is allowed: it is a segment or a point.
/// A plain aggregate `{minimum, maximum}`.
///
/// Use when: boxing a 2D curve or path to frame it in a view or to reject far-away tests
/// Tags: 2D bounding box, AABB 2D, rectangle, extent, Rect, min max, bounding rectangle
/// Node: none (2D outline bounds; boundingBox measures 3D geometry)
struct Bounds2 {
    Vector2 minimum;
    Vector2 maximum;

    bool operator==(const Bounds2&) const = default;

    /// The rectangle from `minimum` to `maximum` (Swift `Bounds2(minimum:maximum:)`). Both
    /// coordinates of `minimum` must be at most those of `maximum`, otherwise this throws.
    static Bounds2 checked(Vector2 minimum, Vector2 maximum);

    /// The smallest rectangle that contains every one of `points` (Swift `Bounds2(enclosing:)`).
    ///
    /// How: the minimum is the smallest x and the smallest y over the points, separately; the
    /// maximum is the largest of each. Needs at least one point, otherwise it throws.
    ///
    /// Use when: boxing the control points or the sampled points of a 2D shape
    /// Tags: bounding rectangle of points, min max of points, fit box, enclose points 2D
    /// Node: none (2D outline bounds; boundingBox measures 3D geometry)
    static Bounds2 enclosing(const std::vector<Vector2>& points);

    /// The width and the height: `maximum − minimum`. Never negative.
    ///
    /// Use when: scaling a view to fit a shape; checking how big an outline is
    /// Tags: rectangle size, width and height, extent size, dimensions 2D
    /// Node: none (2D outline bounds; boundingBox measures 3D geometry)
    Vector2 size() const { return maximum - minimum; }

    /// The smallest rectangle that contains both this one and `other`: the smaller of the two
    /// minimums and the larger of the two maximums, per axis. (Swift `union`.)
    ///
    /// Use when: growing a box as pieces of a path are added
    /// Tags: merge rectangles, combine bounds, union of boxes 2D, expand bounds
    /// Node: none (2D outline bounds; boundingBox measures 3D geometry)
    Bounds2 unionWith(const Bounds2& other) const;

    /// Whether `point` is inside the rectangle or exactly on its edge: minimum ≤ point ≤ maximum
    /// on both axes.
    ///
    /// Use when: rejecting a point that cannot be near a shape; checking a bound in a test
    /// Tags: point in rectangle, inside box 2D, hit test, contains point
    /// Node: none (2D outline bounds; boundingBox measures 3D geometry)
    bool contains(Vector2 point) const;
};

} // namespace editor
