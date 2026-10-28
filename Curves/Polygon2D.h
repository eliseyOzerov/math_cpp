// A closed outline made of straight sides, in the plane: what a `Path2D` becomes once it is
// flattened, and the 2D shape a plate or a slab is extruded from
// (Engine/Math/Paths/Polygon2D.swift). Swift's `corners` is the accessor `corners()`; the
// initializer throws `MathProblem` where Swift's `precondition` stops the program.
#pragma once

#include <vector>

#include <optional>

#include "../Linear/Angle.h"
#include "../Geometry/Bounds2.h"
#include "../Geometry/Segment2.h"
#include "../Linear/Vector2.h"

namespace editor {

/// A closed polygon in the plane: corners joined in order, the last back to the first.
///
/// Use when: the flattened outline of a plate, a blade, a bone's flat profile; asking how far a
///   point is from a flat shape and whether it is inside
/// Tags: polygon 2D, outline, closed polyline, flat shape, profile, contour
/// Node: inside extrudedOutlineField
class Polygon2D {
public:
    /// The polygon through `corners`. A last corner equal to the first (as
    /// `Path2D::flattened` gives for a closed path) is dropped. At least 3 corners
    /// must remain.
    explicit Polygon2D(std::vector<Vector2> corners);

    bool operator==(const Polygon2D&) const = default;

    /// The corners in order. The side from the last corner back to the first closes it.
    const std::vector<Vector2>& corners() const { return cornerList; }

    std::vector<Segment2> sides() const;
    Bounds2 bounds() const;
    int windingNumber(Vector2 point) const;
    float signedDistance(Vector2 point) const;

    /// Three corners of a polygon, by their numbers in `corners()`, counterclockwise.
    ///
    /// Use when: reading the triangles a polygon was cut into
    /// Tags: triangle indices, index triple, triangulation triangle
    /// Node: none (value type)
    struct CornerTriangle {
        int first;
        int second;
        int third;

        bool operator==(const CornerTriangle&) const = default;
    };

    /// A regular star of `points` tips around the origin (Polygon2D+Star.cpp).
    static Polygon2D star(int points, float outerRadius, float innerRatio, Angle turn);

    /// The first point where a ray meets the outline (Polygon2D+Star.cpp).
    std::optional<Vector2> firstCrossing(Vector2 origin, Vector2 direction) const;

    /// The polygon cut into triangles by ear clipping (Polygon2D+Triangulation.cpp).
    std::vector<CornerTriangle> triangulated() const;

    /// The area enclosed, positive when counterclockwise (Polygon2D+Triangulation.cpp).
    float signedArea() const;

private:
    std::vector<Vector2> cornerList;

    float turn(size_t position, const std::vector<int>& remaining) const;
    bool isEar(size_t position, const std::vector<int>& remaining) const;
    size_t mostConvexPosition(const std::vector<int>& remaining) const;
};

} // namespace editor
