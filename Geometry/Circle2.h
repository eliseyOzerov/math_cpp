// A circle of the plane, and the one through three points (graphics_cpp curves/offset `circleThrough`).
#pragma once

#include <optional>

#include "../Linear/Vector2.h"

namespace editor {

/// A circle in the plane: all points `radius` from `centre`.
///
/// Use when: describing the arc a curve bends along; the circle that rounds a corner
/// Tags: circle 2D, centre and radius, round shape, arc circle
/// Node: planned (circleThroughPoints)
struct Circle2 {
    Vector2 centre;
    float radius = 0;

    bool operator==(const Circle2&) const = default;

    /// The circle through three points (the circumcircle of the triangle they make), or nothing
    /// when the points lie on a line.
    ///
    /// How: measured from `first`, with sides b = second − first and c = third − first and
    /// d = 2·(b × c), the centre is first + ( (c.y·|b|² − b.y·|c|²)/d , (b.x·|c|² − c.x·|b|²)/d )
    /// and the radius is its distance to `first`. This is the solution of |p − centre|² equal
    /// for the three points, written relative to one of them so large coordinates do not cancel.
    ///
    ///        second ●
    ///             ╱   ╲       the centre is where the sides' perpendicular
    ///      first ●  ○  ● third   bisectors meet
    ///
    /// Conventions: y-up plane. The points are on a line when |b × c| < 1e-6·s², with s the
    /// longest side: the test is relative, so it holds at any scale.
    ///
    /// Source: circumcircle of a triangle (Coxeter, *Introduction to Geometry*, 1969);
    /// graphics_cpp curves/offset `circleThrough`.
    ///
    /// Use when: fitting an arc through a curve's start, middle and end; finding the round of a
    ///   corner from three points
    /// Tags: circumcircle, circle through three points, three point arc, circumcentre
    /// Node: planned (circleThroughPoints)
    static std::optional<Circle2> through(Vector2 first, Vector2 second, Vector2 third);
};

} // namespace editor
