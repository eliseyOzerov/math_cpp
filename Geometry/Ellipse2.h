// An ellipse of the plane, its points, and the nearest of them to a point (the distance a fitted
// ellipse is measured by).
#pragma once

#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"

namespace editor {

/// An ellipse in the plane: a circle of radius `majorRadius` squashed to `minorRadius` across,
/// turned by `rotation` and moved to `centre`.
///
///            minor ↑   ╱ major axis, at `rotation` from x
///          ╭───────╮ ╱
///         (    ○    )        ○ centre
///          ╰───────╯
///
/// Conventions: y-up plane; majorRadius ≥ minorRadius ≥ 0 (the semi-axes); `rotation` is the
/// angle of the major axis from the x axis, counterclockwise, in −90°…90°.
///
/// Use when: the shape a hand-drawn oval is recognised as; an ellipse tool's result
/// Tags: ellipse 2D, oval, semi-major axis, semi-minor axis, conic, squashed circle
/// Node: planned (ellipse)
struct Ellipse2 {
    Vector2 centre;
    /// The semi-major axis: the distance from the centre to the farthest points.
    float majorRadius = 0;
    /// The semi-minor axis: the distance from the centre to the nearest points.
    float minorRadius = 0;
    /// The direction of the major axis, counterclockwise from x.
    Angle rotation;

    /// The point at `parameter` around the ellipse (not the polar angle of the point unless the
    /// ellipse is a circle).
    ///
    /// How: centre + (a·cos t)·m + (b·sin t)·m⊥, with m = (cos rotation, sin rotation) the
    /// major axis direction, m⊥ it turned 90° counterclockwise, a and b the semi-axes.
    ///
    /// Conventions: t = 0 is the end of the major axis along m; increasing t runs counterclockwise.
    ///
    /// Source: the parametric ellipse (Apostol, *Calculus* I, 1967, §13.20).
    ///
    /// Use when: drawing a recognised ellipse; sampling it
    /// Tags: ellipse point, parametric ellipse, eccentric anomaly, point on oval
    /// Node: planned (ellipse)
    Vector2 point(Angle parameter) const;

    /// The point on the ellipse (its outline) nearest to `position`.
    ///
    /// How: Eberly's robust method. In the ellipse's own frame, folded into the first quadrant
    /// (y₀, y₁ ≥ 0, semi-axes e₀ ≥ e₁), the nearest point x makes position − x normal to the
    /// ellipse at x (the Lagrange condition). With z = (y₀/e₀, y₁/e₁) and r = (e₀/e₁)², it is
    /// x = (r·y₀/(s + r), y₁/(s + 1)) for the root s of
    /// G(s) = (r·z₀/(s + r))² + (z₁/(s + 1))² − 1. G falls strictly on s > −1, and the root lies
    /// between z₁ − 1 and |(r·z₀, z₁)| − 1 (or 0, for a point inside), so bisection run until the
    /// midpoint stops moving finds it to the last bit. Points on an axis are handled in closed
    /// form; the result is unfolded back.
    ///
    ///          position ●
    ///                    ╲  along the normal
    ///          ╭──────────✕──╮
    ///         (       ○       )
    ///
    /// Conventions: computed in double. A circle (a = b) gives the radial projection; the centre
    /// of a circle gives the end of the major axis. A degenerate ellipse (b = 0) is the segment
    /// between the major axis ends.
    ///
    /// Source: D. Eberly, "Distance from a Point to an Ellipse, an Ellipsoid, or a
    /// Hyperellipsoid", Geometric Tools (2013), §2.
    ///
    /// Use when: measuring how far drawn points are from a fitted ellipse; snapping to an ellipse
    /// Tags: nearest point on ellipse, distance to ellipse, project onto ellipse, orthogonal distance
    /// Node: none (measuring a recognised ellipse)
    Vector2 nearestPoint(Vector2 position) const;

    /// The distance from `position` to the ellipse's outline: |position − nearestPoint(position)|.
    ///
    /// Use when: the residual of an ellipse fit; whether a click is on an ellipse
    /// Tags: distance to ellipse, point ellipse distance, orthogonal distance
    /// Node: none (measuring a recognised ellipse)
    float distance(Vector2 position) const;
};

} // namespace editor
