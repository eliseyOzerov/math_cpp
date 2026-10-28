// A map of the plane that keeps straight lines straight and parallel lines parallel: a turn, a
// scaling, a shear and a move, in any mix. Ported from graphics_cpp's `Affine` (Curves2D), in
// floats. Declared here, defined in AffineTransform2.cpp.
#pragma once

#include "Angle.h"
#include "Vector2.h"

namespace editor {

/// An affine map of the plane: where the x axis goes, where the y axis goes, and the move.
///
///     mapped = xAxis·x + yAxis·y + translation
///
///          yAxis ↑
///                │╲          the unit square's corners go to translation,
///                │ ╲         translation + xAxis, translation + yAxis and the sum of all three
///     translation●──→ xAxis
///
/// It is the 3×3 matrix [xAxis yAxis translation; 0 0 1] acting on (x, y, 1), stored as its two
/// linear columns and its translation column. Conventions: y-up plane, counterclockwise turns
/// are positive. `then` composes left to right: `first.then(second)` applies `first` first.
///
/// Use when: turning, scaling, mirroring or shearing a 2D outline about a pivot; placing a
///   profile on the plane before a sweep
/// Tags: affine transform, 2D transform, matrix 2x3, CGAffineTransform, SVG transform, rotate,
///   scale, shear, skew, translate
/// Node: planned (transform2D)
struct AffineTransform2 {
    /// Where the unit x axis goes.
    Vector2 xAxis = Vector2(1, 0);
    /// Where the unit y axis goes.
    Vector2 yAxis = Vector2(0, 1);
    /// Where the origin goes.
    Vector2 translation;

    bool operator==(const AffineTransform2&) const = default;

    /// The map that leaves every point where it is.
    static constexpr AffineTransform2 identity() { return AffineTransform2(); }

    /// A move by `offset`.
    static AffineTransform2 translationBy(Vector2 offset);

    /// A turn by `angle` about `pivot` (counterclockwise for a positive angle).
    static AffineTransform2 rotation(Angle angle, Vector2 pivot = Vector2());

    /// A scaling by `factors.x` across and `factors.y` up about `pivot`; a negative factor mirrors.
    static AffineTransform2 scaling(Vector2 factors, Vector2 pivot = Vector2());

    /// A shear along the axis at `axisAngle` through `pivot`: lines across that axis lean by `lean`.
    static AffineTransform2 shear(Angle lean, Angle axisAngle = Angle(), Vector2 pivot = Vector2());

    /// Where this map takes the point `point` (moved, turned and scaled).
    Vector2 applied(Vector2 point) const;

    /// Where this map takes the direction `vector` (turned and scaled, never moved).
    Vector2 appliedToVector(Vector2 vector) const;

    /// This map followed by `next`: the single map that applies this one first.
    AffineTransform2 then(const AffineTransform2& next) const;

    /// The determinant of the linear part: the factor areas are scaled by (negative = mirrored).
    float determinant() const;

    /// The map that undoes this one, or nothing when it flattens the plane (determinant 0).
    std::optional<AffineTransform2> inverse() const;

    /// This map taken `amount` of the way from the identity: each column blended linearly.
    /// Exact for moves and scalings; for turns it is a shortcut that also shrinks midway.
    AffineTransform2 partly(float amount) const;

    /// This map, made in units where one across is `metric.x` and one up is `metric.y`, as a map
    /// of the plain coordinates: scaling(1/metric) · this · scaling(metric).
    AffineTransform2 inMetric(Vector2 metric) const;
};

} // namespace editor
