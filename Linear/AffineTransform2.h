// A map of the plane that keeps straight lines straight and parallel lines parallel: a turn, a
// scaling, a shear and a move, in any mix. Ported from graphics_cpp's `Affine`, in
// floats. Declared here, defined in AffineTransform2.cpp.
#pragma once

#include <optional>

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

    /// A move by `offset`: xAxis and yAxis stay, translation = offset.
    ///
    /// Use when: sliding an outline; placing a profile at a position
    /// Tags: translate, move, offset, shift by vector, CGAffineTransform.translatedBy
    /// Node: planned (transform2D)
    static AffineTransform2 translationBy(Vector2 offset);

    /// A move of `distance` along the axis at `axisAngle` counterclockwise from the x axis:
    /// translation = distance·(cos, sin). `translationAlong(Angle(), d)` moves d to the right.
    ///
    /// Source: graphics_cpp curves/geometry `shift`.
    ///
    /// Use when: pushing a shape along a slanted direction; the slide that goes with a shear
    /// Tags: shift, slide along axis, move along angle, translate by heading
    /// Node: planned (transform2D)
    static AffineTransform2 translationAlong(Angle axisAngle, float distance);

    /// A turn by `angle` about `pivot` (counterclockwise for a positive angle).
    ///
    /// How: the linear part is xAxis = (cos, sin), yAxis = (−sin, cos); the pivot stays fixed, so
    /// translation = pivot − linear(pivot). y up: a quarter turn takes (1, 0) to (0, 1).
    ///
    /// Use when: turning a shape about its centre or a corner
    /// Tags: rotate, rotation matrix 2D, turn about point, CGAffineTransform.rotated, SVG rotate
    /// Node: planned (transform2D)
    static AffineTransform2 rotation(Angle angle, Vector2 pivot = Vector2());

    /// A scaling by `factors.x` across and `factors.y` up about `pivot`; a negative factor mirrors.
    ///
    /// How: xAxis = (factors.x, 0), yAxis = (0, factors.y), translation = pivot − linear(pivot),
    /// so the pivot stays fixed. A zero factor flattens the plane (no inverse).
    ///
    /// Use when: resizing or mirroring a shape about a point
    /// Tags: scale, scaling matrix 2D, mirror, flip, resize, SVG scale
    /// Node: planned (transform2D)
    static AffineTransform2 scaling(Vector2 factors, Vector2 pivot = Vector2());

    /// A shear along the axis at `axisAngle` through `pivot`: lines across that axis lean by `lean`.
    ///
    /// How: with u the unit vector along the axis and v = u turned a quarter counterclockwise,
    /// mapped = p + k·(v·p)·u with k = −tan(lean), taken about the pivot. For an axis along x,
    /// (x, y) goes to (x − y·tan(lean), y): a positive lean tips a vertical line counterclockwise.
    /// Points on the axis stay put. Leans of ±90° are infinite and give an infinite or NaN map.
    ///
    ///          ╱ ╱ ╱         vertical lines lean over, the axis keeps its length
    ///     ───●───────
    ///
    /// Use when: slanting text or a profile (italic); skewing a shape along a direction
    /// Tags: shear, skew, slant, oblique, italic, SVG skewX, CGAffineTransform skew
    /// Node: planned (transform2D)
    static AffineTransform2 shear(Angle lean, Angle axisAngle = Angle(), Vector2 pivot = Vector2());

    /// Where this map takes the point `point` (moved, turned and scaled):
    /// xAxis·point.x + yAxis·point.y + translation.
    ///
    /// Use when: moving one point through a transform
    /// Tags: apply transform, transform point, map point, CGPointApplyAffineTransform
    /// Node: planned (transform2D)
    Vector2 applied(Vector2 point) const;

    /// Where this map takes the direction `vector` (turned and scaled, never moved):
    /// xAxis·vector.x + yAxis·vector.y.
    ///
    /// Use when: transforming a tangent or an offset, which must not be moved
    /// Tags: transform vector, transform direction, linear part, apply without translation
    /// Node: planned (transform2D)
    Vector2 appliedToVector(Vector2 vector) const;

    /// This map followed by `next`: the single map that applies this one first.
    ///
    /// How: the columns of this map are sent through `next` (its two axes as directions, its
    /// translation as a point). Not commutative: a turn then a move differs from a move then a turn.
    ///
    /// Use when: building one map from a sequence of steps (scale, then turn, then move)
    /// Tags: compose, concatenate, multiply matrices, chain transforms, CGAffineTransformConcat
    /// Node: planned (transform2D)
    AffineTransform2 then(const AffineTransform2& next) const;

    /// The determinant of the linear part: xAxis.x·yAxis.y − xAxis.y·yAxis.x. It is the factor
    /// areas are scaled by (negative = mirrored).
    ///
    /// Use when: telling whether a map mirrors or flattens a shape; keeping an outline's winding
    /// Tags: determinant, area scale, mirror test, orientation flip
    /// Node: none (measurement)
    float determinant() const;

    /// The map that undoes this one, or nothing when it flattens the plane (a determinant that
    /// is zero, NaN or infinite).
    ///
    /// How: with the linear part [a c; b d] and determinant D, its inverse is [d −c; −b a]/D, and
    /// the translation becomes −inverse(translation).
    ///
    /// Use when: turning a point on screen back into the shape's own coordinates
    /// Tags: inverse transform, invert matrix, undo transform, CGAffineTransformInvert
    /// Node: planned (transform2D)
    std::optional<AffineTransform2> inverse() const;

    /// This map taken `amount` of the way from the identity: each column blended linearly,
    /// identity + (this − identity)·amount. Exact for moves and scalings; for turns it is a
    /// shortcut that also shrinks midway (a half of a 90° turn is not a 45° turn).
    ///
    /// Use when: fading a transform in; a quick tween between the identity and a map
    /// Tags: partial transform, interpolate transform, blend with identity, tween, fraction of transform
    /// Node: planned (transform2D)
    AffineTransform2 partly(float amount) const;

    /// This map, made in units where one across is `metric.x` and one up is `metric.y`, as a map
    /// of the plain coordinates: in matrix terms scaling(1/metric) · this · scaling(metric), so a
    /// point is first put in metric units, mapped, and put back.
    ///
    /// How: for xAxis = (a, b) and yAxis = (c, d) the result has xAxis = (a, b·metric.x/metric.y),
    /// yAxis = (c·metric.y/metric.x, d) and the translation divided per axis by `metric`. A turn
    /// made in a stretched drawing's true units comes out as the matching shear of its coordinates.
    ///
    /// Use when: turning a shape drawn on unequally scaled axes without distorting it
    /// Tags: metric transform, anisotropic rotate, aspect-correct rotation, conjugate by scaling
    /// Node: planned (transform2D)
    AffineTransform2 inMetric(Vector2 metric) const;
};

} // namespace editor
