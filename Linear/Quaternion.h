// A quaternion rotation (Engine/Math/Algebra/Quaternion.swift, Quaternion+Alignment.swift).
// Swift's `Quaternion(axis:angle:)` is `Quaternion::fromAxis`, `Quaternion.identity` is
// `Quaternion::identity()`, `Quaternion.slerp(from:to:by:)` is `Quaternion::slerp(start, end, amount)`
// and `Quaternion.rotation(from:to:)` is `Quaternion::rotation(direction, target)`. Swift's unit-length
// `assert`s are `assert`s here.
#pragma once

#include "Angle.h"
#include "Matrix4.h"

namespace editor {

/// A **quaternion**: a compact way to store a rotation in 3D. Four numbers instead of a
/// matrix's nine, and easy to blend smoothly between two rotations (`slerp`).
///
/// A quaternion is a vector part `u` (3 numbers) and a scalar part `s` (1 number). To turn by
/// `angle` around a unit `axis`, use half the angle (Shoemake 1985, "Animating rotation with
/// quaternion curves"):
///
///     u = axis × sin(angle / 2)        s = cos(angle / 2)
///
/// The vector part points along the axis and gets longer the more you turn. The scalar part
/// shrinks from 1 (no turn) to 0 (half a turn).
///
/// **Conventions:**
/// - A **unit** quaternion (length 1) is a rotation. Every function here that turns things needs
///   one; build them with `fromAxis` and `normalized` and they stay unit.
/// - Turns follow the right-hand rule, exactly like `Matrix4::rotation`: counterclockwise when
///   looking down the axis from its tip toward the origin (Z-up, right-handed).
/// - `q` and `−q` are the same rotation (turning by `angle` around `axis` is the same as turning
///   by 360° − angle around −axis).
///
/// Use when: storing an object's or camera's orientation; blending between two orientations without gimbal lock
/// Tags: quaternion, rotation, orientation, quat, slerp, gimbal lock, rotor
/// Node: none (value type)
struct Quaternion {
    /// The part along the rotation axis: axis × sin(angle / 2).
    Vector3 vectorPart;
    /// cos(angle / 2).
    float scalarPart;

    /// The quaternion with the given vector part and scalar part.
    constexpr Quaternion(Vector3 vectorPart, float scalarPart) : vectorPart(vectorPart), scalarPart(scalarPart) {}

    constexpr bool operator==(const Quaternion&) const = default;

    /// The rotation by `angle` around `axis`. `axis` is normalized here, so any length works; it
    /// must not be zero because a zero vector has no direction to turn around (a `MathProblem`
    /// is thrown).
    ///
    /// How: vectorPart = unitAxis × sin(angle / 2), scalarPart = cos(angle / 2). The result has
    /// length sqrt(sin² + cos²) = 1.
    static Quaternion fromAxis(Vector3 axis, Angle angle);

    /// The rotation that does nothing: vector part 0, scalar part 1 (an angle of 0).
    static constexpr Quaternion identity() { return Quaternion(Vector3(0, 0, 0), 1); }

    // MARK: Measuring

    /// The four-component dot product: u₁·u₂ + s₁·s₂.
    ///
    /// For two unit quaternions this is cos(θ/2), where θ is the angle of the turn that takes one
    /// rotation to the other. It is 1 for equal rotations; its sign tells which of the two
    /// equivalent quaternions (`q` or `−q`) is nearer.
    ///
    /// Use when: measuring how alike two rotations are before blending them; picking the shorter way round in slerp
    /// Tags: quaternion dot, 4D dot product, rotation similarity, quat dot, dot
    /// Node: none (value type)
    float dot(Quaternion other) const { return vectorPart.dot(other.vectorPart) + scalarPart * other.scalarPart; }

    /// The length of the four components: sqrt(x² + y² + z² + w²). Rotations have length 1.
    ///
    /// Use when: checking that a quaternion is a unit rotation; measuring drift after many multiplications
    /// Tags: quaternion length, magnitude, norm, unit quaternion check, quat length
    /// Node: none (value type)
    float length() const { return std::sqrt(dot(*this)); }

    /// The same quaternion scaled to length 1 (every component divided by `length`). Use it to
    /// clean up rounding drift after many products. A zero quaternion has no direction and throws
    /// a `MathProblem`.
    ///
    /// Use when: cleaning up a rotation after many multiplications; making an arbitrary quaternion usable as a rotation
    /// Tags: normalize quaternion, unit quaternion, renormalize, quat normalize, normalised
    /// Node: none (value type)
    Quaternion normalized() const;

    /// The opposite turn: the vector part flips sign, (u, s) becomes (−u, s). It turns by the same
    /// angle around the reversed axis. For a unit quaternion this is its inverse.
    ///
    /// Use when: undoing a rotation; turning a world direction into an object's local space
    /// Tags: quaternion conjugate, inverse rotation, reverse rotation, opposite turn, quat conjugate
    /// Node: none (value type)
    constexpr Quaternion conjugate() const { return Quaternion(-vectorPart, scalarPart); }

    /// Every component negated. It is the same rotation (see the type comment).
    constexpr Quaternion operator-() const { return Quaternion(-vectorPart, -scalarPart); }

    // MARK: Combining and applying

    /// The **Hamilton product**: one rotation that does both, first `other`, then this one. The
    /// order is the same as for matrices: `a * b` rotates by `b`, then by `a`.
    ///
    /// How (with this = (u₁, s₁) and other = (u₂, s₂)):
    ///
    ///     vectorPart = s₁·u₂ + s₂·u₁ + u₁ × u₂
    ///     scalarPart = s₁·s₂ − u₁·u₂
    ///
    /// This comes from multiplying out (s₁ + u₁)(s₂ + u₂) using i² = j² = k² = ijk = −1. The
    /// product is not commutative: the cross product u₁ × u₂ changes sign when the order does.
    /// The product of two unit quaternions is a unit quaternion.
    ///
    /// Use when: combining two rotations, such as a parent's and a child's; stacking a pitch on a yaw
    /// Tags: quaternion multiply, Hamilton product, combine rotations, compose rotations, quat mul
    /// Node: inside transform, instance
    Quaternion operator*(Quaternion other) const {
        return Quaternion(scalarPart * other.vectorPart + other.scalarPart * vectorPart + vectorPart.cross(other.vectorPart),
                          scalarPart * other.scalarPart - vectorPart.dot(other.vectorPart));
    }

    /// Turns `vector` by this rotation. Requires a unit quaternion (checked by an `assert`).
    /// Otherwise the vector is also scaled: (0, 0, 2 | 0) sends x to (−7, 0, 0).
    ///
    /// The definition is the sandwich q · (0, v) · q* (a quaternion with scalar part 0 and vector
    /// part v, with q* the conjugate). Multiplying that out for a unit q = (u, s) gives the faster
    /// form used here (the Euler–Rodrigues formula):
    ///
    ///     t  = 2 · (u × v)
    ///     v' = v + s·t + u × t
    ///
    /// Check: with u = axis·sin(a/2) and s = cos(a/2), s·t + u × t expands to
    /// sin a · (axis × v) + (1 − cos a) · axis × (axis × v), which is Rodrigues' rotation formula.
    /// It takes two cross products and no matrix.
    ///
    /// Use when: turning a point or direction by an orientation without building a matrix; orienting a bone or a camera offset
    /// Tags: rotate vector, apply quaternion, rotate point, quat rotate, transform direction
    /// Node: rotate, transform, instance
    Vector3 rotate(Vector3 vector) const;

    /// The same rotation as a `Matrix4` (no movement). Requires a unit quaternion (checked by an
    /// `assert`).
    ///
    /// Derivation: the columns of a rotation matrix are where the x, y and z axes land. Feed
    /// (1, 0, 0) to `rotate` with u = (x, y, z) and s = w: t = (0, 2z, −2y), so
    /// v' = (1, 0, 0) + w·t + u × t = (1 − 2y² − 2z², 2xy + 2wz, 2xz − 2wy). That is column 0. The
    /// other two columns work out the same way, giving
    ///
    ///     ┌ 1−2(y²+z²)   2(xy−wz)    2(xz+wy)   0 ┐
    ///     │ 2(xy+wz)     1−2(x²+z²)  2(yz−wx)   0 │
    ///     │ 2(xz−wy)     2(yz+wx)    1−2(x²+y²) 0 │
    ///     └ 0            0           0          1 ┘
    ///
    /// It is the same matrix as `Matrix4::rotation` for the same axis and angle.
    ///
    /// Use when: handing an orientation to a shader or multiplying it with translations and scales
    /// Tags: quaternion to matrix, rotation matrix, quat to mat4, convert rotation, matrix from quaternion
    /// Node: inside transform, instance
    Matrix4 matrix() const;

    // MARK: Blending

    /// A rotation part of the way from `start` to `end`, turning at a **constant speed** along the
    /// **shortest** path. `amount` 0 gives `start`, 1 gives `end`, 0.5 is halfway. Both must be
    /// unit quaternions. `amount` is not clamped.
    ///
    /// Spherical linear interpolation (Shoemake 1985). Unit quaternions are points on a 4D sphere,
    /// and a smooth turn is an arc on it. With φ = acos(start·end) (the arc between the two points):
    ///
    ///     slerp = (sin((1 − amount)·φ) · start + sin(amount·φ) · end) / sin φ
    ///
    /// The weights make the point travel along the arc by amount × φ, so equal steps of `amount`
    /// give equal steps of angle (a straight blend would speed up in the middle).
    ///
    /// **Shorter arc:** `q` and `−q` are the same rotation, but the arc to `−end` is the other way
    /// around the sphere. When start·end < 0 the arc is longer than a quarter of the sphere, so
    /// `end` is negated first.
    ///
    /// **Tiny angles:** when the two are almost equal, sin φ → 0 and the formula divides 0 by 0
    /// (and acos loses precision near 1). Then the blend is a normalized straight line, which is
    /// the same thing to within rounding.
    ///
    /// Use when: animating the camera between two views; blending a character's joint between two poses
    /// Tags: slerp, spherical interpolation, interpolate rotation, quaternion blend, quat slerp, rotation lerp
    /// Node: slerp
    static Quaternion slerp(Quaternion start, Quaternion end, float amount);

    // MARK: Alignment

    /// The rotation that turns the direction `direction` onto the direction `target` by the
    /// shortest way, around the axis perpendicular to both (Swift: `rotation(from:to:)`).
    ///
    /// Method (the half-way quaternion: Melax, *The Shortest Arc Quaternion*, Game Programming
    /// Gems 1, 2000; the problem itself is Hughes & Möller 1999, *Efficiently building a matrix to
    /// rotate one vector to another*, which builds a matrix instead). For unit directions a and b
    /// at angle θ apart, the rotation by θ around n = a × b / sin θ is (sin(θ/2) n, cos(θ/2)).
    /// Multiply that by 2 cos(θ/2) and the half-angles disappear:
    ///
    ///     vector part = 2 sin(θ/2) cos(θ/2) n = sin θ · n = a × b
    ///     scalar part = 2 cos²(θ/2)           = 1 + cos θ = 1 + a · b
    ///
    /// So (a × b, 1 + a · b) normalized is the answer, with no trigonometry and no angle:
    ///
    ///     b ●                 seen from the tip of a × b, the turn from a to b
    ///       ╱                     is counterclockwise (right-hand rule)
    ///      ╱ θ
    ///     ●────────● a
    ///
    /// Near opposite both parts are tiny differences of numbers close to 1, and computed that way
    /// they lose most of their digits (1 + a · b at 179.9999° is 1.5 × 10⁻¹², below the rounding of
    /// a · b). So both are computed from the sum s = a + b, whose components are exact differences
    /// when b ≈ −a:
    ///
    ///     a × b = a × (a + b)        (because a × a = 0)
    ///     1 + a · b = |a + b|² / 2   (because |a|² = |b|² = 1)
    ///
    /// The inputs may be any length: each is first reduced to a direction (`Vector3::unitDirection`).
    /// When the two are opposite to within |a × b| ≤ 10⁻⁷ the axis is undefined, since any axis
    /// perpendicular to a turns it onto −a. Then half a turn (180°) around some perpendicular axis
    /// is returned, chosen by crossing a with the coordinate axis it is least aligned with. That
    /// answer is off by at most about 10⁻⁷ rad, as is the half-way formula just outside the
    /// fallback.
    ///
    /// Conventions: turns follow the right-hand rule, like `fromAxis`. The result is a unit
    /// quaternion. Parallel directions give the identity. A zero-length or non-finite input has no
    /// direction, so the result has NaN in every component.
    ///
    /// Use when: pointing a branch, an arrow or a cylinder (modelled along +Z) along a given direction; aligning an object's axis with a surface normal
    /// Tags: rotation between two vectors, align vector, shortest arc, rotation from to, look rotation, FromToRotation, setFromUnitVectors, quaternion from two directions
    /// Node: inside instance
    static Quaternion rotation(Vector3 direction, Vector3 target);

private:
    /// Whether the length is 1 within `unitLengthTolerance`.
    bool isUnit() const;
};

} // namespace editor
