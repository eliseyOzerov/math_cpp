// Where an object is, which way it faces and how big it is (Engine/Scene/Transform.swift). The
// Codable parts (reading a transform from JSON, with its unit-length check) are left to the graph
// code that needs them. `MirrorAxis` is declared here until the skeleton is ported
// (Engine/Skeleton/Skeleton.swift).
#pragma once

#include <optional>

#include "Matrix3.h"
#include "Matrix4.h"
#include "Quaternion.h"

namespace editor {

/// The plane a skeleton is mirrored across: the one that is perpendicular to this axis and passes
/// through the origin. `x` swaps left and right of a character that faces along Y.
///
/// Use when: building one side of a symmetric rig and copying it to the other
/// Tags: mirror axis, symmetry plane, reflection axis, left right mirror
/// Node: none (a value the nodes pass around)
enum class MirrorAxis { x, y, z };

/// Where an object is, which way it faces and how big it is: a translation, a rotation and a
/// scale, kept as three separate values so each can be edited (or animated) on its own.
///
/// The three combine into one matrix that takes a point from the object's own space to the
/// world, in a fixed order:
///
///     matrix = T · R · S
///
/// The scale happens first (stretching along the object's own axes), then the rotation, then the
/// move. So `scale` stretches the object before it is turned, and `translation` is simply where
/// the object's origin ends up in the world.
///
/// Swift's memberwise `Transform(translation:rotation:scale:)` is aggregate initialisation here:
/// `Transform{.translation = ..., .rotation = ...}` leaves the rest at their identity values.
///
/// Use when: placing, turning and sizing an object of a scene
/// Tags: transform, TRS, position rotation scale, object transform, model matrix, local to world
/// Node: none (value type)
struct Transform {
    /// Where the object's origin is in the world.
    Vector3 translation = Vector3(0, 0, 0);
    /// How the object is turned. Must be a unit quaternion.
    Quaternion rotation = Quaternion::identity();
    /// How much the object is stretched along its own x, y and z axes before it is turned.
    Vector3 scale = Vector3(1, 1, 1);

    bool operator==(const Transform&) const = default;

    /// The transform that changes nothing.
    static Transform identity() { return Transform{}; }

    /// The matrix T · R · S that takes object-space points to world space.
    ///
    /// How: `Matrix4::translation(translation) * rotation.matrix() * Matrix4::scale(scale)` (the
    /// rightmost matrix is applied first, see `Matrix4`). For a point p that is
    /// rotation.rotate(scale ⊙ p) + translation, with ⊙ the per-axis product.
    ///
    /// Use when: sending an object's placement to the shader; moving its points to the world
    /// Tags: model matrix, TRS matrix, transform matrix, local to world matrix, object matrix
    /// Node: none (value type)
    Matrix4 matrix() const;

    /// The matrix that undoes `matrix` (world space back to object space), or no value if the
    /// scale flattens the object so it cannot be undone: a scale component that is 0, or so small
    /// that 1 / scale overflows (1e-39 is a float's denormal range) or the resulting matrix has an
    /// entry that is infinite or NaN.
    ///
    /// How: S⁻¹ · Rᵀ · T⁻¹. The inverse of a product reverses the order and inverts each part:
    /// moving back by −translation, turning back by the conjugate quaternion (the inverse of a
    /// unit rotation), then stretching by 1 / scale. No general matrix inverse is needed. The
    /// answer is accepted only if every entry is finite, the same test `Matrix3::inverse` applies.
    ///
    /// Use when: moving a ray or a point from the world into an object's own space, for picking
    /// Tags: inverse matrix, world to local, inverse transform, undo transform, picking
    /// Node: none (value type)
    std::optional<Matrix4> inverseMatrix() const;

    /// Whether the transform turns the object inside out, as a mirror does: an odd number of its
    /// scale factors is negative, so its matrix has a negative determinant.
    ///
    /// How: rotation and translation keep handedness, so the sign of the determinant is the sign
    /// of scale.x · scale.y · scale.z. A mirrored object's triangles, wound counterclockwise from
    /// outside in the mesh, appear clockwise from outside once placed.
    ///
    /// Use when: deciding which winding is the front face when drawing a mirrored object
    /// Tags: mirrored, negative scale, reflection, handedness flip, negative determinant, winding
    /// Node: none (renderer shading)
    bool reversesHandedness() const { return scale.x * scale.y * scale.z < 0; }

    /// The matrix that turns the object's surface normals into world space, or no value if the
    /// scale flattens the object (see `Matrix3::normalMatrix`).
    ///
    /// Why not `matrix` itself: under non-uniform scale it would tilt normals away from
    /// perpendicular. The normal matrix is the inverse transpose of the upper-left 3×3 of
    /// `matrix`, which keeps them perpendicular. Normalize the result after using it.
    ///
    /// Use when: lighting a scaled or squashed object
    /// Tags: normal matrix, inverse transpose, transform normals, lighting matrix
    /// Node: none (renderer shading)
    std::optional<Matrix3> normalMatrix() const;

    // MARK: Rigid transforms

    /// The rigid transform that does `child` first and then this one: the frame `child` describes
    /// inside the frame this one describes, seen from outside this one. Its matrix is
    /// `matrix() * child.matrix()`.
    ///
    /// How: a rigid transform takes a point p to R·p + t. Doing `child` and then this one:
    ///
    ///     R₁·(R₂·p + t₂) + t₁ = (R₁·R₂)·p + (R₁·t₂ + t₁)
    ///
    /// so the rotation is the quaternion product R₁ · R₂ and the translation is t₂ turned by R₁
    /// and moved by t₁.
    ///
    ///     world ──self──▶ parent frame ──child──▶ child frame
    ///
    /// Conventions: both must be rigid (scale 1, unit rotation); scale is ignored and the result
    /// has scale 1. The rotation stays unit up to rounding.
    ///
    /// Use when: finding where a bone's socket is in the world from the bone's placement; chaining a parent's frame and a child's frame
    /// Tags: compose transforms, chain frames, parent times child, rigid composition, local to world, multiply transforms
    /// Node: none (value type)
    Transform composed(const Transform& child) const;

    /// The rigid transform that undoes this one: it takes the world back into the frame this one
    /// describes, and `composed` with it gives the identity either way round.
    ///
    /// How: solving q = R·p + t for p gives p = R⁻¹·q − R⁻¹·t. A unit quaternion's inverse is its
    /// conjugate, so the rotation is R* and the translation is −R*·t.
    ///
    /// Conventions: must be rigid (scale 1, unit rotation); scale is ignored and the result has
    /// scale 1.
    ///
    /// Use when: finding a child's frame relative to its parent from both their world frames; laying a socket of one piece onto a socket of another
    /// Tags: inverse transform, undo transform, rigid inverse, world to local, invert frame
    /// Node: none (value type)
    Transform rigidInverse() const;

    /// The rigid transform seen in a mirror across the plane where the coordinate of `axis` is 0:
    /// M · self · M, where M is the reflection that negates that coordinate.
    ///
    /// How: a point p goes to M·(R·(M·p) + t) = (M·R·M)·p + M·t. So the translation has its
    /// component along `axis` negated. M·R·M is again a proper rotation (the two reflections cancel
    /// each other's change of handedness): the same angle about the axis −M·a. For the quaternion
    /// (v, s) that keeps s and the component of v along `axis` and negates the other two.
    ///
    ///     mirror x = 0:   frame at (+0.2, 0, 1), turned toward +x
    ///                ──▶  frame at (−0.2, 0, 1), turned toward −x
    ///
    /// Conventions: rigid in, rigid out; the scale is kept. Negation is exact in floating point, so
    /// mirroring twice gives the original bit for bit. A frame placed by the result is a proper
    /// (right-handed) frame; to reflect a mesh too, also negate its scale along `axis`.
    ///
    /// Use when: placing a right bone from its left twin; mirroring a frame of a symmetric model
    /// Tags: mirror transform, reflect frame, symmetric twin, flip left to right, conjugate by reflection
    /// Node: none (value type)
    Transform mirrored(MirrorAxis axis) const;
};

} // namespace editor
