// Points and directions in 3D space. The engine is Z-up and right-handed, in metres, exactly
// like the Swift engine's `Vector3` (Engine/Math/Vector3.swift).
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "Scalar.h"

namespace editor {

/// A point or a direction in 3D space: three 32-bit floats `x`, `y` and `z`.
///
/// Conventions: metres; Z is up and the axes are right-handed (x right, y forward, z up).
/// Arithmetic is component by component, like Swift's `SIMD3<Float>`.
///
/// Use when: storing a point of a mesh; moving a point along a direction
/// Tags: vec3, float3, point, direction, position, SIMD3
/// Node: none (a value the nodes pass around)
struct Vector3 {
    float x = 0;
    float y = 0;
    float z = 0;

    /// The origin, (0, 0, 0).
    constexpr Vector3() = default;
    /// The vector (x, y, z).
    constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    /// The vector with `value` on all three axes.
    ///
    /// Use when: reading one number as a vector, as the math nodes do with a number paired with a vector
    /// Tags: splat, broadcast, uniform vector, repeating
    /// Node: inside add, subtract, multiply, divide
    static constexpr Vector3 repeating(float value) { return Vector3(value, value, value); }

    /// The x axis, (1, 0, 0). It points right.
    ///
    /// Use when: naming the x direction instead of writing (1, 0, 0)
    /// Tags: x axis, unit x, right, Vector3.right, basis vector
    /// Node: none (constant)
    static constexpr Vector3 xAxis() { return Vector3(1, 0, 0); }
    /// The y axis, (0, 1, 0). In the Z-up world it points forward.
    ///
    /// Use when: naming the y direction instead of writing (0, 1, 0)
    /// Tags: y axis, unit y, forward, basis vector
    /// Node: none (constant)
    static constexpr Vector3 yAxis() { return Vector3(0, 1, 0); }
    /// The z axis, (0, 0, 1). In the Z-up world it points up.
    ///
    /// Use when: naming the world up direction instead of writing (0, 0, 1)
    /// Tags: z axis, unit z, up, world up, Z-up, basis vector
    /// Node: none (constant)
    static constexpr Vector3 zAxis() { return Vector3(0, 0, 1); }

    /// The component `index` (0 = x, 1 = y, 2 = z), the way Swift's `vector[index]` reads it.
    constexpr float operator[](int index) const { return index == 0 ? x : (index == 1 ? y : z); }

    constexpr Vector3 operator+(Vector3 other) const { return Vector3(x + other.x, y + other.y, z + other.z); }
    constexpr Vector3 operator-(Vector3 other) const { return Vector3(x - other.x, y - other.y, z - other.z); }
    constexpr Vector3 operator*(Vector3 other) const { return Vector3(x * other.x, y * other.y, z * other.z); }
    constexpr Vector3 operator/(Vector3 other) const { return Vector3(x / other.x, y / other.y, z / other.z); }
    constexpr Vector3 operator*(float scale) const { return Vector3(x * scale, y * scale, z * scale); }
    constexpr Vector3 operator/(float divisor) const { return Vector3(x / divisor, y / divisor, z / divisor); }
    constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }
    constexpr Vector3& operator+=(Vector3 other) { return *this = *this + other; }
    constexpr bool operator==(const Vector3&) const = default;

    /// The dot product: how far two vectors point the same way.
    ///
    /// How: x·x' + y·y' + z·z'. For unit vectors it is the cosine of the angle between them.
    ///
    /// Use when: measuring the angle between two directions; projecting onto an axis
    /// Tags: dot product, scalar product, inner product, dot()
    /// Node: planned (dot)
    constexpr float dot(Vector3 other) const { return x * other.x + y * other.y + z * other.z; }

    /// The cross product: a vector at right angles to both, as long as the area of the
    /// parallelogram they span.
    ///
    /// How: (y·z' − z·y', z·x' − x·z', x·y' − y·x'). Right-handed: x × y = z.
    ///
    /// Use when: finding the normal of a face; turning a side of a face inward
    /// Tags: cross product, vector product, perpendicular, normal, cross()
    /// Node: planned (cross)
    constexpr Vector3 cross(Vector3 other) const {
        return Vector3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }

    /// The squared length, x² + y² + z²: the vector's length without the square root.
    ///
    /// Use when: comparing distances without paying for a square root; testing whether a vector is longer than a tolerance
    /// Tags: length squared, squared magnitude, sqrMagnitude, norm squared, dot with self
    /// Node: inside length
    constexpr float lengthSquared() const { return dot(*this); }

    /// The length of the vector, in metres.
    ///
    /// How: √(x² + y² + z²).
    ///
    /// Use when: measuring the distance between two points; checking a vector is not zero
    /// Tags: length, magnitude, norm, euclidean length
    /// Node: planned (length)
    float length() const { return std::sqrt(dot(*this)); }

    /// The vector scaled to length 1, pointing the same way.
    ///
    /// How: the vector divided by its length. A zero-length vector has no direction and is
    /// returned unchanged, as in the Swift engine.
    ///
    /// Use when: turning a sum of face normals into a direction to push along
    /// Tags: normalize, unit vector, direction, normalized
    /// Node: planned (normalize)
    Vector3 normalized() const {
        float size = length();
        return size == 0 ? *this : *this / size;
    }

    /// The vector scaled to length 1, computed safely at every scale, or no value when it has no
    /// direction: zero length, or a component that is infinite or NaN.
    ///
    /// How: divide by the largest component magnitude first, so the length is between 1 and √3,
    /// then normalize. Squaring the components directly (`normalized`) underflows to 0 below
    /// about 1e-19 and overflows to infinity above about 1.8e19; dividing first keeps the
    /// direction exact to rounding from the smallest float (1e-45) to the largest (3e38). Every
    /// component is checked for finiteness on its own: a max of a number and NaN can return the
    /// number, so checking only the largest component would let a NaN through.
    ///
    /// Use when: turning a tangent, an axis or an offset of any size into a unit direction when it may be zero; validating a direction passed in by a recipe
    /// Tags: safe normalize, unit vector, direction, scale-safe normalization, normalize or nil, try normalize
    /// Node: inside normalize, rotate, coneDirection
    std::optional<Vector3> unitDirection() const {
        if (!isFinite()) return std::nullopt;
        float largestComponent = std::max({std::fabs(x), std::fabs(y), std::fabs(z)});
        if (!(largestComponent > 0)) return std::nullopt;
        return (*this / largestComponent).normalized();
    }

    /// Whether every component is a finite number (not infinite and not NaN).
    ///
    /// Use when: refusing a vector that came out of a division by zero
    /// Tags: finite, NaN check, infinity check, isfinite
    /// Node: none (input checking)
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }

    /// The point a fraction `fraction` of the way from `from` to `to`.
    ///
    /// How: from + (to − from)·fraction. 0 gives `from`, 1 gives `to`; outside 0…1 it
    /// extrapolates along the same line.
    ///
    /// Use when: the midpoint of an edge; blending a smooth point toward a sharp one
    /// Tags: lerp, mix, linear interpolation, blend
    /// Node: blend
    static constexpr Vector3 linearBlend(Vector3 from, Vector3 to, float fraction) {
        return from + (to - from) * fraction;
    }

    /// The straight-line distance between this point and `other`: the length of the vector
    /// from one to the other, |other − self|.
    ///
    /// Use when: measuring how far apart two points are; checking a pick radius
    /// Tags: distance, Vector3.Distance, length of difference, point distance, how far
    /// Node: distance
    float distance(Vector3 other) const { return (other - *this).length(); }

    /// Whether this vector is long enough to serve as the normal of a mirror plane in
    /// `reflectedAcrossPlane`: its length is more than `Tolerance::geometric`.
    ///
    /// Use when: checking that a mirror normal is usable before reflecting
    /// Tags: valid normal, mirror plane check, reflection normal, zero length check
    /// Node: inside reflect
    constexpr bool definesReflectionPlane() const { return lengthSquared() > Tolerance::geometric * Tolerance::geometric; }

    /// This vector mirrored in the plane through the origin whose normal is `planeNormal`.
    ///
    /// How (a Householder reflection):
    ///
    ///     self − 2·(self·planeNormal)/(planeNormal·planeNormal)·planeNormal
    ///
    ///          ╲ self      │ planeNormal ─▶
    ///           ╲          │
    ///     ───────●─────────┼──── mirror plane
    ///           ╱          │
    ///          ╱ result    │
    ///
    /// The part along `planeNormal` changes sign; the part within the plane is unchanged.
    /// Lengths are kept, and reflecting twice gives the original vector back.
    ///
    /// Conventions: `planeNormal` need not have length 1. One that does not
    /// `definesReflectionPlane` defines no plane, and the vector comes back unchanged.
    ///
    /// Source: Householder (1958); used this way by Wang et al. (2008) for rotation-minimizing frames.
    ///
    /// Use when: mirroring a point or direction across a plane, such as a reflection in a mirror or a symmetric model
    /// Tags: reflect, mirror, reflection across plane, mirror modifier, symmetry, Vector3.Reflect
    /// Node: reflect
    constexpr Vector3 reflectedAcrossPlane(Vector3 planeNormal) const {
        if (!planeNormal.definesReflectionPlane()) return *this;
        return *this - planeNormal * (2 * dot(planeNormal) / planeNormal.dot(planeNormal));
    }
};

/// `vector` scaled by `scale` on every axis.
constexpr Vector3 operator*(float scale, Vector3 vector) { return vector * scale; }

} // namespace editor
