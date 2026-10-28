// Four numbers for the rows and columns of a Matrix4 and for homogeneous points
// (Engine/Math/Vector.swift, `Vector4`). `perspectiveDivided` is camera-only and not ported.
#pragma once

#include "Vector3.h"

namespace editor {

/// Four numbers: x, y, z and w. Mostly used for the rows and columns of a `Matrix4`, and for
/// "homogeneous" points (see `Matrix4`): a point has w = 1, a direction w = 0.
///
/// Use when: reading a row or column of a Matrix4; applying a Matrix4 to a point or a direction
/// Tags: vec4, float4, SIMD4, homogeneous coordinates, row, column
/// Node: none (a value the matrices pass around)
struct Vector4 {
    float x = 0;
    float y = 0;
    float z = 0;
    float w = 0;

    constexpr Vector4() = default;
    /// The vector (x, y, z, w).
    constexpr Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    /// The vector (xyz.x, xyz.y, xyz.z, w): a point with w = 1, a direction with w = 0.
    constexpr Vector4(Vector3 xyz, float w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}

    /// The component `index` (0 = x, 1 = y, 2 = z, 3 = w), the way Swift's `vector[index]` reads it.
    constexpr float operator[](int index) const {
        switch (index) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        default: return w;
        }
    }

    constexpr Vector4 operator+(Vector4 other) const { return Vector4(x + other.x, y + other.y, z + other.z, w + other.w); }
    constexpr Vector4 operator-(Vector4 other) const { return Vector4(x - other.x, y - other.y, z - other.z, w - other.w); }
    constexpr Vector4 operator*(float scale) const { return Vector4(x * scale, y * scale, z * scale, w * scale); }
    constexpr Vector4 operator-() const { return Vector4(-x, -y, -z, -w); }
    constexpr bool operator==(const Vector4&) const = default;

    /// The first three components as a Vector3.
    constexpr Vector3 xyz() const { return Vector3(x, y, z); }

    /// Whether every component is a finite number (not infinite and not NaN).
    ///
    /// Use when: checking that a matrix row holds no infinity or NaN
    /// Tags: finite, NaN check, infinity check, isfinite
    /// Node: none (input checking)
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w); }
};

constexpr Vector4 operator*(float scale, Vector4 vector) { return vector * scale; }

} // namespace editor
