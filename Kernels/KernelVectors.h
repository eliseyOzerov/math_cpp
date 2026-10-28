// The vector types shared kernels compute with, and the functions on them. On the GPU they are
// Metal's own vectors (`float3` is `Vector3Of<float>`), so a kernel costs nothing extra there; on
// the CPU they are the small structs below, with the same operators. A kernel never uses swizzles
// (`v.xy`) or Metal's vector functions directly: it builds vectors from components and calls the
// functions in this file, which work component by component through the prelude's scalar
// functions, so each formula exists once and the CPU and the GPU run the same steps.
//
// These are not the core's own `Vector3` (Math/Vector3.h), which stays the type for meshes and
// curves; kernels take these so they compile as Metal.
#pragma once

#include "KernelPrelude.h"

namespace editor {
namespace kernels {

#if defined(__METAL_VERSION__)

/// Two numbers (x, y): Metal's own vector on the GPU.
///
/// Use when: a point or direction on a plane, a texture coordinate, inside a kernel
/// Tags: vec2, float2, 2D vector, point, uv
/// Node: none (kernel type)
template <typename Real>
using Vector2Of = metal::vec<Real, 2>;

/// Three numbers (x, y, z): Metal's own vector on the GPU.
///
/// Use when: a point, a direction or a colour (red, green, blue) inside a kernel
/// Tags: vec3, float3, 3D vector, point, rgb
/// Node: none (kernel type)
template <typename Real>
using Vector3Of = metal::vec<Real, 3>;

/// Four numbers (x, y, z, w): Metal's own vector on the GPU.
///
/// Use when: a colour with opacity (red, green, blue, alpha), a point in 4D noise, inside a kernel
/// Tags: vec4, float4, 4D vector, rgba
/// Node: none (kernel type)
template <typename Real>
using Vector4Of = metal::vec<Real, 4>;

#else

/// Two numbers (x, y): on the CPU, a struct with Metal's constructors and operators.
///
/// Use when: a point or direction on a plane, a texture coordinate, inside a kernel
/// Tags: vec2, float2, 2D vector, point, uv
/// Node: none (kernel type)
template <typename Real>
struct Vector2Of {
    Real x;
    Real y;

    /// The zero vector.
    Vector2Of() : x(0), y(0) {}
    /// Every component set to `all`, like Metal's `float2(all)`.
    explicit Vector2Of(Real all) : x(all), y(all) {}
    /// The vector (first, second).
    Vector2Of(Real first, Real second) : x(first), y(second) {}
    /// Each component of `other` converted to `Real`, like Metal's `float2(int2)`.
    template <typename Other>
    explicit Vector2Of(Vector2Of<Other> other) : x(Real(other.x)), y(Real(other.y)) {}
};

/// Three numbers (x, y, z): on the CPU, a struct with Metal's constructors and operators.
///
/// Use when: a point, a direction or a colour (red, green, blue) inside a kernel
/// Tags: vec3, float3, 3D vector, point, rgb
/// Node: none (kernel type)
template <typename Real>
struct Vector3Of {
    Real x;
    Real y;
    Real z;

    /// The zero vector.
    Vector3Of() : x(0), y(0), z(0) {}
    /// Every component set to `all`, like Metal's `float3(all)`.
    explicit Vector3Of(Real all) : x(all), y(all), z(all) {}
    /// The vector (first, second, third).
    Vector3Of(Real first, Real second, Real third) : x(first), y(second), z(third) {}
    /// Each component of `other` converted to `Real`, like Metal's `float3(int3)`.
    template <typename Other>
    explicit Vector3Of(Vector3Of<Other> other) : x(Real(other.x)), y(Real(other.y)), z(Real(other.z)) {}
};

/// Four numbers (x, y, z, w): on the CPU, a struct with Metal's constructors and operators.
///
/// Use when: a colour with opacity (red, green, blue, alpha), a point in 4D noise, inside a kernel
/// Tags: vec4, float4, 4D vector, rgba
/// Node: none (kernel type)
template <typename Real>
struct Vector4Of {
    Real x;
    Real y;
    Real z;
    Real w;

    /// The zero vector.
    Vector4Of() : x(0), y(0), z(0), w(0) {}
    /// Every component set to `all`, like Metal's `float4(all)`.
    explicit Vector4Of(Real all) : x(all), y(all), z(all), w(all) {}
    /// The vector (first, second, third, fourth).
    Vector4Of(Real first, Real second, Real third, Real fourth) : x(first), y(second), z(third), w(fourth) {}
    /// Each component of `other` converted to `Real`, like Metal's `float4(int4)`.
    template <typename Other>
    explicit Vector4Of(Vector4Of<Other> other)
        : x(Real(other.x)), y(Real(other.y)), z(Real(other.z)), w(Real(other.w)) {}
};

// The operators Metal's vectors have built in, component by component. A scalar on either side
// of a binary operator applies to every component.

template <typename Real> inline Vector2Of<Real> operator-(Vector2Of<Real> value) { return Vector2Of<Real>(-value.x, -value.y); }
template <typename Real> inline Vector3Of<Real> operator-(Vector3Of<Real> value) { return Vector3Of<Real>(-value.x, -value.y, -value.z); }
template <typename Real> inline Vector4Of<Real> operator-(Vector4Of<Real> value) { return Vector4Of<Real>(-value.x, -value.y, -value.z, -value.w); }

// One macro writes the four forms of each arithmetic operator for each size (vector with vector,
// with a scalar on the right, with a scalar on the left, and the compound assignment); written
// out, the same three lines would repeat thirty-six times.
#define EDITOR_KERNEL_VECTOR_OPERATOR(symbol)                                                                       \
    template <typename Real> inline Vector2Of<Real> operator symbol(Vector2Of<Real> left, Vector2Of<Real> right) {   \
        return Vector2Of<Real>(left.x symbol right.x, left.y symbol right.y);                                     \
    }                                                                                                              \
    template <typename Real> inline Vector3Of<Real> operator symbol(Vector3Of<Real> left, Vector3Of<Real> right) {   \
        return Vector3Of<Real>(left.x symbol right.x, left.y symbol right.y, left.z symbol right.z);              \
    }                                                                                                              \
    template <typename Real> inline Vector4Of<Real> operator symbol(Vector4Of<Real> left, Vector4Of<Real> right) {   \
        return Vector4Of<Real>(left.x symbol right.x, left.y symbol right.y, left.z symbol right.z, left.w symbol right.w); \
    }                                                                                                              \
    template <typename Real> inline Vector2Of<Real> operator symbol(Vector2Of<Real> left, Real right) {              \
        return left symbol Vector2Of<Real>(right);                                                                 \
    }                                                                                                              \
    template <typename Real> inline Vector3Of<Real> operator symbol(Vector3Of<Real> left, Real right) {              \
        return left symbol Vector3Of<Real>(right);                                                                 \
    }                                                                                                              \
    template <typename Real> inline Vector4Of<Real> operator symbol(Vector4Of<Real> left, Real right) {              \
        return left symbol Vector4Of<Real>(right);                                                                 \
    }                                                                                                              \
    template <typename Real> inline Vector2Of<Real> operator symbol(Real left, Vector2Of<Real> right) {              \
        return Vector2Of<Real>(left) symbol right;                                                                 \
    }                                                                                                              \
    template <typename Real> inline Vector3Of<Real> operator symbol(Real left, Vector3Of<Real> right) {              \
        return Vector3Of<Real>(left) symbol right;                                                                 \
    }                                                                                                              \
    template <typename Real> inline Vector4Of<Real> operator symbol(Real left, Vector4Of<Real> right) {              \
        return Vector4Of<Real>(left) symbol right;                                                                 \
    }                                                                                                              \
    template <typename Real, typename Right> inline Vector2Of<Real>& operator symbol##=(Vector2Of<Real>& left, Right right) { \
        return left = left symbol right;                                                                           \
    }                                                                                                              \
    template <typename Real, typename Right> inline Vector3Of<Real>& operator symbol##=(Vector3Of<Real>& left, Right right) { \
        return left = left symbol right;                                                                           \
    }                                                                                                              \
    template <typename Real, typename Right> inline Vector4Of<Real>& operator symbol##=(Vector4Of<Real>& left, Right right) { \
        return left = left symbol right;                                                                           \
    }

EDITOR_KERNEL_VECTOR_OPERATOR(+)
EDITOR_KERNEL_VECTOR_OPERATOR(-)
EDITOR_KERNEL_VECTOR_OPERATOR(*)
EDITOR_KERNEL_VECTOR_OPERATOR(/)
#undef EDITOR_KERNEL_VECTOR_OPERATOR

// Comparisons give a vector of booleans, as Metal's do, for `chosen` to pick by.
#define EDITOR_KERNEL_VECTOR_COMPARISON(symbol)                                                                  \
    template <typename Real> inline Vector2Of<bool> operator symbol(Vector2Of<Real> left, Vector2Of<Real> right) {  \
        return Vector2Of<bool>(left.x symbol right.x, left.y symbol right.y);                                    \
    }                                                                                                             \
    template <typename Real> inline Vector3Of<bool> operator symbol(Vector3Of<Real> left, Vector3Of<Real> right) {  \
        return Vector3Of<bool>(left.x symbol right.x, left.y symbol right.y, left.z symbol right.z);             \
    }                                                                                                             \
    template <typename Real> inline Vector4Of<bool> operator symbol(Vector4Of<Real> left, Vector4Of<Real> right) {  \
        return Vector4Of<bool>(left.x symbol right.x, left.y symbol right.y, left.z symbol right.z, left.w symbol right.w); \
    }

EDITOR_KERNEL_VECTOR_COMPARISON(<)
EDITOR_KERNEL_VECTOR_COMPARISON(<=)
EDITOR_KERNEL_VECTOR_COMPARISON(>)
EDITOR_KERNEL_VECTOR_COMPARISON(>=)
#undef EDITOR_KERNEL_VECTOR_COMPARISON

#endif

/// The dot product of two vectors: the sum of their component products.
///
/// How: x₁x₂ + y₁y₂ (+ z₁z₂ + w₁w₂), added left to right.
///
/// Use when: the cosine between two unit directions; the length along an axis; weighting a colour
/// Tags: dot product, inner product, scalar product, dot()
/// Node: none (kernel helper)
template <typename Real>
inline Real dotProduct(Vector2Of<Real> first, Vector2Of<Real> second) {
    return first.x * second.x + first.y * second.y;
}
template <typename Real>
inline Real dotProduct(Vector3Of<Real> first, Vector3Of<Real> second) {
    return first.x * second.x + first.y * second.y + first.z * second.z;
}
template <typename Real>
inline Real dotProduct(Vector4Of<Real> first, Vector4Of<Real> second) {
    return first.x * second.x + first.y * second.y + first.z * second.z + first.w * second.w;
}

/// The cross product `first × second`: the vector at right angles to both, as long as the area
/// of the parallelogram they span.
///
/// How: (y₁z₂ − z₁y₂, z₁x₂ − x₁z₂, x₁y₂ − y₁x₂). Right-handed: x × y = z.
///
/// Use when: a surface normal from two edges; a perpendicular direction
/// Tags: cross product, vector product, normal, cross()
/// Node: none (kernel helper)
template <typename Real>
inline Vector3Of<Real> crossProduct(Vector3Of<Real> first, Vector3Of<Real> second) {
    return Vector3Of<Real>(first.y * second.z - first.z * second.y, first.z * second.x - first.x * second.z,
        first.x * second.y - first.y * second.x);
}

/// The length of a vector.
///
/// How: √(v · v). No guard against overflow for components above about 1e19.
///
/// Use when: the distance of a point from the origin; the size of a step
/// Tags: length, magnitude, norm, Euclidean length, length()
/// Node: none (kernel helper)
template <typename Real>
inline Real lengthOf(Vector2Of<Real> value) {
    return squareRoot(dotProduct(value, value));
}
template <typename Real>
inline Real lengthOf(Vector3Of<Real> value) {
    return squareRoot(dotProduct(value, value));
}
template <typename Real>
inline Real lengthOf(Vector4Of<Real> value) {
    return squareRoot(dotProduct(value, value));
}

/// The same direction as `value`, with length 1.
///
/// How: `value / |value|`. A zero vector gives NaN components, as Metal's `normalize` does; the
/// caller checks when zero is possible.
///
/// Use when: a direction from a difference of two points; a normal
/// Tags: normalize, unit vector, direction, normalized
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> normalized(Vector2Of<Real> value) {
    return value / lengthOf(value);
}
template <typename Real>
inline Vector3Of<Real> normalized(Vector3Of<Real> value) {
    return value / lengthOf(value);
}

/// The mirror image of `incident` about the plane with unit normal `normal` (GLSL `reflect`).
///
/// How: incident − 2 (normal · incident) normal.
///
/// Conventions: `normal` must have length 1; `incident` points towards the surface.
///
/// Use when: mirror reflection directions; the specular term of Phong shading
/// Tags: reflect, mirror, bounce, GLSL reflect
/// Node: none (kernel helper)
template <typename Real>
inline Vector3Of<Real> reflectedAbout(Vector3Of<Real> incident, Vector3Of<Real> normal) {
    return incident - normal * (Real(2.0f) * dotProduct(normal, incident));
}
template <typename Real>
inline Vector4Of<Real> normalized(Vector4Of<Real> value) {
    return value / lengthOf(value);
}

// The prelude's scalar functions, applied to each component. Each vector form is the scalar
// form, per component, so CPU and GPU results agree component for component.

/// Each component without its sign (the prelude's `absolute`, per component).
///
/// Use when: folding a shape into one quadrant; a box's signed distance
/// Tags: abs, absolute value, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> absolute(Vector2Of<Real> value) {
    return Vector2Of<Real>(absolute(value.x), absolute(value.y));
}
template <typename Real>
inline Vector3Of<Real> absolute(Vector3Of<Real> value) {
    return Vector3Of<Real>(absolute(value.x), absolute(value.y), absolute(value.z));
}
template <typename Real>
inline Vector4Of<Real> absolute(Vector4Of<Real> value) {
    return Vector4Of<Real>(absolute(value.x), absolute(value.y), absolute(value.z), absolute(value.w));
}

/// Each component rounded down (the prelude's `roundedDown`, per component).
///
/// Use when: the grid cell a point lies in
/// Tags: floor, round down, cell, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> roundedDown(Vector2Of<Real> value) {
    return Vector2Of<Real>(roundedDown(value.x), roundedDown(value.y));
}
template <typename Real>
inline Vector3Of<Real> roundedDown(Vector3Of<Real> value) {
    return Vector3Of<Real>(roundedDown(value.x), roundedDown(value.y), roundedDown(value.z));
}
template <typename Real>
inline Vector4Of<Real> roundedDown(Vector4Of<Real> value) {
    return Vector4Of<Real>(roundedDown(value.x), roundedDown(value.y), roundedDown(value.z), roundedDown(value.w));
}

/// Each component's fractional part (the prelude's `fractionalPart`, per component).
///
/// Use when: the position inside a grid cell
/// Tags: fract, fractional part, cell position, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> fractionalPart(Vector2Of<Real> value) {
    return value - roundedDown(value);
}
template <typename Real>
inline Vector3Of<Real> fractionalPart(Vector3Of<Real> value) {
    return value - roundedDown(value);
}
template <typename Real>
inline Vector4Of<Real> fractionalPart(Vector4Of<Real> value) {
    return value - roundedDown(value);
}

/// Each component's sign (the prelude's `signOf`, per component).
///
/// Use when: which quadrant a point is in; mirroring a formula into every quadrant
/// Tags: sign, signum, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> signOf(Vector2Of<Real> value) {
    return Vector2Of<Real>(signOf(value.x), signOf(value.y));
}
template <typename Real>
inline Vector3Of<Real> signOf(Vector3Of<Real> value) {
    return Vector3Of<Real>(signOf(value.x), signOf(value.y), signOf(value.z));
}

/// The smaller of each pair of components (the prelude's `smallerOf`, per component).
///
/// Use when: the lower corner of two boxes; clipping a colour
/// Tags: min, minimum, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> smallerOf(Vector2Of<Real> first, Vector2Of<Real> second) {
    return Vector2Of<Real>(smallerOf(first.x, second.x), smallerOf(first.y, second.y));
}
template <typename Real>
inline Vector3Of<Real> smallerOf(Vector3Of<Real> first, Vector3Of<Real> second) {
    return Vector3Of<Real>(smallerOf(first.x, second.x), smallerOf(first.y, second.y), smallerOf(first.z, second.z));
}
template <typename Real>
inline Vector4Of<Real> smallerOf(Vector4Of<Real> first, Vector4Of<Real> second) {
    return Vector4Of<Real>(smallerOf(first.x, second.x), smallerOf(first.y, second.y), smallerOf(first.z, second.z),
        smallerOf(first.w, second.w));
}

/// The larger of each pair of components (the prelude's `largerOf`, per component).
///
/// Use when: the upper corner of two boxes; a box's signed distance outside
/// Tags: max, maximum, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> largerOf(Vector2Of<Real> first, Vector2Of<Real> second) {
    return Vector2Of<Real>(largerOf(first.x, second.x), largerOf(first.y, second.y));
}
template <typename Real>
inline Vector3Of<Real> largerOf(Vector3Of<Real> first, Vector3Of<Real> second) {
    return Vector3Of<Real>(largerOf(first.x, second.x), largerOf(first.y, second.y), largerOf(first.z, second.z));
}
template <typename Real>
inline Vector4Of<Real> largerOf(Vector4Of<Real> first, Vector4Of<Real> second) {
    return Vector4Of<Real>(largerOf(first.x, second.x), largerOf(first.y, second.y), largerOf(first.z, second.z),
        largerOf(first.w, second.w));
}

/// Each component moved into `lower`…`upper` (the prelude's `clamped`, per component).
///
/// Use when: keeping a colour inside 0…1
/// Tags: clamp, saturate, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> clamped(Vector2Of<Real> value, Real lower, Real upper) {
    return Vector2Of<Real>(clamped(value.x, lower, upper), clamped(value.y, lower, upper));
}
template <typename Real>
inline Vector3Of<Real> clamped(Vector3Of<Real> value, Real lower, Real upper) {
    return Vector3Of<Real>(clamped(value.x, lower, upper), clamped(value.y, lower, upper), clamped(value.z, lower, upper));
}
template <typename Real>
inline Vector4Of<Real> clamped(Vector4Of<Real> value, Real lower, Real upper) {
    return Vector4Of<Real>(clamped(value.x, lower, upper), clamped(value.y, lower, upper), clamped(value.z, lower, upper),
        clamped(value.w, lower, upper));
}

/// The vector a fraction `amount` of the way from `start` to `end` (the prelude's
/// `linearBlend`, per component).
///
/// Use when: fading between two colours; moving a point along a segment
/// Tags: lerp, mix, blend, componentwise
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> linearBlend(Vector2Of<Real> start, Vector2Of<Real> end, Real amount) {
    return start + (end - start) * amount;
}
template <typename Real>
inline Vector3Of<Real> linearBlend(Vector3Of<Real> start, Vector3Of<Real> end, Real amount) {
    return start + (end - start) * amount;
}
template <typename Real>
inline Vector4Of<Real> linearBlend(Vector4Of<Real> start, Vector4Of<Real> end, Real amount) {
    return start + (end - start) * amount;
}

/// Per component, `ifTrue`'s value where `condition` holds and `ifFalse`'s elsewhere (the
/// prelude's `chosen`, per component; Metal's `select` with the arguments in reading order).
///
/// Use when: picking per channel without an `if`, such as the two branches of the sRGB curve
/// Tags: select, choose, componentwise, mask
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> chosen(Vector2Of<bool> condition, Vector2Of<Real> ifTrue, Vector2Of<Real> ifFalse) {
    return Vector2Of<Real>(chosen(bool(condition.x), ifTrue.x, ifFalse.x), chosen(bool(condition.y), ifTrue.y, ifFalse.y));
}
template <typename Real>
inline Vector3Of<Real> chosen(Vector3Of<bool> condition, Vector3Of<Real> ifTrue, Vector3Of<Real> ifFalse) {
    return Vector3Of<Real>(chosen(bool(condition.x), ifTrue.x, ifFalse.x), chosen(bool(condition.y), ifTrue.y, ifFalse.y),
        chosen(bool(condition.z), ifTrue.z, ifFalse.z));
}
template <typename Real>
inline Vector4Of<Real> chosen(Vector4Of<bool> condition, Vector4Of<Real> ifTrue, Vector4Of<Real> ifFalse) {
    return Vector4Of<Real>(chosen(bool(condition.x), ifTrue.x, ifFalse.x), chosen(bool(condition.y), ifTrue.y, ifFalse.y),
        chosen(bool(condition.z), ifTrue.z, ifFalse.z), chosen(bool(condition.w), ifTrue.w, ifFalse.w));
}

/// `value` moved into the range `lower`…`upper`, component by component, where the bounds are the
/// same type as `value` (the prelude's `clamped` takes number bounds).
///
/// How: smallerOf(largerOf(value, lower), upper), the prelude's tie and NaN rules.
///
/// Use when: clamping a vector between two vectors; clamping inside code written for numbers and vectors alike
/// Tags: clamp, per component clamp, saturate, clamp()
/// Node: none (kernel helper)
template <typename Value>
inline Value clampedPerComponent(Value value, Value lower, Value upper) {
    return smallerOf(largerOf(value, lower), upper);
}

/// `start + (end − start) × amount`, component by component, where `amount` is the same type as
/// the others (the prelude's `linearBlend` takes a number).
///
/// How: the formula of `linearBlend`; `amount` is not clamped.
///
/// Use when: blending two vectors by a different fraction per component
/// Tags: lerp, mix, per component blend, mix()
/// Node: none (kernel helper)
template <typename Value>
inline Value linearBlendPerComponent(Value start, Value end, Value amount) {
    return start + (end - start) * amount;
}

#if !defined(__METAL_VERSION__)
// Metal's sin, pow and log already take vectors, and the prelude's functions call them; on the
// CPU the standard library does not, so these apply the scalar functions per component.

/// The sine of each component of `radians`. Use when: a wave over a vector. Tags: sin, sine
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> sine(Vector2Of<Real> radians) { return Vector2Of<Real>(sine(radians.x), sine(radians.y)); }
template <typename Real>
inline Vector3Of<Real> sine(Vector3Of<Real> radians) { return Vector3Of<Real>(sine(radians.x), sine(radians.y), sine(radians.z)); }
template <typename Real>
inline Vector4Of<Real> sine(Vector4Of<Real> radians) {
    return Vector4Of<Real>(sine(radians.x), sine(radians.y), sine(radians.z), sine(radians.w));
}

/// Each component of `base` raised to the matching component of `exponent`. Use when: a power
/// over a vector. Tags: pow, power
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> power(Vector2Of<Real> base, Vector2Of<Real> exponent) {
    return Vector2Of<Real>(power(base.x, exponent.x), power(base.y, exponent.y));
}
template <typename Real>
inline Vector3Of<Real> power(Vector3Of<Real> base, Vector3Of<Real> exponent) {
    return Vector3Of<Real>(power(base.x, exponent.x), power(base.y, exponent.y), power(base.z, exponent.z));
}
template <typename Real>
inline Vector4Of<Real> power(Vector4Of<Real> base, Vector4Of<Real> exponent) {
    return Vector4Of<Real>(power(base.x, exponent.x), power(base.y, exponent.y), power(base.z, exponent.z),
        power(base.w, exponent.w));
}

/// The natural logarithm of each component. Use when: a logarithm over a vector. Tags: log, ln
/// Node: none (kernel helper)
template <typename Real>
inline Vector2Of<Real> naturalLogarithm(Vector2Of<Real> value) {
    return Vector2Of<Real>(naturalLogarithm(value.x), naturalLogarithm(value.y));
}
template <typename Real>
inline Vector3Of<Real> naturalLogarithm(Vector3Of<Real> value) {
    return Vector3Of<Real>(naturalLogarithm(value.x), naturalLogarithm(value.y), naturalLogarithm(value.z));
}
template <typename Real>
inline Vector4Of<Real> naturalLogarithm(Vector4Of<Real> value) {
    return Vector4Of<Real>(naturalLogarithm(value.x), naturalLogarithm(value.y), naturalLogarithm(value.z),
        naturalLogarithm(value.w));
}
#endif

} // namespace kernels
} // namespace editor
