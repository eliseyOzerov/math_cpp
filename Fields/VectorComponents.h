// Small componentwise helpers that the shape formulas share (Engine/Math/Fields/VectorComponents.swift
// and VectorComponents+Smallest.swift). Each acts on x, y and z independently. Swift's properties
// and methods are free functions here, because Vector3.h stays small.
#pragma once

#include <cmath>

#include "../Linear/Scalar.h"
#include "../Linear/Vector3.h"

namespace editor {

/// The vector with each component made non-negative: `(|x|, |y|, |z|)`.
///
/// Use when: folding space into one octant, so a box formula only handles the positive side
/// Tags: abs, absolute value, componentwise abs, fold, mirror symmetry, fabs
/// Node: absolute
inline Vector3 componentwiseAbsolute(Vector3 vector) {
    return Vector3(std::fabs(vector.x), std::fabs(vector.y), std::fabs(vector.z));
}

/// The vector with each component raised to at least `floor`: `(max(x, floor), …)`.
///
/// Use when: keeping only the part of an offset that sticks out past a box face (floor 0)
/// Tags: max, componentwise max, clamp below, max(v, 0.0), pointwise maximum
/// Node: inside boxField
inline Vector3 componentwiseMaximum(Vector3 vector, float floor) {
    return Vector3(largerOf(vector.x, floor), largerOf(vector.y, floor), largerOf(vector.z, floor));
}

/// The largest of x, y and z.
///
/// Use when: finding how far inside a box a point is (the least-deep axis decides)
/// Tags: max component, maxcomp, largest axis, horizontal max
/// Node: inside boxField
inline float largestComponent(Vector3 vector) {
    return largerOf(vector.x, largerOf(vector.y, vector.z));
}

/// The smallest of x, y and z.
///
/// Use when: finding an ellipsoid's shortest radius; the weakest stretch of a non-uniform scale
/// Tags: min component, mincomp, smallest axis, horizontal min
/// Node: inside boxField
inline float smallestComponent(Vector3 vector) {
    return smallerOf(vector.x, smallerOf(vector.y, vector.z));
}

} // namespace editor
