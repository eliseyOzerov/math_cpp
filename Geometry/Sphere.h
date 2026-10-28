// A sphere (Engine/Math/Primitives/Sphere.swift).
#pragma once

#include "../Linear/GeometryProblem.h"
#include "../Linear/Vector3.h"

namespace editor {

/// The ball of points within `radius` of `centre`. Its surface is the points exactly `radius` away.
///
/// Use when: describing a ball, a bounding sphere or a light's reach
/// Tags: sphere, ball, centre and radius, bounding sphere, round shape
/// Node: sphereField
struct Sphere {
    Vector3 centre;
    float radius = 0;

    /// A sphere around `centre` that reaches `radius` in every direction. A negative radius
    /// throws a `GeometryProblem` (Swift traps).
    Sphere(Vector3 centre, float radius) : centre(centre), radius(radius) {
        if (!(radius >= 0)) throw GeometryProblem("A sphere cannot have a negative radius.");
    }

    bool operator==(const Sphere&) const = default;
};

} // namespace editor
