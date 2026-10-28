// An endless flat sheet (Engine/Math/Primitives/Plane.swift).
#include "Plane.h"

#include "../Linear/GeometryProblem.h"
#include "Triangle.h"

namespace editor {

Plane::Plane(Vector3 point, Vector3 normal) {
    if (!(normal.length() > 0)) throw GeometryProblem("A plane needs a normal that is not zero.");
    Vector3 unitNormal = normal.normalized();
    this->normal = unitNormal;
    distance = unitNormal.dot(point);
}

namespace {

/// (second − first) × (third − first), after checking the corners are not in a line.
Vector3 scaledNormalOfNonDegenerateTriangle(Vector3 first, Vector3 second, Vector3 third) {
    Triangle corners{first, second, third};
    if (corners.isDegenerate()) throw GeometryProblem("Three corners in a line do not define a plane.");
    return (second - first).cross(third - first);
}

} // namespace

Plane::Plane(Vector3 first, Vector3 second, Vector3 third)
    : Plane(first, scaledNormalOfNonDegenerateTriangle(first, second, third)) {}

float Plane::signedDistance(Vector3 point) const { return normal.dot(point) - distance; }

Vector3 Plane::projection(Vector3 point) const { return point - normal * signedDistance(point); }

} // namespace editor
