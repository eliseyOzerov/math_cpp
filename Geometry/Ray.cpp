// A half-line (Engine/Math/Primitives/Ray.swift).
#include "Ray.h"

#include "../Linear/GeometryProblem.h"

namespace editor {

Ray::Ray(Vector3 origin, Vector3 direction) : origin(origin) {
    if (!(direction.length() > 0)) throw GeometryProblem("A ray needs a direction that is not zero.");
    this->direction = direction.normalized();
}

Vector3 Ray::point(float distance) const { return origin + direction * distance; }

} // namespace editor
