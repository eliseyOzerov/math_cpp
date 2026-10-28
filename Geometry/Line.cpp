// An infinite line (Engine/Math/Primitives/Line.swift).
#include "Line.h"

#include "../Linear/GeometryProblem.h"
#include "../Linear/NumberText.h"
#include "../Linear/Scalar.h"

namespace editor {

Line::Line(Vector3 origin, Vector3 direction) : origin(origin) {
    std::optional<Vector3> unit = direction.unitDirection();
    if (!unit) {
        throw GeometryProblem("A line needs a finite direction that is not zero, but got " + vectorText(direction) + ".");
    }
    this->direction = *unit;
}

float Line::distanceAlong(Vector3 point) const { return (point - origin).dot(direction); }

Vector3 Line::closestPoint(Vector3 point) const { return origin + direction * distanceAlong(point); }

float Line::distance(Vector3 point) const {
    // Subtract the shadow from the offset, not the foot from the point: the foot
    // `origin + direction·s` is a big number when the line is far from the world's origin,
    // and rounding it costs digits that the offset (small near the line) still has.
    Vector3 offset = point - origin;
    return (offset - direction * offset.dot(direction)).length();
}

} // namespace editor
