// An axis-aligned bounding box (Engine/Math/Primitives/Bounds.swift).
#include "Bounds.h"

#include "../Linear/GeometryProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

Bounds Bounds::checked(Vector3 minimum, Vector3 maximum) {
    if (!(minimum.x <= maximum.x && minimum.y <= maximum.y && minimum.z <= maximum.z)) {
        throw GeometryProblem("Bounds needs minimum ≤ maximum on every axis.");
    }
    return Bounds{minimum, maximum};
}

Bounds Bounds::fromCentre(Vector3 centre, Vector3 size) {
    Vector3 halfSize = size * 0.5f;
    return checked(centre - halfSize, centre + halfSize);
}

Bounds Bounds::enclosing(const std::vector<Vector3>& points) {
    if (points.empty()) throw GeometryProblem("Cannot enclose an empty list of points.");
    Vector3 smallest = points[0];
    Vector3 largest = points[0];
    for (Vector3 point : points) {
        smallest = Vector3(smallerOf(smallest.x, point.x), smallerOf(smallest.y, point.y),
                           smallerOf(smallest.z, point.z));
        largest = Vector3(largerOf(largest.x, point.x), largerOf(largest.y, point.y),
                          largerOf(largest.z, point.z));
    }
    return checked(smallest, largest);
}

Bounds Bounds::unionWith(const Bounds& other) const {
    return enclosing({minimum, maximum, other.minimum, other.maximum});
}

bool Bounds::contains(Vector3 point) const {
    return point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y && point.y <= maximum.y
        && point.z >= minimum.z && point.z <= maximum.z;
}

} // namespace editor
