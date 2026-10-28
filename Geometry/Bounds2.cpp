// An axis-aligned rectangle on the plane (Engine/Math/Primitives/Bounds2.swift).
#include "Bounds2.h"

#include "../Linear/GeometryProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

Bounds2 Bounds2::checked(Vector2 minimum, Vector2 maximum) {
    if (!(minimum.x <= maximum.x && minimum.y <= maximum.y)) {
        throw GeometryProblem("Bounds2 needs minimum ≤ maximum on both axes.");
    }
    return Bounds2{minimum, maximum};
}

Bounds2 Bounds2::enclosing(const std::vector<Vector2>& points) {
    if (points.empty()) throw GeometryProblem("Cannot enclose an empty list of points.");
    Vector2 smallest = points[0];
    Vector2 largest = points[0];
    for (Vector2 point : points) {
        smallest = Vector2(smallerOf(smallest.x, point.x), smallerOf(smallest.y, point.y));
        largest = Vector2(largerOf(largest.x, point.x), largerOf(largest.y, point.y));
    }
    return checked(smallest, largest);
}

Bounds2 Bounds2::unionWith(const Bounds2& other) const {
    return enclosing({minimum, maximum, other.minimum, other.maximum});
}

bool Bounds2::contains(Vector2 point) const {
    return point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y && point.y <= maximum.y;
}

} // namespace editor
