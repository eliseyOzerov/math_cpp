// A straight piece of line in the plane (Engine/Math/Primitives/Segment2.swift).
#include "Segment2.h"

#include "../Linear/Scalar.h"

namespace editor {

Vector2 Segment2::closestPoint(Vector2 point) const {
    Vector2 direction = end - start;
    float squaredLength = direction.lengthSquared();
    if (!(squaredLength > 0)) return start;

    float fraction = clamped((point - start).dot(direction) / squaredLength, 0, 1);
    return start + direction * fraction;
}

float Segment2::distance(Vector2 point) const { return point.distance(closestPoint(point)); }

} // namespace editor
