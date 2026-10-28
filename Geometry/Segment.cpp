// A straight piece of line in 3D (Engine/Math/Primitives/Segment.swift).
#include "Segment.h"

#include "../Linear/Scalar.h"

namespace editor {

Vector3 Segment::closestPoint(Vector3 point) const {
    Vector3 direction = end - start;
    float squaredLength = direction.dot(direction);
    if (!(squaredLength > 0)) return start;

    float fraction = clamped((point - start).dot(direction) / squaredLength, 0, 1);
    return start + direction * fraction;
}

} // namespace editor
