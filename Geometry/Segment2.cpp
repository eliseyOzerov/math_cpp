// A straight piece of line in the plane (Engine/Math/Primitives/Segment2.swift).
#include "Segment2.h"

#include <cmath>

#include "../Linear/Scalar.h"

namespace editor {

Vector2 Segment2::closestPoint(Vector2 point) const {
    Vector2 direction = end - start;
    float squaredLength = direction.lengthSquared();
    if (!(squaredLength > 0)) return start;

    float fraction = clamped((point - start).dot(direction) / squaredLength, 0, 1);
    return start + direction * fraction;
}

SegmentFoot Segment2::closestFoot(Vector2 point) const {
    Vector2 direction = end - start;
    float squaredLength = direction.lengthSquared();
    if (!(squaredLength > 0)) return SegmentFoot{start, 0};

    float parameter = clamped((point - start).dot(direction) / squaredLength, 0, 1);
    return SegmentFoot{start + direction * parameter, parameter};
}

bool Segment2::isNear(Vector2 point, float tolerance, std::optional<float> overshoot) const {
    float allowedOvershoot = overshoot.value_or(tolerance);
    Vector2 direction = end - start;
    float length = direction.length();
    if (!(length > 1e-9f)) return false;
    float sideways = std::fabs((point - start).cross(direction)) / length;
    float beyondStart = (point - start).dot(direction) / length;
    float beyondEnd = (point - end).dot(direction) / length;
    return sideways <= tolerance && beyondStart >= -allowedOvershoot && beyondEnd <= allowedOvershoot;
}

std::vector<Segment2> Segment2::stitchedLoop(const std::vector<Segment2>& segments) {
    std::vector<Segment2> stitched = segments;
    size_t count = segments.size();
    for (size_t index = 0; index < count; index += 1) {
        stitched[index].start = segments[(index + count - 1) % count].end;
    }
    return stitched;
}

float Segment2::distance(Vector2 point) const { return point.distance(closestPoint(point)); }

} // namespace editor
