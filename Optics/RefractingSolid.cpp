// A convex glass solid and the path of a ray through it (Engine/Math/Optics/RefractingSolid.swift).
#include "RefractingSolid.h"

#include <cassert>

#include "../Linear/Scalar.h"

namespace editor {

std::optional<Vector3> RayLeg::endPoint() const {
    if (!length) return std::nullopt;
    return start.ray.point(*length);
}

std::optional<BoundaryHit> RefractingSolid::firstBoundaryHit(Ray ray, bool startsInside) const {
    std::optional<BoundaryHit> enter;
    std::optional<BoundaryHit> leave;
    for (const Plane& face : faces) {
        float approachSpeed = face.normal.dot(ray.direction);
        float signedDistance = face.signedDistance(ray.origin);
        if (isNearlyZero(approachSpeed)) {
            if (signedDistance > 0 && !startsInside) return std::nullopt;
            continue;
        }
        float crossing = -signedDistance / approachSpeed;
        if (approachSpeed < 0) {
            if (!enter || crossing > enter->distance) enter = BoundaryHit{crossing, face.normal};
        } else if (startsInside && crossing <= 0) {
            // A ray inside can find a leaving crossing at t ≤ 0 by rounding when it has
            // just hit an edge or corner. That plane is behind it, so it is skipped, and
            // the ray leaves through the next face ahead instead of being lost.
            continue;
        } else if (!leave || crossing < leave->distance) {
            leave = BoundaryHit{crossing, face.normal};
        }
    }
    if (startsInside) {
        if (!leave || !(leave->distance > 0)) return std::nullopt;
        return leave;
    }
    if (!enter || !(enter->distance > 0)) return std::nullopt;
    if (leave && leave->distance < enter->distance) return std::nullopt;
    return enter;
}

std::vector<RayLeg> RefractingSolid::path(Ray ray, int maximumLegs) const {
    assert(maximumLegs > 0 && "A path needs room for at least one leg");
    std::vector<RayLeg> legs;
    WeightedRay current{ray, 1};
    bool isInside = false;
    while (static_cast<int>(legs.size()) < maximumLegs) {
        std::optional<BoundaryHit> hit = firstBoundaryHit(current.ray, isInside);
        if (!hit) {
            legs.push_back(RayLeg{current, std::nullopt});
            return legs;
        }
        legs.push_back(RayLeg{current, hit->distance});
        Vector3 hitPoint = current.ray.point(hit->distance);
        InterfaceContinuation result = RefractingInterface::continuation(
            current,
            hitPoint,
            hit->faceNormal,
            isInside ? refractiveIndexInside : refractiveIndexOutside,
            isInside ? refractiveIndexOutside : refractiveIndexInside);
        current = result.continued;
        if (!result.wasReflected) isInside = !isInside;
    }
    return legs;
}

} // namespace editor
