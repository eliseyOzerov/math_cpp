// A ray at the boundary between two media (Engine/Math/Optics/RefractingInterface.swift).
#include "RefractingInterface.h"

#include <cassert>
#include <cmath>

#include "DielectricFresnel.h"
#include "LightDirections.h"
#include "Refraction.h"

namespace editor {

InterfaceContinuation RefractingInterface::continuation(
    WeightedRay incoming,
    Vector3 hitPoint,
    Vector3 surfaceNormal,
    float refractiveIndexBefore,
    float refractiveIndexAfter) {
    assert(refractiveIndexBefore > 0 && refractiveIndexAfter > 0 && "Refractive indices must be positive.");
    float ratio = refractiveIndexBefore / refractiveIndexAfter;
    Vector3 direction = incoming.ray.direction;
    if (std::optional<Vector3> refracted = Refraction::refractedDirection(direction, surfaceNormal, ratio)) {
        FresnelReflectance reflectance = DielectricFresnel::reflectance(
            std::fabs(cosineOfAngle(direction, surfaceNormal)), ratio);
        return InterfaceContinuation{
            WeightedRay{Ray(hitPoint, *refracted), incoming.weight * reflectance.transmittance()}, false};
    }
    return InterfaceContinuation{
        WeightedRay{Ray(hitPoint, direction.reflectedAcrossPlane(surfaceNormal)), incoming.weight}, true};
}

} // namespace editor
