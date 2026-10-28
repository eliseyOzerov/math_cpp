// Refraction of a ray at a smooth surface (Engine/Math/Optics/Refraction.swift).
#include "Refraction.h"

#include <cassert>
#include <cmath>

#include "LightDirections.h"

namespace editor {

std::optional<Vector3> Refraction::refractedDirection(Vector3 incoming, Vector3 normal, float refractiveIndexRatio) {
    assert((std::isnan(refractiveIndexRatio) || refractiveIndexRatio > 0) && "The refractive index ratio must be positive.");
    Vector3 normalFacingRay = cosineOfAngle(incoming, normal) < 0 ? normal : -normal;
    float cosineIncident = -cosineOfAngle(incoming, normalFacingRay);
    float sineSquaredTransmitted = refractiveIndexRatio * refractiveIndexRatio * sineSquaredFromCosine(cosineIncident);
    if (!(sineSquaredTransmitted <= 1)) return std::nullopt;
    float cosineTransmitted = std::sqrt(1 - sineSquaredTransmitted);
    return incoming * refractiveIndexRatio + normalFacingRay * (refractiveIndexRatio * cosineIncident - cosineTransmitted);
}

std::optional<Angle> Refraction::criticalAngle(float refractiveIndexRatio) {
    if (!(refractiveIndexRatio > 1)) return std::nullopt;
    return Angle::fromRadians(std::asin(1 / refractiveIndexRatio));
}

} // namespace editor
