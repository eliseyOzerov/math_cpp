// The exact Fresnel equations for a smooth boundary (Engine/Math/Optics/DielectricFresnel.swift).
#include "DielectricFresnel.h"

#include <cassert>
#include <cmath>

#include "LightDirections.h"

namespace editor {

FresnelReflectance DielectricFresnel::reflectance(float cosineIncident, float refractiveIndexRatio) {
    assert((std::isnan(refractiveIndexRatio) || refractiveIndexRatio > 0) && "The refractive index ratio must be positive.");
    float cosine = clamped(cosineIncident, 0, 1);
    float sineSquaredTransmitted = refractiveIndexRatio * refractiveIndexRatio * sineSquaredFromCosine(cosine);
    if (sineSquaredTransmitted > 1 || cosine == 0) return FresnelReflectance{1, 1};
    float cosineTransmitted = std::sqrt(1 - sineSquaredTransmitted);
    float incidentTerm = refractiveIndexRatio * cosine;
    float transmittedTerm = refractiveIndexRatio * cosineTransmitted;
    float amplitudePerpendicular = (incidentTerm - cosineTransmitted) / (incidentTerm + cosineTransmitted);
    float amplitudeParallel = (cosine - transmittedTerm) / (cosine + transmittedTerm);
    return FresnelReflectance{
        amplitudePerpendicular * amplitudePerpendicular, amplitudeParallel * amplitudeParallel};
}

Angle DielectricFresnel::brewsterAngle(float refractiveIndexRatio) {
    assert((std::isnan(refractiveIndexRatio) || refractiveIndexRatio > 0) && "The refractive index ratio must be positive.");
    return Angle::fromRadians(std::atan(1 / refractiveIndexRatio));
}

} // namespace editor
