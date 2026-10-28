// The ingredients of a microfacet specular BRDF (Engine/Math/Optics/Microfacet.swift).
#include "Microfacet.h"

#include <cmath>
#include <limits>

#include "../Linear/Angle.h"
#include "../Linear/Scalar.h"
#include "LightDirections.h"

namespace editor {

float MicrofacetRoughness::alpha(float roughness) {
    float clampedRoughness = clamped(roughness, 0, 1);
    return largerOf(minimumAlpha, clampedRoughness * clampedRoughness);
}

float GGXDistribution::density(Vector3 normal, Vector3 halfVector, float roughness) {
    return density(cosineOfAngle(normal, halfVector), roughness);
}

float GGXDistribution::density(float cosineToNormal, float roughness) {
    if (!(cosineToNormal > 0)) return 0;
    float alpha = MicrofacetRoughness::alpha(roughness);
    float alphaSquared = alpha * alpha;
    float cosineSquared = smallerOf(cosineToNormal * cosineToNormal, 1);
    // c²(α² − 1) + 1 is rewritten as (1 − c²) + c²α² so the denominator stays accurate
    // near c = 1, where the plain form subtracts two almost equal numbers.
    float denominatorBase = sineSquaredFromCosine(cosineToNormal) + cosineSquared * alphaSquared;
    return alphaSquared / (Angle::pi * denominatorBase * denominatorBase);
}

float SmithMasking::lambda(float cosine, float roughness) {
    if (!(cosine > 0)) return std::numeric_limits<float>::infinity();
    float alpha = MicrofacetRoughness::alpha(roughness);
    float tangentSquared = tangentSquaredFromCosine(cosine);
    return (std::sqrt(1 + alpha * alpha * tangentSquared) - 1) / 2;
}

float SmithMasking::singleDirectionVisibility(float cosine, float roughness) {
    return 1 / (1 + lambda(cosine, roughness));
}

float SmithMasking::maskingShadowing(float cosineToLight, float cosineToViewer, float roughness) {
    // The two Λ are added together first: a + b equals b + a exactly in floating point,
    // while (1 + a) + b may differ from (1 + b) + a in the last digit.
    float combinedLambda = lambda(cosineToLight, roughness) + lambda(cosineToViewer, roughness);
    return 1 / (1 + combinedLambda);
}

float SmithMasking::visibility(float cosineToLight, float cosineToViewer, float roughness) {
    if (!(cosineToLight > 0 && cosineToViewer > 0)) return 0;
    float alpha = MicrofacetRoughness::alpha(roughness);
    float alphaSquared = alpha * alpha;
    float lightRoot = std::sqrt(cosineToLight * cosineToLight + alphaSquared * sineSquaredFromCosine(cosineToLight));
    float viewerRoot = std::sqrt(cosineToViewer * cosineToViewer + alphaSquared * sineSquaredFromCosine(cosineToViewer));
    return 0.5f / (cosineToViewer * lightRoot + cosineToLight * viewerRoot);
}

} // namespace editor
