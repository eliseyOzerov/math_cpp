// Lambert diffuse and Blinn–Phong specular (Engine/Math/Optics/DiffuseAndSpecular.swift).
#include "DiffuseAndSpecular.h"

#include <algorithm>
#include <cmath>

#include "../Linear/Angle.h"
#include "../Linear/Scalar.h"
#include "LightDirections.h"

namespace editor {

float LambertDiffuse::brdf(float albedo) { return albedo / Angle::pi; }

float LambertDiffuse::cosineFactor(Vector3 normal, Vector3 towardLight) {
    return largerOf(0.0f, cosineOfAngle(normal, towardLight));
}

float LambertDiffuse::reflectance(float albedo, Vector3 normal, Vector3 towardLight) {
    return brdf(albedo) * cosineFactor(normal, towardLight);
}

float BlinnPhong::lobe(Vector3 normal, Vector3 towardLight, Vector3 towardViewer, float shininess) {
    if (!(cosineOfAngle(normal, towardLight) > 0) || !(cosineOfAngle(normal, towardViewer) > 0)) return 0;
    Vector3 half = halfVector(towardLight, towardViewer);
    float cosineToHalfVector = largerOf(0.0f, cosineOfAngle(normal, half));
    return std::pow(cosineToHalfVector, shininess);
}

float BlinnPhong::normalizationFactor(float shininess) { return (shininess + 8) / (8 * Angle::pi); }

float BlinnPhong::brdf(Vector3 normal, Vector3 towardLight, Vector3 towardViewer, float shininess) {
    return normalizationFactor(shininess) * lobe(normal, towardLight, towardViewer, shininess);
}

} // namespace editor
