// The gradient and the normal of a field (Engine/Math/Fields/DistanceField.swift).
#include "DistanceField.h"

#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

float FiniteDifference::step(float featureSize, Vector3 point) {
    if (!(featureSize > 0)) throw MathProblem("The feature size must be positive.");
    float magnitude = largerOf(point.length(), featureSize);
    return std::cbrt(unitRoundoff * magnitude * featureSize * featureSize);
}

Vector3 DistanceField::gradient(Vector3 point, float step) const {
    if (!(step > 0)) throw MathProblem("The gradient step must be positive.");
    return Vector3(
        centralDifference(point, Vector3::xAxis(), step),
        centralDifference(point, Vector3::yAxis(), step),
        centralDifference(point, Vector3::zAxis(), step));
}

// The slope of the field at `point` along the unit vector `axis`, by a central difference with
// step `step`: `(f(p + h·axis) − f(p − h·axis)) / separation`.
//
// How: far from the origin `p ± h·axis` is rounded to the nearest representable `Float`, so
// the samples are not exactly `2h` apart (a kilometre out, the spacing of floats is 6·10⁻⁵,
// which is 0.15% of a step of 0.04). The division therefore uses the separation of the
// samples as actually stored, `(p + h·axis) − (p − h·axis)`, which is exact because the
// two are close (Sterbenz). This is the textbook trick of Press et al., *Numerical Recipes*,
// §5.7 ("trick to reduce finite precision error in h"). Only the rounding of the field
// values themselves is left.
//
// Conventions: `axis` must be a unit vector along a coordinate axis.
float DistanceField::centralDifference(Vector3 point, Vector3 axis, float step) const {
    Vector3 plusPoint = point + axis * step;
    Vector3 minusPoint = point - axis * step;
    float separation = (plusPoint - minusPoint).dot(axis);
    return (signedDistance(plusPoint) - signedDistance(minusPoint)) / separation;
}

Vector3 DistanceField::gradientForFeatureSize(Vector3 point, float featureSize) const {
    return gradient(point, FiniteDifference::step(featureSize, point));
}

Vector3 DistanceField::surfaceNormal(Vector3 point, float step) const {
    return gradient(point, step).normalized();
}

Vector3 DistanceField::surfaceNormalForFeatureSize(Vector3 point, float featureSize) const {
    return surfaceNormal(point, FiniteDifference::step(featureSize, point));
}

} // namespace editor
