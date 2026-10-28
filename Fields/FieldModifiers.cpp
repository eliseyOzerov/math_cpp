// Mirror, move, grow, hollow and roughen a field (Engine/Math/Fields/FieldModifiers.swift).
#include "FieldModifiers.h"

#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "FieldBounds.h"
#include "VectorComponents.h"

namespace editor {

// MARK: MirroredField

MirroredField::MirroredField(DistanceField inner, Plane plane) : inner(inner), plane(plane) {
    if (inner.bounds()) {
        Bounds kept = frontPart(*inner.bounds(), plane);
        mirroredBox = kept.unionWith(reflected(kept, plane));
    }
}

float MirroredField::signedDistance(Vector3 point) const {
    Vector3 folded = plane.signedDistance(point) < 0 ? reflection(plane, point) : point;
    return inner.signedDistance(folded);
}

// MARK: TransformedField

TransformedField::TransformedField(DistanceField inner, Transform transform) : inner(inner), transform(transform) {
    if (!(transform.scale.isFinite() && smallestComponent(componentwiseAbsolute(transform.scale)) > 0)) {
        throw MathProblem("A transformed field needs finite, non-zero scale factors.");
    }
    distanceScale = smallestComponent(componentwiseAbsolute(transform.scale));
    if (inner.bounds()) worldBox = transformed(*inner.bounds(), transform);
}

Vector3 TransformedField::ownPoint(Vector3 point) const {
    return transform.rotation.conjugate().rotate(point - transform.translation) / transform.scale;
}

float TransformedField::signedDistance(Vector3 point) const {
    float value = distanceScale * inner.signedDistance(ownPoint(point));
    if (!worldBox) return value;
    return raising(*worldBox, value, point);
}

// MARK: OffsetField

OffsetField::OffsetField(DistanceField inner, float amount) : inner(inner), amount(amount) {
    if (!std::isfinite(amount)) throw MathProblem("An offset needs a finite amount.");
}

std::optional<Bounds> OffsetField::bounds() const {
    if (!inner.bounds()) return std::nullopt;
    return expanded(*inner.bounds(), largerOf(amount, 0));
}

// MARK: ShellField

ShellField::ShellField(DistanceField inner, float thickness) : inner(inner), thickness(thickness) {
    if (!(thickness >= 0 && std::isfinite(thickness))) {
        throw MathProblem("A shell needs a finite thickness of 0 or more.");
    }
}

float ShellField::signedDistance(Vector3 point) const {
    return std::fabs(inner.signedDistance(point)) - thickness / 2;
}

std::optional<Bounds> ShellField::bounds() const {
    if (!inner.bounds()) return std::nullopt;
    return expanded(*inner.bounds(), thickness / 2);
}

// MARK: DisplacedField

DisplacedField::DisplacedField(DistanceField inner, GradientNoise noise, float amplitude, float frequency)
    : inner(inner), noise(noise), amplitude(amplitude), frequency(frequency) {
    if (!std::isfinite(amplitude)) throw MathProblem("A displacement needs a finite amplitude.");
    if (!(frequency >= 0 && std::isfinite(frequency))) {
        throw MathProblem("A displacement needs a finite frequency of 0 or more.");
    }
    slopeDivisor = 1 + std::fabs(amplitude) * frequency * gradientNoiseSlopeBound;
    if (inner.bounds()) grownBox = expanded(*inner.bounds(), std::fabs(amplitude) * gradientNoiseLargestValue);
}

float DisplacedField::signedDistance(Vector3 point) const {
    float displacement = amplitude * noise.value(point * frequency);
    float value = (inner.signedDistance(point) + displacement) / slopeDivisor;
    if (!grownBox) return value;
    return raising(*grownBox, value, point);
}

} // namespace editor
