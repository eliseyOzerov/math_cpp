// Quaternion: construction, products, rotation, blending and alignment
// (Engine/Math/Algebra/Quaternion.swift and Quaternion+Alignment.swift).
#include "Quaternion.h"

#include <cassert>
#include <limits>

#include "MathProblem.h"

namespace editor {

namespace {

/// How far from 1 the length of a "unit" quaternion may be before a debug build stops. Rounding
/// after a few hundred products stays far below this; a real scaling mistake (a length of 2) is
/// far above. The length is about 1, so this is also relative.
constexpr float unitLengthTolerance = 1e-4f;

/// Above this dot product the two rotations count as nearly equal and `slerp` uses a straight
/// blend instead. cos(θ/2) > 0.9995 means θ is under about 3.6°.
constexpr float nearlyEqualDot = 0.9995f;

/// How close to opposite two directions may be before the rotation between them is built by
/// turning half a turn around some perpendicular axis instead, as the sine of the angle that is
/// left (|a × b|, about 1e-7 rad, or 6e-6°). Below it the cross product is too small to name an
/// axis, and a fallback half turn is off by at most this angle.
constexpr float oppositeDirectionSine = 1e-7f;

/// Some unit vector perpendicular to the unit vector `direction`.
///
/// How: direction × axis, where `axis` is the coordinate axis along which `direction` has its
/// smallest component, so the two are far from parallel and the cross product is well away from
/// zero (length at least √(2/3) ≈ 0.82).
Vector3 perpendicularDirection(Vector3 direction) {
    Vector3 magnitudes(std::fabs(direction.x), std::fabs(direction.y), std::fabs(direction.z));
    Vector3 axis;
    if (magnitudes.x <= magnitudes.y && magnitudes.x <= magnitudes.z) {
        axis = Vector3::xAxis();
    } else if (magnitudes.y <= magnitudes.z) {
        axis = Vector3::yAxis();
    } else {
        axis = Vector3::zAxis();
    }
    return direction.cross(axis).normalized();
}

} // namespace

bool Quaternion::isUnit() const { return std::fabs(length() - 1) <= unitLengthTolerance; }

Quaternion Quaternion::fromAxis(Vector3 axis, Angle angle) {
    if (!(axis.length() > 0)) throw MathProblem("A rotation axis must have a direction; got a zero vector.");
    Angle halfAngle = angle / 2;
    return Quaternion(axis.normalized() * halfAngle.sine(), halfAngle.cosine());
}

Quaternion Quaternion::normalized() const {
    float currentLength = length();
    if (!(currentLength > 0)) throw MathProblem("A zero quaternion cannot be normalized.");
    return Quaternion(vectorPart / currentLength, scalarPart / currentLength);
}

Vector3 Quaternion::rotate(Vector3 vector) const {
    assert(isUnit() && "Quaternion::rotate needs a unit quaternion. Use normalized().");
    Vector3 twiceCross = 2 * vectorPart.cross(vector);
    return vector + scalarPart * twiceCross + vectorPart.cross(twiceCross);
}

Matrix4 Quaternion::matrix() const {
    assert(isUnit() && "Quaternion::matrix needs a unit quaternion. Use normalized().");
    float x = vectorPart.x, y = vectorPart.y, z = vectorPart.z;
    float w = scalarPart;

    return Matrix4(Vector4(1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 0),
                   Vector4(2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 0),
                   Vector4(2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y), 0), Vector4(0, 0, 0, 1));
}

Quaternion Quaternion::slerp(Quaternion start, Quaternion end, float amount) {
    assert(start.isUnit() && end.isUnit() && "Quaternion::slerp needs unit quaternions. Use normalized().");
    float cosineOfArc = start.dot(end);
    Quaternion nearerEnd = end;
    if (cosineOfArc < 0) {
        nearerEnd = -end;
        cosineOfArc = -cosineOfArc;
    }

    if (cosineOfArc > nearlyEqualDot) {
        Quaternion blended(Vector3::linearBlend(start.vectorPart, nearerEnd.vectorPart, amount),
                           linearBlend(start.scalarPart, nearerEnd.scalarPart, amount));
        return blended.normalized();
    }

    Angle arc = Angle::arcCosine(cosineOfArc);
    float startWeight = (arc * (1 - amount)).sine() / arc.sine();
    float endWeight = (arc * amount).sine() / arc.sine();
    return Quaternion(startWeight * start.vectorPart + endWeight * nearerEnd.vectorPart,
                      startWeight * start.scalarPart + endWeight * nearerEnd.scalarPart);
}

Quaternion Quaternion::rotation(Vector3 direction, Vector3 target) {
    std::optional<Vector3> source = direction.unitDirection();
    std::optional<Vector3> destination = target.unitDirection();
    if (!source || !destination) {
        float notANumber = std::numeric_limits<float>::quiet_NaN();
        return Quaternion(Vector3(notANumber, notANumber, notANumber), notANumber);
    }
    Vector3 sum = *source + *destination;
    Vector3 crossProduct = source->cross(sum); // = source × destination, accurately near opposite
    bool isNearlyOpposite = source->dot(*destination) < 0 && crossProduct.length() <= oppositeDirectionSine;
    if (isNearlyOpposite) return Quaternion(perpendicularDirection(*source), 0);
    return Quaternion(crossProduct, sum.lengthSquared() / 2).normalized();
}

} // namespace editor
