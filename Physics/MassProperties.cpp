// Mass, volume and inertia tensors of the basic solids (Engine/Math/Physics/MassProperties.swift).
#include "MassProperties.h"

#include "../Linear/Angle.h"

namespace editor {

namespace {


} // namespace

float MassProperties::sphereVolume(float radius) {
    assert(radius >= 0 && "The radius must be zero or positive.");
    return 4.0f / 3.0f * Angle::pi * radius * radius * radius;
}

float MassProperties::boxVolume(Vector3 size) {
    assert(size.x >= 0 && size.y >= 0 && size.z >= 0 && "The box sizes must be zero or positive.");
    return size.x * size.y * size.z;
}

float MassProperties::cylinderVolume(float radius, float height) {
    assert(radius >= 0 && height >= 0 && "The radius and height must be zero or positive.");
    return Angle::pi * radius * radius * height;
}

Matrix3 MassProperties::solidSphereInertia(float mass, float radius) {
    assert(mass >= 0 && radius >= 0 && "The mass and radius must be zero or positive.");
    float moment = 2.0f / 5.0f * mass * radius * radius;
    return Matrix3::diagonal(Vector3(moment, moment, moment));
}

Matrix3 MassProperties::solidBoxInertia(float mass, Vector3 size) {
    assert(mass >= 0 && "The mass must be zero or positive.");
    assert(size.x >= 0 && size.y >= 0 && size.z >= 0 && "The box sizes must be zero or positive.");
    Vector3 squaredSize = size * size;
    float share = mass / 12;
    return Matrix3::diagonal(share * Vector3(squaredSize.y + squaredSize.z, squaredSize.z + squaredSize.x,
                                             squaredSize.x + squaredSize.y));
}

Matrix3 MassProperties::solidCylinderInertia(float mass, float radius, float height) {
    assert(mass >= 0 && radius >= 0 && height >= 0 && "The mass, radius and height must be zero or positive.");
    float squaredRadius = radius * radius;
    float aboutAxis = 0.5f * mass * squaredRadius;
    float acrossAxis = mass / 12 * (3 * squaredRadius + height * height);
    return Matrix3::diagonal(Vector3(acrossAxis, acrossAxis, aboutAxis));
}

Matrix3 MassProperties::parallelAxis(const Matrix3& inertia, float mass, Vector3 offset) {
    assert(mass >= 0 && "The mass must be zero or positive.");
    float squaredOffset = offset.dot(offset);
    auto rows = inertia.rows();
    // Row i of  |r|²·1 − r·rᵀ  is  |r|²·eᵢ − rᵢ·r  (eᵢ the i-th axis).
    return Matrix3(rows[0] + mass * (squaredOffset * Vector3::xAxis() - offset.x * offset),
                   rows[1] + mass * (squaredOffset * Vector3::yAxis() - offset.y * offset),
                   rows[2] + mass * (squaredOffset * Vector3::zAxis() - offset.z * offset));
}

} // namespace editor
