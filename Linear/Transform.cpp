// Transform: matrices and rigid transforms (Engine/Scene/Transform.swift).
#include "Transform.h"

namespace editor {

Matrix4 Transform::matrix() const {
    return Matrix4::translation(translation) * rotation.matrix() * Matrix4::scale(scale);
}

std::optional<Matrix4> Transform::inverseMatrix() const {
    if (scale.x == 0 || scale.y == 0 || scale.z == 0) return std::nullopt;
    Vector3 inverseScale(1 / scale.x, 1 / scale.y, 1 / scale.z);
    Matrix4 result = Matrix4::scale(inverseScale) * rotation.conjugate().matrix() * Matrix4::translation(-translation);
    if (!result.hasOnlyFiniteEntries()) return std::nullopt;
    return result;
}

std::optional<Matrix3> Transform::normalMatrix() const { return Matrix3::normalMatrix(matrix()); }

Transform Transform::composed(const Transform& child) const {
    return Transform{.translation = rotation.rotate(child.translation) + translation,
                     .rotation = rotation * child.rotation};
}

Transform Transform::rigidInverse() const {
    Quaternion inverseRotation = rotation.conjugate();
    return Transform{.translation = -inverseRotation.rotate(translation), .rotation = inverseRotation};
}

Transform Transform::mirrored(MirrorAxis axis) const {
    Vector3 mirroredTranslation = translation;
    Vector3 vector = rotation.vectorPart;
    switch (axis) {
    case MirrorAxis::x:
        mirroredTranslation.x = -mirroredTranslation.x;
        vector.y = -vector.y;
        vector.z = -vector.z;
        break;
    case MirrorAxis::y:
        mirroredTranslation.y = -mirroredTranslation.y;
        vector.x = -vector.x;
        vector.z = -vector.z;
        break;
    case MirrorAxis::z:
        mirroredTranslation.z = -mirroredTranslation.z;
        vector.x = -vector.x;
        vector.y = -vector.y;
        break;
    }
    return Transform{.translation = mirroredTranslation,
                     .rotation = Quaternion(vector, rotation.scalarPart),
                     .scale = scale};
}

} // namespace editor
