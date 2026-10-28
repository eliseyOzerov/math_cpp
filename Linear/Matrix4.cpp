// Matrix4: rotation and the general inverse (Engine/Math/Matrix4.swift, Matrix4+Inverse.swift).
#include "Matrix4.h"

#include "MathProblem.h"
#include "Matrix3.h"

namespace editor {

Matrix4 Matrix4::rotation(Vector3 axis, Angle angle) {
    if (!(axis.length() > 0)) throw MathProblem("A rotation axis must have a direction; got a zero vector.");
    Vector3 unitAxis = axis.normalized();
    float x = unitAxis.x, y = unitAxis.y, z = unitAxis.z;
    float c = angle.cosine();
    float s = angle.sine();
    float t = 1 - c;

    return Matrix4(Vector4(t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0),
                   Vector4(t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0),
                   Vector4(t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0), Vector4(0, 0, 0, 1));
}

namespace {

/// Whether a matrix with this `determinant` and these rows may be inverted. See `Matrix4::inverse`
/// for the reasoning behind the two cases. It takes the already computed determinant and
/// reciprocal so that nothing is computed twice.
bool hasInverse(float determinant, float reciprocal, const std::array<Vector4, 4>& rows) {
    const auto& [a, b, c, d] = rows;
    if (determinant == 0 || !std::isfinite(reciprocal)) return false;
    if (!(d == Vector4(0, 0, 0, 1))) return true;
    return !Matrix3::isSingular(determinant, a.xyz(), b.xyz(), c.xyz());
}

} // namespace

std::optional<Matrix4> Matrix4::inverse() const {
    std::array<Vector4, 4> matrixRows = rows();
    const auto& [a, b, c, d] = matrixRows;

    float top0 = a[0] * b[1] - a[1] * b[0];
    float top1 = a[0] * b[2] - a[2] * b[0];
    float top2 = a[0] * b[3] - a[3] * b[0];
    float top3 = a[1] * b[2] - a[2] * b[1];
    float top4 = a[1] * b[3] - a[3] * b[1];
    float top5 = a[2] * b[3] - a[3] * b[2];

    float bottom0 = c[0] * d[1] - c[1] * d[0];
    float bottom1 = c[0] * d[2] - c[2] * d[0];
    float bottom2 = c[0] * d[3] - c[3] * d[0];
    float bottom3 = c[1] * d[2] - c[2] * d[1];
    float bottom4 = c[1] * d[3] - c[3] * d[1];
    float bottom5 = c[2] * d[3] - c[3] * d[2];

    float determinant = top0 * bottom5 - top1 * bottom4 + top2 * bottom3 + top3 * bottom2 - top4 * bottom1
                      + top5 * bottom0;

    float reciprocal = 1 / determinant;
    if (!hasInverse(determinant, reciprocal, matrixRows)) return std::nullopt;

    // Entry (i, j) of the result is the cofactor of entry (j, i), divided by the determinant.
    Matrix4 result(
        Vector4(b[1] * bottom5 - b[2] * bottom4 + b[3] * bottom3, -a[1] * bottom5 + a[2] * bottom4 - a[3] * bottom3,
                d[1] * top5 - d[2] * top4 + d[3] * top3, -c[1] * top5 + c[2] * top4 - c[3] * top3)
            * reciprocal,
        Vector4(-b[0] * bottom5 + b[2] * bottom2 - b[3] * bottom1, a[0] * bottom5 - a[2] * bottom2 + a[3] * bottom1,
                -d[0] * top5 + d[2] * top2 - d[3] * top1, c[0] * top5 - c[2] * top2 + c[3] * top1)
            * reciprocal,
        Vector4(b[0] * bottom4 - b[1] * bottom2 + b[3] * bottom0, -a[0] * bottom4 + a[1] * bottom2 - a[3] * bottom0,
                d[0] * top4 - d[1] * top2 + d[3] * top0, -c[0] * top4 + c[1] * top2 - c[3] * top0)
            * reciprocal,
        Vector4(-b[0] * bottom3 + b[1] * bottom1 - b[2] * bottom0, a[0] * bottom3 - a[1] * bottom1 + a[2] * bottom0,
                -d[0] * top3 + d[1] * top1 - d[2] * top0, c[0] * top3 - c[1] * top1 + c[2] * top0)
            * reciprocal);
    if (!result.hasOnlyFiniteEntries()) return std::nullopt;
    return result;
}

} // namespace editor
