// Matrix3: the inverse and the normal matrix (Engine/Math/Algebra/Matrix3.swift).
#include "Matrix3.h"

#include "Matrix4.h"

namespace editor {

Matrix3 Matrix3::upperLeftOf(const Matrix4& matrix) {
    return fromColumns(matrix.column(0).xyz(), matrix.column(1).xyz(), matrix.column(2).xyz());
}

std::optional<Matrix3> Matrix3::inverse() const {
    auto [row0, row1, row2] = rows();
    float determinantValue = determinant();
    if (isSingular(determinantValue, row0, row1, row2)) return std::nullopt;
    Matrix3 result = fromColumns(row1.cross(row2) / determinantValue, row2.cross(row0) / determinantValue,
                                 row0.cross(row1) / determinantValue);
    if (!result.hasOnlyFiniteEntries()) return std::nullopt;
    return result;
}

std::optional<Matrix3> Matrix3::normalMatrix(const Matrix4& model) {
    std::optional<Matrix3> inverted = upperLeftOf(model).inverse();
    if (!inverted) return std::nullopt;
    return inverted->transposed();
}

} // namespace editor
