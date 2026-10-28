// A 3×3 matrix (Engine/Math/Algebra/Matrix3.swift). Swift keeps a simd_float3x3; this keeps three
// column Vector3s, in the same order, and writes out the products simd computes. simd may fuse or
// reorder the additions inside a product, so a result can differ from Swift's in the last bit.
#pragma once

#include <array>
#include <optional>

#include "Vector3.h"

namespace editor {

class Matrix4;

/// A 3×3 matrix: rotates, scales and shears directions, with no translation. It is the
/// upper-left block of a `Matrix4`.
///
/// **Conventions:** same as `Matrix4`. Vectors are columns on the right
/// (`transformed = matrix × vector`), `a * b` applies `b` first, and a matrix is written row by
/// row but stored column by column.
///
/// Use when: turning or scaling directions only, such as a surface normal or an object's orientation
/// Tags: 3x3 matrix, mat3, float3x3, rotation matrix, linear transform, basis
/// Node: none (value type)
class Matrix3 {
public:
    /// Builds a matrix from its three rows, written top to bottom as on paper.
    Matrix3(Vector3 row0, Vector3 row1, Vector3 row2)
        : columns{Vector3(row0.x, row1.x, row2.x), Vector3(row0.y, row1.y, row2.y), Vector3(row0.z, row1.z, row2.z)} {}

    /// The upper-left 3×3 block of `matrix`: its rotation, scale and shear without the
    /// translation (the 4th column) and the bottom row.
    static Matrix3 upperLeftOf(const Matrix4& matrix);

    /// The matrix that changes nothing.
    static Matrix3 identity() { return Matrix3(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)); }

    /// The matrix with `entries` down the diagonal and zeros elsewhere: rows (x, 0, 0), (0, y, 0),
    /// (0, 0, z). Scaling matrices and the inertia tensors of symmetric solids are diagonal.
    ///
    /// Use when: building a scale or a principal-axes tensor from three numbers
    /// Tags: diagonal matrix, scale matrix, diag, mat3 from diagonal, principal moments
    /// Node: none (value type)
    static Matrix3 diagonal(Vector3 entries) {
        return Matrix3(Vector3(entries.x, 0, 0), Vector3(0, entries.y, 0), Vector3(0, 0, entries.z));
    }

    /// The entry in `row`, `column` (both 0…2). Swift's tests read `storage[column][row]`.
    float entry(int row, int column) const { return columns[column][row]; }

    /// The column `index` (0…2), left to right.
    Vector3 column(int index) const { return columns[index]; }

    /// The three rows, top to bottom.
    ///
    /// Use when: reading a matrix row by row, for example when writing a formula or a test
    /// Tags: matrix rows, row vectors, matrix access, mat3 rows
    /// Node: none (value type)
    std::array<Vector3, 3> rows() const {
        return {Vector3(columns[0].x, columns[1].x, columns[2].x), Vector3(columns[0].y, columns[1].y, columns[2].y),
                Vector3(columns[0].z, columns[1].z, columns[2].z)};
    }

    /// Combines two matrices into one that does both: first `other`, then this one. Entry
    /// (row i, column j) is the dot product of row i of this matrix with column j of `other`.
    /// Each result column is this matrix applied to the matching column of `other`.
    ///
    /// Use when: applying two linear transforms in one step, such as scale then rotate
    /// Tags: matrix multiply, matrix product, combine transforms, mat3 multiplication, concatenate
    /// Node: none (value type)
    Matrix3 operator*(const Matrix3& other) const {
        return fromColumns(transform(other.columns[0]), transform(other.columns[1]), transform(other.columns[2]));
    }

    /// The sum of two matrices, entry by entry: entry (i, j) is this(i, j) + other(i, j).
    ///
    /// This is *not* "do both transforms" (that is `*`). Adding matrices is for building tensors
    /// from parts, such as the inertia of a compound solid from the inertias of its pieces, and
    /// for blending linear maps.
    ///
    /// Use when: adding up inertia tensors or other quantities stored as matrices
    /// Tags: matrix add, matrix sum, mat3 plus, add matrices, entrywise sum
    /// Node: none (value type)
    Matrix3 operator+(const Matrix3& other) const {
        return fromColumns(columns[0] + other.columns[0], columns[1] + other.columns[1], columns[2] + other.columns[2]);
    }

    /// The difference of two matrices, entry by entry: this(i, j) − other(i, j).
    ///
    /// Use when: measuring how far a matrix is from another, such as M·M⁻¹ − identity in a test; removing one part from a sum of tensors
    /// Tags: matrix subtract, matrix difference, mat3 minus, subtract matrices, entrywise difference
    /// Node: none (value type)
    Matrix3 operator-(const Matrix3& other) const {
        return fromColumns(columns[0] - other.columns[0], columns[1] - other.columns[1], columns[2] - other.columns[2]);
    }

    /// Every entry multiplied by the same plain number: (matrix × factor)(i, j) = matrix(i, j)·factor.
    /// Scales the size of the linear map's effect, for example an inertia tensor times a mass.
    ///
    /// Use when: scaling a tensor by a mass or a weight; blending matrices with weights
    /// Tags: scale matrix entries, matrix times number, mat3 scalar multiply, weight matrix
    /// Node: none (value type)
    Matrix3 operator*(float factor) const {
        return fromColumns(columns[0] * factor, columns[1] * factor, columns[2] * factor);
    }

    /// Applies the matrix to a vector: each result component is the dot product of one row with `vector`.
    ///
    /// How: column 0 × x + column 1 × y + column 2 × z, added left to right. simd may fuse the
    /// multiplies and adds, so the last bit can differ from Swift's.
    ///
    /// Use when: turning a direction by an orientation matrix; moving a vector into another basis
    /// Tags: matrix times vector, apply matrix, transform direction, mat3 multiply, mul
    /// Node: none (value type)
    Vector3 transform(Vector3 vector) const { return columns[0] * vector.x + columns[1] * vector.y + columns[2] * vector.z; }

    // MARK: Transpose, determinant, inverse

    /// How small |determinant| / (product of the row lengths) may get before a matrix counts as
    /// singular. Shared with the `Matrix4` inverse.
    static constexpr float singularityTolerance = 1e-6f;

    /// The matrix mirrored across its diagonal: row i becomes column i, so entry (i, j) swaps
    /// with entry (j, i). For a rotation matrix this is the inverse.
    ///
    /// Use when: undoing a pure rotation cheaply; building the normal matrix; swapping row and column order
    /// Tags: transpose, mirror across diagonal, transposed, inverse of rotation, mat3
    /// Node: none (solver)
    Matrix3 transposed() const { return Matrix3(columns[0], columns[1], columns[2]); }

    /// The signed volume of the box that the matrix turns the unit cube into. It is 0 when the
    /// matrix flattens space (no inverse) and negative when it mirrors.
    ///
    /// How: determinant = row0 · (row1 × row2), the scalar triple product. Expanding the 3×3
    /// determinant along its first row gives a·(ei − fh) − b·(di − fg) + c·(dh − eg) for rows
    /// (a b c), (d e f), (g h i). Each bracket is one component of row1 × row2, so this equals
    /// row0 · (row1 × row2). Geometrically row1 × row2 is the area vector of the parallelogram the
    /// last two rows span, and the dot product with row0 multiplies by the height of the box.
    ///
    /// Use when: testing whether a matrix flips or flattens space; checking a transform can be inverted
    /// Tags: determinant, det, volume scale, handedness, singular test, mat3
    /// Node: none (solver)
    float determinant() const {
        auto [row0, row1, row2] = rows();
        return row0.dot(row1.cross(row2));
    }

    /// Whether a matrix with these rows and this `determinant` counts as flat, judged against its
    /// own scale. Shared with the `Matrix4` inverse, which uses it for the upper-left 3×3 of
    /// moves, turns and scales so that the determinant is computed only once.
    ///
    /// Test: |determinant| ≤ singularityTolerance × |row0||row1||row2|. The product of the row
    /// lengths is the largest the determinant could be for rows of those lengths (Hadamard's
    /// inequality), reached when the rows are at right angles. So the ratio is 1 for
    /// perpendicular rows and falls to 0 as the rows become parallel.
    ///
    /// **What it detects:** rows that are nearly parallel, which is how a flattened matrix looks
    /// at any size (a 1 mm flat matrix and a 1 km one are both rejected). **What it does not
    /// detect:** one tiny scale on its own. `scale(1, 1e-12, 1)` keeps its rows perpendicular, so
    /// the ratio is 1 and it is accepted; its inverse is valid but has a huge entry, and when that
    /// overflows float (`scale(1, 1e-39, 1)`) the caller's finite-entries check rejects it instead.
    ///
    /// Use when: deciding if a matrix is too flat to invert; the shared flatness test for matrix inverses
    /// Tags: singular, degenerate matrix, not invertible, zero determinant, tolerance
    /// Node: none (solver)
    static bool isSingular(float determinant, Vector3 row0, Vector3 row1, Vector3 row2) {
        float largestPossible = row0.length() * row1.length() * row2.length();
        return std::fabs(determinant) <= singularityTolerance * largestPossible;
    }

    /// The matrix that undoes this one (inverse × self = identity), or no value if there is none:
    /// the matrix is flat (see `isSingular`), or some entry of the result would not be a finite
    /// number (the scale is so small that its reciprocal overflows float).
    ///
    /// How (the adjugate over the determinant, with the adjugate written as cross products). With
    /// rows r0, r1, r2, the **columns** of the inverse are
    ///
    ///     column 0 = (r1 × r2) / det     column 1 = (r2 × r0) / det     column 2 = (r0 × r1) / det
    ///
    /// Why: r1 × r2 is perpendicular to both r1 and r2, so its dot product with r1 or r2 is 0, and
    /// with r0 it is det. That is exactly one column of matrix × inverse = identity after dividing
    /// by det; the other two columns work the same way.
    ///
    /// Use when: undoing a transform, such as going from world back to object space
    /// Tags: matrix inverse, invert, inverse(), mat3 inverse, undo transform, adjugate
    /// Node: none (solver)
    std::optional<Matrix3> inverse() const;

    // MARK: Normal matrix

    /// The matrix that turns **normals** (surface directions) correctly when `model` moves the
    /// surface, or no value if `model` is singular (see `inverse`). It is the inverse-transpose of
    /// the upper-left 3×3 block of `model`.
    ///
    /// Why not use the model matrix itself? Under non-uniform scale it bends normals away from
    /// perpendicular. Stretch x by 2: the tangent (1, 1) becomes (2, 1), but the normal (−1, 1)
    /// would become (−2, 1) with the plain matrix, and (−2, 1)·(2, 1) = −3, no longer 0.
    ///
    /// Derivation: a tangent t turns into M t. We need a normal matrix N with (N n)·(M t) = 0
    /// whenever n·t = 0. Writing the dot product as (N n)ᵀ(M t) = nᵀ Nᵀ M t, this is nᵀ t = 0 when
    /// Nᵀ M = identity, i.e. N = (M⁻¹)ᵀ. In the example N = diag(1/2, 1) gives (−1/2, 1)·(2, 1) = 0.
    ///
    /// For a rotation, or a rotation with uniform scale, the result is the same as M up to a
    /// uniform factor. The transformed normal is not unit length in general, so normalize it
    /// after. Translation is ignored: normals are directions.
    ///
    /// **When it returns no value** the model flattens the surface to a plane or a line, so it has
    /// no normals worth lighting. Callers should skip the normals for that object. If a matrix is
    /// still needed, use the **cofactor matrix** det·(M⁻¹)ᵀ: its rows are r1 × r2, r2 × r0, r0 × r1
    /// for the rows r0, r1, r2 of M. It points the same way as the normal matrix (flipped if
    /// det < 0), needs no division, and is defined even when M is singular; its length carries the
    /// factor det, so normalize after.
    ///
    /// Use when: lighting a mesh that is scaled non-uniformly; moving surface normals into world space
    /// Tags: normal matrix, inverse transpose, transform normals, normal transform, mat3
    /// Node: none (renderer shading)
    static std::optional<Matrix3> normalMatrix(const Matrix4& model);

private:
    Matrix3() = default;

    static Matrix3 fromColumns(Vector3 column0, Vector3 column1, Vector3 column2) {
        Matrix3 result;
        result.columns = {column0, column1, column2};
        return result;
    }

    /// Whether no entry is infinity or NaN.
    bool hasOnlyFiniteEntries() const { return columns[0].isFinite() && columns[1].isFinite() && columns[2].isFinite(); }

    std::array<Vector3, 3> columns;
};

} // namespace editor
