// A 4×4 matrix and its inverse (Engine/Math/Matrix4.swift, Engine/Math/Algebra/Matrix4+Inverse.swift).
// The camera matrices (Matrix4+Camera.swift) are renderer-only and not ported. Swift keeps a
// simd_float4x4; this keeps four column Vector4s in the same order (64 bytes, ready for a GPU).
// Products are written out as column-weighted sums; simd may fuse or reorder the additions, so a
// result can differ from Swift's in the last bit.
#pragma once

#include <array>
#include <optional>

#include "Angle.h"
#include "Vector4.h"

namespace editor {

/// A 4×4 matrix: one object that moves, turns or scales points. Several of these can be
/// combined into one.
///
/// **Why 4×4 for 3D?** A 3×3 matrix can rotate and scale, but it can't move (translate) a
/// point. The trick is to give each point a 4th coordinate, w = 1. The 4th column of the matrix
/// is then added to the point as an offset:
///
///     ┌ m00 m01 m02 tx ┐   ┌ x ┐   ┌ m00·x + m01·y + m02·z + tx ┐
///     │ m10 m11 m12 ty │ × │ y │ = │ m10·x + m11·y + m12·z + ty │
///     │ m20 m21 m22 tz │   │ z │   │ m20·x + m21·y + m22·z + tz │
///     └  0   0   0   1 ┘   └ 1 ┘   └             1              ┘
///
/// A direction (rather than a point) gets w = 0. That way it is rotated and scaled but never
/// moved, which is right: "north" stays "north" wherever you stand.
///
/// **Conventions:**
/// - Points are column vectors on the right: `transformed = matrix × point`.
/// - `a * b` means "apply `b` first, then `a`". The transform written closest to the point
///   happens first.
/// - Matrices are written **row by row**, as in a textbook. They are stored **column by
///   column**, which is the order Metal's `float4x4` expects, so a `Matrix4` can go to the GPU
///   unchanged (64 bytes).
///
/// Use when: moving, turning and scaling an object, a camera or a bone in one value
/// Tags: 4x4 matrix, mat4, float4x4, transform, model matrix, homogeneous
/// Node: none (value type)
class Matrix4 {
public:
    /// Builds a matrix from its four rows, written top to bottom, the same way it is written on paper.
    Matrix4(Vector4 row0, Vector4 row1, Vector4 row2, Vector4 row3)
        : columns{Vector4(row0.x, row1.x, row2.x, row3.x), Vector4(row0.y, row1.y, row2.y, row3.y),
                  Vector4(row0.z, row1.z, row2.z, row3.z), Vector4(row0.w, row1.w, row2.w, row3.w)} {}

    /// The column `index` (0…3), left to right, as stored: the 4th column of a translation holds
    /// the offset. Swift's tests read `storage.columns`.
    Vector4 column(int index) const { return columns[index]; }

    /// The entry in `row`, `column` (both 0…3). Swift's tests read `storage[column][row]`.
    float entry(int row, int column) const { return columns[column][row]; }

    /// The four rows, top to bottom, as written on paper. (The matrix is stored by columns, so
    /// reading a row means picking one component from each column.) The inverse needs rows
    /// because its 2×2 minors pair up rows.
    ///
    /// Use when: reading a 4x4 matrix row by row, for example in the inverse or in tests
    /// Tags: matrix rows, row vectors, mat4 rows, matrix access
    /// Node: none (value type)
    std::array<Vector4, 4> rows() const {
        return {Vector4(columns[0].x, columns[1].x, columns[2].x, columns[3].x),
                Vector4(columns[0].y, columns[1].y, columns[2].y, columns[3].y),
                Vector4(columns[0].z, columns[1].z, columns[2].z, columns[3].z),
                Vector4(columns[0].w, columns[1].w, columns[2].w, columns[3].w)};
    }

    // MARK: Basic transforms

    /// The matrix that changes nothing: every point stays where it is. It plays the same role for
    /// matrices that 1 does for multiplication.
    static Matrix4 identity() {
        return Matrix4(Vector4(1, 0, 0, 0), Vector4(0, 1, 0, 0), Vector4(0, 0, 1, 0), Vector4(0, 0, 0, 1));
    }

    /// Moves every point by `offset`.
    ///
    /// The offset goes in the 4th column. A point's w = 1 picks it up, so (x, y, z) becomes
    /// (x + offset.x, y + offset.y, z + offset.z). Directions (w = 0) are not affected.
    ///
    /// Use when: moving an object to a position; placing a camera
    /// Tags: translation matrix, move, translate, offset, position matrix
    /// Node: inside transform, instance
    static Matrix4 translation(Vector3 offset) {
        return Matrix4(Vector4(1, 0, 0, offset.x), Vector4(0, 1, 0, offset.y), Vector4(0, 0, 1, offset.z),
                       Vector4(0, 0, 0, 1));
    }

    /// Stretches every point away from the origin (or squeezes it toward it), separately along
    /// each axis: (x, y, z) becomes (x × factors.x, y × factors.y, z × factors.z).
    ///
    /// A factor of 1 leaves that axis alone, 2 doubles it, and 0.5 halves it. A negative factor
    /// mirrors along that axis.
    ///
    /// Use when: resizing an object, or flattening or stretching it along one axis
    /// Tags: scale matrix, resize, stretch, non-uniform scale, scaling
    /// Node: inside transform, instance
    static Matrix4 scale(Vector3 factors) {
        return Matrix4(Vector4(factors.x, 0, 0, 0), Vector4(0, factors.y, 0, 0), Vector4(0, 0, factors.z, 0),
                       Vector4(0, 0, 0, 1));
    }

    /// Turns every point around an axis through the origin (Swift: `rotation(around:by:)`).
    ///
    /// The turn is **counterclockwise when you look down the axis from its tip toward the origin**
    /// (the right-hand rule: point your right thumb along the axis and your fingers curl the way
    /// of a positive turn). For example, a positive turn around `Vector3::zAxis()` takes the x axis
    /// toward the y axis.
    ///
    /// This is Rodrigues' rotation formula, written as a matrix. With (x, y, z) the axis at length
    /// 1, c = cos(angle), s = sin(angle) and t = 1 − c:
    ///
    ///     ┌ t·x·x + c     t·x·y − s·z   t·x·z + s·y   0 ┐
    ///     │ t·x·y + s·z   t·y·y + c     t·y·z − s·x   0 │
    ///     │ t·x·z − s·y   t·y·z + s·x   t·z·z + c     0 │
    ///     └ 0             0             0             1 ┘
    ///
    /// Around the z axis (x = 0, y = 0, z = 1) this reduces to the familiar 2D rotation:
    /// x' = c·x − s·y and y' = s·x + c·y.
    ///
    /// `axis` doesn't need to have length 1, because it is normalized here. It must not be zero,
    /// since a zero vector has no direction to turn around (a `MathProblem` is thrown).
    ///
    /// Use when: spinning an object about an axis; turning a camera
    /// Tags: rotation matrix, rotate about axis, axis angle, Rodrigues, spin
    /// Node: none (value type)
    static Matrix4 rotation(Vector3 axis, Angle angle);

    // MARK: Combining and applying

    /// Combines two transforms into one that does both: first `other`, then this one.
    ///
    /// `rotation * scale` scales a point first, then rotates it. The order matters. Moving then
    /// turning puts a point somewhere other than turning then moving.
    ///
    /// How: the entry in row i, column j of the result is the dot product of row i of this matrix
    /// with column j of `other`; each result column is this matrix applied to that column.
    ///
    /// Use when: building a model matrix from scale, rotation and translation; chaining a parent's and a child's transform
    /// Tags: matrix multiply, combine transforms, concatenate, compose, model view projection, mat4 product
    /// Node: none (value type)
    Matrix4 operator*(const Matrix4& other) const {
        Matrix4 result = *this;
        for (int index = 0; index < 4; index += 1) result.columns[index] = transform(other.columns[index]);
        return result;
    }

    /// Applies the matrix to a 4-component vector: `matrix × vector`.
    ///
    /// This is the general form. `transformPoint` and `transformDirection` call it with w = 1 and
    /// w = 0. Each component of the result is the dot product of one row of the matrix with `vector`.
    ///
    /// How: column 0 × x + column 1 × y + column 2 × z + column 3 × w, added left to right.
    ///
    /// Use when: applying a projection to a homogeneous point before the perspective divide
    /// Tags: matrix times vector, transform vector4, homogeneous coordinates, mul, apply matrix
    /// Node: inside transform, instance
    Vector4 transform(Vector4 vector) const {
        return columns[0] * vector.x + columns[1] * vector.y + columns[2] * vector.z + columns[3] * vector.w;
    }

    /// Applies the matrix to a **point** (w = 1), so translation affects it.
    ///
    /// Works for moves, turns and scales, which all leave w = 1. A projection changes w; for
    /// that, use `transform(Vector4)` and divide by w.
    ///
    /// Use when: moving a vertex or a mouse-picked position into another space
    /// Tags: transform point, apply matrix to point, MultiplyPoint, local to world, move point
    /// Node: inside transform, instance
    Vector3 transformPoint(Vector3 point) const { return transform(Vector4(point, 1)).xyz(); }

    /// Applies the matrix to a **direction** (w = 0), so translation is ignored. Only rotation
    /// and scale affect it.
    ///
    /// Use when: turning a normal, an axis or a ray direction without moving it
    /// Tags: transform direction, MultiplyVector, rotate direction, ignore translation, vector transform
    /// Node: none (renderer shading)
    Vector3 transformDirection(Vector3 direction) const { return transform(Vector4(direction, 0)).xyz(); }

    // MARK: Inverse

    /// The matrix that undoes this one (inverse × self = identity), or no value if there is none
    /// because the matrix is singular (it flattens space). Picking uses it: the inverse of
    /// `projection × view` turns a screen point back into a ray in the world.
    ///
    /// Method: Cramer's rule with the adjugate, inverse = adjugate / determinant. Entry (i, j) of
    /// the adjugate is the cofactor of entry (j, i): the determinant of the 3×3 that is left when
    /// row j and column i are deleted, with sign (−1)^(i+j). This is the standard cofactor
    /// (Laplace) expansion; the arrangement into 2×2 minors of row pairs below is a common way to
    /// write the 4×4 case out without loops.
    ///
    /// To keep 16 cofactors readable, they are built from 2×2 determinants of **row pairs**. Name
    /// the rows a, b (top two) and c, d (bottom two) and take the 2×2 determinant of any two
    /// columns i < j:
    ///
    ///     top(i, j)    = a[i]·b[j] − a[j]·b[i]
    ///     bottom(i, j) = c[i]·d[j] − c[j]·d[i]
    ///
    /// There are 6 column pairs, so 6 of each, numbered 0…5 for the pairs (0,1) (0,2) (0,3) (1,2)
    /// (1,3) (2,3). Laplace expansion along the two row-pairs gives
    ///
    ///     determinant = top₀·bottom₅ − top₁·bottom₄ + top₂·bottom₃
    ///                 + top₃·bottom₂ − top₄·bottom₁ + top₅·bottom₀
    ///
    /// and every cofactor is a sum of three terms of the same kind. For example, the cofactor of
    /// entry (0, 0) deletes row 0 and column 0 and expands what is left along row b:
    /// b[1]·bottom₅ − b[2]·bottom₄ + b[3]·bottom₃.
    ///
    /// **No inverse (no value):**
    /// - **Moves, turns and scales** (bottom row exactly 0 0 0 1): the determinant equals that of
    ///   the upper-left 3×3, and the translation column has no say in whether space is flattened.
    ///   So the test is the `Matrix3` one (`Matrix3::isSingular`, rows nearly parallel, judged
    ///   against the block's own scale). Using the 4-component rows would let a 1 km translation
    ///   make a perfectly good matrix look singular. 1 mm scale (det 1e-9) and 1 km translations
    ///   are both fine.
    /// - **Anything else** (projections, projection × view): only a determinant of exactly 0. No
    ///   scale-relative test is reliable here: for P × V a Hadamard ratio shrinks with the eye's
    ///   distance from the origin, because the translation column grows the row lengths without
    ///   making the matrix any less invertible. A true inverse of such a matrix is just badly
    ///   conditioned, not absent.
    /// - **Either case:** the result is also rejected when the reciprocal of the determinant or
    ///   any entry of the result is not a finite number (the determinant is so small that dividing
    ///   by it overflows float).
    ///
    /// **Picking:** inverting projection × view works at any eye distance. For the best precision,
    /// invert projection and view separately instead and apply the two inverses in turn.
    ///
    /// Use when: going from clip or world space back to object space; unprojecting a mouse click into a ray
    /// Tags: matrix inverse, invert mat4, inverse view matrix, unproject, undo transform, inverse()
    /// Node: none (solver)
    std::optional<Matrix4> inverse() const;

    /// Whether no entry is infinity or NaN.
    ///
    /// Use when: refusing a matrix that came out of a division by zero
    /// Tags: finite entries, NaN check, infinity check
    /// Node: none (input checking)
    bool hasOnlyFiniteEntries() const {
        return columns[0].isFinite() && columns[1].isFinite() && columns[2].isFinite() && columns[3].isFinite();
    }

private:
    /// The 16 numbers, column by column, the way Metal expects them.
    std::array<Vector4, 4> columns;
};

static_assert(sizeof(Vector4) == 16, "a Vector4 is four floats with no padding");
static_assert(sizeof(Matrix4) == 64, "a Matrix4 is sixteen floats with no padding, so it can be copied to the GPU");

} // namespace editor
