// Small dense linear systems, solved by Gaussian elimination (graphics_cpp curves/bezn `solve`).
//
// The systems here are the normal equations of a least-squares curve fit: a handful of unknowns
// (a Bézier of order ten has eleven control points), where elimination with a row swap is exact
// enough and needs no linear-algebra package. Work is in Double, as the fitters that call it are.
#pragma once

#include <vector>

namespace editor {

/// A square or tall system of linear equations `matrix · x = constants`, and its solutions.
///
/// Use when: solving for the control points of a fitted curve; any small system that Matrix3 and
///   Matrix4 are too small for
/// Tags: linear system, Gaussian elimination, solve Ax = b, least squares, normal equations
/// Node: none (numerics helper of fitBezier)
struct LinearSystem {
    using Row = std::vector<double>;
    using Matrix = std::vector<Row>;

    /// The solution of `coefficients · x = constants` for a square system.
    ///
    /// How: Gauss–Jordan elimination with partial pivoting. For each column the row whose entry
    /// is largest in size is swapped up (this keeps the multipliers at most 1 in size), and its
    /// multiples are subtracted from every other row; the solution is then each right-hand side
    /// over its diagonal entry.
    ///
    ///     [ 2 1 | 3 ]      [ 2 0 | 1.6 ]
    ///     [ 1 3 | 5 ]  →   [ 0 2.5 | 3.5 ]   x = (0.8, 1.4)
    ///
    /// Conventions: a column whose pivot is smaller than 1e-14 in size is singular; its unknown
    /// is returned as 0 (the equations keep what they can, as the original did). Throws a
    /// `MathProblem` when the matrix is not square or its size differs from the constants.
    ///
    /// Source: Gauss–Jordan elimination (Golub & Van Loan, *Matrix Computations*, 1996, §3.2).
    ///
    /// Use when: finding the unknown handles of a curve from its normal equations
    /// Tags: solve, Gaussian elimination, Gauss-Jordan, partial pivoting, Ax=b, linsolve
    /// Node: none (numerics helper of fitBezier)
    static std::vector<double> solved(const Matrix& coefficients, const Row& constants);

    /// The `x` that makes `rows · x` closest to `constants` in the least-squares sense, for a
    /// system with more equations than unknowns.
    ///
    /// How: the normal equations `(Aᵀ A) x = Aᵀ b`, solved by `solved`.
    ///
    /// Conventions: `rows` has one row per equation and the same number of columns in each.
    /// Throws a `MathProblem` when the row count differs from the constants or the rows differ in
    /// length. Fewer equations than unknowns leave the extra unknowns at 0 (singular, see
    /// `solved`).
    ///
    /// Source: Golub & Van Loan, *Matrix Computations* (1996), §5.3.
    ///
    /// Use when: fitting Bézier control points to many points
    /// Tags: least squares, normal equations, regression, overdetermined system, fit
    /// Node: none (numerics helper of fitBezier)
    static std::vector<double> leastSquares(const Matrix& rows, const Row& constants);
};

} // namespace editor
