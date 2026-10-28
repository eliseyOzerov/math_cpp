// Real roots of a quadratic equation, computed in a way that does not lose digits
// (Engine/Math/Numerics/QuadraticRoots.swift).
#pragma once

namespace editor {

/// The real solutions of `quadratic·x² + linear·x + constant = 0`: none, one or two.
///
/// Swift's enum cases `.none`, `.one(root)` and `.two(smaller:larger:)` are the factory
/// functions `none()`, `one(root)` and `two(smaller, larger)`; `kind` tells which one it is.
///
/// Use when: reporting zero, one or two solutions of an equation of degree two
/// Tags: quadratic roots, solutions, discriminant, equation result, roots
struct QuadraticRoots {
    enum class Kind { none, one, two };

    Kind kind = Kind::none;
    /// The smaller root of `two`, or the single root of `one`. 0 for `none`.
    float smaller = 0;
    /// The larger root of `two`, or the single root of `one`. 0 for `none`.
    float larger = 0;

    bool operator==(const QuadraticRoots&) const = default;

    /// No real solution: the parabola never crosses zero (or the equation is `constant = 0` with
    /// `constant ≠ 0`, or `0 = 0`, which has no single answer).
    static QuadraticRoots none() { return QuadraticRoots{Kind::none, 0, 0}; }
    /// Exactly one solution: the parabola touches zero without crossing it, or the equation is
    /// really linear.
    static QuadraticRoots one(float root) { return QuadraticRoots{Kind::one, root, root}; }
    /// Two different solutions, `smaller < larger`.
    static QuadraticRoots two(float smaller, float larger) { return QuadraticRoots{Kind::two, smaller, larger}; }

    /// The single root of a `one`.
    float root() const { return smaller; }

    /// The real roots of `quadratic·x² + linear·x + constant = 0`.
    ///
    /// Source: Press et al., *Numerical Recipes* §5.6, "Quadratic and Cubic Equations".
    ///
    /// The textbook formula x = (−linear ± √discriminant) / (2·quadratic) loses digits when
    /// linear² is much larger than 4·quadratic·constant: then √discriminant is almost |linear|,
    /// and one of the two sums subtracts two nearly equal numbers. The stable method never does
    /// that subtraction:
    ///
    ///     discriminant = linear² − 4·quadratic·constant
    ///     stableHalfSum = −½·(linear + sign(linear)·√discriminant)
    ///     roots         = stableHalfSum / quadratic   and   constant / stableHalfSum
    ///
    /// `stableHalfSum` adds two numbers of the same sign, so it is accurate. The second root comes
    /// from Vieta's formula: the two roots multiply to constant / quadratic.
    ///
    /// Conventions and edge cases:
    /// - discriminant < 0: `none`. discriminant == 0 exactly: `one`. A grazing case that rounds to
    ///   a tiny negative number reports `none`.
    /// - quadratic == 0 exactly: the equation is linear, so the answer is `one(−constant / linear)`.
    ///   If `linear` is also 0 there is no single solution, and the result is `none`.
    /// - A tiny but non-zero `quadratic` is not special-cased.
    /// - Results are in ascending order. No tolerance is used; inputs are taken as exact.
    /// - A NaN discriminant (b² and 4ac both overflowing to infinity) also reports `none`.
    ///
    /// Use when: intersecting a ray with a sphere; finding where a curve crosses a line
    /// Tags: quadratic formula, solve quadratic, roots of equation, discriminant, ax2+bx+c, find roots
    /// Node: none (solver)
    static QuadraticRoots solve(float quadratic, float linear, float constant);
};

} // namespace editor
