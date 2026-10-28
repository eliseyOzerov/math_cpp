// Real roots of a cubic equation (Engine/Math/Numerics/CubicRoots.swift). Bézier curves need
// them: the extremes of a cubic curve, where it is furthest left or highest, are the roots of
// its derivative, and a crossing with a line is a root of a cubic.
#pragma once

#include <vector>

namespace editor {

/// The real solutions of `cubic·t³ + quadratic·t² + linear·t + constant = 0`.
///
/// Use when: finding where a cubic curve crosses a line or reaches its extreme; solving any equation of degree three
/// Tags: cubic roots, cubic equation, Kahan, Newton, deflation, solve cubic, roots of polynomial, discriminant
/// Node: none (solver)
struct CubicRoots {
    /// How many units of rounding (`Double.ulpOfOne`, 2.2e-16) of the sizes of its terms the cubic
    /// may be away from zero and still count as zero, when deciding whether two roots are really
    /// one double root.
    ///
    /// Evaluating a cubic by Horner's rule rounds six times, so the computed value is off by at
    /// most about 3·ulpOfOne times the sum of the sizes of its terms (Higham, *Accuracy and
    /// Stability of Numerical Algorithms*, 2002, §5.1). Every (t − r)²(t − s) with whole r ≠ s in
    /// −60…60 needs at least 3 to keep its double root; 8 is about three times that.
    static constexpr double roundingAllowance = 8;

    /// The real roots of `cubic·t³ + quadratic·t² + linear·t + constant = 0`, in ascending order,
    /// each listed once: 0, 1, 2 or 3 numbers.
    ///
    /// Sources: Kahan, "To Solve a Real Cubic Equation" (1986), for the method: a Newton
    /// iteration from a starting point that cannot miss, then deflation to a quadratic in the
    /// form that does not cancel; Press et al., *Numerical Recipes* §5.6, for the stable
    /// quadratic; Blinn, "How to Solve a Cubic Equation" part 5 (IEEE CG&A, 2007), for why the
    /// closed forms (Cardano, Viète) are not used: they shift by quadratic/(3·cubic), which wipes
    /// out the small roots when the cubic term is small.
    ///
    /// Method, all in `double`:
    ///
    /// 1. **Scale.** Substitute t = 2ᵏ·u, with 2ᵏ about the size of the largest root (from the
    ///    bound max(|b/a|, √|c/a|, ∛|d/a|)), and multiply the equation by a power of two so its
    ///    leading coefficient is about 1. Multiplying by powers of two is exact, so nothing
    ///    overflows, even for float coefficients from 1e-45 to 3e38.
    /// 2. **One root by Newton.** Start at the inflection point X = −B/(3A) and step outward to a
    ///    point that cannot be inside the root cluster (Kahan; 1.324718 is the plastic number),
    ///    then Newton's method moves towards the outermost root without overshooting; it stops
    ///    when a step no longer moves forward.
    /// 3. **Deflate.** Dividing by (u − r) leaves A·u² + b₁·u + c₂ (synthetic division). When the
    ///    found root is the big one, b₁ and c₂ are taken from the other end of the cubic:
    ///    c₂ = −D/r and b₁ = (c₂ − C)/r.
    /// 4. **The quadratic**, by the stable formula: h = −b₁/2, discriminant = h² − A·c₂, roots
    ///    c₂/s and s/A with s = h + sign(h)·√discriminant. A negative discriminant with the cubic
    ///    and its slope both zero within rounding at the vertex h/A is a double root there.
    /// 5. **Merge.** Two neighbouring roots whose midpoint is a zero of the cubic within rounding
    ///    cannot be told apart in `double`: they are one double root, reported at the midpoint.
    /// 6. **Unscale** each root by 2ᵏ and round to float; a root too big for float is dropped.
    ///
    /// Conventions and edge cases:
    /// - cubic == 0 exactly: solved as the quadratic with `QuadraticRoots`. A tiny but non-zero
    ///   `cubic` is a cubic: 1e-15·t³ + t² − 3t + 2 has the roots −1e15, 1 and 2.
    /// - All four coefficients zero: no roots. `cubic ≠ 0` with the other three zero is the
    ///   triple root 0.
    /// - A root bigger than the largest float is not reported; a root smaller than the smallest
    ///   float comes back as 0.
    /// - A root of multiplicity two or three is listed once, and so are roots too close for
    ///   `double` to separate and roots that become the same float.
    /// - A coefficient that is NaN or infinite gives no roots.
    ///
    /// Use when: finding where a cubic Bézier curve is extreme in x or y (roots of its derivative); finding where a cubic curve crosses an axis
    /// Tags: cubic formula, solve cubic, Kahan cubic, Newton deflation, depressed cubic, roots of t^3, cubic discriminant, third degree equation
    /// Node: none (solver)
    static std::vector<float> solve(float cubic, float quadratic, float linear, float constant);

    /// The real roots that lie in `lowerBound ... upperBound` (Swift `solve(..., within:)`),
    /// ascending. The same method, conventions and edge cases as `solve`.
    ///
    /// A root a hair outside the interval counts as inside and is moved onto the nearest end: a
    /// root that is exactly at an end, such as the 1 of t³ − t, can come out a float step outside.
    /// The slack is 4 float steps (ulps) of the larger finite end, relative to the interval's own
    /// numbers, so it is the same at every scale. An infinite end has no slack and needs none.
    ///
    /// Use when: finding the extremes of a Bézier curve that lie on the curve itself (the parameter range 0…1); finding where a curve segment crosses a line
    /// Tags: roots in interval, roots between 0 and 1, bounded roots, Bezier extrema roots, cubic roots on a range
    /// Node: none (solver)
    static std::vector<float> solve(float cubic, float quadratic, float linear, float constant, float lowerBound,
                                    float upperBound);
};

} // namespace editor
