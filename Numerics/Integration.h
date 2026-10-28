// Numerical integration: finding the area under a function when no formula for it is at hand
// (Engine/Math/Numerics/Integration.swift). Arc length needs it, because the length of a curve
// is the integral of its speed.
#pragma once

#include <array>
#include <functional>

namespace editor {

/// **Gauss–Legendre quadrature** with 5 sample points.
///
/// Source: Abramowitz & Stegun, *Handbook of Mathematical Functions*, 25.4.29 and Table 25.4.
///
/// Idea: instead of sampling the function at evenly spaced points, sample it at 5 cleverly chosen
/// points and add the samples up with 5 matching weights. The points and weights are chosen so
/// that the sum is **exact for every polynomial up to degree 9** (2 × 5 − 1). On the reference
/// interval −1…1:
///
///     ∫ f(x) dx  ≈  w₀·f(x₀) + w₁·f(x₁) + w₂·f(x₂) + w₃·f(x₃) + w₄·f(x₄)
///
/// Any other interval start…end is first mapped onto −1…1 (see `integral`).
///
/// Use when: integrating a smooth function numerically, such as curve speed to get length
/// Tags: Gauss-Legendre, numerical integration, quadrature, integrate, area under curve
/// Node: none (solver)
struct GaussLegendre {
    /// The 5 sample points on −1…1: the roots of the Legendre polynomial P₅. Symmetric around 0.
    /// A&S Table 25.4, n = 5.
    static constexpr std::array<float, 5> nodes = {
        -0.9061798459386640f, -0.5384693101056831f, 0.0f, 0.5384693101056831f, 0.9061798459386640f,
    };

    /// The weight of each sample point, in the same order as `nodes`. They add up to 2, the width
    /// of the interval −1…1. A&S Table 25.4, n = 5.
    static constexpr std::array<float, 5> weights = {
        0.2369268850561891f, 0.4786286704993665f, 0.5688888888888889f, 0.4786286704993665f, 0.2369268850561891f,
    };

    /// The area under `function` between `start` and `end`.
    ///
    /// How: split start…end into `pieceCount` equal pieces and apply the 5-point rule to each.
    /// For one piece a…b, with middle = (a + b)/2 and halfWidth = (b − a)/2, the node x on −1…1
    /// maps to the sample position middle + halfWidth·x, and the sum of weighted samples is
    /// scaled by halfWidth:
    ///
    ///     ∫ₐᵇ f  ≈  halfWidth · Σ wᵢ · f(middle + halfWidth·xᵢ)
    ///
    /// Conventions: exact for polynomials up to degree 9 within each piece. Functions that are not
    /// smooth (a kink, a jump) lose accuracy, so put piece borders at the kinks. If `end < start`
    /// the result is negative, as in school calculus. `pieceCount` must be at least 1, otherwise
    /// this throws a `GeometryProblem` (Swift traps).
    ///
    /// Use when: measuring arc length from a speed function; integrating any smooth function over a range
    /// Tags: integral, area under function, numerical integral, quadrature, definite integral
    /// Node: none (solver)
    static float integral(const std::function<float(float)>& function, float start, float end, int pieceCount = 1);
};

} // namespace editor
