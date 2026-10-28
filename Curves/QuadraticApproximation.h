// A cubic as a spline of quadratics: how outlines become TrueType contours (graphics_cpp
// curves/fit `cubicToQuads`, after fontTools' cu2qu).
#pragma once

#include <vector>

#include "../Linear/Vector2.h"
#include "CubicBezier.h"

namespace editor {

/// Approximating a cubic Bézier by a quadratic spline.
///
/// Use when: exporting a curve to TrueType, which has only quadratics; any renderer that draws
///   quadratic pieces
/// Tags: cubic to quadratic, cu2qu, TrueType outline, quadratic spline, degree reduction
/// Node: planned (cubicToQuadratics)
struct QuadraticApproximation {
    /// The most pieces tried before the fallback: sixteen.
    static constexpr int mostPieces = 16;

    /// The control points of a quadratic spline that stays within `tolerance` of `cubic`.
    ///
    /// How (cu2qu, Fonttools): try one quadratic, then 2, 3 … up to sixteen equal-parameter pieces,
    /// until the spline fits. A quadratic spline has an off-curve control `qᵢ` per piece, and its
    /// on-curve joins are the midpoints of neighbouring controls, so n pieces are n controls
    /// and the spline starts at `cubic.start` and ends at `cubic.end`. For one piece the control
    /// is where the end tangents meet (if ahead of both ends). For n pieces the control of
    /// piece i is the point at its share of the line from `start + 1.5·(startHandle − start)`
    /// to `end + 1.5·(endHandle − end)` (the quadratic that matches a cubic's end tangents at
    /// both ends, best fit on average). A candidate fits when, for every piece, the
    /// difference cubic between the piece and the quadratic raised to a cubic stays inside
    /// `tolerance`; this is tested on its control points and by halving until they are inside.
    ///
    ///     cubic  ●╮       ╭●        one control per piece:
    ///             ╰──╮ ╭──╯         q₀      q₁      q₂
    ///                ╰─╯            ●───○───●───○───●  (○ implied joins)
    ///
    /// Conventions: y-up plane. The result has between 1 and 16 controls. A cubic too twisty for
    /// sixteen pieces gets sixteen controls placed at the meeting of each piece's end tangents
    /// (the midpoint of its handles where they do not meet): the best effort, not within the
    /// tolerance. A non-positive tolerance is never met, so the fallback is used.
    ///
    /// Source: Penner (2016) and Lowis, fontTools `cu2qu`; graphics_cpp curves/fit
    /// `cubicToQuads`.
    ///
    /// Use when: converting a font outline made of cubics to TrueType quadratics; drawing a
    ///   cubic with a quadratic-only renderer
    /// Tags: cu2qu, cubic to quad, quadratic approximation, TrueType, degree lowering
    /// Node: planned (cubicToQuadratics)
    static std::vector<Vector2> controlPoints(const CubicBezier<Vector2>& cubic, float tolerance);

    /// Whether the cubic of differences with control points `first … fourth` stays within
    /// `tolerance` of the origin.
    ///
    /// How: if both inner controls are within the tolerance of the origin the whole curve is
    /// (convex hull property, as the ends are tested by the caller); otherwise its midpoint
    /// `(p₀ + 3p₁ + 3p₂ + p₃)/8` must be inside and both halves (de Casteljau at ½) must pass.
    ///
    /// Conventions: the ends are not tested here. Depth is limited to 24 halvings, past which the
    /// answer is false.
    ///
    /// Use when: checking that a quadratic spline follows a cubic
    /// Tags: bounded difference curve, hull test, distance bound, subdivision test
    /// Node: none (solver)
    static bool staysWithin(Vector2 first, Vector2 second, Vector2 third, Vector2 fourth, float tolerance);
};

} // namespace editor
