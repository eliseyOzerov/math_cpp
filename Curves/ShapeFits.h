// Least-squares fits of a line, a circle and an ellipse to points, each with the root mean square
// of the points' distances from it, so a caller can tell how well a drawn stroke matches the shape.
// These are the measuring step of shape recognition (ShapeRecognition.h).
#pragma once

#include <optional>
#include <vector>

#include "../Geometry/Circle2.h"
#include "../Geometry/Ellipse2.h"
#include "../Geometry/Segment2.h"

namespace editor {

/// A line fitted to points, and how far the points are from it.
struct FittedLine {
    /// The mean of the points: the fitted line passes through it.
    Vector2 centroid;
    /// The line's unit direction, oriented from the first point towards the last.
    Vector2 direction;
    /// The part of the line the points cover: the feet of the first and the last point on it.
    Segment2 span;
    /// √(Σ dᵢ² / n), dᵢ the perpendicular distance of point i from the line.
    float rmsResidual = 0;
};

/// The line that is closest to `points` in the total-least-squares sense (smallest sum of squared
/// perpendicular distances), or nothing for fewer than two distinct points.
///
/// How: with the centroid p̄ and the scatter sums Sxx = Σ(x − x̄)², Sxy = Σ(x − x̄)(y − ȳ),
/// Syy = Σ(y − ȳ)², the best direction is the principal eigenvector of the scatter matrix
/// [Sxx Sxy; Sxy Syy] (principal component analysis): the angle θ = ½·atan2(2Sxy, Sxx − Syy).
/// The sum of squared perpendicular distances is the smaller eigenvalue, the least any line
/// through the points can have.
///
///      ●   ●  ●                 the fitted line minimises the perpendicular
///    ──────────────── θ          distances (│), not the vertical ones
///    ●  ●     ●   ●
///
/// Conventions: y-up plane; computed in double. Unlike y-on-x regression it treats x and y alike,
/// so a vertical line fits as well as a horizontal one. The span runs between the projections of
/// the first and last points. Non-finite points give nothing.
///
/// Source: K. Pearson, "On lines and planes of closest fit to systems of points in space",
/// Philosophical Magazine 2 (1901); Golub & Van Loan, *Matrix Computations* (1996), §12.3
/// (total least squares).
///
/// Use when: recognising a drawn stroke as a straight line; the axis of a cloud of points
/// Tags: line fit, total least squares, orthogonal regression, PCA line, principal axis, Deming
/// Node: planned (fitLine)
std::optional<FittedLine> fittedLine(const std::vector<Vector2>& points);

/// A circle fitted to points, and how far the points are from it.
struct FittedCircle {
    Circle2 circle;
    /// √(Σ (|pᵢ − centre| − r)² / n): the root mean square of the points' distances from the
    /// circle.
    float rmsResidual = 0;
};

/// The circle of Kåsa's algebraic fit: the one that minimises Σ (|pᵢ − c|² − r²)², or nothing
/// for fewer than three points or points on a line.
///
/// How: in coordinates u = x − x̄, v = y − ȳ about the centroid, the condition is linear in the
/// centre (u_c, v_c) and in r²:
///     [Suu Suv] [u_c]   ½ [Suuu + Suvv]
///     [Suv Svv] [v_c] =   [Svvv + Suuv],   r² = u_c² + v_c² + (Suu + Svv)/n,
/// with Suv = Σuᵢvᵢ, Suuv = Σuᵢ²vᵢ and so on. It is exact on points of a circle, but it weighs
/// each point by about 2r times its distance, so on a short noisy arc the radius comes out too
/// small; `fittedCircle` refines it.
///
/// Conventions: y-up plane; computed in double. Points on a line (a singular 2×2 system, relative
/// to the scatter) give nothing.
///
/// Source: I. Kåsa, "A circle fitting procedure and its error analysis", IEEE Transactions on
/// Instrumentation and Measurement 25 (1976); the centred form is R. Bullock, "Least-squares
/// circle fit" (2006).
///
/// Use when: a quick circle through many points; the starting guess of a geometric fit
/// Tags: Kasa fit, algebraic circle fit, least squares circle, circle regression
/// Node: planned (fitCircle)
std::optional<FittedCircle> fittedCircleAlgebraic(const std::vector<Vector2>& points);

/// The circle that is closest to `points` in the geometric sense (smallest Σ (|pᵢ − c| − r)²), or
/// nothing for fewer than three points or points on a line.
///
/// How: Kåsa's fit (`fittedCircleAlgebraic`) gives the start; Gauss–Newton then refines (c, r).
/// Each residual is eᵢ = |pᵢ − c| − r with gradient (−(pᵢ − c)/|pᵢ − c|, −1), so each step solves
/// the 3×3 normal equations JᵀJ·δ = −Jᵀe and moves (c, r) by δ. It stops when a step is below
/// 1e-12 of the circle's size, or after 50 steps, and keeps a step only while it lowers the sum
/// of squares (so the result is never worse than Kåsa's).
///
///        ● ●               Kåsa's circle (algebraic) is shrunk on a short arc;
///      ●     ●             the geometric fit pulls it back out to the points
///
/// Conventions: y-up plane; computed in double. A point exactly at the centre has no gradient
/// direction and counts with zero weight in that step.
///
/// Source: N. Chernov & C. Lesort, "Least squares fitting of circles", Journal of Mathematical
/// Imaging and Vision 23 (2005), §3 (geometric fit by Gauss–Newton / Levenberg–Marquardt);
/// starting from Kåsa (1976).
///
/// Use when: recognising a drawn circle or arc; finding the radius of a scanned round edge
/// Tags: circle fit, geometric circle fit, Gauss-Newton, orthogonal distance regression, arc fit
/// Node: planned (fitCircle)
std::optional<FittedCircle> fittedCircle(const std::vector<Vector2>& points);

/// An ellipse fitted to points, and how far the points are from it.
struct FittedEllipse {
    Ellipse2 ellipse;
    /// √(Σ dᵢ² / n), dᵢ the distance of point i from the ellipse's outline (`Ellipse2::distance`).
    float rmsResidual = 0;
};

/// The ellipse of the direct least-squares fit, or nothing for fewer than five points or points
/// that no ellipse fits (on a line, or a conic that comes out a hyperbola or parabola).
///
/// How: a conic is a·x² + b·xy + c·y² + d·x + e·y + f = 0, an ellipse when 4ac − b² > 0. Fitzgibbon,
/// Pilu and Fisher minimise the algebraic error Σ (conic at pᵢ)² under 4ac − b² = 1, which always
/// yields an ellipse. Halíř and Flusser split the design matrix into its quadratic part
/// D₁ = [x², xy, y²] and linear part D₂ = [x, y, 1], with S₁ = D₁ᵀD₁, S₂ = D₁ᵀD₂, S₃ = D₂ᵀD₂:
/// the linear coefficients are (d, e, f) = T·(a, b, c) with T = −S₃⁻¹S₂ᵀ, and (a, b, c) is the
/// eigenvector of C₁⁻¹(S₁ + S₂T) with 4ac − b² > 0, C₁ = [0 0 2; 0 −1 0; 2 0 0] (the constraint).
/// That avoids the singular 6×6 problem of the original. The points are first moved to their
/// centroid and scaled to unit spread, which keeps the 3×3 matrices well conditioned. The conic
/// then gives the centre (where its gradient vanishes), the rotation ½·atan2(b, a − c), and the
/// semi-axes √(−f′/λ) along each axis, with f′ the conic's value at the centre and λ the
/// quadratic form along that axis.
///
/// Conventions: y-up plane; computed in double. Exact on points of an ellipse (a circle comes
/// out as an ellipse with equal semi-axes, in any rotation). The 3×3 eigenvalues are the roots
/// of the characteristic cubic, solved by Viète's trigonometric form.
///
/// Source: A. Fitzgibbon, M. Pilu & R. Fisher, "Direct least square fitting of ellipses", IEEE
/// Transactions on Pattern Analysis and Machine Intelligence 21 (1999); R. Halíř & J. Flusser,
/// "Numerically stable direct least squares fitting of ellipses", WSCG (1998).
///
/// Use when: recognising a drawn oval; the outline of a round thing seen at an angle
/// Tags: ellipse fit, direct least squares, Fitzgibbon, Halir Flusser, conic fit, oval fit
/// Node: planned (fitEllipse)
std::optional<FittedEllipse> fittedEllipse(const std::vector<Vector2>& points);

} // namespace editor
