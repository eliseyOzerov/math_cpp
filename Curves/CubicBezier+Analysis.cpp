// What a flat cubic Bézier curve looks like as a shape: where it bulges out furthest, which
// rectangle holds it tightly, where it switches from bending left to bending right, how sharply
// it bends, and how much area it sweeps. All of it is exact algebra on the curve's polynomial,
// no sampling (Engine/Math/Curves/CubicBezier+Analysis.swift).
//
// The curve is a polynomial in the parameter t, and it is easiest to differentiate in the
// "power basis". Writing the control points P0…P3 (`start`, `startHandle`, `endHandle`, `end`)
// and measuring them from the start, dᵢ = Pᵢ − P0:
//
//     B(t) = P0 + a·t³ + b·t² + c·t
//     a = d3 − 3·d2 + 3·d1      b = 3·d2 − 6·d1      c = 3·d1
//
// which follows by multiplying out the Bernstein form (the P0 terms of a, b, c add up to zero,
// so they can be dropped). Each of a, b, c is a vector. Building them from the differences dᵢ,
// not from the absolute points, matters far from the origin: at x = 1e5 a Float point is only
// known to 0.008, and `−P0 + 3·P1 − 3·P2 + P3` would cancel those large numbers down to a small
// one, keeping the error of the large ones. The differences are small to begin with.
#include <algorithm>
#include <cmath>
#include <limits>

#include "../Numerics/QuadraticRoots.h"
#include "../Linear/Scalar.h"
#include "CubicBezier.h"

namespace editor {

// MARK: - Shared pieces

namespace {

/// A quadratic `quadratic·t² + linear·t + constant` whose coefficients were computed in Float,
/// each with an upper bound on its rounding error, and the roots at which it changes sign.
///
/// Rounding bounds, with ε = `Float.ulpOfOne` = 2⁻²³ (one rounding errs by at most ε/2 of
/// the result):
/// - A coefficient vector a, b or c summed from the differences dᵢ: each difference is off
///   by at most ε/2 of itself, and each of the (at most three) additions and multiplications
///   by ε/2 of the terms' sizes, so each component errs by at most `4·ε/2 = 2ε` times its size
///   (A, B or C of `PowerBasis`).
/// - A cross product `u × v = uₓ·v_y − u_y·vₓ`, with components at most U and V that err by
///   at most 2εU and 2εV: the errors carried in give `2·(2εU·V + U·2εV) = 8εUV`; the two
///   products and the subtraction add at most `3·(ε/2)·…` of `2UV`, under `2εUV`. In all
///   `10ε·U·V` (`crossProductBound`).
struct QuadraticWithRoundingBounds {
    float quadratic;
    float quadraticBound;
    float linear;
    float linearBound;
    float constant;
    float constantBound;

    /// The rounding bound `10ε·U·V` of a Float cross product of two vectors whose components
    /// are at most `firstSize` and `secondSize` and carry errors up to 2ε of those sizes
    /// (derivation on the type).
    ///
    /// Use when: bounding the error of a coefficient made from a 2D cross product
    /// Tags: rounding error bound, cross product error, forward error analysis
    /// Node: none (solver)
    static float crossProductBound(float firstSize, float secondSize) {
        return 10 * std::numeric_limits<float>::epsilon() * firstSize * secondSize;
    }

    /// The sign of the exact quadratic at `parameter` when rounding cannot have flipped it:
    /// +1 or −1, and 0 when the computed value is within its error bound.
    ///
    /// How: the value is `(quadratic·t + linear)·t + constant` (Horner). Its error is at most
    /// the coefficient errors carried through, `quadraticBound·t² + linearBound·|t| +
    /// constantBound`, plus the rounding of the evaluation itself, two operations per step,
    /// under `2ε·(|quadratic|·t² + |linear|·|t| + |constant|)`.
    ///
    /// Use when: deciding whether the quadratic is really positive or negative at a point
    /// Tags: certified sign, Horner evaluation, error bound, robust predicate
    /// Node: none (solver)
    float certainSign(float parameter) const {
        float value = (quadratic * parameter + linear) * parameter + constant;
        float size = std::fabs(parameter);
        float evaluationRounding = 2 * std::numeric_limits<float>::epsilon();
        float errorBound = (quadraticBound + evaluationRounding * std::fabs(quadratic)) * size * size
            + (linearBound + evaluationRounding * std::fabs(linear)) * size
            + constantBound + evaluationRounding * std::fabs(constant);
        if (!(std::fabs(value) > errorBound)) return 0;
        return value > 0 ? 1 : -1;
    }

    /// The roots in `range` at which the quadratic certainly changes sign, ascending.
    ///
    /// How: the roots come from `QuadraticRoots.solve` on the computed coefficients. A root
    /// is kept only when the quadratic has a *certain* sign on both sides of it, and the two
    /// signs differ (`certainSign(at:)`). The sides are tested midway to the neighbouring
    /// root, or one unit away when there is none or it is further: the error bound grows
    /// with t², so testing far out, by a distant root of a nearly linear quadratic, would
    /// find nothing certain. If both test values are certain and
    /// opposite, the exact quadratic changes sign between them too, so every root reported
    /// is a true sign change of the exact quadratic, up to where in between it lies.
    ///
    ///     certain +   ┊ uncertain ┊   certain −
    ///     ────────────┼─────●─────┼────────────▶ t      kept: + on one side, − on the other
    ///
    /// This leaves out, without any special case: a double root (same sign on both sides);
    /// two roots closer together than rounding can separate (the value between them is too
    /// small to be certain); and the roots of pure rounding noise, such as those of the
    /// curvature of a straight curve (no value anywhere is certain). Banding each coefficient
    /// and the discriminant separately was tried first and is not enough: when only the t²
    /// coefficient is noise, it swaps one pair of roots for a single bogus one (measured: 108
    /// of 116 inflections reported on random straight curves disagreed with exact arithmetic).
    ///
    /// Conventions: NaN coefficients give no roots, because a NaN value is never certain.
    ///
    /// Use when: keeping only the extrema or inflections that rounding cannot have invented
    /// Tags: certified roots, robust quadratic roots, sign change, root filtering
    /// Node: none (solver)
    std::vector<float> signChangeRoots(ParameterRange range) const {
        std::vector<float> roots;
        QuadraticRoots solved = QuadraticRoots::solve(quadratic, linear, constant);
        switch (solved.kind) {
        case QuadraticRoots::Kind::none: return {};
        case QuadraticRoots::Kind::one: roots = {solved.smaller}; break;
        case QuadraticRoots::Kind::two: roots = {solved.smaller, solved.larger}; break;
        }
        std::vector<bool> changesSign;
        for (size_t index = 0; index < roots.size(); index += 1) {
            float root = roots[index];
            float gapBefore = index > 0 ? (root - roots[index - 1]) / 2 : 1;
            float gapAfter = index < roots.size() - 1 ? (roots[index + 1] - root) / 2 : 1;
            float before = root - smallerOf(gapBefore, 1);
            float after = root + smallerOf(gapAfter, 1);
            changesSign.push_back(certainSign(before) * certainSign(after) < 0);
        }
        std::vector<float> kept;
        for (size_t index = 0; index < roots.size(); index += 1) {
            if (changesSign[index] && range.contains(roots[index])) kept.push_back(roots[index]);
        }
        return kept;
    }
};

/// The power-basis coefficients of a 2D cubic (see the top of the file), built from the
/// control points measured from the start, with an upper bound on the size of each.
///
/// The sizes are the sums of the sizes of the terms each coefficient is made of, by the
/// triangle inequality: `|a| ≤ A = |d3| + 3|d2| + 3|d1|`, `|b| ≤ B = 3|d2| + 6|d1|`,
/// `|c| ≤ C = 3|d1|`. Rounding errors are proportional to them, not to the (possibly much
/// smaller, cancelled) coefficients.
struct PowerBasis {
    /// a, the coefficient of t³.
    Vector2 cubic;
    /// b, the coefficient of t².
    Vector2 quadratic;
    /// c, the coefficient of t: the velocity at the start.
    Vector2 linear;
    /// d3 = P3 − P0, the chord.
    Vector2 startToEnd;
    /// A, B, C: bounds on |a|, |b|, |c| from the terms they are summed from.
    float cubicSize;
    float quadraticSize;
    float linearSize;

    /// The coefficients of the cubic whose control points, measured from its start, are
    /// `toStartHandle` (d1), `toEndHandle` (d2) and `toEnd` (d3).
    PowerBasis(Vector2 toStartHandle, Vector2 toEndHandle, Vector2 toEnd)
        : cubic(toEnd - toEndHandle * 3 + toStartHandle * 3),
          quadratic(toEndHandle * 3 - toStartHandle * 6),
          linear(toStartHandle * 3),
          startToEnd(toEnd),
          cubicSize(toEnd.length() + 3 * toEndHandle.length() + 3 * toStartHandle.length()),
          quadraticSize(3 * toEndHandle.length() + 6 * toStartHandle.length()),
          linearSize(3 * toStartHandle.length()) {}

    /// One coordinate of the velocity, `3a·t² + 2b·t + c` (x or y by `component`), with the
    /// rounding bound of each coefficient: `2ε·size` for a, b and c (see
    /// `QuadraticWithRoundingBounds.coefficientBound`), times 3 and 2. Multiplying by 2 is
    /// exact; multiplying by 3 adds one rounding of `ε/2`, so 3a gets `(6 + 1.5)ε·A ≤ 8ε·A`.
    ///
    /// Use when: finding where the curve turns back in x or in y (extrema, tight bounds)
    /// Tags: velocity component, derivative quadratic, hodograph, extrema equation
    /// Node: none (solver)
    QuadraticWithRoundingBounds derivative(float Vector2::*component) const {
        float epsilon = std::numeric_limits<float>::epsilon();
        return QuadraticWithRoundingBounds{
            cubic.*component * 3, 8 * epsilon * cubicSize,
            quadratic.*component * 2, 4 * epsilon * quadraticSize,
            linear.*component, 2 * epsilon * linearSize};
    }

    /// `−3(a×b)·t² + 3(c×a)·t + (c×b)`, which has the sign of the curvature (see
    /// `inflectionParameters`), with the rounding bound of each coefficient from
    /// `QuadraticWithRoundingBounds.crossProductBound`; times 3 adds one rounding, so the
    /// first two get `3·10ε + 1.5ε ≤ 32ε` times the sizes.
    ///
    /// Use when: finding where the curve changes from turning left to turning right
    /// Tags: inflection equation, curvature numerator, B′ × B″, sign of curvature
    /// Node: none (solver)
    QuadraticWithRoundingBounds curvatureSign() const {
        float epsilon = std::numeric_limits<float>::epsilon();
        return QuadraticWithRoundingBounds{
            -3 * cubic.cross(quadratic), 32 * epsilon * cubicSize * quadraticSize,
            3 * linear.cross(cubic), 32 * epsilon * linearSize * cubicSize,
            linear.cross(quadratic), QuadraticWithRoundingBounds::crossProductBound(linearSize, quadraticSize)};
    }
};

/// The power-basis coefficients a, b, c (see the top of the file) together with the size
/// of the terms each one was summed from, for the rounding bounds.
PowerBasis powerBasis(const CubicBezier<Vector2>& curve) {
    return PowerBasis(
        curve.startHandle - curve.start,
        curve.endHandle - curve.start,
        curve.end - curve.start);
}

} // namespace

// MARK: - Extrema and bounds

/// The parameters in 0…1 where the curve's x coordinate is at a local extreme, ascending.
/// These are the points that are furthest left or right locally.
///
/// How: the extremes of x(t) are where its derivative is zero. With the power-basis
/// coefficients a, b, c (x components), x′(t) = 3a·t² + 2b·t + c, a quadratic, solved in
/// closed form. A *double* root (the derivative touches zero but does not change sign)
/// is not an extreme and is left out.
///
///     x
///     │    ╭──╮        ← x′ changes sign here: extreme
///     │   ╱    ╲
///     └──╱──────╲──▶ t
///
/// Rounding: the coefficients are computed in Float, so a root counts only where x′ has
/// a certain sign on both sides of it (see `QuadraticWithRoundingBounds`). A curve whose
/// x is constant up to rounding has no extremes, and two extremes closer together than
/// rounding can tell apart are dropped as a double root; what they would stick out by
/// is itself only rounding.
///
/// Conventions: a root at 0 or 1 is reported when x′ changes sign there (the curve's
/// ends can be extremes); a root a hair outside 0…1 from rounding is not. A curve with x
/// constant has no roots.
///
/// Source: Pomax, *A Primer on Bézier Curves*, "Finding extremities".
///
/// Use when: finding where a profile reaches its widest point; tight bounds of a curve
/// Tags: Bezier extrema, local maximum, x extremes, derivative roots, curve extremities,
///   leftmost rightmost point
/// Node: none (2D outline bounds; boundingBox measures 3D geometry)
template <CurvePoint PointType>
std::vector<float> CubicBezier<PointType>::horizontalExtremaParameters() const
    requires std::same_as<PointType, Vector2>
{
    return powerBasis(*this).derivative(&Vector2::x).signChangeRoots(ParameterRange{0, 1});
}

/// The parameters in 0…1 where the curve's y coordinate is at a local extreme, ascending:
/// the local tops and bottoms. Same method and conventions as
/// `horizontalExtremaParameters`, applied to y.
///
/// Use when: finding the highest point of an arch, the top of a profile; splitting a
///   curve into pieces that only rise or only fall
/// Tags: Bezier extrema, local maximum, y extremes, derivative roots, highest point, top
///   of curve, monotone pieces
/// Node: none (2D outline bounds; boundingBox measures 3D geometry)
template <CurvePoint PointType>
std::vector<float> CubicBezier<PointType>::verticalExtremaParameters() const
    requires std::same_as<PointType, Vector2>
{
    return powerBasis(*this).derivative(&Vector2::y).signChangeRoots(ParameterRange{0, 1});
}

/// All parameters in 0…1 where x or y is at a local extreme, ascending, without
/// duplicates (a parameter that is an extreme of both is listed once).
///
/// How: the two lists `horizontalExtremaParameters` and `verticalExtremaParameters`,
/// merged. At most 4 values (2 per axis).
///
/// Use when: finding every point that can stick out of the curve's end points
/// Tags: Bezier extrema, extremal points, bounding box parameters, extremities
/// Node: none (2D outline bounds; boundingBox measures 3D geometry)
template <CurvePoint PointType>
std::vector<float> CubicBezier<PointType>::extremaParameters() const
    requires std::same_as<PointType, Vector2>
{
    std::vector<float> merged = horizontalExtremaParameters();
    for (float parameter : verticalExtremaParameters()) merged.push_back(parameter);
    std::sort(merged.begin(), merged.end());
    std::vector<float> unique;
    for (float parameter : merged) {
        if (unique.empty() || unique.back() != parameter) unique.push_back(parameter);
    }
    return unique;
}

/// The smallest axis-aligned rectangle that contains the whole curve, not just its
/// control points.
///
/// How: a coordinate's largest and smallest value on the curve occur at an end or where
/// its derivative is zero. So the rectangle is the one around `start`, `end` and the
/// curve's points at every extremum parameter. It is usually much tighter than the
/// rectangle around the four control points, which only bounds the curve loosely.
///
///        handles ●─────────● (loose box of control points)
///             ╭──────────╮
///      start ●╯  tight   ╰● end
///
/// Conventions: y-up plane. A straight or degenerate curve gives a rectangle with zero
/// width or height. The curve is inside up to rounding: a point evaluated on it may lie
/// outside by a few ulps of its coordinates (measured: at most 4.5e-8 over 20,000 random
/// curves of size about 1), so containment tests on samples need that slack.
///
/// Source: Pomax, *A Primer on Bézier Curves*, "Tight bounding boxes".
///
/// Use when: framing a curve in a view; testing whether a point can possibly be near it;
///   finding the size of a drawn profile
/// Tags: Bezier bounding box, tight bounds, AABB, curve extent, bounding rectangle
/// Node: none (2D outline bounds; boundingBox measures 3D geometry)
template <CurvePoint PointType>
Bounds2 CubicBezier<PointType>::bounds() const
    requires std::same_as<PointType, Vector2>
{
    std::vector<Vector2> enclosed = {start, end};
    for (float parameter : extremaParameters()) enclosed.push_back(point(parameter));
    return Bounds2::enclosing(enclosed);
}

// MARK: - Curvature

/// The parameters strictly between 0 and 1 where the curve changes from bending
/// counterclockwise to bending clockwise, or back: its inflection points. Ascending; at
/// most 2.
///
/// How: the signed curvature has the sign of the cross product `B′(t) × B″(t)`. Writing
/// the derivatives with the power-basis coefficients and multiplying out (the t³ terms
/// cancel because `a × a = 0`):
///
///     B′ × B″ = 2·[ −3(a × b)·t² + 3(c × a)·t + (c × b) ]
///
/// a quadratic in t. Its roots where the sign really changes are the inflections. A
/// double root (curvature touches zero and bounces back, as at a cusp) is not an
/// inflection; a straight line, where the whole expression is zero, has none.
///
///              ╱╲            bends left (+)
///      ───────╱──╲────── ●  ← inflection: bending switches side
///                   ╲╱       bends right (−)
///
/// Rounding: for a straight curve the three cross products are zero only in exact
/// arithmetic; in Float they come out as noise of either sign, and the noise has roots.
/// So a root counts only where the quadratic's sign is certain on both sides of it,
/// given the rounding bound of each coefficient (`QuadraticWithRoundingBounds`, which
/// derives the bounds from the cross products, about 10ε·|u|·|v| each). The bounds scale
/// with the curve, so a straight curve of any size has no inflections, a cusp has none
/// whether or not its points are exact in binary, and every inflection reported is a
/// true sign change of the exact curvature of the stored control points.
///
/// Source: Pomax, *A Primer on Bézier Curves*, "Inflections"; Farin, *Curves and
/// Surfaces for CAGD: A Practical Guide*, 5th ed. (2002), on the derivatives of a Bézier
/// curve.
///
/// Use when: splitting a profile into pieces that each bend one way; finding where an
///   S-shaped road changes side
/// Tags: inflection points, Bezier inflection, curvature sign change, S-curve, flex
///   point, bend switches
/// Node: fitProfile
template <CurvePoint PointType>
std::vector<float> CubicBezier<PointType>::inflectionParameters() const
    requires std::same_as<PointType, Vector2>
{
    std::vector<float> inflections;
    for (float parameter : powerBasis(*this).curvatureSign().signChangeRoots(ParameterRange{0, 1})) {
        if (parameter > 0 && parameter < 1) inflections.push_back(parameter);
    }
    return inflections;
}

/// The signed curvature at `parameter`: how fast the direction of travel turns per unit
/// of distance travelled. It is `1/radius` of the circle that hugs the curve there,
/// positive when the curve turns counterclockwise (to its left) and negative clockwise.
///
/// How: `κ = (B′ × B″) / |B′|³`, with `×` the 2D cross product (the signed area z).
/// Derivation: the direction angle is θ = atan2(y′, x′); dθ/dt = (x′·y″ − y′·x″)/|B′|²
/// and the speed is ds/dt = |B′|, so κ = dθ/ds = (B′ × B″)/|B′|³.
///
///     counterclockwise circle of radius r:  κ = +1/r      straight line: κ = 0
///
/// Written that way, `|B′|³` overflows Float for a curve 1e13 long and loses digits
/// to underflow for one 1e-15 long. So the velocity is first divided by its largest
/// component m, giving v′ = B′/m with |v′| between 1 and √2, and
///
///     κ = (v′ × B″) / |v′|³ / m / m          (since B′ × B″ = m·(v′ × B″), |B′|³ = m³|v′|³)
///
/// whose every step stays in range whenever the answer does.
///
/// Conventions: y-up plane. Where the velocity is exactly zero (a cusp, or a handle
/// sitting on its end point) the direction is undefined and the function returns 0.
/// Very near such a point the true curvature is huge, so values there are unreliable and
/// may overflow to ±infinity. A control point that is not finite (so a velocity or
/// acceleration that is not finite) gives NaN.
///
/// Source: do Carmo, *Differential Geometry of Curves and Surfaces* (1976), curvature of
/// a regular plane curve.
///
/// Use when: shading how tightly a profile bends; limiting the bend of a road; choosing
///   how finely to flatten a curve
/// Tags: curvature, signed curvature, radius of curvature, Bezier curvature, bend
///   amount, kappa
/// Node: fitProfile
template <CurvePoint PointType>
float CubicBezier<PointType>::signedCurvature(float parameter) const
    requires std::same_as<PointType, Vector2>
{
    Vector2 velocity = firstDerivative(parameter);
    Vector2 acceleration = secondDerivative(parameter);
    if (!(std::isfinite(velocity.x) && std::isfinite(velocity.y) && std::isfinite(acceleration.x)
          && std::isfinite(acceleration.y))) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    float largestVelocityComponent = largerOf(std::fabs(velocity.x), std::fabs(velocity.y));
    if (!(largestVelocityComponent > 0)) return 0;
    Vector2 scaledVelocity = velocity / largestVelocityComponent;
    float scaledSpeed = scaledVelocity.length();
    float turning = scaledVelocity.cross(acceleration) / (scaledSpeed * scaledSpeed * scaledSpeed);
    return turning / largestVelocityComponent / largestVelocityComponent;
}

/// Whether all four control points have finite coordinates (no NaN, no infinity).
///
/// Use when: refusing to measure or flatten a curve that came from a failed computation
/// Tags: is finite, NaN check, valid curve, finite control points
/// Node: none (validation)
template <CurvePoint PointType>
bool CubicBezier<PointType>::hasFiniteControlPoints() const
    requires std::same_as<PointType, Vector2>
{
    for (Vector2 controlPoint : {start, startHandle, endHandle, end}) {
        if (!(std::isfinite(controlPoint.x) && std::isfinite(controlPoint.y))) return false;
    }
    return true;
}

// MARK: - Area

/// The signed area of the fan between the curve and the two straight lines from `pivot`
/// to its start and to its end. Positive when the curve runs counterclockwise around
/// `pivot`.
///
///            end ●
///               ╱ ╲___
///      pivot ●─╱──────╲╲      the shaded region: pivot, start, along the curve, end
///               ╲_____ ● start
///
/// How: Green's theorem. The area enclosed by a loop is ½∮(x dy − y dx), and for the
/// curve piece measured from `pivot` this is ½∫ (B − pivot) × B′ dt over 0…1. Writing
/// B = P0 + B̃ with B̃ = a·t³ + b·t² + c·t the curve relative to its own start,
///
///     B̃ × B̃′ = −(a×b)·t⁴ − 2(a×c)·t³ − (b×c)·t²        (a×a = 0 removes the t⁵ term)
///     fan area = ½·(P0 − pivot) × (P3 − P0) − ½·[ (a×b)/5 + (a×c)/2 + (b×c)/3 ]
///
/// The first term is the triangle pivot, start, end; the second is the area between the
/// curve and its chord. Both are exact for a cubic: a polynomial integrates exactly.
/// Summing the fans of all pieces of a closed outline gives the area it encloses
/// (pieces that go clockwise subtract).
///
/// Precision: a, b, c and P3 − P0 are built from the differences Pᵢ − P0, so the area
/// between chord and curve keeps its digits wherever the curve is (measured: error
/// 1.9e-7 for a curve of size 2 at x = 1e5; building a, b, c from the absolute points
/// gave 3.5e-2).
///
/// Conventions: y-up plane. `pivot` defaults to the origin; choosing a pivot near the
/// shape (such as its first point) keeps the triangle term accurate when the shape is
/// far from the origin.
///
/// Source: Green's theorem, applied to the power-basis polynomial.
///
/// Use when: computing the area of a profile or a filled path made of curves; the signed
///   area under a curve
/// Tags: Bezier area, signed area, Green's theorem, shoelace for curves, area under
///   curve, polygon area, fan area, swept area
/// Node: inside profileArea
template <CurvePoint PointType>
float CubicBezier<PointType>::signedAreaSwept(Vector2 pivot) const
    requires std::same_as<PointType, Vector2>
{
    PowerBasis basis = powerBasis(*this);
    float triangle = (start - pivot).cross(basis.startToEnd) / 2;
    float betweenChordAndCurve = basis.cubic.cross(basis.quadratic) / 5
        + basis.cubic.cross(basis.linear) / 2
        + basis.quadratic.cross(basis.linear) / 3;
    return triangle - betweenChordAndCurve / 2;
}

template class CubicBezier<Vector2>;

} // namespace editor
