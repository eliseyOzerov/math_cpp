// A cubic Bézier curve is the smoothest simple curve a designer can shape with four points:
// it starts at the first, ends at the last, and is pulled toward the two in the middle
// (Engine/Math/Curves/CubicBezier.swift and its extensions +Analysis, +Tangents, +Flattening
// and +Arc).
//
// Swift's per-type extensions (`extension CubicBezier where Point == Vector2`) are members
// here that need `Point` to be `Vector2` (`requires std::same_as<Point, Vector2>`). Their
// definitions are in CubicBezier+Analysis.cpp, +Tangents.cpp, +Flattening.cpp and +Arc.cpp, which
// instantiate `CubicBezier<Vector2>`; the members declared here are the whole list, so a later
// extension (offset, fitting, crossings) adds its declaration in the section for its file.
#pragma once

#include <concepts>
#include <optional>
#include <vector>

#include "../Linear/Angle.h"
#include "../Geometry/Bounds2.h"
#include "../Linear/Vector2.h"
#include "../Linear/Vector3.h"
#include "Curve.h"

namespace editor {

template <CurvePoint Point>
class CubicBezier;

/// The two halves a Bézier curve is cut into by `CubicBezier.split(at:)`.
///
/// Use when: using the two parts that come from cutting a Bézier curve, such as when
///   subdividing a road
/// Tags: Bézier split, subdivision, de Casteljau split, curve halves, left and right curve
/// Node: inside roundCorners, fitProfile, offsetProfile, profileArc, profileCrossings
template <CurvePoint Point>
struct CubicBezierSplit {
    /// The part from the original start up to the cut.
    CubicBezier<Point> first;
    /// The part from the cut to the original end.
    CubicBezier<Point> second;
};

/// A point where two curves (or a curve and a line) meet, with the parameter on each.
///
/// Use when: cutting two outlines where they meet; marking where a road crosses a river
/// Tags: intersection, crossing point, curve intersection result, hit
/// Node: profileCrossings
struct CurveCrossing {
    /// The parameter on the first curve, 0…1.
    float firstParameter = 0;
    /// The parameter on the second curve (0…1), or on the line (any number for an unbounded
    /// line, see `CubicBezier::crossings(lineStart, lineEnd, withinSegment)`).
    float secondParameter = 0;
    /// Where they meet, on the first curve.
    Vector2 point;

    bool operator==(const CurveCrossing&) const = default;
};

/// The two parameters at which a cubic passes through the same point, the knot of its loop
/// (Swift's tuple `(first: Float, second: Float)`).
///
/// Use when: cutting a loop off a curve; reporting where a single curve crosses itself
/// Tags: self intersection, loop parameters, double point, knot
/// Node: inside profileCrossings
struct SelfCrossingParameters {
    float first = 0;
    float second = 0;

    bool operator==(const SelfCrossingParameters&) const = default;
};

/// A **cubic Bézier curve**, given by four control points. `Point` is `Vector2` for a profile
/// and `Vector3` for a path in space; the formulas are the same.
///
///                 startHandle ●─ ─ ─ ─ ─ ● endHandle
///                            ╱             ╲
///                           ╱   ╭─────────╮ ╲
///                    start ●───╯           ╰──● end
///
/// The curve starts at `start` heading toward `startHandle`, and ends at `end` coming from
/// `endHandle`. The handles are usually *not* on the curve. The curve always stays inside the
/// convex hull of the four points.
///
/// Formula (Bernstein form), with `u = 1 − parameter`:
///
///     B(parameter) = u³·start + 3u²·parameter·startHandle + 3u·parameter²·endHandle + parameter³·end
///
/// The code evaluates it with **de Casteljau's algorithm** instead, which gives the same
/// number but is built only from straight-line blends and is numerically very stable.
///
/// Conventions: the curve lives on the parameter range 0…1. Outside that range the same
/// polynomial simply continues (it is not clamped), which is handy for extending a curve
/// but usually not a shape you want.
///
/// Source: Farin, *Curves and Surfaces for Computer-Aided Geometric Design*; de Casteljau
/// (1959).
///
/// Use when: drawing a smooth curve with handles, like a pen tool or a road bend
/// Tags: Bezier, cubic Bezier, curve, spline, control points, handles, path
/// Node: curveBezier
template <CurvePoint PointType>
class CubicBezier : public Curve<CubicBezier<PointType>, PointType> {
public:
    using Point = PointType;

    Point start{};
    Point startHandle{};
    Point endHandle{};
    Point end{};

    CubicBezier() = default;
    CubicBezier(Point start, Point startHandle, Point endHandle, Point end)
        : start(start), startHandle(startHandle), endHandle(endHandle), end(end) {}

    /// Two cubic Béziers are equal when their four control points are.
    bool operator==(const CubicBezier&) const = default;

    /// The parameter values of the curve: 0 at the first control point to 1 at the last.
    ///
    /// Use when: sampling a Bézier curve from start to end; checking that a parameter is
    ///   valid
    /// Tags: Bezier parameter range, curve domain, 0 to 1
    /// Node: none (protocol plumbing)
    ParameterRange parameterRange() const { return ParameterRange{0, 1}; }

    /// The point on the curve at `parameter`, by de Casteljau's algorithm.
    ///
    /// How: blend each neighbouring pair of control points by `parameter`, giving three
    /// points; blend those pairwise, giving two; blend those, giving one: the answer.
    ///
    ///     start ──a── startHandle ──b── endHandle ──c── end      (a, b, c: first blends)
    ///           d = blend(a, b)    e = blend(b, c)                (second blends)
    ///                     f = blend(d, e)  ← the point on the curve
    ///
    /// `parameter` 0 gives `start` and 1 gives `end` exactly. Outside 0…1 the blends
    /// extrapolate, continuing the polynomial.
    ///
    /// Use when: sampling a Bézier curve to draw it or to place objects along it
    /// Tags: Bezier point, evaluate curve, de Casteljau, curve position, sample curve,
    ///   lerp curve
    /// Node: curveBezier
    Point point(float parameter) const {
        DeCasteljauLevels levels = deCasteljauLevels(parameter);
        return levels.point;
    }

    /// The velocity at `parameter`: the derivative of the curve with respect to the parameter.
    ///
    /// How: differentiating the Bernstein form gives another Bézier curve, one degree
    /// lower, whose control points are the three "steps" between neighbouring control
    /// points, times 3:
    ///
    ///     B′(parameter) = 3 · quadraticBlend(step₀, step₁, step₂, parameter)
    ///     step₀ = startHandle − start, step₁ = endHandle − startHandle, step₂ = end − endHandle
    ///
    /// At 0 this is `3·(startHandle − start)` and at 1 it is `3·(end − endHandle)`, so the
    /// curve leaves `start` toward `startHandle`. It is the zero vector wherever the curve
    /// stops momentarily, for example when `startHandle == start`.
    ///
    /// Source: Farin, derivative of a Bézier curve.
    ///
    /// Use when: getting the direction of a Bézier curve for orienting a vehicle or
    ///   building a road cross-section
    /// Tags: Bezier tangent, derivative, curve direction, velocity, tangent vector
    /// Node: inside fitProfile, offsetProfile
    Point firstDerivative(float parameter) const {
        Point firstStep = startHandle - start;
        Point secondStep = endHandle - startHandle;
        Point thirdStep = end - endHandle;
        Point firstBlend = Point::linearBlend(firstStep, secondStep, parameter);
        Point secondBlend = Point::linearBlend(secondStep, thirdStep, parameter);
        return Point::linearBlend(firstBlend, secondBlend, parameter) * 3;
    }

    /// The acceleration at `parameter`: the derivative of the velocity.
    ///
    /// How: differentiating once more gives a straight line between two "bends":
    ///
    ///     B″(parameter) = 6 · linearBlend(bend₀, bend₁, parameter)
    ///     bend₀ = endHandle − 2·startHandle + start
    ///     bend₁ = end − 2·endHandle + startHandle
    ///
    /// A curve with evenly spaced collinear control points has zero acceleration
    /// everywhere: it moves at constant speed in a straight line.
    ///
    /// Source: Farin, derivative of a Bézier curve.
    ///
    /// Use when: measuring how sharply a Bézier curve bends; computing curvature for road
    ///   banking
    /// Tags: Bezier second derivative, acceleration, curvature, bend, curve curvature
    /// Node: inside fitProfile, offsetProfile
    Point secondDerivative(float parameter) const {
        Point firstBend = endHandle - startHandle * 2 + start;
        Point secondBend = end - endHandle * 2 + startHandle;
        return Point::linearBlend(firstBend, secondBend, parameter) * 6;
    }

    /// Cuts the curve in two at `parameter`. Together the halves trace exactly the same
    /// shape; each is a full Bézier curve on its own 0…1 range.
    ///
    /// How: de Casteljau's blends are the control points of both halves. The first blends
    /// down the left edge of the triangle (`start`, `a`, `d`, `f`) belong to the first
    /// half, and the ones down the right edge (`f`, `e`, `c`, `end`) to the second:
    ///
    ///     first  = (start, a, d, f)        second = (f, e, c, end)
    ///
    /// The cut point `f` is shared, so the halves join without a gap. The first half
    /// covers the original parameters 0…`parameter`, the second `parameter`…1.
    ///
    /// Source: Farin, subdivision of Bézier curves.
    ///
    /// Use when: adding a point in the middle of a road curve without changing its shape;
    ///   drawing a curve in pieces
    /// Tags: split Bezier, subdivide curve, de Casteljau, cut curve, insert point, divide
    ///   curve
    /// Node: inside roundCorners, profileCrossings
    CubicBezierSplit<Point> split(float parameter) const {
        DeCasteljauLevels levels = deCasteljauLevels(parameter);
        return CubicBezierSplit<Point>{
            CubicBezier<Point>(start, levels.firstBlendStart, levels.secondBlendStart, levels.point),
            CubicBezier<Point>(levels.point, levels.secondBlendEnd, levels.firstBlendEnd, end)};
    }

    // MARK: Analysis (CubicBezier+Analysis.cpp, flat curves only)

    std::vector<float> horizontalExtremaParameters() const requires std::same_as<Point, Vector2>;
    std::vector<float> verticalExtremaParameters() const requires std::same_as<Point, Vector2>;
    std::vector<float> extremaParameters() const requires std::same_as<Point, Vector2>;
    Bounds2 bounds() const requires std::same_as<Point, Vector2>;
    std::vector<float> inflectionParameters() const requires std::same_as<Point, Vector2>;
    float signedCurvature(float parameter) const requires std::same_as<Point, Vector2>;
    bool hasFiniteControlPoints() const requires std::same_as<Point, Vector2>;
    float signedAreaSwept(Vector2 pivot = Vector2()) const requires std::same_as<Point, Vector2>;

    // MARK: Tangents (CubicBezier+Tangents.cpp, flat curves only)

    Vector2 startDirection() const requires std::same_as<Point, Vector2>;
    Vector2 endDirection() const requires std::same_as<Point, Vector2>;
    CubicBezier<Vector2> translated(Vector2 offset) const requires std::same_as<Point, Vector2>;
    CubicBezier<Vector2> portion(float lower, float upper) const requires std::same_as<Point, Vector2>;

    // MARK: Flattening (CubicBezier+Flattening.cpp, flat curves only)

    /// The most pieces `flattened(tolerance:)` will ever use for one curve, a guard against
    /// an absurdly small tolerance asking for millions of points.
    ///
    /// Use when: knowing the most points a flattened curve can have; budgeting memory
    /// Tags: flattening limit, maximum segments, tessellation cap, polyline size limit
    /// Node: none (validation)
    static constexpr int mostFlatteningPieces = 10'000;

    int flatteningPieceCount(float tolerance) const requires std::same_as<Point, Vector2>;
    std::vector<Vector2> flattened(float tolerance) const requires std::same_as<Point, Vector2>;

    // MARK: Arc (CubicBezier+Arc.cpp, flat curves only)

    /// The longest arc one cubic is allowed to cover: a quarter turn. Above that the radial
    /// error grows with the sixth power of the angle.
    ///
    /// Use when: knowing how many cubics an arc needs; checking the limit that
    ///   `circularArc` cuts a sweep by
    /// Tags: arc piece limit, quarter turn, 90 degrees, maximum arc per curve
    /// Node: none (validation)
    static Angle longestArcPerCubic() { return Angle::degrees(90); }

    /// How much more than a quarter turn a piece of `circularArc` may cover, as a fraction
    /// of a quarter turn: 1e-5. A sweep of exactly 90° or 360° written in Float radians can
    /// come out a rounding step over a whole number of quarter turns, and without this slack
    /// it would be split into one piece more than it needs.
    ///
    /// Use when: counting the pieces of an arc whose sweep is a whole number of quarter turns
    /// Tags: rounding slack, piece count, arc split, epsilon
    /// Node: none (validation)
    static constexpr float arcPieceCountSlack = 1e-5f;

    /// The most cubics `circularArc` will produce for one sweep: 10,000, about 2,500 full
    /// turns. A larger sweep gives no pieces rather than millions of them.
    ///
    /// Use when: knowing the most pieces an arc can have; budgeting memory for a wound spiral
    /// Tags: arc piece limit, maximum arc pieces, circle cap, too many turns
    /// Node: none (validation)
    static constexpr int mostArcPieces = 10'000;

    static float circularArcHandleFactor(Angle sweep) requires std::same_as<Point, Vector2>;
    static std::vector<CubicBezier<Vector2>> circularArc(
        Vector2 centre, float radius, Angle startAngle, Angle sweep) requires std::same_as<Point, Vector2>;

    // MARK: Crossings (CubicBezier+Crossings.cpp, flat curves only)

    std::vector<CurveCrossing> crossings(Vector2 lineStart, Vector2 lineEnd, bool withinSegment = false) const
        requires std::same_as<Point, Vector2>;
    static float heightRoundingBound(Vector2 offset) requires std::same_as<Point, Vector2>;
    static float lineParameterSlack(Vector2 offset, float lineLength) requires std::same_as<Point, Vector2>;
    static std::vector<float> rootsOfBernsteinCubic(const std::vector<float>& heights)
        requires std::same_as<Point, Vector2>;
    Vector2 storedEndOrPoint(float parameter) const requires std::same_as<Point, Vector2>;

    /// Below this many halvings a piece's box is about `2⁻¹⁴` of the curves' size, small enough
    /// for Newton's method to take over (see `crossings(other)`).
    ///
    /// Use when: deciding when halving two cubics stops and Newton's method takes over
    /// Tags: subdivision depth, recursion limit, bezier clipping, intersection
    /// Node: none (solver)
    static constexpr int crossingSubdivisionDepth = 14;

    /// The most pairs of tiny pieces kept as candidates. Two curves that overlap along a
    /// stretch produce a candidate for every piece of it; past this many the search gives up
    /// and `crossings(other)` returns nothing (Swift's nil).
    ///
    /// Use when: bounding the work when two curves overlap along a stretch
    /// Tags: candidate limit, overlapping curves, intersection budget
    /// Node: none (solver)
    static constexpr int mostCrossingCandidates = 4096;

    /// The Newton steps used to polish each candidate crossing.
    ///
    /// Use when: refining a rough crossing of two cubics to full precision
    /// Tags: Newton iterations, polish, refine intersection
    /// Node: none (solver)
    static constexpr int crossingPolishSteps = 12;

    /// Two polished crossings closer than this in both parameters (and at the same point) are
    /// one crossing found from two neighbouring candidates.
    ///
    /// Use when: merging the same crossing found from two neighbouring candidates
    /// Tags: duplicate crossing, merge distance, deduplicate, epsilon
    /// Node: none (solver)
    static constexpr float crossingMergeParameterDistance = 1.0f / 256;

    /// The most points at which two different cubics can cross: 9. By Bézout's theorem two
    /// plane curves of degrees m and n with no common piece meet at most m·n times, and a
    /// cubic Bézier is a curve of degree 3. More than 9 separate crossings means the curves
    /// overlap along a stretch.
    ///
    /// Use when: telling two curves that overlap from two that cross many times
    /// Tags: Bezout, intersection count, maximum crossings, overlapping curves, coincident
    /// Node: none (solver)
    static constexpr int mostCrossingsOfTwoCubics = 9;

    static float crossingMergeDistance(float extent) requires std::same_as<Point, Vector2>;
    static float crossingAcceptance(float extent) requires std::same_as<Point, Vector2>;
    std::optional<std::vector<CurveCrossing>> crossings(const CubicBezier<Vector2>& other) const
        requires std::same_as<Point, Vector2>;
    std::optional<SelfCrossingParameters> selfCrossingParameters() const requires std::same_as<Point, Vector2>;

    // MARK: Fitting (CubicBezier+Fitting.cpp, flat curves only)

    /// The most points `fitted` accepts: 1,000,000. A longer list gives no pieces, so a runaway
    /// sampler cannot make the fit run for minutes.
    ///
    /// Use when: knowing the largest stroke the fitter takes; splitting a very long stroke
    /// Tags: fitting limit, maximum points, input cap, stroke size
    /// Node: none (validation)
    static constexpr int mostFittedPoints = 1'000'000;

    /// The deepest the fit splits a stretch of points in two before it gives up on smooth
    /// pieces and joins the remaining points by straight pieces: 48 halvings.
    ///
    /// Use when: knowing why a pathological stroke comes out as straight pieces
    /// Tags: fitting depth, recursion limit, split limit, straight fallback
    /// Node: none (validation)
    static constexpr int deepestFittingSplit = 48;

    /// Swift's `fitted(to:tolerance:startDirection:endDirection:closed:)`; `closed` is Swift's
    /// `Bool?`.
    static std::vector<CubicBezier<Vector2>> fitted(
        const std::vector<Vector2>& points, float tolerance, std::optional<Vector2> startDirection = std::nullopt,
        std::optional<Vector2> endDirection = std::nullopt, std::optional<bool> closed = std::nullopt)
        requires std::same_as<Point, Vector2>;
    static float smallestFittingTolerance(const std::vector<Vector2>& points) requires std::same_as<Point, Vector2>;
    static bool fitsAsClosedLoop(const std::vector<Vector2>& distinctPoints, std::optional<bool> closed)
        requires std::same_as<Point, Vector2>;
    static std::vector<Vector2> withoutDuplicates(const std::vector<Vector2>& points)
        requires std::same_as<Point, Vector2>;

    // MARK: Offset (CubicBezier+Offset.cpp, flat curves only)

    /// How many times a piece may be halved while looking for simple pieces, and again while
    /// looking for a fit within the tolerance: 12 each. A curve is first cut at its inflections
    /// (at most 2, so at most 3 portions); each portion becomes at most 2¹² = 4096 simple
    /// pieces, and each of those at most 4096 fitted pieces. In practice a smooth curve needs a
    /// handful (see the tests); only a cusp runs into the limit, and then the offset is refused.
    ///
    /// Use when: bounding how finely a curve may be cut while offsetting it
    /// Tags: halving limit, recursion limit, offset subdivision, piece cap
    /// Node: none (validation)
    static constexpr int mostOffsetHalvings = 12;

    /// How many equal steps each fitted piece is divided into when its error is measured: the
    /// error is checked at the `offsetErrorSteps − 1` points between the ends (the ends are on
    /// the true offset by construction).
    ///
    /// Use when: checking an offset piece against the true offset distance
    /// Tags: error samples, offset check, tolerance test
    /// Node: none (validation)
    static constexpr int offsetErrorSteps = 16;

    /// The share of the tolerance a fitted piece may use at the points where its error is
    /// measured: ½. The error is only measured at `offsetErrorSteps − 1` points, and between
    /// them it can be a little larger; holding the measured points to half the tolerance keeps
    /// the error everywhere within the whole tolerance (the tests measure it densely).
    ///
    /// Use when: deciding whether a fitted offset piece is close enough
    /// Tags: tolerance margin, safety factor, error budget
    /// Node: none (validation)
    static constexpr float measuredErrorShare = 0.5f;

    static Angle largestSimpleTurn() requires std::same_as<Point, Vector2>;
    bool isSimpleForOffset() const requires std::same_as<Point, Vector2>;
    std::vector<CubicBezier<Vector2>> simplePiecesForOffset() const requires std::same_as<Point, Vector2>;
    std::vector<ParameterRange> simplePieceParameters() const requires std::same_as<Point, Vector2>;
    std::optional<std::vector<CubicBezier<Vector2>>> offset(float distance, float tolerance) const
        requires std::same_as<Point, Vector2>;
    std::optional<std::vector<CubicBezier<Vector2>>> offset(float startDistance, float endDistance, float tolerance) const
        requires std::same_as<Point, Vector2>;
    std::optional<CubicBezier<Vector2>> offsetFit(ParameterRange range, float startDistance, float endDistance) const
        requires std::same_as<Point, Vector2>;
    std::optional<Vector2> offsetPoint(float parameter, float startDistance, float endDistance) const
        requires std::same_as<Point, Vector2>;
    std::optional<Vector2> offsetDerivative(float parameter, float startDistance, float endDistance) const
        requires std::same_as<Point, Vector2>;
    float largestOffsetError(
        const CubicBezier<Vector2>& offsetPiece, ParameterRange range, float startDistance, float endDistance) const
        requires std::same_as<Point, Vector2>;

private:
    /// The triangle of blends de Casteljau's algorithm builds. `point` and `split` read
    /// different edges of it.
    struct DeCasteljauLevels {
        /// First level, between `start` and `startHandle`.
        Point firstBlendStart;
        /// First level, between `endHandle` and `end`.
        Point firstBlendEnd;
        /// Second level, the left one.
        Point secondBlendStart;
        /// Second level, the right one.
        Point secondBlendEnd;
        /// The final blend: the point on the curve.
        Point point;
    };

    // Offset helpers (CubicBezier+Offset.cpp)
    std::vector<ParameterRange> halvedUntilSimple(ParameterRange range, int halvings) const
        requires std::same_as<Point, Vector2>;
    std::optional<std::vector<CubicBezier<Vector2>>> offsetWithinTolerance(
        ParameterRange range, float startDistance, float endDistance, float tolerance, int halvings) const
        requires std::same_as<Point, Vector2>;
    std::optional<Vector2> offsetTangent(float parameter) const requires std::same_as<Point, Vector2>;
    Vector2 scaledThirdDerivative(float extent) const requires std::same_as<Point, Vector2>;

    DeCasteljauLevels deCasteljauLevels(float parameter) const {
        Point firstBlendStart = Point::linearBlend(start, startHandle, parameter);
        Point firstBlendMiddle = Point::linearBlend(startHandle, endHandle, parameter);
        Point firstBlendEnd = Point::linearBlend(endHandle, end, parameter);
        Point secondBlendStart = Point::linearBlend(firstBlendStart, firstBlendMiddle, parameter);
        Point secondBlendEnd = Point::linearBlend(firstBlendMiddle, firstBlendEnd, parameter);
        return DeCasteljauLevels{
            firstBlendStart, firstBlendEnd, secondBlendStart, secondBlendEnd,
            Point::linearBlend(secondBlendStart, secondBlendEnd, parameter)};
    }
};

} // namespace editor
