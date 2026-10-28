// Fitting a curve to points is the pen tool run backwards. A stroke traced with a finger or a
// mouse arrives as hundreds of samples; a designer wants a handful of Bézier pieces with handles.
// The fit chooses the pieces so that no sample is further from the result than a tolerance
// (Engine/Math/Curves/CubicBezier+Fitting.swift).
//
// This file returns the pieces themselves (`std::vector<CubicBezier<Vector2>>`), the primitive
// that every other consumer can use; `Path2D::fitted` (in Path2D+Fitting.cpp) chains them into a
// path.
//
// The arithmetic is Swift's, operation for operation: the fit works in Double on a small vector
// type of its own (Swift's `SIMD2<Double>`), measured from the first point, and rounds each handle
// to Float once.
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "ArcLength.h"
#include "CubicBezier.h"

namespace editor {

namespace {

// Swift's `Swift.min` and `Swift.max` for Double (the two-argument forms: the second argument wins
// a tie or a comparison with NaN the way Swift's definitions do).
double smallerOf(double first, double second) { return second < first ? second : first; }
double largerOf(double first, double second) { return second >= first ? second : first; }

/// Swift's `SIMD2<Double>`: two doubles with the component-wise arithmetic the fitter uses.
struct Vector2D {
    double x = 0;
    double y = 0;

    Vector2D operator+(Vector2D other) const { return Vector2D{x + other.x, y + other.y}; }
    Vector2D operator-(Vector2D other) const { return Vector2D{x - other.x, y - other.y}; }
    Vector2D operator*(double scale) const { return Vector2D{x * scale, y * scale}; }
    Vector2D operator/(double divisor) const { return Vector2D{x / divisor, y / divisor}; }
    Vector2D operator-() const { return Vector2D{-x, -y}; }
    Vector2D& operator+=(Vector2D other) { return *this = *this + other; }
};

double dot(Vector2D first, Vector2D second) { return first.x * second.x + first.y * second.y; }

double length(Vector2D vector) { return std::sqrt(dot(vector, vector)); }

/// The working part of `CubicBezier::fitted`: it fits stretches of points, in
/// Double and relative to the first point, and records the pieces as indices and Float handles
/// that have been checked after rounding.
class CubicFitter {
public:
    /// One fitted piece: it runs from the input point `firstIndex` to the input point
    /// `lastIndex`, with handles already rounded to Float.
    struct Piece {
        int firstIndex;
        int lastIndex;
        Vector2 startHandle;
        Vector2 endHandle;
    };

    /// The input points, as given.
    std::vector<Vector2> inputPoints;
    /// The first input point in Double: the origin of `points`.
    Vector2D origin;
    /// The input points in Double, measured from the first.
    std::vector<Vector2D> points;
    double tolerance;
    int deepestSplit;
    /// The distance along the polyline from the first point to each point.
    std::vector<double> distanceAlong;
    /// How far from a point the tangent estimate looks for its two partner points: 4·tolerance,
    /// at most an eighth of the polyline. See `direction(at:)`.
    double reach;
    std::vector<Piece> pieces;

    CubicFitter(const std::vector<Vector2>& inputPoints, double tolerance, int deepestSplit)
        : inputPoints(inputPoints), tolerance(tolerance), deepestSplit(deepestSplit) {
        origin = Vector2D{static_cast<double>(inputPoints[0].x), static_cast<double>(inputPoints[0].y)};
        // A difference of two Floats is exact in Double unless their exponents are far apart.
        for (const Vector2& point : inputPoints) {
            points.push_back(Vector2D{static_cast<double>(point.x), static_cast<double>(point.y)} - origin);
        }
        std::vector<double> distances{0.0};
        for (size_t index = 1; index < points.size(); index += 1) {
            distances.push_back(distances[index - 1] + length(points[index] - points[index - 1]));
        }
        distanceAlong = distances;
        reach = smallerOf(4 * tolerance, distances[distances.size() - 1] / 8);
    }

    // MARK: Entry points

    /// Fits all the points as an open stroke.
    void fitOpen(std::optional<Vector2D> startDirection, std::optional<Vector2D> endDirection) {
        int last = static_cast<int>(points.size()) - 1;
        Vector2D start = startDirection ? *startDirection : direction(0);
        // The end tangent points back into the curve, so a direction of travel is negated.
        Vector2D end = endDirection ? -*endDirection : -direction(last);
        fit(0, last, start, end, 0);
    }

    /// Fits a closed loop (last point == first): cut at the point furthest from the start and
    /// fit both halves; the start and end share one smooth tangent unless directions are given.
    void fitClosed(std::optional<Vector2D> startDirection, std::optional<Vector2D> endDirection) {
        int last = static_cast<int>(points.size()) - 1;
        // Around the start of the loop: the partners are the points `reach` before it (counted from
        // the end of the list) and `reach` after it.
        double total = distanceAlong[last];
        int before = partner(last, -1);
        int after = partner(0, 1);
        std::optional<Vector2D> smooth = tangent(
            {TangentEntry{points[before], distanceAlong[before] - total}, TangentEntry{points[0], 0},
             TangentEntry{points[after], distanceAlong[after]}},
            1);
        if (!smooth) smooth = unitDirection(points[after] - points[before]);
        Vector2D around = smooth ? *smooth : Vector2D{1, 0};
        Vector2D start = startDirection ? *startDirection : around;
        Vector2D end = endDirection ? -*endDirection : -around;
        int cut = 1;
        double furthest = -1.0;
        for (int index = 1; index < last; index += 1) {
            double distance = length(points[index] - points[0]);
            if (distance > furthest) {
                furthest = distance;
                cut = index;
            }
        }
        Vector2D forward = direction(cut);
        fit(0, cut, start, -forward, 1);
        fit(cut, last, forward, end, 1);
    }

    /// A float point in the fitter's coordinates (exact for points near the stroke), kept public
    /// for the entry function: the unit direction of a Float vector.
    static std::optional<Vector2D> unitDirection(Vector2 vector) {
        return unitDirection(Vector2D{static_cast<double>(vector.x), static_cast<double>(vector.y)});
    }

private:
    /// The best cubic found for a stretch of points, with how far the points are from it.
    struct Candidate {
        Vector2D startOffset;
        Vector2D endOffset;
        double error;
        /// Index into `points` of the point furthest from the curve.
        int worstIndex;
        /// The parameter of each point of the stretch on this cubic, where `error` was measured.
        std::vector<double> parameters;
    };

    /// A point of the stretch with its node (the distance along the polyline), for `tangent`.
    struct TangentEntry {
        Vector2D point;
        double node;
    };

    /// The rounds of solve-and-reparameterize every stretch gets (unless it fits sooner).
    static constexpr int leastRounds = 8;
    /// The most rounds for one stretch; past `leastRounds` only while they still help.
    static constexpr int mostRounds = 128;
    /// A round past `leastRounds` must bring the error below this share of the best so far.
    static constexpr double worthwhileImprovement = 0.9999;
    /// A handle shorter than this share of the chord is replaced by chord/3.
    static constexpr double shortestHandleShare = 1e-3;

    // MARK: Fitting a stretch

    /// Fits the stretch `first...last`, splitting as needed, and appends the pieces.
    /// `endDirection` points from the stretch's last point back into the curve.
    void fit(int first, int last, Vector2D startDirection, Vector2D endDirection, int depth) {
        int pointCount = last - first + 1;
        if (pointCount == 2) {
            // No point lies between the ends, so any finite handles keep the tolerance.
            double handle = length(points[last] - points[first]) / 3;
            Piece piece = roundedPiece(first, last, startDirection * handle, endDirection * handle);
            pieces.push_back(piece.startHandle.isFinite() && piece.endHandle.isFinite() ? piece : straightPiece(first, last));
            return;
        }
        Candidate candidate = bestFit(first, last, startDirection, endDirection);
        if (candidate.error <= tolerance) {
            // The Double cubic fits; the Float one the caller gets must fit too.
            Piece piece = roundedPiece(first, last, candidate.startOffset, candidate.endOffset);
            if (roundedError(piece, candidate.parameters) <= tolerance) {
                pieces.push_back(piece);
                return;
            }
        }
        if (depth >= deepestSplit) {
            for (int index = first; index < last; index += 1) pieces.push_back(straightPiece(index, index + 1));
            return;
        }
        int cut = candidate.worstIndex;
        Vector2D forward = direction(cut);
        fit(first, cut, startDirection, -forward, depth + 1);
        fit(cut, last, forward, endDirection, depth + 1);
    }

    /// The best cubic for a stretch of at least three points, by rounds of least squares and
    /// Newton reparameterization (steps 1 to 3 of the algorithm).
    Candidate bestFit(int first, int last, Vector2D startDirection, Vector2D endDirection) const {
        std::vector<Vector2D> stretch(points.begin() + first, points.begin() + last + 1);
        int count = static_cast<int>(stretch.size());
        std::vector<double> parameters = chordLengthParameters(stretch);
        double polylineLength = polylineLengthOf(stretch);
        double chord = length(stretch[count - 1] - stretch[0]);
        double fallbackLength = (chord > 0 ? chord : polylineLength) / 3;
        std::optional<Candidate> best;

        for (int round = 0; round < mostRounds; round += 1) {
            auto [solvedStart, solvedEnd] = solveHandleLengths(stretch, parameters, startDirection, endDirection);
            double startLength = solvedStart;
            double endLength = solvedEnd;
            if (!(startLength > chord * shortestHandleShare) || !(endLength > chord * shortestHandleShare)) {
                startLength = fallbackLength;
                endLength = fallbackLength;
            }
            startLength = smallerOf(startLength, polylineLength);
            endLength = smallerOf(endLength, polylineLength);
            LocalCubic cubic{
                stretch[0], stretch[0] + startDirection * startLength,
                stretch[count - 1] + endDirection * endLength, stretch[count - 1]};

            double error = 0.0;
            int worst = 1;
            for (int index = 1; index < count - 1; index += 1) {
                parameters[index] = cubic.newtonStep(parameters[index], stretch[index]);
                double distance = length(cubic.point(parameters[index]) - stretch[index]);
                if (distance > error) {
                    error = distance;
                    worst = index;
                }
            }
            double bestError = best ? best->error : std::numeric_limits<double>::infinity();
            if (error < bestError) {
                best = Candidate{startDirection * startLength, endDirection * endLength, error, first + worst, parameters};
            }
            if (error <= tolerance) break;
            if (round + 1 >= leastRounds && !(error < bestError * worthwhileImprovement)) break;
        }
        return best.value();
    }

    // MARK: Rounding to Float

    /// The piece over `first...last` with the handles `points[first] + startOffset` and
    /// `points[last] + endOffset` rounded to Float (each component rounded once, from the
    /// absolute Double value). A handle beyond the Float range comes out infinite.
    Piece roundedPiece(int first, int last, Vector2D startOffset, Vector2D endOffset) const {
        return Piece{first, last, floatPoint(points[first] + startOffset), floatPoint(points[last] + endOffset)};
    }

    /// The straight piece from `points[first]` to `points[last]`, its handles at the thirds. Always
    /// finite: the thirds lie between two finite Float points.
    Piece straightPiece(int first, int last) const {
        Vector2D step = (points[last] - points[first]) / 3;
        return roundedPiece(first, last, step, -step);
    }

    /// The largest distance from the points strictly inside the piece to the rounded piece,
    /// starting from `parameters` (one per point of the stretch) and refining each by a few
    /// Newton steps, keeping the smallest distance seen. Infinite if a handle is not finite.
    double roundedError(const Piece& piece, const std::vector<double>& parameters) const {
        if (!piece.startHandle.isFinite() || !piece.endHandle.isFinite()) return std::numeric_limits<double>::infinity();
        int first = piece.firstIndex;
        LocalCubic cubic{points[first], localPoint(piece.startHandle), localPoint(piece.endHandle), points[piece.lastIndex]};
        double error = 0.0;
        for (int index = first + 1; index < piece.lastIndex; index += 1) {
            double parameter = parameters[index - first];
            double distance = length(cubic.point(parameter) - points[index]);
            for (int step = 0; step < 3; step += 1) {
                parameter = cubic.newtonStep(parameter, points[index]);
                distance = smallerOf(distance, length(cubic.point(parameter) - points[index]));
            }
            error = largerOf(error, distance);
        }
        return error;
    }

    /// A point in the fitter's coordinates as an absolute Float point (rounded once).
    Vector2 floatPoint(Vector2D local) const {
        Vector2D absolute = local + origin;
        return Vector2(static_cast<float>(absolute.x), static_cast<float>(absolute.y));
    }

    /// A Float point in the fitter's coordinates (exact for points near the stroke).
    Vector2D localPoint(Vector2 point) const {
        return Vector2D{static_cast<double>(point.x), static_cast<double>(point.y)} - origin;
    }

    /// The least-squares handle lengths `(α₁, α₂)` for the given parameters (step 2), or
    /// `(NaN, NaN)` when the 2×2 system is singular, which the caller treats as "use the fallback".
    static std::pair<double, double> solveHandleLengths(
        const std::vector<Vector2D>& stretch, const std::vector<double>& parameters, Vector2D startDirection,
        Vector2D endDirection) {
        Vector2D first = stretch[0];
        Vector2D last = stretch[stretch.size() - 1];
        double a11 = 0.0, a12 = 0.0, a22 = 0.0, b1 = 0.0, b2 = 0.0;
        for (size_t index = 0; index < stretch.size(); index += 1) {
            Vector2D point = stretch[index];
            BernsteinWeights weights = LocalCubic::bernsteinWeights(parameters[index]);
            Vector2D startTerm = startDirection * weights.second;
            Vector2D endTerm = endDirection * weights.third;
            Vector2D remainder = point - first * (weights.first + weights.second) - last * (weights.third + weights.fourth);
            a11 += dot(startTerm, startTerm);
            a12 += dot(startTerm, endTerm);
            a22 += dot(endTerm, endTerm);
            b1 += dot(remainder, startTerm);
            b2 += dot(remainder, endTerm);
        }
        double determinant = a11 * a22 - a12 * a12;
        if (!(std::fabs(determinant) > 1e-12 * a11 * a22)) {
            return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
        }
        return {(b1 * a22 - b2 * a12) / determinant, (a11 * b2 - a12 * b1) / determinant};
    }

    // MARK: Tangent estimates

    /// The index of the first point from `index`, going `step` (±1) along the polyline, that is at
    /// least `reach` further along; the end of the list if there is none. Always differs from
    /// `index` unless `index` is the end of the list in that direction.
    int partner(int index, int step) const {
        int candidate = index + step;
        int lastIndex = static_cast<int>(points.size()) - 1;
        while (candidate > 0 && candidate < lastIndex && std::fabs(distanceAlong[candidate] - distanceAlong[index]) < reach) {
            candidate += step;
        }
        return std::min(std::max(candidate, 0), lastIndex);
    }

/// The direction of travel at `points[index]`: the direction of the derivative, at that point, of
/// the parabola through it and two **partner points** about `reach` (4 tolerances) away along
/// the polyline, parameterized by distance along the polyline. Inside the list the partners are
/// one before and one after; at the start both after, at the end both before. Using partners
/// further than the neighbours averages out wobble in a hand-drawn stroke that is smaller than
/// the tolerance, and for smooth data the parabola's direction is off by an angle of order
/// (reach/radius of curvature)² only.
///
/// How: the parabola through the points at nodes `s₀, s₁, s₂` is `Σ pⱼ·Lⱼ(s)` (Lagrange form), so its
/// derivative at `s` is `Σ pⱼ·Lⱼ′(s)`:
///
///     L₀′ = ((s−s₁)+(s−s₂)) / ((s₀−s₁)(s₀−s₂))     L₁′ = ((s−s₀)+(s−s₂)) / ((s₁−s₀)(s₁−s₂))
///     L₂′ = ((s−s₀)+(s−s₁)) / ((s₂−s₀)(s₂−s₁))
///
/// Falls back to the difference to the neighbours if there are no two partners (a stroke of 2
/// points) or the derivative is zero.
    Vector2D direction(int index) const {
        int last = static_cast<int>(points.size()) - 1;
        Vector2D neighbourDifference = index == 0 ? points[1] - points[0]
            : index == last ? points[last] - points[last - 1] : points[index + 1] - points[index - 1];
        std::optional<Vector2D> neighbourDirection = unitDirection(neighbourDifference);
        Vector2D fallback = neighbourDirection ? *neighbourDirection : Vector2D{1, 0};
        if (!(last >= 2)) return fallback;
        int indices[3];
        int position;
        if (index == 0) {
            int first = partner(0, 1);
            indices[0] = 0;
            indices[1] = first;
            indices[2] = partner(first, 1);
            position = 0;
        } else if (index == last) {
            int first = partner(last, -1);
            indices[0] = partner(first, -1);
            indices[1] = first;
            indices[2] = last;
            position = 2;
        } else {
            indices[0] = partner(index, -1);
            indices[1] = index;
            indices[2] = partner(index, 1);
            position = 1;
        }
        if (!(indices[0] < indices[1] && indices[1] < indices[2])) return fallback;
        std::optional<Vector2D> found = tangent(
            {TangentEntry{points[indices[0]], distanceAlong[indices[0]]},
             TangentEntry{points[indices[1]], distanceAlong[indices[1]]},
             TangentEntry{points[indices[2]], distanceAlong[indices[2]]}},
            position);
        return found ? *found : fallback;
    }

    /// The unit direction of the derivative of the parabola through three (point, node) pairs, at
    /// the node of entry `which`; nothing if the derivative is zero or not finite.
    static std::optional<Vector2D> tangent(const std::vector<TangentEntry>& entries, int which) {
        Vector2D derivative;
        for (int current = 0; current < 3; current += 1) {
            int others[2];
            int found = 0;
            for (int candidate = 0; candidate < 3; candidate += 1) {
                if (candidate != current) others[found++] = candidate;
            }
            double node = entries[which].node;
            double numerator = (node - entries[others[0]].node) + (node - entries[others[1]].node);
            double denominator = (entries[current].node - entries[others[0]].node)
                * (entries[current].node - entries[others[1]].node);
            derivative += entries[current].point * (numerator / denominator);
        }
        return unitDirection(derivative);
    }

    // MARK: Small helpers

    /// The four Bernstein weights `(1−t)³, 3(1−t)²t, 3(1−t)t², t³` (Swift's tuple of four).
    struct BernsteinWeights {
        double first;
        double second;
        double third;
        double fourth;
    };

    /// A cubic in the fitter's own coordinates, with the derivatives Newton's method needs.
    struct LocalCubic {
        Vector2D p0, p1, p2, p3;

        static BernsteinWeights bernsteinWeights(double t) {
            double u = 1 - t;
            return BernsteinWeights{u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t};
        }

        Vector2D point(double t) const {
            BernsteinWeights w = bernsteinWeights(t);
            return p0 * w.first + p1 * w.second + p2 * w.third + p3 * w.fourth;
        }

        Vector2D derivative(double t) const {
            double u = 1 - t;
            return (p1 - p0) * (3 * u * u) + (p2 - p1) * (6 * u * t) + (p3 - p2) * (3 * t * t);
        }

        Vector2D secondDerivative(double t) const {
            double u = 1 - t;
            return (p2 - p1 * 2 + p0) * (6 * u) + (p3 - p2 * 2 + p1) * (6 * t);
        }

        /// One Newton step on `(B(t) − target)·B′(t) = 0`, clamped to 0...1; `t` itself when
        /// the denominator is zero or not finite.
        double newtonStep(double t, Vector2D target) const {
            Vector2D gap = point(t) - target;
            Vector2D slope = derivative(t);
            double denominator = dot(slope, slope) + dot(gap, secondDerivative(t));
            if (!(denominator != 0 && std::isfinite(denominator))) return t;
            double next = t - dot(gap, slope) / denominator;
            return std::isfinite(next) ? smallerOf(largerOf(next, 0), 1) : t;
        }
    };

    /// The vector scaled to length 1, dividing by the largest component first so that tiny or
    /// huge vectors neither underflow nor overflow; nothing for a zero or non-finite vector.
    static std::optional<Vector2D> unitDirection(Vector2D vector) {
        if (!std::isfinite(vector.x) || !std::isfinite(vector.y)) return std::nullopt;
        double largest = largerOf(std::fabs(vector.x), std::fabs(vector.y));
        if (!(largest > 0)) return std::nullopt;
        Vector2D scaled = vector / largest;
        return scaled / length(scaled);
    }

    static double polylineLengthOf(const std::vector<Vector2D>& stretch) {
        double total = 0.0;
        for (size_t index = 1; index < stretch.size(); index += 1) total += length(stretch[index] - stretch[index - 1]);
        return total;
    }

    /// Parameters 0...1 proportional to the polyline length up to each point.
    static std::vector<double> chordLengthParameters(const std::vector<Vector2D>& stretch) {
        std::vector<double> parameters{0.0};
        double total = 0.0;
        for (size_t index = 1; index < stretch.size(); index += 1) {
            total += length(stretch[index] - stretch[index - 1]);
            parameters.push_back(total);
        }
        for (size_t index = 1; index < parameters.size(); index += 1) parameters[index] /= total;
        parameters[parameters.size() - 1] = 1;
        return parameters;
    }
};

} // namespace

/// Cubic Bézier pieces, in order, that pass through the first and the last point and keep
/// every point of `points` within `tolerance` of the curve: the pen tool run backwards.
///
/// How (Schneider's algorithm):
/// 1. **Parameterize.** Give every point a parameter `u` from 0 to 1 in proportion to the
///    length of the polyline up to it (chord length).
/// 2. **Solve the handles.** The end points and the unit end directions `t₀` (leaving the
///    start) and `t₃` (leaving the end, pointing back into the curve) are fixed; only the two
///    handle lengths `α₁`, `α₂` are free, with `P₁ = P₀ + α₁·t₀` and `P₂ = P₃ + α₂·t₃`.
///    Minimizing `Σ |B(uᵢ) − dᵢ|²` over the points `dᵢ` is a least-squares problem. Setting
///    the two derivatives to zero gives a 2×2 linear system
///
///        [ Σ A₁·A₁   Σ A₁·A₂ ] [α₁]   [ Σ r·A₁ ]     A₁ = b₁(uᵢ)·t₀,  A₂ = b₂(uᵢ)·t₃,
///        [ Σ A₁·A₂   Σ A₂·A₂ ] [α₂] = [ Σ r·A₂ ]     r = dᵢ − (b₀+b₁)(uᵢ)·P₀ − (b₂+b₃)(uᵢ)·P₃
///
///    where `bⱼ` are the Bernstein weights. It is solved by Cramer's rule. If the system is
///    singular, or a handle comes out shorter than a thousandth of the chord (or negative:
///    the curve would double back), both handles fall back to a third of the chord (to a
///    third of the polyline length if the chord is zero). A handle never exceeds the
///    polyline length.
/// 3. **Reparameterize.** Move each `uᵢ` one Newton step toward the nearest point of the
///    curve: `u ← u − (B(u) − d)·B′(u) / (B′(u)·B′(u) + (B(u) − d)·B″(u))`, then solve again.
///    At least 8 rounds and at most 128, stopping as soon as the error is within `tolerance`,
///    or, after the eighth, at the first round that does not lower the best error so far by at
///    least 0.01%; the best round is kept. Letting the rounds run while they still help saves
///    pieces where the chord-length parameters start far from the final ones, as on a tight U
///    turn whose speed varies twentyfold (measured: 3 pieces instead of 6 at a tolerance of
///    0.01, 7 instead of 8 at 0.001). More Newton steps per round were tried and gave more pieces, not fewer.
/// 4. **Split.** If the largest distance `|B(uᵢ) − dᵢ|` is still above `tolerance`, cut the
///    points at that worst point, give both halves the same tangent there (the direction
///    of the parabola through it and its two neighbours), and fit each half the same way. The cut point
///    is the end of one piece and the start of the next, so the join is exact and smooth.
///
///        points  ● ● ●●  ● ●    ●  ●      one cubic misses the worst point by too much →
///                         ▲ worst          split there, two cubics sharing that point
///
/// The end directions are estimated from the parabola through the end point and two points about
/// four tolerances further along the polyline (parameterized by distance along it): much closer
/// to the true tangent than the difference to the neighbour, and calm where the stroke wobbles by
/// less than the tolerance. They are taken from `startDirection` / `endDirection` instead when
/// given (as directions of travel; a zero or non-finite one counts as not given). The tangent at
/// a split uses the same parabola through the cut point and one partner on each side.
///
/// **What the tolerance means.** `tolerance` is a *distance* in the points' own units (not a
/// squared distance). It bounds the distance from each input point to the curve; between the
/// points the curve may stray further, as with any fit to samples. The distance is measured
/// at the point's refined parameter, which is never smaller than the true nearest distance, so
/// the true distance is within the tolerance too.
///
/// **Rounding.** The fit works in Double and the result is Float, so each handle is rounded
/// once. Every piece is checked again after that rounding, against the input points, and split
/// further if it no longer meets the tolerance. For the same reason the tolerance actually
/// used is never below `smallestFittingTolerance(for:)`, four Float steps at the largest
/// coordinate: a smaller one could only be met by straight pieces between neighbours.
///
/// **Edge cases.**
/// - Consecutive points that are exactly equal are duplicates and only one is kept. Points
///   that differ at all are kept, however close: dropping them would leave them outside the
///   tolerance. Fewer than two distinct points give no pieces.
/// - Two points give one straight piece (handles a third of the chord), curved if a direction
///   is given.
/// - Points on a line give one straight piece.
/// - A **closed** input is cut first at the point furthest from the start, and its start and
///   end directions default to the same smooth tangent, so the result is a smooth closed loop:
///   the last piece ends exactly where the first begins. `closed` says whether the input is
///   closed: `nil` (the default) means closed when there are four or more points (after exact
///   duplicates are dropped) and the last is exactly the first, so `[A, B, A]` is an open stroke that goes there and back
///   with a cusp at `B`; `true` closes it, adding the first point at the end if it is not
///   there already, and gives no pieces for fewer than three distinct points; `false` fits it
///   as an open stroke even when it ends where it began.
/// - Empty result for: fewer than two points, more than `mostFittedPoints`, any non-finite
///   coordinate, or a `tolerance` that is not positive and finite.
/// - Past `deepestFittingSplit` halvings the fit ends in short straight pieces through the
///   points, never more than `points.count − 1` pieces.
///
/// All the arithmetic is in Double, measured from the first point, so the result does not
/// depend on how far from the origin the stroke is, nor on its scale (beyond the floor above).
/// The end point of every piece is an input point, copied unchanged.
///
/// Source: Schneider, "An algorithm for automatically fitting digitized curves", *Graphics
/// Gems* (1990); the Newton step is the one in Schneider's `Reparameterize`.
///
/// Use when: turning a hand-drawn stroke or a dense polyline into a few editable Bézier
///   curves; simplifying a flattened outline; fitting a curve to sampled data
/// Tags: curve fitting, fit Bezier to points, Schneider, digitized curve, stroke smoothing,
///   path simplification, least squares Bezier, freehand to Bezier, potrace-like
/// Node: inside fitProfile
template <CurvePoint PointType>
std::vector<CubicBezier<Vector2>> CubicBezier<PointType>::fitted(
    const std::vector<Vector2>& points, float tolerance, std::optional<Vector2> startDirection,
    std::optional<Vector2> endDirection, std::optional<bool> closed)
    requires std::same_as<PointType, Vector2>
{
    // Written so that NaN fails every test.
    if (!(tolerance > 0 && std::isfinite(tolerance) && points.size() >= 2 && points.size() <= static_cast<size_t>(mostFittedPoints))) {
        return {};
    }
    for (Vector2 point : points) {
        if (!point.isFinite()) return {};
    }
    std::vector<Vector2> distinct = withoutDuplicates(points);
    bool isClosed = fitsAsClosedLoop(distinct, closed);
    if (closed == true) {
        if (!isClosed) return {};
        if (distinct[distinct.size() - 1] != distinct[0]) distinct.push_back(distinct[0]);
    }
    if (!(distinct.size() >= 2)) return {};

    float usedTolerance = largerOf(tolerance, smallestFittingTolerance(distinct));
    CubicFitter fitter(distinct, static_cast<double>(usedTolerance), deepestFittingSplit);
    std::optional<Vector2D> givenStart;
    if (startDirection) givenStart = CubicFitter::unitDirection(*startDirection);
    std::optional<Vector2D> givenEnd;
    if (endDirection) givenEnd = CubicFitter::unitDirection(*endDirection);
    if (isClosed) {
        fitter.fitClosed(givenStart, givenEnd);
    } else {
        fitter.fitOpen(givenStart, givenEnd);
    }
    std::vector<CubicBezier<Vector2>> result;
    for (const CubicFitter::Piece& piece : fitter.pieces) {
        result.push_back(CubicBezier<Vector2>(
            distinct[piece.firstIndex], piece.startHandle, piece.endHandle, distinct[piece.lastIndex]));
    }
    return result;
}

/// The smallest tolerance `fitted(to:tolerance:)` works to for `points`: four Float steps
/// (ulp) at the largest absolute coordinate, `4·ulp(max |xᵢ|, |yᵢ|)`. A smaller tolerance is
/// raised to it.
///
/// Why four: the handles come out of a Double computation and are rounded to Float, each
/// component by up to half a step. A handle can lie a little outside the points, where the
/// step can be twice as large, and a point of the curve moves by at most `b₁ + b₂ ≤ 3/4` of the
/// handles' movement, so rounding alone can move the curve by about one step. A tolerance of
/// a step or two could then be met only by splitting down to straight pieces between
/// neighbours; four leaves room for smooth pieces. For points all at the origin it is four
/// times the smallest positive Float, the step at 0.
///
/// Use when: knowing how tight a fit can be for points far from the origin; picking a
///   tolerance for a stroke in large world coordinates
/// Tags: tolerance floor, Float precision, ulp, rounding limit, smallest tolerance
/// Node: none (solver)
template <CurvePoint PointType>
float CubicBezier<PointType>::smallestFittingTolerance(const std::vector<Vector2>& points)
    requires std::same_as<PointType, Vector2>
{
    float largest = 0;
    for (Vector2 point : points) largest = largerOf(largerOf(largest, std::fabs(point.x)), std::fabs(point.y));
    return curve_detail::unitInTheLastPlace(largest) * 4;
}

/// Whether `fitted(to:tolerance:startDirection:endDirection:closed:)` fits these points
/// (already without duplicates) as a closed loop. `closed == false`: never. `closed == nil`:
/// when there are four or more points and the last is exactly the first. `closed == true`:
/// when there are at least three distinct points, not counting a last one equal to the first.
///
/// Use when: deciding whether to close the path built from the fitted pieces
/// Tags: closed stroke, loop detection, closed curve, closed path
/// Node: none (solver)
template <CurvePoint PointType>
bool CubicBezier<PointType>::fitsAsClosedLoop(const std::vector<Vector2>& points, std::optional<bool> closed)
    requires std::same_as<PointType, Vector2>
{
    if (points.empty()) return false;
    Vector2 first = points.front();
    Vector2 last = points.back();
    if (closed == false) return false;
    if (!closed) return points.size() >= 4 && last == first;
    return (last == first ? points.size() - 1 : points.size()) >= 3;
}

/// The points without consecutive exact duplicates: a point equal to the one before it is
/// left out. Points that differ at all are kept, however close, because the fit promises
/// every input point is within the tolerance. Never empty for a non-empty input.
///
/// Use when: cleaning a sampled stroke before fitting, so no zero-length step gives a
///   direction of NaN
/// Tags: remove duplicates, dedupe points, consecutive duplicates, clean polyline
/// Node: none (solver)
template <CurvePoint PointType>
std::vector<Vector2> CubicBezier<PointType>::withoutDuplicates(const std::vector<Vector2>& points)
    requires std::same_as<PointType, Vector2>
{
    if (points.empty()) return {};
    std::vector<Vector2> kept{points.front()};
    for (size_t index = 1; index < points.size(); index += 1) {
        if (points[index] != kept[kept.size() - 1]) kept.push_back(points[index]);
    }
    return kept;
}

template class CubicBezier<Vector2>;

} // namespace editor
