// The length table of a path, and places named by a share of length.
#include "MeasuredPath2D.h"

#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

namespace {

/// A tangent shorter than this has no direction.
constexpr float shortestTangent = 1e-6f;

/// The pieces of `path` as measured: the drawn ones, then the closing line of a closed path whose
/// pen is away from its start.
std::vector<OutlinePiece> measuredPieces(const Path2D& path) {
    std::vector<OutlinePiece> pieces = drawnPieces(path);
    if (path.isClosed() && !(path.currentPoint() == path.start)) pieces.push_back(OutlinePiece::line(path.currentPoint(), path.start));
    return pieces;
}

CubicBezier<Vector2> scaledBy(const CubicBezier<Vector2>& cubic, Vector2 metric) {
    return CubicBezier<Vector2>(cubic.start * metric, cubic.startHandle * metric, cubic.endHandle * metric, cubic.end * metric);
}

} // namespace

/// Measures `path`: the pieces, and the length of each in `metric`.
///
/// How: a line is the distance between its scaled ends; a curve is `Curve::totalLength` of the
/// curve with every control point multiplied by `metric` (an adaptive Gauss–Legendre integral of
/// speed). Graphics_cpp used 24-point Gauss; the two agree to about 1e-4 of the length.
///
/// Conventions: a `metric` component that is not above 0 and finite throws a `MathProblem`.
///
/// Use when: measuring a profile before placing marks along it
/// Tags: measure path, arc length table, path length
/// Node: planned (measurePath)
MeasuredPath2D::MeasuredPath2D(Path2D path, Vector2 metric)
    : pathValue(std::move(path)), metricValue(metric), pieceList(measuredPieces(pathValue)) {
    if (!(metric.x > 0) || !(metric.y > 0) || !std::isfinite(metric.x) || !std::isfinite(metric.y)) {
        throw MathProblem("A metric needs both components above 0 and finite.");
    }
    for (size_t index = 0; index < pieceList.size(); index += 1) {
        const OutlinePiece& piece = pieceList[index];
        lengthList.push_back(piece.isLine ? (piece.end() * metric).distance(piece.start() * metric)
                                          : scaledBy(piece.cubic, metric).totalLength());
    }
}

CubicBezier<Vector2> MeasuredPath2D::pieceInMetric(int pieceIndex) const {
    return scaledBy(pieceList[static_cast<size_t>(pieceIndex)].cubic, metricValue);
}

/// The length measured so far along the whole path up to the start of piece `pieceIndex`.
float MeasuredPath2D::lengthBefore(int pieceIndex) const {
    float total = 0;
    for (int index = 0; index < pieceIndex && index < static_cast<int>(lengthList.size()); index += 1) {
        total += lengthList[static_cast<size_t>(index)];
    }
    return total;
}

/// The length along piece `pieceIndex` from its start to `parameter`, in the metric (a line is
/// linear in parameter).
float MeasuredPath2D::lengthOfPieceUpTo(int pieceIndex, float parameter) const {
    const OutlinePiece& piece = pieceList[static_cast<size_t>(pieceIndex)];
    if (piece.isLine) return lengthList[static_cast<size_t>(pieceIndex)] * parameter;
    return pieceInMetric(pieceIndex).length(0, parameter);
}

/// The parameter reached on piece `pieceIndex` after `distance` along it in the metric, held in
/// 0…1.
float MeasuredPath2D::parameterAtDistanceOnPiece(int pieceIndex, float distance) const {
    float pieceLength = lengthList[static_cast<size_t>(pieceIndex)];
    if (distance <= 0) return 0;
    if (distance >= pieceLength) return 1;
    if (pieceList[static_cast<size_t>(pieceIndex)].isLine) return distance / pieceLength;
    return pieceInMetric(pieceIndex).parameterAtDistance(distance);
}

/// The length of the whole path in the metric: the sum of its pieces' lengths.
///
/// Use when: the total to divide a share of length by; checking that a profile is long enough
/// Tags: path length, perimeter, total length, arc length
/// Node: planned (pathLength)
float MeasuredPath2D::length() const { return lengthBefore(static_cast<int>(lengthList.size())); }

/// The place at `progress`, a share of the length (held in 0…1): the piece, the parameter on it
/// and the point. `distance` of the result is 0.
///
/// How: the target length is progress × total; walk the pieces, taking the first whose length so
/// far reaches it (the last, if rounding leaves a sliver), and turn what is left over into a
/// parameter with `Curve::parameterAtDistance` (a line: left over ÷ length). An exact node
/// belongs to the piece that ends there.
///
/// Conventions: a path with no pieces gives piece 0, parameter 0, at its start.
///
/// Use when: stepping along a profile at equal distances; the inverse of `progress`
/// Tags: locate, point at length, parameter at progress, normalized arc length, walk along path
/// Node: planned (pointOnPath)
PathPoint2 MeasuredPath2D::locate(float progress) const {
    if (pieceList.empty()) return PathPoint2{0, 0, pathValue.start, 0};
    float remaining = clamped(progress, 0.0f, 1.0f) * length();
    int lastIndex = static_cast<int>(pieceList.size()) - 1;
    for (int index = 0; index <= lastIndex; index += 1) {
        float pieceLength = lengthList[static_cast<size_t>(index)];
        if (remaining <= pieceLength || index == lastIndex) {
            float parameter = parameterAtDistanceOnPiece(index, remaining);
            return PathPoint2{index, parameter, pieceList[static_cast<size_t>(index)].pointAt(parameter), 0};
        }
        remaining -= pieceLength;
    }
    return PathPoint2{0, 0, pathValue.start, 0};
}

/// The share of the length reached at `parameter` of piece `pieceIndex`: `locate` backwards.
///
/// How: (length of the pieces before + length of the piece up to `parameter`) ÷ total; a total of 0
/// is counted as 1, so a path of no length answers 0.
///
/// Conventions: a piece the path does not have throws a `MathProblem`.
///
/// Use when: turning a crossing or a nearest point into a position along the path
/// Tags: progress, arc length fraction, parameter to length share, normalized length
/// Node: planned (pointOnPath)
float MeasuredPath2D::progress(int pieceIndex, float parameter) const {
    if (pieceIndex < 0 || pieceIndex >= static_cast<int>(pieceList.size())) throw MathProblem("The path has no such piece.");
    float total = length();
    return (lengthBefore(pieceIndex) + lengthOfPieceUpTo(pieceIndex, parameter)) / (total > 0 ? total : 1);
}

/// The point at `progress` along the path (the start for a path with no pieces).
///
/// Use when: placing an object a share of the way along an outline
/// Tags: point at progress, point along path, sample path
/// Node: planned (pointOnPath)
Vector2 MeasuredPath2D::pointAt(float progress) const { return locate(progress).point; }

/// The unit direction the path runs in at `progress`, in the metric: the tangent of the path with
/// its coordinates scaled by `metric`, so it is the direction on the page when the drawing is
/// stretched.
///
/// How: the derivative of the scaled piece at the parameter; where that is shorter than 1e-6 (a
/// handle sitting on its node) the direction of the nearer handle, then of the chord. (1, 0) for
/// a path with no pieces.
///
/// Use when: turning an object to follow an outline; the normal for an offset
/// Tags: path direction, tangent at progress, heading along path
/// Node: planned (pathDirection)
Vector2 MeasuredPath2D::directionAt(float progress) const {
    if (pieceList.empty()) return Vector2(1, 0);
    PathPoint2 place = locate(progress);
    CubicBezier<Vector2> scaled = pieceInMetric(place.pieceIndex);
    Vector2 velocity = scaled.firstDerivative(place.parameter);
    if (velocity.length() > shortestTangent) return velocity.normalized();
    Vector2 handleDirection = place.parameter < 0.5f ? scaled.startHandle - scaled.start : scaled.end - scaled.endHandle;
    if (handleDirection.length() > shortestTangent) return handleDirection.normalized();
    return (scaled.end - scaled.start).normalized();
}

/// The direction from the start to the end of the path, as an angle from the horizontal (counter-
/// clockwise positive). 0 for a path that ends where it starts.
///
/// Use when: laying a road or a bridge along its chord; tilting an object between two points
/// Tags: chord angle, start to end angle, slope, overall direction
/// Node: planned (pathDirection)
Angle MeasuredPath2D::chordAngle() const {
    Vector2 chord = pathValue.currentPoint() - pathValue.start;
    return Angle::fromRadians(std::atan2(chord.y, chord.x));
}

/// The nodes in order: the start, then the end of each piece. A closed path's start comes again at
/// its end.
///
/// Use when: listing the points a user can grab; the corners of a polygon
/// Tags: nodes, anchor points, vertices of path
/// Node: planned (pathNodes)
std::vector<Vector2> MeasuredPath2D::nodes() const {
    std::vector<Vector2> points{pathValue.start};
    for (const OutlinePiece& piece : pieceList) points.push_back(piece.end());
    return points;
}

/// The share of length at node `nodeIndex` (0 is the start, then the end of each piece): the
/// lengths of the pieces before it ÷ the total, or 0 for a path of no length.
///
/// Conventions: a node the path does not have throws a `MathProblem`.
///
/// Use when: placing labels at the nodes; weighting a falloff by distance along the path
/// Tags: node progress, anchor position, length at node
/// Node: planned (pathNodes)
float MeasuredPath2D::nodeProgress(int nodeIndex) const {
    if (nodeIndex < 0 || nodeIndex > static_cast<int>(pieceList.size())) throw MathProblem("The path has no such node.");
    float total = length();
    return total > 0 ? lengthBefore(nodeIndex) / total : 0;
}

/// How far along the path each node is, in the metric: 0 for the start, then the running sum of
/// the piece lengths.
///
/// Use when: spacing marks by length; texture coordinates along an outline
/// Tags: distances along path, cumulative length, node distances, chainage
/// Node: planned (pathNodes)
std::vector<float> MeasuredPath2D::nodeDistances() const {
    std::vector<float> distances{0};
    for (float pieceLength : lengthList) distances.push_back(distances.back() + pieceLength);
    return distances;
}

/// A share of the length along some pieces only, as a share along the whole path: `progress` 0
/// is the start of `firstPiece` and 1 the end of `lastPiece`.
///
/// How: blend between nodeProgress(firstPiece) and nodeProgress(lastPiece + 1) by `progress`.
///
/// Conventions: negative piece numbers count from the end (-1 is the last piece); a run that is
/// not inside the path or runs backwards throws a `MathProblem`.
///
/// Use when: placing something along one side of a shape, by the side's own 0…1
/// Tags: progress within pieces, sub range, local to global progress
/// Node: planned (pathNodes)
float MeasuredPath2D::progressWithin(int firstPiece, int lastPiece, float progress) const {
    int count = static_cast<int>(pieceList.size());
    int first = firstPiece < 0 ? count + firstPiece : firstPiece;
    int last = lastPiece < 0 ? count + lastPiece : lastPiece;
    if (!(first >= 0 && last < count && first <= last)) throw MathProblem("These pieces are not in the path.");
    return linearBlend(nodeProgress(first), nodeProgress(last + 1), progress);
}

} // namespace editor
