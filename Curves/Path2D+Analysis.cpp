// A path asked about places along it: where it is level or upright, where it changes its bend,
// the furthest it reaches, how tight it turns, the circle that matches it, dragging the curve
// itself, the nearest place to a point and how near each place is to some others. Ported from
// graphics_cpp's analysis.cpp, where these were free functions on `Path`; here they
// are members of `Path2D` that measure a copy of the path in a `metric` and answer in progress
// (a share of length, see `MeasuredPath2D`).
#include <algorithm>
#include <cmath>
#include <limits>

#include "../Linear/MathProblem.h"
#include "MeasuredPath2D.h"
#include "Path2D.h"

namespace editor {

namespace {

/// How close to a piece's end a parameter may be and still count as the node there, not a place
/// inside the piece.
constexpr float nodeParameterMargin = 1e-6f;

/// How much further a candidate must reach than the best so far to replace it, so that ties (a
/// flat top, two equal peaks) keep the first.
constexpr float reachTieSlack = 1e-5f;

/// The curve with every control point scaled by `metric`: the curve as it looks on the page.
CubicBezier<Vector2> curveInMetric(const CubicBezier<Vector2>& curve, Vector2 metric) {
    return CubicBezier<Vector2>(curve.start * metric, curve.startHandle * metric, curve.endHandle * metric, curve.end * metric);
}

/// The progresses where the pieces of `measured` are level or upright inside the piece (the
/// nodes, where a piece meets the next, are not counted).
std::vector<float> extremaOf(const MeasuredPath2D& measured) {
    std::vector<float> progresses;
    const std::vector<OutlinePiece>& pieces = measured.pieces();
    for (size_t index = 0; index < pieces.size(); index += 1) {
        if (pieces[index].isLine) continue;
        std::vector<float> parameters = pieces[index].cubic.extremaParameters();
        std::sort(parameters.begin(), parameters.end());
        for (float parameter : parameters) {
            if (parameter > nodeParameterMargin && parameter < 1 - nodeParameterMargin) {
                progresses.push_back(measured.progress(static_cast<int>(index), parameter));
            }
        }
    }
    return progresses;
}

/// The nodes the piece at `pieceIndex` runs between, for a path of `pieceCount` pieces: the one it
/// leaves and the one it arrives at (a closed path's last piece arrives at node 0).
int arrivalNode(int pieceIndex, int pieceCount, bool closed) {
    return closed ? (pieceIndex + 1) % pieceCount : pieceIndex + 1;
}

/// The path with the straight line that closes it drawn as a piece of its own (a cubic with its
/// handles at thirds), so that its handles can be moved. The same path if it ends on its start.
Path2D withClosingLineDrawn(const Path2D& path) {
    if (!path.isClosed() || path.currentPoint() == path.start) return path;
    Path2D drawn(path.start);
    for (const Path2D::Segment& segment : path.segments()) {
        if (segment.isLine()) drawn.addLine(segment.end());
        else drawn.addCubic(segment.startHandle, segment.endHandle, segment.end());
    }
    CubicBezier<Vector2> closing = Path2D::evenlyPacedCubic(path.currentPoint(), path.start);
    drawn.addCubic(closing.startHandle, closing.endHandle, closing.end);
    drawn.close();
    for (int node = 0; node <= static_cast<int>(path.segments().size()); node += 1) drawn.setNodeKind(node, path.nodeKind(node));
    return drawn;
}

} // namespace

/// Where the path is level or upright inside its pieces, as progress in order along the path.
///
/// How: each curved piece's `extremaParameters` (where the x or y derivative is zero), keeping the
/// ones strictly inside the piece (a node is not counted: cut a path with `MeasuredPath2D::cut`
/// at these to get pieces that stay on one side). Straight pieces have none.
///
///        ╭───╮ ← level (y extremum)
///       ╱     ╲
///     ──       ● upright (x extremum)
///
/// Conventions: y-up plane; progress is a share of length in `metric` (1, 1 is true length), from
/// 0 to 1. Empty for a path of straight pieces.
///
/// Source: graphics_cpp curves/analysis.cpp `extrema`.
///
/// Use when: cutting a stroke where it turns round so each piece is monotone; finding where an
///   outline touches its bounds
/// Tags: extrema, turning points, tangent horizontal, tangent vertical, bounding box points
/// Node: planned (pathExtrema)
std::vector<float> Path2D::extremaProgresses(Vector2 metric) const {
    return extremaOf(MeasuredPath2D(*this, metric));
}

/// Where the path changes the way it bends inside its pieces (a curve going from turning left to
/// turning right, or back), as progress in order. Corners at nodes are not inflections.
///
/// How: each curved piece's `inflectionParameters` (where the signed curvature passes through
/// zero), turned into progress.
///
///     ╭──╮        ╭─ ← turns left, then right: the point between is the inflection
///        ╰──╮   ╭─╯
///           ╰───╯
///
/// Conventions: y-up plane; progress is a share of length in `metric`. Straight pieces have none.
///
/// Source: graphics_cpp curves/analysis.cpp `inflections`.
///
/// Use when: splitting an S-shaped stroke into pieces that bend one way; finding where a road
///   changes from a left bend to a right bend
/// Tags: inflection points, change of curvature, S bend, flex points
/// Node: planned (pathInflections)
std::vector<float> Path2D::inflectionProgresses(Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    std::vector<float> progresses;
    const std::vector<OutlinePiece>& pieces = measured.pieces();
    for (size_t index = 0; index < pieces.size(); index += 1) {
        if (pieces[index].isLine) continue;
        std::vector<float> parameters = pieces[index].cubic.inflectionParameters();
        std::sort(parameters.begin(), parameters.end());
        for (float parameter : parameters) progresses.push_back(measured.progress(static_cast<int>(index), parameter));
    }
    return progresses;
}

/// The place the path reaches furthest to one side, as progress; of places equally far, the first.
///
/// How: the candidates are the start of every piece, every extremum inside a piece, and the end;
/// the answer is the one with the largest coordinate along the side (smallest for left and
/// bottom). A candidate must beat the best by more than 1e-5 to replace it, so a flat top is the
/// progress where it begins.
///
///     top ● ← progressOfSide(top)
///        ╱ ╲
///     ───   ───
///
/// Conventions: y-up plane (top is the largest y); progress is a share of length in `metric`. A
/// path with no pieces gives 1.
///
/// Source: graphics_cpp curves/analysis.cpp `side`.
///
/// Use when: anchoring a label to the top of a bump; finding the tip of a leaf outline
/// Tags: extreme point, topmost, leftmost, furthest point, bounds touch point
/// Node: planned (pathSide)
float Path2D::progressOfSide(PathSide side, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    bool usesX = side == PathSide::left || side == PathSide::right;
    float sign = side == PathSide::left || side == PathSide::bottom ? -1.0f : 1.0f;
    std::vector<float> candidates;
    for (size_t index = 0; index < measured.pieces().size(); index += 1) {
        candidates.push_back(measured.progress(static_cast<int>(index), 0));
    }
    for (float extremum : extremaOf(measured)) candidates.push_back(extremum);
    candidates.push_back(1);
    float best = 0;
    float bestReach = -std::numeric_limits<float>::infinity();
    for (float candidate : candidates) {
        Vector2 point = measured.pointAt(candidate);
        float reach = sign * (usesX ? point.x : point.y);
        if (reach > bestReach + reachTieSlack) {
            bestReach = reach;
            best = candidate;
        }
    }
    return best;
}

/// How tight the path turns at a progress, in `metric`: one over the radius of the circle that
/// matches it, positive when it turns left (counterclockwise), 0 where it runs straight.
///
/// How: the signed curvature of the piece at that progress, taken on the piece scaled by `metric`,
/// `(x′·y″ − y′·x″) / |(x′, y′)|³`.
///
/// Conventions: y-up plane; 0 for a path with no pieces. At a node the piece that starts there is
/// read.
///
/// Source: graphics_cpp curves/analysis.cpp `curvature`.
///
/// Use when: slowing a vehicle on tight bends; shading a stroke by how it bends
/// Tags: curvature, bend, turning rate, 1/radius
/// Node: planned (pathCurvature)
float Path2D::curvatureAt(float progress, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    if (measured.pieces().empty()) return 0;
    PathPoint2 place = measured.locate(progress);
    return curveInMetric(measured.pieces()[static_cast<size_t>(place.pieceIndex)].cubic, metric).signedCurvature(place.parameter);
}

/// The unit normal of the path at a progress, in `metric`: its direction turned a quarter turn to
/// the left.
///
/// How: with direction (x, y) the normal is (−y, x).
///
///     direction →      normal ↑
///
/// Conventions: y-up plane; the left of the way the path runs. (0, 1) for a path with no pieces
/// (direction (1, 0)).
///
/// Source: graphics_cpp curves/analysis.cpp `normal`.
///
/// Use when: pushing a label off a stroke; the side an offset curve goes
/// Tags: normal, perpendicular, left of path, offset direction
/// Node: planned (pathNormal)
Vector2 Path2D::normalAt(float progress, Vector2 metric) const {
    Vector2 direction = MeasuredPath2D(*this, metric).directionAt(progress);
    return Vector2(-direction.y, direction.x);
}

/// The circle that matches the path at a progress: its centre (nothing where the path runs
/// straight), its radius in `metric` and the signed curvature.
///
/// How: the piece's `osculatingCircle`, worked on the piece scaled by `metric`; the centre is then
/// scaled back to the path's own coordinates, the radius and curvature stay in the metric.
///
/// Conventions: y-up plane; an empty path or a straight place gives no centre, infinite radius and
/// curvature 0.
///
/// Source: graphics_cpp curves/analysis.cpp `osculating`.
///
/// Use when: rounding a corner to match the bend next to it; drawing the curvature circle
/// Tags: osculating circle, curvature circle, centre of curvature, radius of curvature
/// Node: planned (osculatingCircle)
OsculatingCircle Path2D::osculatingCircleAt(float progress, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    if (measured.pieces().empty()) return OsculatingCircle{std::nullopt, std::numeric_limits<float>::infinity(), 0};
    PathPoint2 place = measured.locate(progress);
    OsculatingCircle circle =
        curveInMetric(measured.pieces()[static_cast<size_t>(place.pieceIndex)].cubic, metric).osculatingCircle(place.parameter);
    if (circle.centre) circle.centre = *circle.centre / metric;
    return circle;
}

/// The path with its point at a progress dragged to `destination`, the piece there bent through it
/// (the Primer's molding: dragging the curve itself, not a handle).
///
/// How: the piece at the progress is `CubicBezier::molded` to pass through `destination` at its
/// parameter; its two handles are then set with `withHandleMoved`, so the nodes at its ends keep
/// their kinds (a smooth node's other handle follows).
///
/// Conventions: y-up plane; a path with no pieces is returned unchanged. The closing line of a
/// closed path can be molded too: it is drawn as a piece of its own first (so the result ends on
/// its start and its closing line is a curve).
///
/// Source: graphics_cpp curves/analysis.cpp `mold`.
///
/// Use when: pulling a stroke into shape by grabbing it anywhere, as in a vector editor
/// Tags: mold curve, drag curve, edit curve by point, bend through point
/// Node: planned (moldPath)
Path2D Path2D::moldedAt(float progress, Vector2 destination, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    if (measured.pieces().empty()) return *this;
    PathPoint2 place = measured.locate(progress);
    int pieceCount = static_cast<int>(measured.pieces().size());
    CubicBezier<Vector2> bent = measured.pieces()[static_cast<size_t>(place.pieceIndex)].cubic.molded(place.parameter, destination);
    Path2D leaving = withClosingLineDrawn(*this).withHandleMoved(place.pieceIndex, HandleSide::outgoing, bent.startHandle, metric);
    return leaving.withHandleMoved(arrivalNode(place.pieceIndex, pieceCount, closed), HandleSide::incoming, bent.endHandle, metric);
}

/// The place on the path nearest a point: its progress, the point and the distance.
///
/// How: `CubicBezier::nearest` over the pieces scaled by `metric` (so the distance is the one on
/// the page), then the point scaled back and the place turned into progress.
///
/// Conventions: y-up plane; the distance is in the metric. A path with no pieces gives its start.
///
/// Source: graphics_cpp curves/analysis.cpp `project`.
///
/// Use when: grabbing the stroke under a pointer; attaching a dot or a serif to the nearest place
/// Tags: project point onto path, nearest point, closest point on curve, snapping, hit test
/// Node: planned (projectOntoPath)
PathProjection Path2D::projection(Vector2 target, Vector2 metric) const {
    MeasuredPath2D measured(*this, metric);
    if (measured.pieces().empty()) return PathProjection{0, start, ((target - start) * metric).length()};
    std::vector<CubicBezier<Vector2>> curves;
    for (const OutlinePiece& piece : measured.pieces()) curves.push_back(curveInMetric(piece.cubic, metric));
    PathPoint2 nearest = CubicBezier<Vector2>::nearest(curves, target * metric, 64);
    return PathProjection{measured.progress(nearest.pieceIndex, nearest.parameter), nearest.point / metric, nearest.distance};
}

/// A function of progress giving how near each place of the path is to the nearest of some places,
/// along the path, eased by a falloff: 1 at a place, 0 from `reach` away.
///
/// How: for a progress u, the distance along the path to each place `a` is |u − a| × length (the
/// shorter way round for a closed path); with share x = distance / reach, the weight is
/// `falloffWeight(falloff, x)` for x < 1 and 0 otherwise; the answer is the largest weight.
///
///     weight 1 ┤      ╭─╮
///              │    ╭─╯ ╰─╮          places at the peak, easing to 0 at `reach`
///            0 ┴────╯     ╰────
///
/// Conventions: progress is a share of length and `reach` is in the units of `metric` (a path of
/// length 10 with reach 2 reaches 20% each way). A reach of 0 or less gives 0 everywhere.
///
/// Source: graphics_cpp curves/analysis.cpp `nearness`.
///
/// Use when: softening an edit around chosen places; weighting a brush along a stroke
/// Tags: soft selection, falloff along path, proximity weight, proportional editing
/// Node: planned (pathNearness)
std::function<float(float)> Path2D::nearnessTo(
    const std::vector<float>& progresses, float reach, Falloff falloff, Vector2 metric) const {
    float length = MeasuredPath2D(*this, metric).length();
    bool wrapsRound = closed;
    return [progresses, reach, falloff, length, wrapsRound](float progress) {
        float weight = 0;
        for (float place : progresses) {
            float gap = std::fabs(progress - place);
            if (wrapsRound) gap = std::min(gap, 1 - gap);
            float share = reach > 0 ? gap * length / reach : std::numeric_limits<float>::infinity();
            if (share < 1) weight = std::max(weight, falloffWeight(falloff, share));
        }
        return weight;
    };
}

} // namespace editor
