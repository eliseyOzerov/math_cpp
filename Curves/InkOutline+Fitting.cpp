// Fitting the rails and the end caps of an ink's outline to cubics (InkOutline.h).
#include <algorithm>
#include <cmath>
#include <limits>

#include "../Geometry/Polyline2.h"
#include "../Geometry/Segment2.h"
#include "InkOutline+Internal.h"

namespace editor::inkoutline {

namespace {

/// An untrimmed end's stamp is sampled with this many points per corner, finer than the ink's, as the cap is fitted to
/// it directly.
constexpr int capPointsPerCorner = 24;
/// The cap leaves smoothly along the rail where the two differ by less than this many times `Nib::cornerTurn`. A stamp
/// turning with the stroke meets its rails at a slight angle that a smooth join should still take.
constexpr float capJoinFactor = 1.5f;

const std::optional<int>& planEnd(const InkOutlinePlan& plan, InkEnd end) { return end == InkEnd::start ? plan.start : plan.end; }

/// A rail's anchors as stamp indices, by the plan (pieces per skeleton span), each with its span: span k is cut into
/// plan[k] equal steps of stamp index between its nodes, and the last anchor is the rail's end.
struct Anchors {
    std::vector<float> places;
    std::vector<int> spans;
};

Anchors railAnchors(const Rail& rail, const std::vector<int>& nodes, const std::vector<int>& plan) {
    float end = rail.back().at;
    int spanCount = static_cast<int>(nodes.size()) - 1;
    Anchors anchors;
    anchors.places = {rail[0].at};
    for (int span = 0; span < spanCount; span++) {
        for (int piece = 1; piece <= plan[span]; piece++) {
            float place = span == spanCount - 1 && piece == plan[span]
                              ? end
                              : static_cast<float>(nodes[span]) + static_cast<float>(nodes[span + 1] - nodes[span]) * static_cast<float>(piece) / static_cast<float>(plan[span]);
            anchors.places.push_back(std::min(end, std::max(anchors.places.back(), place)));
            anchors.spans.push_back(span);
        }
    }
    return anchors;
}

Vector2 middleOf(Vector2 first, Vector2 second) { return (first + second).normalized(); }

/// Whether every point of a run lies on the straight line from its first point to its last.
bool isStraight(const std::vector<Vector2>& run) {
    Segment2 chord{run[0], run.back()};
    for (Vector2 point : run) {
        if (!chord.isNear(point, 1e-3f, std::numeric_limits<float>::infinity())) return false;
    }
    return true;
}

/// A cap's piece leaves smoothly into the rail when the stamp turns with it, a corner otherwise.
Vector2 joinDirection(Vector2 railDirection, Vector2 ownDirection) {
    return turnBetween(railDirection, ownDirection) < Nib::cornerTurn().radians * capJoinFactor ? railDirection : ownDirection;
}

} // namespace

std::pair<Vector2, Vector2> endTangents(const Piece& piece) {
    if (piece.isLine) {
        Vector2 direction = (piece.cubic.end - piece.cubic.start).normalized();
        return {direction, direction};
    }
    auto firstNonZero = [](std::initializer_list<Vector2> candidates) {
        for (Vector2 candidate : candidates) {
            Vector2 unit = candidate.normalized();
            if (unit.x != 0 || unit.y != 0) return unit;
        }
        return Vector2(0, 0);
    };
    const CubicBezier<Vector2>& c = piece.cubic;
    return {firstNonZero({c.startHandle - c.start, c.endHandle - c.start, c.end - c.start}),
            firstNonZero({c.end - c.endHandle, c.end - c.startHandle, c.end - c.start})};
}

RailFit fitRail(const Rail& rail, const InkResult& ink, const std::vector<int>& plan) {
    Anchors anchors = railAnchors(rail, ink.nodes, plan);
    const auto& places = anchors.places;
    int last = static_cast<int>(places.size()) - 1;
    // An anchor whose two sides agree on the rail's direction is smooth and both share it; otherwise it is a corner.
    struct Directions {
        std::optional<Vector2> leaving, arriving;
    };
    std::vector<Directions> directions;
    for (int index = 0; index <= last; index++) {
        std::optional<Vector2> leaving, arriving;
        if (index < last) leaving = railDirection(rail, places[index], 1);
        if (index > 0) arriving = railDirection(rail, places[index], -1);
        if (leaving && arriving && turnBetween(*leaving, *arriving) < Nib::cornerTurn().radians) {
            Vector2 shared = middleOf(*leaving, *arriving);
            directions.push_back({shared, shared});
        } else {
            directions.push_back({leaving, arriving});
        }
    }
    RailFit fit;
    fit.errors.assign(plan.size(), 0);
    for (int index = 0; index < last; index++) {
        float from = places[index], to = places[index + 1];
        std::vector<Vector2> samples = {railAt(rail, from)};
        for (int sample = firstAt(rail, from, false); sample < static_cast<int>(rail.size()) && rail[sample].at < to; sample++) samples.push_back(rail[sample].point);
        samples.push_back(railAt(rail, to));
        FittedCubic piece = CubicBezier<Vector2>::fittedSinglePiece(samples, *directions[index].leaving, -*directions[index + 1].arriving);
        fit.cubics.push_back(piece.cubic);
        fit.errors[anchors.spans[index]] = std::max(fit.errors[anchors.spans[index]], piece.largestError);
    }
    return fit;
}

/// An untrimmed end: the end stamp's far side from one rail's end to the other's, in `pieces` cubics of equal length
/// per rounded run (a straight run is one line). Where the stamp meets a rail without a corner, the cap leaves along the
/// rail's own direction.
///
/// How: the stamp polygon (24 points a corner) is walked clockwise from the point nearest `from` to the one nearest
/// `to`, keeping the points ahead of the stroke's direction; the walk is split at the nib's sharp corners; each run
/// is a line if straight, else cut into equal-length pieces fitted with tangents by central differences.
Cap fitCap(const InkResult& ink, InkEnd end, Vector2 from, Vector2 to, Vector2 tangentFrom, Vector2 tangentTo, int pieces) {
    int count = static_cast<int>(ink.points.size()), stampIndex = end == InkEnd::end ? count - 1 : 0;
    const NibFrame& frame = ink.frames[stampIndex];
    std::vector<Vector2> polygon = Nib::stampPolygon(frame, frame.rotation, capPointsPerCorner);
    for (Vector2& corner : polygon) corner = corner + frame.centre;
    int polygonCount = static_cast<int>(polygon.size());
    Vector2 along = end == InkEnd::end ? (ink.points[count - 1] - ink.points[std::max(0, count - 2)]).normalized()
                                       : (ink.points[std::min(count - 1, 1)] - ink.points[0]).normalized();
    Vector2 outward = end == InkEnd::end ? along : -along;
    auto isAhead = [&](Vector2 candidate, Vector2 origin) { return (candidate - origin).dot(outward) > distinctDistance; };
    auto nearestCorner = [&](Vector2 target) {
        int best = 0;
        for (int corner = 0; corner < polygonCount; corner++) {
            if (polygon[corner].distance(target) < polygon[best].distance(target)) best = corner;
        }
        return best;
    };
    // Clockwise round the stamp (its polygon runs the other way), over its far side.
    int firstCorner = nearestCorner(from), lastCorner = nearestCorner(to);
    std::vector<Vector2> arc = {from};
    if (isAhead(polygon[firstCorner], from)) arc.push_back(polygon[firstCorner]);
    for (int corner = (firstCorner - 1 + polygonCount) % polygonCount, walked = 0; corner != lastCorner && walked < polygonCount;
         corner = (corner - 1 + polygonCount) % polygonCount, walked++) {
        arc.push_back(polygon[corner]);
    }
    if (lastCorner != firstCorner && isAhead(polygon[lastCorner], to)) arc.push_back(polygon[lastCorner]);
    arc.push_back(to);
    // Runs between the nib's sharp corners: a straight one is a line, a rounded one is `pieces` cubics.
    float sharp = Nib::cornerTurn().radians;
    std::vector<std::vector<Vector2>> split = {{arc[0]}};
    for (size_t index = 1; index < arc.size(); index++) {
        Vector2 point = arc[index];
        std::vector<Vector2>& run = split.back();
        if (point.distance(run.back()) < distinctDistance) continue;
        run.push_back(point);
        if (index + 1 < arc.size()) {
            Vector2 next = arc[index + 1];
            if (next.distance(point) > distinctDistance && turnBetween((point - run[run.size() - 2]).normalized(), (next - point).normalized()) > sharp) split.push_back({point});
        }
    }
    std::vector<std::vector<Vector2>> runs;
    for (const auto& run : split) {
        if (run.size() > 1) runs.push_back(run);
    }
    Cap cap;
    for (size_t runIndex = 0; runIndex < runs.size(); runIndex++) {
        const std::vector<Vector2>& run = runs[runIndex];
        if (isStraight(run)) {
            cap.pieces.push_back(Piece::line(run[0], run.back()));
            cap.runs += 'l';
            continue;
        }
        cap.runs += 'c';
        Polyline2 polyline{run};
        std::vector<float> lengths = polyline.runLengths();
        float total = lengths.back();
        auto pointAt = [&](float distance) { return polyline.pointAlong(lengths, distance); };
        // Tangents by central differences, a quarter piece wide (at most a sixteenth of the run): wide enough to step over
        // the stamp's sampling, narrow enough to stay on one side of a rounded corner.
        float step = total / static_cast<float>(pieces), half = std::min(step / 4, total / 16);
        auto tangentAt = [&](float distance) {
            if (distance <= 0) return (pointAt(half) - pointAt(0)).normalized();
            if (distance >= total) return (pointAt(total) - pointAt(total - half)).normalized();
            return (pointAt(distance + half) - pointAt(distance - half)).normalized();
        };
        for (int piece = 0; piece < pieces; piece++) {
            float startDistance = static_cast<float>(piece) * step, endDistance = static_cast<float>(piece + 1) * step;
            std::vector<Vector2> samples = {pointAt(startDistance)};
            for (size_t sample = 0; sample < lengths.size(); sample++) {
                if (lengths[sample] > startDistance && lengths[sample] < endDistance) samples.push_back(run[sample]);
            }
            samples.push_back(pointAt(endDistance));
            Vector2 leaving = piece == 0 && runIndex == 0 ? joinDirection(tangentFrom, tangentAt(0)) : tangentAt(startDistance);
            Vector2 arriving = piece == pieces - 1 && runIndex == runs.size() - 1 ? joinDirection(tangentTo, tangentAt(total)) : tangentAt(endDistance);
            FittedCubic fitted = CubicBezier<Vector2>::fittedSinglePiece(samples, leaving, -arriving);
            cap.pieces.push_back(Piece::curve(fitted.cubic));
            cap.error = std::max(cap.error, fitted.largestError);
        }
    }
    if (cap.pieces.empty()) cap.pieces.push_back(Piece::line(from, to));
    return cap;
}

Fit fitOutline(const InkResult& ink, const InkOutlinePlan& plan, const RailPair& parts) {
    const Rail& left = parts.left;
    const Rail& right = parts.right;
    RailFit leftFit = fitRail(left, ink, plan.left), rightFit = fitRail(right, ink, plan.right);
    auto piecesOf = [](const std::vector<CubicBezier<Vector2>>& cubics) {
        std::vector<Piece> pieces;
        for (const CubicBezier<Vector2>& cubic : cubics) pieces.push_back(Piece::curve(cubic));
        return pieces;
    };
    auto reversedPiecesOf = [](const std::vector<CubicBezier<Vector2>>& cubics) {
        std::vector<Piece> pieces;
        for (size_t index = cubics.size(); index-- > 0;) pieces.push_back(Piece::curve(cubics[index].reversed()));
        return pieces;
    };
    if (ink.closed) {
        // The last piece ends exactly where the first starts.
        auto ring = [](std::vector<Piece> pieces) {
            pieces.back().cubic.end = pieces[0].cubic.start;
            return pieces;
        };
        std::vector<Piece> leftRing = ring(piecesOf(leftFit.cubics)), rightRing = ring(reversedPiecesOf(rightFit.cubics));
        return {leftRing, {leftRing, rightRing}, {leftFit.errors, rightFit.errors, 0, 0}, {"l", "l"}};
    }
    auto tangentOf = [](const std::vector<CubicBezier<Vector2>>& cubics, InkEnd end) {
        return end == InkEnd::start ? endTangents(Piece::curve(cubics.front())).first : endTangents(Piece::curve(cubics.back())).second;
    };
    Cap ends[2];
    for (InkEnd end : {InkEnd::start, InkEnd::end}) {
        Vector2 leftPoint = end == InkEnd::start ? left[0].point : left.back().point;
        Vector2 rightPoint = end == InkEnd::start ? right[0].point : right.back().point;
        const std::optional<int>& endPieces = planEnd(plan, end);
        int slot = end == InkEnd::start ? 0 : 1;
        if (!endPieces) {
            ends[slot] = {{end == InkEnd::end ? Piece::line(leftPoint, rightPoint) : Piece::line(rightPoint, leftPoint)}, 0, "l"};
            continue;
        }
        Vector2 leftTangent = tangentOf(leftFit.cubics, end), rightTangent = tangentOf(rightFit.cubics, end);
        ends[slot] = end == InkEnd::end ? fitCap(ink, InkEnd::end, leftPoint, rightPoint, leftTangent, -rightTangent, *endPieces)
                                        : fitCap(ink, InkEnd::start, rightPoint, leftPoint, -rightTangent, leftTangent, *endPieces);
    }
    std::vector<Piece> contour = piecesOf(leftFit.cubics);
    contour.insert(contour.end(), ends[1].pieces.begin(), ends[1].pieces.end());
    std::vector<Piece> backwards = reversedPiecesOf(rightFit.cubics);
    contour.insert(contour.end(), backwards.begin(), backwards.end());
    contour.insert(contour.end(), ends[0].pieces.begin(), ends[0].pieces.end());
    return {contour, {contour}, {leftFit.errors, rightFit.errors, ends[0].error, ends[1].error}, {ends[0].runs, ends[1].runs}};
}

} // namespace editor::inkoutline
