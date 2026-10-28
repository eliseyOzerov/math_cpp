// The rails of an ink's outline: its two edges, found, trimmed by the end planes and untangled (InkOutline.h).
#include <algorithm>
#include <cmath>
#include <limits>

#include "../Geometry/Line2.h"
#include "../Linear/MathProblem.h"
#include "InkOutline+Internal.h"

namespace editor::inkoutline {

namespace {

/// The rail point's direction is searched within this of square across the stroke.
Angle searchRange() { return Angle::degrees(80); }
/// ...by this many halvings.
constexpr int halvings = 24;

/// The area of a polygon, positive counterclockwise (the shoelace formula: half the sum of cross(pᵢ, pᵢ₊₁)). Zero
/// for fewer than three points.
float polygonArea(const std::vector<Vector2>& corners) {
    float twiceArea = 0;
    for (size_t index = 0; index < corners.size(); index++) twiceArea += corners[index].cross(corners[(index + 1) % corners.size()]);
    return twiceArea / 2;
}

/// The furthest point of a placed nib in the unit `direction`, on its exact rounded rectangle: each corner's ellipse arc
/// answers for the directions in its own quadrant. A side lying across the direction touches it all along: the middle
/// of the corners that tie stands for it.
///
/// How: in the nib's frame, an ellipse with radii (a, b) is furthest along l at c + (a²nx, b²ny)/hypot(a·nx, b·ny),
/// where n is l clamped into the corner's quadrant.
Vector2 support(const NibFrame& frame, Vector2 direction) {
    Vector2 local = direction.rotated(-frame.rotation);
    float tieDistance = 1e-6f * (frame.width / 2 + frame.height / 2 + 1);
    float best = -std::numeric_limits<float>::infinity();
    std::vector<Vector2> ties;
    for (const NibCornerArc& arc : Nib::cornerArcs(frame)) {
        float signX = arc.signs.x, signY = arc.signs.y, radiusX = arc.radii.x, radiusY = arc.radii.y;
        float inQuadrantX = signX * std::max(0.0f, signX * local.x), inQuadrantY = signY * std::max(0.0f, signY * local.y);
        float denominator = std::hypot(radiusX * inQuadrantX, radiusY * inQuadrantY);
        Vector2 point = arc.arcCentre;
        if (denominator > 1e-6f) {
            point = point + Vector2(radiusX * radiusX * inQuadrantX / denominator, radiusY * radiusY * inQuadrantY / denominator);
        } else if (radiusX > 1e-6f || radiusY > 1e-6f) {
            point = point + Vector2(radiusX * signX * (inQuadrantX != 0 ? 1.0f : 0.0f), radiusY * signY * (inQuadrantY != 0 ? 1.0f : 0.0f));
        }
        float reach = point.dot(local);
        if (reach > best + tieDistance) {
            best = reach;
            ties = {point};
        } else if (reach > best - tieDistance) {
            ties.push_back(point);
        }
    }
    return frame.centre + Vector2::mean(ties).rotated(frame.rotation);
}

/// The two rails. A rail point is where the stamps' envelope touches the stamp: the direction, near square across the
/// stroke, in which the nib reaches no further from one stamp to the next (found by halving). Turning or growing nibs
/// lean it off square. Round a sharp corner (stamps with headings) it is square across each heading in turn.
void railsOf(const InkResult& ink, Rail& left, Rail& right) {
    const auto& points = ink.points;
    const auto& frames = ink.frames;
    int count = static_cast<int>(points.size());
    auto reach = [](const NibFrame& frame, Vector2 direction) { return support(frame, direction).dot(direction); };
    for (int index = 0; index < count; index++) {
        std::optional<Angle> heading = index < static_cast<int>(ink.headings.size()) ? ink.headings[index] : std::nullopt;
        const NibFrame& frame = frames[index];
        int beforeIndex = index > 0 ? index - 1 : ink.closed ? count - 2 : 0;
        int afterIndex = index < count - 1 ? index + 1 : ink.closed ? 1 : count - 1;
        Vector2 along = heading ? Vector2::fromAngle(*heading) : (points[afterIndex] - points[beforeIndex]).normalized();
        const NibFrame& before = frames[beforeIndex];
        const NibFrame& after = frames[afterIndex];
        for (int side = 0; side < 2; side++) {
            Rail& rail = side == 0 ? left : right;
            float sign = side == 0 ? 1.0f : -1.0f;
            float base = std::atan2(sign * along.x, -sign * along.y);
            auto growth = [&](float angle) {
                Vector2 direction = Vector2::fromAngle(Angle::fromRadians(angle));
                return reach(after, direction) - reach(before, direction);
            };
            float low = base - searchRange().radians, high = base + searchRange().radians;
            float lowGrowth = growth(low), highGrowth = growth(high), angle = base;
            if (count > 1 && !heading && lowGrowth * highGrowth < 0) {
                for (int step = 0; step < halvings; step++) {
                    float middle = (low + high) / 2, middleGrowth = growth(middle);
                    if ((middleGrowth < 0) == (lowGrowth < 0)) {
                        low = middle;
                        lowGrowth = middleGrowth;
                    } else {
                        high = middle;
                    }
                }
                angle = (low + high) / 2;
            }
            rail.push_back({support(frame, Vector2::fromAngle(Angle::fromRadians(angle))), static_cast<float>(index)});
        }
    }
}

/// A rail trimmed by a plane at its own end: from where it crosses into the kept side. A rail that stops short of the
/// plane (a flat end with the nib turned) runs on along its last direction to meet it.
Rail clippedRail(const Rail& rail, const EndPlane& plane) {
    auto side = [&](const RailPoint& railPoint) { return plane.signedDistance(railPoint.point); };
    bool forwards = plane.end == InkEnd::start;
    Rail ordered = rail;
    if (!forwards) std::reverse(ordered.begin(), ordered.end());
    int keptFrom = -1;
    for (size_t index = 0; index < ordered.size(); index++) {
        if (side(ordered[index]) >= -planeSlack) {
            keptFrom = static_cast<int>(index);
            break;
        }
    }
    if (keptFrom < 0) return rail;
    Rail out;
    if (keptFrom > 0) {
        const RailPoint& outside = ordered[keptFrom - 1];
        const RailPoint& inside = ordered[keptFrom];
        float sideOutside = side(outside), sideInside = side(inside), amount = sideOutside / (sideOutside - sideInside);
        out.push_back({Vector2::linearBlend(outside.point, inside.point, amount), outside.at + (inside.at - outside.at) * amount});
        out.insert(out.end(), ordered.begin() + keptFrom, ordered.end());
    } else {
        const RailPoint& edge = ordered[0];
        const RailPoint* other = nullptr;
        for (const RailPoint& candidate : ordered) {
            if (candidate.point.distance(edge.point) > distinctDistance) {
                other = &candidate;
                break;
            }
        }
        std::optional<Vector2> back = other ? std::optional<Vector2>((edge.point - other->point).normalized()) : std::nullopt;
        float alongPlane = back ? back->dot(plane.normal) : 0;
        out = ordered;
        if (back && alongPlane < -1e-3f && side(edge) > distinctDistance) {
            float distance = side(edge) / -alongPlane;
            out.insert(out.begin(), {edge.point + *back * distance, edge.at + (forwards ? -1e-4f : 1e-4f)});
        }
    }
    if (!forwards) std::reverse(out.begin(), out.end());
    return out;
}

/// A rail without the loops a turn tighter than the nib makes on its inner side: the points of a loop all move to where
/// it crosses itself, so the stamp indices still find their places. A closed rail is a ring: of the two loops a crossing
/// makes, the shorter is the tangle. A tangle winds against the centreline under it (the rail runs backwards round the
/// inside of the turn); a loop that winds with it is the stroke's own (an e's bar running into its bowl), whose counter
/// must stay open, so it is kept. Shortest loops are collapsed first, so a tangle inside a kept loop is gone before the
/// loop round it is judged.
Rail untangledRail(const Rail& rail, const std::vector<Vector2>& points, bool closed) {
    Rail out = rail;
    auto crossingAt = [](Vector2 firstStart, Vector2 firstEnd, Vector2 secondStart, Vector2 secondEnd) -> std::optional<Vector2> {
        std::optional<LineMeeting> meeting =
            Line2{firstStart, firstEnd - firstStart}.meets(Line2{secondStart, secondEnd - secondStart}, 1e-6f, 1e-6f);
        return meeting ? std::optional<Vector2>(Vector2::linearBlend(firstStart, firstEnd, meeting->alongFirst)) : std::nullopt;
    };
    int count = static_cast<int>(out.size());
    for (int span = 2; span < count - 1; span++) {
        if (closed && 2 * span > count) break;
        for (int first = 0; first + span + 1 < count; first++) {
            int last = first + span;
            std::optional<Vector2> crossing = crossingAt(out[first].point, out[first + 1].point, out[last].point, out[last + 1].point);
            if (!crossing) continue;
            std::vector<Vector2> loop = {*crossing};
            for (int index = first + 1; index <= last; index++) loop.push_back(out[index].point);
            size_t lowest = static_cast<size_t>(std::floor(out[first + 1].at));
            size_t highest = std::min(points.size(), static_cast<size_t>(std::ceil(out[last].at)) + 1);
            std::vector<Vector2> under;
            for (size_t index = lowest; index < highest; index++) under.push_back(points[index]);
            float underArea = polygonArea(under);
            if (std::fabs(underArea) > 1e-3f && underArea * polygonArea(loop) > 0) continue;
            for (int index = first + 1; index <= last; index++) out[index].point = *crossing;
        }
    }
    return out;
}

} // namespace

float turnBetween(Vector2 first, Vector2 second) { return std::fabs(Angle::signedTurn(first, second).radians); }

RailPair outlineParts(const InkResult& ink) {
    if (ink.points.empty() || ink.frames.size() != ink.points.size()) throw MathProblem("An ink with no points or no frames has no outline.");
    RailPair parts;
    railsOf(ink, parts.left, parts.right);
    for (const EndPlane& plane : ink.planes) {
        parts.left = clippedRail(parts.left, plane);
        parts.right = clippedRail(parts.right, plane);
    }
    parts.left = untangledRail(parts.left, ink.points, ink.closed);
    parts.right = untangledRail(parts.right, ink.points, ink.closed);
    for (const EndPlane& plane : ink.planes) (plane.end == InkEnd::start ? parts.trimmedStart : parts.trimmedEnd) = true;
    return parts;
}

int firstAt(const Rail& rail, float at, bool inclusive) {
    int low = 0, high = static_cast<int>(rail.size());
    while (low < high) {
        int middle = (low + high) >> 1;
        if (inclusive ? rail[middle].at >= at : rail[middle].at > at) high = middle;
        else low = middle + 1;
    }
    return low;
}

Vector2 railAt(const Rail& rail, float at) {
    if (at <= rail[0].at) return rail[0].point;
    if (at > rail.back().at) return rail.back().point;
    int index = firstAt(rail, at, true);
    const RailPoint& before = rail[index - 1];
    const RailPoint& after = rail[index];
    return Vector2::linearBlend(before.point, after.point, after.at > before.at ? (at - before.at) / (after.at - before.at) : 1);
}

Vector2 railDirection(const Rail& rail, float at, float side) {
    float first = rail[0].at, last = rail.back().at;
    auto pointAt = [&](float place) { return railAt(rail, std::max(first, std::min(last, place))); };
    for (float step : {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f}) {
        // A parabola through three samples, parametrized by the distance along them (the stamps' spacing changes at the
        // nodes and the run-ons) and differentiated at the first: with distances 0, d1, d2 the weights are
        // −(d1 + d2)/(d1·d2), d2/(d1·(d2 − d1)) and −d1/(d2·(d2 − d1)) (three-point Lagrange derivative).
        Vector2 p0 = pointAt(at), p1 = pointAt(at + side * step), p2 = pointAt(at + side * 2 * step);
        float d1 = p0.distance(p1), d2 = d1 + p1.distance(p2);
        if (d1 < distinctDistance || d2 - d1 < distinctDistance) continue;
        float weight0 = -(1 / d1 + 1 / d2), weight1 = d2 / (d1 * (d2 - d1)), weight2 = -d1 / (d2 * (d2 - d1));
        Vector2 velocity = (p0 * weight0 + p1 * weight1 + p2 * weight2) * side;
        if (velocity.length() > 1e-6f) return velocity.normalized();
    }
    Vector2 near = pointAt(at + side), here = pointAt(at);
    return near.distance(here) > distinctDistance ? ((near - here) * side).normalized() : Vector2(0, 0);
}

} // namespace editor::inkoutline
