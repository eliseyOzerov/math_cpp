// See Shapes2D.h. Ported from graphics_cpp's shape.cpp.
#include "Shapes2D.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/NumberText.h"

namespace editor {

namespace {

/// The most a corner may turn once its sides are bowed: short of turning straight back, where the
/// sides would cross (11/12 of a half turn).
constexpr float largestCornerTurn = Angle::pi * 11 / 12;

/// A side piece or corner smaller than this (a share of a side's length, or a length) is not drawn: in floats the
/// ends of a side cut away whole differ from its length by rounding, and would leave a speck.
constexpr float smallestPiece = 1e-5f;

/// A point as "x,y", for a problem message.
std::string pointText(Vector2 point) { return numberText(point.x) + "," + numberText(point.y); }

/// A cubic with its handles at the thirds of the line from `start` to `end`.
CubicBezier<Vector2> straightCubic(Vector2 start, Vector2 end) {
    return CubicBezier<Vector2>(
        start, Vector2::linearBlend(start, end, 1.0f / 3), Vector2::linearBlend(start, end, 2.0f / 3), end);
}

/// The cubic with every control point moved from the metric's units back to plain coordinates.
CubicBezier<Vector2> fromMetric(const CubicBezier<Vector2>& cubic, Vector2 metric) {
    return CubicBezier<Vector2>(
        cubic.start.fromMetric(metric), cubic.startHandle.fromMetric(metric), cubic.endHandle.fromMetric(metric),
        cubic.end.fromMetric(metric));
}

} // namespace

CubicBezier<Vector2> Shapes2D::bowed(Vector2 start, Vector2 end, float bulge) {
    Vector2 normal = (end - start).perpendicular();
    float push = bulge / 0.75f; // both handles moved push·normal put the middle 0.75·push·normal out
    Vector2 firstThird = Vector2::linearBlend(start, end, 1.0f / 3);
    Vector2 secondThird = Vector2::linearBlend(start, end, 2.0f / 3);
    return CubicBezier<Vector2>(start, firstThird + normal * push, secondThird + normal * push, end);
}

std::vector<float> Shapes2D::fitBulges(const std::vector<Vector2>& points, const std::vector<float>& bulges) {
    size_t sideCount = points.empty() ? 0 : points.size() - 1;
    if (bulges.size() != sideCount) throw MathProblem("fitBulges needs one bulge per side of the polyline.");
    std::vector<float> scale(bulges.size(), 1.0f);
    for (size_t index = 1; index + 1 < points.size(); index++) {
        float turn = Angle::signedTurn(points[index] - points[index - 1], points[index + 1] - points[index]).radians;
        auto turned = [&](float share) {
            return std::fabs(turn + std::atan(4 * share * bulges[index - 1]) + std::atan(4 * share * bulges[index]));
        };
        if (turned(1) <= largestCornerTurn) continue;
        float low = 0, high = 1;
        for (int step = 0; step < 40; step++) {
            float middle = (low + high) / 2;
            if (turned(middle) <= largestCornerTurn) low = middle;
            else high = middle;
        }
        scale[index - 1] = std::min(scale[index - 1], low);
        scale[index] = std::min(scale[index], low);
    }
    std::vector<float> result;
    for (size_t index = 0; index < bulges.size(); index++) result.push_back(bulges[index] * scale[index]);
    return result;
}

std::vector<CubicBezier<Vector2>> Shapes2D::softPolyline(const std::vector<Vector2>& points, const Softness& softness) {
    const int count = int(points.size());
    if (count < 2) return {};
    std::vector<float> given;
    for (int side = 0; side < count - 1; side++) {
        if (softness.sideBulges.empty()) given.push_back(softness.bulge);
        else given.push_back(size_t(side) < softness.sideBulges.size() ? softness.sideBulges[side] : 0.0f);
    }
    std::vector<float> bulges = fitBulges(points, given);
    std::vector<CubicBezier<Vector2>> sides;
    std::vector<float> lengths;
    for (int side = 0; side < count - 1; side++) {
        sides.push_back(bowed(points[side], points[side + 1], bulges[side]));
        lengths.push_back(points[side].distance(points[side + 1]));
    }
    // The room each inner corner takes from its sides: half a side, or all of one that ends the polyline.
    auto room = [&](int side) { return side == 0 || side == count - 2 ? lengths[side] : lengths[side] / 2; };
    float radius = std::clamp(softness.radius, 0.0f, 1.0f);
    std::vector<float> cut;
    for (int node = 0; node < count; node++)
        cut.push_back(node == 0 || node == count - 1 ? 0.0f : radius * std::min(room(node - 1), room(node)));
    auto lengthOrOne = [](float length) { return length > 0 ? length : 1.0f; };
    std::vector<CubicBezier<Vector2>> result;
    for (int side = 0; side < count - 1; side++) {
        const CubicBezier<Vector2>& current = sides[side];
        float lower = cut[side] / lengthOrOne(lengths[side]);
        float upper = 1 - cut[side + 1] / lengthOrOne(lengths[side]);
        if (side > 0 && cut[side] > smallestPiece) {
            // The corner at points[side], from where the last side was cut to where this one starts.
            const CubicBezier<Vector2>& previous = sides[side - 1];
            float previousEnd = 1 - cut[side] / lengthOrOne(lengths[side - 1]);
            result.push_back(CubicBezier<Vector2>::arcCubic(
                previous.point(previousEnd), previous.firstDerivative(previousEnd), current.point(lower),
                current.firstDerivative(lower)));
        }
        if (upper - lower > smallestPiece) result.push_back(current.portion(lower, upper));
    }
    return result;
}

Path2D Shapes2D::soft(const std::vector<Vector2>& points, const Softness& softness, Vector2 metric) {
    if (points.size() < 2) throw MathProblem("A soft polyline needs at least two points.");
    std::vector<Vector2> inMetric;
    for (Vector2 point : points) inMetric.push_back(point.toMetric(metric));
    std::vector<CubicBezier<Vector2>> cubics = softPolyline(inMetric, softness);
    for (CubicBezier<Vector2>& cubic : cubics) cubic = fromMetric(cubic, metric);
    return pathOfCubics(cubics, false);
}

Path2D Shapes2D::oval(float firstX, float firstY, float secondX, float secondY, const OvalSettings& settings) {
    float centreX = (firstX + secondX) / 2, centreY = (firstY + secondY) / 2;
    const std::array<Vector2, 4> at{Vector2(centreX, firstY), Vector2(secondX, centreY), Vector2(centreX, secondY), Vector2(firstX, centreY)};
    int first = int(settings.start);
    // Quarters alternate between leaving horizontally and vertically, starting with the axis
    // the first one leaves along (set explicitly: a flat oval's arrivals are not told apart by the path).
    bool horizontal = settings.start == OvalStart::bottom || settings.start == OvalStart::top;
    Path2D path(at[first]);
    for (int quarter = 0; quarter < 4; quarter++) {
        path.addQuarter(at[(first + quarter + 1) % 4], horizontal ? Heading::horizontal : Heading::vertical,
                        settings.tension);
        horizontal = !horizontal;
    }
    path.close();
    return path;
}

Path2D Shapes2D::lobe(Vector2 start, Vector2 end, const LobeSettings& settings, Vector2 metric) {
    // Worked in the metric's units, in a frame of the axis (along) and outward from it (out), and back.
    Vector2 metricStart = start.toMetric(metric), metricEnd = end.toMetric(metric);
    float axisAngle = settings.axis ? settings.axis->radians : std::atan2(metricStart.y - metricEnd.y, metricStart.x - metricEnd.x);
    Vector2 along(std::cos(axisAngle), std::sin(axisAngle));
    Vector2 out(along.y, -along.x);
    // (out, along) coordinates of a point, and back.
    auto local = [&](Vector2 point) { return Vector2(point.dot(out), point.dot(along)); };
    auto world = [&](float outward, float position) { return out * outward + along * position; };
    Vector2 localStart = local(metricStart), localEnd = local(metricEnd);
    float base = std::min(localStart.x, localEnd.x);
    float far = (settings.reachFrom == LobeReach::fromStart ? localStart.x : base) + settings.reach * metric.x;
    if (!(far > base) || !(localStart.y > localEnd.y))
        throw MathProblem("a lobe needs room: reach " + numberText(settings.reach) + ", from " + pointText(start) +
                          " round to " + pointText(end));
    int cornerCount = int(std::max(1.0f, std::floor(settings.nodes + 0.5f)));
    float middle = (localStart.y + localEnd.y) / 2, halfSpan = (localStart.y - localEnd.y) / 2;
    std::vector<Vector2> corners{metricStart};
    for (int index = 0; index < cornerCount; index++) {
        // A ray from the box's middle on the line, spread evenly from along the axis round to against it.
        float rayAngle = Angle::pi / 2 - Angle::pi * (index + 0.5f) / cornerCount;
        float rayCosine = std::cos(rayAngle), raySine = std::sin(rayAngle);
        float toBox = std::max(std::fabs(rayCosine), std::fabs(raySine));
        corners.push_back(world(base + (far - base) * rayCosine / toBox, middle + halfSpan * raySine / toBox));
    }
    corners.push_back(metricEnd);
    // Running clockwise (out along the top, round, back along the bottom), outward is the left of
    // travel. Sides lying across the axis at either end stay straight, so the ink keeps touching its line.
    Softness softness;
    softness.radius = settings.radius;
    for (size_t index = 1; index < corners.size(); index++) {
        bool acrossAxis = std::fabs(local(corners[index]).y - local(corners[index - 1]).y) < 1e-6f;
        softness.sideBulges.push_back(acrossAxis ? 0.0f : settings.bulge);
    }
    std::vector<CubicBezier<Vector2>> cubics = softPolyline(corners, softness);
    for (CubicBezier<Vector2>& cubic : cubics) cubic = fromMetric(cubic, metric);
    return pathOfCubics(cubics, false);
}

Path2D Shapes2D::bend(const Path2D& path, float amount, Vector2 from) {
    if (amount == 0) return path;
    std::vector<CubicBezier<Vector2>> cubics;
    for (const CubicBezier<Vector2>& cubic : cubicsOf(path)) {
        Vector2 chord = cubic.end - cubic.start, normal = chord.perpendicular();
        Vector2 middle = Vector2::linearBlend(cubic.start, cubic.end, 0.5f);
        if (chord.length() < 1e-6f) {
            cubics.push_back(cubic);
            continue;
        }
        float push = (normal.dot(middle - from) < 0 ? -amount : amount) / 0.75f;
        cubics.push_back(CubicBezier<Vector2>(
            cubic.start, cubic.startHandle + normal * push, cubic.endHandle + normal * push, cubic.end));
    }
    Path2D result = cubics.empty() ? Path2D(path.start) : pathOfCubics(cubics, path.isClosed());
    for (int node = 0; node <= int(path.segments().size()) && node <= int(cubics.size()); node++)
        if (path.nodeKind(node)) result.setNodeKind(node, path.nodeKind(node));
    return result;
}

std::vector<CubicBezier<Vector2>> Shapes2D::knob(Vector2 point, Vector2 direction, float half, const KnobShape& shape) {
    float radius = shape.size * half, alongRadius = shape.length * radius;
    float sign = shape.side > 0 ? 1.0f : shape.side < 0 ? -1.0f : 0.0f;
    float offset = sign * (radius - half);
    Vector2 normal = direction.perpendicular();
    Vector2 centre = point + normal * offset;
    auto placed = [&](Vector2 local) { return centre + direction * local.x + normal * local.y; };
    std::vector<CubicBezier<Vector2>> cubics;
    for (const CubicBezier<Vector2>& piece : cubicsOf(oval(-alongRadius, -radius, alongRadius, radius)))
        cubics.push_back(CubicBezier<Vector2>(
            placed(piece.start), placed(piece.startHandle), placed(piece.endHandle), placed(piece.end)));
    return cubics;
}

CubicBezier<Vector2> Shapes2D::runOn(Vector2 point, Vector2 direction, float length, Angle turn) {
    float theta = turn.radians;
    Vector2 normal = direction.perpendicular();
    auto at = [&](float x, float y) { return point + direction * x + normal * y; };
    if (std::fabs(theta) < 1e-6f)
        return CubicBezier<Vector2>(point, at(length / 3, 0), at(2 * length / 3, 0), at(length, 0));
    float radius = length / theta;
    float handle = (4.0f / 3) * std::tan(theta / 4) * radius;
    float endX = radius * std::sin(theta), endY = radius * (1 - std::cos(theta));
    return CubicBezier<Vector2>(
        point, at(handle, 0), at(endX - handle * std::cos(theta), endY - handle * std::sin(theta)), at(endX, endY));
}

Path2D Shapes2D::pathOfCubics(const std::vector<CubicBezier<Vector2>>& cubics, bool closed) {
    Path2D path(cubics.empty() ? Vector2() : cubics.front().start);
    for (const CubicBezier<Vector2>& cubic : cubics) path.addCubic(cubic.startHandle, cubic.endHandle, cubic.end);
    if (closed) path.close();
    return path;
}

std::vector<CubicBezier<Vector2>> Shapes2D::cubicsOf(const Path2D& path) {
    std::vector<CubicBezier<Vector2>> cubics;
    std::vector<Path2D::Edge> edges = path.isClosed() ? path.edgesWithClosingLine() : path.edges();
    for (const Path2D::Edge& edge : edges)
        cubics.push_back(edge.cubic() ? *edge.cubic() : straightCubic(edge.start, edge.segment.end()));
    return cubics;
}

} // namespace editor
