// Circle constructions (see CircleConstruction.h). Each works in double, so a crossing found from
// float inputs is the float nearest the true one, and a nearly tangent line is judged without
// float cancellation in r² − h².
#include "CircleConstruction.h"

#include <algorithm>
#include <cmath>

namespace editor {

namespace {

/// A point or direction of the plane in double, for the constructions' working.
struct WidePoint {
    double x = 0;
    double y = 0;

    WidePoint operator+(WidePoint other) const { return {x + other.x, y + other.y}; }
    WidePoint operator-(WidePoint other) const { return {x - other.x, y - other.y}; }
    WidePoint operator*(double scale) const { return {x * scale, y * scale}; }
    double dot(WidePoint other) const { return x * other.x + y * other.y; }
    double length() const { return std::hypot(x, y); }
    /// Turned 90° counterclockwise.
    WidePoint perpendicular() const { return {-y, x}; }
    Vector2 narrowed() const { return Vector2(static_cast<float>(x), static_cast<float>(y)); }
};

WidePoint widened(Vector2 point) { return {point.x, point.y}; }

/// The distance below which two lengths of a construction at `size` count as equal.
double touchingDistance(double size) { return touchingShare * std::max(size, 1.0); }

bool isUsableCircle(const Circle2& circle) {
    return circle.centre.isFinite() && std::isfinite(circle.radius) && circle.radius >= 0;
}

} // namespace

std::vector<Vector2> circleLineCrossings(const Circle2& circle, const Line2& line) {
    if (!isUsableCircle(circle) || !line.origin.isFinite() || !line.direction.isFinite()) return {};
    WidePoint direction = widened(line.direction);
    double directionLength = direction.length();
    if (!(directionLength > 0)) return {};
    WidePoint unitDirection = direction * (1 / directionLength);
    WidePoint centre = widened(circle.centre);
    WidePoint origin = widened(line.origin);
    WidePoint foot = origin + unitDirection * (centre - origin).dot(unitDirection);
    double footDistance = (centre - foot).length();
    double radius = circle.radius;
    double slack = touchingDistance(radius);
    if (footDistance > radius + slack) return {};
    if (std::fabs(footDistance - radius) <= slack) return {foot.narrowed()};
    double halfChord = std::sqrt(radius * radius - footDistance * footDistance);
    return {(foot - unitDirection * halfChord).narrowed(), (foot + unitDirection * halfChord).narrowed()};
}

std::vector<Vector2> circleCircleCrossings(const Circle2& first, const Circle2& second) {
    if (!isUsableCircle(first) || !isUsableCircle(second)) return {};
    WidePoint firstCentre = widened(first.centre);
    WidePoint between = widened(second.centre) - firstCentre;
    double centreDistance = between.length();
    double firstRadius = first.radius;
    double secondRadius = second.radius;
    double slack = touchingDistance(std::max(firstRadius, secondRadius));
    if (centreDistance <= slack) return {};
    WidePoint unitBetween = between * (1 / centreDistance);
    double alongBetween =
        (centreDistance * centreDistance + firstRadius * firstRadius - secondRadius * secondRadius) / (2 * centreDistance);
    double heightSquared = firstRadius * firstRadius - alongBetween * alongBetween;
    WidePoint middle = firstCentre + unitBetween * alongBetween;
    if (std::fabs(heightSquared) <= slack * slack) return {middle.narrowed()};
    if (heightSquared < 0) return {};
    WidePoint offset = unitBetween.perpendicular() * std::sqrt(heightSquared);
    return {(middle + offset).narrowed(), (middle - offset).narrowed()};
}

std::vector<Vector2> tangentPointsFrom(const Circle2& circle, Vector2 point) {
    if (!isUsableCircle(circle) || !point.isFinite()) return {};
    WidePoint centre = widened(circle.centre);
    WidePoint outward = widened(point) - centre;
    double distance = outward.length();
    double radius = circle.radius;
    double slack = touchingDistance(radius);
    if (std::fabs(distance - radius) <= slack) return {point};
    if (distance < radius) return {};
    WidePoint unitOutward = outward * (1 / distance);
    double cosine = radius / distance;
    double sine = std::sqrt(std::max(0.0, 1 - cosine * cosine));
    WidePoint along = unitOutward * (radius * cosine);
    WidePoint across = unitOutward.perpendicular() * (radius * sine);
    return {(centre + along + across).narrowed(), (centre + along - across).narrowed()};
}

std::vector<CircleTangent> commonTangents(const Circle2& first, const Circle2& second) {
    if (!isUsableCircle(first) || !isUsableCircle(second)) return {};
    WidePoint firstCentre = widened(first.centre);
    WidePoint secondCentre = widened(second.centre);
    WidePoint between = secondCentre - firstCentre;
    double centreDistance = between.length();
    double firstRadius = first.radius;
    double secondRadius = second.radius;
    if (centreDistance <= touchingDistance(std::max(firstRadius, secondRadius))) return {};
    WidePoint unitBetween = between * (1 / centreDistance);
    std::vector<CircleTangent> tangents;
    // One pair: the normals n with n·û = cosine, turned either way from û.
    auto addPair = [&](double cosine, bool crossesBetween) {
        if (std::fabs(cosine) > 1 + touchingShare) return;
        double sine = std::sqrt(std::max(0.0, 1 - cosine * cosine));
        double secondSide = crossesBetween ? 1 : -1;
        for (double turn : {1.0, -1.0}) {
            WidePoint normal = unitBetween * cosine + unitBetween.perpendicular() * (sine * turn);
            tangents.push_back(CircleTangent{(firstCentre - normal * firstRadius).narrowed(),
                                             (secondCentre + normal * (secondRadius * secondSide)).narrowed(),
                                             crossesBetween});
            if (sine <= touchingShare) return;
        }
    };
    addPair((secondRadius - firstRadius) / centreDistance, false);
    addPair(-(firstRadius + secondRadius) / centreDistance, true);
    return tangents;
}

} // namespace editor
