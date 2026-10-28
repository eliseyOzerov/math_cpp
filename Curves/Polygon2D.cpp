// Polygon2D: sides, box, winding number and signed distance (Engine/Math/Paths/Polygon2D.swift).
#include "Polygon2D.h"

#include <limits>
#include <string>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

Polygon2D::Polygon2D(std::vector<Vector2> corners) {
    if (corners.size() > 1 && corners.front() == corners.back()) corners.pop_back();
    if (!(corners.size() >= 3)) {
        throw MathProblem("A polygon needs at least 3 corners, but got " + std::to_string(corners.size()) + ".");
    }
    cornerList = std::move(corners);
}

/// The sides, each from a corner to the next, the last back to the first.
///
/// Use when: walking the outline side by side
/// Tags: polygon sides, edges, segments of polygon, outline sides
/// Node: inside extrudedOutlineField
std::vector<Segment2> Polygon2D::sides() const {
    std::vector<Segment2> result;
    for (size_t index = 0; index < cornerList.size(); index += 1) {
        result.push_back(Segment2{cornerList[index], cornerList[(index + 1) % cornerList.size()]});
    }
    return result;
}

/// The smallest box holding every corner.
///
/// Use when: the extent of a flat shape; the box of a plate extruded from it
/// Tags: polygon bounds, bounding rectangle, extent, AABB 2D
/// Node: none (2D outline bounds; boundingBox measures 3D geometry)
Bounds2 Polygon2D::bounds() const { return Bounds2::enclosing(cornerList); }

/// How many times the outline winds around `point`: +1 for each counterclockwise loop, −1 for
/// each clockwise one, 0 outside.
///
/// Source: Sunday, *Inclusion of a point in a polygon* (2001).
///
/// How: follow the ray from `point` toward +x. A side going upward that crosses it (the
/// point is left of the side) adds 1; one going downward (the point is right of it)
/// takes 1 away. "Left of `a → b`" is the sign of the cross product `(b − a) × (point − a)`.
/// A side counts as crossing when one end is at or below the ray's height and the other
/// above it (half-open), so a corner exactly at the ray's height is counted once, not twice.
///
///        b ●              upward side a → b, point on its left: +1
///         ╱
///     ○──╱────────▶ ray
///   point╱
///     a ●
///
/// Conventions: a point exactly on a side may count as inside or outside.
///
/// Use when: telling inside from outside for a flat outline; the sign of its distance
/// Tags: winding number, point in polygon, inside test, non-zero rule, crossing number
/// Node: inside extrudedOutlineField
int Polygon2D::windingNumber(Vector2 point) const {
    int total = 0;
    for (const Segment2& side : sides()) {
        float isLeft = (side.end - side.start).cross(point - side.start);
        if (side.start.y <= point.y) {
            total = side.end.y > point.y && isLeft > 0 ? total + 1 : total;
        } else {
            total = side.end.y <= point.y && isLeft < 0 ? total - 1 : total;
        }
    }
    return total;
}

/// The signed distance from `point` to the outline: the distance to the nearest side,
/// negative inside (winding number not 0, the non-zero rule), positive outside.
///
/// Source: Quílez, *sdPolygon* (iquilezles.org/articles/distfunctions2d), with the sign from
/// `windingNumber`.
///
/// How: `±min over sides of Segment2::distance(point)`.
///
/// Exact for a simple polygon (one that does not cross itself): the nearest boundary point
/// lies on some side. For a self-crossing one the inside follows the non-zero rule and the
/// value is a bound: a side that runs through the inside still counts as near, so the value
/// can only be smaller than the true distance, never larger. Either way it is 1-Lipschitz
/// everywhere: the sign only flips where the distance is 0.
///
/// Use when: a 2D shape's distance for extruding or revolving it; testing how far a point is
///   inside a flat outline
/// Tags: sdPolygon, polygon sdf, 2D signed distance, distance to outline, inside outside
/// Node: inside extrudedOutlineField
float Polygon2D::signedDistance(Vector2 point) const {
    float distance = std::numeric_limits<float>::infinity();
    for (const Segment2& side : sides()) {
        distance = smallerOf(distance, side.distance(point));
    }
    return windingNumber(point) == 0 ? distance : -distance;
}

} // namespace editor
