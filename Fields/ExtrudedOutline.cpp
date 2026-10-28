// A flat outline pushed out into a slab (Engine/Math/Fields/ExtrudedOutline.swift).
#include "ExtrudedOutline.h"

#include <cmath>

#include "../Linear/GeometryProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

namespace {

Polygon2D flattenedOutline(const Path2D& path, float tolerance) {
    if (!path.isClosed()) throw GeometryProblem("An extruded outline needs a closed path.");
    if (!(tolerance > 0 && std::isfinite(tolerance))) throw GeometryProblem("An extruded outline needs a positive tolerance.");
    return Polygon2D(path.flattened(tolerance));
}

} // namespace

ExtrudedOutline::ExtrudedOutline(const Path2D& path, float thickness, float tolerance)
    : ExtrudedOutline(flattenedOutline(path, tolerance), thickness) {}

ExtrudedOutline::ExtrudedOutline(const Polygon2D& outline, float thickness)
    : outline(outline), thickness(thickness), boundingBox(Bounds{}) {
    if (!(thickness > 0 && std::isfinite(thickness))) throw GeometryProblem("An extruded outline needs a positive thickness.");
    Bounds2 flat = outline.bounds();
    boundingBox = Bounds::checked(
        Vector3(flat.minimum.x, flat.minimum.y, -thickness / 2), Vector3(flat.maximum.x, flat.maximum.y, thickness / 2));
}

float ExtrudedOutline::signedDistance(Vector3 point) const {
    float acrossOutline = outline.signedDistance(Vector2(point.x, point.y));
    float acrossFaces = std::fabs(point.z) - thickness / 2;
    float inside = smallerOf(largerOf(acrossOutline, acrossFaces), 0);
    float outside = Vector2(largerOf(acrossOutline, 0), largerOf(acrossFaces, 0)).length();
    return inside + outside;
}

} // namespace editor
