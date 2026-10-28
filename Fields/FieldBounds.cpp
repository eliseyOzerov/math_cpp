// Boxes around fields (Engine/Math/Fields/FieldBounds.swift).
#include "FieldBounds.h"

#include <cmath>
#include <functional>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "VectorComponents.h"

namespace editor {

std::optional<Bounds> boundsOf(const Sphere& sphere) {
    Vector3 reach(sphere.radius, sphere.radius, sphere.radius);
    return Bounds::checked(sphere.centre - reach, sphere.centre + reach);
}

std::optional<Bounds> boundsOf(const Plane&) { return std::nullopt; }

std::optional<Bounds> boundsOf(const Bounds& box) { return box; }

std::optional<Bounds> Capsule::bounds() const {
    return expanded(Bounds::enclosing({axis.start, axis.end}), radius);
}

std::optional<Bounds> RoundCone::bounds() const { return enclosingBox(); }

Bounds RoundCone::enclosingBox() const {
    return expanded(Bounds::enclosing({start, end}), largerOf(startRadius, endRadius));
}

Bounds expanded(const Bounds& box, float margin) {
    if (!(margin >= 0)) throw MathProblem("A box can only be grown by a margin of 0 or more.");
    Vector3 reach(margin, margin, margin);
    return Bounds::checked(box.minimum - reach, box.maximum + reach);
}

std::optional<Bounds> intersection(const Bounds& first, const Bounds& second) {
    Vector3 lowest(largerOf(first.minimum.x, second.minimum.x), largerOf(first.minimum.y, second.minimum.y),
                   largerOf(first.minimum.z, second.minimum.z));
    Vector3 highest(smallerOf(first.maximum.x, second.maximum.x), smallerOf(first.maximum.y, second.maximum.y),
                    smallerOf(first.maximum.z, second.maximum.z));
    if (!(lowest.x <= highest.x && lowest.y <= highest.y && lowest.z <= highest.z)) return std::nullopt;
    return Bounds::checked(lowest, highest);
}

Bounds frontPart(const Bounds& box, const Plane& plane) {
    Vector3 normal = plane.normal;
    Vector3 lowest = box.minimum;
    Vector3 highest = box.maximum;
    if (normal.y == 0 && normal.z == 0) {
        if (normal.x > 0) lowest.x = largerOf(lowest.x, plane.distance);
        else highest.x = smallerOf(highest.x, -plane.distance);
    } else if (normal.x == 0 && normal.z == 0) {
        if (normal.y > 0) lowest.y = largerOf(lowest.y, plane.distance);
        else highest.y = smallerOf(highest.y, -plane.distance);
    } else if (normal.x == 0 && normal.y == 0) {
        if (normal.z > 0) lowest.z = largerOf(lowest.z, plane.distance);
        else highest.z = smallerOf(highest.z, -plane.distance);
    } else {
        return box;
    }
    if (!(lowest.x <= highest.x && lowest.y <= highest.y && lowest.z <= highest.z)) return box;
    return Bounds::checked(lowest, highest);
}

std::optional<Bounds> enclosing(const std::optional<Bounds>& first, const std::optional<Bounds>& second) {
    if (!first || !second) return std::nullopt;
    return first->unionWith(*second);
}

namespace {

/// The box with its centre at `centre` and half size the sum of the absolute images of the
/// three half-edge vectors under `linearPart` (Arvo's method, see `transformed`).
Bounds enclosingImage(const Bounds& box, Vector3 centre, const std::function<Vector3(Vector3)>& linearPart) {
    Vector3 halfSize = box.size() * 0.5f;
    Vector3 reach = componentwiseAbsolute(linearPart(Vector3(halfSize.x, 0, 0)))
        + componentwiseAbsolute(linearPart(Vector3(0, halfSize.y, 0)))
        + componentwiseAbsolute(linearPart(Vector3(0, 0, halfSize.z)));
    return Bounds::checked(centre - reach, centre + reach);
}

} // namespace

Bounds transformed(const Bounds& box, const Transform& transform) {
    return enclosingImage(
        box, transform.rotation.rotate(transform.scale * box.centre()) + transform.translation,
        [&](Vector3 vector) { return transform.rotation.rotate(transform.scale * vector); });
}

Bounds reflected(const Bounds& box, const Plane& plane) {
    return enclosingImage(box, reflection(plane, box.centre()),
                          [&](Vector3 vector) { return vector.reflectedAcrossPlane(plane.normal); });
}

float raising(const Bounds& box, float value, Vector3 point) {
    float boxDistance = signedDistanceOf(box, point);
    if (std::isnan(value) || std::isnan(boxDistance)) return NAN;
    return largerOf(value, boxDistance);
}

Vector3 reflection(const Plane& plane, Vector3 point) {
    return point - plane.normal * (2 * plane.signedDistance(point));
}

} // namespace editor
