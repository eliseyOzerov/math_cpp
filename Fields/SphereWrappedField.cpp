// A flat shape bent onto a sphere (Engine/Math/Fields/SphereWrappedField.swift).
#include "SphereWrappedField.h"

#include <cmath>

#include "../Linear/Angle.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "../Linear/Vector2.h"
#include "FieldBounds.h"

namespace editor {

namespace {


/// The sphere's centre in the shape's frame: `(0, 0, −radius)`.
Vector3 sphereCentre(float radius) { return Vector3(0, 0, -radius); }

/// The radius check, then the box of the inner field, which a wrap needs (Swift's
/// `precondition` and `preconditionFailure`), in Swift's order.
Bounds checkedInnerBox(const DistanceField& inner, float radius) {
    if (!(radius > 0 && std::isfinite(radius))) throw MathProblem("A sphere wrap needs a positive radius.");
    if (!inner.bounds()) throw MathProblem("A sphere wrap needs a bounded inner field.");
    return *inner.bounds();
}

} // namespace

SphereWrappedField::SphereWrappedField(DistanceField inner, float radius)
    : inner(inner),
      innerBounds(checkedInnerBox(inner, radius)),
      radius(radius),
      guardRadius(0),
      guardAngle(Angle::zero()),
      stretch(0),
      margin(0),
      worldBox(Bounds{}) {
    float lowestReach = radius + innerBounds.minimum.z;
    float highestReach = radius + innerBounds.maximum.z;
    Angle widestArc = Angle::fromRadians(widestDistanceFromAxis(innerBounds) / radius);
    if (!(lowestReach > 0)) throw MathProblem("A sphere wrap's shape must stay above the sphere's centre.");
    if (!(widestArc.radians < Angle::pi)) {
        throw MathProblem("A sphere wrap's shape must reach less than halfway round the sphere.");
    }

    guardRadius = (1 - guardDepthShare()) * lowestReach;
    float widening = smallerOf(largestGuardWidening().radians, (Angle::pi - widestArc.radians) / 2);
    guardAngle = widestArc + Angle::fromRadians(widening);
    stretch = largestStretch(radius, guardRadius, guardAngle);
    margin = smallerOf(
        smallerOf((lowestReach - guardRadius) / stretch, radius * widening / stretch),
        lowestReach * Angle::fromRadians(widening).sine());
    worldBox = boundsOfCap(sphereCentre(radius), lowestReach, highestReach, widestArc);
}

float SphereWrappedField::largestStretch(float sphereRadius, float guardRadius, Angle guardAngle) {
    float angleOverSine = guardAngle.radians > 0 ? guardAngle.radians / guardAngle.sine() : 1;
    return largerOf(1, sphereRadius / guardRadius * angleOverSine);
}

float SphereWrappedField::widestDistanceFromAxis(const Bounds& box) {
    float farthestX = largerOf(std::fabs(box.minimum.x), std::fabs(box.maximum.x));
    float farthestY = largerOf(std::fabs(box.minimum.y), std::fabs(box.maximum.y));
    return Vector2(farthestX, farthestY).length();
}

Bounds SphereWrappedField::boundsOfCap(Vector3 centre, float nearest, float farthest, Angle widestArc) {
    float across = farthest * Angle::fromRadians(smallerOf(widestArc.radians, Angle::pi / 2)).sine();
    float lowest = widestArc.radians <= Angle::pi / 2 ? nearest * widestArc.cosine() : farthest * widestArc.cosine();
    return Bounds::checked(centre + Vector3(-across, -across, lowest), centre + Vector3(across, across, farthest));
}

bool SphereWrappedField::isInGuardRegion(Vector3 point) const {
    Vector3 fromCentre = point - sphereCentre(radius);
    return fromCentre.length() >= guardRadius
        && Angle::between(fromCentre, Vector3::zAxis()).radians <= guardAngle.radians;
}

Vector3 SphereWrappedField::flatPoint(Vector3 point) const {
    Vector3 fromCentre = point - sphereCentre(radius);
    Vector2 across(fromCentre.x, fromCentre.y);
    float arc = radius * Angle::between(fromCentre, Vector3::zAxis()).radians;
    Vector2 direction = across.unitDirection().value_or(Vector2(0, 0));
    return Vector3(direction.x * arc, direction.y * arc, fromCentre.length() - radius);
}

Vector3 SphereWrappedField::wrappedPoint(Vector3 flat) const {
    Vector2 across(flat.x, flat.y);
    Angle arc = Angle::fromRadians(across.length() / radius);
    Vector2 direction = across.unitDirection().value_or(Vector2(0, 0));
    float height = radius + flat.z;
    return Vector3(0, 0, -radius)
        + Vector3(direction.x * arc.sine() * height, direction.y * arc.sine() * height, arc.cosine() * height);
}

float SphereWrappedField::signedDistance(Vector3 point) const {
    float value = margin;
    if (isInGuardRegion(point)) {
        Vector3 flat = flatPoint(point);
        float innerValue = raising(innerBounds, inner.signedDistance(flat), flat);
        value = smallerOf(innerValue / stretch, margin);
    }
    return raising(worldBox, value, point);
}

} // namespace editor
