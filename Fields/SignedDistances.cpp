// Exact signed distances to the basic solids (Engine/Math/Fields/SignedDistances.swift).
#include "SignedDistances.h"

#include <cmath>

#include "../Geometry/Line.h"
#include "../Linear/Scalar.h"
#include "VectorComponents.h"

namespace editor {

float signedDistanceOf(const Sphere& sphere, Vector3 point) {
    return point.distance(sphere.centre) - sphere.radius;
}

float signedDistanceOf(const Plane& plane, Vector3 point) { return plane.signedDistance(point); }

float Capsule::signedDistance(Vector3 point) const {
    return point.distance(axis.closestPoint(point)) - radius;
}

float signedDistanceOf(const Bounds& box, Vector3 point) {
    Vector3 halfSize = box.size() * 0.5f;
    Vector3 overshoot = componentwiseAbsolute(point - box.centre()) - halfSize;
    float outsidePart = componentwiseMaximum(overshoot, 0).length();
    float insidePart = smallerOf(largestComponent(overshoot), 0);
    return outsidePart + insidePart;
}

float RoundCone::signedDistance(Vector3 point) const {
    Sphere startSphere(start, startRadius);
    Sphere endSphere(end, endRadius);
    Vector3 axisVector = end - start;
    float axisLength = axisVector.length();
    float radiusDifference = startRadius - endRadius;
    if (!(axisLength > std::fabs(radiusDifference))) {
        // One sphere swallows the other: the shape is the bigger sphere.
        const Sphere& biggerSphere = startRadius >= endRadius ? startSphere : endSphere;
        return signedDistanceOf(biggerSphere, point);
    }

    float sideTilt = radiusDifference / axisLength;
    float sideCosine = std::sqrt(1 - sideTilt * sideTilt);
    Line axis(start, axisVector);
    float alongAxis = axis.distanceAlong(point);
    float radial = axis.distance(point);
    float alongSide = sideCosine * alongAxis - sideTilt * radial;

    if (alongSide < 0) {
        return signedDistanceOf(startSphere, point);
    } else if (alongSide > sideCosine * axisLength) {
        return signedDistanceOf(endSphere, point);
    } else {
        return sideCosine * radial + sideTilt * alongAxis - startRadius;
    }
}

} // namespace editor
