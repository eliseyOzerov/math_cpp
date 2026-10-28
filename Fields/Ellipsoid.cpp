// An ellipsoid as a distance bound (Engine/Math/Fields/Ellipsoid.swift).
#include "Ellipsoid.h"

#include <cmath>

#include "../Linear/GeometryProblem.h"
#include "FieldBounds.h"
#include "VectorComponents.h"

namespace editor {

Ellipsoid::Ellipsoid(Vector3 centre, Vector3 radii, Quaternion rotation)
    : centre(centre), radii(radii), rotation(rotation), worldBox(Bounds{}) {
    if (!(radii.isFinite() && smallestComponent(radii) > 0)) {
        throw GeometryProblem("An ellipsoid needs three positive, finite radii.");
    }
    worldBox = tightBox(centre, radii, rotation);
}

Bounds Ellipsoid::localBox() const { return Bounds::checked(-radii, radii); }

float Ellipsoid::signedDistance(Vector3 point) const {
    Vector3 local = rotation.conjugate().rotate(point - centre);
    float stretchedSphere = ((local / radii).length() - 1) * smallestComponent(radii);
    float withOwnBox = raising(localBox(), stretchedSphere, local);
    return raising(worldBox, withOwnBox, point);
}

Bounds Ellipsoid::tightBox(Vector3 centre, Vector3 radii, const Quaternion& rotation) {
    Vector3 axes[3] = {Vector3::xAxis(), Vector3::yAxis(), Vector3::zAxis()};
    Vector3 squaredReach(0, 0, 0);
    for (int index = 0; index < 3; index += 1) {
        Vector3 semiAxis = rotation.rotate(axes[index]) * radii[index];
        squaredReach = squaredReach + semiAxis * semiAxis;
    }
    Vector3 reach(std::sqrt(squaredReach.x), std::sqrt(squaredReach.y), std::sqrt(squaredReach.z));
    return Bounds::checked(centre - reach, centre + reach);
}

} // namespace editor
