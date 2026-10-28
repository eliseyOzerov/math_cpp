// How light fades as it travels through an absorbing material (Engine/Math/Optics/BeerLambert.swift).
#include "BeerLambert.h"

#include <cassert>
#include <cmath>

#include "../Linear/Scalar.h"

namespace editor {

Vector3 BeerLambert::transmittance(Vector3 absorptionCoefficient, float thickness) {
    // Written as `!(thickness <= 0)` so that a NaN thickness falls through to `exp` and
    // comes out NaN, as documented, instead of passing as "no material".
    if (!(thickness <= 0)) {
        return Vector3(
            std::exp(-largerOf(absorptionCoefficient.x, 0) * thickness),
            std::exp(-largerOf(absorptionCoefficient.y, 0) * thickness),
            std::exp(-largerOf(absorptionCoefficient.z, 0) * thickness));
    }
    return Vector3(1, 1, 1);
}

Vector3 BeerLambert::absorptionCoefficient(Vector3 transmittance, float distance) {
    assert(distance > 0 && "The distance must be positive.");
    float clampedRed = clamped(transmittance.x, minimumTransmittance, 1);
    float clampedGreen = clamped(transmittance.y, minimumTransmittance, 1);
    float clampedBlue = clamped(transmittance.z, minimumTransmittance, 1);
    return Vector3(-std::log(clampedRed), -std::log(clampedGreen), -std::log(clampedBlue)) / distance;
}

} // namespace editor
