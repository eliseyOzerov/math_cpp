// Real roots of a quadratic equation (Engine/Math/Numerics/QuadraticRoots.swift).
#include "QuadraticRoots.h"

#include <cmath>

#include "../Linear/Scalar.h"

namespace editor {

QuadraticRoots QuadraticRoots::solve(float quadratic, float linear, float constant) {
    if (quadratic == 0) {
        if (linear == 0) return none();
        return one(-constant / linear);
    }

    float discriminant = linear * linear - 4 * quadratic * constant;
    // Written as `>= 0` so a NaN discriminant (b² and 4ac both overflowing to infinity) also
    // reports no roots.
    if (!(discriminant >= 0)) return none();
    if (discriminant == 0) return one(-linear / (2 * quadratic));

    // Choosing the sign of the square root to match `linear`'s sign is what keeps the addition
    // free of cancellation.
    float squareRoot = std::sqrt(discriminant);
    float signedRoot = linear >= 0 ? squareRoot : -squareRoot;
    float stableHalfSum = -0.5f * (linear + signedRoot);

    float firstRoot = stableHalfSum / quadratic;
    float secondRoot = constant / stableHalfSum;
    return two(smallerOf(firstRoot, secondRoot), largerOf(firstRoot, secondRoot));
}

} // namespace editor
