// Numerical integration (Engine/Math/Numerics/Integration.swift).
#include "Integration.h"

#include "../Linear/GeometryProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

namespace {

/// The 5-point rule applied once over start…end. See `GaussLegendre::integral` for the formula.
float singlePieceIntegral(const std::function<float(float)>& function, float start, float end) {
    float middle = linearBlend(start, end, 0.5f);
    float halfWidth = (end - start) * 0.5f;
    float weightedSum = 0;
    for (size_t index = 0; index < GaussLegendre::nodes.size(); index += 1) {
        weightedSum += GaussLegendre::weights[index] * function(middle + halfWidth * GaussLegendre::nodes[index]);
    }
    return halfWidth * weightedSum;
}

} // namespace

float GaussLegendre::integral(const std::function<float(float)>& function, float start, float end, int pieceCount) {
    if (pieceCount < 1) throw GeometryProblem("An integral needs at least one piece.");
    float total = 0;
    for (int pieceIndex = 0; pieceIndex < pieceCount; pieceIndex += 1) {
        float pieceStart = linearBlend(
            start, end, static_cast<float>(pieceIndex) / static_cast<float>(pieceCount));
        float pieceEnd = linearBlend(
            start, end, static_cast<float>(pieceIndex + 1) / static_cast<float>(pieceCount));
        total += singlePieceIntegral(function, pieceStart, pieceEnd);
    }
    return total;
}

} // namespace editor
