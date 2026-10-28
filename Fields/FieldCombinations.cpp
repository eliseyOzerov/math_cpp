// Carving, trimming and blending many parts (Engine/Math/Fields/FieldCombinations.swift).
#include "FieldCombinations.h"

#include "../Linear/GeometryProblem.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "FieldUnion.h"

namespace editor {

float SmoothSubtraction::signedDistance(Vector3 point) const {
    return smoothMaximum(kept.signedDistance(point), -removed.signedDistance(point), blendRadius);
}

SmoothIntersection::SmoothIntersection(DistanceField first, DistanceField second, float blendRadius)
    : first(first), second(second), blendRadius(blendRadius) {
    if (!first.bounds()) {
        overlapBox = second.bounds();
    } else if (!second.bounds()) {
        overlapBox = first.bounds();
    } else {
        const Bounds& firstBox = *first.bounds();
        const Bounds& secondBox = *second.bounds();
        std::optional<Bounds> overlap = intersection(firstBox, secondBox);
        overlapBox = overlap ? overlap : std::optional<Bounds>(Bounds::checked(firstBox.centre(), firstBox.centre()));
    }
}

float SmoothIntersection::signedDistance(Vector3 point) const {
    float blended = smoothMaximum(first.signedDistance(point), second.signedDistance(point), blendRadius);
    if (!overlapBox) return blended;
    return raising(*overlapBox, blended, point);
}

SmoothUnionOfMany::SmoothUnionOfMany(std::vector<DistanceField> parts, float blendRadius)
    : parts(parts), blendRadius(blendRadius) {
    if (parts.empty()) throw MathProblem("A smooth union needs at least one part.");
    for (const DistanceField& part : this->parts) partBoxes.push_back(part.bounds());
    chainBox = chainBounds(partBoxes, blendRadius);
}

std::optional<Bounds> SmoothUnionOfMany::chainBounds(const std::vector<std::optional<Bounds>>& boxes, float blendRadius) {
    std::vector<Bounds> known;
    for (const std::optional<Bounds>& box : boxes) {
        if (box) known.push_back(*box);
    }
    if (known.size() != boxes.size()) return std::nullopt;
    Bounds total = known[0];
    for (std::size_t index = 1; index < known.size(); index += 1) total = total.unionWith(known[index]);
    float chainDip = smoothMinimumChainLargestDip(blendRadius, static_cast<int>(boxes.size()) - 1);
    return expanded(total, chainDip);
}

float SmoothUnionOfMany::signedDistance(Vector3 point) const {
    float blended = parts[0].signedDistance(point);
    float reach = largerOf(blendRadius, 0);
    for (std::size_t index = 1; index < parts.size(); index += 1) {
        if (partBoxes[index] && signedDistanceOf(*partBoxes[index], point) >= blended + reach) continue;
        blended = smoothMinimum(blended, parts[index].signedDistance(point), blendRadius);
    }
    return blended;
}

} // namespace editor
