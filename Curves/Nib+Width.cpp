// How wide a nib's ink is, the transition keys that shape thick to thin, and shapes given loosely (Nib.h).
// Ported from graphics_cpp curves/nib (nib.cpp), in floats.
#include <algorithm>
#include <cmath>
#include <limits>

#include "Nib.h"

namespace editor {

float Nib::nibWidth(const NibInput& input, Angle direction, float scale) {
    Nib nib = from(input);
    Angle quarterTurn = Angle::fromRadians(Angle::pi / 2);
    Angle rotation = nib.angle + (direction.halfTurn() - quarterTurn) * clamped(nib.follow, 0, 1);
    Vector2 across(-direction.sine(), direction.cosine());
    NibShape scaled{nib.width * scale, nib.height * scale, nib.radiusX, nib.radiusY};
    float lowest = std::numeric_limits<float>::infinity();
    float highest = -std::numeric_limits<float>::infinity();
    for (Vector2 point : stampPolygon(scaled, rotation, nib.pointsPerCorner)) {
        float reach = point.dot(across);
        lowest = std::min(lowest, reach);
        highest = std::max(highest, reach);
    }
    return highest - lowest;
}

float Nib::stemWidth(const NibInput& input, float scale) { return nibWidth(input, Angle::fromRadians(Angle::pi / 2), scale); }

std::optional<profile::Keys<float>> Nib::transitionKeys(const NibInput& input, float transition,
                                                        const std::function<Angle(float)>& direction, int samples) {
    if (transition == 0 || std::isnan(transition)) return std::nullopt;
    if (samples < 2) throw MathProblem("Transition keys need at least two samples.");
    float low = std::numeric_limits<float>::infinity();
    float high = -std::numeric_limits<float>::infinity();
    for (int degrees = 0; degrees < 180; degrees += 1) {
        float width = nibWidth(input, Angle::degrees(static_cast<float>(degrees)));
        low = std::min(low, width);
        high = std::max(high, width);
    }
    if (high - low < 1e-5f) return std::nullopt;
    float power = std::pow(4.0f, clamped(transition, -1, 1));
    profile::Keys<float> keys;
    for (int index = 0; index < samples; index += 1) {
        float share = static_cast<float>(index) / static_cast<float>(samples - 1);
        float width = nibWidth(input, direction(share));
        float position = clamped((width - low) / (high - low), 0, 1);
        float pushed = position < 0.5f ? 0.5f * std::pow(2 * position, power) : 1 - 0.5f * std::pow(2 - 2 * position, power);
        keys.push_back(profile::Key<float>{share, (low + (high - low) * pushed) / width});
    }
    return keys;
}

std::optional<profile::Keys<NibShape>> Nib::shapesOf(const std::optional<NibShapesInput>& input, const Nib& nib) {
    if (!input) return std::nullopt;
    if (const profile::Keys<NibShape>* keys = std::get_if<profile::Keys<NibShape>>(&*input)) return *keys;
    const std::vector<PartialNibShape>& partials = std::get<std::vector<PartialNibShape>>(*input);
    if (partials.empty()) return profile::Keys<NibShape>{};
    std::vector<NibShape> shapes;
    for (const PartialNibShape& partial : partials) {
        shapes.push_back(NibShape{partial.width.value_or(1), partial.height.value_or(1), partial.radiusX.value_or(nib.radiusX),
                                  partial.radiusY.value_or(nib.radiusY)});
    }
    return profile::spread(shapes);
}

} // namespace editor
