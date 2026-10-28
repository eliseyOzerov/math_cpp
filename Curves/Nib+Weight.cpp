// Weight compensation: how heavy the nib is for each segment of a stroke, and the keys that ease between them
// (Nib.h). Ported from graphics_cpp curves/nib (nib.cpp), in floats.
#include <algorithm>
#include <cmath>

#include "../Linear/Scalar.h"
#include "Nib.h"

namespace editor {

namespace {

/// The lean off the nearest axis, in degrees, from which a line counts fully as diagonal: below it the factor eases
/// in, so a nearly upright or level stroke isn't thinned.
constexpr float diagonalLean = 15;

/// How far a line leaning `degrees` off the nearest axis counts as diagonal: 0 upright or level, 1 from
/// `diagonalLean`, eased by smoothStep.
float diagonalness(float degrees) {
    float modulo = std::fmod(std::fmod(degrees, 90) + 90, 90);
    return smoothStep(std::min(1.0f, std::min(modulo, 90 - modulo) / diagonalLean));
}

} // namespace

float Nib::segmentWeight(const CubicBezier<Vector2>& segment, const WeightCompensation& compensation) {
    if (!segment.isStraight()) return compensation.round;
    Vector2 chord = segment.end - segment.start;
    float degrees = Angle::fromRadians(std::atan2(chord.y, chord.x)).degrees();
    return 1 + (compensation.diagonal - 1) * diagonalness(degrees);
}

std::optional<profile::Keys<float>> Nib::weightCurve(const std::vector<CubicBezier<Vector2>>& segments, bool closed,
                                                     const WeightCompensation& compensation, float ramp) {
    if (segments.empty()) return std::nullopt;
    std::vector<float> factors;
    std::vector<float> lengths;
    for (const CubicBezier<Vector2>& segment : segments) {
        factors.push_back(segmentWeight(segment, compensation));
        lengths.push_back(segment.totalLength());
    }
    bool flat = true;
    for (float factor : factors) {
        if (!(std::fabs(factor - 1) < 1e-6f)) flat = false;
    }
    if (flat) return std::nullopt;
    float sum = 0;
    for (float length : lengths) sum += length;
    float total = sum > 0 ? sum : 1;
    size_t count = segments.size();
    // A node's ease reaches at most half of either neighbour, and half the ramp.
    auto half = [&](size_t index) { return std::min(ramp / 2, lengths[index] / 2); };
    profile::Keys<float> keys;
    auto append = [&keys](const profile::Keys<float>& more) { keys.insert(keys.end(), more.begin(), more.end()); };
    bool wraps = closed && std::fabs(factors[count - 1] - factors[0]) > 1e-6f;
    float seamReach = std::min(half(count - 1), half(0));
    float seamFactor = (factors[count - 1] + factors[0]) / 2;
    if (wraps) append(profile::easeKeys(0, seamFactor, seamReach / total, factors[0]));
    else keys.push_back(profile::Key<float>{0, factors[0]});
    float travelled = 0;
    for (size_t index = 0; index + 1 < count; index += 1) {
        travelled += lengths[index];
        if (std::fabs(factors[index] - factors[index + 1]) < 1e-6f) continue;
        float reach = std::min(half(index), half(index + 1));
        append(profile::easeKeys((travelled - reach) / total, factors[index], (travelled + reach) / total, factors[index + 1]));
    }
    if (wraps) append(profile::easeKeys(1 - seamReach / total, factors[count - 1], 1, seamFactor));
    else keys.push_back(profile::Key<float>{1, factors[count - 1]});
    return keys;
}

} // namespace editor
