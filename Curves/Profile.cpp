// Waves and keys along a share 0…1 (declared in Profile.handles).
#include "Profile.h"

#include <cmath>
#include <set>

#include "../Linear/Angle.h"

namespace editor::profile {

Wave constant(float value) {
    return [value](float) { return value; };
}

Wave gradient(std::vector<float> values) {
    if (values.empty()) throw MathProblem("A gradient needs a value.");
    if (values.size() == 1) return constant(values[0]);
    float gaps = static_cast<float>(values.size() - 1);
    return [values = std::move(values), gaps](float x) {
        float position = clamped(x, 0, 1) * gaps;
        if (std::isnan(position)) return position;
        float lower = smallerOf(std::floor(position), gaps - 1);
        size_t index = static_cast<size_t>(lower);
        return linearBlend(values[index], values[index + 1], position - lower);
    };
}

Wave steps(std::vector<float> values) {
    if (values.empty()) throw MathProblem("Steps need a value.");
    return [values = std::move(values)](float x) {
        float position = clamped(x, 0, 1) * static_cast<float>(values.size());
        if (std::isnan(position)) return position;
        return values[static_cast<size_t>(smallerOf(std::floor(position), static_cast<float>(values.size() - 1)))];
    };
}

namespace {

/// A repeating wave from its shape over one cycle (`shape` takes 0…1 and gives −1…1), laid out by `swing`.
Wave periodic(std::function<float(float)> shape, Swing swing) {
    return [shape = std::move(shape), swing](float x) {
        float position = swing.cycles * x + swing.phase;
        float level = shape(position - std::floor(position));
        return swing.low + (level + 1) / 2 * (swing.high - swing.low);
    };
}

/// One full turn in radians, for the circular waves.
Angle turnsToAngle(float turns) { return Angle::fromRadians(2 * Angle::pi * turns); }

} // namespace

Wave sine(Swing swing) {
    return periodic([](float cycleShare) { return turnsToAngle(cycleShare).sine(); }, swing);
}

Wave cosine(Swing swing) {
    return periodic([](float cycleShare) { return turnsToAngle(cycleShare).cosine(); }, swing);
}

Wave square(SquareSwing swing) {
    float duty = swing.duty;
    return periodic([duty](float cycleShare) { return cycleShare < duty ? 1.0f : -1.0f; }, swing);
}

Wave triangle(Swing swing) {
    return periodic([](float cycleShare) { return 1 - 4 * std::fabs(cycleShare - 0.5f); }, swing);
}

Wave saw(Swing swing) {
    return periodic([](float cycleShare) { return 2 * cycleShare - 1; }, swing);
}

Wave wave(const Strength& strength) {
    if (auto function = std::get_if<Wave>(&strength)) return *function;
    if (auto number = std::get_if<float>(&strength)) return constant(*number);
    return gradient(std::get<std::vector<float>>(strength));
}

Wave times(const std::vector<Strength>& strengths) {
    std::vector<Wave> factors;
    for (const Strength& strength : strengths) factors.push_back(wave(strength));
    return [factors = std::move(factors)](float x) {
        float product = 1;
        for (const Wave& factor : factors) product = product * factor(x);
        return product;
    };
}

Wave plus(const std::vector<Strength>& strengths) {
    std::vector<Wave> terms;
    for (const Strength& strength : strengths) terms.push_back(wave(strength));
    return [terms = std::move(terms)](float x) {
        float sum = 0;
        for (const Wave& term : terms) sum = sum + term(x);
        return sum;
    };
}

Keys<float> easeKeys(float startTime, float startValue, float endTime, float endValue) {
    float handleLength = (endTime - startTime) / 2;
    return {Key<float>{startTime, startValue, Handles{std::nullopt, Vector2(handleLength, 0)}},
            Key<float>{endTime, endValue, Handles{Vector2(-handleLength, 0), std::nullopt}}};
}

Keys<float> bulge(float mid, float ends) {
    float handleHeight = (mid - ends) * 4 / 3;
    return {Key<float>{0, ends, Handles{std::nullopt, Vector2(1.0f / 3, handleHeight)}},
            Key<float>{1, ends, Handles{Vector2(-1.0f / 3, handleHeight), std::nullopt}}};
}

std::optional<Keys<float>> keysOf(const std::optional<KeysInput>& input, int samples) {
    if (!input) return std::nullopt;
    if (auto number = std::get_if<float>(&*input)) return Keys<float>{Key<float>{0, *number}};
    if (auto function = std::get_if<Wave>(&*input)) {
        Keys<float> keys;
        for (int index = 0; index < samples; index += 1) {
            float share = static_cast<float>(index) / static_cast<float>(samples - 1);
            keys.push_back(Key<float>{share, (*function)(share)});
        }
        return keys;
    }
    if (auto keys = std::get_if<Keys<float>>(&*input)) return *keys;
    const std::vector<float>& values = std::get<std::vector<float>>(*input);
    if (values.empty()) return Keys<float>{};
    return spread<float>(values);
}

std::optional<Keys<float>> keysTimes(const std::optional<Keys<float>>& first, const std::optional<Keys<float>>& second) {
    if (!first || !second) return first ? first : second;
    std::set<float> times;
    for (int index = 0; index < 129; index += 1) times.insert(static_cast<float>(index) / 128);
    for (const Key<float>& key : *first) times.insert(key.time);
    for (const Key<float>& key : *second) times.insert(key.time);
    Keys<float> product;
    for (float time : times) {
        if (time >= 0 && time <= 1) product.push_back(Key<float>{time, keysAt(*first, time, 1.0f) * keysAt(*second, time, 1.0f)});
    }
    return product;
}

} // namespace editor::profile
