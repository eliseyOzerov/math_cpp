// Profiles: a value along a share x (0…1, round a ring or along a path), to vary a move or a pen from place to place.
// Two kinds. A wave is a function of x: one value is the same everywhere, a list of values a gradient or steps, and
// sine, cosine, square, triangle and saw waves repeat; waves multiply and add, so they combine into any profile.
// Keys are data: (t, value) points with optional bezier handles, like animation keyframes, so they save as JSON and
// a panel can drag them, and the value can be any type (a nib morphs between shapes this way).
//
// Ported from graphics_cpp curves/profile (profile.hpp, profile.cpp), in floats.
//
// Known overlap (as in the original): a wave given where keys are wanted is sampled (keysOf), and keys multiply by
// resampling (keysTimes) where waves multiply exactly.
//
// Declared here, defined in Profile.cpp; the key templates (keysAt, sorted, spread) live in this header.
#pragma once

#include <algorithm>
#include <functional>
#include <optional>
#include <variant>
#include <vector>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "../Linear/Vector2.h"
#include "CubicBezier.h"

namespace editor::profile {

/// A strength for each share `x` of the way (0…1).
///
/// Use when: any move or pen width that varies from node to node
/// Tags: wave, profile, strength function, envelope, 1D function
/// Node: planned (profile)
using Wave = std::function<float(float)>;

/// A strength as given: one value everywhere, a list of values as a gradient between them, or a wave.
/// Build a number with a float (`0.5f`): a double is a narrowing conversion the variant refuses.
///
/// Use when: an option that accepts a constant, a gradient or a full wave
/// Tags: strength, number or function, profile input
/// Node: planned (profile)
using Strength = std::variant<float, std::vector<float>, Wave>;

/// The same `value` everywhere.
///
/// Use when: a profile that does not vary; the neutral factor of `times`
/// Tags: constant wave, flat profile, uniform strength
/// Node: planned (profile)
Wave constant(float value);

/// A smooth run through `values`, spread evenly from x = 0 to x = 1 (one value is the same everywhere).
///
/// How: with n = count − 1 gaps, u = clamp(x, 0, 1)·n; i = min(floor(u), n − 1); the result is the straight blend of
/// values[i] and values[i + 1] by u − i. x outside 0…1 holds the end values; a NaN x gives NaN. For a ring, end on the
/// first value so the profile meets itself. Throws a `MathProblem` with no values.
///
/// Use when: a strength that eases between a few set values
/// Tags: gradient, piecewise linear, ramp, lerp through values, multi-stop
/// Node: planned (profile)
Wave gradient(std::vector<float> values);

/// Hard jumps between `values`: x split into as many equal parts, each part one value.
///
/// How: index = min(floor(clamp(x, 0, 1)·count), count − 1). Throws a `MathProblem` with no values.
///
/// Use when: a stepped profile, such as bands of different thickness
/// Tags: steps, staircase, piecewise constant, quantized profile
/// Node: planned (profile)
Wave steps(std::vector<float> values);

/// How a repeating wave is laid out: `cycles` over x from 0 to 1, starting `phase` of a cycle in, swinging between
/// `low` and `high`.
struct Swing {
    float cycles = 1;
    float phase = 0;
    float low = -1;
    float high = 1;
};

/// A square wave's layout: a swing and its duty (the share of each cycle spent high).
struct SquareSwing : Swing {
    float duty = 0.5f;
};

/// A sine wave: at its middle at the start (for phase 0), rising.
///
/// How: with u = cycles·x + phase, low + (sin(2π·frac(u)) + 1)/2·(high − low).
///
/// Use when: a smooth repeating wobble or ripple
/// Tags: sine wave, oscillation, ripple, periodic
/// Node: planned (profile)
Wave sine(Swing swing = {});

/// A cosine wave: at its high at the start (for phase 0).
///
/// How: as `sine`, with cos(2π·frac(u)).
///
/// Use when: a repeating profile that starts at its peak
/// Tags: cosine wave, oscillation, periodic
/// Node: planned (profile)
Wave cosine(Swing swing = {});

/// A square wave: high for the first `duty` of each cycle, then low.
///
/// How: as `sine`, with +1 while frac(u) < duty and −1 after.
///
/// Use when: alternating thick and thin, on and off
/// Tags: square wave, pulse, duty cycle, on off
/// Node: planned (profile)
Wave square(SquareSwing swing = {});

/// A triangle wave: from its low up to its high halfway through each cycle and back.
///
/// How: as `sine`, with 1 − 4·|frac(u) − ½|, which is −1 at frac 0, +1 at frac ½.
///
/// Use when: a zigzag profile
/// Tags: triangle wave, zigzag, linear oscillation
/// Node: planned (profile)
Wave triangle(Swing swing = {});

/// A saw wave: from its low up to its high over each cycle, then straight back down.
///
/// How: as `sine`, with 2·frac(u) − 1.
///
/// Use when: a repeating ramp, such as teeth
/// Tags: saw wave, sawtooth, ramp, repeating ramp
/// Node: planned (profile)
Wave saw(Swing swing = {});

/// A strength as a wave: a wave as it is, a number as `constant`, a list as `gradient`.
///
/// Use when: accepting any kind of strength and needing a function
/// Tags: to wave, strength to function, normalize profile input
/// Node: planned (profile)
Wave wave(const Strength& strength);

/// The strengths multiplied: a wave shaped by another (a sine fading out under a gradient, say). No strengths gives 1.
///
/// Use when: modulating one profile by another
/// Tags: multiply waves, modulate, envelope, product of functions
/// Node: planned (profile)
Wave times(const std::vector<Strength>& strengths);

/// The strengths added. No strengths gives 0.
///
/// Use when: layering waves, a base plus a ripple
/// Tags: add waves, sum of functions, layer, mix profiles
/// Node: planned (profile)
Wave plus(const std::vector<Strength>& strengths);

/// A key's bezier handles, each (dt, dv) from it.
struct Handles {
    std::optional<Vector2> in;
    std::optional<Vector2> out;
};

/// A key: a value at a share t, and optionally handles.
template <class Value = float>
struct Key {
    /// Where the key sits, a share 0…1.
    float time;
    /// The value there.
    Value value;
    /// Its handles, or nothing for a plain key.
    std::optional<Handles> handles = std::nullopt;
};

/// Keys: (t, value) points, or with handles, over a share 0…1 (along a stroke, 0 start … 1 end).
template <class Value = float>
using Keys = std::vector<Key<Value>>;

/// The keys' value at `t`, level past the first and last; `fallback` with none. Between two keys the value runs
/// straight unless either has a handle on that side, and then as the cubic through the handles (a missing one sits on
/// its key). Values that aren't numbers are blended by `mix`, which must then be given. Handles are held inside their
/// span along t, so there is one value at each t.
///
/// How (numbers with handles): the span t0…t1 becomes the cubic (t0, a), (t0 + out.dt, a + out.dv),
/// (t1 + in.dt, b + in.dv), (t1, b) with out.dt held in 0…span and in.dt in −span…0, so its t rises with its parameter;
/// the parameter whose t equals `t` is found by halving (30 times), and the value is the cubic's second coordinate.
///
///     value │      ●─ ─ out handle        two keys, the first with an out handle
///           │    ╱    ╲___
///           │ ●╱          ●
///           └───────────── t
///
/// Equal key times (zero span) give the later key's value. Source: graphics_cpp curves/profile `keysAt`.
///
/// Use when: reading a width or strength envelope at a position along a stroke
/// Tags: keyframes, animation curve, sample keys, envelope, bezier keyframe interpolation
/// Node: planned (keysAt)
template <class Value = float>
Value keysAt(const Keys<Value>& keys, float t, Value fallback = Value{},
             std::function<Value(const Value&, const Value&, float)> mix = {}) {
    if (keys.empty()) return fallback;
    if (!mix) {
        if constexpr (std::is_same_v<Value, float>) {
            mix = [](const float& start, const float& end, float amount) { return linearBlend(start, end, amount); };
        } else {
            throw MathProblem("keysAt needs a mix for values that aren't numbers.");
        }
    }
    if (t <= keys[0].time) return keys[0].value;
    for (size_t index = 1; index < keys.size(); index += 1) {
        if (!(t <= keys[index].time)) continue;
        float startTime = keys[index - 1].time;
        float endTime = keys[index].time;
        const Value& startValue = keys[index - 1].value;
        const Value& endValue = keys[index].value;
        if (!(endTime > startTime)) return endValue;
        std::optional<Vector2> outHandle = keys[index - 1].handles ? keys[index - 1].handles->out : std::nullopt;
        std::optional<Vector2> inHandle = keys[index].handles ? keys[index].handles->in : std::nullopt;
        if constexpr (std::is_same_v<Value, float>) {
            if (!outHandle && !inHandle) return mix(startValue, endValue, (t - startTime) / (endTime - startTime));
            float span = endTime - startTime;
            Vector2 out = outHandle.value_or(Vector2());
            Vector2 in = inHandle.value_or(Vector2());
            CubicBezier<Vector2> cubic(Vector2(startTime, startValue),
                                       Vector2(startTime + clamped(out.x, 0, span), startValue + out.y),
                                       Vector2(endTime + clamped(in.x, -span, 0), endValue + in.y), Vector2(endTime, endValue));
            float lower = 0;
            float upper = 1;
            for (int step = 0; step < 30; step += 1) {
                float middle = (lower + upper) / 2;
                if (cubic.point(middle).x < t) lower = middle; else upper = middle;
            }
            return cubic.point((lower + upper) / 2).y;
        } else {
            return mix(startValue, endValue, (t - startTime) / (endTime - startTime));
        }
    }
    return keys.back().value;
}

/// keysAt for keys that may be none (an empty optional gives `fallback`).
template <class Value = float>
Value keysAt(const std::optional<Keys<Value>>& keys, float t, Value fallback = Value{},
             std::function<Value(const Value&, const Value&, float)> mix = {}) {
    return keysAt<Value>(keys ? *keys : Keys<Value>{}, t, std::move(fallback), std::move(mix));
}

/// The keys in order along t (a stable sort: keys at the same t keep their order), or none for none or empty.
///
/// Use when: keys arrive from a panel or file in any order
/// Tags: sort keys, order keyframes, normalize keys
/// Node: planned (keysAt)
template <class Value = float>
std::optional<Keys<Value>> sorted(const std::optional<Keys<Value>>& keys) {
    if (!keys || keys->empty()) return std::nullopt;
    Keys<Value> ordered = *keys;
    std::stable_sort(ordered.begin(), ordered.end(), [](const Key<Value>& first, const Key<Value>& second) { return first.time < second.time; });
    return ordered;
}

/// Values spread evenly over 0…1 as keys: key i at i/(count − 1); one value is a single key at 0; none is no keys.
///
/// Use when: turning a list of numbers into editable keys
/// Tags: spread, evenly spaced keys, list to keyframes
/// Node: planned (keysAt)
template <class Value = float>
Keys<Value> spread(const std::vector<Value>& values) {
    if (values.size() == 1) return {Key<Value>{0, values[0]}};
    Keys<Value> keys;
    for (size_t index = 0; index < values.size(); index += 1) {
        keys.push_back(Key<Value>{static_cast<float>(index) / static_cast<float>(values.size() - 1), values[index]});
    }
    return keys;
}

/// Two keys easing from `startValue` at share `startTime` to `endValue` at `endTime`: flat handles half the span long,
/// so the value leaves and meets its levels smoothly.
///
/// Use when: a smooth step between two levels along a stroke
/// Tags: ease keys, smooth transition, flat handles, ease in out
/// Node: planned (keysAt)
Keys<float> easeKeys(float startTime, float startValue, float endTime, float endValue);

/// One cubic from `ends` at 0 to `mid` at the middle and back to `ends` at 1: a waist below `ends` (a nib narrowing
/// along a stroke) or a swell above.
///
/// How: a cubic whose ends sit at `ends` with handles a third of the way along at height h reaches
/// ends + 3/4·(h − ends) at its middle, so the handles go (mid − ends)·4/3 above the ends.
///
/// Use when: a stroke that pinches or swells in the middle
/// Tags: bulge, waist, swell, pinch, symmetric envelope
/// Node: planned (keysAt)
Keys<float> bulge(float mid, float ends = 1);

/// Keys given loosely: keys as they are, a number, a list of numbers or a wave.
using KeysInput = std::variant<Keys<float>, float, std::vector<float>, Wave>;

/// Keys from a loose input: keys as they are, a number as one key, a list spread evenly, and a wave sampled into
/// `samples` points (enough that its shape survives the straight runs between them; keys with handles are the way
/// to draw a smooth one exactly). No input gives none; an empty list gives empty keys.
///
/// Use when: an option accepts a number, a wave or keys and the code wants keys
/// Tags: to keys, sample wave, normalize keys input
/// Node: planned (keysAt)
std::optional<Keys<float>> keysOf(const std::optional<KeysInput>& input, int samples = 65);

/// Two keys multiplied, either left out (the other is returned): read at 129 even places and at each one's own key
/// times, so a step in either stays sharp. Key times outside 0…1 are skipped.
///
/// Use when: combining two envelopes along a stroke
/// Tags: multiply keys, combine envelopes, resample product
/// Node: planned (keysAt)
std::optional<Keys<float>> keysTimes(const std::optional<Keys<float>>& first, const std::optional<Keys<float>>& second);

} // namespace editor::profile
