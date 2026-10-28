// The clock, curves over time, and what the host reports about the pointer, keys, MIDI, the date
// and the microphone (graphics_metal's Nodes/Time.swift).
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

NodeGroup timeNodes() {
    NodeBuilder c(NodeCategory::time);
    const std::string period = "max({period}, 0.000001f)";
    return {c.category, {
        c.node({.name = "time", .label = "Time", .summary = "Seconds since the canvas started, and the frame's timing.",
                .symbol = "clock",
                .outputs = {out("seconds", "Seconds", number, "u.time"), out("delta", "Delta", number, "u.deltaTime"),
                            out("frame", "Frame", number, "u.frame"), out("frameRate", "Frame Rate", number, "u.frameRate")}}),
        c.node({.name = "time-loop", .label = "Time Loop", .summary = "Time wrapped to a period, for animations that loop seamlessly.",
                .symbol = "repeat", .inputs = {timeIn(), num("period", "Period", 4)},
                .outputs = {out("time", "Time", number, "flooredModulo({time}, " + period + ")"),
                            out("phase", "Phase", number, "fract({time} / " + period + ")"),
                            out("angle", "Angle", number, "fract({time} / " + period + ") * tau<float>()")}}),
        c.simple("ease", "Ease", "Reshapes a 0-to-1 value with an easing curve.", {num("t", "T", 0.5, between(0, 1))}, number,
                 "easedCurve({t}, {#curve})",
                 {.symbol = "point.topleft.down.to.point.bottomright.curvepath",
                  .options = {option("curve", "Curve", {
                      "Linear", "Quad In", "Quad Out", "Quad In-Out", "Cubic In", "Cubic Out", "Cubic In-Out",
                      "Expo In", "Expo Out", "Expo In-Out", "Sine In", "Sine Out", "Sine In-Out", "Back In",
                      "Back Out", "Back In-Out", "Elastic In", "Elastic Out", "Bounce In", "Bounce Out"}, 3)}}),
        c.simple("keyframes", "Keyframes", "Four values in turn over time, looping, with a curve between them.",
                 {timeIn(), num("a", "A"), num("b", "B", 1), num("c", "C", 0.25), num("d", "D", 0.75), num("duration", "Duration", 4)},
                 number, "keyframesOfFour({time}, float4({a}, {b}, {c}, {d}), {duration}, {#interpolation})",
                 {.symbol = "diamond", .options = {option("interpolation", "Interpolation", {"Linear", "Smooth", "Step"}, 1)}}),
        c.simple("delay", "Delay", "Time held at 0 until the delay has passed.", {timeIn(), num("delay", "Delay", 1)}, number,
                 "max({time} - {delay}, 0.0f)", {.symbol = "hourglass"}),
        c.node({.name = "random-over-time", .label = "Random Over Time",
                .summary = "A new random value at each step of time, held or eased between.", .symbol = "dice",
                .inputs = {timeIn(), num("rate", "Rate", 2), num("seed", "Seed", 1)},
                .outputs = {out("value", "Value", number,
                                "({#smooth} == 1 ? mix(hash21(float2(floor({$}), {seed})), hash21(float2(floor({$}) + 1.0f, {seed})), smoothstep(0.0f, 1.0f, fract({$}))) : hash21(float2(floor({$}), {seed})))")},
                .options = {option("smooth", "Change", {"Stepped", "Smooth"})}, .setup = "{time} * {rate}"}),
        c.node({.name = "beat", .label = "Beat", .summary = "Pulses on a tempo: a spike each beat that decays.", .symbol = "metronome",
                .inputs = {timeIn(), num("bpm", "BPM", 120, between(1, 300)), num("decay", "Decay", 8, between(0, 40))},
                .outputs = {out("pulse", "Pulse", number, "exp(-{decay} * fract({$}))"),
                            out("count", "Count", number, "floor({$})"), out("phase", "Phase", number, "fract({$})")},
                .setup = "{time} * {bpm} / 60.0f"}),
        c.node({.name = "frame-index", .label = "Frame Index", .summary = "How many frames have been drawn.", .symbol = "film",
                .outputs = {out("frame", "Frame", number, "u.frame")}}),
        c.node({.name = "delta-time", .label = "Delta Time", .summary = "Seconds since the last frame.", .symbol = "stopwatch",
                .outputs = {out("delta", "Delta", number, "u.deltaTime")}}),
    }};
}

NodeGroup interactionNodes() {
    NodeBuilder c(NodeCategory::interaction);
    return {c.category, {
        c.node({.name = "pointer", .label = "Pointer", .summary = "Where the pointer is over the canvas, and whether it is pressed.",
                .symbol = "cursorarrow",
                .outputs = {out("uv", "UV", vector2, "(u.pointer.xy / max(u.resolution, float2(1.0f)))"),
                            out("pixels", "Pixels", vector2, "u.pointer.xy"), out("pressed", "Pressed", number, "u.pointer.z")}}),
        c.node({.name = "pointer-delta", .label = "Pointer Delta", .summary = "How far the pointer moved since the last frame.",
                .symbol = "cursorarrow.motionlines",
                .outputs = {out("pixels", "Pixels", vector2, "u.pointerDelta"),
                            out("uv", "UV", vector2, "(u.pointerDelta / max(u.resolution, float2(1.0f)))"),
                            out("velocity", "Velocity", vector2,
                                "(u.pointerDelta / max(u.resolution, float2(1.0f)) / max(u.deltaTime, 0.000001f))")}}),
        c.node({.name = "click-count", .label = "Click Count", .summary = "How many times the canvas has been clicked.",
                .symbol = "cursorarrow.click.2",
                .outputs = {out("count", "Count", number, "u.clickCount"),
                            out("parity", "Odd", number, "flooredModulo(u.clickCount, 2.0f)")}}),
        c.node({.name = "keyboard-key", .label = "Keyboard Key", .summary = "1 while a key is held down, by its key code (Space is 49).",
                .symbol = "keyboard", .inputs = {num("key", "Key Code", 49, between(0, 255))},
                .outputs = {out("down", "Down", number, "{feed}.read(uint2(uint(clamp({key}, 0.0f, 255.0f)), 0u)).r")},
                .feed = TextureFeed::controls}),
        c.node({.name = "midi", .label = "MIDI", .summary = "A MIDI controller's value and a note's velocity, each 0 to 1.",
                .symbol = "pianokeys",
                .inputs = {num("controller", "Controller", 1, between(0, 127)), num("note", "Note", 60, between(0, 127))},
                .outputs = {out("controller", "Controller", number,
                                "{feed}.read(uint2(uint(clamp({controller}, 0.0f, 127.0f)), 1u)).r"),
                            out("velocity", "Velocity", number,
                                "{feed}.read(uint2(128u + uint(clamp({note}, 0.0f, 127.0f)), 1u)).r")},
                .feed = TextureFeed::controls}),
        c.node({.name = "osc", .label = "OSC Input", .summary = "A value sent over OSC to /kagiro/<index>, 0 to 1.",
                .symbol = "antenna.radiowaves.left.and.right", .inputs = {num("index", "Index", 0, between(0, 255))},
                .outputs = {out("value", "Value", number, "{feed}.read(uint2(uint(clamp({index}, 0.0f, 255.0f)), 2u)).r")},
                .feed = TextureFeed::controls}),
        c.node({.name = "date", .label = "Date", .summary = "The wall clock: the date, and the time of day.", .symbol = "calendar",
                .outputs = {out("year", "Year", number, "u.date.x"), out("month", "Month", number, "u.date.y"),
                            out("day", "Day", number, "u.date.z"),
                            out("hours", "Hours", number, "floor(u.date.w / 3600.0f)"),
                            out("minutes", "Minutes", number, "floor(flooredModulo(u.date.w, 3600.0f) / 60.0f)"),
                            out("seconds", "Seconds", number, "flooredModulo(u.date.w, 60.0f)")}}),
        c.node({.name = "audio-level", .label = "Audio Level", .summary = "The microphone's loudness, overall and in three bands.",
                .symbol = "waveform",
                .outputs = {out("level", "Level", number, "u.audio.x"), out("bass", "Bass", number, "u.audio.y"),
                            out("mids", "Mids", number, "u.audio.z"), out("treble", "Treble", number, "u.audio.w")}}),
    }};
}

} // namespace editor::shaders
