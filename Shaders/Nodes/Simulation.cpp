// Simulations that step once a frame (graphics_metal's Nodes/Simulation.swift). Each reads the state
// from Previous Frame and gives the next state as a colour: wire it into Feedback Output, and show the
// state's View output. A Previous Frame that has never been written is clear (alpha 0), which every
// step reads as "begin". Particles are the exception: with no compute pass they are a function of
// index and time.
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

namespace {

const Fallback pointerUv{"(u.pointer.xy / max(u.resolution, float2(1.0f)))", "pointer", std::nullopt};
const Fallback pointerPressed{"u.pointer.z", "pressed", std::nullopt};
const Fallback pointerPush{"(u.pointerDelta / max(u.resolution, float2(1.0f)) * float2(1.0f, -1.0f))", "pointer movement", std::nullopt};

/// A number that reads `fallback` until something is wired in, as the pointer's pressed state does.
InputSpec numberFrom(const std::string& id, const std::string& label, const Fallback& fallback) {
    return InputSpec{NodePort{id, label, number, fallback, std::nullopt}, std::nullopt, std::nullopt};
}

} // namespace

NodeGroup simulationNodes() {
    NodeBuilder c(NodeCategory::simulation);
    const std::string reactionStep =
        "reactionDiffusionStep({state}, {uv}, {feedRate}, {kill}, {diffusionA}, {diffusionB}, {rate}, {brush}, {brushing})";
    const std::string lifeNext = "lifeStep({state}, {uv}, {cell}, {interval}, {frame}, {density}, {fade})";
    const std::string fluidNext =
        "fluidStep({state}, {uv}, {source}, {push}, {force}, {radius}, {viscosity}, {damping}, {dissipation})";
    const std::string particles =
        "particleField({uv}, float2(u.resolution.x / u.resolution.y, 1.0f), {count}, {size}, {speed}, {time}, {seed}, {#motion}, {spread})";
    return {c.category, {
        c.node({.name = "reaction-diffusion", .label = "Reaction-Diffusion",
                .summary = "Gray-Scott: spots, stripes and coral from two chemicals; the pointer drops B.",
                .symbol = "circle.hexagongrid.fill",
                .inputs = {tex("state", "State"), vec2("uv", "UV"), num("feedRate", "Feed", 0.055, between(0.01, 0.1)),
                           num("kill", "Kill", 0.062, between(0.04, 0.07)), num("diffusionA", "Diffusion A", 1, between(0, 1)),
                           num("diffusionB", "Diffusion B", 0.5, between(0, 1)), num("rate", "Rate", 1, between(0, 1.5)),
                           vec2("brush", "Brush", pointerUv), numberFrom("brushing", "Brushing", pointerPressed),
                           col("low", "Low", Fallback::black()), col("high", "High", Fallback::white())},
                .outputs = {out("state", "State", color, reactionStep),
                            out("view", "View", color, "grayScottViewColor(" + reactionStep + ", {low}.rgb, {high}.rgb)")}}),
        c.node({.name = "game-of-life", .label = "Game of Life",
                .summary = "Conway's cellular automaton on a grid of cells, with fading trails.", .symbol = "square.grid.3x3.square",
                .inputs = {tex("state", "State"), vec2("uv", "UV"), num("cell", "Cell Size", 6, between(1, 32)),
                           num("interval", "Frames per Step", 6, between(1, 60)),
                           numberFrom("frame", "Frame", Fallback{"u.frame", "frame", std::nullopt}),
                           num("density", "Start Density", 0.3, between(0, 1)), num("fade", "Trail Fade", 0.9, between(0, 1)),
                           col("alive", "Alive", Fallback::white()),
                           col("dead", "Dead", Fallback{"float4(0.02f, 0.03f, 0.06f, 1.0f)", "dark", std::nullopt})},
                .outputs = {out("state", "State", color, lifeNext),
                            out("view", "View", color, "lifeViewColor(" + lifeNext + ", {alive}.rgb, {dead}.rgb)")}}),
        c.node({.name = "fluid", .label = "Fluid",
                .summary = "Advects velocity and dye with viscosity and divergence damping (not a full pressure solve); the pointer stirs it.",
                .symbol = "drop.fill",
                .inputs = {tex("state", "State"), vec2("uv", "UV"), vec2("source", "Source", pointerUv),
                           vec2("push", "Push", pointerPush), num("force", "Force", 12, between(0, 50)),
                           num("radius", "Radius", 0.05, between(0.005, 0.3)), num("viscosity", "Viscosity", 0.05, between(0, 1)),
                           num("damping", "Divergence Damping", 0.5, between(0, 1)),
                           num("dissipation", "Dye Persistence", 0.99, between(0.8, 1)),
                           col("dye", "Dye", Fallback{"float4(0.2f, 0.7f, 1.0f, 1.0f)", "blue", std::nullopt})},
                .outputs = {out("state", "State", color, fluidNext),
                            out("view", "View", color, "fluidViewColor(" + fluidNext + ", {dye}.rgb)")}}),
        c.node({.name = "particles", .label = "Particles",
                .summary = "Dots whose positions are a function of their index and the time (no stored state): drift, rise, orbit or burst.",
                .symbol = "sparkles",
                .inputs = {vec2("uv", "UV"), timeIn(), num("count", "Count", 48, between(1, 64)),
                           num("size", "Size", 0.02, between(0.002, 0.1)), num("speed", "Speed", 1, between(0, 5)),
                           num("spread", "Spread", 1, between(0, 2)), num("seed", "Seed", 0, between(0, 100))},
                .outputs = {out("color", "Color", color, "float4(" + particles + ".rgb, 1.0f)"),
                            out("density", "Density", number, particles + ".a")},
                .options = {option("motion", "Motion", {"Drift", "Rise", "Orbit", "Burst"}, 1)}}),
    }};
}

} // namespace editor::shaders
