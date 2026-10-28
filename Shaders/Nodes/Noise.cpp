// Noise: smooth, cellular and random fields (graphics_metal's Nodes/Noise.swift). Coordinates
// default to the canvas UV; `scale` is how many features fit across it, and `evolution` moves the
// field through a third dimension. Helper names are the kernels' (Kernels/ShaderNoise.h);
// graphics_metal's name is in each kernel's Tags line.
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

namespace {

NodeOption noiseTypes() { return option("type", "Noise", {"Value", "Perlin", "Simplex"}, 1); }

std::vector<InputSpec> fractalInputs() {
    return {vec2("coordinates", "Coordinates"), num("scale", "Scale", 3, between(0, 50)), num("octaves", "Octaves", 5, between(1, 8)),
            num("lacunarity", "Lacunarity", 2, between(1, 4)), num("gain", "Gain", 0.5, between(0, 1)),
            num("evolution", "Evolution", 0, between(-10, 10))};
}

/// A fractal that differs from FBM only in how it combines its octaves (`mode`).
NodeDefinition fractal(const NodeBuilder& c, const std::string& name, const std::string& label, const std::string& summary, int mode) {
    return c.simple(name, label, summary, fractalInputs(), number,
                    "fractalNoise(float3({coordinates} * {scale}, {evolution}), {#type}, " + std::to_string(mode)
                        + ", {octaves}, {lacunarity}, {gain})",
                    {.options = {noiseTypes()}});
}

/// One noise type (0 value, 1 Perlin, 2 simplex) from -1...1 moved to 0...1.
NodeDefinition basic(const NodeBuilder& c, const std::string& name, const std::string& label, const std::string& summary, int type) {
    return c.simple(name, label, summary,
                    {vec2("coordinates", "Coordinates"), num("scale", "Scale", 6, between(0, 50)),
                     num("evolution", "Evolution", 0, between(-10, 10))},
                    number, "(baseNoise(float3({coordinates} * {scale}, {evolution}), " + std::to_string(type) + ") * 0.5f + 0.5f)");
}

} // namespace

NodeGroup noiseNodes() {
    NodeBuilder c(NodeCategory::noise);
    const NodeOption metrics = option("metric", "Distance", {"Euclidean", "Manhattan", "Chebyshev"});
    return {c.category, {
        basic(c, "value-noise", "Value Noise", "Smooth random values interpolated between a grid of random numbers.", 0),
        basic(c, "perlin-noise", "Perlin Noise", "Smooth gradient noise, 0 to 1.", 1),
        basic(c, "simplex-noise", "Simplex Noise", "Gradient noise on a simplex grid: fewer grid artefacts than Perlin.", 2),
        c.node({.name = "voronoi", .label = "Voronoi",
                .summary = "Cells around random points: distance to the nearest and second nearest point, the cell's id and its edges.",
                .symbol = "circle.hexagongrid",
                .inputs = {vec2("coordinates", "Coordinates"), num("scale", "Scale", 6, between(0.5, 50)),
                           num("randomness", "Randomness", 1, between(0, 1)), num("evolution", "Evolution", 0, between(-10, 10))},
                .outputs = {out("f1", "F1", number, "{$}.nearest"),
                            out("f2", "F2", number, "{$}.second"),
                            out("difference", "F2 − F1", number, "({$}.second - {$}.nearest)"),
                            out("id", "Cell ID", number, "{$}.id"),
                            out("edge", "Edge Distance", number,
                                "cellularBorderDistance({coordinates} * {scale}, {randomness}, {evolution})"),
                            out("point", "Cell Point", vector2, "({$}.nearestPoint / {scale})")},
                .options = {metrics},
                .setup = "cellularNoise({coordinates} * {scale}, {randomness}, {evolution}, {#metric})"}),
        c.simple("crackle", "Crackle", "Cracked-earth lines along the borders between cells.",
                 {vec2("coordinates", "Coordinates"), num("scale", "Scale", 5, between(0.5, 50)), num("width", "Width", 0.15, between(0.01, 1)),
                  num("randomness", "Randomness", 1, between(0, 1)), num("evolution", "Evolution", 0, between(-10, 10))},
                 number,
                 "(1.0f - safeSmoothStep(0.0f, {width}, cellularBorderDistance({coordinates} * {scale}, {randomness}, {evolution})))",
                 {.symbol = "bolt"}),
        c.simple("fbm", "FBM", "Layers of noise at rising frequency and falling strength.", fractalInputs(), number,
                 "fractalNoise(float3({coordinates} * {scale}, {evolution}), {#type}, 0, {octaves}, {lacunarity}, {gain})",
                 {.symbol = "cloud", .options = {noiseTypes()}}),
        fractal(c, "ridged-multifractal", "Ridged Multifractal", "Sharp ridges where the noise crosses zero.", 1),
        fractal(c, "turbulence", "Turbulence", "The sum of the absolute value of each noise layer: billowing, creased.", 2),
        fractal(c, "billow-noise", "Billow Noise", "Soft rounded puffs, like clouds.", 3),
        c.node({.name = "curl-noise", .label = "Curl Noise", .summary = "A flow field that never bunches up, from the curl of noise.",
                .symbol = "hurricane",
                .inputs = {vec2("coordinates", "Coordinates"), num("scale", "Scale", 3, between(0.1, 30)),
                           num("strength", "Strength", 0.5, between(0, 4)), timeIn("evolution", "Evolution")},
                .outputs = {out("flow", "Flow", vector2, "({$} * {strength})"),
                            out("magnitude", "Magnitude", number, "length({$} * {strength})")},
                .setup = "curlNoise({coordinates} * {scale}, {evolution} * 0.3f)"}),
        c.simple("loop-noise", "Gradient Noise 4D", "Gradient noise that loops cleanly: time travels round a circle in the fourth dimension.",
                 {vec2("coordinates", "Coordinates"), num("scale", "Scale", 4, between(0.1, 50)), timeIn(),
                  num("period", "Period", 4, between(0.1, 60)), num("radius", "Loop Radius", 1, between(0.1, 4))},
                 number, "(loopingNoise({coordinates} * {scale}, {time}, {period}, {radius}) * 0.5f + 0.5f)", {.symbol = "infinity"}),
        c.node({.name = "white-noise", .label = "White Noise",
                .summary = "A random number, one per pixel or one per cell. Unrelated neighbours, so it looks like static.",
                .symbol = "aqi.high",
                .inputs = {vec2("coordinates", "Coordinates"), num("cells", "Cells", 16, between(1, 256)), num("seed", "Seed", 0, between(0, 100))},
                .outputs = {out("value", "Value", number, "whiteNoise(fragCoord, {coordinates}, {cells}, {seed}, {#mode})"),
                            out("random", "Random Vector", vector3, "whiteNoiseVector3(fragCoord, {coordinates}, {cells}, {seed}, {#mode})")},
                .options = {option("mode", "Per", {"Pixel", "Cell"}, 1)}}),
        c.node({.name = "domain-warped-fbm", .label = "Domain-Warped FBM", .summary = "FBM bent by FBM twice over, for marbled, smoky swirls.",
                .symbol = "cloud.fog",
                .inputs = {vec2("coordinates", "Coordinates"), num("scale", "Scale", 2, between(0.1, 20)), num("warp", "Warp", 1, between(0, 3)),
                           num("octaves", "Octaves", 4, between(1, 8)), num("lacunarity", "Lacunarity", 2, between(1, 4)),
                           num("gain", "Gain", 0.5, between(0, 1)), num("evolution", "Evolution", 0, between(-10, 10))},
                .outputs = {out("value", "Value", number, "{$}.x"), out("warp", "Warp", vector2, "{$}.yz")},
                .options = {noiseTypes()},
                .setup = "domainWarpedNoise({coordinates} * {scale}, {evolution}, {warp}, {#type}, {octaves}, {lacunarity}, {gain})"}),
        c.simple("flow-noise", "Flow Noise", "Gradient noise whose gradients rotate with time, so detail swirls in place.",
                 {vec2("coordinates", "Coordinates"), num("scale", "Scale", 5, between(0.1, 50)), timeIn("time", "Time"),
                  num("speed", "Speed", 1, between(-8, 8))},
                 number, "(flowNoise({coordinates} * {scale}, {time} * {speed}) * 0.5f + 0.5f)", {.symbol = "wind"}),
        c.simple("gabor-noise", "Gabor Noise", "Sparse windowed waves: control the frequency, direction and how much each wave spreads.",
                 {vec2("coordinates", "Coordinates"), num("scale", "Scale", 4, between(0.5, 30)), num("frequency", "Frequency", 3, between(0.5, 12)),
                  num("bandwidth", "Bandwidth", 1.6, between(0.5, 6)), num("angle", "Angle", 45, between(-360, 360)),
                  num("anisotropy", "Randomness", 0, between(0, 1))},
                 number,
                 "(gaborNoise({coordinates} * {scale}, {frequency}, {bandwidth}, degreesToRadians({angle}), {anisotropy}) * 0.5f + 0.5f)",
                 {.symbol = "water.waves"}),
        c.simple("wavelet-noise", "Wavelet Noise",
                 "Band-limited noise, free of low-frequency blotches (approximated by removing a blur of Perlin noise).",
                 {vec2("coordinates", "Coordinates"), num("scale", "Scale", 8, between(0.5, 60)), num("evolution", "Evolution", 0, between(-10, 10))},
                 number, "(bandLimitedNoise(float3({coordinates} * {scale}, {evolution})) * 0.5f + 0.5f)", {.symbol = "waveform.path"}),
    }};
}

} // namespace editor::shaders
