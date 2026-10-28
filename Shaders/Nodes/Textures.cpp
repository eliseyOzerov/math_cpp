// Textures the host supplies, and reading them (graphics_metal's Nodes/Textures.swift).
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

NodeGroup textureNodes() {
    NodeBuilder c(NodeCategory::textures);
    return {c.category, {
        c.node({.name = "image", .label = "Image", .summary = "An image file, as a texture.", .symbol = "photo",
                .outputs = {out("texture", "Texture", texture)}, .feed = TextureFeed::image}),
        c.node({.name = "video", .label = "Video", .summary = "A video file playing on a loop, as a texture.", .symbol = "film",
                .outputs = {out("texture", "Texture", texture)}, .feed = TextureFeed::video}),
        c.node({.name = "camera", .label = "Camera", .summary = "The camera, as a texture.", .symbol = "camera",
                .outputs = {out("texture", "Texture", texture)}, .feed = TextureFeed::camera}),
        c.node({.name = "audio", .label = "Audio Spectrum",
                .summary = "The microphone as a texture, and its magnitude and waveform at a band.",
                .symbol = "waveform.circle", .inputs = {num("band", "Band", 0.1, between(0, 1))},
                .outputs = {out("texture", "Texture", texture, "{feed}"),
                            out("magnitude", "Magnitude", number,
                                "{feed}.read(uint2(uint(clamp({band}, 0.0f, 1.0f) * 255.0f), 0u)).r"),
                            out("waveform", "Waveform", number,
                                "{feed}.read(uint2(uint(clamp({band}, 0.0f, 1.0f) * 255.0f), 0u)).g")},
                .feed = TextureFeed::audioSpectrum}),
        c.node({.name = "previous-frame", .label = "Previous Frame",
                .summary = "What the graph drew last frame, for trails and simulations.", .symbol = "arrow.uturn.backward",
                .outputs = {out("texture", "Texture", texture)}, .feed = TextureFeed::previousFrame}),
        c.node({.name = "noise-texture", .label = "Noise Texture",
                .summary = "256 × 256 of repeating white noise, four independent channels.", .symbol = "aqi.low",
                .outputs = {out("texture", "Texture", texture, "{feed}"),
                            out("pixel", "Pixel", color,
                                "{feed}.read(uint2(fragCoord) % uint2({feed}.get_width(), {feed}.get_height()))")},
                .feed = TextureFeed::noise}),
        c.node({.name = "blue-noise", .label = "Blue Noise", .summary = "64 × 64 of repeating blue noise, for dithering.",
                .symbol = "aqi.medium",
                .outputs = {out("texture", "Texture", texture, "{feed}"),
                            out("pixel", "Pixel", color,
                                "{feed}.read(uint2(fragCoord) % uint2({feed}.get_width(), {feed}.get_height()))")},
                .feed = TextureFeed::blueNoise}),
        c.node({.name = "sample-texture", .label = "Sample Texture", .summary = "A texture's colour at some coordinates.",
                .symbol = "eyedropper", .inputs = {tex(), vec2("uv", "UV"), num("level", "Mip Level", 0, between(0, 12))},
                .outputs = {out("color", "Color", color, "sampleImageWith({texture}, {uv}, {#wrap}, {#filter}, {level})")},
                .options = {option("wrap", "Edges", {"Clamp", "Repeat", "Mirror"}),
                            option("filter", "Filter", {"Linear", "Nearest"})}}),
        c.node({.name = "texture-size", .label = "Texture Size", .summary = "A texture's width and height in pixels.",
                .symbol = "arrow.up.left.and.arrow.down.right", .inputs = {tex()},
                .outputs = {out("size", "Size", vector2, "float2({texture}.get_width(), {texture}.get_height())")}}),
        c.node({.name = "triplanar", .label = "Triplanar",
                .summary = "A texture projected onto a 3D surface from three sides, blended by its normal.",
                .symbol = "cube.transparent",
                .inputs = {tex(), vec3("position", "Position"), vec3("normal", "Normal", Fallback::up()),
                           num("scale", "Scale", 1), num("sharpness", "Sharpness", 4, between(1, 16))},
                .outputs = {out("color", "Color", color, "triplanar({texture}, {position}, {normal}, {scale}, {sharpness})")}}),
    }};
}

} // namespace editor::shaders
