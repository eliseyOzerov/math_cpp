// The uniforms every compiled shader reads at buffer 0, laid out as the Metal `Uniforms` struct
// in MetalBackend.cpp (graphics_metal's ShaderUniforms).
#pragma once

#include <cstddef>

namespace editor::shaders {

/// The values a host fills each frame and binds at fragment buffer 0. The alignment matches
/// Metal's: a float2 sits on 8 bytes, a float4 on 16.
///
/// Use when: drawing a compiled shader
/// Tags: uniforms, shader constants, time, resolution, pointer
/// Node: none (shader graph)
struct ShaderUniforms {
    alignas(8) float resolution[2] = {0, 0};
    float time = 0;
    float deltaTime = 0;
    /// Pixels from the bottom left, then whether it is pressed.
    alignas(16) float pointer[4] = {0, 0, 0, 0};
    float frame = 0;
    float frameRate = 60;
    /// How far the pointer moved since the last frame, in pixels.
    alignas(8) float pointerDelta[2] = {0, 0};
    /// How many times the canvas has been clicked.
    float clickCount = 0;
    /// A number fixed for the session, for seeding random nodes.
    float seed = 0;
    /// Year, month (1 to 12), day (1 to 31), and seconds since midnight.
    alignas(16) float date[4] = {0, 0, 0, 0};
    /// The microphone: overall level, then bass, mids and treble, each 0 to about 1.
    alignas(16) float audio[4] = {0, 0, 0, 0};
};

static_assert(offsetof(ShaderUniforms, pointer) == 16, "Uniforms layout must match the Metal struct");
static_assert(offsetof(ShaderUniforms, pointerDelta) == 40, "Uniforms layout must match the Metal struct");
static_assert(offsetof(ShaderUniforms, date) == 64, "Uniforms layout must match the Metal struct");
static_assert(offsetof(ShaderUniforms, audio) == 80, "Uniforms layout must match the Metal struct");
static_assert(sizeof(ShaderUniforms) == 96, "Uniforms layout must match the Metal struct");

} // namespace editor::shaders
