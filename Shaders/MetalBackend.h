// The Metal backend of the shader compiler: everything that is Metal Shading Language rather than
// graph logic. The uniforms struct, the vertex function, the fragment function's wrapper and the
// texture arguments live here, so an HLSL backend can sit beside this file later. The node
// templates and the converters in ShaderCompilation.cpp still write Metal-style `float2` types;
// that is also what HLSL's `float2` looks like.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "ShaderGraph.h"

namespace editor::shaders::metalBackend {

/// The Metal type of a value of `type`: float, float2, float3, float4 (colours too), or
/// `texture2d<float>`.
std::string typeName(SocketType type);

/// The complete Metal source of one shader: the prelude (uniforms, vertex function), the shader
/// library, the helpers that take a function (`loopAccumulate`) and the fragment function whose
/// body is `body` and which returns `color` clamped to 0..1, plus `feedback` at `[[color(1)]]`
/// when there is one. `textureCount` textures are bound at indices 0 and up.
std::string shaderSource(const std::vector<std::string>& body, const std::string& color,
                         const std::optional<std::string>& feedback, int textureCount);

} // namespace editor::shaders::metalBackend
