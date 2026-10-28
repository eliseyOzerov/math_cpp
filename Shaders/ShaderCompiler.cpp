#include "ShaderCompiler.h"

#include <cmath>
#include <cstdio>

#include "MetalBackend.h"
#include "ShaderCompilation.h"

namespace editor::shaders {

namespace {

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// Compiles `graph`; `chooseColor` finds the value to draw, and may throw a GraphEditError, which
/// becomes the shader's `error` (the source then draws a dark red).
template <typename ChooseColor>
CompiledShader compileGraph(const ShaderGraph& graph, ChooseColor chooseColor) {
    Compilation compiler(graph);
    try {
        Value color = chooseColor(compiler);
        std::optional<Value> feedback;
        const GraphNode* end = nullptr;
        for (const GraphNode& candidate : graph.nodes) {
            if (end == nullptr && candidate.kind == kinds::feedbackOutput) end = &candidate;
        }
        if (end != nullptr) {
            if (const GraphConnection* wire = graph.connection(end->id, "color")) {
                Value value = compiler.output(wire->sourceNode, wire->sourcePort);
                if (value.type == SocketType::texture) throw GraphEditError("Feedback Output expects a color.");
                feedback = value.converted(SocketType::color);
            }
        }
        const std::vector<std::string>& body = compiler.lines();
        std::string text = color.code + (feedback ? feedback->code : "");
        for (const std::string& line : body) text += line + "\n";
        bool animated = false;
        for (const char* name : {"u.time", "u.deltaTime", "u.frame", "u.date", "u.audio", "u.seed"}) animated = animated || contains(text, name);
        bool interactive = contains(text, "u.pointer") || contains(text, "u.clickCount");
        for (const ShaderTexture& texture : compiler.textures()) {
            if (texture.feed != TextureFeed::image && texture.feed != TextureFeed::noise && texture.feed != TextureFeed::blueNoise) animated = true;
            if (texture.feed == TextureFeed::controls) interactive = true;
        }
        CompiledShader shader;
        shader.source = metalBackend::shaderSource(body, color.code, feedback ? std::optional<std::string>(feedback->code) : std::nullopt,
                                                   static_cast<int>(compiler.textures().size()));
        shader.textures = compiler.textures();
        shader.usesTime = animated;
        shader.usesPointer = interactive;
        shader.writesFeedback = feedback.has_value();
        return shader;
    } catch (const GraphEditError& problem) {
        CompiledShader shader;
        shader.source = metalBackend::shaderSource({}, "float4(0.22f, 0.03f, 0.05f, 1.0f)", std::nullopt, 0);
        shader.error = problem.message;
        return shader;
    }
}

} // namespace

CompiledShader ShaderCompiler::compile(const ShaderGraph& graph) {
    return compileGraph(graph, [&](Compilation& compiler) {
        const GraphNode* shader = graph.outputNode();
        if (shader == nullptr) throw GraphEditError("The graph has no Shader node.");
        const GraphConnection* wire = graph.connection(shader->id, "color");
        if (wire == nullptr) return Value{"float4(0.431373f, 0.352941f, 0.635294f, 1.0f)", SocketType::color, {}};
        Value color = compiler.output(wire->sourceNode, wire->sourcePort);
        if (color.type == SocketType::texture) throw GraphEditError("Shader expects a color input.");
        return color.converted(SocketType::color);
    });
}

CompiledShader ShaderCompiler::compile(const ShaderGraph& graph, const std::string& previewing, const std::string& port) {
    return compileGraph(graph, [&](Compilation& compiler) {
        Value value = compiler.output(previewing, port);
        switch (value.type) {
            case SocketType::texture: return Value{"sampleImage(" + value.code + ", linearSampler, vUv)", SocketType::color, {}};
            case SocketType::number:
            case SocketType::any: return Value{"float4(float3(" + value.code + "), 1.0f)", SocketType::color, {}};
            case SocketType::vector2: return Value{"float4(" + value.code + ", 0.0f, 1.0f)", SocketType::color, {}};
            case SocketType::vector3: return Value{"float4(" + value.code + ", 1.0f)", SocketType::color, {}};
            case SocketType::vector4: return Value{"float4((" + value.code + ").xyz, 1.0f)", SocketType::color, {}};
            case SocketType::color: return value;
        }
        return value;
    });
}

std::string ShaderCompiler::literal(double value) {
    if (!std::isfinite(value)) return "0.0f";
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.8g", value);
    std::string text = buffer;
    bool hasPoint = text.find_first_of(".eE") != std::string::npos;
    return (hasPoint ? text : text + ".0") + "f";
}

} // namespace editor::shaders
