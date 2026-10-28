// Compiles a shader graph to Metal Shading Language (graphics_metal's Compiler/ShaderCompiler.swift):
// one fragment function that reads the uniforms and textures and returns the colour wired into
// the Shader node, after the shader library (the shared kernels). UV has its origin at the bottom
// left, as in GLSL.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "NodeSpec.h"
#include "ShaderGraph.h"

namespace editor::shaders {

/// A texture a compiled shader samples, bound at `index`.
struct ShaderTexture {
    int index = 0;
    TextureFeed feed = TextureFeed::image;
    /// An Image or Video node's file, absolute or relative to the project folder; nothing when it has none.
    std::optional<std::string> path;

    /// The file of an Image node.
    std::optional<std::string> imagePath() const {
        return feed == TextureFeed::image ? path : std::nullopt;
    }

    bool operator==(const ShaderTexture& other) const = default;
};

/// A graph compiled to Metal, or why it could not be.
///
/// Use when: handing a shader to the platform's Metal compiler; drawing a node's preview
/// Tags: compiled shader, Metal source, shader output
/// Node: none (shader graph)
struct CompiledShader {
    /// The whole Metal source: uniforms, the library, the vertex and the fragment function.
    std::string source;
    std::vector<ShaderTexture> textures;
    /// Whether the picture changes over time, so the view has to keep drawing.
    bool usesTime = false;
    bool usesPointer = false;
    /// Whether the fragment writes a second colour, at `[[color(1)]]`, for next frame's Previous
    /// Frame; the pipeline then needs a second attachment.
    bool writesFeedback = false;
    /// Why the graph could not compile; the source then draws a dark red.
    std::optional<std::string> error;

    /// Whether some texture is the last frame, so the host keeps the frames it draws.
    bool readsPreviousFrame() const {
        for (const ShaderTexture& texture : textures) {
            if (texture.feed == TextureFeed::previousFrame) return true;
        }
        return false;
    }
};

/// Compiles shader graphs.
///
/// Use when: turning a graph into Metal source
/// Tags: shader compiler, graph to Metal, code generation
/// Node: none (shader graph)
class ShaderCompiler {
public:
    /// The compiled source's vertex function: a full-screen triangle, drawn with 3 vertices.
    static const char* vertexFunction() { return "graphVertex"; }
    /// The compiled source's fragment function.
    static const char* fragmentFunction() { return "graphFragment"; }

    /// Compiles `graph` from its Shader node back through every wire that feeds it.
    static CompiledShader compile(const ShaderGraph& graph);

    /// Compiles `graph` to show output `port` of `node` instead of the Shader's colour, as a
    /// preview on the node: numbers as greys, vectors as red, green and blue, and textures
    /// sampled across the canvas.
    static CompiledShader compile(const ShaderGraph& graph, const std::string& previewing, const std::string& port);

    /// A number as a Metal float literal: "1.0f", "-0.25f", "1e+20f"; NaN and infinity give "0.0f".
    /// Always a float literal, so the kernels' templates deduce one number type (NodeSpec.h).
    static std::string literal(double value);
};

} // namespace editor::shaders
