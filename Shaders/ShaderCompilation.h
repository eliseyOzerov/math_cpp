// One graph's compilation (graphics_metal's Compiler/Compilation.swift): walking back from the
// Shader node through the wires, each node becomes Metal statements, one constant per output,
// filled in from its kind's templates (NodeSpec.h). A few kinds the compiler builds itself: Color
// (its channels depend on its space), texture sources, subgraphs and their ends, named variables
// and Custom Code.
//
// Inputs that are a function of loop variables (`InputScope`) compile into a lambda: while one
// is being built, statements that read its variables go inside it, and the rest are hoisted out
// so they run once per pixel.
#pragma once

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ShaderCompiler.h"

namespace editor::shaders {

/// A compiled value: the Metal naming it, its type, and the loop variables it reads.
struct Value {
    std::string code;
    SocketType type = SocketType::number;
    std::set<std::string> scopes;

    /// This value as `target`: numbers spread, vectors drop or pad components (a colour's alpha
    /// with 1), and a colour read as a number is its luminance.
    Value converted(SocketType target) const;
};

/// The wider of two types, which an `any` node works in; a colour beats a plain Vector 4.
SocketType widerType(SocketType first, SocketType second);

/// `templateText` with each `{...}` placeholder replaced by `value(sign, id)`, where sign is
/// '#', '@', '$' or '\0' for none (the placeholder `{$}` has an empty id).
std::string fillTemplate(const std::string& templateText, const std::function<std::string(char, const std::string&)>& value);

/// A graph being compiled, and where it sits: the root graph, or a subgraph inside a node.
struct GraphContext {
    const ShaderGraph* graph = nullptr;
    /// Prefixes the ids of its nodes in the compilation's caches, so subgraphs' nodes stay apart.
    std::string prefix;
    /// The Subgraph node holding it and the context of the graph that node is in.
    const GraphNode* holder = nullptr;
    const GraphContext* outer = nullptr;
};

/// Statements being collected: the fragment's body, or a lambda's.
struct Frame {
    /// The loop variables its lambda takes; none for the fragment's body.
    std::set<std::string> variables;
    std::vector<std::string> lines;
    /// Compiled outputs, by prefixed node id then port.
    std::map<std::string, std::map<std::string, Value>> cache;
};

class Compilation {
public:
    explicit Compilation(const ShaderGraph& graph);

    /// The body of the fragment function, so far.
    const std::vector<std::string>& lines() const { return frames[0].lines; }
    const std::vector<ShaderTexture>& textures() const { return boundTextures; }
    const GraphContext& rootContext() const { return root; }

    /// The value at `port` of node `nodeID` in the root graph, compiling what it needs.
    Value output(const std::string& nodeID, const std::string& port) { return output(nodeID, port, root, {}); }

private:
    using Outputs = std::map<std::string, Value>;

    Value output(const std::string& nodeID, const std::string& port, const GraphContext& context, std::set<std::string> visiting);
    std::optional<Value> wired(const GraphNode& node, const std::string& port, const GraphContext& context,
                               const std::set<std::string>& visiting);
    std::pair<Outputs, std::size_t> compile(const GraphNode& node, const NodeDefinition& definition,
                                            const GraphContext& context, const std::set<std::string>& visiting);
    std::pair<Outputs, std::size_t> templated(const GraphNode& node, const NodeDefinition& definition,
                                              const std::optional<std::string>& feed, const GraphContext& context,
                                              const std::set<std::string>& visiting);
    Value input(const GraphNode& node, const NodePort& port, SocketType type, const NodeDefinition& definition,
                const GraphContext& context, const std::set<std::string>& visiting);
    Value scopedInput(const GraphNode& node, const NodePort& port, SocketType type, const InputScope& scope,
                      const GraphContext& context, const std::set<std::string>& visiting);
    std::size_t frameIndex(const std::set<std::string>& scopes, const std::string& label) const;
    std::string fresh();

    std::pair<Outputs, std::size_t> colorNode(const GraphNode& node, const GraphContext& context, const std::set<std::string>& visiting);
    std::pair<Outputs, std::size_t> emit(const std::vector<std::pair<std::string, std::pair<std::string, SocketType>>>& outputs,
                                         const std::vector<Value>& reading, const std::string& label);
    std::string textureName(TextureFeed feed, const GraphNode& node, const GraphContext& context);
    std::pair<Outputs, std::size_t> groupInput(const GraphContext& context, const std::set<std::string>& visiting);
    std::pair<Outputs, std::size_t> subgraphNode(const GraphNode& node, const GraphContext& context, const std::set<std::string>& visiting);
    std::pair<Outputs, std::size_t> getVariable(const GraphNode& node, const GraphContext& context, const std::set<std::string>& visiting);
    std::pair<Outputs, std::size_t> customCode(const GraphNode& node, const NodeDefinition& definition,
                                               const GraphContext& context, const std::set<std::string>& visiting);

    GraphContext root;
    std::vector<Frame> frames;
    std::vector<ShaderTexture> boundTextures;
    std::map<std::string, int> textureIndex;
    int counter = 0;
};

} // namespace editor::shaders
