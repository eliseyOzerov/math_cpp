// Every kind of node, gathered from the families in `Nodes/` (graphics_metal's
// Graph/NodeCatalog.swift). A family is one file in `Nodes/` that returns a `NodeGroup`; it is
// listed once, in `Nodes/NodeFamilies.h`.
#pragma once

#include <string>
#include <vector>

#include "NodeSpec.h"
#include "ShaderGraph.h"

namespace editor::shaders {

/// One channel of a colour space, with the range its slider spans.
struct ColorChannel {
    std::string id;
    std::string label;
    double lowest = 0;
    double highest = 1;
    double defaultValue = 0;
};

/// A heading of the add-node menu and the kinds under it.
struct NodeMenuGroup {
    std::string id;
    std::string label;
    std::vector<NodeKind> kinds;
};

/// The catalog of node kinds.
///
/// Use when: looking up what a kind takes and gives; filling the add-node menu
/// Tags: node catalog, node library, kinds, definitions
/// Node: none (shader graph)
class NodeCatalog {
public:
    /// Every kind, in menu order.
    static const std::vector<NodeDefinition>& all();

    /// Whether `name` is a kind.
    static bool knows(const std::string& name);

    /// The definition of `kind`; throws a GraphEditError for a name that is not a kind.
    static const NodeDefinition& definition(const NodeKind& kind);
    static const NodeDefinition& definition(const std::string& name);

    /// The input sockets of `node`; a Color node's are its colour space's channels.
    static std::vector<NodePort> inputs(const GraphNode& node);
    static std::vector<NodePort> outputs(const GraphNode& node);

    static std::map<std::string, double> defaultParameters(const NodeKind& kind, ColorSpace space = ColorSpace::rgb);
    static std::vector<ColorChannel> channels(ColorSpace space);

    /// The range a number field drags across, where the input has a natural one.
    static Range range(const std::string& port, const GraphNode& node);

    static std::vector<NodeMenuGroup> menuGroups();
};

} // namespace editor::shaders
