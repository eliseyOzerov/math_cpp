#include "NodeCatalog.h"

#include <set>

#include "Nodes/NodeFamilies.h"

namespace editor::shaders {

namespace {

std::vector<NodeGroup> groups() {
    std::vector<NodeGroup> result;
#define EDITOR_ADD_FAMILY(name) result.push_back(name());
    EDITOR_SHADER_NODE_FAMILIES(EDITOR_ADD_FAMILY)
#undef EDITOR_ADD_FAMILY
    return result;
}

/// The definitions by kind name, built once; a kind defined twice is a mistake in a family file.
const std::map<std::string, std::size_t>& indexByName() {
    static const std::map<std::string, std::size_t> index = [] {
        std::map<std::string, std::size_t> built;
        const std::vector<NodeDefinition>& definitions = NodeCatalog::all();
        for (std::size_t position = 0; position < definitions.size(); position += 1) {
            bool inserted = built.emplace(definitions[position].kind.rawValue, position).second;
            if (!inserted) throw GraphEditError(definitions[position].kind.rawValue + " is defined twice.");
        }
        return built;
    }();
    return index;
}

} // namespace

const std::vector<NodeDefinition>& NodeCatalog::all() {
    static const std::vector<NodeDefinition> definitions = [] {
        std::vector<NodeDefinition> flat;
        for (NodeGroup& group : groups()) {
            for (NodeDefinition& definition : group.nodes) flat.push_back(std::move(definition));
        }
        return flat;
    }();
    return definitions;
}

bool NodeCatalog::knows(const std::string& name) { return indexByName().count(name) > 0; }

const NodeDefinition& NodeCatalog::definition(const std::string& name) {
    auto found = indexByName().find(name);
    if (found == indexByName().end()) throw GraphEditError(name + " is not a node kind.");
    return all()[found->second];
}

const NodeDefinition& NodeCatalog::definition(const NodeKind& kind) { return definition(kind.rawValue); }

std::vector<NodePort> NodeCatalog::inputs(const GraphNode& node) {
    if (node.kind != kinds::color) return definition(node.kind).inputs;
    std::vector<NodePort> ports;
    for (const ColorChannel& channel : channels(node.colorSpace.value_or(ColorSpace::rgb))) {
        ports.push_back(NodePort{channel.id, channel.label, SocketType::number, std::nullopt,
                                 std::make_pair(channel.lowest, channel.highest)});
    }
    return ports;
}

std::vector<NodePort> NodeCatalog::outputs(const GraphNode& node) { return definition(node.kind).outputs; }

std::map<std::string, double> NodeCatalog::defaultParameters(const NodeKind& kind, ColorSpace space) {
    if (kind != kinds::color) return definition(kind).defaults;
    std::map<std::string, double> parameters;
    for (const ColorChannel& channel : channels(space)) parameters[channel.id] = channel.defaultValue;
    return parameters;
}

std::vector<ColorChannel> NodeCatalog::channels(ColorSpace space) {
    switch (space) {
        case ColorSpace::rgb:
            return {{"r", "R", 0, 1, 0.45}, {"g", "G", 0, 1, 0.35}, {"b", "B", 0, 1, 0.75}};
        case ColorSpace::cmyk:
            return {{"c", "C", 0, 1, 0.4}, {"m", "M", 0, 1, 0.53}, {"y", "Y", 0, 1, 0}, {"k", "K", 0, 1, 0.25}};
        case ColorSpace::oklab:
            return {{"l", "L", 0, 1, 0.55}, {"a", "a", -0.4, 0.4, 0.08}, {"b", "b", -0.4, 0.4, -0.16}};
        case ColorSpace::okhsv:
            return {{"h", "H", 0, 360, 280}, {"s", "S", 0, 1, 0.55}, {"v", "V", 0, 1, 0.75}};
    }
    return {};
}

Range NodeCatalog::range(const std::string& port, const GraphNode& node) {
    for (const NodePort& input : inputs(node)) {
        if (input.id == port) return input.range;
    }
    return std::nullopt;
}

std::vector<NodeMenuGroup> NodeCatalog::menuGroups() {
    std::vector<NodeMenuGroup> result;
    for (NodeCategory category : allCategories()) {
        std::vector<NodeKind> kindsInCategory;
        for (const NodeDefinition& definition : all()) {
            if (definition.category == category && !definition.isHidden()) kindsInCategory.push_back(definition.kind);
        }
        if (!kindsInCategory.empty()) {
            result.push_back(NodeMenuGroup{categoryName(category), categoryLabel(category), kindsInCategory});
        }
    }
    return result;
}

} // namespace editor::shaders
