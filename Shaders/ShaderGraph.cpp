#include "ShaderGraph.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <set>
#include <sstream>

#include "NodeCatalog.h"

namespace editor::shaders {

std::string socketTypeName(SocketType type) {
    switch (type) {
        case SocketType::number: return "number";
        case SocketType::vector2: return "vector2";
        case SocketType::vector3: return "vector3";
        case SocketType::vector4: return "vector4";
        case SocketType::color: return "color";
        case SocketType::texture: return "texture";
        case SocketType::any: return "any";
    }
    return "";
}

const std::vector<SocketType>& allSocketTypes() {
    static const std::vector<SocketType> all = {SocketType::number, SocketType::vector2, SocketType::vector3,
        SocketType::vector4, SocketType::color, SocketType::texture, SocketType::any};
    return all;
}

bool canFeed(SocketType from, SocketType to) { return (from == SocketType::texture) == (to == SocketType::texture); }

bool isNumeric(SocketType type) { return type == SocketType::number || type == SocketType::any; }

// Fallbacks. Every number is a float literal; see NodeSpec.h.
Fallback Fallback::uv() { return {"vUv", "UV", std::nullopt}; }
Fallback Fallback::centred() { return {"centredUv", "centred UV", std::nullopt}; }
Fallback Fallback::zero() { return {"0.0f", "0", std::nullopt}; }
Fallback Fallback::one() { return {"1.0f", "1", std::nullopt}; }
Fallback Fallback::half() { return {"0.5f", "0.5", std::nullopt}; }
Fallback Fallback::black() { return {"float4(0.0f, 0.0f, 0.0f, 1.0f)", "black", std::nullopt}; }
Fallback Fallback::white() { return {"float4(1.0f)", "white", std::nullopt}; }
Fallback Fallback::gray() { return {"float4(0.5f, 0.5f, 0.5f, 1.0f)", "grey", std::nullopt}; }
Fallback Fallback::clear() { return {"float4(0.0f)", "clear", std::nullopt}; }
Fallback Fallback::up() { return {"float3(0.0f, 1.0f, 0.0f)", "up", std::nullopt}; }
Fallback Fallback::forward() { return {"float3(0.0f, 0.0f, 1.0f)", "forward", std::nullopt}; }
Fallback Fallback::sun() { return {"normalize(float3(0.6f, 0.8f, -0.4f))", "sun", std::nullopt}; }
Fallback Fallback::marchPoint() { return {"marchPoint", "march point", "marchPoint"}; }
Fallback Fallback::rayOrigin() { return {"float3(0.0f, 0.0f, -3.0f)", "camera", std::nullopt}; }
Fallback Fallback::rayDirection() { return {"normalize(float3(centredUv, 1.5f))", "camera ray", std::nullopt}; }
Fallback Fallback::time() { return {"u.time", "time", std::nullopt}; }
Fallback Fallback::middle() { return {"float2(0.5f)", "centre", std::nullopt}; }
Fallback Fallback::loopValue() { return {"loopValue", "last value", "loopValue"}; }

std::string colorSpaceName(ColorSpace space) {
    switch (space) {
        case ColorSpace::rgb: return "rgb";
        case ColorSpace::cmyk: return "cmyk";
        case ColorSpace::oklab: return "oklab";
        case ColorSpace::okhsv: return "okhsv";
    }
    return "";
}

const std::vector<ColorSpace>& allColorSpaces() {
    static const std::vector<ColorSpace> all = {ColorSpace::rgb, ColorSpace::cmyk, ColorSpace::oklab, ColorSpace::okhsv};
    return all;
}

std::optional<NodeKind> NodeKind::named(const std::string& name) {
    if (!NodeCatalog::knows(name)) return std::nullopt;
    return NodeKind{name};
}

std::vector<NodeKind> NodeKind::allCases() {
    std::vector<NodeKind> result;
    for (const NodeDefinition& definition : NodeCatalog::all()) result.push_back(definition.kind);
    return result;
}

NestedGraph::NestedGraph() : inner(std::make_unique<ShaderGraph>()) {}
NestedGraph::NestedGraph(const ShaderGraph& graph) : inner(std::make_unique<ShaderGraph>(graph)) {}
NestedGraph::NestedGraph(const NestedGraph& other) : inner(std::make_unique<ShaderGraph>(*other.inner)) {}
NestedGraph& NestedGraph::operator=(const NestedGraph& other) {
    if (this != &other) inner = std::make_unique<ShaderGraph>(*other.inner);
    return *this;
}
NestedGraph::NestedGraph(NestedGraph&&) noexcept = default;
NestedGraph& NestedGraph::operator=(NestedGraph&&) noexcept = default;
NestedGraph::~NestedGraph() = default;

namespace {

/// A number as text the way a sentence wants it: "inf", "nan" or the shortest decimal.
std::string numberText(double value) {
    std::ostringstream text;
    text << value;
    return text.str();
}

} // namespace

const GraphNode* ShaderGraph::node(const std::string& id) const {
    for (const GraphNode& candidate : nodes) {
        if (candidate.id == id) return &candidate;
    }
    return nullptr;
}

GraphNode* ShaderGraph::node(const std::string& id) {
    return const_cast<GraphNode*>(static_cast<const ShaderGraph*>(this)->node(id));
}

std::vector<NodePort> ShaderGraph::inputs(const GraphNode& node) const { return NodeCatalog::inputs(node); }

std::vector<NodePort> ShaderGraph::outputs(const GraphNode& node) const { return NodeCatalog::outputs(node); }

const GraphConnection* ShaderGraph::connection(const std::string& node, const std::string& port) const {
    for (const GraphConnection& wire : connections) {
        if (wire.targetNode == node && wire.targetPort == port) return &wire;
    }
    return nullptr;
}

const GraphNode* ShaderGraph::outputNode() const {
    for (const GraphNode& candidate : nodes) {
        if (candidate.kind == kinds::shader) return &candidate;
    }
    return nullptr;
}

std::string ShaderGraph::freshID(const NodeKind& kind) const {
    // "sine" stays "sine"; "map-range" becomes "mapRange".
    std::string stem;
    bool capitalizeNext = false;
    for (char letter : kind.rawValue) {
        if (letter == '-') {
            capitalizeNext = true;
            continue;
        }
        stem += capitalizeNext ? static_cast<char>(std::toupper(static_cast<unsigned char>(letter))) : letter;
        capitalizeNext = false;
    }
    int index = 1;
    while (node(stem + std::to_string(index)) != nullptr) index += 1;
    return stem + std::to_string(index);
}

GraphNode ShaderGraph::addNode(const NodeKind& kind, CanvasPoint position, const std::string& requestedID) {
    if (kind == kinds::shader && outputNode() != nullptr) throw GraphEditError("The graph already has a Shader output node.");
    std::string id = requestedID.empty() ? freshID(kind) : requestedID;
    if (node(id) != nullptr) throw GraphEditError("A node named " + id + " already exists.");
    const NodeDefinition& definition = NodeCatalog::definition(kind);
    GraphNode created;
    created.id = id;
    created.kind = kind;
    created.x = position.x;
    created.y = position.y;
    created.parameters = NodeCatalog::defaultParameters(kind, ColorSpace::rgb);
    if (kind == kinds::color) created.colorSpace = ColorSpace::rgb;
    if (definition.text) created.text = definition.text->initial;
    if (kind == kinds::subgraph) created.subgraph = NestedGraph(emptyGroup());
    nodes.push_back(created);
    return created;
}

void ShaderGraph::removeNode(const std::string& id) {
    if (node(id) == nullptr) throw GraphEditError("There is no node " + id + ".");
    std::erase_if(nodes, [&](const GraphNode& candidate) { return candidate.id == id; });
    std::erase_if(connections, [&](const GraphConnection& wire) { return wire.sourceNode == id || wire.targetNode == id; });
}

GraphConnection ShaderGraph::connect(const std::string& sourceNode, const std::string& sourcePort,
                                     const std::string& targetNode, const std::string& targetPort) {
    const GraphNode* source = node(sourceNode);
    if (source == nullptr) throw GraphEditError("There is no node " + sourceNode + ".");
    const GraphNode* target = node(targetNode);
    if (target == nullptr) throw GraphEditError("There is no node " + targetNode + ".");
    std::optional<NodePort> output;
    for (const NodePort& port : outputs(*source)) {
        if (port.id == sourcePort) output = port;
    }
    if (!output) throw GraphEditError(sourceNode + " has no output " + sourcePort + ".");
    std::optional<NodePort> input;
    for (const NodePort& port : inputs(*target)) {
        if (port.id == targetPort) input = port;
    }
    if (!input) throw GraphEditError(targetNode + " has no input " + targetPort + ".");
    if (!canFeed(output->type, input->type)) {
        throw GraphEditError(sourceNode + "." + sourcePort + " is a " + socketTypeName(output->type) + ", but "
                             + targetNode + "." + targetPort + " takes a " + socketTypeName(input->type) + ".");
    }
    if (sourceNode == targetNode || reaches(targetNode, sourceNode)) throw GraphEditError("That wire would make a loop.");
    std::erase_if(connections, [&](const GraphConnection& wire) {
        return wire.targetNode == targetNode && wire.targetPort == targetPort;
    });
    GraphConnection wire{sourceNode + "." + sourcePort + "->" + targetNode + "." + targetPort,
                         sourceNode, sourcePort, targetNode, targetPort};
    connections.push_back(wire);
    return wire;
}

void ShaderGraph::disconnect(const std::string& nodeID, const std::string& port) {
    if (connection(nodeID, port) == nullptr) throw GraphEditError("Nothing is connected to " + nodeID + "." + port + ".");
    std::erase_if(connections, [&](const GraphConnection& wire) { return wire.targetNode == nodeID && wire.targetPort == port; });
}

void ShaderGraph::setParameter(const std::string& nodeID, const std::string& parameter, double value) {
    GraphNode* target = node(nodeID);
    if (target == nullptr) throw GraphEditError("There is no node " + nodeID + ".");
    std::set<std::string> known;
    for (const NodePort& port : inputs(*target)) {
        if (port.isDraggable()) known.insert(port.id);
    }
    for (const auto& entry : NodeCatalog::definition(target->kind).defaults) known.insert(entry.first);
    if (known.count(parameter) == 0) {
        std::string list;
        for (const std::string& name : known) list += (list.empty() ? "" : ", ") + name;
        throw GraphEditError(nodeID + " has no number " + parameter + "; it has " + list + ".");
    }
    if (!std::isfinite(value)) throw GraphEditError(numberText(value) + " is not a finite number.");
    target->parameters[parameter] = value;
}

void ShaderGraph::setColorSpace(const std::string& nodeID, ColorSpace space) {
    GraphNode* target = node(nodeID);
    if (target == nullptr || target->kind != kinds::color) throw GraphEditError(nodeID + " is not a Color node.");
    target->colorSpace = space;
    target->parameters = NodeCatalog::defaultParameters(kinds::color, space);
    std::set<std::string> channels;
    for (const NodePort& port : inputs(*target)) channels.insert(port.id);
    std::erase_if(connections, [&](const GraphConnection& wire) {
        return wire.targetNode == nodeID && channels.count(wire.targetPort) == 0;
    });
}

void ShaderGraph::setText(const std::string& nodeID, const std::string& text) {
    GraphNode* target = node(nodeID);
    if (target == nullptr) throw GraphEditError("There is no node " + nodeID + ".");
    if (!NodeCatalog::definition(target->kind).text) throw GraphEditError(nodeID + " holds no text.");
    target->text = text;
}

void ShaderGraph::setOption(const std::string& nodeID, const std::string& optionID, int choice) {
    GraphNode* target = node(nodeID);
    if (target == nullptr) throw GraphEditError("There is no node " + nodeID + ".");
    const std::vector<NodeOption>& options = NodeCatalog::definition(target->kind).options;
    const NodeOption* found = nullptr;
    for (const NodeOption& candidate : options) {
        if (candidate.id == optionID) found = &candidate;
    }
    if (found == nullptr) {
        std::string list;
        for (const NodeOption& candidate : options) list += (list.empty() ? "" : ", ") + candidate.id;
        throw GraphEditError(nodeID + " has no option " + optionID + (options.empty() ? "" : "; it has " + list) + ".");
    }
    if (choice < 0 || choice >= static_cast<int>(found->choices.size())) {
        std::string list;
        for (const std::string& name : found->choices) list += (list.empty() ? "" : ", ") + name;
        throw GraphEditError(optionID + " has choices 0 to " + std::to_string(found->choices.size() - 1) + ": " + list + ".");
    }
    target->parameters[optionID] = choice;
}

void ShaderGraph::moveNode(const std::string& nodeID, CanvasPoint position) {
    GraphNode* target = node(nodeID);
    if (target == nullptr) return;
    target->x = position.x;
    target->y = position.y;
}

bool ShaderGraph::reaches(const std::string& start, const std::string& end) const {
    std::vector<std::string> stack = {start};
    std::set<std::string> seen;
    while (!stack.empty()) {
        std::string current = stack.back();
        stack.pop_back();
        if (current == end) return true;
        if (!seen.insert(current).second) continue;
        for (const GraphConnection& wire : connections) {
            if (wire.sourceNode == current) stack.push_back(wire.targetNode);
        }
    }
    return false;
}

ShaderGraph ShaderGraph::emptyGroup() {
    ShaderGraph graph;
    graph.addNode(kinds::groupInput, {40, 60}, "groupInput1");
    graph.addNode(kinds::groupOutput, {480, 60}, "groupOutput1");
    return graph;
}

ShaderGraph ShaderGraph::starter() {
    ShaderGraph graph;
    graph.addNode(kinds::uv, {40, 60}, "uv1");
    graph.addNode(kinds::separateVector2, {260, 60}, "separateVector21");
    graph.addNode(kinds::time, {40, 220}, "time1");
    graph.addNode(kinds::add, {480, 60}, "add1");
    graph.addNode(kinds::sine, {480, 240}, "sine1");
    graph.addNode(kinds::mapRange, {700, 240}, "mapRange1");
    graph.addNode(kinds::multiply, {700, 60}, "multiply1");
    graph.addNode(kinds::color, {920, 60}, "color1");
    graph.addNode(kinds::shader, {1140, 60}, "shader1");
    graph.setColorSpace("color1", ColorSpace::okhsv);
    graph.setParameter("multiply1", "b", 120);
    graph.setParameter("mapRange1", "fromMin", -1);
    graph.setParameter("mapRange1", "toMin", 0.6);
    graph.setParameter("mapRange1", "toMax", 0.95);
    graph.connect("uv1", "uv", "separateVector21", "vector");
    graph.connect("separateVector21", "x", "add1", "a");
    graph.connect("time1", "seconds", "add1", "b");
    graph.connect("time1", "seconds", "sine1", "radians");
    graph.connect("sine1", "value", "mapRange1", "value");
    graph.connect("mapRange1", "value", "color1", "v");
    graph.connect("add1", "value", "multiply1", "a");
    graph.connect("multiply1", "value", "color1", "h");
    graph.connect("color1", "color", "shader1", "color");
    return graph;
}

} // namespace editor::shaders
