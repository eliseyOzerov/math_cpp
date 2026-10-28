#include "NodeSpec.h"

namespace editor::shaders {

std::string categoryLabel(NodeCategory category) {
    switch (category) {
        case NodeCategory::values: return "Values";
        case NodeCategory::vectors: return "Vectors";
        case NodeCategory::vectorMath: return "Vector Math";
        case NodeCategory::transforms: return "Coordinates & Transforms";
        case NodeCategory::math: return "Math";
        case NodeCategory::trigonometry: return "Trigonometry";
        case NodeCategory::waves: return "Waves & Oscillators";
        case NodeCategory::noise: return "Noise";
        case NodeCategory::patterns: return "Patterns";
        case NodeCategory::shapes: return "Shapes (2D SDF)";
        case NodeCategory::sdfOperations: return "SDF Operations";
        case NodeCategory::color: return "Color";
        case NodeCategory::blending: return "Blending";
        case NodeCategory::textures: return "Textures & Sampling";
        case NodeCategory::imageEffects: return "Image Effects";
        case NodeCategory::time: return "Time & Animation";
        case NodeCategory::interaction: return "Interaction";
        case NodeCategory::raymarching: return "Raymarching & 3D";
        case NodeCategory::simulation: return "Simulation";
        case NodeCategory::utility: return "Utility";
        case NodeCategory::output: return "Output";
    }
    return "";
}

std::string categoryName(NodeCategory category) {
    switch (category) {
        case NodeCategory::values: return "values";
        case NodeCategory::vectors: return "vectors";
        case NodeCategory::vectorMath: return "vectorMath";
        case NodeCategory::transforms: return "transforms";
        case NodeCategory::math: return "math";
        case NodeCategory::trigonometry: return "trigonometry";
        case NodeCategory::waves: return "waves";
        case NodeCategory::noise: return "noise";
        case NodeCategory::patterns: return "patterns";
        case NodeCategory::shapes: return "shapes";
        case NodeCategory::sdfOperations: return "sdfOperations";
        case NodeCategory::color: return "color";
        case NodeCategory::blending: return "blending";
        case NodeCategory::textures: return "textures";
        case NodeCategory::imageEffects: return "imageEffects";
        case NodeCategory::time: return "time";
        case NodeCategory::interaction: return "interaction";
        case NodeCategory::raymarching: return "raymarching";
        case NodeCategory::simulation: return "simulation";
        case NodeCategory::utility: return "utility";
        case NodeCategory::output: return "output";
    }
    return "";
}

const std::vector<NodeCategory>& allCategories() {
    static const std::vector<NodeCategory> all = {
        NodeCategory::values, NodeCategory::vectors, NodeCategory::vectorMath, NodeCategory::transforms,
        NodeCategory::math, NodeCategory::trigonometry, NodeCategory::waves, NodeCategory::noise,
        NodeCategory::patterns, NodeCategory::shapes, NodeCategory::sdfOperations, NodeCategory::color,
        NodeCategory::blending, NodeCategory::textures, NodeCategory::imageEffects, NodeCategory::time,
        NodeCategory::interaction, NodeCategory::raymarching, NodeCategory::simulation, NodeCategory::utility,
        NodeCategory::output};
    return all;
}

bool isSharedFeed(TextureFeed feed) { return feed != TextureFeed::image && feed != TextureFeed::video; }

bool readsFile(TextureFeed feed) { return feed == TextureFeed::image || feed == TextureFeed::video; }

namespace spec {

InputSpec num(const std::string& id, const std::string& label, double initial, Range range) {
    return InputSpec{NodePort{id, label, SocketType::number, std::nullopt, range}, initial, std::nullopt};
}

InputSpec anyIn(const std::string& id, const std::string& label, double initial, Range range) {
    return InputSpec{NodePort{id, label, SocketType::any, std::nullopt, range}, initial, std::nullopt};
}

InputSpec timeIn(const std::string& id, const std::string& label) {
    return InputSpec{NodePort{id, label, SocketType::number, Fallback::time(), std::nullopt}, std::nullopt, std::nullopt};
}

InputSpec vec2(const std::string& id, const std::string& label, std::optional<Fallback> fallback) {
    return InputSpec{NodePort{id, label, SocketType::vector2, std::move(fallback), std::nullopt}, std::nullopt, std::nullopt};
}

InputSpec vec3(const std::string& id, const std::string& label, std::optional<Fallback> fallback) {
    return InputSpec{NodePort{id, label, SocketType::vector3, std::move(fallback), std::nullopt}, std::nullopt, std::nullopt};
}

InputSpec vec4(const std::string& id, const std::string& label, std::optional<Fallback> fallback) {
    return InputSpec{NodePort{id, label, SocketType::vector4, std::move(fallback), std::nullopt}, std::nullopt, std::nullopt};
}

InputSpec col(const std::string& id, const std::string& label, std::optional<Fallback> fallback) {
    return InputSpec{NodePort{id, label, SocketType::color, std::move(fallback), std::nullopt}, std::nullopt, std::nullopt};
}

InputSpec tex(const std::string& id, const std::string& label, std::optional<Fallback> fallback) {
    return InputSpec{NodePort{id, label, SocketType::texture, std::move(fallback), std::nullopt}, std::nullopt, std::nullopt};
}

InputSpec scoped(InputSpec input, std::vector<std::pair<std::string, std::string>> parameters) {
    input.scope = InputScope{std::move(parameters)};
    return input;
}

OutputSpec out(const std::string& id, const std::string& label, SocketType type, std::optional<std::string> templateText) {
    return OutputSpec{NodePort{id, label, type, std::nullopt, std::nullopt}, std::move(templateText)};
}

NodeOption option(const std::string& id, const std::string& label, std::vector<std::string> choices, int defaultIndex) {
    return NodeOption{id, label, std::move(choices), defaultIndex};
}

} // namespace spec

NodeDefinition NodeBuilder::node(NodeParts parts) const {
    NodeDefinition definition;
    definition.kind = NodeKind{parts.name};
    definition.label = std::move(parts.label);
    definition.summary = std::move(parts.summary);
    definition.category = category;
    definition.options = std::move(parts.options);
    definition.text = std::move(parts.text);
    definition.setup = std::move(parts.setup);
    definition.symbol = std::move(parts.symbol);
    definition.feed = parts.feed;
    definition.defaults = std::move(parts.numbers);
    for (InputSpec& input : parts.inputs) {
        if (input.initial) definition.defaults[input.port.id] = *input.initial;
        if (input.scope) definition.scopes[input.port.id] = *input.scope;
        definition.inputs.push_back(std::move(input.port));
    }
    for (const NodeOption& option : definition.options) {
        definition.defaults[option.id] = static_cast<double>(option.defaultIndex);
    }
    for (OutputSpec& output : parts.outputs) {
        if (output.templateText) definition.templates[output.port.id] = *output.templateText;
        definition.outputs.push_back(std::move(output.port));
    }
    return definition;
}

NodeDefinition NodeBuilder::simple(const std::string& name, const std::string& label, const std::string& summary,
                                   std::vector<InputSpec> inputs, SocketType type, const std::string& templateText,
                                   SimpleExtras extras) const {
    NodeParts parts;
    parts.name = name;
    parts.label = label;
    parts.summary = summary;
    parts.symbol = std::move(extras.symbol);
    parts.inputs = std::move(inputs);
    parts.outputs = {spec::out(extras.output.first, extras.output.second, type, templateText)};
    parts.options = std::move(extras.options);
    return node(std::move(parts));
}

} // namespace editor::shaders
