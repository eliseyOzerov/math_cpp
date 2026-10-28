#include "ShaderCompilation.h"

#include <algorithm>
#include <cctype>

#include "MetalBackend.h"
#include "NodeCatalog.h"

namespace editor::shaders {

namespace {

/// The variables scoped inputs pass, which nodes read through their templates and fallbacks.
const std::vector<std::string> scopeVariables = {"marchPoint", "loopIndex", "loopValue"};

int widthOf(SocketType type) {
    switch (type) {
        case SocketType::number:
        case SocketType::any:
        case SocketType::texture: return 1;
        case SocketType::vector2: return 2;
        case SocketType::vector3: return 3;
        case SocketType::vector4:
        case SocketType::color: return 4;
    }
    return 1;
}

std::string trimmed(const std::string& text) {
    std::size_t first = 0;
    std::size_t last = text.size();
    while (first < last && std::isspace(static_cast<unsigned char>(text[first]))) first += 1;
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) last -= 1;
    return text.substr(first, last - first);
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

std::string join(const std::vector<std::string>& parts, const std::string& separator) {
    std::string result;
    for (std::size_t index = 0; index < parts.size(); index += 1) {
        if (index > 0) result += separator;
        result += parts[index];
    }
    return result;
}

const ShaderGraph& emptyGraph() {
    static const ShaderGraph graph;
    return graph;
}

} // namespace

SocketType widerType(SocketType first, SocketType second) {
    if (widthOf(first) != widthOf(second)) return widthOf(first) > widthOf(second) ? first : second;
    bool eitherIsColor = first == SocketType::color || second == SocketType::color;
    if (!eitherIsColor) return first;
    return widthOf(first) == 4 ? SocketType::color : first;
}

Value Value::converted(SocketType target) const {
    SocketType goal = target == SocketType::any ? SocketType::number : target;
    SocketType source = type == SocketType::any ? SocketType::number : type;
    if (source == goal) return Value{code, goal, scopes};
    std::string wrapped = "(" + code + ")";
    using T = SocketType;
    std::string result = code;
    if (source == T::number && goal == T::vector2) result = "float2" + wrapped;
    else if (source == T::number && goal == T::vector3) result = "float3" + wrapped;
    else if (source == T::number && goal == T::vector4) result = "float4" + wrapped;
    else if (source == T::number && goal == T::color) result = "float4(float3" + wrapped + ", 1.0f)";
    else if (goal == T::number && (source == T::vector2 || source == T::vector3 || source == T::vector4)) result = wrapped + ".x";
    else if (source == T::vector2 && goal == T::vector3) result = "float3(" + code + ", 0.0f)";
    else if (source == T::vector2 && goal == T::vector4) result = "float4(" + code + ", 0.0f, 0.0f)";
    else if (source == T::vector2 && goal == T::color) result = "float4(" + code + ", 0.0f, 1.0f)";
    else if (goal == T::vector2 && (source == T::vector3 || source == T::vector4 || source == T::color)) result = wrapped + ".xy";
    else if (source == T::vector3 && goal == T::vector4) result = "float4(" + code + ", 0.0f)";
    else if (source == T::vector3 && goal == T::color) result = "float4(" + code + ", 1.0f)";
    else if (goal == T::vector3 && (source == T::vector4 || source == T::color)) result = wrapped + ".xyz";
    else if (source == T::color && goal == T::number) result = "luminance" + wrapped;
    // Vector 4 and colour share a Metal type, so the code stays as it is.
    return Value{result, goal, scopes};
}

std::string fillTemplate(const std::string& templateText, const std::function<std::string(char, const std::string&)>& value) {
    std::string result;
    std::size_t index = 0;
    while (index < templateText.size()) {
        char character = templateText[index];
        std::size_t close = character == '{' ? templateText.find('}', index) : std::string::npos;
        if (close == std::string::npos) {
            result += character;
            index += 1;
            continue;
        }
        std::string inside = templateText.substr(index + 1, close - index - 1);
        char sign = '\0';
        if (!inside.empty() && (inside[0] == '#' || inside[0] == '@' || inside[0] == '$')) {
            sign = inside[0];
            inside = inside.substr(1);
        }
        bool isName = std::all_of(inside.begin(), inside.end(),
                                  [](char letter) { return std::isalnum(static_cast<unsigned char>(letter)) || letter == '_'; });
        result += isName ? value(sign, inside) : templateText.substr(index, close - index + 1);
        index = close + 1;
    }
    return result;
}

Compilation::Compilation(const ShaderGraph& graph) : root{&graph, "", nullptr, nullptr}, frames(1) {}

// Outputs

Value Compilation::output(const std::string& nodeID, const std::string& port, const GraphContext& context,
                          std::set<std::string> visiting) {
    std::string key = context.prefix + nodeID;
    for (auto frame = frames.rbegin(); frame != frames.rend(); ++frame) {
        auto node = frame->cache.find(key);
        if (node == frame->cache.end()) continue;
        auto value = node->second.find(port);
        if (value != node->second.end()) return value->second;
    }
    if (visiting.count(key) > 0) throw GraphEditError("The shader graph contains a cycle.");
    const GraphNode* node = context.graph->node(nodeID);
    if (node == nullptr) throw GraphEditError("A connected node is missing.");
    const NodeDefinition& definition = NodeCatalog::definition(node->kind);
    bool hasPort = std::any_of(definition.outputs.begin(), definition.outputs.end(),
                               [&](const NodePort& candidate) { return candidate.id == port; });
    if (!hasPort) throw GraphEditError(definition.label + " has no output " + port + ".");
    visiting.insert(key);
    auto [values, frame] = compile(*node, definition, context, visiting);
    frames[frame].cache[key] = values;
    auto found = values.find(port);
    if (found == values.end()) throw GraphEditError(definition.label + " has no output " + port + ".");
    return found->second;
}

std::optional<Value> Compilation::wired(const GraphNode& node, const std::string& port, const GraphContext& context,
                                        const std::set<std::string>& visiting) {
    const GraphConnection* wire = context.graph->connection(node.id, port);
    if (wire == nullptr) return std::nullopt;
    return output(wire->sourceNode, wire->sourcePort, context, visiting);
}

// Nodes

std::pair<Compilation::Outputs, std::size_t> Compilation::compile(const GraphNode& node, const NodeDefinition& definition,
                                                                  const GraphContext& context,
                                                                  const std::set<std::string>& visiting) {
    const NodeKind& kind = node.kind;
    if (kind == kinds::shader || kind == kinds::feedbackOutput || kind == kinds::groupOutput || kind == kinds::setVariable) {
        throw GraphEditError("A " + definition.label + " node cannot feed another node.");
    }
    if (kind == kinds::color) return colorNode(node, context, visiting);
    if (kind == kinds::groupInput) return groupInput(context, visiting);
    if (kind == kinds::subgraph) return subgraphNode(node, context, visiting);
    if (kind == kinds::getVariable) return getVariable(node, context, visiting);
    if (kind == kinds::customCode) return customCode(node, definition, context, visiting);
    if (definition.feed) {
        std::string code = textureName(*definition.feed, node, context);
        if (definition.templates.empty()) return {Outputs{{"texture", Value{code, SocketType::texture, {}}}}, 0};
        return templated(node, definition, code, context, visiting);
    }
    return templated(node, definition, std::nullopt, context, visiting);
}

std::pair<Compilation::Outputs, std::size_t> Compilation::templated(const GraphNode& node, const NodeDefinition& definition,
                                                                    const std::optional<std::string>& feed,
                                                                    const GraphContext& context,
                                                                    const std::set<std::string>& visiting) {
    std::vector<NodePort> ports = NodeCatalog::inputs(node);

    // The type an `any` node works in: the widest value wired into its `any` inputs.
    SocketType working = SocketType::number;
    for (const NodePort& port : ports) {
        if (port.type != SocketType::any) continue;
        if (const GraphConnection* wire = context.graph->connection(node.id, port.id)) {
            Value upstream = output(wire->sourceNode, wire->sourcePort, context, visiting);
            working = widerType(working, upstream.type);
        }
    }

    std::map<std::string, Value> inputs;
    for (const NodePort& port : ports) {
        SocketType type = port.type == SocketType::any ? working : port.type;
        auto scope = definition.scopes.find(port.id);
        inputs[port.id] = scope != definition.scopes.end()
            ? scopedInput(node, port, type, scope->second, context, visiting)
            : input(node, port, type, definition, context, visiting);
    }

    std::set<std::string> scopes;
    for (const auto& entry : inputs) scopes.insert(entry.second.scopes.begin(), entry.second.scopes.end());
    std::string templateText;
    for (const auto& entry : definition.templates) templateText += entry.second;
    templateText += definition.setup.value_or("");
    for (const std::string& variable : scopeVariables) {
        if (contains(templateText, variable)) scopes.insert(variable);
    }
    std::size_t frame = frameIndex(scopes, definition.label);

    auto fill = [&](const std::string& text, const std::optional<std::string>& setup) {
        return fillTemplate(text, [&](char sign, const std::string& id) -> std::string {
            if (sign == '$') return setup.value_or("0.0f");
            if (sign == '#' || sign == '@') {
                auto given = node.parameters.find(id);
                auto initial = definition.defaults.find(id);
                double number = given != node.parameters.end() ? given->second : initial != definition.defaults.end() ? initial->second : 0;
                return sign == '#' ? std::to_string(static_cast<long long>(number)) : ShaderCompiler::literal(number);
            }
            if (id == "T") return metalBackend::typeName(working);
            if (id == "feed" && feed) return *feed;
            auto found = inputs.find(id);
            return found != inputs.end() ? "(" + found->second.code + ")" : "0.0f";
        });
    };

    std::optional<std::string> setupName;
    if (definition.setup) {
        std::string name = fresh();
        frames[frame].lines.push_back("const auto " + name + " = " + fill(*definition.setup, std::nullopt) + ";");
        setupName = name;
    }
    Outputs values;
    for (const NodePort& port : definition.outputs) {
        SocketType type = port.type == SocketType::any ? working : port.type;
        auto templateFound = definition.templates.find(port.id);
        if (templateFound == definition.templates.end()) continue;
        std::string code = fill(templateFound->second, setupName);
        if (type == SocketType::texture) {
            values[port.id] = Value{code, SocketType::texture, scopes};
        } else {
            std::string name = fresh();
            frames[frame].lines.push_back("const " + metalBackend::typeName(type) + " " + name + " = " + code + ";");
            values[port.id] = Value{name, type, scopes};
        }
    }
    return {values, frame};
}

Value Compilation::input(const GraphNode& node, const NodePort& port, SocketType type, const NodeDefinition& definition,
                         const GraphContext& context, const std::set<std::string>& visiting) {
    if (std::optional<Value> upstream = wired(node, port.id, context, visiting)) {
        if (type == SocketType::texture || upstream->type == SocketType::texture) {
            if (type != upstream->type) {
                throw GraphEditError(definition.label + " expects a " + socketTypeName(type) + " at " + port.label + ".");
            }
            return *upstream;
        }
        return upstream->converted(type);
    }
    if (port.fallback) {
        std::set<std::string> scopes;
        if (port.fallback->scope) scopes.insert(*port.fallback->scope);
        return Value{port.fallback->code, port.type, scopes}.converted(type);
    }
    if (isNumeric(port.type)) {
        auto given = node.parameters.find(port.id);
        auto initial = definition.defaults.find(port.id);
        double number = given != node.parameters.end() ? given->second : initial != definition.defaults.end() ? initial->second : 0;
        return Value{ShaderCompiler::literal(number), SocketType::number, {}}.converted(type);
    }
    if (type == SocketType::texture) throw GraphEditError(definition.label + " needs a texture wired into " + port.label + ".");
    return Value{"0.0f", SocketType::number, {}}.converted(type);
}

Value Compilation::scopedInput(const GraphNode& node, const NodePort& port, SocketType type, const InputScope& scope,
                               const GraphContext& context, const std::set<std::string>& visiting) {
    std::set<std::string> variables;
    for (const auto& parameter : scope.parameters) variables.insert(parameter.second);
    frames.push_back(Frame{variables, {}, {}});
    Value body;
    try {
        body = input(node, port, type, NodeCatalog::definition(node.kind), context, visiting);
    } catch (...) {
        frames.pop_back();
        throw;
    }
    Frame frame = std::move(frames.back());
    frames.pop_back();

    std::vector<std::string> parameters;
    std::string unused;
    for (const auto& parameter : scope.parameters) {
        parameters.push_back(parameter.first + " " + parameter.second);
        unused += (unused.empty() ? "" : " ") + std::string("(void)") + parameter.second + ";";
    }
    std::string statements;
    for (const std::string& line : frame.lines) statements += "    " + line + "\n";
    std::string lambda = "[&](" + join(parameters, ", ") + ") -> " + metalBackend::typeName(type) + " {\n    " + unused + "\n"
                         + statements + "    return " + body.code + ";\n}";
    std::set<std::string> outer;
    for (const std::string& scopeName : body.scopes) {
        if (variables.count(scopeName) == 0) outer.insert(scopeName);
    }
    std::size_t index = frameIndex(outer, NodeCatalog::definition(node.kind).label);
    std::string name = fresh();
    frames[index].lines.push_back("const auto " + name + " = " + lambda + ";");
    return Value{name, type, outer};
}

std::size_t Compilation::frameIndex(const std::set<std::string>& scopes, const std::string& label) const {
    // The innermost frame whose lambda takes one of `scopes`; the fragment's body for none.
    std::size_t index = 0;
    std::set<std::string> covered;
    for (std::size_t position = 0; position < frames.size(); position += 1) {
        const std::set<std::string>& variables = frames[position].variables;
        bool shares = std::any_of(scopes.begin(), scopes.end(), [&](const std::string& name) { return variables.count(name) > 0; });
        if (!shares) continue;
        index = position;
        covered.insert(variables.begin(), variables.end());
    }
    for (const std::string& missing : scopes) {
        if (covered.count(missing) > 0) continue;
        std::string owner = missing == "marchPoint" ? "a Raymarch node's scene" : "a Loop node's body";
        throw GraphEditError(label + " reads " + missing + ", which only exists inside " + owner + ".");
    }
    return index;
}

std::string Compilation::fresh() {
    counter += 1;
    return "v" + std::to_string(counter);
}

// Kinds built here

std::pair<Compilation::Outputs, std::size_t> Compilation::colorNode(const GraphNode& node, const GraphContext& context,
                                                                    const std::set<std::string>& visiting) {
    ColorSpace space = node.colorSpace.value_or(ColorSpace::rgb);
    std::map<std::string, Value> channels;
    std::vector<Value> reading;
    for (const NodePort& port : NodeCatalog::inputs(node)) {
        channels[port.id] = input(node, port, SocketType::number, NodeCatalog::definition(kinds::color), context, visiting);
        reading.push_back(channels[port.id]);
    }
    auto c = [&](const std::string& id) {
        auto found = channels.find(id);
        return found != channels.end() ? found->second.code : std::string("0.0f");
    };
    auto unit = [&](const std::string& id) { return "clamp(" + c(id) + ", 0.0f, 1.0f)"; };
    std::string code;
    switch (space) {
        case ColorSpace::rgb:
            code = "float4(" + unit("r") + ", " + unit("g") + ", " + unit("b") + ", 1.0f)";
            break;
        case ColorSpace::cmyk:
            code = "float4((float3(1.0f) - clamp(float3(" + c("c") + ", " + c("m") + ", " + c("y") + "), 0.0f, 1.0f)) * (1.0f - "
                   + unit("k") + "), 1.0f)";
            break;
        case ColorSpace::oklab:
            code = "float4(encodedSRGBFromOklab(float3(" + unit("l") + ", clamp(" + c("a") + ", -0.4f, 0.4f), clamp(" + c("b")
                   + ", -0.4f, 0.4f))), 1.0f)";
            break;
        case ColorSpace::okhsv:
            code = "float4(encodedSRGBFromOkhsv(" + c("h") + ", " + unit("s") + ", " + unit("v") + "), 1.0f)";
            break;
    }
    return emit({{"color", {code, SocketType::color}}}, reading, "Color");
}

std::pair<Compilation::Outputs, std::size_t> Compilation::emit(
    const std::vector<std::pair<std::string, std::pair<std::string, SocketType>>>& outputs, const std::vector<Value>& reading,
    const std::string& label) {
    std::set<std::string> scopes;
    for (const Value& value : reading) scopes.insert(value.scopes.begin(), value.scopes.end());
    std::size_t frame = frames.size() - 1;
    try {
        frame = frameIndex(scopes, label);
    } catch (const GraphEditError&) {
        // Scopes with no frame to hold them fall to the innermost frame, as graphics_metal does.
    }
    Outputs values;
    for (const auto& [port, codeAndType] : outputs) {
        std::string name = fresh();
        frames[frame].lines.push_back("const " + metalBackend::typeName(codeAndType.second) + " " + name + " = " + codeAndType.first + ";");
        values[port] = Value{name, codeAndType.second, scopes};
    }
    return {values, frame};
}

std::string Compilation::textureName(TextureFeed feed, const GraphNode& node, const GraphContext& context) {
    // Every Previous Frame node reads the same thing, so shared feeds bind once.
    std::string key = isSharedFeed(feed) ? "feed." + std::to_string(static_cast<int>(feed)) : context.prefix + node.id;
    auto found = textureIndex.find(key);
    if (found != textureIndex.end()) return "uTexture" + std::to_string(found->second);
    int index = static_cast<int>(boundTextures.size());
    textureIndex[key] = index;
    boundTextures.push_back(ShaderTexture{index, feed, readsFile(feed) ? node.imagePath : std::nullopt});
    return "uTexture" + std::to_string(index);
}

std::pair<Compilation::Outputs, std::size_t> Compilation::groupInput(const GraphContext& context,
                                                                     const std::set<std::string>& visiting) {
    Outputs values;
    std::size_t frame = 0;
    for (const NodePort& port : NodeCatalog::definition(kinds::groupInput).outputs) {
        if (context.holder == nullptr) {
            values[port.id] = Value{"0.0f", SocketType::number, {}};
            continue;
        }
        Value value;
        if (std::optional<Value> upstream = wired(*context.holder, port.id, *context.outer, visiting)) {
            value = *upstream;
        } else {
            auto given = context.holder->parameters.find(port.id);
            value = Value{ShaderCompiler::literal(given != context.holder->parameters.end() ? given->second : 0), SocketType::number, {}};
        }
        values[port.id] = value;
        frame = std::max(frame, frameIndex(value.scopes, "Group Input"));
    }
    return {values, frame};
}

std::pair<Compilation::Outputs, std::size_t> Compilation::subgraphNode(const GraphNode& node, const GraphContext& context,
                                                                       const std::set<std::string>& visiting) {
    GraphContext inner{node.subgraph ? &node.subgraph->graph() : &emptyGraph(), context.prefix + node.id + "/", &node, &context};
    Outputs values;
    std::size_t frame = 0;
    const GraphNode* end = nullptr;
    for (const GraphNode& candidate : inner.graph->nodes) {
        if (end == nullptr && candidate.kind == kinds::groupOutput) end = &candidate;
    }
    for (const NodePort& port : NodeCatalog::definition(kinds::subgraph).outputs) {
        Value value{"0.0f", SocketType::number, {}};
        if (end != nullptr) {
            if (std::optional<Value> upstream = wired(*end, port.id, inner, visiting)) value = *upstream;
        }
        values[port.id] = value;
        frame = std::max(frame, frameIndex(value.scopes, "Subgraph"));
    }
    return {values, frame};
}

std::pair<Compilation::Outputs, std::size_t> Compilation::getVariable(const GraphNode& node, const GraphContext& context,
                                                                      const std::set<std::string>& visiting) {
    std::string name = trimmed(node.text.value_or(""));
    const GraphNode* setter = nullptr;
    for (const GraphNode& candidate : context.graph->nodes) {
        if (setter == nullptr && candidate.kind == kinds::setVariable && trimmed(candidate.text.value_or("")) == name) setter = &candidate;
    }
    if (setter == nullptr) throw GraphEditError("No Set Variable node is named “" + name + "”.");
    std::string key = context.prefix + setter->id;
    if (visiting.count(key) > 0) throw GraphEditError("The shader graph contains a cycle.");
    std::set<std::string> deeper = visiting;
    deeper.insert(key);
    std::optional<Value> value = wired(*setter, "value", context, deeper);
    if (!value) {
        auto given = setter->parameters.find("value");
        value = Value{ShaderCompiler::literal(given != setter->parameters.end() ? given->second : 0), SocketType::number, {}};
    }
    return {Outputs{{"value", *value}}, frameIndex(value->scopes, "Get Variable")};
}

std::pair<Compilation::Outputs, std::size_t> Compilation::customCode(const GraphNode& node, const NodeDefinition& definition,
                                                                     const GraphContext& context,
                                                                     const std::set<std::string>& visiting) {
    std::vector<std::string> arguments;
    std::vector<Value> reading;
    for (const NodePort& port : definition.inputs) {
        Value value = input(node, port, port.type == SocketType::any ? SocketType::vector4 : port.type, definition, context, visiting);
        reading.push_back(value);
        arguments.push_back("const " + metalBackend::typeName(value.type) + " " + port.id + " = " + value.code + ";");
    }
    std::string source = node.text.value_or(definition.text ? definition.text->initial : "return float4(0.0f);");
    std::string body;
    std::size_t start = 0;
    while (true) {
        std::size_t end = source.find('\n', start);
        body += "        " + source.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (end == std::string::npos) break;
        body += "\n";
        start = end + 1;
    }
    std::string code = "[&]() -> float4 {\n        " + join(arguments, "\n        ")
                       + "\n        (void)a; (void)b; (void)c; (void)d; (void)uv;\n" + body + "\n    }()";
    auto [values, frame] = emit({{"color", {code, SocketType::color}}}, reading, definition.label);
    auto color = values.find("color");
    if (color == values.end()) return {values, frame};
    std::string luminanceName = fresh();
    frames[frame].lines.push_back("const float " + luminanceName + " = luminance(" + color->second.code + ");");
    values["value"] = Value{luminanceName, SocketType::number, color->second.scopes};
    return {values, frame};
}

} // namespace editor::shaders
