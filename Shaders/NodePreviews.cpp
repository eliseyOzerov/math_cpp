#include "NodePreviews.h"

#include <map>
#include <set>

#include "NodeCatalog.h"

namespace editor::shaders {

namespace {

/// Where a node reading a scope variable has to sit, by variable: a kind with a scoped input and
/// the output to preview it through.
struct ScopeHost {
    std::string kind;
    std::string input;
    std::string output;
};

const std::map<std::string, ScopeHost>& scopeHosts() {
    static const std::map<std::string, ScopeHost> hosts = {
        {"loopIndex", {"loop", "body", "value"}},
        {"loopValue", {"loop", "body", "value"}},
        {"marchPoint", {"raymarch", "scene", "color"}},
    };
    return hosts;
}

/// Colour fallbacks that say nothing about a node, so its preview feeds a picture instead.
bool isNeutral(const std::optional<Fallback>& fallback) {
    if (!fallback) return true;
    return *fallback == Fallback::black() || *fallback == Fallback::white() || *fallback == Fallback::gray()
        || *fallback == Fallback::clear();
}

/// Number inputs a preview sweeps across the tile from 0 to 1, for kinds that map a position to a
/// colour and would otherwise show one flat colour.
const std::map<std::string, std::string>& sweeps() {
    static const std::map<std::string, std::string> table = {
        {"gradient-ramp", "position"}, {"cosine-palette", "t"}, {"ease", "t"}};
    return table;
}

/// Shapes the shape operations' number inputs are fed, by input, so they work on a circle and a
/// bar crossing it rather than on zero.
const std::map<std::string, std::string>& shapeFeeds() {
    static const std::map<std::string, std::string> table = {
        {"distance", "circle-sdf"}, {"a", "circle-sdf"}, {"b", "rectangle-sdf"}, {"displacement", "perlin-noise"}};
    return table;
}

/// Shapes with no inside, whose filled picture would be a hairline.
bool isCurveKind(const std::string& kind) { return kind == "line-segment-sdf" || kind == "bezier-sdf"; }

bool knows(const std::string& kind) { return NodeCatalog::knows(kind); }

NodeKind kindNamed(const std::string& name) { return NodeKind{name}; }

/// Kinds a graph has to hold for a kind to compile, beyond its own inputs.
void addCompanions(const NodeKind& kind, ShaderGraph& graph) {
    if (kind == kinds::getVariable) {
        graph.addNode(kinds::setVariable, {}, "setter");
        graph.addNode(kinds::uv, {}, "setterUV");
        graph.connect("setterUV", "uv", "setter", "value");
    }
}

/// Adds a Color node called "tint" once, and wires it into `input` of "subject".
void feedTint(ShaderGraph& graph, const std::string& input) {
    if (graph.node("tint") == nullptr) graph.addNode(kinds::color, {}, "tint");
    graph.connect("tint", "color", "subject", input);
}

void feedUV(ShaderGraph& graph, const std::string& input) {
    if (graph.node("uv") == nullptr) graph.addNode(kinds::uv, {}, "uv");
    graph.connect("uv", "uv", "subject", input);
}

} // namespace

std::string feedingName(Feeding feeding) {
    switch (feeding) {
        case Feeding::numbers: return "numbers";
        case Feeding::vectors: return "vectors";
        case Feeding::colors: return "colors";
    }
    return "";
}

const std::vector<Feeding>& allFeedings() {
    static const std::vector<Feeding> all = {Feeding::numbers, Feeding::vectors, Feeding::colors};
    return all;
}

std::string Preview::name() const {
    return kind.rawValue + "." + output.id + (feeding == Feeding::numbers ? "" : " (" + feedingName(feeding) + ")");
}

CompiledShader Preview::compile() const { return ShaderCompiler::compile(graph, node, port); }

std::vector<Preview> previews(const NodeKind& kind, Feeding feeding) {
    const NodeDefinition& definition = NodeCatalog::definition(kind);
    ShaderGraph graph;
    graph.addNode(kind, {}, "subject");
    addCompanions(kind, graph);
    int colorInputs = 0;
    for (const NodePort& input : definition.inputs) {
        if (definition.scopes.count(input.id) > 0) continue;
        auto sweep = sweeps().find(kind.rawValue);
        bool isSwept = sweep != sweeps().end() && sweep->second == input.id;
        auto shapeFeed = shapeFeeds().find(input.id);
        switch (input.type) {
            case SocketType::texture:
                if (knows("image")) {
                    if (graph.node("image") == nullptr) graph.addNode(kinds::image, {}, "image");
                    graph.connect("image", "texture", "subject", input.id);
                }
                break;
            case SocketType::color:
                if (colorInputs == 0) {
                    // The first colour input gets the stand-in image, so colour nodes show what they do to a
                    // picture rather than to grey; without an Image kind it gets a flat colour like the others.
                    colorInputs += 1;
                    if (knows("image") && knows("sample-texture")) {
                        if (graph.node("image") == nullptr) graph.addNode(kinds::image, {}, "image");
                        graph.addNode(kinds::sampleTexture, {}, "photo");
                        graph.connect("image", "texture", "photo", "texture");
                        graph.connect("photo", "color", "subject", input.id);
                    } else {
                        feedTint(graph, input.id);
                    }
                } else if (isNeutral(input.fallback)) {
                    // Later ones keep a telling fallback or get a flat colour.
                    colorInputs += 1;
                    feedTint(graph, input.id);
                }
                break;
            case SocketType::any:
                if (feeding == Feeding::vectors) feedUV(graph, input.id);
                if (feeding == Feeding::colors) feedTint(graph, input.id);
                break;
            case SocketType::number:
                if (isSwept) {
                    graph.addNode(kinds::uv, {}, "sweepUV");
                    graph.addNode(kinds::separateVector2, {}, "sweep");
                    graph.connect("sweepUV", "uv", "sweep", "vector");
                    graph.connect("sweep", "x", "subject", input.id);
                } else if (definition.category == NodeCategory::sdfOperations && shapeFeed != shapeFeeds().end()
                           && knows(shapeFeed->second)) {
                    std::string id = "feed-" + input.id;
                    graph.addNode(kindNamed(shapeFeed->second), {}, id);
                    if (shapeFeed->second == "rectangle-sdf") {
                        graph.setParameter(id, "width", 0.9);
                        graph.setParameter(id, "height", 0.2);
                    }
                    graph.connect(id, NodeCatalog::definition(shapeFeed->second).outputs[0].id, "subject", input.id);
                }
                break;
            case SocketType::vector3:
            case SocketType::vector4:
                // Vectors with no fallback would read zero; UV shows what a node does to each channel.
                if (!input.fallback) feedUV(graph, input.id);
                break;
            case SocketType::vector2:
                break;
        }
    }
    // A scene to march: a sphere, which reads March Point on its own.
    for (const NodePort& input : definition.inputs) {
        if (input.id == "scene" && definition.scopes.count("scene") > 0 && knows("sphere-3d")) {
            graph.addNode(kindNamed("sphere-3d"), {}, "scene");
            graph.connect("scene", "distance", "subject", input.id);
        }
    }
    std::optional<ScopeHost> host;
    for (const auto& [variable, candidate] : scopeHosts()) {
        for (const auto& entry : definition.templates) {
            if (!host && entry.second.find(variable) != std::string::npos) host = candidate;
        }
    }
    // The 3D shapes and position nodes read March Point through an input's fallback, not a template.
    for (const NodePort& input : definition.inputs) {
        if (!host && input.fallback && input.fallback->scope) {
            auto candidate = scopeHosts().find(*input.fallback->scope);
            if (candidate != scopeHosts().end()) host = candidate->second;
        }
    }
    bool hosted = host && host->kind != kind.rawValue && knows(host->kind);
    if (hosted) graph.addNode(kindNamed(host->kind), {}, "host");

    std::vector<Preview> result;
    for (const NodePort& output : definition.outputs) {
        Preview preview{kind, output, feeding, graph, "subject", output.id};
        if (hosted) {
            preview.graph.connect("subject", output.id, "host", host->input);
            preview.node = "host";
            preview.port = host->output;
        }
        result.push_back(std::move(preview));
    }
    return result;
}

std::vector<Preview> allPreviews() {
    std::vector<Preview> result;
    for (const NodeDefinition& definition : NodeCatalog::all()) {
        bool anyInputs = false;
        for (const NodePort& input : definition.inputs) anyInputs = anyInputs || input.type == SocketType::any;
        std::vector<Feeding> feedings = anyInputs ? allFeedings() : std::vector<Feeding>{Feeding::numbers};
        for (Feeding feeding : feedings) {
            for (Preview& preview : previews(definition.kind, feeding)) result.push_back(std::move(preview));
        }
    }
    return result;
}

std::optional<CompiledShader> previewShader(const NodeKind& kind) {
    std::vector<Preview> all;
    try {
        all = previews(kind);
    } catch (const GraphEditError&) {
        return std::nullopt;
    }
    if (all.empty()) return std::nullopt;
    Preview preview = std::move(all.front());
    const NodeDefinition& definition = NodeCatalog::definition(kind);
    std::set<std::string> outputs;
    for (const NodePort& output : definition.outputs) outputs.insert(output.id);
    // Each edit below is best effort, as in graphics_metal (`try?`): a refused edit leaves the preview as it was.
    auto attempt = [](auto edit) {
        try {
            edit();
            return true;
        } catch (const GraphEditError&) {
            return false;
        }
    };
    if (outputs.count("state") > 0 && outputs.count("view") > 0 && knows("previous-frame") && knows("feedback-output")) {
        attempt([&] { preview.graph.addNode(kinds::previousFrame, {}, "memory"); });
        attempt([&] { preview.graph.addNode(kinds::feedbackOutput, {}, "keep"); });
        attempt([&] { preview.graph.connect("memory", "texture", "subject", "state"); });
        attempt([&] { preview.graph.connect("subject", "state", "keep", "color"); });
        // Its colours keep their own fallbacks rather than the stand-in picture.
        for (const NodePort& input : definition.inputs) {
            if (input.type == SocketType::color) attempt([&] { preview.graph.disconnect("subject", input.id); });
        }
        preview.port = "view";
    }
    if (preview.node == "subject" && preview.output.id == "distance" && preview.output.type == SocketType::number) {
        bool curve = isCurveKind(kind.rawValue);
        std::string fill = curve ? "sdf-stroke" : "sdf-fill";
        if (knows(fill)) {
            attempt([&] { preview.graph.addNode(kindNamed(fill), {}, "fill"); });
            if (curve) attempt([&] { preview.graph.setParameter("fill", "width", 0.05); });
            if (attempt([&] { preview.graph.connect("subject", "distance", "fill", "distance"); })) {
                preview.node = "fill";
                preview.port = "mask";
            }
        }
    }
    CompiledShader shader = preview.compile();
    if (shader.error) return std::nullopt;
    return shader;
}

} // namespace editor::shaders
