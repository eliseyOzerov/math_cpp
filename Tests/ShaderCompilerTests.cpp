// The shader graph compiler (Shaders), ported from graphics_metal's CompilerTests.swift.
// These run on the CPU and check the generated text and the compiler's errors; that every preview
// really compiles as Metal is `make shaders` (Tests/ShaderPreviews).
#include <set>

#include "../Shaders/NodeCatalog.h"
#include "../Shaders/NodePreviews.h"
#include "../Shaders/ShaderCompilation.h"
#include "../Shaders/ShaderCompiler.h"
#include "Testing.h"

using namespace editor::shaders;

namespace {

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

/// Kinds with no outputs, which only matter inside a graph that draws.
bool isEnd(const std::string& name) {
    return name == "shader" || name == "feedback-output" || name == "comment" || name == "group-output" || name == "set-variable";
}

/// Whether `text` holds a decimal number without the float suffix, such as `0.5` or `1.0)`.
bool hasDoubleLiteral(const std::string& text) {
    for (std::size_t index = 1; index + 1 < text.size(); index += 1) {
        if (text[index] != '.') continue;
        bool digitBefore = std::isdigit(static_cast<unsigned char>(text[index - 1])) != 0;
        bool digitAfter = std::isdigit(static_cast<unsigned char>(text[index + 1])) != 0;
        if (!digitBefore || !digitAfter) continue;
        std::size_t end = index + 1;
        while (end < text.size() && std::isdigit(static_cast<unsigned char>(text[end]))) end += 1;
        if (end < text.size() && (text[end] == 'f' || text[end] == 'e')) continue;
        return true;
    }
    return false;
}

} // namespace

TEST(shaderEveryKindCompilesFedNumbersVectorsAndColors) {
    for (const NodeDefinition& definition : NodeCatalog::all()) {
        if (isEnd(definition.kind.rawValue)) continue;
        bool anyInputs = false;
        for (const NodePort& input : definition.inputs) anyInputs = anyInputs || input.type == SocketType::any;
        for (Feeding feeding : anyInputs ? allFeedings() : std::vector<Feeding>{Feeding::numbers}) {
            std::vector<Preview> shown = previews(definition.kind, feeding);
            EXPECT(!shown.empty());
            for (const Preview& preview : shown) {
                CompiledShader shader = preview.compile();
                if (shader.error) {
                    editor::testing::fail(__FILE__, __LINE__, preview.name() + ": " + *shader.error);
                }
                EXPECT(contains(shader.source, "fragment float4 graphFragment"));
            }
        }
    }
}

TEST(shaderTemplatesNameOnlyRealInputsOptionsAndNumbers) {
    for (const NodeDefinition& definition : NodeCatalog::all()) {
        const std::string kind = definition.kind.rawValue;
        std::set<std::string> known = {"T", "feed"};
        for (const NodePort& input : definition.inputs) known.insert(input.id);
        std::vector<std::string> templates;
        for (const auto& entry : definition.templates) templates.push_back(entry.second);
        if (definition.setup) templates.push_back(*definition.setup);
        for (const std::string& templateText : templates) {
            fillTemplate(templateText, [&](char sign, const std::string& id) -> std::string {
                bool isOption = false;
                for (const NodeOption& option : definition.options) isOption = isOption || option.id == id;
                if (sign == '#') {
                    if (!isOption) editor::testing::fail(__FILE__, __LINE__, kind + " has no option " + id);
                } else if (sign == '@') {
                    if (definition.defaults.count(id) == 0) editor::testing::fail(__FILE__, __LINE__, kind + " has no number " + id);
                } else if (sign == '$') {
                    if (!definition.setup) editor::testing::fail(__FILE__, __LINE__, kind + " reads {$} with no setup");
                } else {
                    if (known.count(id) == 0) editor::testing::fail(__FILE__, __LINE__, kind + " has no input " + id);
                    if (id == "feed" && !definition.feed) editor::testing::fail(__FILE__, __LINE__, kind + " reads {feed} with no feed");
                }
                return "";
            });
        }
        for (const NodeOption& option : definition.options) {
            EXPECT(option.defaultIndex >= 0 && option.defaultIndex < static_cast<int>(option.choices.size()));
        }
        std::set<std::string> inputIDs;
        std::set<std::string> outputIDs;
        for (const NodePort& input : definition.inputs) inputIDs.insert(input.id);
        for (const NodePort& output : definition.outputs) outputIDs.insert(output.id);
        EXPECT(inputIDs.size() == definition.inputs.size());
        EXPECT(outputIDs.size() == definition.outputs.size());
    }
}

TEST(shaderEveryTemplatedKindHasATemplateForEachOutput) {
    std::set<std::string> built = {"color", "group-input", "subgraph", "get-variable", "custom-code"};
    for (const NodeDefinition& definition : NodeCatalog::all()) {
        if (built.count(definition.kind.rawValue) > 0) continue;
        for (const NodePort& output : definition.outputs) {
            bool textureSource = definition.feed && output.type == SocketType::texture;
            if (!textureSource && definition.templates.count(output.id) == 0) {
                editor::testing::fail(__FILE__, __LINE__, definition.kind.rawValue + "." + output.id + " has no template");
            }
        }
    }
}

TEST(shaderTemplatesWriteEveryNumberAsAFloatLiteral) {
    // The kernels are templates on the number type, so an unsuffixed 0.5 would be a double next to a float.
    for (const NodeDefinition& definition : NodeCatalog::all()) {
        std::vector<std::string> texts;
        for (const auto& entry : definition.templates) texts.push_back(entry.second);
        if (definition.setup) texts.push_back(*definition.setup);
        for (const NodePort& input : definition.inputs) {
            if (input.fallback) texts.push_back(input.fallback->code);
        }
        for (const std::string& text : texts) {
            if (hasDoubleLiteral(text)) editor::testing::fail(__FILE__, __LINE__, definition.kind.rawValue + ": " + text);
        }
    }
}

TEST(shaderEndsCompile) {
    ShaderGraph graph;
    graph.addNode(kinds::shader, {}, "shader");
    graph.addNode(kinds::feedbackOutput, {}, "feedback");
    graph.addNode(kinds::comment, {}, "note");
    graph.addNode(kinds::setVariable, {}, "setter");
    graph.addNode(kinds::uv, {}, "uv");
    graph.addNode(kinds::color, {}, "color");
    graph.connect("uv", "uv", "setter", "value");
    graph.connect("color", "color", "shader", "color");
    graph.connect("color", "color", "feedback", "color");
    CompiledShader shader = ShaderCompiler::compile(graph);
    EXPECT(!shader.error.has_value());
    EXPECT(shader.writesFeedback);
    EXPECT(contains(shader.source, "[[color(1)]]"));
}

TEST(shaderSubgraphPassesValuesThrough) {
    ShaderGraph inner;
    inner.addNode(kinds::groupInput, {}, "in");
    inner.addNode(kinds::groupOutput, {}, "out");
    inner.addNode(NodeKind{"one-minus"}, {}, "invert");
    inner.connect("in", "a", "invert", "value");
    inner.connect("invert", "value", "out", "a");
    ShaderGraph graph;
    graph.addNode(kinds::subgraph, {}, "group");
    graph.nodes[0].subgraph = NestedGraph(inner);
    graph.setParameter("group", "a", 0.25);
    CompiledShader shader = ShaderCompiler::compile(graph, "group", "a");
    EXPECT(!shader.error.has_value());
    // The wired-in 0.25 goes through One Minus: 1 - 0.25, read back by the preview as a grey.
    EXPECT(contains(shader.source, "const float v1 = 1.0f - (0.25f);"));
    EXPECT(contains(shader.source, "return clamp(float4(float3(v1), 1.0f), 0.0f, 1.0f);"));
}

TEST(shaderEveryColourSpaceCompiles) {
    for (ColorSpace space : allColorSpaces()) {
        ShaderGraph graph;
        graph.addNode(kinds::color, {}, "color");
        graph.setColorSpace("color", space);
        graph.addNode(kinds::shader, {}, "shader");
        graph.connect("color", "color", "shader", "color");
        EXPECT(!ShaderCompiler::compile(graph).error.has_value());
    }
}

TEST(shaderStarterCompilesAndAnimates) {
    CompiledShader shader = ShaderCompiler::compile(ShaderGraph::starter());
    EXPECT(!shader.error.has_value());
    EXPECT(shader.usesTime);
    EXPECT(contains(shader.source, "encodedSRGBFromOkhsv("));
}

TEST(shaderUnwiredShaderDrawsPurple) {
    ShaderGraph graph;
    graph.addNode(kinds::shader, {});
    CompiledShader shader = ShaderCompiler::compile(graph);
    EXPECT(!shader.error.has_value());
    EXPECT(contains(shader.source, "0.431373f"));
}

TEST(shaderMissingShaderNodeIsAnError) {
    CompiledShader shader = ShaderCompiler::compile(ShaderGraph());
    EXPECT(shader.error == std::string("The graph has no Shader node."));
    EXPECT(contains(shader.source, "0.22f, 0.03f, 0.05f"));
}

TEST(shaderSharedFeedsBindOnce) {
    ShaderGraph graph;
    graph.addNode(NodeKind{"keyboard-key"}, {}, "key");
    graph.addNode(NodeKind{"midi"}, {}, "midi");
    graph.addNode(kinds::add, {}, "sum");
    graph.connect("key", "down", "sum", "a");
    graph.connect("midi", "controller", "sum", "b");
    CompiledShader shader = ShaderCompiler::compile(graph, "sum", "value");
    EXPECT(!shader.error.has_value());
    EXPECT_EQUAL(shader.textures.size(), static_cast<std::size_t>(1));
    EXPECT(shader.usesPointer);
    EXPECT(contains(shader.source, "texture2d<float> uTexture0 [[texture(0)]]"));
}

TEST(shaderCyclesThroughVariablesAreErrors) {
    ShaderGraph graph;
    graph.addNode(kinds::setVariable, {}, "setter");
    graph.addNode(kinds::getVariable, {}, "getter");
    graph.addNode(kinds::add, {}, "sum");
    graph.connect("getter", "value", "sum", "a");
    graph.connect("sum", "value", "setter", "value");
    CompiledShader shader = ShaderCompiler::compile(graph, "sum", "value");
    EXPECT(shader.error == std::string("The shader graph contains a cycle."));
}

TEST(shaderScopeVariablesOutsideTheirScopeAreErrors) {
    ShaderGraph graph;
    graph.addNode(NodeKind{"loop-index"}, {}, "index");
    CompiledShader shader = ShaderCompiler::compile(graph, "index", "index");
    EXPECT(shader.error.has_value());
    EXPECT(contains(*shader.error, "only exists inside a Loop node's body"));
}

TEST(shaderLoopBodyCompilesToALambda) {
    ShaderGraph graph;
    graph.addNode(NodeKind{"loop"}, {}, "loop");
    graph.addNode(NodeKind{"loop-index"}, {}, "index");
    graph.connect("index", "index", "loop", "body");
    CompiledShader shader = ShaderCompiler::compile(graph, "loop", "value");
    EXPECT(!shader.error.has_value());
    EXPECT(contains(shader.source, "[&](float loopIndex, float4 loopValue) -> float4"));
    EXPECT(contains(shader.source, "loopAccumulate("));
}

TEST(shaderLiteralsAreFloats) {
    EXPECT_EQUAL(ShaderCompiler::literal(1), std::string("1.0f"));
    EXPECT_EQUAL(ShaderCompiler::literal(-0.25), std::string("-0.25f"));
    EXPECT_EQUAL(ShaderCompiler::literal(std::nan("")), std::string("0.0f"));
    EXPECT_EQUAL(ShaderCompiler::literal(1e20), std::string("1e+20f"));
}

TEST(shaderAnyInputsWorkInTheWidestTypeWired) {
    ShaderGraph graph;
    graph.addNode(kinds::uv, {}, "uv");
    graph.addNode(kinds::add, {}, "sum");
    graph.connect("uv", "uv", "sum", "a");
    CompiledShader shader = ShaderCompiler::compile(graph, "sum", "value");
    EXPECT(!shader.error.has_value());
    EXPECT(contains(shader.source, "const float2 v2 = (v1) + (float2(0.0f));"));
}

TEST(shaderGraphEditsRefuseWhatCannotBeDone) {
    ShaderGraph graph;
    graph.addNode(kinds::sine, {});
    EXPECT_EQUAL(graph.nodes[0].id, std::string("sine1"));
    EXPECT_EQUAL(graph.freshID(kinds::mapRange), std::string("mapRange1"));
    graph.addNode(kinds::add, {}, "sum");
    graph.addNode(kinds::shader, {}, "shader");
    EXPECT_PROBLEM(graph.addNode(kinds::shader, {}), "already has a Shader output node");
    EXPECT_PROBLEM(graph.addNode(kinds::add, {}, "sum"), "A node named sum already exists.");
    EXPECT_PROBLEM(graph.connect("sum", "value", "sum", "a"), "would make a loop");
    EXPECT_PROBLEM(graph.connect("sum", "value", "shader", "nothing"), "has no input nothing");
    EXPECT_PROBLEM(graph.setParameter("sum", "zzz", 1), "has no number zzz; it has a, b.");
    EXPECT_PROBLEM(graph.setParameter("sum", "a", std::nan("")), "is not a finite number");
    EXPECT_PROBLEM(graph.setOption("sum", "mode", 1), "has no option mode");
    EXPECT_PROBLEM(graph.disconnect("sum", "a"), "Nothing is connected to sum.a.");
    graph.connect("sum", "value", "shader", "color");
    graph.removeNode("sum");
    EXPECT(graph.connections.empty());
    EXPECT(NodeKind::named("add").has_value());
    EXPECT(!NodeKind::named("no-such-kind").has_value());
}

namespace {

/// The source of the one-node graph `kind`, with the preview feeds the previews give it, read at `output`.
std::string sourceOfKind(const std::string& kind, const std::string& output) {
    for (const Preview& preview : previews(NodeKind{kind}, Feeding::numbers)) {
        if (preview.output.id != output) continue;
        CompiledShader shader = preview.compile();
        EXPECT(!shader.error.has_value());
        return shader.source;
    }
    editor::testing::fail(__FILE__, __LINE__, kind + "." + output + " has no preview");
    return "";
}

} // namespace

TEST(shaderColorKindsCallTheColourKernels) {
    EXPECT(contains(sourceOfKind("hue-shift", "color"), "hueShifted("));
    EXPECT(contains(sourceOfKind("blend", "color"), "compositedWithBlend("));
}

TEST(shaderNoiseKindsCallTheNoiseKernels) {
    EXPECT(contains(sourceOfKind("perlin-noise", "value"), "baseNoise("));
    // Voronoi's setup is computed once and every output reads it.
    std::string voronoi = sourceOfKind("voronoi", "difference");
    EXPECT(contains(voronoi, "cellularNoise("));
    EXPECT(contains(voronoi, ".second"));
}

TEST(shaderPatternKindsPassTheirPixelFootprintFromFwidth) {
    // The kernels cannot call fwidth, so the template hands them the footprint.
    EXPECT(contains(sourceOfKind("checker", "value"), "checkerboardPattern("));
    EXPECT(contains(sourceOfKind("checker", "value"), "fwidth("));
    EXPECT(contains(sourceOfKind("stripes", "value"), "fwidth(stripeCoordinate("));
    EXPECT(contains(sourceOfKind("halftone", "value"), "fwidth(halftoneCoordinate("));
}

TEST(shaderShapeKindsCallTheShapeKernels) {
    EXPECT(contains(sourceOfKind("star-sdf", "distance"), "starDistance(rotatedShapePoint("));
    EXPECT(contains(sourceOfKind("sdf-fill", "mask"), "distanceFillMask("));
}

TEST(shaderRaymarchSceneIsALambdaOfTheMarchPoint) {
    ShaderGraph graph;
    graph.addNode(NodeKind{"raymarch"}, {}, "march");
    graph.addNode(NodeKind{"box-3d"}, {}, "box");
    graph.connect("box", "distance", "march", "scene");
    CompiledShader shader = ShaderCompiler::compile(graph, "march", "color");
    EXPECT(!shader.error.has_value());
    EXPECT(contains(shader.source, "[&](float3 marchPoint) -> float"));
    EXPECT(contains(shader.source, "boxDistance("));
    EXPECT(contains(shader.source, "sphereTrace("));
    EXPECT(contains(shader.source, "shadedSurface("));
}

TEST(shaderMarchPointOutsideAScenePreviewsInsideARaymarchHost) {
    // A shape reads March Point through its position's fallback, so its preview needs a Raymarch node around it.
    CompiledShader shader = previews(NodeKind{"torus-3d"}, Feeding::numbers).front().compile();
    EXPECT(!shader.error.has_value());
    EXPECT(contains(shader.source, "torusDistance("));
    EXPECT(contains(shader.source, "[&](float3 marchPoint) -> float"));
}

namespace {

/// The text of the Metal function `name`, from its signature to the next blank line, of a compiled shader.
std::string metalFunction(const std::string& source, const std::string& signature) {
    std::size_t start = source.find(signature);
    if (start == std::string::npos) return "";
    return source.substr(start, source.find("\n}\n", start) - start);
}

} // namespace

TEST(shaderTextureKindsBindTheirTexturesAndSample) {
    // Sample Texture reads an Image; the image is texture 0 of the fragment function.
    ShaderGraph graph;
    graph.addNode(kinds::image, {}, "image");
    graph.addNode(kinds::sampleTexture, {}, "photo");
    graph.connect("image", "texture", "photo", "texture");
    CompiledShader shader = ShaderCompiler::compile(graph, "photo", "color");
    EXPECT(!shader.error.has_value());
    EXPECT_EQUAL(shader.textures.size(), static_cast<std::size_t>(1));
    EXPECT(shader.textures[0].feed == TextureFeed::image);
    EXPECT(contains(shader.source, "texture2d<float> uTexture0 [[texture(0)]]"));
    EXPECT(contains(shader.source, "sampleImageWith((uTexture0),"));
    // Previous Frame and Video are textures too; the former tells the host to keep the frames it draws.
    ShaderGraph memory;
    memory.addNode(kinds::previousFrame, {}, "memory");
    memory.addNode(NodeKind{"texture-size"}, {}, "size");
    memory.connect("memory", "texture", "size", "texture");
    CompiledShader remembered = ShaderCompiler::compile(memory, "size", "size");
    EXPECT(!remembered.error.has_value());
    EXPECT(remembered.readsPreviousFrame());
    EXPECT(contains(remembered.source, ".get_width()"));
    EXPECT(NodeCatalog::knows("video") && NodeCatalog::definition("video").feed == TextureFeed::video);
}

TEST(shaderImageEffectKindsCallTheEffectKernels) {
    EXPECT(contains(sourceOfKind("blur", "color"), "blurImage((uTexture0),"));
    EXPECT(contains(sourceOfKind("vignette", "color"), "vignetteColor("));
    // The taps are Metal's and the arithmetic is the kernels': check the helper bodies call them.
    std::string source = sourceOfKind("bloom", "color");
    EXPECT(contains(metalFunction(source, "float4 bloomImage("), "bloomRingOffset("));
    EXPECT(contains(metalFunction(source, "float4 bloomImage("), "bloomCombined("));
    EXPECT(contains(metalFunction(source, "float4 kuwahara("), "kuwaharaSelectedMean("));
    EXPECT(contains(metalFunction(source, "float2 sobelOf("), "sobelGradient("));
    EXPECT(contains(sourceOfKind("edge-detect", "edge"), "edgeStrength((uTexture0),"));
}

TEST(shaderSimulationKindsCallTheSimulationKernels) {
    std::string source = sourceOfKind("reaction-diffusion", "state");
    EXPECT(contains(source, "reactionDiffusionStep((uTexture0),"));
    std::string step = metalFunction(source, "float4 reactionDiffusionStep(");
    EXPECT(contains(step, "grayScottReaction("));
    EXPECT(contains(step, "grayScottLaplacian("));
    EXPECT(contains(step, "grayScottBrushed("));
    EXPECT(contains(metalFunction(source, "float4 fluidStep("), "fluidNextState("));
    EXPECT(contains(metalFunction(source, "float4 lifeStep("), "lifeRule") || contains(metalFunction(source, "float4 lifeStep("), "lifeNextState("));
    EXPECT(contains(sourceOfKind("particles", "density"), "particleField("));
}

TEST(shaderSimulationPreviewKeepsItsStateInFeedback) {
    std::optional<CompiledShader> shader = previewShader(NodeKind{"game-of-life"});
    EXPECT(shader.has_value());
    EXPECT(shader->writesFeedback);
    EXPECT(shader->readsPreviousFrame());
    EXPECT(contains(shader->source, "GraphOutput {"));
}

TEST(shaderColorLookupCompilesWithAnImageFeedingTheTable) {
    ShaderGraph graph;
    graph.addNode(kinds::image, {}, "table");
    graph.addNode(NodeKind{"color-lookup"}, {}, "grade");
    graph.connect("table", "texture", "grade", "table");
    CompiledShader shader = ShaderCompiler::compile(graph, "grade", "color");
    EXPECT(!shader.error.has_value());
    EXPECT(contains(shader.source, "lookupColor((uTexture0),"));
    // Without a table the node says what is missing instead of compiling.
    ShaderGraph bare;
    bare.addNode(NodeKind{"color-lookup"}, {}, "grade");
    EXPECT(ShaderCompiler::compile(bare, "grade", "color").error.has_value());
}
