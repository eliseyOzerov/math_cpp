// How a node kind is described: its sockets, dropdowns and text, and the Metal each output
// compiles to (graphics_metal's Graph/NodeSpec.swift). Most kinds are nothing but this
// description, a template per output, so adding one is a few lines in `Nodes/`.
//
// A template is Metal with placeholders the compiler fills in:
//   {id}   the value at input `id`, converted to the input's type (an `any` input: to the type
//          the node works in)
//   {#id}  dropdown `id`'s choice, as an integer literal
//   {@id}  the node's number `id` (one with no socket, such as a Number node's value), as a
//          float literal
//   {$}    the node's setup value: its `setup` template, compiled once into a constant
//   {T}    the Metal type the node works in (`float`, `float2`, ...), for `any` nodes
//   {feed} the texture a source kind (one with a `feed`) binds
//
// Every number in a template is a float literal (`0.5f`): the kernels are templates on the
// number type, and `smoothUnion(a, b, 0.5)` would not deduce one type from a float and a double.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ShaderGraph.h"

namespace editor::shaders {

/// The menu a kind sits under.
enum class NodeCategory {
    values, vectors, vectorMath, transforms, math, trigonometry, waves, noise, patterns, shapes,
    sdfOperations, color, blending, textures, imageEffects, time, interaction, raymarching,
    simulation, utility, output
};

/// What the menu calls `category`.
std::string categoryLabel(NodeCategory category);
const std::vector<NodeCategory>& allCategories();
/// The id graphs and menus use for `category`, such as "vectorMath".
std::string categoryName(NodeCategory category);

/// What the host binds for a texture a compiled shader reads.
enum class TextureFeed {
    image,         ///< An image file.
    video,         ///< A video file, playing on a loop.
    camera,        ///< The default camera.
    audioSpectrum, ///< The microphone's spectrum: 256 x 1, red the magnitude of each band, green the waveform.
    controls,      ///< Keys, MIDI and OSC: 256 x 3.
    previousFrame, ///< What the graph drew last frame, at the canvas's size, in half floats.
    noise,         ///< 256 x 256 of white noise, four independent channels, repeating.
    blueNoise      ///< 64 x 64 of blue noise (void-and-cluster), repeating.
};

/// Whether every node of this feed reads the same texture (all but Image and Video).
bool isSharedFeed(TextureFeed feed);
/// Whether the node names a file (Image and Video).
bool readsFile(TextureFeed feed);

/// A dropdown on a node, stored in its parameters as the chosen index.
struct NodeOption {
    std::string id;
    std::string label;
    std::vector<std::string> choices;
    int defaultIndex = 0;
};

/// The words a node holds, such as a comment or a snippet of Metal.
struct NodeText {
    std::string label;
    std::string initial;
    /// Whether it is code, edited in a bigger field and shown in a fixed-width font.
    bool isCode = false;
};

/// An input compiled once per call rather than once per pixel: a Raymarch node's scene is a
/// function of the point being marched, a Loop node's body a function of the step. Inside it,
/// the nodes that read those variables (March Point, Loop Index) mean the call's values.
struct InputScope {
    /// The variables the function takes, as Metal parameters: (type, name), such as
    /// ("float3", "marchPoint").
    std::vector<std::pair<std::string, std::string>> parameters;
};

/// One kind of node.
///
/// Use when: reading what a kind takes, gives and compiles to
/// Tags: node definition, node spec, template, catalogue entry
/// Node: none (shader graph)
struct NodeDefinition {
    NodeKind kind;
    std::string label;
    std::string summary;
    NodeCategory category = NodeCategory::utility;
    std::vector<NodePort> inputs;
    std::vector<NodePort> outputs;
    std::vector<NodeOption> options;
    std::optional<NodeText> text;
    /// The starting values of its number inputs and dropdowns, by id.
    std::map<std::string, double> defaults;
    /// Compiled once per node, before its outputs; they read it as `{$}`.
    std::optional<std::string> setup;
    /// Each output's Metal, by output id. Kinds the compiler builds itself have none.
    std::map<std::string, std::string> templates;
    /// Where an SF Symbol or a short sign stands for the kind on the canvas.
    std::optional<std::string> symbol;
    /// Inputs compiled as a function of loop variables, such as a Raymarch node's scene, by id.
    std::map<std::string, InputScope> scopes;
    /// What the host binds for the texture this kind gives, for kinds that are a texture source.
    std::optional<TextureFeed> feed;

    /// Kinds that only make sense inside something else (a subgraph's ends), left out of the menu.
    bool isHidden() const {
        return kind == kinds::groupInput || kind == kinds::groupOutput;
    }
};

// Building specs. A family file lists its kinds with these.

/// An input as a spec lists it: the socket and its starting number.
struct InputSpec {
    NodePort port;
    std::optional<double> initial;
    std::optional<InputScope> scope;
};

/// An output as a spec lists it: the socket and its Metal.
struct OutputSpec {
    NodePort port;
    std::optional<std::string> templateText;
};

using Range = std::optional<std::pair<double, double>>;

namespace spec {

/// The span `lowest` to `highest`, for a number input's `range`.
inline Range between(double lowest, double highest) { return std::make_pair(lowest, highest); }

/// A number input, dragged when unwired.
InputSpec num(const std::string& id, const std::string& label, double initial = 0, Range range = std::nullopt);
/// An input that takes any value: the node works on the widest one wired in. Unwired, it is a
/// number to drag.
InputSpec anyIn(const std::string& id, const std::string& label, double initial = 0, Range range = std::nullopt);
/// A number input that reads the clock until something is wired in.
InputSpec timeIn(const std::string& id = "time", const std::string& label = "Time");
InputSpec vec2(const std::string& id, const std::string& label, std::optional<Fallback> fallback = Fallback::uv());
InputSpec vec3(const std::string& id, const std::string& label, std::optional<Fallback> fallback = std::nullopt);
InputSpec vec4(const std::string& id, const std::string& label, std::optional<Fallback> fallback = std::nullopt);
InputSpec col(const std::string& id, const std::string& label, std::optional<Fallback> fallback = Fallback::black());
InputSpec tex(const std::string& id = "texture", const std::string& label = "Texture",
              std::optional<Fallback> fallback = std::nullopt);
/// An input compiled as a function of `parameters`; see `InputScope`.
InputSpec scoped(InputSpec input, std::vector<std::pair<std::string, std::string>> parameters);
OutputSpec out(const std::string& id, const std::string& label, SocketType type,
               std::optional<std::string> templateText = std::nullopt);
NodeOption option(const std::string& id, const std::string& label, std::vector<std::string> choices,
                  int defaultIndex = 0);

} // namespace spec

/// Everything `NodeBuilder::node` takes, in this order (designated initializers).
struct NodeParts {
    std::string name;
    std::string label;
    std::string summary;
    std::optional<std::string> symbol;
    std::vector<InputSpec> inputs;
    std::vector<OutputSpec> outputs;
    std::vector<NodeOption> options;
    std::optional<std::string> setup;
    std::optional<NodeText> text;
    std::optional<TextureFeed> feed;
    std::map<std::string, double> numbers;
};

/// What `NodeBuilder::simple` takes beyond its fixed arguments.
struct SimpleExtras {
    std::optional<std::string> symbol;
    std::vector<NodeOption> options;
    /// The id and label of the one output.
    std::pair<std::string, std::string> output{"value", "Value"};
};

/// Builds the kinds of one category; a family function receives one.
class NodeBuilder {
public:
    explicit NodeBuilder(NodeCategory category) : category(category) {}

    /// A kind with any sockets.
    NodeDefinition node(NodeParts parts) const;

    /// A kind with one output, `value` unless `extras.output` names it, of `type`.
    NodeDefinition simple(const std::string& name, const std::string& label, const std::string& summary,
                          std::vector<InputSpec> inputs, SocketType type, const std::string& templateText,
                          SimpleExtras extras = {}) const;

    NodeCategory category;
};

/// A list of kinds under one menu heading.
struct NodeGroup {
    NodeCategory category;
    std::vector<NodeDefinition> nodes;
};

} // namespace editor::shaders
