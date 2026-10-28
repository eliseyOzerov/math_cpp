// A shader graph: nodes with their parameters and positions on a node canvas, and the connections
// between their sockets (graphics_metal's Graph/ShaderGraph.swift). An app edits it, by hand or
// through an agent, and `ShaderCompiler` compiles it to Metal Shading Language. Kinds and
// parameter names match Kagiro's web editor, so graphs can travel between the two.
#pragma once

#include <exception>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace editor::shaders {

/// Why an edit or a compilation could not go on, as one readable sentence.
///
/// Use when: refusing a wire, a missing node, a cycle, a name that is not a kind
/// Tags: shader graph error, GraphEditError, exception
/// Node: none (shader graph)
class GraphEditError : public std::exception {
public:
    explicit GraphEditError(std::string message) : message(std::move(message)) {}
    const char* what() const noexcept override { return message.c_str(); }

    std::string message;
};

/// What a socket carries. Any value but a texture can feed any other: numbers spread across
/// vectors, vectors drop or pad components, and a colour read as a number is its luminance.
///
/// Use when: declaring a node's sockets; checking a wire
/// Tags: socket type, port type, number, vector, colour, texture, any
/// Node: none (shader graph)
enum class SocketType { number, vector2, vector3, vector4, color, texture, any };

/// The name graphs store for `type` ("number", "vector2", ...).
std::string socketTypeName(SocketType type);

/// Every socket type, in declaration order.
const std::vector<SocketType>& allSocketTypes();

/// Whether a wire may run from a socket of type `from` into one of type `to`: textures only
/// feed textures, and everything else feeds everything else.
bool canFeed(SocketType from, SocketType to);

/// Whether an unwired input of this type is a number to drag (`number` and `any`).
bool isNumeric(SocketType type);

/// What an unwired input reads: Metal code and what the canvas calls it.
///
/// Use when: an input should default to something other than a number, such as UV or a colour
/// Tags: fallback, default input, unwired input
/// Node: none (shader graph)
struct Fallback {
    std::string code;
    std::string label;
    /// The loop variable the code reads, for inputs that only make sense inside a loop.
    std::optional<std::string> scope;

    bool operator==(const Fallback& other) const {
        return code == other.code && label == other.label && scope == other.scope;
    }

    static Fallback uv();
    static Fallback centred();
    static Fallback zero();
    static Fallback one();
    static Fallback half();
    static Fallback black();
    static Fallback white();
    static Fallback gray();
    static Fallback clear();
    static Fallback up();
    static Fallback forward();
    static Fallback sun();
    static Fallback marchPoint();
    static Fallback rayOrigin();
    static Fallback rayDirection();
    static Fallback time();
    static Fallback middle();
    static Fallback loopValue();
};

/// One input or output socket of a node kind.
///
/// Use when: reading what a node takes and gives
/// Tags: port, socket, input, output
/// Node: none (shader graph)
struct NodePort {
    std::string id;
    std::string label;
    SocketType type = SocketType::number;
    /// What an unwired input that is not a number reads instead.
    std::optional<Fallback> fallback;
    /// The span a number input's field drags across, where it has a natural one.
    std::optional<std::pair<double, double>> range;

    /// Whether the input is a number to drag when unwired (rather than reading its fallback).
    bool isDraggable() const { return isNumeric(type) && !fallback.has_value(); }
};

/// The colour spaces a Color node can be edited in.
enum class ColorSpace { rgb, cmyk, oklab, okhsv };

std::string colorSpaceName(ColorSpace space);
const std::vector<ColorSpace>& allColorSpaces();

/// A kind of node, by the name graphs store. `NodeCatalog` knows every kind; a name it does not
/// know is not a kind.
///
/// Use when: naming the kind of a node; looking a kind up from text in a stored graph
/// Tags: node kind, node type, kind name
/// Node: none (shader graph)
struct NodeKind {
    std::string rawValue;

    /// The kind called `name`, or nothing when the catalog has no such kind.
    static std::optional<NodeKind> named(const std::string& name);
    /// Every kind in the catalog, in menu order.
    static std::vector<NodeKind> allCases();

    bool operator==(const NodeKind& other) const { return rawValue == other.rawValue; }
    bool operator!=(const NodeKind& other) const { return rawValue != other.rawValue; }
    bool operator<(const NodeKind& other) const { return rawValue < other.rawValue; }
};

/// The kinds the web editor shares and the ones apps reach for by name. Only these have a
/// constant; every other kind is found by `NodeKind::named`.
namespace kinds {
inline const NodeKind number{"number"};
inline const NodeKind time{"time"};
inline const NodeKind uv{"uv"};
inline const NodeKind separateVector2{"separate-vector2"};
inline const NodeKind color{"color"};
inline const NodeKind add{"add"};
inline const NodeKind multiply{"multiply"};
inline const NodeKind sine{"sine"};
inline const NodeKind mapRange{"map-range"};
inline const NodeKind shader{"shader"};
inline const NodeKind feedbackOutput{"feedback-output"};
inline const NodeKind previousFrame{"previous-frame"};
inline const NodeKind image{"image"};
inline const NodeKind sampleTexture{"sample-texture"};
inline const NodeKind reroute{"reroute"};
inline const NodeKind comment{"comment"};
inline const NodeKind subgraph{"subgraph"};
inline const NodeKind groupInput{"group-input"};
inline const NodeKind groupOutput{"group-output"};
inline const NodeKind customCode{"custom-code"};
inline const NodeKind setVariable{"set-variable"};
inline const NodeKind getVariable{"get-variable"};
} // namespace kinds

class ShaderGraph;

/// A graph held inside a node: a Subgraph node's insides. Copying it copies the graph.
class NestedGraph {
public:
    NestedGraph();
    explicit NestedGraph(const ShaderGraph& graph);
    NestedGraph(const NestedGraph& other);
    NestedGraph& operator=(const NestedGraph& other);
    NestedGraph(NestedGraph&&) noexcept;
    NestedGraph& operator=(NestedGraph&&) noexcept;
    ~NestedGraph();

    ShaderGraph& graph() { return *inner; }
    const ShaderGraph& graph() const { return *inner; }

private:
    std::unique_ptr<ShaderGraph> inner;
};

/// A point on the node canvas, in canvas points; the origin is the top left.
struct CanvasPoint {
    double x = 0;
    double y = 0;
};

/// A node on the canvas.
///
/// Use when: reading or building the nodes of a graph
/// Tags: graph node, node instance
/// Node: none (shader graph)
struct GraphNode {
    std::string id;
    NodeKind kind;
    /// The top-left corner of the node's box on the canvas.
    double x = 0;
    double y = 0;
    /// The numbers of its unconnected number inputs (and a Number node's `value`), by port id.
    std::map<std::string, double> parameters;
    /// The space a Color node's channels are in; nothing for every other kind.
    std::optional<ColorSpace> colorSpace;
    /// The file an Image node reads, absolute or relative to the project folder.
    std::optional<std::string> imagePath;
    /// Whether the box is folded to its title.
    std::optional<bool> isCollapsed;
    /// The words of a node that holds some: a comment, a name, or Metal code.
    std::optional<std::string> text;
    /// A Subgraph node's insides.
    std::optional<NestedGraph> subgraph;
};

/// A wire from an output socket to an input socket.
struct GraphConnection {
    std::string id;
    std::string sourceNode;
    std::string sourcePort;
    std::string targetNode;
    std::string targetPort;

    bool operator==(const GraphConnection& other) const = default;
};

/// A shader graph.
///
/// Use when: building, editing or compiling a shader
/// Tags: shader graph, node graph, material graph, Kagiro graph
/// Node: none (shader graph)
class ShaderGraph {
public:
    std::vector<GraphNode> nodes;
    std::vector<GraphConnection> connections;

    // Reading

    /// The node called `id`, or nothing.
    const GraphNode* node(const std::string& id) const;
    GraphNode* node(const std::string& id);

    /// The input sockets of `node`: its kind's, or a Color node's channels in its colour space.
    std::vector<NodePort> inputs(const GraphNode& node) const;
    std::vector<NodePort> outputs(const GraphNode& node) const;

    /// The wire feeding `port` of `node`, if any.
    const GraphConnection* connection(const std::string& node, const std::string& port) const;

    /// The node every graph renders through (the first Shader node), or nothing.
    const GraphNode* outputNode() const;

    // Editing. Every edit that cannot be made throws a GraphEditError.

    /// A fresh id for a node of `kind`, such as `sine2`.
    std::string freshID(const NodeKind& kind) const;

    /// Adds a node of `kind` with its default parameters at `position` and returns it. An empty
    /// `id` picks a fresh one.
    GraphNode addNode(const NodeKind& kind, CanvasPoint position, const std::string& id = "");

    /// Removes the node `id` and every wire to or from it.
    void removeNode(const std::string& id);

    /// Wires `sourcePort` of `sourceNode` into `targetPort` of `targetNode`, replacing the wire
    /// already feeding that input. Refuses mismatched types and wires that would make a loop.
    GraphConnection connect(const std::string& sourceNode, const std::string& sourcePort,
                            const std::string& targetNode, const std::string& targetPort);

    /// Removes the wire feeding `port` of `node`.
    void disconnect(const std::string& node, const std::string& port);

    /// Sets the number `parameter` of `node`.
    void setParameter(const std::string& node, const std::string& parameter, double value);

    /// Changes a Color node's space, resetting its channels to that space's defaults and
    /// dropping wires into channels the new space lacks.
    void setColorSpace(const std::string& node, ColorSpace space);

    /// Sets the words of a node that holds some.
    void setText(const std::string& node, const std::string& text);

    /// Picks choice `choice` of the dropdown `option` of `node`.
    void setOption(const std::string& node, const std::string& option, int choice);

    void moveNode(const std::string& node, CanvasPoint position);

    /// A new Subgraph node's insides: its two ends, apart.
    static ShaderGraph emptyGroup();

    /// A graph to start from: an animated OKHSV colour sweeping across UV, into the Shader output.
    static ShaderGraph starter();

private:
    /// Whether following wires downstream from `start` reaches `end`.
    bool reaches(const std::string& start, const std::string& end) const;
};

} // namespace editor::shaders
