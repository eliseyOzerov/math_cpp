// Tidying a graph, reusing parts of it, writing Metal by hand, and the ends a graph draws to
// (graphics_metal's Nodes/Utility.swift). Loop's `loopAccumulate` takes a function, so it is
// Metal-backend text (MetalBackend.cpp), not a kernel.
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

NodeGroup utilityNodes() {
    NodeBuilder c(NodeCategory::utility);
    return {c.category, {
        c.simple("reroute", "Reroute", "A dot to tidy a wire; passes its value through.", {anyIn("value", "Value")}, any,
                 "{value}", {.symbol = "circle.fill"}),
        c.simple("preview", "Preview", "Shows the value wired through it on the node.", {anyIn("value", "Value")}, any,
                 "{value}", {.symbol = "eye"}),
        c.node({.name = "comment", .label = "Comment", .summary = "A note on the canvas.", .symbol = "text.bubble",
                .text = NodeText{"Note", "A note", false}}),
        c.node({.name = "subgraph", .label = "Subgraph", .summary = "A graph inside a node, with up to four values in and out.",
                .symbol = "square.stack.3d.up",
                .inputs = {anyIn("a", "A"), anyIn("b", "B"), anyIn("c", "C"), anyIn("d", "D")},
                .outputs = {out("a", "A", any), out("b", "B", any), out("c", "C", any), out("d", "D", any)}}),
        c.node({.name = "group-input", .label = "Group Input", .summary = "Inside a subgraph: what is wired into it.",
                .symbol = "arrow.right.to.line",
                .outputs = {out("a", "A", any), out("b", "B", any), out("c", "C", any), out("d", "D", any)}}),
        c.node({.name = "group-output", .label = "Group Output", .summary = "Inside a subgraph: what it gives out.",
                .symbol = "rectangle.portrait.and.arrow.right",
                .inputs = {anyIn("a", "A"), anyIn("b", "B"), anyIn("c", "C"), anyIn("d", "D")}}),
        c.node({.name = "custom-code", .label = "Custom Code",
                .summary = "A Metal function body of inputs a to d (as float4) and uv, returning a colour.",
                .symbol = "chevron.left.forwardslash.chevron.right",
                .inputs = {anyIn("a", "A"), anyIn("b", "B"), anyIn("c", "C"), anyIn("d", "D"), vec2("uv", "UV")},
                .outputs = {out("color", "Color", color), out("value", "Value", number)},
                .text = NodeText{"Code", "return float4(uv, 0.5f + 0.5f * sin(a.x), 1.0f);", true}}),
        c.node({.name = "set-variable", .label = "Set Variable",
                .summary = "Names a value so Get Variable nodes can read it without a wire.", .symbol = "tray.and.arrow.down",
                .inputs = {anyIn("value", "Value")}, .text = NodeText{"Name", "value", false}}),
        c.node({.name = "get-variable", .label = "Get Variable", .summary = "The value of the Set Variable node with the same name.",
                .symbol = "tray.and.arrow.up", .outputs = {out("value", "Value", any)}, .text = NodeText{"Name", "value", false}}),
        c.node({.name = "loop", .label = "Loop", .summary = "Runs its body a number of times, each step reading the last step's value.",
                .symbol = "arrow.clockwise",
                .inputs = {scoped(vec4("body", "Body", Fallback::loopValue()), {{"float", "loopIndex"}, {"float4", "loopValue"}}),
                           vec4("initial", "Initial", Fallback::clear()), num("count", "Count", 8, between(0, 64))},
                .outputs = {out("value", "Value", vector4, "loopAccumulate({body}, {initial}, {count})")}}),
        c.node({.name = "loop-index", .label = "Loop Index", .summary = "Inside a Loop's body: the step, from 0.",
                .symbol = "number", .outputs = {out("index", "Index", number, "loopIndex")}}),
        c.node({.name = "loop-value", .label = "Loop Value", .summary = "Inside a Loop's body: the last step's value.",
                .symbol = "arrow.clockwise", .outputs = {out("value", "Value", vector4, "loopValue")}}),
        c.simple("switch", "Switch", "One of four values, chosen by index (0 to 3).",
                 {num("index", "Index", 0, between(0, 3)), anyIn("a", "A"), anyIn("b", "B", 0.33), anyIn("c", "C", 0.66),
                  anyIn("d", "D", 1)}, any, "selectedByIndex({index}, {a}, {b}, {c}, {d})", {.symbol = "arrow.triangle.branch"}),
        c.simple("debug", "Debug", "A value as colour: a heatmap, contour bands, or red for negative and green for positive.",
                 {anyIn("value", "Value", 0.5)}, color, "debugColor({value}, {#mode})",
                 {.symbol = "ladybug", .options = {option("mode", "View", {"Heatmap", "Bands", "Sign"})}}),
        c.simple("layers", "Layers", "Up to four colours stacked by their alpha, A at the bottom.",
                 {col("a", "A", Fallback::clear()), col("b", "B", Fallback::clear()), col("c", "C", Fallback::clear()),
                  col("d", "D", Fallback::clear())}, color, "layersOver({a}, {b}, {c}, {d})", {.symbol = "square.3.layers.3d"}),
    }};
}

NodeGroup outputNodes() {
    NodeBuilder c(NodeCategory::output);
    return {c.category, {
        c.node({.name = "shader", .label = "Shader", .summary = "What the canvas shows.", .symbol = "display",
                .inputs = {col("color", "Color")}}),
        c.node({.name = "feedback-output", .label = "Feedback Output",
                .summary = "Keeps a colour for Previous Frame to hand back next frame instead of the picture: how a simulation remembers its state while showing a view.",
                .symbol = "arrow.triangle.2.circlepath", .inputs = {col("color", "Color")}}),
    }};
}

} // namespace editor::shaders
