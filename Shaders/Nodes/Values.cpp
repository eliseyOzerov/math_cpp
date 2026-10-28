// Constants and the numbers a graph exposes (graphics_metal's Nodes/Values.swift).
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

NodeGroup valueNodes() {
    NodeBuilder c(NodeCategory::values);
    return {c.category, {
        c.node({.name = "number", .label = "Number", .summary = "A number.", .symbol = "number",
                .outputs = {out("value", "Value", number, "{@value}")}, .numbers = {{"value", 1}}}),
        c.node({.name = "integer", .label = "Integer", .summary = "A whole number.", .symbol = "textformat.123",
                .outputs = {out("value", "Value", number, "floor({@value} + 0.5f)")}, .numbers = {{"value", 3}}}),
        c.node({.name = "boolean", .label = "Boolean", .summary = "True (1) or false (0).", .symbol = "switch.2",
                .outputs = {out("value", "Value", number, "float({#value})")},
                .options = {option("value", "Value", {"False", "True"}, 1)}}),
        c.node({.name = "constant", .label = "Constant", .summary = "Pi, tau, e, the square root of two or the golden ratio.",
                .symbol = "pi", .outputs = {out("value", "Value", number, "namedConstant<float>({#which})")},
                .options = {option("which", "Constant", {"π", "τ", "e", "√2", "φ"})}}),
        c.node({.name = "vector2-constant", .label = "Vector 2", .summary = "A fixed two-component vector.",
                .outputs = {out("vector", "Vector", vector2, "float2({@x}, {@y})")}, .numbers = {{"x", 0.5}, {"y", 0.5}}}),
        c.node({.name = "vector3-constant", .label = "Vector 3", .summary = "A fixed three-component vector.",
                .outputs = {out("vector", "Vector", vector3, "float3({@x}, {@y}, {@z})")},
                .numbers = {{"x", 0.5}, {"y", 0.5}, {"z", 0.5}}}),
        c.node({.name = "vector4-constant", .label = "Vector 4", .summary = "A fixed four-component vector.",
                .outputs = {out("vector", "Vector", vector4, "float4({@x}, {@y}, {@z}, {@w})")},
                .numbers = {{"x", 0.5}, {"y", 0.5}, {"z", 0.5}, {"w", 1}}}),
        c.node({.name = "random-seed", .label = "Random Seed", .summary = "A random number from 0 to 1, fixed by its seed.",
                .symbol = "dice", .outputs = {out("value", "Value", number, "hash11({@seed} * 7.31f + 0.5f)")},
                .numbers = {{"seed", 1}}}),
        c.node({.name = "parameter", .label = "Parameter",
                .summary = "A named number shown as a control outside the graph, so a shader becomes a preset.",
                .symbol = "slider.horizontal.3",
                .outputs = {out("value", "Value", number, "mix({@minimum}, {@maximum}, clamp({@value}, 0.0f, 1.0f))")},
                .text = NodeText{"Name", "Amount", false}, .numbers = {{"value", 0.5}, {"minimum", 0}, {"maximum", 1}}}),
    }};
}

} // namespace editor::shaders
