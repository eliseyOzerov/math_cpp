// Building vectors from numbers and back, and the maths of vectors (graphics_metal's
// Nodes/Vectors.swift).
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

NodeGroup vectorNodes() {
    NodeBuilder c(NodeCategory::vectors);
    const std::vector<std::string> components = {"X", "Y", "Z", "W", "0", "1"};
    return {c.category, {
        c.simple("combine-vector2", "Combine Vector 2", "A vector from two numbers.", {num("x", "X"), num("y", "Y")}, vector2,
                 "float2({x}, {y})", {.symbol = "arrow.merge"}),
        c.simple("combine-vector3", "Combine Vector 3", "A vector from three numbers.",
                 {num("x", "X"), num("y", "Y"), num("z", "Z")}, vector3, "float3({x}, {y}, {z})", {.symbol = "arrow.merge"}),
        c.simple("combine-vector4", "Combine Vector 4", "A vector from four numbers.",
                 {num("x", "X"), num("y", "Y"), num("z", "Z"), num("w", "W")}, vector4, "float4({x}, {y}, {z}, {w})",
                 {.symbol = "arrow.merge"}),
        c.node({.name = "separate-vector2", .label = "Separate Vector 2", .summary = "A vector's two components.",
                .symbol = "arrow.branch", .inputs = {vec2("vector", "Vector")},
                .outputs = {out("x", "X", number, "{vector}.x"), out("y", "Y", number, "{vector}.y")}}),
        c.node({.name = "separate-vector3", .label = "Separate Vector 3", .summary = "A vector's three components.",
                .symbol = "arrow.branch", .inputs = {vec3("vector", "Vector")},
                .outputs = {out("x", "X", number, "{vector}.x"), out("y", "Y", number, "{vector}.y"),
                            out("z", "Z", number, "{vector}.z")}}),
        c.node({.name = "separate-vector4", .label = "Separate Vector 4", .summary = "A vector's four components.",
                .symbol = "arrow.branch", .inputs = {vec4("vector", "Vector")},
                .outputs = {out("x", "X", number, "{vector}.x"), out("y", "Y", number, "{vector}.y"),
                            out("z", "Z", number, "{vector}.z"), out("w", "W", number, "{vector}.w")}}),
        c.node({.name = "swizzle", .label = "Swizzle", .summary = "Picks and reorders a vector's components, such as .yx or .xxy.",
                .inputs = {vec4("vector", "Vector")},
                .outputs = {out("vector2", "XY", vector2, "{$}.xy"), out("vector3", "XYZ", vector3, "{$}.xyz"),
                            out("vector4", "XYZW", vector4, "{$}")},
                .options = {option("first", "1st", components, 0), option("second", "2nd", components, 1),
                            option("third", "3rd", components, 2), option("fourth", "4th", components, 3)},
                .setup = "swizzled({vector}, int4({#first}, {#second}, {#third}, {#fourth}))"}),
        c.node({.name = "separate-color", .label = "Separate Color", .summary = "A colour's red, green, blue and alpha.",
                .symbol = "rectangle.split.3x1", .inputs = {col("color", "Color", Fallback::white())},
                .outputs = {out("r", "R", number, "{color}.r"), out("g", "G", number, "{color}.g"),
                            out("b", "B", number, "{color}.b"), out("a", "A", number, "{color}.a")}}),
        c.simple("combine-color", "Combine Color", "A colour from red, green, blue and alpha.",
                 {num("r", "R", 0.5, between(0, 1)), num("g", "G", 0.5, between(0, 1)), num("b", "B", 0.5, between(0, 1)),
                  num("a", "A", 1, between(0, 1))}, color, "float4({r}, {g}, {b}, {a})", {.symbol = "swatchpalette"}),
        c.simple("to-number", "To Number", "Any value as a number: a vector's x, a colour's luminance.",
                 {num("value", "Value")}, number, "{value}"),
        c.simple("to-vector2", "To Vector 2", "Any value as a Vector 2.", {vec2("value", "Value", Fallback::zero())}, vector2, "{value}"),
        c.simple("to-vector3", "To Vector 3", "Any value as a Vector 3.", {vec3("value", "Value", Fallback::zero())}, vector3, "{value}"),
        c.simple("to-vector4", "To Vector 4", "Any value as a Vector 4.", {vec4("value", "Value", Fallback::zero())}, vector4, "{value}"),
        c.simple("to-color", "To Color", "Any value as a colour: a number as a grey.", {col("value", "Value")}, color, "{value}",
                 {.symbol = "paintpalette"}),
    }};
}

NodeGroup vectorMathNodes() {
    NodeBuilder c(NodeCategory::vectorMath);
    return {c.category, {
        c.simple("length", "Length", "How long a vector is.", {anyIn("vector", "Vector", 1)}, number, "lengthOfValue({vector})"),
        c.simple("distance", "Distance", "How far apart two points are.", {anyIn("a", "A", 0), anyIn("b", "B", 1)}, number,
                 "distanceBetweenValues({a}, {b})"),
        c.simple("normalize", "Normalize", "A vector scaled to length 1.", {anyIn("vector", "Vector", 1)}, any,
                 "normalizedValue({vector})"),
        c.simple("dot", "Dot Product", "The dot product of two vectors.", {anyIn("a", "A", 1), anyIn("b", "B", 1)}, number,
                 "dotProductOfValues({a}, {b})"),
        c.simple("cross", "Cross Product", "The vector at right angles to two 3D vectors.",
                 {vec3("a", "A", Fallback::up()), vec3("b", "B", Fallback::forward())}, vector3, "cross({a}, {b})"),
        c.simple("scale-vector", "Scale", "A vector times a number.", {anyIn("vector", "Vector", 1), num("scale", "Scale", 2)}, any,
                 "{vector} * {scale}", {.symbol = "arrow.up.left.and.arrow.down.right"}),
        c.simple("reflect", "Reflect", "A direction bounced off a surface with the given normal.",
                 {anyIn("incident", "Incident", 1), anyIn("normal", "Normal", 1)}, any,
                 "{incident} - 2.0f * dotProductOfValues({normal}, {incident}) * {normal}", {.symbol = "arrow.uturn.up"}),
        c.simple("refract", "Refract", "A direction bent through a surface by the ratio of refractive indices.",
                 {anyIn("incident", "Incident", 1), anyIn("normal", "Normal", 1), num("eta", "Ratio", 0.75, between(0, 2))}, any,
                 "refractedValue({incident}, {normal}, {eta})", {.symbol = "arrow.down.right"}),
        c.simple("project", "Project", "The part of one vector along another.", {anyIn("a", "Vector", 1), anyIn("onto", "Onto", 1)}, any,
                 "projectedValue({a}, {onto})", {.symbol = "arrow.down.to.line"}),
        c.simple("angle-between", "Angle Between", "The angle between two vectors, in radians.",
                 {anyIn("a", "A", 1), anyIn("b", "B", 1)}, number, "angleBetweenValues({a}, {b})", {.symbol = "angle"}),
        c.simple("perpendicular", "Perpendicular", "A 2D vector turned a quarter turn anticlockwise.",
                 {vec2("vector", "Vector")}, vector2, "float2(-{vector}.y, {vector}.x)"),
    }};
}

} // namespace editor::shaders
