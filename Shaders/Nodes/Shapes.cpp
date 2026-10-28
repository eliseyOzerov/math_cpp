// Two-dimensional signed distance fields and masks, and ways to combine them (graphics_metal's
// Nodes/Shapes.swift). A distance is negative inside a shape and in the canvas's units (one is
// its height); Fill and Stroke turn it into an antialiased mask. Helper names are the kernels'
// (Kernels/ShaderShapes.h, Kernels/ShaderDistances.h); graphics_metal's name is in each kernel's
// Tags line.
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

namespace {

/// One pixel in the canvas's units.
const char* const pixel = "(1.0f / max(u.resolution.y, 1.0f))";

/// A shape's distance, from coordinates (centred by default) and the shape's own inputs.
NodeDefinition shapeDistance(const NodeBuilder& c, const std::string& name, const std::string& label, const std::string& summary,
                             const std::string& symbol, std::vector<InputSpec> inputs, const std::string& templateText,
                             std::vector<NodeOption> options = {}) {
    inputs.insert(inputs.begin(), vec2("coordinates", "Coordinates", Fallback::centred()));
    return c.simple(name, label, summary, std::move(inputs), number, templateText,
                    {.symbol = symbol, .options = std::move(options), .output = {"distance", "Distance"}});
}

Fallback pointAt(const std::string& code) { return Fallback{code, "point", std::nullopt}; }

} // namespace

NodeGroup shapeNodes() {
    NodeBuilder c(NodeCategory::shapes);
    const std::vector<Fallback> polygonPoints = {
        pointAt("float2(0.0f, 0.3f)"), pointAt("float2(0.285f, 0.093f)"), pointAt("float2(0.176f, -0.243f)"),
        pointAt("float2(-0.176f, -0.243f)"), pointAt("float2(-0.285f, 0.093f)"), pointAt("float2(0.0f, 0.0f)")};
    return {c.category, {
        c.simple("regular-polygon-mask", "Regular Polygon", "An antialiased regular polygon mask.",
                 {vec2("coordinates", "Coordinates", Fallback::centred()), num("sides", "Sides", 6, between(3, 16)), num("radius", "Radius", 0.32),
                  num("rotation", "Rotation °", 0, between(0, 360)), num("feather", "Feather px", 1)},
                 number,
                 "polygonEdgeMask(regularPolygonDistance({coordinates}, {sides}, {radius}, degreesToRadians({rotation})), "
                 "1.0f / max(u.resolution.y, 1.0f), {feather} / max(u.resolution.y, 1.0f))",
                 {.symbol = "hexagon", .output = {"mask", "Mask"}}),
        shapeDistance(c, "circle-sdf", "Circle", "The distance to a circle.", "circle",
                      {num("radius", "Radius", 0.3)}, "circleDistance({coordinates}, {radius})"),
        shapeDistance(c, "rectangle-sdf", "Rectangle", "The distance to a rectangle.", "rectangle",
                      {num("width", "Width", 0.6), num("height", "Height", 0.4)},
                      "rectangleDistance({coordinates}, 0.5f * abs(float2({width}, {height})))"),
        shapeDistance(c, "rounded-rectangle-sdf", "Rounded Rectangle", "The distance to a rectangle with rounded corners.",
                      "rectangle.roundedtop",
                      {num("width", "Width", 0.6), num("height", "Height", 0.4), num("radius", "Corner Radius", 0.08)},
                      "roundedRectangleDistance({coordinates}, 0.5f * abs(float2({width}, {height})), {radius})"),
        c.simple("line-segment-sdf", "Line Segment", "The distance to a line segment (zero on the line; Stroke gives it width).",
                 {vec2("coordinates", "Coordinates", Fallback::centred()), num("ax", "Start X", -0.25), num("ay", "Start Y", -0.15),
                  num("bx", "End X", 0.25), num("by", "End Y", 0.15)},
                 number, "segmentDistance({coordinates}, float2({ax}, {ay}), float2({bx}, {by}))",
                 {.symbol = "line.diagonal", .output = {"distance", "Distance"}}),
        shapeDistance(c, "ellipse-sdf", "Ellipse", "The distance to an ellipse (an approximation away from its edge).", "oval",
                      {num("width", "Width", 0.7), num("height", "Height", 0.4)},
                      "ellipseDistance({coordinates}, 0.5f * float2({width}, {height}))"),
        shapeDistance(c, "triangle-sdf", "Triangle", "The distance to an equilateral triangle.", "triangle",
                      {num("radius", "Size", 0.2), num("rotation", "Rotation °", 0, between(0, 360))},
                      "triangleDistance(rotatedShapePoint({coordinates}, {rotation}), {radius})"),
        shapeDistance(c, "ring-sdf", "Ring", "The distance to a ring.", "circle.dashed",
                      {num("radius", "Radius", 0.3), num("thickness", "Thickness", 0.06)},
                      "ringDistance({coordinates}, {radius}, {thickness})"),
        shapeDistance(c, "arc-sdf", "Arc / Pie", "The distance to an arc, or to a pie slice, opening upwards.", "chart.pie",
                      {num("aperture", "Aperture °", 90, between(0, 180)), num("radius", "Radius", 0.3),
                       num("thickness", "Thickness", 0.06), num("rotation", "Rotation °", 0, between(0, 360))},
                      "arcDistance(rotatedShapePoint({coordinates}, {rotation}), {aperture}, {radius}, {thickness}, {#kind} == 1)",
                      {option("kind", "Kind", {"Arc", "Pie"})}),
        shapeDistance(c, "star-sdf", "Star", "The distance to a star.", "star",
                      {num("points", "Points", 5, between(3, 12)), num("radius", "Radius", 0.3),
                       num("pointiness", "Pointiness", 0.67, between(0, 1)), num("rotation", "Rotation °", 0, between(0, 360))},
                      "starDistance(rotatedShapePoint({coordinates}, {rotation}), {radius}, {points}, {pointiness})"),
        shapeDistance(c, "heart-sdf", "Heart", "The distance to a heart.", "heart",
                      {num("size", "Size", 0.3)}, "heartDistance({coordinates}, {size})"),
        shapeDistance(c, "cross-sdf", "Cross", "The distance to a plus sign.", "plus",
                      {num("size", "Size", 0.3), num("thickness", "Thickness", 0.12)},
                      "crossDistance({coordinates}, {size}, {thickness})"),
        shapeDistance(c, "capsule-sdf", "Capsule", "The distance to a capsule: a thick line with round ends.", "capsule",
                      {num("ax", "Start X", -0.2), num("ay", "Start Y", 0), num("bx", "End X", 0.2), num("by", "End Y", 0),
                       num("radius", "Radius", 0.1)},
                      "segmentDistance({coordinates}, float2({ax}, {ay}), float2({bx}, {by})) - {radius}"),
        shapeDistance(c, "polygon-sdf", "Polygon", "The distance to a polygon through three to six points.", "pentagon",
                      {vec2("p0", "Point 1", polygonPoints[0]), vec2("p1", "Point 2", polygonPoints[1]),
                       vec2("p2", "Point 3", polygonPoints[2]), vec2("p3", "Point 4", polygonPoints[3]),
                       vec2("p4", "Point 5", polygonPoints[4]), vec2("p5", "Point 6", polygonPoints[5]),
                       num("count", "Points Used", 5, between(3, 6))},
                      "polygonDistance({coordinates}, {p0}, {p1}, {p2}, {p3}, {p4}, {p5}, {count})"),
        shapeDistance(c, "bezier-sdf", "Quadratic Bézier", "The distance to a quadratic Bézier curve through three control points.",
                      "point.topleft.down.to.point.bottomright.curvepath",
                      {vec2("a", "Start", pointAt("float2(-0.25f, -0.15f)")), vec2("b", "Control", pointAt("float2(0.0f, 0.3f)")),
                       vec2("c", "End", pointAt("float2(0.25f, -0.15f)"))},
                      "bezierDistance({coordinates}, {a}, {b}, {c})"),
        shapeDistance(c, "moon-sdf", "Moon", "The distance to a crescent moon.", "moon",
                      {num("offset", "Offset", 0.12), num("outer", "Outer Radius", 0.3), num("inner", "Inner Radius", 0.25),
                       num("rotation", "Rotation °", 0, between(0, 360))},
                      "moonDistance(rotatedShapePoint({coordinates}, {rotation}), {offset}, {outer}, {inner})"),
        shapeDistance(c, "vesica-sdf", "Vesica", "The distance to a vesica: the lens where two circles overlap.", "oval.portrait",
                      {num("radius", "Radius", 0.4), num("spread", "Spread", 0.2), num("rotation", "Rotation °", 0, between(0, 360))},
                      "vesicaDistance(rotatedShapePoint({coordinates}, {rotation}), {radius}, {spread})"),
        shapeDistance(c, "egg-sdf", "Egg", "The distance to an egg.", "oval.portrait.fill",
                      {num("bottom", "Bottom Radius", 0.2), num("top", "Top Radius", 0.1)},
                      "eggDistance({coordinates}, {bottom}, {top})"),
    }};
}

NodeGroup sdfOperationNodes() {
    NodeBuilder c(NodeCategory::sdfOperations);
    const std::string pixelText = pixel;
    return {c.category, {
        c.simple("sdf-fill", "Fill", "Turns a distance into an antialiased mask: 1 inside, 0 outside.",
                 {num("distance", "Distance", 0), num("feather", "Feather px", 1)}, number,
                 "distanceFillMask({distance}, " + pixelText + ", {feather})", {.symbol = "circle.fill", .output = {"mask", "Mask"}}),
        c.simple("sdf-stroke", "Stroke", "Turns a distance into an outline mask of a given width.",
                 {num("distance", "Distance", 0), num("width", "Width", 0.02), num("feather", "Feather px", 1)}, number,
                 "distanceStrokeMask({distance}, {width}, {#alignment}, " + pixelText + ", {feather})",
                 {.symbol = "circle", .options = {option("alignment", "Alignment", {"Center", "Inside", "Outside"})},
                  .output = {"mask", "Mask"}}),
        c.simple("sdf-smooth-union", "Smooth Union", "Both shapes together, with a rounded join; a Smoothness of 0 joins them sharply.",
                 {num("a", "A", 0), num("b", "B", 0), num("smoothness", "Smoothness", 0.1)}, number,
                 "smoothUnion({a}, {b}, {smoothness})", {.symbol = "drop", .output = {"distance", "Distance"}}),
        c.simple("sdf-smooth-subtraction", "Smooth Subtraction", "A with B cut out, with a rounded cut; a Smoothness of 0 cuts it sharply.",
                 {num("a", "A", 0), num("b", "B", 0), num("smoothness", "Smoothness", 0.1)}, number,
                 "smoothSubtraction({a}, {b}, {smoothness})", {.symbol = "drop.halffull", .output = {"distance", "Distance"}}),
        c.simple("sdf-smooth-intersection", "Smooth Intersection", "Where both overlap, with rounded corners; a Smoothness of 0 keeps them sharp.",
                 {num("a", "A", 0), num("b", "B", 0), num("smoothness", "Smoothness", 0.1)}, number,
                 "smoothIntersection({a}, {b}, {smoothness})", {.symbol = "drop.degreesign", .output = {"distance", "Distance"}}),
        c.simple("sdf-round", "Round", "Inflates a shape and rounds its corners.",
                 {num("distance", "Distance", 0), num("radius", "Radius", 0.05)}, number, "({distance}) - {radius}",
                 {.symbol = "arrow.up.left.and.arrow.down.right", .output = {"distance", "Distance"}}),
        c.simple("sdf-onion", "Onion", "Turns a shape into a shell of the given thickness; chain it for rings.",
                 {num("distance", "Distance", 0), num("thickness", "Thickness", 0.03)}, number, "abs({distance}) - {thickness}",
                 {.symbol = "circle.hexagongrid", .output = {"distance", "Distance"}}),
        c.simple("sdf-glow", "Glow", "Light that fades exponentially away from the shape.",
                 {num("distance", "Distance", 0), num("radius", "Radius", 0.1), num("intensity", "Intensity", 1)}, number,
                 "distanceGlow({distance}, {radius}, {intensity})", {.symbol = "sparkle", .output = {"glow", "Glow"}}),
        c.simple("sdf-shadow", "Drop Shadow", "A soft shadow mask of the shape moved by an offset.",
                 {num("distance", "Distance", 0), num("offsetX", "Offset X", 0.02), num("offsetY", "Offset Y", -0.02),
                  num("blur", "Blur", 0.04), num("opacity", "Opacity", 0.6)}, number,
                 // The gradient is the distance's slope on screen, which is what only a fragment shader can measure.
                 "distanceShadow({distance}, float2(dfdx({distance}), dfdy({distance})), float2({offsetX}, {offsetY}), {blur}, {opacity}, "
                     + pixelText + ")",
                 {.symbol = "shadow", .output = {"shadow", "Shadow"}}),
        c.node({.name = "sdf-repeat", .label = "Repeat",
                .summary = "Repeats coordinates in a grid, infinitely or a limited number of cells; feed them to a shape.",
                .symbol = "square.grid.3x3",
                .inputs = {vec2("coordinates", "Coordinates", Fallback::centred()), num("spacing", "Spacing", 0.3),
                           num("limit", "Limit", 0, between(0, 16))},
                .outputs = {out("coordinates", "Coordinates", vector2, "repeatedCell2({coordinates}, {spacing}, {limit}).xy"),
                            out("cell", "Cell", vector2, "repeatedCell2({coordinates}, {spacing}, {limit}).zw")}}),
        c.simple("sdf-displace", "Displace", "Pushes a shape's edge in and out by a value, such as noise.",
                 {num("distance", "Distance", 0), num("displacement", "Displacement", 0), num("amount", "Amount", 0.05)}, number,
                 "({distance}) + {amount} * ({displacement})",
                 {.symbol = "waveform.path", .output = {"distance", "Distance"}}),
    }};
}

} // namespace editor::shaders
