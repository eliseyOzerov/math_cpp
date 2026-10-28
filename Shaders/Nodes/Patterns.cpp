// Patterns: repeating masks, 1 where the pattern is drawn (graphics_metal's Nodes/Patterns.swift).
// Lines and edges are antialiased to the pixel; `softness` blurs them on purpose. Coordinates
// default to the canvas UV (or the centred UV for patterns that radiate from a point).
//
// The kernels (Kernels/ShaderPatterns.h) take the pixel footprint as `pixelSize` instead of
// calling `fwidth`, which exists only in a fragment shader. These templates are fragment code, so
// they pass the `fwidth` of the coordinate each kernel's comment names. Helper names are the
// kernels'; graphics_metal's name is in each kernel's Tags line.
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

NodeGroup patternNodes() {
    NodeBuilder c(NodeCategory::patterns);
    const InputSpec softness = num("softness", "Softness", 0, between(0, 0.5));
    return {c.category, {
        c.simple("checker", "Checker", "A checkerboard: 1 on one colour of squares, 0 on the other.",
                 {vec2("coordinates", "Coordinates"), num("scale", "Scale", 8, between(1, 64)), softness}, number,
                 "checkerboardPattern({coordinates} * {scale}, {softness}, fwidth({coordinates} * {scale}))",
                 {.symbol = "checkerboard.rectangle"}),
        c.node({.name = "stripes", .label = "Stripes", .summary = "Parallel stripes at an angle: the mask and each stripe's number.",
                .symbol = "line.3.horizontal",
                .inputs = {vec2("coordinates", "Coordinates"), num("count", "Count", 8, between(1, 64)), num("angle", "Angle", 0, between(-180, 180)),
                           num("width", "Width", 0.5, between(0, 1)), softness},
                .outputs = {out("value", "Value", number, "{$}.x"), out("index", "Index", number, "{$}.y")},
                .setup = "stripesPattern({coordinates}, {count}, degreesToRadians({angle}), {width}, {softness}, "
                         "fwidth(stripeCoordinate({coordinates}, {count}, degreesToRadians({angle}))))"}),
        c.node({.name = "grid", .label = "Grid",
                .summary = "Lines between cells, with widths that stay antialiased: the lines and each cell's random number.",
                .symbol = "grid",
                .inputs = {vec2("coordinates", "Coordinates"), num("columns", "Columns", 8, between(1, 64)), num("rows", "Rows", 8, between(1, 64)),
                           num("width", "Line Width", 0.08, between(0, 1)), softness},
                .outputs = {out("lines", "Lines", number, "{$}.x"), out("cells", "Cells", number, "(1.0f - {$}.x)"),
                            out("random", "Random", number, "{$}.y")},
                .setup = "gridPattern({coordinates}, float2({columns}, {rows}), {width}, {softness}, fwidth({coordinates}))"}),
        c.node({.name = "dots", .label = "Dots", .summary = "Polka dots on a grid, optionally staggered: the mask and each dot's random number.",
                .symbol = "circle.grid.3x3",
                .inputs = {vec2("coordinates", "Coordinates"), num("scale", "Scale", 8, between(1, 64)),
                           num("radius", "Radius", 0.35, between(0, 0.75)), softness},
                .outputs = {out("value", "Value", number, "{$}.x"), out("random", "Random", number, "{$}.y")},
                .options = {option("layout", "Layout", {"Grid", "Staggered"})},
                .setup = "dotsPattern({coordinates}, {scale}, {radius}, {softness}, {#layout} == 1, length(fwidth({coordinates})))"}),
        c.node({.name = "bricks", .label = "Bricks",
                .summary = "Rows of bricks with mortar between: the bricks, the mortar and each brick's random number.",
                .symbol = "rectangle.split.3x3",
                .inputs = {vec2("coordinates", "Coordinates"), num("columns", "Columns", 4, between(1, 32)), num("rows", "Rows", 8, between(1, 64)),
                           num("offset", "Row Offset", 0.5, between(0, 1)), num("mortar", "Mortar", 0.1, between(0, 1)), softness},
                .outputs = {out("bricks", "Bricks", number, "{$}.x"), out("mortar", "Mortar", number, "(1.0f - {$}.x)"),
                            out("random", "Random", number, "{$}.y")},
                .setup = "bricksPattern({coordinates}, float2({columns}, {rows}), {offset}, {mortar}, {softness}, fwidth({coordinates}))"}),
        c.node({.name = "hex-tiles", .label = "Hex Tiles",
                .summary = "Hexagonal tiles with gaps: the mask, the distance to each tile's edge and a random number per tile.",
                .symbol = "hexagon",
                .inputs = {vec2("coordinates", "Coordinates"), num("scale", "Scale", 6, between(0.5, 40)), num("gap", "Gap", 0.08, between(0, 0.5)),
                           softness},
                .outputs = {out("value", "Value", number, "{$}.x"), out("edge", "Edge Distance", number, "{$}.y"),
                            out("random", "Random", number, "{$}.z")},
                .setup = "hexagonTilesPattern({coordinates}, {scale}, {gap}, {softness}, length(fwidth({coordinates})))"}),
        c.simple("truchet", "Truchet Tiles", "Random tiles whose arcs or diagonals join into winding paths.",
                 {vec2("coordinates", "Coordinates"), num("scale", "Scale", 8, between(1, 48)), num("width", "Width", 0.2, between(0, 1)), softness},
                 number, "truchetPattern({coordinates}, {scale}, {width}, {softness}, {#variant}, length(fwidth({coordinates})))",
                 {.symbol = "point.topleft.down.to.point.bottomright.curvepath", .options = {option("variant", "Tile", {"Arcs", "Diagonals"})}}),
        c.simple("ripples", "Waves / Ripples", "Concentric rings spreading from the centre.",
                 {vec2("coordinates", "Coordinates", Fallback::centred()), num("frequency", "Frequency", 12, between(0, 80)), timeIn("time", "Time"),
                  num("speed", "Speed", 0.5, between(-4, 4)), num("decay", "Decay", 0, between(0, 10))},
                 number, "ripplesPattern({coordinates}, {frequency}, {time} * {speed}, {decay}, {#shape})",
                 {.symbol = "circle.dotted.circle", .options = {option("shape", "Wave", {"Sine", "Triangle", "Square", "Sawtooth"})}}),
        c.simple("spiral", "Spiral", "Spiral arms winding out from the centre.",
                 {vec2("coordinates", "Coordinates", Fallback::centred()), num("arms", "Arms", 2, between(1, 12)),
                  num("frequency", "Tightness", 6, between(0.5, 40)), timeIn("time", "Time"), num("speed", "Spin", 0.2, between(-4, 4)),
                  num("width", "Width", 0.5, between(0, 1)), softness},
                 number,
                 "spiralPattern({coordinates}, {arms}, {frequency}, {time} * {speed}, {width}, {softness}, {#form}, length(fwidth({coordinates})))",
                 {.symbol = "tornado", .options = {option("form", "Spiral", {"Archimedean", "Logarithmic"})}}),
        c.simple("sunburst", "Sunburst / Rays", "Wedges radiating from the centre.",
                 {vec2("coordinates", "Coordinates", Fallback::centred()), num("rays", "Rays", 12, between(1, 64)),
                  num("rotation", "Rotation", 0, between(-360, 360)), num("width", "Width", 0.5, between(0, 1)), softness},
                 number,
                 "sunburstPattern({coordinates}, {rays}, degreesToRadians({rotation}), {width}, {softness}, length(fwidth({coordinates})))",
                 {.symbol = "sun.max"}),
        c.simple("zigzag", "Zigzag / Chevron", "Stripes bent into zigzags (chevrons) or waves.",
                 {vec2("coordinates", "Coordinates"), num("frequency", "Frequency", 8, between(1, 48)), num("rows", "Rows", 8, between(1, 48)),
                  num("amplitude", "Amplitude", 1, between(0, 4)), num("width", "Width", 0.5, between(0, 1)), softness},
                 number,
                 "zigzagPattern({coordinates}, {frequency}, {rows}, {amplitude}, {width}, {softness}, {#shape}, "
                 "fwidth(zigzagPhase({coordinates}, {frequency}, {rows}, {amplitude}, {#shape})))",
                 {.symbol = "chevron.down.2", .options = {option("shape", "Bend", {"Zigzag", "Wave"})}}),
        c.node({.name = "herringbone", .label = "Herringbone", .summary = "Slanted bricks that lean one way then the other, column by column.",
                .symbol = "chevron.right.2",
                .inputs = {vec2("coordinates", "Coordinates"), num("columns", "Columns", 8, between(1, 48)), num("rows", "Rows", 8, between(1, 48)),
                           num("slant", "Slant", 1, between(0, 3)), num("mortar", "Mortar", 0.1, between(0, 1)), softness},
                .outputs = {out("bricks", "Bricks", number, "{$}.x"), out("random", "Random", number, "{$}.y")},
                .setup = "herringbonePattern({coordinates}, {columns}, {rows}, {slant}, {mortar}, {softness}, "
                         "float2(fwidth({coordinates}.x * {columns}), fwidth(herringboneBand({coordinates}, {columns}, {rows}, {slant}))))"}),
        c.node({.name = "basketweave", .label = "Basketweave",
                .summary = "Bars woven over and under in alternating squares: the mask and a shading that rounds each bar.",
                .symbol = "square.grid.2x2",
                .inputs = {vec2("coordinates", "Coordinates"), num("scale", "Scale", 6, between(1, 32)), num("bars", "Bars", 3, between(1, 6)),
                           num("gap", "Gap", 0.08, between(0, 0.5)), softness},
                .outputs = {out("value", "Value", number, "{$}.x"), out("shade", "Shade", number, "{$}.y")},
                .setup = "basketweavePattern({coordinates}, {scale}, {bars}, {gap}, {softness}, length(fwidth({coordinates})))"}),
        c.simple("moire", "Moiré", "Two fine gratings laid over each other, which beat into broad bands.",
                 {vec2("coordinates", "Coordinates", Fallback::centred()), num("frequency", "Frequency", 40, between(2, 200)),
                  num("angle", "Angle", 4, between(-45, 45)), num("spread", "Spread", 0.15, between(0, 1))},
                 number, "moirePattern({coordinates}, {frequency}, degreesToRadians({angle}), {spread}, {#form})",
                 {.symbol = "circle.dotted.circle", .options = {option("form", "Gratings", {"Lines", "Rings"})}}),
        c.simple("halftone", "Halftone", "Printer's dots covering as much of each cell as the value says.",
                 {vec2("coordinates", "Coordinates"), num("value", "Value", 0.5, between(0, 1)), num("scale", "Scale", 32, between(2, 128)),
                  num("angle", "Angle", 45, between(-90, 90))},
                 number,
                 "halftonePattern({coordinates}, {value}, {scale}, degreesToRadians({angle}), {#shape}, "
                 "length(fwidth(halftoneCoordinate({coordinates}, {scale}, degreesToRadians({angle})))))",
                 {.symbol = "circle.dotted", .options = {option("shape", "Shape", {"Dots", "Squares", "Lines"})}}),
        c.node({.name = "dither", .label = "Dither",
                .summary = "Ordered dithering with a Bayer matrix: the value cut to a few levels, and the matrix's threshold itself.",
                .symbol = "square.grid.4x3.fill",
                .inputs = {num("value", "Value", 0.5, between(0, 1)), num("levels", "Levels", 2, between(2, 16)),
                           num("size", "Pixel Size", 1, between(1, 16))},
                .outputs = {out("value", "Value", number, "ditheredLevels({value}, {levels}, {$})"),
                            out("threshold", "Threshold", number, "{$}")},
                .options = {option("matrix", "Matrix", {"2 × 2", "4 × 4", "8 × 8"}, 1)},
                .setup = "bayerMatrixThreshold(fragCoord, {size}, {#matrix} + 1)"}),
        c.simple("gradient", "Gradient", "A 0 to 1 ramp in a shape: linear, radial, angular, diamond or square.",
                 {vec2("coordinates", "Coordinates"), vec2("center", "Centre", Fallback::middle()), num("angle", "Angle", 0, between(-180, 180)),
                  num("size", "Size", 1, between(0.01, 4))},
                 number, "gradientRamp({coordinates}, {center}, degreesToRadians({angle}), {size}, {#shape})",
                 {.symbol = "circle.lefthalf.filled",
                  .options = {option("shape", "Shape", {"Linear", "Radial", "Angular", "Diamond", "Square"})}}),
    }};
}

} // namespace editor::shaders
