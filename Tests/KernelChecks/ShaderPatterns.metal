// Compiles Kernels/ShaderPatterns.h as Metal Shading Language (`make kernels`). Every function is called here.
#include <metal_stdlib>

#include "../../Kernels/ShaderPatterns.h"

using namespace editor::kernels;

kernel void shaderPatternsCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f - 0.5f;
    Vector2Of<float> p(x, 1.3f);
    Vector2Of<float> pixels(0.01f, 0.02f);
    float pixel = 0.01f;
    int variant = int(index % 3);

    Vector2Of<float> counts(4.0f, 3.0f);
    results[index * 16 + 0] = antialiasedCoverage(x, pixel, 0.0f) + checkerboardPattern(p, 0.0f, pixels)
        + stripeCoordinate(p, 5.0f, 0.3f);
    Vector2Of<float> stripes = stripesPattern(p, 5.0f, 0.3f, 0.5f, 0.0f, pixel);
    Vector2Of<float> grid = gridPattern(p, counts, 0.1f, 0.0f, pixels);
    Vector2Of<float> dots = dotsPattern(p, 4.0f, 0.3f, 0.0f, index % 2 == 0, pixel);
    results[index * 16 + 1] = stripes.x + stripes.y + grid.x + grid.y + dots.x + dots.y;
    Vector2Of<float> bricks = bricksPattern(p, counts, 0.5f, 0.1f, 0.0f, pixels);
    Vector4Of<float> hexagons = hexagonTilesPattern(p, 3.0f, 0.1f, 0.0f, pixel);
    results[index * 16 + 2] = bricks.x + bricks.y + hexagons.x + hexagons.y + hexagons.z + hexagons.w;
    results[index * 16 + 3] = truchetPattern(p, 3.0f, 0.2f, 0.0f, variant, pixel) + ripplesPattern(p, 4.0f, 0.5f, 1.0f, variant)
        + spiralPattern(p, 3.0f, 2.0f, 0.1f, 0.3f, 0.0f, variant % 2, pixel) + sunburstPattern(p, 8.0f, 0.2f, 0.4f, 0.0f, pixel);
    results[index * 16 + 4] = zigzagPhase(p, 3.0f, 4.0f, 0.5f, variant % 2)
        + zigzagPattern(p, 3.0f, 4.0f, 0.5f, 0.3f, 0.0f, variant % 2, pixel);
    Vector2Of<float> herringbone = herringbonePattern(p, 4.0f, 4.0f, 0.5f, 0.1f, 0.0f, pixels);
    Vector2Of<float> basket = basketweavePattern(p, 3.0f, 2.0f, 0.1f, 0.0f, pixel);
    results[index * 16 + 5] = herringboneBand(p, 4.0f, 4.0f, 0.5f) + herringbone.x + herringbone.y + basket.x + basket.y;
    Vector2Of<float> screen = halftoneCoordinate(p, 6.0f, 0.4f);
    results[index * 16 + 6] = moirePattern(p, 5.0f, 0.1f, 0.4f, int(index % 2)) + screen.x + halftonePattern(p, 0.5f, 6.0f, 0.4f, variant, pixel);
    results[index * 16 + 7] = bayerMatrixThreshold(p * 8.0f, 2.0f, 3) + ditheredLevels(0.4f, 4.0f, 0.5f) + gradientRamp(p, Vector2Of<float>(0.5f), 0.2f, 1.0f, int(index % 5));
}
