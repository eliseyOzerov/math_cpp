// Kernels/ShaderPatterns.h: the GPU (ShaderPatterns.metal) against the CPU, for the checkerboard,
// hexagon tiles, the two spirals and the Bayer threshold. The masks are continuous, so only rounding
// differs: the checkerboard divides a difference of two nearly equal numbers by the footprint 0.05,
// which magnifies the rounding of positions near 6 (about 5e-7) by about 40, so its tolerance is a few
// 1e-5 (measured 4.8e-6); the hexagons (measured 8.3e-6) and spirals (6.3e-5, from the arc tangent and logarithm) are a smooth step of a distance with a few operations. The hexagon's
// random numbers are not compared: they jump between cells where two centres are equally near. The
// Bayer threshold is exact, so its positions are chosen away from cell borders (where the floor of
// the division could land one cell over) and it must agree to the last bit.
#include "../../Kernels/ShaderPatterns.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector2Of;
using editor::kernels::Vector4Of;

namespace {
/// Points with irrational-looking spacing so none sits on a cell border.
std::vector<float> plane() { return everyPair(evenlySpaced(-3.137f, 3.071f, 61), evenlySpaced(-2.953f, 3.219f, 61)); }
} // namespace

AGREEMENT({
    "checkerboardPatternAgreement", 2, 1, plane(),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::checkerboardPattern(Vector2Of<float>(inputs[0], inputs[1]), 0.0f, Vector2Of<float>(0.05f, 0.05f));
    },
    5e-5f, 1e-5f,
});

AGREEMENT({
    "hexagonTilesPatternAgreement", 2, 2, plane(),
    [](const float* inputs, float* outputs) {
        Vector4Of<float> tile = editor::kernels::hexagonTilesPattern(Vector2Of<float>(inputs[0], inputs[1]), 2.0f, 0.1f, 0.0f, 0.02f);
        outputs[0] = tile.x;
        outputs[1] = tile.y;
    },
    2e-5f, 1e-5f,
});

AGREEMENT({
    "spiralPatternAgreement", 2, 2, plane(),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> p(inputs[0], inputs[1]);
        outputs[0] = editor::kernels::spiralPattern(p, 3.0f, 2.0f, 0.1f, 0.3f, 0.0f, 1, 0.02f);
        outputs[1] = editor::kernels::spiralPattern(p, 3.0f, 2.0f, 0.1f, 0.3f, 0.0f, 0, 0.02f);
    },
    2e-4f, 1e-5f,
});

AGREEMENT({
    // Positions n + 0.37 (and 0.61) over 0…23, cells of 2 pixels: never near a cell border.
    "bayerMatrixThresholdAgreement", 2, 1,
    everyPair(evenlySpaced(0.37f, 23.37f, 24), evenlySpaced(0.61f, 23.61f, 24)),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::bayerMatrixThreshold(Vector2Of<float>(inputs[0], inputs[1]), 2.0f, 3);
    },
    0.0f, 0.0f,
});
