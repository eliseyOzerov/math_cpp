// Kernels/ShaderNoise.h: the GPU (ShaderNoise.metal) against the CPU, for Perlin and simplex noise,
// four octaves of ridged Perlin noise and cellular noise. The noises are continuous, so a point the
// GPU's floor puts one cell over still gives the same value; only rounding differs. Fast math lets
// the GPU fuse multiplies and add, reorder sums and use cheaper roots and divisions: a few units in
// the last place per step (about 6e-8 for values near 1), and the sums here have tens of steps
// (the 1.15 and 32 scale factors, 4 octaves, 4⁴ weights), so the tolerances are a few 1e-6 (measured differences: 6e-7 Perlin, 1.1e-6 simplex, 7e-7 fractal, 3e-7 cellular).
#include "../../Kernels/ShaderNoise.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector2Of;
using editor::kernels::Vector3Of;

namespace {
/// Points on a grid with irrational-looking spacing, so none sits on a lattice point or a cell border.
std::vector<float> space() {
    return everyTriple(evenlySpaced(-3.137f, 3.071f, 13), evenlySpaced(-2.953f, 4.219f, 13), evenlySpaced(-1.771f, 2.617f, 13));
}
} // namespace

AGREEMENT({
    "perlinNoiseAgreement", 3, 1, space(),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::perlinNoise3(Vector3Of<float>(inputs[0], inputs[1], inputs[2]));
    },
    3e-6f, 1e-6f,
});

AGREEMENT({
    // The sum is multiplied by 32 before clamping, so rounding is magnified more than in Perlin noise.
    "simplexNoiseAgreement", 3, 1, space(),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::simplexNoise3(Vector3Of<float>(inputs[0], inputs[1], inputs[2]));
    },
    6e-6f, 1e-6f,
});

AGREEMENT({
    "fractalNoiseAgreement", 3, 1, space(),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::fractalNoise(Vector3Of<float>(inputs[0], inputs[1], inputs[2]), 1, 1, 4.0f, 2.0f, 0.5f);
    },
    4e-6f, 1e-6f,
});

AGREEMENT({
    // Only the distances: the id jumps between cells where two sites are equally near.
    "cellularNoiseAgreement", 2, 2,
    everyPair(evenlySpaced(-4.137f, 4.071f, 41), evenlySpaced(-3.953f, 4.219f, 41)),
    [](const float* inputs, float* outputs) {
        editor::kernels::CellularResult result = editor::kernels::cellularNoise(Vector2Of<float>(inputs[0], inputs[1]), 1.0f, 0.5f, 0);
        outputs[0] = result.nearest;
        outputs[1] = result.second;
    },
    2e-6f, 1e-6f,
});
