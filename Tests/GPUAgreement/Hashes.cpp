// Kernels/Hashes.h: the GPU (Hashes.metal) against the CPU. The integer mixing is exact on both
// sides, so only the final conversion to 0…1 may differ, by at most one step.
#include "../../Kernels/Hashes.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector3Of;

AGREEMENT({
    // Negative cells, fractions within a cell, and cells far from the origin.
    "hash11Agreement", 1, 1,
    evenlySpaced(-1000.0f, 1000.0f, 4001),
    [](const float* inputs, float* outputs) { outputs[0] = editor::kernels::hash11(inputs[0]); },
    1e-7f, 0.0f,
});

AGREEMENT({
    "hash33Agreement", 3, 3,
    everyTriple(evenlySpaced(-8.0f, 8.0f, 17), evenlySpaced(-8.0f, 8.0f, 17), evenlySpaced(-2.5f, 2.5f, 11)),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> hash = editor::kernels::hash33(Vector3Of<float>(inputs[0], inputs[1], inputs[2]));
        outputs[0] = hash.x;
        outputs[1] = hash.y;
        outputs[2] = hash.z;
    },
    1e-7f, 0.0f,
});
