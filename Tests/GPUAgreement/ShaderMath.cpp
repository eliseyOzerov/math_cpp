// Kernels/ShaderMath.h: the GPU (ShaderMath.metal) against the CPU, for a sample of the library:
// a modulo, a smooth minimum, an easing curve, a guarded power and a colour ramp.
#include "../../Kernels/ShaderMath.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector3Of;

AGREEMENT({
    // Both signs of value and divisor; the divisors avoid zero. The values stay off exact multiples
    // of the divisors: there the result jumps from almost y to 0, and the GPU's fast division may
    // land x / y one step to the other side of a whole number.
    "flooredModuloAgreement", 2, 1,
    everyPair(evenlySpaced(-10.1234f, 9.8766f, 81), evenlySpaced(-3.3f, 3.3f, 12)),
    [](const float* inputs, float* outputs) { outputs[0] = editor::kernels::flooredModulo(inputs[0], inputs[1]); },
    1e-6f, 1e-6f,
});

AGREEMENT({
    "smoothMinimumAgreement", 3, 1,
    everyTriple(evenlySpaced(-2.0f, 2.0f, 21), evenlySpaced(-2.0f, 2.0f, 21), evenlySpaced(0.0f, 1.0f, 6)),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::smoothMinimum(inputs[0], inputs[1], inputs[2]);
    },
    1e-6f, 1e-6f,
});

AGREEMENT({
    "bounceEaseOutAgreement", 1, 1,
    evenlySpaced(0.0f, 1.0f, 1001),
    [](const float* inputs, float* outputs) { outputs[0] = editor::kernels::bounceEaseOut(inputs[0]); },
    1e-6f, 1e-6f,
});

AGREEMENT({
    // Negative bases and a zero base check the guards.
    "safePowerAgreement", 2, 1,
    everyPair(evenlySpaced(-2.0f, 4.0f, 25), evenlySpaced(-2.0f, 3.0f, 21)),
    [](const float* inputs, float* outputs) { outputs[0] = editor::kernels::safePower(inputs[0], inputs[1]); },
    1e-6f, 1e-5f,
});

AGREEMENT({
    "heatmapAgreement", 1, 3,
    evenlySpaced(-0.1f, 1.1f, 241),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> color = editor::kernels::heatmap(inputs[0]);
        outputs[0] = color.x;
        outputs[1] = color.y;
        outputs[2] = color.z;
    },
    1e-6f, 1e-6f,
});
