// Kernels/ShaderSimulation.h: the GPU (ShaderSimulation.metal) against the CPU, for the Gray–Scott
// reaction, the two start states, the Life rule's next state, the fluid's falloff and next state and the
// particle field in two motions. The reaction, life and fluid cases are a few multiplications and
// additions, so only rounding differs. The start states are a hash of a cell (exact) behind a floor, so their
// inputs are chosen away from cell borders. The particle field adds up to twelve smooth steps whose
// positions come from the sine and cosine of fast-math arguments.
#include "../../Kernels/ShaderSimulation.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector2Of;
using editor::kernels::Vector4Of;

namespace {
/// Points with irrational-looking spacing so none sits on a cell border.
std::vector<float> plane(int count) { return everyPair(evenlySpaced(0.0113f, 0.9871f, count), evenlySpaced(0.0197f, 0.9923f, count)); }

/// Every (A, B, laplacian A, laplacian B) on a small grid.
std::vector<float> reactionInputs() {
    std::vector<float> values;
    for (float a : evenlySpaced(0.0f, 1.0f, 9)) {
        for (float b : evenlySpaced(0.0f, 1.0f, 9)) {
            for (float lapA : evenlySpaced(-0.8f, 0.8f, 5)) {
                for (float lapB : evenlySpaced(-0.6f, 0.6f, 5)) {
                    values.push_back(a);
                    values.push_back(b);
                    values.push_back(lapA);
                    values.push_back(lapB);
                }
            }
        }
    }
    return values;
}

/// Every (red 0 or 1, trail, neighbours 0…8).
std::vector<float> lifeInputs() {
    std::vector<float> values;
    for (float red : {0.0f, 1.0f}) {
        for (float trail : evenlySpaced(0.0f, 1.0f, 5)) {
            for (int neighbours = 0; neighbours <= 8; neighbours += 1) {
                values.push_back(red);
                values.push_back(trail);
                values.push_back(static_cast<float>(neighbours));
            }
        }
    }
    return values;
}
} // namespace

AGREEMENT({
    "grayScottReactionAgreement", 4, 2, reactionInputs(),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> next = editor::kernels::grayScottReaction(Vector2Of<float>(inputs[0], inputs[1]), Vector2Of<float>(inputs[2], inputs[3]),
                                                                    0.055f, 0.062f, 1.0f, 0.5f, 1.5f);
        outputs[0] = next.x;
        outputs[1] = next.y;
    },
    1e-6f, 1e-6f,
});

AGREEMENT({
    "startStatesAgreement", 2, 2, plane(31),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> uv(inputs[0], inputs[1]);
        outputs[0] = editor::kernels::grayScottStartState(uv, Vector2Of<float>(1.0f / 1024.0f)).y;
        outputs[1] = editor::kernels::lifeStartColor(uv, Vector2Of<float>(1.0f / 64.0f), 0.3f).x;
    },
    0.0f, 0.0f,
});

AGREEMENT({
    "lifeNextStateAgreement", 3, 3, lifeInputs(),
    [](const float* inputs, float* outputs) {
        Vector4Of<float> next = editor::kernels::lifeNextState(Vector4Of<float>(inputs[0], 0.0f, inputs[1], 1.0f), static_cast<int>(inputs[2]), 0.8f);
        outputs[0] = next.x;
        outputs[1] = next.y;
        outputs[2] = next.z;
    },
    1e-6f, 1e-6f,
});

AGREEMENT({
    "fluidAgreement", 2, 3, plane(41),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> uv(inputs[0], inputs[1]);
        float falloff = editor::kernels::fluidFalloff(uv, Vector2Of<float>(0.5f), Vector2Of<float>(4.0f / 3.0f, 1.0f), 0.2f);
        Vector4Of<float> next = editor::kernels::fluidNextState(Vector2Of<float>(uv.x - 0.5f, 0.2f), uv.y, Vector2Of<float>(0.3f, 0.4f), 0.8f, falloff, 0.97f);
        outputs[0] = falloff;
        outputs[1] = next.x;
        outputs[2] = next.z;
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    "particleFieldAgreement", 2, 8, plane(25),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> uv(inputs[0], inputs[1]);
        Vector4Of<float> rising = editor::kernels::particleField(uv, Vector2Of<float>(1.5f, 1.0f), 12.0f, 0.15f, 1.0f, 2.3f, 4.0f, 1, 1.0f);
        Vector4Of<float> orbiting = editor::kernels::particleField(uv, Vector2Of<float>(1.5f, 1.0f), 12.0f, 0.15f, 1.0f, 2.3f, 4.0f, 2, 1.0f);
        outputs[0] = rising.x;
        outputs[1] = rising.y;
        outputs[2] = rising.z;
        outputs[3] = rising.w;
        outputs[4] = orbiting.x;
        outputs[5] = orbiting.y;
        outputs[6] = orbiting.z;
        outputs[7] = orbiting.w;
    },
    5e-4f, 1e-4f,
});
