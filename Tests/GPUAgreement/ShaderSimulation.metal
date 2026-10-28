// Kernels/ShaderSimulation.h on the GPU, for ShaderSimulation.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderSimulation.h"

using namespace editor::kernels;

/// Inputs per item: (A, B, laplacian A, laplacian B). Outputs: (A, B) after grayScottReaction with feed 0.055, kill 0.062,
/// diffusion (1, 0.5) and rate 1.5.
kernel void grayScottReactionAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                       uint item [[thread_position_in_grid]]) {
    Vector2Of<float> next = grayScottReaction(Vector2Of<float>(inputs[item * 4 + 0], inputs[item * 4 + 1]),
                                              Vector2Of<float>(inputs[item * 4 + 2], inputs[item * 4 + 3]), 0.055f, 0.062f, 1.0f, 0.5f, 1.5f);
    outputs[item * 2 + 0] = next.x;
    outputs[item * 2 + 1] = next.y;
}

/// Inputs per item: (x, y), a uv. Outputs: B of grayScottStartState with texel 1/1024 and the red of lifeStartColor with
/// 1/64 cells and density 0.3.
kernel void startStatesAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                 uint item [[thread_position_in_grid]]) {
    Vector2Of<float> uv(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    outputs[item * 2 + 0] = grayScottStartState(uv, Vector2Of<float>(1.0f / 1024.0f)).y;
    outputs[item * 2 + 1] = lifeStartColor(uv, Vector2Of<float>(1.0f / 64.0f), 0.3f).x;
}

/// Inputs per item: (red, trail, neighbours). Outputs: lifeNextState with fade 0.8 (red, green, blue).
kernel void lifeNextStateAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    Vector4Of<float> next = lifeNextState(Vector4Of<float>(inputs[item * 3 + 0], 0.0f, inputs[item * 3 + 1], 1.0f),
                                          static_cast<int>(inputs[item * 3 + 2]), 0.8f);
    outputs[item * 3 + 0] = next.x;
    outputs[item * 3 + 1] = next.y;
    outputs[item * 3 + 2] = next.z;
}

/// Inputs per item: (x, y), a uv. Outputs: fluidFalloff at the middle with aspect (4/3, 1) and radius 0.2, then the dye and
/// first velocity channel of fluidNextState from moving (x − 0.5, 0.2), moved dye y, push (0.3, 0.4).
kernel void fluidAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                           uint item [[thread_position_in_grid]]) {
    Vector2Of<float> uv(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    float falloff = fluidFalloff(uv, Vector2Of<float>(0.5f), Vector2Of<float>(4.0f / 3.0f, 1.0f), 0.2f);
    Vector4Of<float> next = fluidNextState(Vector2Of<float>(uv.x - 0.5f, 0.2f), uv.y, Vector2Of<float>(0.3f, 0.4f), 0.8f, falloff, 0.97f);
    outputs[item * 3 + 0] = falloff;
    outputs[item * 3 + 1] = next.x;
    outputs[item * 3 + 2] = next.z;
}

/// Inputs per item: (x, y), a uv. Outputs: particleField (red, green, blue, density) for 12 particles of size 0.15 at time 2.3,
/// seed 4, spread 1 and motion 1 (rising), then for motion 2 (orbiting).
kernel void particleFieldAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    Vector2Of<float> uv(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    Vector4Of<float> rising = particleField(uv, Vector2Of<float>(1.5f, 1.0f), 12.0f, 0.15f, 1.0f, 2.3f, 4.0f, 1, 1.0f);
    Vector4Of<float> orbiting = particleField(uv, Vector2Of<float>(1.5f, 1.0f), 12.0f, 0.15f, 1.0f, 2.3f, 4.0f, 2, 1.0f);
    outputs[item * 8 + 0] = rising.x;
    outputs[item * 8 + 1] = rising.y;
    outputs[item * 8 + 2] = rising.z;
    outputs[item * 8 + 3] = rising.w;
    outputs[item * 8 + 4] = orbiting.x;
    outputs[item * 8 + 5] = orbiting.y;
    outputs[item * 8 + 6] = orbiting.z;
    outputs[item * 8 + 7] = orbiting.w;
}
