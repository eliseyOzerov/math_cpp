// Kernels/ShaderNoise.h on the GPU, for ShaderNoise.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderNoise.h"

using namespace editor::kernels;

/// Inputs per item: (x, y, z). Output: perlinNoise3(x, y, z).
kernel void perlinNoiseAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                 uint item [[thread_position_in_grid]]) {
    outputs[item] = perlinNoise3(Vector3Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]));
}

/// Inputs per item: (x, y, z). Output: simplexNoise3(x, y, z).
kernel void simplexNoiseAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                  uint item [[thread_position_in_grid]]) {
    outputs[item] = simplexNoise3(Vector3Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]));
}

/// Inputs per item: (x, y, z). Output: ridged Perlin fractalNoise with four octaves.
kernel void fractalNoiseAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                  uint item [[thread_position_in_grid]]) {
    outputs[item] = fractalNoise(Vector3Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]), 1, 1,
                                 4.0f, 2.0f, 0.5f);
}

/// Inputs per item: (x, y). Outputs: nearest and second distance of cellularNoise with jitter 1, evolution 0.5.
kernel void cellularNoiseAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    CellularResult result = cellularNoise(Vector2Of<float>(inputs[item * 2 + 0], inputs[item * 2 + 1]), 1.0f, 0.5f, 0);
    outputs[item * 2 + 0] = result.nearest;
    outputs[item * 2 + 1] = result.second;
}
