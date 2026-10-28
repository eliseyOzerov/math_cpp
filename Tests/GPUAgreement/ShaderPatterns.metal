// Kernels/ShaderPatterns.h on the GPU, for ShaderPatterns.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderPatterns.h"

using namespace editor::kernels;

/// Inputs per item: (x, y). Output: checkerboardPattern with softness 0 and a footprint of 0.05 on both axes.
kernel void checkerboardPatternAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                         uint item [[thread_position_in_grid]]) {
    outputs[item] = checkerboardPattern(Vector2Of<float>(inputs[item * 2 + 0], inputs[item * 2 + 1]), 0.0f,
                                        Vector2Of<float>(0.05f, 0.05f));
}

/// Inputs per item: (x, y). Outputs: the mask and the edge distance of hexagonTilesPattern, scale 2, gap 0.1.
kernel void hexagonTilesPatternAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                         uint item [[thread_position_in_grid]]) {
    Vector4Of<float> tile = hexagonTilesPattern(Vector2Of<float>(inputs[item * 2 + 0], inputs[item * 2 + 1]), 2.0f, 0.1f, 0.0f, 0.02f);
    outputs[item * 2 + 0] = tile.x;
    outputs[item * 2 + 1] = tile.y;
}

/// Inputs per item: (x, y). Outputs: the logarithmic and the straight spiralPattern, three arms.
kernel void spiralPatternAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    Vector2Of<float> p(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    outputs[item * 2 + 0] = spiralPattern(p, 3.0f, 2.0f, 0.1f, 0.3f, 0.0f, 1, 0.02f);
    outputs[item * 2 + 1] = spiralPattern(p, 3.0f, 2.0f, 0.1f, 0.3f, 0.0f, 0, 0.02f);
}

/// Inputs per item: (x, y). Output: bayerMatrixThreshold of three bits and cell size 2 at (x, y).
kernel void bayerMatrixThresholdAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                          uint item [[thread_position_in_grid]]) {
    outputs[item] = bayerMatrixThreshold(Vector2Of<float>(inputs[item * 2 + 0], inputs[item * 2 + 1]), 2.0f, 3);
}
