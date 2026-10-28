// Kernels/ShaderMath.h on the GPU, for ShaderMath.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderMath.h"

using namespace editor::kernels;

/// Inputs per item: (x, y). Output: flooredModulo(x, y).
kernel void flooredModuloAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    outputs[item] = flooredModulo(inputs[item * 2 + 0], inputs[item * 2 + 1]);
}

/// Inputs per item: (a, b, smoothness). Output: smoothMinimum(a, b, smoothness).
kernel void smoothMinimumAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    outputs[item] = smoothMinimum(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]);
}

/// Inputs per item: (t). Output: bounceEaseOut(t).
kernel void bounceEaseOutAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    outputs[item] = bounceEaseOut(inputs[item]);
}

/// Inputs per item: (base, exponent). Output: safePower(base, exponent).
kernel void safePowerAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                               uint item [[thread_position_in_grid]]) {
    outputs[item] = safePower(inputs[item * 2 + 0], inputs[item * 2 + 1]);
}

/// Inputs per item: (t). Outputs: the three channels of heatmap(t).
kernel void heatmapAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                             uint item [[thread_position_in_grid]]) {
    Vector3Of<float> color = heatmap(inputs[item]);
    outputs[item * 3 + 0] = color.x;
    outputs[item * 3 + 1] = color.y;
    outputs[item * 3 + 2] = color.z;
}
