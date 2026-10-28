// Kernels/BlendModes.h on the GPU, for BlendModes.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/BlendModes.h"

using namespace editor::kernels;

/// Inputs per item: (base, top, mode). Output: blendedChannel(base, top, mode).
kernel void blendedChannelAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                    uint item [[thread_position_in_grid]]) {
    outputs[item] = blendedChannel(inputs[item * 3 + 0], inputs[item * 3 + 1], int(inputs[item * 3 + 2]));
}

/// Inputs per item: (base rgb, top rgb, mode). Outputs: the three channels of blendedColor.
kernel void blendedColorAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                  uint item [[thread_position_in_grid]]) {
    device const float* in = inputs + item * 7;
    Vector3Of<float> color = blendedColor(Vector3Of<float>(in[0], in[1], in[2]), Vector3Of<float>(in[3], in[4], in[5]), int(in[6]));
    outputs[item * 3 + 0] = color.x;
    outputs[item * 3 + 1] = color.y;
    outputs[item * 3 + 2] = color.z;
}

/// Inputs per item: (base rgba, top rgba, mode, opacity). Outputs: compositedWithBlend.
kernel void compositedWithBlendAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                         uint item [[thread_position_in_grid]]) {
    device const float* in = inputs + item * 10;
    Vector4Of<float> color = compositedWithBlend(Vector4Of<float>(in[0], in[1], in[2], in[3]),
        Vector4Of<float>(in[4], in[5], in[6], in[7]), int(in[8]), in[9]);
    outputs[item * 4 + 0] = color.x;
    outputs[item * 4 + 1] = color.y;
    outputs[item * 4 + 2] = color.z;
    outputs[item * 4 + 3] = color.w;
}
