// Kernels/ShaderColor.h on the GPU, for ShaderColor.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderColor.h"

using namespace editor::kernels;

/// Inputs per item: (red, green, blue). Outputs: hsvFromRGB.
kernel void hsvFromRGBAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                uint item [[thread_position_in_grid]]) {
    Vector3Of<float> hsv = hsvFromRGB(Vector3Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]));
    outputs[item * 3 + 0] = hsv.x;
    outputs[item * 3 + 1] = hsv.y;
    outputs[item * 3 + 2] = hsv.z;
}

/// Inputs per item: (hue, saturation, value). Outputs: rgbFromHSV.
kernel void rgbFromHSVAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                uint item [[thread_position_in_grid]]) {
    Vector3Of<float> rgb = rgbFromHSV(Vector3Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]));
    outputs[item * 3 + 0] = rgb.x;
    outputs[item * 3 + 1] = rgb.y;
    outputs[item * 3 + 2] = rgb.z;
}

/// Inputs per item: (red, green, blue), linear. Outputs: oklabFromLinearSRGB.
kernel void oklabFromLinearSRGBAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                         uint item [[thread_position_in_grid]]) {
    Vector3Of<float> lab = oklabFromLinearSRGB(Vector3Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]));
    outputs[item * 3 + 0] = lab.x;
    outputs[item * 3 + 1] = lab.y;
    outputs[item * 3 + 2] = lab.z;
}

/// Inputs per item: (hue in degrees, saturation, value). Outputs: encodedSRGBFromOkhsv.
kernel void encodedSRGBFromOkhsvAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                          uint item [[thread_position_in_grid]]) {
    Vector3Of<float> rgb = encodedSRGBFromOkhsv(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]);
    outputs[item * 3 + 0] = rgb.x;
    outputs[item * 3 + 1] = rgb.y;
    outputs[item * 3 + 2] = rgb.z;
}

/// Inputs per item: (x, low, mid, high). Output: toneCurveValue.
kernel void toneCurveValueAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                    uint item [[thread_position_in_grid]]) {
    outputs[item] = toneCurveValue(inputs[item * 4 + 0], inputs[item * 4 + 1], inputs[item * 4 + 2], inputs[item * 4 + 3]);
}
