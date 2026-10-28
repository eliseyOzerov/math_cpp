// Kernels/SRGBTransfer.h on the GPU, for SRGBTransfer.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/SRGBTransfer.h"

using namespace editor::kernels;

/// Inputs per item: (linear). Output: the encoded value.
kernel void srgbEncodedFromLinearAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                           uint item [[thread_position_in_grid]]) {
    outputs[item] = SRGBTransfer::encodedFromLinear(inputs[item]);
}

/// Inputs per item: (encoded). Output: the linear value.
kernel void srgbLinearFromEncodedAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                           uint item [[thread_position_in_grid]]) {
    outputs[item] = SRGBTransfer::linearFromEncoded(inputs[item]);
}
