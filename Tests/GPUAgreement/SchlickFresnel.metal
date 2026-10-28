// Kernels/SchlickFresnel.h on the GPU, for SchlickFresnel.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/SchlickFresnel.h"

using namespace editor::kernels;

/// Inputs per item: (first index, second index). Output: F0.
kernel void schlickNormalIncidenceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                            uint item [[thread_position_in_grid]]) {
    outputs[item] = SchlickFresnel::reflectanceAtNormalIncidence(inputs[item * 2 + 0], inputs[item * 2 + 1]);
}

/// Inputs per item: (cosine, F0). Output: the reflectance.
kernel void schlickReflectanceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                        uint item [[thread_position_in_grid]]) {
    outputs[item] = SchlickFresnel::reflectance(inputs[item * 2 + 0], inputs[item * 2 + 1]);
}
