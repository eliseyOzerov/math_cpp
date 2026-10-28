// Compiles Kernels/SRGBTransfer.h as Metal Shading Language (`make kernels`). Templates are only
// checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/SRGBTransfer.h"

using namespace editor::kernels;

kernel void srgbTransferCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float value = float(index) / 64.0f;
    results[index * 4 + 0] = SRGBTransfer::encodedFromLinear(value);
    results[index * 4 + 1] = SRGBTransfer::linearFromEncoded(value);
    results[index * 4 + 2] = SRGBTransfer::linearThreshold<float>() + SRGBTransfer::encodedThreshold<float>()
        + SRGBTransfer::linearSegmentSlope<float>();
    results[index * 4 + 3] = SRGBTransfer::powerSegmentScale<float>() + SRGBTransfer::powerSegmentOffset<float>()
        + SRGBTransfer::gamma<float>();
}
