// Kernels/Hashes.h on the GPU, for Hashes.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/Hashes.h"

using namespace editor::kernels;

/// Inputs per item: (p). Output: hash11(p).
kernel void hash11Agreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                            uint item [[thread_position_in_grid]]) {
    outputs[item] = hash11(inputs[item]);
}

/// Inputs per item: (x, y, z). Outputs: the three components of hash33.
kernel void hash33Agreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                            uint item [[thread_position_in_grid]]) {
    Vector3Of<float> hash = hash33(Vector3Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]));
    outputs[item * 3 + 0] = hash.x;
    outputs[item * 3 + 1] = hash.y;
    outputs[item * 3 + 2] = hash.z;
}
