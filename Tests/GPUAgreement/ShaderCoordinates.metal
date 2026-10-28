// Kernels/ShaderCoordinates.h on the GPU, for ShaderCoordinates.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderCoordinates.h"

using namespace editor::kernels;

/// Inputs per item: (x, y, segments, angle). Outputs: the two components of kaleidoscopeFolded.
kernel void kaleidoscopeFoldedAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                        uint item [[thread_position_in_grid]]) {
    Vector2Of<float> folded = kaleidoscopeFolded(Vector2Of<float>(inputs[item * 4 + 0], inputs[item * 4 + 1]), inputs[item * 4 + 2],
        inputs[item * 4 + 3]);
    outputs[item * 2 + 0] = folded.x;
    outputs[item * 2 + 1] = folded.y;
}

/// Inputs per item: (x, y, strength, radius). Outputs: the two components of twirled.
kernel void twirledAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                             uint item [[thread_position_in_grid]]) {
    Vector2Of<float> twisted = twirled(Vector2Of<float>(inputs[item * 4 + 0], inputs[item * 4 + 1]), inputs[item * 4 + 2], inputs[item * 4 + 3]);
    outputs[item * 2 + 0] = twisted.x;
    outputs[item * 2 + 1] = twisted.y;
}

/// Inputs per item: (x, y). Outputs: the four components of hexagonalGrid.
kernel void hexagonalGridAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    Vector4Of<float> cell = hexagonalGrid(Vector2Of<float>(inputs[item * 2 + 0], inputs[item * 2 + 1]));
    outputs[item * 4 + 0] = cell.x;
    outputs[item * 4 + 1] = cell.y;
    outputs[item * 4 + 2] = cell.z;
    outputs[item * 4 + 3] = cell.w;
}
