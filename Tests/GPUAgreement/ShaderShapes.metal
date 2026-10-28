// Kernels/ShaderShapes.h on the GPU, for ShaderShapes.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderShapes.h"

using namespace editor::kernels;

/// Inputs per item: (x, y, ax, ay, bx, by, cx, cy). Output: bezierDistance.
kernel void bezierDistanceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                    uint item [[thread_position_in_grid]]) {
    const device float* values = inputs + item * 8;
    outputs[item] = bezierDistance(Vector2Of<float>(values[0], values[1]), Vector2Of<float>(values[2], values[3]),
        Vector2Of<float>(values[4], values[5]), Vector2Of<float>(values[6], values[7]));
}

/// Inputs per item: (x, y, radius, points, pointiness). Output: starDistance.
kernel void starDistanceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                  uint item [[thread_position_in_grid]]) {
    const device float* values = inputs + item * 5;
    outputs[item] = starDistance(Vector2Of<float>(values[0], values[1]), values[2], values[3], values[4]);
}

/// Inputs per item: (x, y, p0 … p5 as x, y pairs, count). Output: polygonDistance.
kernel void polygonDistanceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                     uint item [[thread_position_in_grid]]) {
    const device float* values = inputs + item * 15;
    outputs[item] = polygonDistance(Vector2Of<float>(values[0], values[1]), Vector2Of<float>(values[2], values[3]),
        Vector2Of<float>(values[4], values[5]), Vector2Of<float>(values[6], values[7]), Vector2Of<float>(values[8], values[9]),
        Vector2Of<float>(values[10], values[11]), Vector2Of<float>(values[12], values[13]), values[14]);
}

/// Inputs per item: (x, y, size). Output: heartDistance.
kernel void heartDistanceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                   uint item [[thread_position_in_grid]]) {
    outputs[item] = heartDistance(Vector2Of<float>(inputs[item * 3 + 0], inputs[item * 3 + 1]), inputs[item * 3 + 2]);
}
