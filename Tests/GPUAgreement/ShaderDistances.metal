// Kernels/ShaderDistances.h on the GPU, for ShaderDistances.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderDistances.h"

using namespace editor::kernels;

/// Inputs per item: (x, y, z, halfX, halfY, halfZ, rounding). Output: boxDistance.
kernel void boxDistanceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                 uint item [[thread_position_in_grid]]) {
    const device float* values = inputs + item * 7;
    outputs[item] = boxDistance(Vector3Of<float>(values[0], values[1], values[2]), Vector3Of<float>(values[3], values[4], values[5]), values[6]);
}

/// Inputs per item: (x, y, z, radius, height). Output: coneDistance.
kernel void coneDistanceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                  uint item [[thread_position_in_grid]]) {
    const device float* values = inputs + item * 5;
    outputs[item] = coneDistance(Vector3Of<float>(values[0], values[1], values[2]), values[3], values[4]);
}

/// Inputs per item: (x, y, z, degreesX, degreesY, degreesZ). Outputs: the point of rotatedPoint.
kernel void rotatedPointAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                  uint item [[thread_position_in_grid]]) {
    const device float* values = inputs + item * 6;
    Vector3Of<float> turned = rotatedPoint(Vector3Of<float>(values[0], values[1], values[2]), Vector3Of<float>(values[3], values[4], values[5]));
    outputs[item * 3 + 0] = turned.x;
    outputs[item * 3 + 1] = turned.y;
    outputs[item * 3 + 2] = turned.z;
}

/// Inputs per item: (distance, width, alignment, pixelHeight, feather). Output: distanceStrokeMask.
kernel void distanceStrokeMaskAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                        uint item [[thread_position_in_grid]]) {
    const device float* values = inputs + item * 5;
    outputs[item] = distanceStrokeMask(values[0], values[1], int(values[2]), values[3], values[4]);
}
