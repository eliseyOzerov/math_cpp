// Compiles Kernels/KernelVectors.h as Metal Shading Language (`make kernels`): the vector types
// are Metal's own here, so this checks that every function deduces its template from `float2`,
// `float3` and `float4`.
#include <metal_stdlib>

#include "../../Kernels/KernelVectors.h"

using namespace editor::kernels;

kernel void kernelVectorsCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float value = float(index) / 64.0f - 0.5f;
    Vector2Of<float> plane(value, 1.0f);
    Vector3Of<float> space(value, 1.0f, -2.0f);
    Vector4Of<float> colour(value, 0.5f, 0.25f, 1.0f);
    results[index * 4 + 0] = dotProduct(plane, plane) + lengthOf(normalized(space)) + lengthOf(colour);
    results[index * 4 + 1] = crossProduct(space, Vector3Of<float>(0.0f, 0.0f, 1.0f)).x
        + absolute(space).y + roundedDown(plane).x + fractionalPart(colour).w;
    results[index * 4 + 2] = smallerOf(space, Vector3Of<float>(0.5f)).z + largerOf(plane, Vector2Of<float>(0.5f)).y
        + clamped(colour, 0.0f, 1.0f).x + linearBlend(space, -space, 0.25f).z;
    results[index * 4 + 3] = chosen(space < Vector3Of<float>(0.0f), space, -space).x
        + chosen(plane >= Vector2Of<float>(0.0f), plane, plane * 2.0f).y
        + chosen(colour > Vector4Of<float>(0.5f), colour, colour / 2.0f).z;
}
