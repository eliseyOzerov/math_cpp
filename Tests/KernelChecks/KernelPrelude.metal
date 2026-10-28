// Compiles Kernels/KernelPrelude.h as Metal Shading Language (`make kernels`). Templates are only
// checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/KernelPrelude.h"

using namespace editor::kernels;

kernel void kernelPreludeCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float value = float(index) / 64.0f - 0.5f;
    results[index * 8 + 0] = squareRoot(absolute(value)) + linearBlend(0.0f, 1.0f, clamped(value, 0.0f, 1.0f));
    results[index * 8 + 1] = smallerOf(value, 0.25f) + largerOf(value, 0.25f);
    results[index * 8 + 2] = roundedDown(value) + fractionalPart(value);
    results[index * 8 + 3] = sine(value) + cosine(value) + arcTangent(value, 1.0f);
    results[index * 8 + 4] = power(absolute(value), 2.2f);
    results[index * 8 + 5] = exponential(value) + naturalLogarithm(absolute(value) + 1.0f);
    results[index * 8 + 6] = chosen(value < 0.0f, -1.0f, 1.0f);
    results[index * 8 + 7] = float(bitsOf(value) >> 23);
}
