// Compiles Kernels/SchlickFresnel.h as Metal Shading Language (`make kernels`). Templates are only
// checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/SchlickFresnel.h"

using namespace editor::kernels;

kernel void schlickFresnelCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float cosine = float(index) / 64.0f;
    results[index * 2 + 0] = SchlickFresnel::reflectanceAtNormalIncidence(1.0f, 1.5f);
    results[index * 2 + 1] = SchlickFresnel::reflectance(cosine, 0.04f);
}
