// Compiles Kernels/BlendModes.h as Metal Shading Language (`make kernels`). Templates are only
// checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/BlendModes.h"

using namespace editor::kernels;

kernel void blendModesCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f;
    Vector3Of<float> base(x, 0.5f, 0.25f);
    Vector3Of<float> top(0.75f, x, 0.1f);
    Vector4Of<float> baseAlpha(x, 0.5f, 0.25f, 0.8f);
    Vector4Of<float> topAlpha(0.75f, x, 0.1f, 0.6f);

    results[index * 8 + 0] = normalBlend(x, 0.5f) + multiplyBlend(x, 0.5f) + screenBlend(x, 0.5f) + overlayBlend(x, 0.5f)
        + softLightBlend(x, 0.5f) + hardLightBlend(x, 0.5f) + addBlend(x, 0.5f);
    results[index * 8 + 1] = subtractBlend(x, 0.5f) + differenceBlend(x, 0.5f) + exclusionBlend(x, 0.5f)
        + colorDodgeBlend(x, 0.5f) + colorBurnBlend(x, 0.5f) + darkenBlend(x, 0.5f) + lightenBlend(x, 0.5f);
    results[index * 8 + 2] = blendedChannel(x, 0.5f, int(index % 14));
    results[index * 8 + 3] = blendLuminance(base) + blendSaturation(top) + clippedIntoRange(base * 2.0f - 0.5f).y
        + withBlendLuminance(base, x).z + withBlendSaturation(top, x).x;
    results[index * 8 + 4] = blendedColor(base, top, int(index % 18)).y;
    results[index * 8 + 5] = compositedWithBlend(baseAlpha, topAlpha, int(index % 18), 0.7f).y + straightAlphaOver(baseAlpha, topAlpha).w;
}
