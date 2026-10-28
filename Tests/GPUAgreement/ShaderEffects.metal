// Kernels/ShaderEffects.h on the GPU, for ShaderEffects.cpp beside it.
#include <metal_stdlib>

#include "../../Kernels/ShaderEffects.h"

using namespace editor::kernels;

/// Inputs per item: (x, y). Outputs: hazeUv (strength 2, scale 6, time 1.5) then lensUv (strength 0.3, zoom 1.1).
kernel void hazeAndLensAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                 uint item [[thread_position_in_grid]]) {
    Vector2Of<float> uv(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    Vector2Of<float> haze = hazeUv(uv, 2.0f, 6.0f, 1.5f);
    Vector2Of<float> lens = lensUv(uv, 0.3f, 1.1f);
    outputs[item * 4 + 0] = haze.x;
    outputs[item * 4 + 1] = haze.y;
    outputs[item * 4 + 2] = lens.x;
    outputs[item * 4 + 3] = lens.y;
}

/// Inputs per item: (x, y). Outputs: the red of vignetteColor of (0.8, 0.5, 0.25, 1) at position (x, y), radius 0.8, softness 0.4,
/// intensity 0.7; then the red of filmGrain at pixel (100 x, 100 y), time 1.3, amount 0.2, size 2.
kernel void vignetteAndGrainAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                      uint item [[thread_position_in_grid]]) {
    float x = inputs[item * 2 + 0];
    float y = inputs[item * 2 + 1];
    Vector4Of<float> color(0.8f, 0.5f, 0.25f, 1.0f);
    outputs[item * 2 + 0] = vignetteColor(color, Vector2Of<float>(x, y), 0.8f, 0.4f, 0.7f).x;
    outputs[item * 2 + 1] = filmGrain(color, Vector2Of<float>(100.0f * x, 100.0f * y), 1.3f, 0.2f, 2.0f).x;
}

/// Inputs per item: (x, y), a uv. Outputs: crtWarpedUv of crtCentredWarp (curvature 0.8), then the red of crtShadedColor of
/// (0.8, 0.5, 0.25, 1) at pixel (20 x, 20 y), spacing 3, intensity 0.6.
kernel void crtAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                         uint item [[thread_position_in_grid]]) {
    Vector2Of<float> uv(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    Vector2Of<float> centred = crtCentredWarp(uv, 0.8f);
    Vector2Of<float> warped = crtWarpedUv(centred);
    Vector4Of<float> shaded = crtShadedColor(Vector4Of<float>(0.8f, 0.5f, 0.25f, 1.0f), centred, uv * 20.0f, 3.0f, 0.6f, 0.8f);
    outputs[item * 3 + 0] = warped.x;
    outputs[item * 3 + 1] = warped.y;
    outputs[item * 3 + 2] = shaded.x;
}

/// Inputs per item: (slope x, slope y). Outputs: normalFromHeightColor (strength 2) of mode 0 (red, green, blue), then mode 1
/// lit from 40 degrees over (0.8, 0.5, 0.25, 1) (red).
kernel void normalFromHeightAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                      uint item [[thread_position_in_grid]]) {
    Vector2Of<float> slope(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    Vector4Of<float> color(0.8f, 0.5f, 0.25f, 1.0f);
    Vector4Of<float> flat = normalFromHeightColor(slope, 2.0f, 0, 0.0f, color);
    Vector4Of<float> lit = normalFromHeightColor(slope, 2.0f, 1, 40.0f, color);
    outputs[item * 4 + 0] = flat.x;
    outputs[item * 4 + 1] = flat.y;
    outputs[item * 4 + 2] = flat.z;
    outputs[item * 4 + 3] = lit.x;
}

/// Inputs per item: (x, y), a uv. Outputs: the red and alpha of mosaicTileColor of (0.8, 0.5, 0.25, 0.9) with 6 tiles, aspect scale
/// (1.5, 1), gap 0.08, variation 0.3.
kernel void mosaicAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                            uint item [[thread_position_in_grid]]) {
    Vector4Of<float> tile = mosaicTileColor(Vector4Of<float>(0.8f, 0.5f, 0.25f, 0.9f), Vector2Of<float>(inputs[item * 2 + 0], inputs[item * 2 + 1]),
                                            Vector2Of<float>(1.5f, 1.0f), 6.0f, 0.08f, 0.3f);
    outputs[item * 2 + 0] = tile.x;
    outputs[item * 2 + 1] = tile.w;
}

/// Inputs per item: (x, y), a uv. Outputs: radialBlurPoint (middle 0.5, fraction 0.3, 40 degrees) then zoomBlurPoint (fraction 0.3, 25 percent).
kernel void blurPointsAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                uint item [[thread_position_in_grid]]) {
    Vector2Of<float> uv(inputs[item * 2 + 0], inputs[item * 2 + 1]);
    Vector2Of<float> radial = radialBlurPoint(uv, Vector2Of<float>(0.5f), 0.3f, 40.0f);
    Vector2Of<float> zoom = zoomBlurPoint(uv, Vector2Of<float>(0.5f), 0.3f, 25.0f);
    outputs[item * 4 + 0] = radial.x;
    outputs[item * 4 + 1] = radial.y;
    outputs[item * 4 + 2] = zoom.x;
    outputs[item * 4 + 3] = zoom.y;
}
