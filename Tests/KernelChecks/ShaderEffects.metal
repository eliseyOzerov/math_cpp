// Compiles Kernels/ShaderEffects.h as Metal Shading Language (`make kernels`). Every function is called here.
#include <metal_stdlib>

#include "../../Kernels/ShaderEffects.h"

using namespace editor::kernels;

kernel void shaderEffectsCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f;
    Vector2Of<float> uv(x, 1.0f - x);
    Vector2Of<float> texel(0.01f, 0.02f);
    Vector2Of<float> middle(0.5f, 0.5f);
    Vector2Of<float> aspectScale(1.5f, 1.0f);
    Vector4Of<float> color(x, 0.5f, 0.25f, 0.8f);

    Vector2Of<float> tap = blurTapOffset(int(index % 7) - 3, 1);
    results[index * 16 + 0] = tap.x + blurTapWeight(tap, int(index % 2)) + blurTapFraction(int(index % 16))
        + radialBlurPoint(uv, middle, 0.2f, 30.0f).x + zoomBlurPoint(uv, middle, 0.2f, 10.0f).y;
    Vector4Of<float> sharp = sharpenedColor(color, color * 4.0f, 0.5f);
    Vector2Of<float> slope = luminanceSlope(0.4f, 0.2f, 0.7f, 0.1f);
    Vector2Of<float> sobel = sobelGradient(0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, x);
    results[index * 16 + 1] = sharp.x + sharp.w + slope.x + sobel.y + edgeStrengthOf(sobel, 2.0f);
    Vector2Of<float> emboss = embossOffset(45.0f, texel);
    Vector4Of<float> embossed = embossColor(0.7f, 0.2f, 1.5f);
    Vector4Of<float> fringe = chromaticColor(color, color * 0.5f, color * 0.25f);
    results[index * 16 + 2] = emboss.x + embossed.x + chromaticOffset(uv, 2.0f, middle).x + fringe.x + fringe.y + fringe.z;
    Vector4Of<float> vignette = vignetteColor(color, uv - 0.5f, 0.8f, 0.4f, 0.7f);
    Vector4Of<float> grain = filmGrain(color, Vector2Of<float>(x * 100.0f, 7.0f), 1.3f, 0.2f, 2.0f);
    results[index * 16 + 3] = vignette.x + vignette.w + grain.x + grain.y + grain.z;
    Vector2Of<float> centred = crtCentredWarp(uv, 0.8f);
    Vector2Of<float> warped = crtWarpedUv(centred);
    Vector4Of<float> crt = crtShadedColor(color, centred, Vector2Of<float>(x * 300.0f, x * 200.0f), 3.0f, 0.6f, 0.8f);
    results[index * 16 + 4] = warped.x + (isOutsideUnitSquare(warped) ? 1.0f : 0.0f) + crt.x + crt.y + crt.z + crt.w
        + crtScanlineFactor(x * 200.0f, 3.0f, 0.6f) + crtMask(x * 300.0f, 0.6f).y + crtCornerDarkening(centred, 0.8f);
    Vector2Of<float> ring = bloomRingOffset(1 + int(index % 3), int(index % 8), 20.0f, texel);
    Vector4Of<float> bloom = bloomCombined(color, bloomBrightPart(Vector3Of<float>(color.x, color.y, color.z), 0.3f), 14.0f, 0.5f);
    results[index * 16 + 5] = ring.x + bloomRingWeight(2) + bloom.x + bloom.y;
    Vector2Of<float> id = mosaicTileId(uv, aspectScale, 8.0f);
    Vector4Of<float> tile = mosaicTileColor(color, uv, aspectScale, 8.0f, 0.1f, 0.3f);
    results[index * 16 + 6] = pixelateTapUv(uv, 8.0f, Vector2Of<float>(640.0f, 480.0f)).x + id.x
        + mosaicTileCentreUv(id, aspectScale, 8.0f).y + tile.x + tile.w;
    Vector2Of<float> shift = displacementFromMap(Vector2Of<float>(x, 0.3f));
    Vector4Of<float> flat = normalFromHeightColor(slope, 2.0f, 0, 0.0f, color);
    Vector4Of<float> lit = normalFromHeightColor(slope, 2.0f, 1, 40.0f, color);
    results[index * 16 + 7] = displacedUv(uv, shift, 0.1f).x + flat.x + flat.z + lit.x + lit.y;
    results[index * 16 + 8] = hazeUv(uv, 2.0f, 6.0f, 1.5f).x + lensUv(uv, 0.3f, 1.1f).y;
    Vector2Of<float> sign = kuwaharaQuadrantSign(int(index % 4));
    int radius = kuwaharaRadius(float(index % 6));
    Vector4Of<float> first = kuwaharaQuadrantStatistics(Vector3Of<float>(1.0f, 2.0f, 3.0f), 20.0f, radius);
    Vector4Of<float> second = kuwaharaQuadrantStatistics(Vector3Of<float>(1.0f, 1.0f, 1.0f), x, radius);
    Vector3Of<float> mean = kuwaharaSelectedMean(first, second, first, second);
    results[index * 16 + 9] = sign.x + sign.y + float(kuwaharaBestQuadrant(first, second, second, first)) + mean.x + mean.y + mean.z;
}
