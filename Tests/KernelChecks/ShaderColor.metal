// Compiles Kernels/ShaderColor.h as Metal Shading Language (`make kernels`). Templates are only
// checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/ShaderColor.h"

using namespace editor::kernels;

kernel void shaderColorCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f;
    Vector3Of<float> color(x, 0.5f, 0.25f);
    Vector3Of<float> tint(0.2f, 0.4f, 0.8f);
    Vector4Of<float> stop(x, 0.5f, 0.25f, 1.0f);

    results[index * 16 + 0] = stepFunction(0.5f, x) + okhsvToeInverse(x) + okhsvMaximumSaturation(Vector2Of<float>(cosine(x), sine(x)))
        + okhsvCuspSlopes(Vector2Of<float>(cosine(x), sine(x))).y;
    results[index * 16 + 1] = linearSRGBFromOklab(color).y + encodedSRGBFromLinear(color).z + encodedSRGBFromOklab(color).x
        + linearFromEncodedColor(color).x;
    results[index * 16 + 2] = encodedSRGBFromOkhsv(x * 360.0f, 0.5f, 0.75f).y;
    results[index * 16 + 3] = hsvFromRGB(color).x + rgbFromHSV(color).y + hslFromRGB(color).z + rgbFromHSL(color).x;
    results[index * 16 + 4] = oklabFromLinearSRGB(color).y + oklchFromOklab(color).z + oklabFromOklch(color).x;
    results[index * 16 + 5] = convertedColorSpace(color, int(index % 10)).y;
    results[index * 16 + 6] = hueShifted(color, x).x + saturationAdjusted(color, x).y + vibranceAdjusted(color, x).z
        + grayscaled(color, int(index % 4)).x;
    results[index * 16 + 7] = levelsAdjusted(color, 0.1f, 0.9f, 1.5f, 0.0f, 1.0f).y + toneCurveValue(x, 0.2f, 0.5f, 0.8f)
        + toneCurveColor(color, 0.2f, 0.5f, 0.8f).z;
    results[index * 16 + 8] = posterized(color, 4.0f).x + luminanceThresholdMask(color, 0.5f, 0.2f)
        + colorized(color, tint, 0.5f).y + whiteBalanced(color, 0.3f, -0.2f).z;
    results[index * 16 + 9] = splitToned(color, tint, tint, 0.5f, 0.5f).x + rampColor(x, stop, stop * 0.5f, stop * 0.25f, stop, 0.0f, 0.3f, 0.6f, 1.0f).y
        + cosinePalette(x, color, tint, color, tint).y;
}
