// Kernels/ShaderColor.h: the GPU (ShaderColor.metal) against the CPU, for a sample of the library:
// RGB to HSV and back, RGB to OKLab, OKHSV to sRGB (the longest chain) and the tone curve.
//
// Tolerances: the GPU builds with fast math, so division, `pow` and the trig functions are good to
// a few units in the last place (about 1e-7 for values near 1). Plain arithmetic and comparisons
// agree exactly. HSV and the tone curve are almost all arithmetic: 1e-5 is generous. OKLab takes
// three cube roots by `pow`, and OKHSV chains a cosine, a sine, two cube roots, divisions and the
// sRGB power curve, so those two allow 1e-4 (a 255th of an 8-bit step is 1.5e-5).
#include "../../Kernels/ShaderColor.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector3Of;

AGREEMENT({
    // Corners, greys (equal channels), and mixes of 0…1 in 0.1 steps.
    "hsvFromRGBAgreement", 3, 3,
    everyTriple(evenlySpaced(0.0f, 1.0f, 11), evenlySpaced(0.0f, 1.0f, 11), evenlySpaced(0.0f, 1.0f, 11)),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> hsv = editor::kernels::hsvFromRGB(Vector3Of<float>(inputs[0], inputs[1], inputs[2]));
        outputs[0] = hsv.x;
        outputs[1] = hsv.y;
        outputs[2] = hsv.z;
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    // Hue from -0.25 to 1.25 covers the wrap on both sides; the steps avoid exact sixths, where
    // fast-math rounding in `fract` could land a value on the other side of a whole number.
    "rgbFromHSVAgreement", 3, 3,
    everyTriple(evenlySpaced(-0.2501f, 1.2501f, 61), evenlySpaced(0.0f, 1.0f, 6), evenlySpaced(0.0f, 1.0f, 6)),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> rgb = editor::kernels::rgbFromHSV(Vector3Of<float>(inputs[0], inputs[1], inputs[2]));
        outputs[0] = rgb.x;
        outputs[1] = rgb.y;
        outputs[2] = rgb.z;
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    "oklabFromLinearSRGBAgreement", 3, 3,
    everyTriple(evenlySpaced(0.0f, 1.0f, 11), evenlySpaced(0.0f, 1.0f, 11), evenlySpaced(0.0f, 1.0f, 11)),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> lab = editor::kernels::oklabFromLinearSRGB(Vector3Of<float>(inputs[0], inputs[1], inputs[2]));
        outputs[0] = lab.x;
        outputs[1] = lab.y;
        outputs[2] = lab.z;
    },
    1e-4f, 1e-4f,
});

AGREEMENT({
    // Hues every 15° from -30° to 390° (the wrap), saturation and value 0…1.
    "encodedSRGBFromOkhsvAgreement", 3, 3,
    everyTriple(evenlySpaced(-30.0f, 390.0f, 29), evenlySpaced(0.0f, 1.0f, 6), evenlySpaced(0.05f, 1.0f, 6)),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> rgb = editor::kernels::encodedSRGBFromOkhsv(inputs[0], inputs[1], inputs[2]);
        outputs[0] = rgb.x;
        outputs[1] = rgb.y;
        outputs[2] = rgb.z;
    },
    1e-4f, 1e-4f,
});

AGREEMENT({
    // x from -0.1 to 1.1 (the clamp) against two sets of control points.
    "toneCurveValueAgreement", 4, 1,
    [] {
        std::vector<float> inputs;
        for (float x : evenlySpaced(-0.1f, 1.1f, 241)) {
            for (float low : {0.1f, 0.25f}) {
                inputs.insert(inputs.end(), {x, low, 0.6f, 0.9f});
            }
        }
        return inputs;
    }(),
    [](const float* inputs, float* outputs) { outputs[0] = editor::kernels::toneCurveValue(inputs[0], inputs[1], inputs[2], inputs[3]); },
    1e-5f, 1e-5f,
});
