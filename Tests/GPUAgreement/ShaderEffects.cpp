// Kernels/ShaderEffects.h: the GPU (ShaderEffects.metal) against the CPU, for the haze and lens warps, the vignette and
// grain, the CRT warp and shading, the normal from a height, the mosaic tile and the radial and zoom blur points. They
// are short formulas, so only rounding differs, except for the sine and cosine of the haze (arguments up to about 25) and of
// the scanlines (up to about 20), where fast math is a little looser. The grain and the mosaic hash a cell, so their inputs
// avoid cell borders.
#include "../../Kernels/ShaderEffects.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector2Of;
using editor::kernels::Vector4Of;

namespace {
/// Points with irrational-looking spacing so none sits on a cell border.
std::vector<float> plane(float first, float last, int count) {
    return everyPair(evenlySpaced(first + 0.0113f, last - 0.0129f, count), evenlySpaced(first + 0.0197f, last - 0.0077f, count));
}
} // namespace

AGREEMENT({
    "hazeAndLensAgreement", 2, 4, plane(0.0f, 1.0f, 41),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> uv(inputs[0], inputs[1]);
        Vector2Of<float> haze = editor::kernels::hazeUv(uv, 2.0f, 6.0f, 1.5f);
        Vector2Of<float> lens = editor::kernels::lensUv(uv, 0.3f, 1.1f);
        outputs[0] = haze.x;
        outputs[1] = haze.y;
        outputs[2] = lens.x;
        outputs[3] = lens.y;
    },
    2e-4f, 1e-5f,
});

AGREEMENT({
    "vignetteAndGrainAgreement", 2, 2, plane(-1.2f, 1.2f, 41),
    [](const float* inputs, float* outputs) {
        Vector4Of<float> color(0.8f, 0.5f, 0.25f, 1.0f);
        outputs[0] = editor::kernels::vignetteColor(color, Vector2Of<float>(inputs[0], inputs[1]), 0.8f, 0.4f, 0.7f).x;
        outputs[1] = editor::kernels::filmGrain(color, Vector2Of<float>(100.0f * inputs[0], 100.0f * inputs[1]), 1.3f, 0.2f, 2.0f).x;
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    "crtAgreement", 2, 3, plane(0.0f, 1.0f, 41),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> uv(inputs[0], inputs[1]);
        Vector2Of<float> centred = editor::kernels::crtCentredWarp(uv, 0.8f);
        Vector2Of<float> warped = editor::kernels::crtWarpedUv(centred);
        Vector4Of<float> shaded = editor::kernels::crtShadedColor(Vector4Of<float>(0.8f, 0.5f, 0.25f, 1.0f), centred, uv * 20.0f, 3.0f, 0.6f, 0.8f);
        outputs[0] = warped.x;
        outputs[1] = warped.y;
        outputs[2] = shaded.x;
    },
    1e-3f, 1e-5f,
});

AGREEMENT({
    "normalFromHeightAgreement", 2, 4, plane(-1.5f, 1.5f, 41),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> slope(inputs[0], inputs[1]);
        Vector4Of<float> color(0.8f, 0.5f, 0.25f, 1.0f);
        Vector4Of<float> flat = editor::kernels::normalFromHeightColor(slope, 2.0f, 0, 0.0f, color);
        Vector4Of<float> lit = editor::kernels::normalFromHeightColor(slope, 2.0f, 1, 40.0f, color);
        outputs[0] = flat.x;
        outputs[1] = flat.y;
        outputs[2] = flat.z;
        outputs[3] = lit.x;
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    "mosaicAgreement", 2, 2, plane(0.0f, 1.0f, 41),
    [](const float* inputs, float* outputs) {
        Vector4Of<float> tile = editor::kernels::mosaicTileColor(Vector4Of<float>(0.8f, 0.5f, 0.25f, 0.9f), Vector2Of<float>(inputs[0], inputs[1]),
                                                                  Vector2Of<float>(1.5f, 1.0f), 6.0f, 0.08f, 0.3f);
        outputs[0] = tile.x;
        outputs[1] = tile.w;
    },
    1e-4f, 1e-5f,
});

AGREEMENT({
    "blurPointsAgreement", 2, 4, plane(0.0f, 1.0f, 41),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> uv(inputs[0], inputs[1]);
        Vector2Of<float> radial = editor::kernels::radialBlurPoint(uv, Vector2Of<float>(0.5f), 0.3f, 40.0f);
        Vector2Of<float> zoom = editor::kernels::zoomBlurPoint(uv, Vector2Of<float>(0.5f), 0.3f, 25.0f);
        outputs[0] = radial.x;
        outputs[1] = radial.y;
        outputs[2] = zoom.x;
        outputs[3] = zoom.y;
    },
    1e-5f, 1e-5f,
});
