// Kernels/BlendModes.h: the GPU (BlendModes.metal) against the CPU, for the per-channel modes
// (every mode over a grid, with the dodge and burn divisions), the whole-colour modes (including the
// non-separable Hue to Luminosity, with their clipping) and compositing with alpha.
//
// Tolerance: outputs are 0…1 colours made from products, sums and at most two divisions, so the GPU's
// fast-math divide (a few units in the last place, about 3e-7) is the only difference: 1e-5 is
// generous. The grids are on exact binary fractions, so the mode comparisons (`base <= 0.5`)
// see identical values on both sides.
#include "../../Kernels/BlendModes.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector3Of;
using editor::kernels::Vector4Of;

namespace {
/// Every combination of the given levels for `count` positions, then each mode and the extra
/// numbers after them: inputs for the colour kernels.
std::vector<float> colorGrid(const std::vector<float>& levels, int modeCount, const std::vector<float>& extras) {
    std::vector<float> inputs;
    for (float baseRed : levels)
        for (float baseGreen : levels)
            for (float baseBlue : levels)
                for (float topRed : levels)
                    for (float topGreen : levels)
                        for (float topBlue : levels)
                            for (int mode = 0; mode < modeCount; mode += 1) {
                                inputs.insert(inputs.end(), {baseRed, baseGreen, baseBlue, topRed, topGreen, topBlue,
                                    static_cast<float>(mode)});
                                inputs.insert(inputs.end(), extras.begin(), extras.end());
                            }
    return inputs;
}
} // namespace

AGREEMENT({
    "blendedChannelAgreement", 3, 1,
    [] {
        std::vector<float> inputs;
        for (int mode = 0; mode < 14; mode += 1) {
            for (float base : evenlySpaced(0.0f, 1.0f, 17)) {
                for (float top : evenlySpaced(0.0f, 1.0f, 17)) inputs.insert(inputs.end(), {base, top, static_cast<float>(mode)});
            }
        }
        return inputs;
    }(),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::blendedChannel(inputs[0], inputs[1], static_cast<int>(inputs[2]));
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    // Four levels per channel (0, 1/4, 3/4, 1... all exact in binary), all 18 modes.
    "blendedColorAgreement", 7, 3,
    colorGrid({0.0f, 0.25f, 0.75f, 1.0f}, 18, {}),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> color = editor::kernels::blendedColor(Vector3Of<float>(inputs[0], inputs[1], inputs[2]),
            Vector3Of<float>(inputs[3], inputs[4], inputs[5]), static_cast<int>(inputs[6]));
        outputs[0] = color.x;
        outputs[1] = color.y;
        outputs[2] = color.z;
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    // Colours with alpha 0.5 and opacity 0.7 (the alpha rules matter more than the grid here).
    "compositedWithBlendAgreement", 10, 4,
    [] {
        std::vector<float> inputs;
        for (float baseAlpha : {0.0f, 0.5f, 1.0f}) {
            for (float topAlpha : {0.0f, 0.5f, 1.0f}) {
                for (int mode = 0; mode < 18; mode += 1) {
                    for (float red : {0.0f, 0.25f, 0.75f}) {
                        for (float green : {0.25f, 1.0f}) {
                            inputs.insert(inputs.end(), {red, green, 0.5f, baseAlpha, 1.0f - red, 0.5f, green, topAlpha,
                                static_cast<float>(mode), 0.7f});
                        }
                    }
                }
            }
        }
        return inputs;
    }(),
    [](const float* inputs, float* outputs) {
        Vector4Of<float> color = editor::kernels::compositedWithBlend(Vector4Of<float>(inputs[0], inputs[1], inputs[2], inputs[3]),
            Vector4Of<float>(inputs[4], inputs[5], inputs[6], inputs[7]), static_cast<int>(inputs[8]), inputs[9]);
        outputs[0] = color.x;
        outputs[1] = color.y;
        outputs[2] = color.z;
        outputs[3] = color.w;
    },
    1e-5f, 1e-5f,
});
