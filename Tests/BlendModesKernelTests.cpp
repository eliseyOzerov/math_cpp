// The shader-library blend kernels (Kernels/BlendModes.h) on the CPU, against values worked out by
// hand from the W3C formulas, and the properties each must keep (identities, ranges, symmetry). The
// same header is compiled as Metal by `make kernels`.
#include <cmath>

#include "../Kernels/BlendModes.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel3 = kernels::Vector3Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }
bool nearly(Kernel3 first, Kernel3 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance);
}
bool nearly(Kernel4 first, Kernel4 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance)
        && nearly(first.w, second.w, tolerance);
}

const float channelLevels[] = {0.0f, 0.1f, 0.25f, 0.4f, 0.5f, 0.6f, 0.75f, 0.9f, 1.0f};
} // namespace

TEST(separableModesOnHandWorkedValues) {
    // base 0.5, top 0.4
    EXPECT(nearly(kernels::normalBlend(0.5f, 0.4f), 0.4f));
    EXPECT(nearly(kernels::multiplyBlend(0.5f, 0.4f), 0.2f));
    EXPECT(nearly(kernels::screenBlend(0.5f, 0.4f), 0.7f));          // 0.5 + 0.4 − 0.2
    EXPECT(nearly(kernels::overlayBlend(0.5f, 0.4f), 0.4f));         // base ≤ ½: 2 × 0.5 × 0.4
    EXPECT(nearly(kernels::hardLightBlend(0.5f, 0.4f), 0.4f));       // top ≤ ½: 2 × 0.5 × 0.4
    EXPECT(nearly(kernels::addBlend(0.7f, 0.6f), 1.0f));             // held at 1
    EXPECT(nearly(kernels::addBlend(0.3f, 0.4f), 0.7f));
    EXPECT(nearly(kernels::subtractBlend(0.3f, 0.5f), 0.0f));        // held at 0
    EXPECT(nearly(kernels::subtractBlend(0.8f, 0.5f), 0.3f));
    EXPECT(nearly(kernels::differenceBlend(0.3f, 0.8f), 0.5f));
    EXPECT(nearly(kernels::exclusionBlend(0.3f, 0.8f), 0.62f));      // 1.1 − 2 × 0.24
    EXPECT(nearly(kernels::darkenBlend(0.3f, 0.8f), 0.3f));
    EXPECT(nearly(kernels::lightenBlend(0.3f, 0.8f), 0.8f));
    // Overlay and hard light differ in which colour decides: base 0.75, top 0.25.
    EXPECT(nearly(kernels::overlayBlend(0.75f, 0.25f), 0.625f));     // 1 − 2 × 0.25 × 0.75
    EXPECT(nearly(kernels::hardLightBlend(0.75f, 0.25f), 0.375f));   // 2 × 0.75 × 0.25
}

TEST(dodgeAndBurnGuardTheirDivisions) {
    EXPECT(nearly(kernels::colorDodgeBlend(0.25f, 0.5f), 0.5f));     // 0.25 / (1 − 0.5)
    EXPECT(nearly(kernels::colorDodgeBlend(0.75f, 0.5f), 1.0f));     // 1.5 held at 1
    EXPECT(kernels::colorDodgeBlend(0.0f, 1.0f) == 0.0f);            // a black base stays black
    EXPECT(kernels::colorDodgeBlend(0.5f, 1.0f) == 1.0f);            // a white top gives white
    EXPECT(nearly(kernels::colorBurnBlend(0.75f, 0.5f), 0.5f));      // 1 − 0.25 / 0.5
    EXPECT(nearly(kernels::colorBurnBlend(0.25f, 0.5f), 0.0f));      // 1 − min(1, 1.5)
    EXPECT(kernels::colorBurnBlend(1.0f, 0.0f) == 1.0f);             // a white base stays white
    EXPECT(kernels::colorBurnBlend(0.5f, 0.0f) == 0.0f);             // a black top gives black
}

TEST(softLightMatchesTheW3CFormula) {
    // base 0.25 (shade √0.25 = 0.5, which the polynomial branch also gives), top 0.75:
    // 0.25 + (1.5 − 1) × (0.5 − 0.25) = 0.375
    EXPECT(nearly(kernels::softLightBlend(0.25f, 0.75f), 0.375f));
    // base 0.5, top 0.25: 0.5 − (1 − 0.5) × 0.5 × 0.5 = 0.375
    EXPECT(nearly(kernels::softLightBlend(0.5f, 0.25f), 0.375f));
    // base 0.1 ≤ ¼, top 1: shade ((1.6 − 12) × 0.1 + 4) × 0.1 = 0.296; 0.1 + 1 × (0.296 − 0.1) = 0.296
    EXPECT(nearly(kernels::softLightBlend(0.1f, 1.0f), 0.296f));
}

TEST(identitiesOfTheSeparableModes) {
    for (float base : channelLevels) {
        EXPECT(nearly(kernels::multiplyBlend(base, 1.0f), base));       // white
        EXPECT(nearly(kernels::multiplyBlend(base, 0.0f), 0.0f));       // black
        EXPECT(nearly(kernels::screenBlend(base, 0.0f), base));         // black
        EXPECT(nearly(kernels::screenBlend(base, 1.0f), 1.0f));         // white
        EXPECT(nearly(kernels::softLightBlend(base, 0.5f), base));      // mid grey
        EXPECT(nearly(kernels::hardLightBlend(base, 0.5f), base));      // mid grey
        EXPECT(nearly(kernels::overlayBlend(0.5f, base), base));        // overlay on mid grey gives the top
        EXPECT(nearly(kernels::addBlend(base, 0.0f), base));
        EXPECT(nearly(kernels::subtractBlend(base, 0.0f), base));
        EXPECT(nearly(kernels::differenceBlend(base, base), 0.0f));
        EXPECT(nearly(kernels::exclusionBlend(base, 0.5f), 0.5f));
        EXPECT(nearly(kernels::darkenBlend(base, 1.0f), base));
        EXPECT(nearly(kernels::lightenBlend(base, 0.0f), base));
    }
}

TEST(everySeparableModeStaysWithinZeroAndOne) {
    for (int mode = 0; mode <= 13; mode += 1) {
        for (float base : channelLevels) {
            for (float top : channelLevels) {
                float result = kernels::blendedChannel(base, top, mode);
                EXPECT(result >= 0.0f && result <= 1.0f);
            }
        }
    }
}

TEST(someModesAreSymmetricAndOverlayIsHardLightSwapped) {
    for (float first : channelLevels) {
        for (float second : channelLevels) {
            EXPECT(nearly(kernels::multiplyBlend(first, second), kernels::multiplyBlend(second, first)));
            EXPECT(nearly(kernels::screenBlend(first, second), kernels::screenBlend(second, first)));
            EXPECT(nearly(kernels::addBlend(first, second), kernels::addBlend(second, first)));
            EXPECT(nearly(kernels::differenceBlend(first, second), kernels::differenceBlend(second, first)));
            EXPECT(nearly(kernels::exclusionBlend(first, second), kernels::exclusionBlend(second, first)));
            EXPECT(nearly(kernels::darkenBlend(first, second), kernels::darkenBlend(second, first)));
            EXPECT(nearly(kernels::lightenBlend(first, second), kernels::lightenBlend(second, first)));
            EXPECT(nearly(kernels::overlayBlend(first, second), kernels::hardLightBlend(second, first)));
        }
    }
}

TEST(blendedChannelSelectsByModeNumber) {
    float base = 0.3f;
    float top = 0.8f;
    EXPECT(kernels::blendedChannel(base, top, 0) == kernels::normalBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 1) == kernels::multiplyBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 2) == kernels::screenBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 3) == kernels::overlayBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 4) == kernels::softLightBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 5) == kernels::hardLightBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 6) == kernels::addBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 7) == kernels::subtractBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 8) == kernels::differenceBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 9) == kernels::exclusionBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 10) == kernels::colorDodgeBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 11) == kernels::colorBurnBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 12) == kernels::darkenBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 13) == kernels::lightenBlend(base, top));
    EXPECT(kernels::blendedChannel(base, top, 99) == kernels::lightenBlend(base, top));
}

TEST(blendLuminanceUsesTheW3CWeights) {
    // 0.3 + 0.59 + 0.11 = 1 for white; (0.2, 0.5, 0.8): 0.06 + 0.295 + 0.088 = 0.443
    EXPECT(nearly(kernels::blendLuminance(Kernel3(1, 1, 1)), 1.0f));
    EXPECT(nearly(kernels::blendLuminance(Kernel3(0.2f, 0.5f, 0.8f)), 0.443f));
    EXPECT(nearly(kernels::blendSaturation(Kernel3(0.2f, 0.5f, 0.8f)), 0.6f));
    EXPECT(nearly(kernels::blendSaturation(Kernel3(0.4f, 0.4f, 0.4f)), 0.0f));
}

TEST(clipAndSetSaturationOnKnownColors) {
    // Inside the range nothing changes.
    EXPECT(nearly(kernels::clippedIntoRange(Kernel3(0.2f, 0.5f, 0.8f)), Kernel3(0.2f, 0.5f, 0.8f)));
    // (1.2, 0.2, 0.2): l = 0.36 + 0.118 + 0.022 = 0.5, x = 1.2 → 0.5 + (c − 0.5) × 0.5 / 0.7
    EXPECT(nearly(kernels::clippedIntoRange(Kernel3(1.2f, 0.2f, 0.2f)), Kernel3(1.0f, 0.285714f, 0.285714f), 1e-5f));
    // Clipping keeps the luminance.
    EXPECT(nearly(kernels::blendLuminance(kernels::clippedIntoRange(Kernel3(1.2f, 0.2f, 0.2f))), 0.5f, 1e-5f));
    // (0.2, 0.4, 0.8) rescaled to saturation 0.3: (c − 0.2) × 0.3 / 0.6 = (0, 0.1, 0.3)
    EXPECT(nearly(kernels::withBlendSaturation(Kernel3(0.2f, 0.4f, 0.8f), 0.3f), Kernel3(0.0f, 0.1f, 0.3f)));
    // A grey has no hue to rescale.
    EXPECT(nearly(kernels::withBlendSaturation(Kernel3(0.4f, 0.4f, 0.4f), 0.3f), Kernel3(0, 0, 0)));
    // Setting a luminance moves all channels together: base (0.4, 0.5, 0.6) has l = 0.12 + 0.295 + 0.066 = 0.481.
    EXPECT(nearly(kernels::withBlendLuminance(Kernel3(0.4f, 0.5f, 0.6f), 0.5f), Kernel3(0.419f, 0.519f, 0.619f), 1e-5f));
}

TEST(nonSeparableModesOnHandWorkedValues) {
    Kernel3 red(1, 0, 0);
    Kernel3 grey(0.5f, 0.5f, 0.5f);
    // Color: the top's hue and saturation with the base's luminance. red over grey: set the luminance
    // of (1, 0, 0) (0.3) to 0.5 → (1.2, 0.2, 0.2) → clipped to (1, 0.285714, 0.285714).
    EXPECT(nearly(kernels::blendedColor(grey, red, 16), Kernel3(1.0f, 0.285714f, 0.285714f), 1e-5f));
    // Hue: a grey top has no hue, so the result is grey with the luminance of the base red (0.3).
    EXPECT(nearly(kernels::blendedColor(red, grey, 14), Kernel3(0.3f, 0.3f, 0.3f)));
    // Saturation: a grey top has saturation 0, so the base becomes grey of its own luminance.
    EXPECT(nearly(kernels::blendedColor(red, grey, 15), Kernel3(0.3f, 0.3f, 0.3f)));
    // Luminosity: the base's colour with the top's luminance. Base (0.4, 0.5, 0.6), grey 0.5 top.
    EXPECT(nearly(kernels::blendedColor(Kernel3(0.4f, 0.5f, 0.6f), grey, 17), Kernel3(0.419f, 0.519f, 0.619f), 1e-5f));
}

TEST(nonSeparableModesKeepTheirDefiningParts) {
    Kernel3 base(0.7f, 0.4f, 0.2f);
    Kernel3 top(0.1f, 0.5f, 0.9f);
    // Luminosity gives the top's luminance; Color and Hue give the base's.
    EXPECT(nearly(kernels::blendLuminance(kernels::blendedColor(base, top, 17)), kernels::blendLuminance(top), 1e-5f));
    EXPECT(nearly(kernels::blendLuminance(kernels::blendedColor(base, top, 16)), kernels::blendLuminance(base), 1e-5f));
    EXPECT(nearly(kernels::blendLuminance(kernels::blendedColor(base, top, 14)), kernels::blendLuminance(base), 1e-5f));
    EXPECT(nearly(kernels::blendLuminance(kernels::blendedColor(base, top, 15)), kernels::blendLuminance(base), 1e-5f));
    // Hue keeps the base's saturation; Saturation takes the top's.
    EXPECT(nearly(kernels::blendSaturation(kernels::blendedColor(base, top, 14)), kernels::blendSaturation(base), 1e-5f));
    EXPECT(nearly(kernels::blendSaturation(kernels::blendedColor(base, top, 15)), kernels::blendSaturation(top), 1e-5f));
    // Every mode stays within 0…1 for in-range inputs (the clip guarantees it for 14…17).
    for (int mode = 0; mode <= 17; mode += 1) {
        Kernel3 result = kernels::blendedColor(base, top, mode);
        EXPECT(result.x >= -1e-6f && result.x <= 1.000001f && result.y >= -1e-6f && result.y <= 1.000001f
            && result.z >= -1e-6f && result.z <= 1.000001f);
    }
}

TEST(blendedColorBlendsEachChannelForTheSeparableModes) {
    Kernel3 base(0.2f, 0.5f, 0.8f);
    Kernel3 top(0.6f, 0.5f, 0.1f);
    EXPECT(nearly(kernels::blendedColor(base, top, 1), Kernel3(0.12f, 0.25f, 0.08f)));
    // Multiplying by white gives the base back; so does screening with black.
    EXPECT(nearly(kernels::blendedColor(base, Kernel3(1, 1, 1), 1), base));
    EXPECT(nearly(kernels::blendedColor(base, Kernel3(0, 0, 0), 2), base));
}

TEST(compositingFollowsTheAlphaRules) {
    Kernel4 base(1, 0, 0, 1);
    Kernel4 top(0, 0, 1, 0.5f);
    // Normal, opaque base: alpha = 0.5 + 1 × 0.5 = 1; colour = 0.5 × red + 0.5 × blue.
    EXPECT(nearly(kernels::compositedWithBlend(base, top, 0, 1.0f), Kernel4(0.5f, 0, 0.5f, 1)));
    // Opacity 0 gives the base back; opacity above 1 acts as 1.
    EXPECT(nearly(kernels::compositedWithBlend(base, top, 0, 0.0f), base));
    EXPECT(nearly(kernels::compositedWithBlend(base, top, 0, 5.0f), Kernel4(0.5f, 0, 0.5f, 1)));
    // A transparent top changes nothing.
    EXPECT(nearly(kernels::compositedWithBlend(base, Kernel4(0, 1, 0, 0), 3, 1.0f), base));
    // Over a transparent base the top shows unblended, whatever the mode.
    EXPECT(nearly(kernels::compositedWithBlend(Kernel4(0, 0, 0, 0), Kernel4(0.2f, 0.4f, 0.6f, 0.5f), 1, 1.0f), Kernel4(0.2f, 0.4f, 0.6f, 0.5f)));
    // Nothing on nothing is transparent black.
    EXPECT(nearly(kernels::compositedWithBlend(Kernel4(0.3f, 0.3f, 0.3f, 0), Kernel4(0.7f, 0.7f, 0.7f, 0), 1, 1.0f), Kernel4(0, 0, 0, 0)));
    // Multiply of two opaque colours: (0.5, 0.5, 0.5) × (0.5, 1, 0) = (0.25, 0.5, 0), alpha 1.
    EXPECT(nearly(kernels::compositedWithBlend(Kernel4(0.5f, 0.5f, 0.5f, 1), Kernel4(0.5f, 1, 0, 1), 1, 1.0f), Kernel4(0.25f, 0.5f, 0, 1)));
    // Half opacity of that multiply: lerp(base, blended, 0.5) = (0.375, 0.5, 0.25).
    EXPECT(nearly(kernels::compositedWithBlend(Kernel4(0.5f, 0.5f, 0.5f, 1), Kernel4(0.5f, 1, 0, 1), 1, 0.5f), Kernel4(0.375f, 0.5f, 0.25f, 1)));
}

TEST(normalBlendingIsTheStraightAlphaOver) {
    const float alphas[] = {0.0f, 0.25f, 0.6f, 1.0f};
    for (float baseAlpha : alphas) {
        for (float topAlpha : alphas) {
            Kernel4 base(0.9f, 0.3f, 0.1f, baseAlpha);
            Kernel4 top(0.2f, 0.6f, 0.8f, topAlpha);
            EXPECT(nearly(kernels::compositedWithBlend(base, top, 0, 1.0f), kernels::straightAlphaOver(base, top), 1e-5f));
        }
    }
}

TEST(straightAlphaOverOnKnownValues) {
    // alpha = 0.5 + 1 × 0.5 = 1; colour = (0, 0, 1) × 0.5 + (1, 0, 0) × 1 × 0.5
    EXPECT(nearly(kernels::straightAlphaOver(Kernel4(1, 0, 0, 1), Kernel4(0, 0, 1, 0.5f)), Kernel4(0.5f, 0, 0.5f, 1)));
    // Two half-transparent layers: alpha 0.5 + 0.5 × 0.5 = 0.75; colour = (0.5 × top + 0.25 × base) / 0.75
    EXPECT(nearly(kernels::straightAlphaOver(Kernel4(1, 0, 0, 0.5f), Kernel4(0, 0, 1, 0.5f)), Kernel4(1.0f / 3.0f, 0, 2.0f / 3.0f, 0.75f)));
    EXPECT(nearly(kernels::straightAlphaOver(Kernel4(1, 1, 1, 0), Kernel4(1, 1, 1, 0)), Kernel4(0, 0, 0, 0)));
}
