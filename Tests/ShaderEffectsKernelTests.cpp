// The shader-library image effects (Kernels/ShaderEffects.h) on the CPU: the arithmetic around the texture
// reads, worked out by hand. The reads themselves stay in the Metal backend, so these tests feed the functions
// the numbers a read would have produced. The same header is compiled as Metal by `make kernels` and compared
// on the GPU by `make gpu`.
#include <cmath>

#include "../Kernels/ShaderEffects.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;
using Kernel3 = kernels::Vector3Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }
bool nearly(Kernel2 first, Kernel2 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance);
}
bool nearly(Kernel3 first, Kernel3 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance);
}
bool nearly(Kernel4 first, Kernel4 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance)
        && nearly(first.w, second.w, tolerance);
}
} // namespace

// ---- Blur -----------------------------------------------------------------------------------------

TEST(blurTapsReachOneRadiusAndWeightByBoxOrBell) {
    EXPECT(nearly(kernels::blurTapOffset(3, -3), Kernel2(1.0f, -1.0f)));
    EXPECT(nearly(kernels::blurTapOffset(0, 0), Kernel2(0.0f, 0.0f)));
    EXPECT(nearly(kernels::blurTapOffset(-3, 0), Kernel2(-1.0f, 0.0f)));
    // Box: 1 everywhere. Gaussian: exp(−2 |offset|²): 1 at the centre, e⁻² = 0.135335 at (1, 0), e⁻⁴ = 0.018316 at (1, 1).
    EXPECT(nearly(kernels::blurTapWeight(Kernel2(1.0f, 1.0f), 0), 1.0f));
    EXPECT(nearly(kernels::blurTapWeight(Kernel2(0.0f, 0.0f), 1), 1.0f));
    EXPECT(nearly(kernels::blurTapWeight(Kernel2(1.0f, 0.0f), 1), 0.135335f));
    EXPECT(nearly(kernels::blurTapWeight(Kernel2(1.0f, 1.0f), 1), 0.018316f));
    // The bell is symmetric and falls with distance, over all 49 taps.
    float total = 0.0f;
    for (int y = -3; y <= 3; y += 1) {
        for (int x = -3; x <= 3; x += 1) {
            float weight = kernels::blurTapWeight(kernels::blurTapOffset(x, y), 1);
            EXPECT(nearly(weight, kernels::blurTapWeight(kernels::blurTapOffset(-x, -y), 1), 1e-6f));
            EXPECT(weight > 0.0f && weight <= 1.0f);
            total += weight;
        }
    }
    EXPECT(total > 1.0f && total < 49.0f);
}

TEST(radialAndZoomBlurPathsAreSymmetricAboutThePixel) {
    EXPECT(nearly(kernels::blurTapFraction(0), -0.5f));
    EXPECT(nearly(kernels::blurTapFraction(15), 0.5f));
    EXPECT(nearly(kernels::blurTapFraction(5) + kernels::blurTapFraction(10), 0.0f, 1e-6f));
    Kernel2 middle(0.5f, 0.5f);
    // Radial: (1, 0.5) is (0.5, 0) from the middle; turned 45° (fraction 0.5 × 90°) it is 0.5 (cos 45°, sin 45°) = (0.353553, 0.353553).
    EXPECT(nearly(kernels::radialBlurPoint(Kernel2(1.0f, 0.5f), middle, 0.5f, 90.0f), Kernel2(0.853553f, 0.853553f)));
    EXPECT(nearly(kernels::radialBlurPoint(Kernel2(1.0f, 0.5f), middle, 0.0f, 90.0f), Kernel2(1.0f, 0.5f)));
    // The middle never moves, and the distance from it is kept.
    EXPECT(nearly(kernels::radialBlurPoint(middle, middle, 0.4f, 30.0f), middle));
    EXPECT(nearly(kernels::lengthOf(kernels::radialBlurPoint(Kernel2(0.9f, 0.2f), middle, 0.3f, 40.0f) - middle),
                  kernels::lengthOf(Kernel2(0.4f, -0.3f)), 1e-5f));
    // Zoom: fraction −0.5 reads the pixel itself; fraction 0.5 with 50% reads halfway to the middle: (1, 0.5) → (0.75, 0.5).
    EXPECT(nearly(kernels::zoomBlurPoint(Kernel2(1.0f, 0.5f), middle, -0.5f, 50.0f), Kernel2(1.0f, 0.5f)));
    EXPECT(nearly(kernels::zoomBlurPoint(Kernel2(1.0f, 0.5f), middle, 0.5f, 50.0f), Kernel2(0.75f, 0.5f)));
}

// ---- Sharpen, edges, emboss, chromatic aberration --------------------------------------------------

TEST(sharpenKeepsFlatAreasAndPushesAwayFromTheAverage) {
    // Flat: the four neighbours sum to 4 × 0.5, the average is the centre, nothing changes; the alpha is the centre's.
    EXPECT(nearly(kernels::sharpenedColor(Kernel4(0.5f, 0.5f, 0.5f, 0.7f), Kernel4(2.0f, 2.0f, 2.0f, 4.0f), 0.5f), Kernel4(0.5f, 0.5f, 0.5f, 0.7f)));
    // Centre 0.8 over neighbours averaging 0.4 (sum 1.6): 0.8 + (0.8 − 0.4) × 0.5 = 1.0. Amount 0 changes nothing.
    EXPECT(nearly(kernels::sharpenedColor(Kernel4(0.8f, 0.8f, 0.8f, 1.0f), Kernel4(1.6f, 1.6f, 1.6f, 1.6f), 0.5f), Kernel4(1.0f, 1.0f, 1.0f, 1.0f)));
    EXPECT(nearly(kernels::sharpenedColor(Kernel4(0.8f, 0.2f, 0.1f, 1.0f), Kernel4(1.6f, 1.6f, 1.6f, 1.6f), 0.0f), Kernel4(0.8f, 0.2f, 0.1f, 1.0f)));
}

TEST(sobelFindsRampsAndIgnoresFlatAreas) {
    // Luminance equal to x (columns −1, 0, 1): tl = l = bl = −1, t = b = 0, tr = r = br = 1.
    // x = (1 + 2 + 1) − (−1 − 2 − 1) = 8; y = (−1 + 0 + 1) − (−1 + 0 + 1) = 0.
    EXPECT(nearly(kernels::sobelGradient(-1.0f, 0.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 1.0f), Kernel2(8.0f, 0.0f)));
    // Luminance equal to y (rows −1, 0, 1): top row 1, middle 0, bottom −1. x = 0; y = (1 + 2 + 1) − (−1 − 2 − 1) = 8.
    EXPECT(nearly(kernels::sobelGradient(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, -1.0f, -1.0f, -1.0f), Kernel2(0.0f, 8.0f)));
    // Flat: no edge. A single bright centre (not read by Sobel) is no edge either.
    EXPECT(nearly(kernels::sobelGradient(0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f), Kernel2(0.0f, 0.0f), 1e-6f));
    // Negating the picture negates the gradient.
    EXPECT(nearly(kernels::sobelGradient(1.0f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f, 0.0f, -1.0f), Kernel2(-8.0f, 0.0f)));
}

TEST(edgeStrengthIsTheScaledGradientLengthClampedToOne) {
    EXPECT(nearly(kernels::edgeStrengthOf(Kernel2(8.0f, 0.0f), 0.05f), 0.4f));
    EXPECT(nearly(kernels::edgeStrengthOf(Kernel2(3.0f, 4.0f), 0.1f), 0.5f));
    EXPECT(nearly(kernels::edgeStrengthOf(Kernel2(8.0f, 0.0f), 1.0f), 1.0f));
    EXPECT(nearly(kernels::edgeStrengthOf(Kernel2(0.0f, 0.0f), 5.0f), 0.0f));
}

TEST(embossLooksAlongTheLightAndCentresOnGrey) {
    // Angle 0 reads one texel to the right; 90° one texel up.
    EXPECT(nearly(kernels::embossOffset(0.0f, Kernel2(0.1f, 0.2f)), Kernel2(0.1f, 0.0f)));
    EXPECT(nearly(kernels::embossOffset(90.0f, Kernel2(0.1f, 0.2f)), Kernel2(0.0f, 0.2f), 1e-6f));
    // 45° with equal texels: 0.1 × (cos 45°, sin 45°) = (0.0707107, 0.0707107).
    EXPECT(nearly(kernels::embossOffset(45.0f, Kernel2(0.1f, 0.1f)), Kernel2(0.0707107f, 0.0707107f)));
    // Flat image: mid grey. Brighter ahead by 0.5, amount 1.5: 0.5 + 0.75 = 1.25, alpha 1.
    EXPECT(nearly(kernels::embossColor(0.4f, 0.4f, 2.0f), Kernel4(0.5f, 0.5f, 0.5f, 1.0f)));
    EXPECT(nearly(kernels::embossColor(0.7f, 0.2f, 1.5f), Kernel4(1.25f, 1.25f, 1.25f, 1.0f)));
    EXPECT(nearly(kernels::embossColor(0.2f, 0.7f, 1.0f), Kernel4(0.0f, 0.0f, 0.0f, 1.0f)));
}

TEST(chromaticAberrationShiftsRedAndBlueApartAwayFromTheMiddle) {
    // (0.7, 0.5) is (0.2, 0) from the middle: × amount 5 × 0.02 = (0.02, 0). At the middle there is no shift.
    EXPECT(nearly(kernels::chromaticOffset(Kernel2(0.7f, 0.5f), 5.0f, Kernel2(0.5f, 0.5f)), Kernel2(0.02f, 0.0f)));
    EXPECT(nearly(kernels::chromaticOffset(Kernel2(0.5f, 0.5f), 5.0f, Kernel2(0.5f, 0.5f)), Kernel2(0.0f, 0.0f)));
    // Red from one tap, green and alpha from the centre, blue from the other tap.
    EXPECT(nearly(kernels::chromaticColor(Kernel4(0.1f, 0.2f, 0.3f, 0.4f), Kernel4(0.5f, 0.6f, 0.7f, 0.8f), Kernel4(0.9f, 1.0f, 1.1f, 1.2f)),
                  Kernel4(0.1f, 0.6f, 1.1f, 0.8f)));
}

// ---- Vignette and grain ---------------------------------------------------------------------------

TEST(vignetteIsClearInTheMiddleAndDarkestAtTheRadius) {
    Kernel4 color(0.8f, 0.5f, 0.25f, 0.9f);
    // At the centre the fade is 1: no change, for any intensity and radius.
    EXPECT(nearly(kernels::vignetteColor(color, Kernel2(0.0f, 0.0f), 0.8f, 0.4f, 0.7f), color));
    EXPECT(nearly(kernels::vignetteColor(color, Kernel2(0.0f, 0.0f), 0.3f, 0.1f, 1.0f), color));
    // At and beyond the radius the fade is 0: the colour × (1 − 0.7) = × 0.3; alpha stays.
    EXPECT(nearly(kernels::vignetteColor(color, Kernel2(0.8f, 0.0f), 0.8f, 0.4f, 0.7f), Kernel4(0.24f, 0.15f, 0.075f, 0.9f)));
    EXPECT(nearly(kernels::vignetteColor(color, Kernel2(2.0f, 2.0f), 0.8f, 0.4f, 0.7f), Kernel4(0.24f, 0.15f, 0.075f, 0.9f)));
    // Halfway through the soft edge (|p| = 0.6, between 0.4 and 0.8) the smooth step is 0.5: scale 1 − 0.7 / 2 = 0.65.
    EXPECT(nearly(kernels::vignetteColor(color, Kernel2(0.6f, 0.0f), 0.8f, 0.4f, 0.7f), Kernel4(0.52f, 0.325f, 0.1625f, 0.9f)));
    // Intensity 0 never darkens. The darkening only grows with distance.
    EXPECT(nearly(kernels::vignetteColor(color, Kernel2(5.0f, 0.0f), 0.8f, 0.4f, 0.0f), color));
    float previous = 1.0f;
    for (int step = 0; step <= 20; step += 1) {
        float scale = kernels::vignetteColor(Kernel4(1.0f, 1.0f, 1.0f, 1.0f), Kernel2(0.05f * static_cast<float>(step), 0.0f), 0.8f, 0.4f, 0.7f).x;
        EXPECT(scale <= previous + 1e-6f && scale >= 0.3f - 1e-6f);
        previous = scale;
    }
}

TEST(filmGrainAddsBlockyFlickeringNoiseThatBrightPixelsShowLess) {
    Kernel4 black(0.0f, 0.0f, 0.0f, 0.6f);
    Kernel4 white(1.0f, 1.0f, 1.0f, 0.6f);
    // No amount, no change.
    EXPECT(nearly(kernels::filmGrain(Kernel4(0.3f, 0.4f, 0.5f, 0.6f), Kernel2(10.0f, 20.0f), 1.0f, 0.0f, 1.0f), Kernel4(0.3f, 0.4f, 0.5f, 0.6f)));
    // Pixel (0.5, 0.5), size 1, time 0: noise = hash31(0, 0, 0) − 0.5. Black is shifted by noise × amount, white by half as much (luminance 1).
    float noise = kernels::hash31(Kernel3(0.0f, 0.0f, 0.0f)) - 0.5f;
    EXPECT(nearly(kernels::filmGrain(black, Kernel2(0.5f, 0.5f), 0.0f, 0.4f, 1.0f), Kernel4(noise * 0.4f, noise * 0.4f, noise * 0.4f, 0.6f), 1e-6f));
    EXPECT(nearly(kernels::filmGrain(white, Kernel2(0.5f, 0.5f), 0.0f, 0.4f, 1.0f).x, 1.0f + noise * 0.4f * 0.5f, 1e-6f));
    // Size 2 makes 2 × 2 blocks: (0.2, 0.3) and (1.9, 1.1) share block (0, 0); (2.1, 0.3) is block (1, 0). The time block is ⌊time × 24⌋:
    // 0.01 and 0.04 are both frame 0, 0.05 is frame 1.
    Kernel4 first = kernels::filmGrain(black, Kernel2(0.2f, 0.3f), 0.01f, 1.0f, 2.0f);
    EXPECT(nearly(first, kernels::filmGrain(black, Kernel2(1.9f, 1.1f), 0.04f, 1.0f, 2.0f), 0.0f));
    EXPECT(!nearly(first.x, kernels::filmGrain(black, Kernel2(2.1f, 0.3f), 0.01f, 1.0f, 2.0f).x, 1e-6f));
    EXPECT(!nearly(first.x, kernels::filmGrain(black, Kernel2(0.2f, 0.3f), 0.05f, 1.0f, 2.0f).x, 1e-6f));
    // Always within half the amount, and spread on both sides of zero.
    int negative = 0;
    int positive = 0;
    for (int index = 0; index < 400; index += 1) {
        float grain = kernels::filmGrain(black, Kernel2(static_cast<float>(index) + 0.5f, 3.5f), 0.0f, 1.0f, 1.0f).x;
        EXPECT(grain >= -0.5f && grain <= 0.5f);
        negative += grain < 0.0f ? 1 : 0;
        positive += grain > 0.0f ? 1 : 0;
    }
    EXPECT(negative > 120 && positive > 120);
}

// ---- CRT ------------------------------------------------------------------------------------------

TEST(crtWarpCurvesTheScreenAndIsTheIdentityWhenFlat) {
    // Flat: centred = 2 uv − 1, and back again.
    EXPECT(nearly(kernels::crtCentredWarp(Kernel2(0.5f, 0.5f), 0.0f), Kernel2(0.0f, 0.0f)));
    EXPECT(nearly(kernels::crtCentredWarp(Kernel2(1.0f, 0.25f), 0.0f), Kernel2(1.0f, -0.5f)));
    EXPECT(nearly(kernels::crtWarpedUv(kernels::crtCentredWarp(Kernel2(0.3f, 0.9f), 0.0f)), Kernel2(0.3f, 0.9f)));
    // Curvature 1: (1, 0) × (1 + 1 × 0.15) = (1.15, 0) → uv 1.075, outside. The corner (1, 1): |c|² = 2, × (1 + 0.3) = 1.3.
    EXPECT(nearly(kernels::crtCentredWarp(Kernel2(1.0f, 0.5f), 1.0f), Kernel2(1.15f, 0.0f)));
    EXPECT(nearly(kernels::crtCentredWarp(Kernel2(1.0f, 1.0f), 1.0f), Kernel2(1.3f, 1.3f)));
    EXPECT(nearly(kernels::crtWarpedUv(Kernel2(1.15f, 0.0f)), Kernel2(1.075f, 0.5f)));
    // The middle stays.
    EXPECT(nearly(kernels::crtCentredWarp(Kernel2(0.5f, 0.5f), 3.0f), Kernel2(0.0f, 0.0f)));
}

TEST(theUnitSquareTestSeesOnlyUvOutsideZeroToOne) {
    EXPECT(!kernels::isOutsideUnitSquare(Kernel2(0.0f, 0.0f)));
    EXPECT(!kernels::isOutsideUnitSquare(Kernel2(1.0f, 1.0f)));
    EXPECT(!kernels::isOutsideUnitSquare(Kernel2(0.5f, 0.5f)));
    EXPECT(kernels::isOutsideUnitSquare(Kernel2(-0.01f, 0.5f)));
    EXPECT(kernels::isOutsideUnitSquare(Kernel2(0.5f, 1.01f)));
    EXPECT(kernels::isOutsideUnitSquare(Kernel2(0.5f, -0.2f)));
    EXPECT(kernels::isOutsideUnitSquare(Kernel2(1.2f, 0.5f)));
}

TEST(crtScanlinesMaskAndCornersHaveTheirHandWorkedValues) {
    // Spacing 4: row 0 is on a bright line (cos 0 = 1, line 1), row 2 on a dark one (cos π = −1, line 0), row 1 between (line 0.5).
    EXPECT(nearly(kernels::crtScanlineFactor(0.0f, 4.0f, 0.6f), 1.0f));
    EXPECT(nearly(kernels::crtScanlineFactor(2.0f, 4.0f, 0.6f), 0.4f));
    EXPECT(nearly(kernels::crtScanlineFactor(1.0f, 4.0f, 1.0f), 0.5f, 1e-6f));
    EXPECT(nearly(kernels::crtScanlineFactor(2.0f, 4.0f, 0.0f), 1.0f));
    // Spacing under 1 is 1: a period of one row, so every whole row is bright.
    EXPECT(nearly(kernels::crtScanlineFactor(7.0f, 0.0f, 1.0f), 1.0f, 1e-5f));
    // Mask at intensity 2 (blend factor 1): the full mask by column mod 3, negative columns by floored modulo (−1 → 2).
    EXPECT(nearly(kernels::crtMask(0.4f, 2.0f), Kernel3(1.0f, 0.8f, 0.8f)));
    EXPECT(nearly(kernels::crtMask(1.7f, 2.0f), Kernel3(0.8f, 1.0f, 0.8f)));
    EXPECT(nearly(kernels::crtMask(2.2f, 2.0f), Kernel3(0.8f, 0.8f, 1.0f)));
    EXPECT(nearly(kernels::crtMask(3.0f, 2.0f), Kernel3(1.0f, 0.8f, 0.8f)));
    EXPECT(nearly(kernels::crtMask(-0.5f, 2.0f), Kernel3(0.8f, 0.8f, 1.0f)));
    // Intensity 1 blends halfway to white: (1, 0.9, 0.9). Intensity 0 is white.
    EXPECT(nearly(kernels::crtMask(0.4f, 1.0f), Kernel3(1.0f, 0.9f, 0.9f)));
    EXPECT(nearly(kernels::crtMask(1.7f, 0.0f), Kernel3(1.0f, 1.0f, 1.0f)));
    // Corners: 1 at the middle and when flat; at (1, 1) with curvature 1 it is 1 − 0.3 × 2 × 0.25 = 0.85.
    EXPECT(nearly(kernels::crtCornerDarkening(Kernel2(0.0f, 0.0f), 1.0f), 1.0f));
    EXPECT(nearly(kernels::crtCornerDarkening(Kernel2(1.0f, 1.0f), 0.0f), 1.0f));
    EXPECT(nearly(kernels::crtCornerDarkening(Kernel2(1.0f, 1.0f), 1.0f), 0.85f));
    // Together, at the centre of the screen on row 0, column 0, intensity 0.5: scanline 1, mask (1, 0.95, 0.95), corner 1.
    EXPECT(nearly(kernels::crtShadedColor(Kernel4(0.8f, 0.6f, 0.4f, 0.9f), Kernel2(0.0f, 0.0f), Kernel2(0.0f, 0.0f), 4.0f, 0.5f, 0.0f),
                  Kernel4(0.8f, 0.57f, 0.38f, 0.9f)));
    // Intensity 0 and curvature 0 leave the colour alone, whatever the pixel.
    EXPECT(nearly(kernels::crtShadedColor(Kernel4(0.8f, 0.6f, 0.4f, 0.9f), Kernel2(0.7f, -0.2f), Kernel2(13.0f, 29.0f), 3.0f, 0.0f, 0.0f),
                  Kernel4(0.8f, 0.6f, 0.4f, 0.9f)));
}

// ---- Bloom, pixelate, mosaic ----------------------------------------------------------------------

TEST(bloomTapsFormRingsAndTheCutoffKeepsOnlyTheBrightPart) {
    // Ring 1, tap 0, radius 3, texel 0.1: angle 0.4 rad: (cos 0.4, sin 0.4) × 1 × 3 × 0.1 / 3 = (0.0921061, 0.0389418).
    EXPECT(nearly(kernels::bloomRingOffset(1, 0, 3.0f, Kernel2(0.1f, 0.1f)), Kernel2(0.0921061f, 0.0389418f)));
    // Ring 3, tap 0: angle 1.2 rad: (0.362358, 0.932039) × 3 × 3 × 0.1 / 3 = × 0.3 = (0.108707, 0.279612).
    EXPECT(nearly(kernels::bloomRingOffset(3, 0, 3.0f, Kernel2(0.1f, 0.1f)), Kernel2(0.108707f, 0.279612f)));
    // Every tap of a ring is the same distance from the pixel: ring × radius × texel / 3.
    for (int ring = 1; ring <= 3; ring += 1) {
        for (int tap = 0; tap < 8; tap += 1) {
            Kernel2 offset = kernels::bloomRingOffset(ring, tap, 6.0f, Kernel2(0.01f, 0.01f));
            EXPECT(nearly(kernels::lengthOf(offset), static_cast<float>(ring) * 6.0f * 0.01f / 3.0f, 1e-6f));
        }
    }
    // Weights 1, ½, ⅓; the 24 taps weigh 8 + 4 + 8/3 = 14.6667 together.
    EXPECT(nearly(kernels::bloomRingWeight(1), 1.0f));
    EXPECT(nearly(kernels::bloomRingWeight(2), 0.5f));
    EXPECT(nearly(8.0f * (kernels::bloomRingWeight(1) + kernels::bloomRingWeight(2) + kernels::bloomRingWeight(3)), 14.666667f));
    // Bright part: max(channel − 0.4, 0) = (0.5, 0.1, 0); a cutoff above everything leaves nothing.
    EXPECT(nearly(kernels::bloomBrightPart(Kernel3(0.9f, 0.5f, 0.2f), 0.4f), Kernel3(0.5f, 0.1f, 0.0f)));
    EXPECT(nearly(kernels::bloomBrightPart(Kernel3(0.9f, 0.5f, 0.2f), 1.0f), Kernel3(0.0f, 0.0f, 0.0f)));
}

TEST(bloomAddsTheAverageGlowToTheCentre) {
    // glowSum (4, 8, 0), weightSum 3: glow = (4, 8, 0) / 4 × intensity 0.5 × 4 = (2, 4, 0).
    EXPECT(nearly(kernels::bloomCombined(Kernel4(0.2f, 0.3f, 0.4f, 0.5f), Kernel3(4.0f, 8.0f, 0.0f), 3.0f, 0.5f), Kernel4(2.2f, 4.3f, 0.4f, 0.5f)));
    // No glow, or no intensity: the centre.
    EXPECT(nearly(kernels::bloomCombined(Kernel4(0.2f, 0.3f, 0.4f, 0.5f), Kernel3(0.0f), 14.0f, 0.5f), Kernel4(0.2f, 0.3f, 0.4f, 0.5f)));
    EXPECT(nearly(kernels::bloomCombined(Kernel4(0.2f, 0.3f, 0.4f, 0.5f), Kernel3(1.0f), 14.0f, 0.0f), Kernel4(0.2f, 0.3f, 0.4f, 0.5f)));
}

TEST(pixelateReadsTheCentreOfEachBlock) {
    // 8-pixel blocks on 64 × 32: cell (0.125, 0.25). uv (0.30, 0.77) is block (⌊2.4⌋, ⌊3.08⌋) = (2, 3), centre (2.5 × 0.125, 3.5 × 0.25) = (0.3125, 0.875).
    EXPECT(nearly(kernels::pixelateTapUv(Kernel2(0.30f, 0.77f), 8.0f, Kernel2(64.0f, 32.0f)), Kernel2(0.3125f, 0.875f)));
    // Every point of a block reads the same place.
    EXPECT(nearly(kernels::pixelateTapUv(Kernel2(0.26f, 0.51f), 8.0f, Kernel2(64.0f, 32.0f)), kernels::pixelateTapUv(Kernel2(0.37f, 0.74f), 8.0f, Kernel2(64.0f, 32.0f))));
    // A block under one pixel is one pixel: (⌊0.3 × 64⌋ + 0.5) / 64 = 19.5 / 64 = 0.3046875.
    EXPECT(nearly(kernels::pixelateTapUv(Kernel2(0.3f, 0.5f), 0.0f, Kernel2(64.0f, 32.0f)), Kernel2(0.3046875f, (16.5f / 32.0f))));
}

TEST(mosaicTilesTakeOneColourAndLeaveGrout) {
    Kernel2 square(1.0f, 1.0f);
    // 4 tiles: uv (0.3, 0.6) → grid (1.2, 2.4), tile (1, 2) with centre (1.5, 2.5) / 4 = (0.375, 0.625).
    EXPECT(nearly(kernels::mosaicTileId(Kernel2(0.3f, 0.6f), square, 4.0f), Kernel2(1.0f, 2.0f)));
    EXPECT(nearly(kernels::mosaicTileCentreUv(Kernel2(1.0f, 2.0f), square, 4.0f), Kernel2(0.375f, 0.625f)));
    // Aspect scale (2, 1): twice the tiles along x. uv (0.3, 0.6) → grid (2.4, 2.4), tile (2, 2), centre (2.5 / 8, 2.5 / 4).
    EXPECT(nearly(kernels::mosaicTileId(Kernel2(0.3f, 0.6f), Kernel2(2.0f, 1.0f), 4.0f), Kernel2(2.0f, 2.0f)));
    EXPECT(nearly(kernels::mosaicTileCentreUv(Kernel2(2.0f, 2.0f), Kernel2(2.0f, 1.0f), 4.0f), Kernel2(0.3125f, 0.625f)));
    Kernel4 color(0.8f, 0.6f, 0.4f, 0.9f);
    // Inside the tile (local (−0.3, −0.1), max 0.3 < 0.4 = 0.5 − gap), no variation: the colour; alpha kept.
    EXPECT(nearly(kernels::mosaicTileColor(color, Kernel2(0.3f, 0.6f), square, 4.0f, 0.1f, 0.0f), color));
    // In the grout (grid.x = 1.95: local 0.45 > 0.4): black with the alpha kept.
    EXPECT(nearly(kernels::mosaicTileColor(color, Kernel2(0.4875f, 0.6f), square, 4.0f, 0.1f, 0.0f), Kernel4(0.0f, 0.0f, 0.0f, 0.9f)));
    // Variation scales by 1 + (hash21(tile) − 0.5) × variation.
    float scale = 1.0f + (kernels::hash21(Kernel2(1.0f, 2.0f)) - 0.5f) * 0.4f;
    EXPECT(nearly(kernels::mosaicTileColor(color, Kernel2(0.3f, 0.6f), square, 4.0f, 0.1f, 0.4f), Kernel4(0.8f * scale, 0.6f * scale, 0.4f * scale, 0.9f)));
    // Halfway across the 0.02 grout edge (local 0.39 with gap 0.1): the mask is 0.5.
    EXPECT(nearly(kernels::mosaicTileColor(color, Kernel2(1.89f / 4.0f, 0.6f), square, 4.0f, 0.1f, 0.0f).x, 0.8f * 0.5f, 1e-4f));
}

// ---- Displacement, normals, haze, lens ------------------------------------------------------------

TEST(displacementCentresOnTheMapAndTheSlopeIsACentralDifference) {
    EXPECT(nearly(kernels::displacementFromMap(Kernel2(0.5f, 0.5f)), Kernel2(0.0f, 0.0f)));
    EXPECT(nearly(kernels::displacementFromMap(Kernel2(1.0f, 0.0f)), Kernel2(0.5f, -0.5f)));
    EXPECT(nearly(kernels::displacedUv(Kernel2(0.4f, 0.6f), Kernel2(0.5f, -0.5f), 0.2f), Kernel2(0.5f, 0.5f)));
    EXPECT(nearly(kernels::displacedUv(Kernel2(0.4f, 0.6f), Kernel2(0.5f, -0.5f), 0.0f), Kernel2(0.4f, 0.6f)));
    // right − left, up − down.
    EXPECT(nearly(kernels::luminanceSlope(0.4f, 0.2f, 0.7f, 0.1f), Kernel2(0.2f, 0.6f)));
    EXPECT(nearly(kernels::luminanceSlope(0.3f, 0.3f, 0.3f, 0.3f), Kernel2(0.0f, 0.0f)));
}

TEST(normalFromHeightPacksFlatGroundAsBlueAndLightsTheSlope) {
    Kernel4 color(1.0f, 0.5f, 0.25f, 0.3f);
    // Flat: normal (0, 0, 1) → colour (0.5, 0.5, 1), alpha 1, whatever the strength.
    EXPECT(nearly(kernels::normalFromHeightColor(Kernel2(0.0f, 0.0f), 5.0f, 0, 0.0f, color), Kernel4(0.5f, 0.5f, 1.0f, 1.0f)));
    // Slope (1, 0), strength 1: normal = normalize(−1, 0, 1) = (−0.707107, 0, 0.707107) → (0.146447, 0.5, 0.853553).
    EXPECT(nearly(kernels::normalFromHeightColor(Kernel2(1.0f, 0.0f), 1.0f, 0, 0.0f, color), Kernel4(0.146447f, 0.5f, 0.853553f, 1.0f)));
    // Strength 0 is flat whatever the slope.
    EXPECT(nearly(kernels::normalFromHeightColor(Kernel2(3.0f, -2.0f), 0.0f, 0, 0.0f, color), Kernel4(0.5f, 0.5f, 1.0f, 1.0f)));
    // Lit, flat, light from angle 0: light = normalize(1, 0, 0.8) = (0.780869, 0, 0.624695); lit = 0.624695;
    // scale = 0.35 + 0.9 × 0.624695 = 0.912226; the colour times that, alpha 1.
    EXPECT(nearly(kernels::normalFromHeightColor(Kernel2(0.0f, 0.0f), 1.0f, 1, 0.0f, color), Kernel4(0.912226f, 0.456113f, 0.228056f, 1.0f), 1e-5f));
    // A slope facing away from the light is darker than one facing it: the height rising toward +x tilts the normal to −x.
    float facing = kernels::normalFromHeightColor(Kernel2(-1.0f, 0.0f), 1.0f, 1, 0.0f, color).x;
    float away = kernels::normalFromHeightColor(Kernel2(1.0f, 0.0f), 1.0f, 1, 0.0f, color).x;
    EXPECT(facing > away);
    // The unlit scale is 0.35 at least: a normal pointing right away from the light still has the ambient part.
    EXPECT(away >= 0.35f - 1e-6f);
}

TEST(hazeWobblesByAtMostTwiceTheStrengthAndLensKeepsTheMiddle) {
    // No strength, no wobble.
    EXPECT(nearly(kernels::hazeUv(Kernel2(0.3f, 0.7f), 0.0f, 6.0f, 1.0f), Kernel2(0.3f, 0.7f)));
    // At uv 0, time 0: wobble = (sin 0 + sin 0, cos 0 + cos 0) = (0, 2); strength 5 → (0, 2 × 5 × 0.01) = (0, 0.1).
    EXPECT(nearly(kernels::hazeUv(Kernel2(0.0f, 0.0f), 5.0f, 1.0f, 0.0f), Kernel2(0.0f, 0.1f)));
    for (int index = 0; index < 100; index += 1) {
        Kernel2 uv(static_cast<float>(index % 10) * 0.1f, static_cast<float>(index / 10) * 0.1f);
        Kernel2 hazy = kernels::hazeUv(uv, 3.0f, 8.0f, 0.37f * static_cast<float>(index));
        EXPECT(std::fabs(hazy.x - uv.x) <= 2.0f * 3.0f * 0.01f + 1e-6f);
        EXPECT(std::fabs(hazy.y - uv.y) <= 2.0f * 3.0f * 0.01f + 1e-6f);
    }
    // Lens: strength 0 and zoom 1 is the identity; the middle stays for any strength; (1, 0.5) → p = (1, 0), × (1 + 0.5) = 1.5 → uv 1.25.
    EXPECT(nearly(kernels::lensUv(Kernel2(0.2f, 0.9f), 0.0f, 1.0f), Kernel2(0.2f, 0.9f)));
    EXPECT(nearly(kernels::lensUv(Kernel2(0.5f, 0.5f), 0.8f, 1.7f), Kernel2(0.5f, 0.5f)));
    EXPECT(nearly(kernels::lensUv(Kernel2(1.0f, 0.5f), 0.5f, 1.0f), Kernel2(1.25f, 0.5f)));
    // Pincushion pulls inward: (1, 0.5) with strength −0.5 → p = 0.5 → uv 0.75. Zoom 2 halves p: uv 0.75 with no strength → 0.625.
    EXPECT(nearly(kernels::lensUv(Kernel2(1.0f, 0.5f), -0.5f, 1.0f), Kernel2(0.75f, 0.5f)));
    EXPECT(nearly(kernels::lensUv(Kernel2(0.75f, 0.5f), 0.0f, 2.0f), Kernel2(0.625f, 0.5f)));
    // A zoom under 0.01 is 0.01 (no division by zero): p = 0.5 / 0.01 = 50 → uv 25.5.
    EXPECT(nearly(kernels::lensUv(Kernel2(0.75f, 0.5f), 0.0f, 0.0f), Kernel2(25.5f, 0.5f), 1e-3f));
}

// ---- Kuwahara -------------------------------------------------------------------------------------

TEST(kuwaharaRadiusIsRoundedAndKeptWithinOneToFour) {
    EXPECT(kernels::kuwaharaRadius(0.4f) == 1);
    EXPECT(kernels::kuwaharaRadius(1.4f) == 1);
    EXPECT(kernels::kuwaharaRadius(2.4f) == 2);
    EXPECT(kernels::kuwaharaRadius(2.5f) == 3);
    EXPECT(kernels::kuwaharaRadius(9.0f) == 4);
    EXPECT(kernels::kuwaharaRadius(-3.0f) == 1);
}

TEST(kuwaharaQuadrantsPointToTheFourCorners) {
    EXPECT(nearly(kernels::kuwaharaQuadrantSign(0), Kernel2(1.0f, 1.0f)));
    EXPECT(nearly(kernels::kuwaharaQuadrantSign(1), Kernel2(-1.0f, 1.0f)));
    EXPECT(nearly(kernels::kuwaharaQuadrantSign(2), Kernel2(1.0f, -1.0f)));
    EXPECT(nearly(kernels::kuwaharaQuadrantSign(3), Kernel2(-1.0f, -1.0f)));
}

TEST(kuwaharaStatisticsGiveTheMeanAndTheVarianceOfAWindow) {
    // Radius 1: 2 × 2 = 4 taps. All four the colour (1, 2, 3): sum (4, 8, 12), squares 4 × 14 = 56; mean (1, 2, 3), variance 56 / 4 − 14 = 0.
    EXPECT(nearly(kernels::kuwaharaQuadrantStatistics(Kernel3(4.0f, 8.0f, 12.0f), 56.0f, 1), Kernel4(1.0f, 2.0f, 3.0f, 0.0f), 1e-5f));
    // Two taps black and two (2, 2, 2): sum (4, 4, 4), squares 2 × 12 = 24; mean (1, 1, 1), variance 24 / 4 − 3 = 3.
    EXPECT(nearly(kernels::kuwaharaQuadrantStatistics(Kernel3(4.0f, 4.0f, 4.0f), 24.0f, 1), Kernel4(1.0f, 1.0f, 1.0f, 3.0f)));
    // Radius 2: 9 taps, all grey 0.5: sum 4.5 each, squares 9 × 0.75 = 6.75; variance 6.75 / 9 − 0.75 = 0.
    EXPECT(nearly(kernels::kuwaharaQuadrantStatistics(Kernel3(4.5f, 4.5f, 4.5f), 6.75f, 2), Kernel4(0.5f, 0.5f, 0.5f, 0.0f), 1e-6f));
}

TEST(kuwaharaPicksTheQuadrantWithTheSmallestVarianceAndTheLowestOnATie) {
    Kernel4 a(0.1f, 0.1f, 0.1f, 3.0f);
    Kernel4 b(0.2f, 0.2f, 0.2f, 1.0f);
    Kernel4 c(0.3f, 0.3f, 0.3f, 2.0f);
    Kernel4 d(0.4f, 0.4f, 0.4f, 0.5f);
    EXPECT(kernels::kuwaharaBestQuadrant(a, b, c, d) == 3);
    EXPECT(kernels::kuwaharaBestQuadrant(a, b, c, a) == 1);
    EXPECT(kernels::kuwaharaBestQuadrant(d, b, c, a) == 0);
    EXPECT(kernels::kuwaharaBestQuadrant(a, c, b, a) == 2);
    // A tie keeps the lowest number: b and the copy with variance 1 both win, quadrant 1 first.
    Kernel4 tied(0.9f, 0.9f, 0.9f, 1.0f);
    EXPECT(kernels::kuwaharaBestQuadrant(a, b, tied, tied) == 1);
    EXPECT(kernels::kuwaharaBestQuadrant(tied, tied, tied, tied) == 0);
    EXPECT(nearly(kernels::kuwaharaSelectedMean(a, b, c, d), Kernel3(0.4f, 0.4f, 0.4f)));
    EXPECT(nearly(kernels::kuwaharaSelectedMean(a, b, tied, tied), Kernel3(0.2f, 0.2f, 0.2f)));
}
