// The shader-library colour kernels (Kernels/ShaderColor.h) on the CPU, against values worked out
// by hand from each function's formula, and the properties each must keep (round trips, ranges,
// identities). The same header is compiled as Metal by `make kernels`.
#include <cmath>

#include "../Kernels/ShaderColor.h"
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

/// Colours spread over the cube, including the corners, greys and mid values.
std::vector<Kernel3> sampleColors() {
    std::vector<Kernel3> colors;
    const float levels[] = {0.0f, 0.2f, 0.5f, 0.8f, 1.0f};
    for (float red : levels) {
        for (float green : levels) {
            for (float blue : levels) colors.push_back(Kernel3(red, green, blue));
        }
    }
    return colors;
}
} // namespace

TEST(hsvOfThePrimariesAndGreys) {
    // Pure red is (0, 1, 1); green a third of a turn on, blue two thirds.
    EXPECT(nearly(kernels::hsvFromRGB(Kernel3(1, 0, 0)), Kernel3(0.0f, 1.0f, 1.0f)));
    EXPECT(nearly(kernels::hsvFromRGB(Kernel3(0, 1, 0)), Kernel3(1.0f / 3.0f, 1.0f, 1.0f)));
    EXPECT(nearly(kernels::hsvFromRGB(Kernel3(0, 0, 1)), Kernel3(2.0f / 3.0f, 1.0f, 1.0f)));
    // Yellow is between red and green (1/6), cyan between green and blue (1/2).
    EXPECT(nearly(kernels::hsvFromRGB(Kernel3(1, 1, 0)), Kernel3(1.0f / 6.0f, 1.0f, 1.0f)));
    EXPECT(nearly(kernels::hsvFromRGB(Kernel3(0, 1, 1)), Kernel3(0.5f, 1.0f, 1.0f)));
    // A grey has hue 0 and saturation 0; black is (0, 0, 0).
    EXPECT(nearly(kernels::hsvFromRGB(Kernel3(0.5f, 0.5f, 0.5f)), Kernel3(0.0f, 0.0f, 0.5f)));
    EXPECT(nearly(kernels::hsvFromRGB(Kernel3(0, 0, 0)), Kernel3(0.0f, 0.0f, 0.0f)));
}

TEST(rgbFromHSVOfKnownColors) {
    EXPECT(nearly(kernels::rgbFromHSV(Kernel3(0.0f, 1.0f, 1.0f)), Kernel3(1, 0, 0)));
    EXPECT(nearly(kernels::rgbFromHSV(Kernel3(1.0f / 3.0f, 1.0f, 1.0f)), Kernel3(0, 1, 0)));
    // Hue ½ is cyan (0, 1, 1); saturation ½ lerps from white: (½, 1, 1); value 0.8 scales: (0.4, 0.8, 0.8).
    EXPECT(nearly(kernels::rgbFromHSV(Kernel3(0.5f, 0.5f, 0.8f)), Kernel3(0.4f, 0.8f, 0.8f)));
    // Hue 1 is hue 0.
    EXPECT(nearly(kernels::rgbFromHSV(Kernel3(1.0f, 1.0f, 1.0f)), Kernel3(1, 0, 0)));
    // Saturation 0 is grey of the value.
    EXPECT(nearly(kernels::rgbFromHSV(Kernel3(0.7f, 0.0f, 0.3f)), Kernel3(0.3f, 0.3f, 0.3f)));
}

TEST(hsvRoundTripsAndStaysInRange) {
    for (Kernel3 color : sampleColors()) {
        Kernel3 hsv = kernels::hsvFromRGB(color);
        EXPECT(hsv.x >= 0.0f && hsv.x <= 1.0f && hsv.y >= 0.0f && hsv.y <= 1.0f && hsv.z >= 0.0f && hsv.z <= 1.0f);
        EXPECT(nearly(kernels::rgbFromHSV(hsv), color, 1e-5f));
    }
}

TEST(hslOfKnownColors) {
    EXPECT(nearly(kernels::hslFromRGB(Kernel3(1, 0, 0)), Kernel3(0.0f, 1.0f, 0.5f)));
    EXPECT(nearly(kernels::hslFromRGB(Kernel3(1, 1, 1)), Kernel3(0.0f, 0.0f, 1.0f)));
    EXPECT(nearly(kernels::hslFromRGB(Kernel3(0, 0, 0)), Kernel3(0.0f, 0.0f, 0.0f)));
    // (0.5, 0.25, 0.25): value 0.5, HSV saturation 0.5, l = 0.5 (1 − 0.25) = 0.375,
    // s = (0.5 − 0.375) / min(0.375, 0.625) = 1/3.
    EXPECT(nearly(kernels::hslFromRGB(Kernel3(0.5f, 0.25f, 0.25f)), Kernel3(0.0f, 1.0f / 3.0f, 0.375f)));
    EXPECT(nearly(kernels::rgbFromHSL(Kernel3(0.0f, 1.0f, 0.5f)), Kernel3(1, 0, 0)));
    EXPECT(nearly(kernels::rgbFromHSL(Kernel3(0.3f, 0.0f, 0.6f)), Kernel3(0.6f, 0.6f, 0.6f)));
}

TEST(hslRoundTrips) {
    for (Kernel3 color : sampleColors()) EXPECT(nearly(kernels::rgbFromHSL(kernels::hslFromRGB(color)), color, 1e-5f));
}

TEST(linearFromEncodedColorKeepsOutOfRangeValues) {
    // 0.5 encoded is ((0.5 + 0.055) / 1.055)^2.4 = 0.214041.
    EXPECT(nearly(kernels::linearFromEncodedColor(Kernel3(0.5f, 0.5f, 0.5f)), Kernel3(0.214041f, 0.214041f, 0.214041f), 1e-5f));
    // Below the threshold: 0.04 / 12.92.
    EXPECT(nearly(kernels::linearFromEncodedColor(Kernel3(0.04f, 0.0f, 1.0f)), Kernel3(0.04f / 12.92f, 0.0f, 1.0f), 1e-6f));
    // Unlike SRGBTransfer::linearFromEncoded, -0.1292 goes through the straight piece to -0.01
    // and 2 grows past 1: ((2 + 0.055) / 1.055)^2.4 = 4.9347.
    EXPECT(nearly(kernels::linearFromEncodedColor(Kernel3(-0.1292f, 0.0f, 0.0f)).x, -0.01f, 1e-6f));
    EXPECT(kernels::linearFromEncodedColor(Kernel3(2.0f, 0.0f, 0.0f)).x > 4.9f);
    EXPECT(kernels::SRGBTransfer::linearFromEncoded(-0.1292f) == 0.0f);
}

TEST(encodedSRGBFromLinearMatchesTheTransferCurve) {
    // 0.5 linear is stored as 0.7354 (SRGBTransfer's doc); 0.001 is 12.92 × 0.001.
    EXPECT(nearly(kernels::encodedSRGBFromLinear(Kernel3(0.5f, 0.001f, 1.0f)), Kernel3(0.735357f, 0.01292f, 1.0f), 1e-5f));
    // Clamped to 0…1.
    EXPECT(nearly(kernels::encodedSRGBFromLinear(Kernel3(-1.0f, 0.0f, 2.0f)), Kernel3(0.0f, 0.0f, 1.0f)));
    // A round trip with the decoding curve.
    for (Kernel3 color : sampleColors()) {
        EXPECT(nearly(kernels::encodedSRGBFromLinear(kernels::linearFromEncodedColor(color)), color, 1e-5f));
    }
}

TEST(oklabOfKnownColors) {
    // White is (1, 0, 0) and black (0, 0, 0). Linear red is (0.627955, 0.224863, 0.125846)
    // (Ottosson's published values).
    EXPECT(nearly(kernels::oklabFromLinearSRGB(Kernel3(1, 1, 1)), Kernel3(1, 0, 0), 1e-5f));
    EXPECT(nearly(kernels::oklabFromLinearSRGB(Kernel3(0, 0, 0)), Kernel3(0, 0, 0)));
    EXPECT(nearly(kernels::oklabFromLinearSRGB(Kernel3(1, 0, 0)), Kernel3(0.627955f, 0.224863f, 0.125846f), 1e-4f));
    EXPECT(nearly(kernels::linearSRGBFromOklab(Kernel3(1, 0, 0)), Kernel3(1, 1, 1), 1e-5f));
    EXPECT(nearly(kernels::encodedSRGBFromOklab(Kernel3(1, 0, 0)), Kernel3(1, 1, 1), 1e-5f));
}

TEST(oklabRoundTripsThroughLinearSRGB) {
    for (Kernel3 color : sampleColors()) {
        EXPECT(nearly(kernels::linearSRGBFromOklab(kernels::oklabFromLinearSRGB(color)), color, 2e-4f));
    }
}

TEST(oklchIsOklabInPolarForm) {
    // (L, a, b) = (0.5, 0, 0.2): chroma 0.2, angle 90° = a quarter turn.
    EXPECT(nearly(kernels::oklchFromOklab(Kernel3(0.5f, 0.0f, 0.2f)), Kernel3(0.5f, 0.2f, 0.25f)));
    // (0.5, −0.3, 0): angle 180° = ½ turn.
    EXPECT(nearly(kernels::oklchFromOklab(Kernel3(0.5f, -0.3f, 0.0f)), Kernel3(0.5f, 0.3f, 0.5f)));
    // A negative b wraps into 0…1: (0.5, 0, −0.2) is 270° = ¾ turn.
    EXPECT(nearly(kernels::oklchFromOklab(Kernel3(0.5f, 0.0f, -0.2f)), Kernel3(0.5f, 0.2f, 0.75f)));
    EXPECT(nearly(kernels::oklabFromOklch(Kernel3(0.5f, 0.2f, 0.25f)), Kernel3(0.5f, 0.0f, 0.2f)));
    for (float hue = 0.05f; hue < 1.0f; hue += 0.1f) {
        Kernel3 lab = kernels::oklabFromOklch(Kernel3(0.6f, 0.15f, hue));
        EXPECT(nearly(kernels::oklchFromOklab(lab), Kernel3(0.6f, 0.15f, hue), 1e-5f));
    }
}

TEST(okhsvToeInverseKeepsTheEnds) {
    // (0 + 0) / … = 0; (1 + 0.206) / (k3 × 1.03) with k3 = 1.206 / 1.03 = 1.
    EXPECT(kernels::okhsvToeInverse(0.0f) == 0.0f);
    EXPECT(nearly(kernels::okhsvToeInverse(1.0f), 1.0f));
    float previous = 0.0f;
    for (float value = 0.1f; value <= 1.0f; value += 0.1f) {
        float lightness = kernels::okhsvToeInverse(value);
        EXPECT(lightness > previous);
        previous = lightness;
    }
}

TEST(okhsvGivesGreysAndStaysInGamut) {
    EXPECT(nearly(kernels::encodedSRGBFromOkhsv(0.0f, 0.5f, 0.0f), Kernel3(0, 0, 0)));
    EXPECT(nearly(kernels::encodedSRGBFromOkhsv(200.0f, 0.0f, 1.0f), Kernel3(1, 1, 1), 1e-4f));
    float previousGrey = -1.0f;
    for (float value = 0.1f; value <= 1.0f; value += 0.1f) {
        // Saturation 0 is grey whatever the hue, and a lighter grey as value grows.
        Kernel3 grey = kernels::encodedSRGBFromOkhsv(123.0f, 0.0f, value);
        EXPECT(nearly(grey.x, grey.y, 1e-4f) && nearly(grey.y, grey.z, 1e-4f));
        EXPECT(grey.x > previousGrey);
        previousGrey = grey.x;
    }
    for (float hue = 0.0f; hue < 360.0f; hue += 20.0f) {
        for (float saturation = 0.0f; saturation <= 1.0f; saturation += 0.25f) {
            for (float value = 0.0f; value <= 1.0f; value += 0.25f) {
                Kernel3 color = kernels::encodedSRGBFromOkhsv(hue, saturation, value);
                EXPECT(color.x >= 0.0f && color.x <= 1.0f && color.y >= 0.0f && color.y <= 1.0f && color.z >= 0.0f && color.z <= 1.0f);
            }
        }
    }
}

TEST(okhsvFullSaturationAndValueReachesTheGamutEdge) {
    // Saturation 1, value 1 is the brightest colour of the hue: the largest channel is (nearly) 1.
    for (float hue = 0.0f; hue < 360.0f; hue += 30.0f) {
        Kernel3 color = kernels::encodedSRGBFromOkhsv(hue, 1.0f, 1.0f);
        float largest = kernels::largerOf(color.x, kernels::largerOf(color.y, color.z));
        EXPECT(nearly(largest, 1.0f, 5e-3f));
    }
}

TEST(okhsvHueWrapsAroundACircle) {
    Kernel3 reference = kernels::encodedSRGBFromOkhsv(40.0f, 0.7f, 0.8f);
    EXPECT(nearly(kernels::encodedSRGBFromOkhsv(400.0f, 0.7f, 0.8f), reference, 1e-4f));
    EXPECT(nearly(kernels::encodedSRGBFromOkhsv(-320.0f, 0.7f, 0.8f), reference, 1e-4f));
}

TEST(okhsvCuspSlopesArePositive) {
    for (float angle = 0.0f; angle < 6.28f; angle += 0.5f) {
        kernels::Vector2Of<float> slopes = kernels::okhsvCuspSlopes(kernels::Vector2Of<float>(std::cos(angle), std::sin(angle)));
        EXPECT(slopes.x > 0.0f && slopes.y > 0.0f);
    }
}

TEST(convertedColorSpaceDispatchesAndRoundTrips) {
    Kernel3 color(0.8f, 0.4f, 0.2f);
    EXPECT(nearly(kernels::convertedColorSpace(color, 0), kernels::linearFromEncodedColor(color)));
    EXPECT(nearly(kernels::convertedColorSpace(color, 1), kernels::encodedSRGBFromLinear(color)));
    EXPECT(nearly(kernels::convertedColorSpace(color, 2), kernels::hsvFromRGB(color)));
    EXPECT(nearly(kernels::convertedColorSpace(color, 4), kernels::hslFromRGB(color)));
    EXPECT(nearly(kernels::convertedColorSpace(color, 6), kernels::oklabFromLinearSRGB(kernels::linearFromEncodedColor(color))));
    // Each forward mode followed by its inverse (3, 5, 7, 9) gives the colour back.
    for (Kernel3 sample : sampleColors()) {
        for (int forward = 0; forward <= 8; forward += 2) {
            Kernel3 there = kernels::convertedColorSpace(sample, forward);
            EXPECT(nearly(kernels::convertedColorSpace(there, forward + 1), sample, 5e-4f));
        }
    }
}

TEST(hueShiftTurnsTheWheel) {
    // A third of a turn takes red to green, and a whole turn changes nothing.
    EXPECT(nearly(kernels::hueShifted(Kernel3(1, 0, 0), 1.0f / 3.0f), Kernel3(0, 1, 0)));
    EXPECT(nearly(kernels::hueShifted(Kernel3(0.8f, 0.4f, 0.2f), 1.0f), Kernel3(0.8f, 0.4f, 0.2f), 1e-5f));
    EXPECT(nearly(kernels::hueShifted(Kernel3(0.5f, 0.5f, 0.5f), 0.37f), Kernel3(0.5f, 0.5f, 0.5f)));
}

TEST(saturationAndVibranceScaleAboutGrey) {
    Kernel3 color(0.8f, 0.4f, 0.2f);
    float grey = kernels::luminance(color);
    EXPECT(nearly(kernels::saturationAdjusted(color, 0.0f), Kernel3(grey, grey, grey)));
    EXPECT(nearly(kernels::saturationAdjusted(color, 1.0f), color));
    // Halfway: grey + (c − grey) × 0.5.
    EXPECT(nearly(kernels::saturationAdjusted(color, 0.5f).x, grey + (0.8f - grey) * 0.5f));
    EXPECT(nearly(kernels::vibranceAdjusted(color, 0.0f), color));
    // A fully vivid colour (spread 1) is unchanged by any vibrance amount.
    EXPECT(nearly(kernels::vibranceAdjusted(Kernel3(1, 0, 0), 0.8f), Kernel3(1, 0, 0)));
    // A dull colour grows: spread 0.2 → factor 1 + 0.5 × 0.8 = 1.4.
    Kernel3 dull(0.5f, 0.4f, 0.3f);
    float dullGrey = kernels::luminance(dull);
    EXPECT(nearly(kernels::vibranceAdjusted(dull, 0.5f).x, dullGrey + (0.5f - dullGrey) * 1.4f));
}

TEST(grayscaleModes) {
    Kernel3 color(0.2f, 0.5f, 0.8f);
    // luminance 0.2126 × 0.2 + 0.7152 × 0.5 + 0.0722 × 0.8 = 0.04252 + 0.3576 + 0.05776 = 0.45788
    EXPECT(nearly(kernels::grayscaled(color, 0), Kernel3(0.45788f, 0.45788f, 0.45788f)));
    EXPECT(nearly(kernels::grayscaled(color, 1), Kernel3(0.5f, 0.5f, 0.5f)));
    EXPECT(nearly(kernels::grayscaled(color, 2), Kernel3(0.8f, 0.8f, 0.8f)));
    EXPECT(nearly(kernels::grayscaled(color, 3), Kernel3(0.2f, 0.2f, 0.2f)));
    EXPECT(nearly(kernels::grayscaled(color, 9), kernels::grayscaled(color, 0)));
}

TEST(levelsStretchAndBendTheMidtones) {
    Kernel3 color(0.4f, 0.4f, 0.4f);
    // Identity settings leave the colour.
    EXPECT(nearly(kernels::levelsAdjusted(color, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f), color));
    // Black point 0.2, white point 0.6: (0.4 − 0.2) / 0.4 = 0.5.
    EXPECT(nearly(kernels::levelsAdjusted(color, 0.2f, 0.6f, 1.0f, 0.0f, 1.0f), Kernel3(0.5f, 0.5f, 0.5f)));
    // Gamma 2 takes the square root: √0.5 = 0.70711.
    EXPECT(nearly(kernels::levelsAdjusted(color, 0.2f, 0.6f, 2.0f, 0.0f, 1.0f), Kernel3(0.707107f, 0.707107f, 0.707107f)));
    // Output range 0.25…0.75 puts 0.5 at 0.5 and 0 at 0.25.
    EXPECT(nearly(kernels::levelsAdjusted(color, 0.2f, 0.6f, 1.0f, 0.25f, 0.75f), Kernel3(0.5f, 0.5f, 0.5f)));
    EXPECT(nearly(kernels::levelsAdjusted(Kernel3(0.0f, 0.0f, 0.0f), 0.2f, 0.6f, 1.0f, 0.25f, 0.75f), Kernel3(0.25f, 0.25f, 0.25f)));
}

TEST(toneCurvePassesThroughItsControlPoints) {
    EXPECT(nearly(kernels::toneCurveValue(0.0f, 0.1f, 0.6f, 0.9f), 0.0f));
    EXPECT(nearly(kernels::toneCurveValue(0.25f, 0.1f, 0.6f, 0.9f), 0.1f));
    EXPECT(nearly(kernels::toneCurveValue(0.5f, 0.1f, 0.6f, 0.9f), 0.6f));
    EXPECT(nearly(kernels::toneCurveValue(0.75f, 0.1f, 0.6f, 0.9f), 0.9f));
    EXPECT(nearly(kernels::toneCurveValue(1.0f, 0.1f, 0.6f, 0.9f), 1.0f));
    // x outside 0…1 is clamped.
    EXPECT(nearly(kernels::toneCurveValue(-3.0f, 0.1f, 0.6f, 0.9f), 0.0f));
    EXPECT(nearly(kernels::toneCurveValue(7.0f, 0.1f, 0.6f, 0.9f), 1.0f));
    // Collinear points give the line x ↦ x in the two middle segments (0.25…0.75), whose four
    // control points all lie on it; the end segments repeat their end point, so they bend.
    for (float x = 0.25f; x <= 0.75f; x += 0.05f) EXPECT(nearly(kernels::toneCurveValue(x, 0.25f, 0.5f, 0.75f), x, 1e-5f));
    EXPECT(nearly(kernels::toneCurveColor(Kernel3(0.25f, 0.5f, 0.75f), 0.1f, 0.6f, 0.9f), Kernel3(0.1f, 0.6f, 0.9f)));
}

TEST(posterizeSnapsToEvenLevels) {
    // Two steps: below ½ → 0, from ½ up → 1. (0.4 × 1 + 0.5 = 0.9 → 0; 0.6 + 0.5 = 1.1 → 1)
    EXPECT(nearly(kernels::posterized(Kernel3(0.4f, 0.6f, 0.0f), 2.0f), Kernel3(0.0f, 1.0f, 0.0f)));
    // Fewer than two steps count as two.
    EXPECT(nearly(kernels::posterized(Kernel3(0.4f, 0.6f, 0.0f), 0.0f), Kernel3(0.0f, 1.0f, 0.0f)));
    // Five steps (n = 4): 0.3 → ⌊1.7⌋ / 4 = 0.25; 0.9 → ⌊4.1⌋ / 4 = 1.
    EXPECT(nearly(kernels::posterized(Kernel3(0.3f, 0.9f, 0.5f), 5.0f), Kernel3(0.25f, 1.0f, 0.5f)));
}

TEST(thresholdMaskIsAnS_CurveAroundTheCutoff) {
    EXPECT(nearly(kernels::luminanceThresholdMask(Kernel3(0, 0, 0), 0.5f, 0.0f), 0.0f));
    EXPECT(nearly(kernels::luminanceThresholdMask(Kernel3(1, 1, 1), 0.5f, 0.0f), 1.0f));
    // A grey at the cutoff is halfway through the S-curve.
    EXPECT(nearly(kernels::luminanceThresholdMask(Kernel3(0.5f, 0.5f, 0.5f), 0.5f, 0.2f), 0.5f, 1e-3f));
    // Softness 0.4 around 0.5 spans 0.3…0.7: a grey of 0.4 is t = (0.4 − 0.2999) / 0.4002 = 0.25 → 0.15625.
    EXPECT(nearly(kernels::luminanceThresholdMask(Kernel3(0.4f, 0.4f, 0.4f), 0.5f, 0.4f), 0.15625f, 2e-3f));
}

TEST(colorizeToneMapsBetweenBlackTintAndWhite) {
    Kernel3 tint(0.8f, 0.4f, 0.2f);
    EXPECT(nearly(kernels::colorized(Kernel3(0, 0, 0), tint, 1.0f), Kernel3(0, 0, 0)));
    EXPECT(nearly(kernels::colorized(Kernel3(1, 1, 1), tint, 1.0f), Kernel3(1, 1, 1), 1e-4f));
    EXPECT(nearly(kernels::colorized(Kernel3(0.5f, 0.5f, 0.5f), tint, 1.0f), tint, 1e-4f));
    // A grey of ¼ is halfway from black to the tint: tint × 0.5.
    EXPECT(nearly(kernels::colorized(Kernel3(0.25f, 0.25f, 0.25f), tint, 1.0f), Kernel3(0.4f, 0.2f, 0.1f), 1e-4f));
    // Amount 0 leaves the colour.
    EXPECT(nearly(kernels::colorized(Kernel3(0.3f, 0.6f, 0.9f), tint, 0.0f), Kernel3(0.3f, 0.6f, 0.9f)));
}

TEST(whiteBalanceScalesTheChannels) {
    // temperature 0.5, tint −0.5: (1 + 0.2, 1 + 0.2, 1 − 0.2) = (1.2, 1.2, 0.8)
    EXPECT(nearly(kernels::whiteBalanced(Kernel3(1, 1, 1), 0.5f, -0.5f), Kernel3(1.2f, 1.2f, 0.8f)));
    EXPECT(nearly(kernels::whiteBalanced(Kernel3(0.3f, 0.6f, 0.9f), 0.0f, 0.0f), Kernel3(0.3f, 0.6f, 0.9f)));
}

TEST(splitToneTintsShadowsAndHighlights) {
    Kernel3 neutral(0.5f, 0.5f, 0.5f);
    Kernel3 color(0.3f, 0.6f, 0.9f);
    // A tint of ½ changes nothing, and so does amount 0.
    EXPECT(nearly(kernels::splitToned(color, neutral, neutral, 0.5f, 1.0f), color));
    EXPECT(nearly(kernels::splitToned(color, Kernel3(1, 0, 0), Kernel3(0, 0, 1), 0.5f, 0.0f), color));
    // White: t = 1, so the highlights apply: 2 × (0.25, 0.5, 1) = (0.5, 1, 2).
    EXPECT(nearly(kernels::splitToned(Kernel3(1, 1, 1), Kernel3(1, 0, 0), Kernel3(0.25f, 0.5f, 1.0f), 0.5f, 1.0f), Kernel3(0.5f, 1.0f, 2.0f)));
    // Black: t = 0, so the shadows apply, but black stays black.
    EXPECT(nearly(kernels::splitToned(Kernel3(0, 0, 0), Kernel3(1, 0, 0), Kernel3(0, 0, 1), 0.5f, 1.0f), Kernel3(0, 0, 0)));
}

TEST(rampColorHoldsTheEndsAndBlendsBetween) {
    Kernel4 black(0, 0, 0, 1);
    Kernel4 red(1, 0, 0, 1);
    Kernel4 green(0, 1, 0, 1);
    Kernel4 white(1, 1, 1, 0);
    // Stops at 0, 0.25, 0.5, 1.
    EXPECT(nearly(kernels::rampColor(-1.0f, black, red, green, white, 0.0f, 0.25f, 0.5f, 1.0f), black));
    EXPECT(nearly(kernels::rampColor(0.125f, black, red, green, white, 0.0f, 0.25f, 0.5f, 1.0f), Kernel4(0.5f, 0, 0, 1)));
    EXPECT(nearly(kernels::rampColor(0.25f, black, red, green, white, 0.0f, 0.25f, 0.5f, 1.0f), red));
    EXPECT(nearly(kernels::rampColor(0.5f, black, red, green, white, 0.0f, 0.25f, 0.5f, 1.0f), green));
    // Halfway from green to white: (0.5, 1, 0.5, 0.5).
    EXPECT(nearly(kernels::rampColor(0.75f, black, red, green, white, 0.0f, 0.25f, 0.5f, 1.0f), Kernel4(0.5f, 1.0f, 0.5f, 0.5f)));
    EXPECT(nearly(kernels::rampColor(2.0f, black, red, green, white, 0.0f, 0.25f, 0.5f, 1.0f), white));
}

TEST(cosinePaletteIsPeriodicWithTheFrequency) {
    Kernel3 offset(0.5f, 0.5f, 0.5f);
    Kernel3 amplitude(0.5f, 0.5f, 0.5f);
    Kernel3 frequency(1.0f, 1.0f, 1.0f);
    Kernel3 phase(0.0f, 0.25f, 0.5f);
    // t = 0: cos(0) = 1, cos(π/2) = 0, cos(π) = −1 → (1, ½, 0)
    EXPECT(nearly(kernels::cosinePalette(0.0f, offset, amplitude, frequency, phase), Kernel3(1.0f, 0.5f, 0.0f)));
    // Frequency 1 repeats every whole t.
    EXPECT(nearly(kernels::cosinePalette(1.0f, offset, amplitude, frequency, phase), Kernel3(1.0f, 0.5f, 0.0f), 1e-4f));
    EXPECT(nearly(kernels::cosinePalette(0.3f, offset, amplitude, frequency, phase),
        kernels::cosinePalette(1.3f, offset, amplitude, frequency, phase), 1e-4f));
}
