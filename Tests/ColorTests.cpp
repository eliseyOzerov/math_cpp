// The colour module with values worked out by hand: sRGB, wavelengths and the colour of a pure
// spectral light (EditorTests/Math/Optics/SRGBTests.swift, WavelengthColorTests.swift and the
// wavelength part of DispersionTests.swift).
#include <algorithm>
#include <cmath>
#include <functional>

#include "../Color/SRGB.h"
#include "../Color/Wavelength.h"
#include "../Color/WavelengthColor.h"
#include "Testing.h"

using namespace editor;

namespace {

bool isApproximatelyEqual(float first, float second, float tolerance = 1e-5f) {
    return testing::isApproximatelyEqual(first, second, tolerance);
}

bool isApproximatelyEqual(Vector3 first, Vector3 second, float tolerance = 1e-5f) {
    return testing::isApproximatelyEqual(first, second, tolerance);
}

Wavelength nanometres(float value) { return Wavelength::fromNanometres(value); }

/// Swift's `stride(from:through:by:)`: start, start + by, … up to and including `end`.
void strideThrough(float start, float end, float by, const std::function<void(float)>& body) {
    for (int step = 0; start + static_cast<float>(step) * by <= end; step += 1) body(start + static_cast<float>(step) * by);
}

struct Peak {
    float nanometres;
    float value;
};

/// The wavelength in 1 nm steps over 380…780 where `channel` of the colour matching functions is
/// largest, and the value there.
Peak peakOf(float (*channel)(Vector3)) {
    Peak best{380, -1};
    strideThrough(380, 780, 1, [&](float value) {
        float response = channel(WavelengthColor::colorMatching(nanometres(value)));
        if (response > best.value) best = Peak{value, response};
    });
    return best;
}

float xOf(Vector3 vector) { return vector.x; }
float yOf(Vector3 vector) { return vector.y; }
float zOf(Vector3 vector) { return vector.z; }

} // namespace

// MARK: sRGB

TEST(endpointsAreFixed) {
    EXPECT(SRGB::encodedFromLinear(0) == 0);
    EXPECT(isApproximatelyEqual(SRGB::encodedFromLinear(1), 1));
    EXPECT(SRGB::linearFromEncoded(0) == 0);
    EXPECT(isApproximatelyEqual(SRGB::linearFromEncoded(1), 1));
}

TEST(halfTheLightEncodesToAboutSeventyFourPercent) {
    // 1.055·0.5^(1/2.4) − 0.055: 0.5^0.416667 = e^(−0.693147·0.416667) = e^(−0.288811)
    // = 0.7491535; 1.055·0.7491535 = 0.7903570; − 0.055 = 0.7353570
    EXPECT(isApproximatelyEqual(SRGB::encodedFromLinear(0.5f), 0.735357f, 1e-6f));
}

TEST(decodingHalfMatchesTheHandWorkedValue) {
    // ((0.5 + 0.055)/1.055)^2.4 = 0.526066^2.4: ln 0.526066 = −0.642328;
    // × 2.4 = −1.541587; e^(−1.541587) = 0.223130 · 0.959265 = 0.214041
    EXPECT(isApproximatelyEqual(SRGB::linearFromEncoded(0.5f), 0.214041f, 1e-6f));
}

TEST(darkValuesUseTheStraightLine) {
    // 12.92 × 0.002 = 0.02584, and back: 0.02584 / 12.92 = 0.002
    EXPECT(isApproximatelyEqual(SRGB::encodedFromLinear(0.002f), 0.02584f, 1e-7f));
    EXPECT(isApproximatelyEqual(SRGB::linearFromEncoded(0.02584f), 0.002f, 1e-7f));
}

TEST(roundTripReturnsTheStartingValue) {
    for (int step = 0; step <= 200; step += 1) {
        float linear = static_cast<float>(step) / 200;
        float roundTrip = SRGB::linearFromEncoded(SRGB::encodedFromLinear(linear));
        EXPECT(isApproximatelyEqual(roundTrip, linear, 1e-6f));
        float encodedRoundTrip = SRGB::encodedFromLinear(SRGB::linearFromEncoded(linear));
        EXPECT(isApproximatelyEqual(encodedRoundTrip, linear, 1e-6f));
    }
}

TEST(encodingIsContinuousAtItsThreshold) {
    // Line: 12.92 × 0.0031308 = 0.04045 (rounded in the standard). Curve:
    // 1.055·0.0031308^(1/2.4) − 0.055 = 0.040449... The two pieces differ by 2.9e-8
    // (measured in double precision); the bound 1e-7 is 0.25% of the value 0.04045.
    float threshold = SRGB::linearThreshold;
    float line = 12.92f * threshold;
    float curve = 1.055f * std::pow(threshold, 1 / 2.4f) - 0.055f;
    EXPECT(isApproximatelyEqual(line, curve, 1e-7f));
    float justBelow = SRGB::encodedFromLinear(threshold * 0.999f);
    float justAbove = SRGB::encodedFromLinear(threshold * 1.001f);
    EXPECT(justAbove >= justBelow);
    EXPECT((justAbove - justBelow) < 1e-4f);
}

TEST(decodingIsContinuousAtItsThreshold) {
    float threshold = SRGB::encodedThreshold;
    float line = threshold / 12.92f;
    float curve = std::pow((threshold + 0.055f) / 1.055f, 2.4f);
    // The pieces differ by 2.3e-9 (double precision); the bound 1e-8 is 0.3% of 0.0031308.
    EXPECT(isApproximatelyEqual(line, curve, 1e-8f));
    float justBelow = SRGB::linearFromEncoded(threshold * 0.999f);
    float justAbove = SRGB::linearFromEncoded(threshold * 1.001f);
    EXPECT(justAbove >= justBelow);
    EXPECT((justAbove - justBelow) < 1e-5f);
}

TEST(bothDirectionsIncreaseSteadily) {
    float previousEncoded = -1;
    float previousLinear = -1;
    for (int step = 0; step <= 1000; step += 1) {
        float value = static_cast<float>(step) / 1000;
        float encoded = SRGB::encodedFromLinear(value);
        float linear = SRGB::linearFromEncoded(value);
        EXPECT(encoded >= previousEncoded);
        EXPECT(linear >= previousLinear);
        previousEncoded = encoded;
        previousLinear = linear;
    }
}

TEST(encodingBrightensAndDecodingDarkensTheMiddle) {
    for (float value : {0.05f, 0.25f, 0.5f, 0.75f, 0.95f}) {
        EXPECT(SRGB::encodedFromLinear(value) > value);
        EXPECT(SRGB::linearFromEncoded(value) < value);
    }
}

TEST(valuesOutsideZeroToOneAreClamped) {
    EXPECT(SRGB::encodedFromLinear(-0.5f) == 0);
    EXPECT(isApproximatelyEqual(SRGB::encodedFromLinear(3), 1));
    EXPECT(SRGB::linearFromEncoded(-0.5f) == 0);
    EXPECT(isApproximatelyEqual(SRGB::linearFromEncoded(3), 1));
}

// MARK: Wavelength

TEST(wavelengthUnitsConvert) {
    EXPECT(isApproximatelyEqual(Wavelength::micrometres(0.5876f).nanometres, 587.6f, 1e-3f));
    EXPECT(isApproximatelyEqual(nanometres(587.6f).micrometres(), 0.5876f, 1e-6f));
    EXPECT(Wavelength::dLine().nanometres == 587.5618f);
    EXPECT(Wavelength::fLine().nanometres < Wavelength::dLine().nanometres);
    EXPECT(Wavelength::dLine().nanometres < Wavelength::cLine().nanometres);
    EXPECT(Wavelength::visibleRange.lower == 380 && Wavelength::visibleRange.upper == 780);
}

// MARK: WavelengthColor

TEST(colourMatchingFunctionsPeakWhereTheCIETablesDo) {
    // The CIE 1931 tables peak at x̄ = 1.0622 (600 nm), ȳ = 1 (555 nm), z̄ = 1.7826 (445 nm).
    // The Wyman fit measured here: x̄ 1.0559 at 600, ȳ 0.9982 at 554, z̄ 1.7842 at 448.
    Peak x = peakOf(xOf);
    Peak y = peakOf(yOf);
    Peak z = peakOf(zOf);
    EXPECT(isApproximatelyEqual(x.nanometres, 600, 3));
    EXPECT(isApproximatelyEqual(x.value, 1.062f, 0.01f));
    EXPECT(isApproximatelyEqual(y.nanometres, 555, 3));
    EXPECT(isApproximatelyEqual(y.value, 1, 0.005f));
    EXPECT(isApproximatelyEqual(z.nanometres, 445, 5));
    EXPECT(isApproximatelyEqual(z.value, 1.7826f, 0.01f));
}

TEST(brightnessResponseIsOneAtGreen) {
    // ȳ(555) = 0.821·exp(−½(13.8/46.9)²) + 0.286·exp(−½(24.1/31.1)²)
    //        = 0.821×0.957665 + 0.286×0.740650 = 0.786244 + 0.211826 = 0.998038
    Vector3 value = WavelengthColor::colorMatching(nanometres(555));
    EXPECT(isApproximatelyEqual(value.y, 0.998038f, 1e-5f));
    // x̄(555) = 0.517327 and z̄(555) = 0.005653, from the same formulas.
    EXPECT(isApproximatelyEqual(value.x, 0.517327f, 1e-5f));
    EXPECT(isApproximatelyEqual(value.z, 0.005653f, 1e-5f));
}

TEST(theEndsOfTheVisibleRangeAreNearlyBlack) {
    Vector3 violet = WavelengthColor::colorMatching(nanometres(380));
    Vector3 red = WavelengthColor::colorMatching(nanometres(780));
    // Hand values: z̄(380) = 0.006746 (the largest at that end), x̄(380) = 0.000199.
    EXPECT(isApproximatelyEqual(violet.z, 0.006746f, 1e-5f));
    EXPECT(violet.x < 0.001f && violet.y < 0.001f);
    EXPECT(std::max({red.x, red.y, red.z}) < 1e-5f);
}

TEST(linearRGBOfPureColoursMatchesTheHandWorkedValues) {
    // XYZ from the fit, then the IEC matrix; negative channels clamp to 0.
    // 450 nm: XYZ = (0.344, 0.033, 1.782): R 0.1745, G −0.1968 → 0, B 1.8955
    Vector3 blue = WavelengthColor::linearRGB(nanometres(450));
    EXPECT(isApproximatelyEqual(blue, Vector3(0.17452f, 0, 1.89550f), 1e-3f));
    // 550 nm: R −0.1263 → 0, G 1.4452, B −0.1694 → 0
    EXPECT(isApproximatelyEqual(WavelengthColor::linearRGB(nanometres(550)), Vector3(0, 1.44517f, 0), 1e-3f));
    // 650 nm: R 0.7529, G and B negative → 0
    EXPECT(isApproximatelyEqual(WavelengthColor::linearRGB(nanometres(650)), Vector3(0.75287f, 0, 0), 1e-3f));
    // 490 nm (cyan): red is negative → 0, green and blue about equal
    EXPECT(isApproximatelyEqual(WavelengthColor::linearRGB(nanometres(490)), Vector3(0, 0.38704f, 0.45139f), 1e-3f));
}

TEST(colourNeverHasNegativeChannelsAndFollowsTheRainbow) {
    strideThrough(380, 780, 5, [](float value) {
        Vector3 color = WavelengthColor::linearRGB(nanometres(value));
        EXPECT(color.x >= 0 && color.y >= 0 && color.z >= 0);
        Vector3 display = WavelengthColor::displayColor(nanometres(value));
        EXPECT(isApproximatelyEqual(std::max({display.x, display.y, display.z}), 1, 1e-6f));
    });
    // The dominant channel walks blue → green → red along the spectrum.
    Vector3 violetBlue = WavelengthColor::displayColor(nanometres(440));
    Vector3 green = WavelengthColor::displayColor(nanometres(540));
    Vector3 red = WavelengthColor::displayColor(nanometres(640));
    EXPECT(violetBlue.z == 1 && violetBlue.y < 0.5f);
    EXPECT(green.y == 1 && green.x < 0.5f && green.z < 0.5f);
    EXPECT(red.x == 1 && red.y < 0.5f && red.z < 0.5f);
}

TEST(theHueStaysRedPastTheEndOfTheSpectrum) {
    // The fit's own tail would turn green after 715 nm; the hue is held at 680 nm,
    // where the colour is pure red (1, 0, 0) after clamping.
    for (float value : {680.0f, 700.0f, 740.0f, 780.0f, 2000.0f}) {
        EXPECT(WavelengthColor::displayColor(nanometres(value)) == Vector3(1, 0, 0));
    }
    // And at the violet end it is held at 400 nm: R 0.005164, B 0.064646 → (0.0799, 0, 1).
    Vector3 violet = WavelengthColor::displayColor(nanometres(380));
    EXPECT(isApproximatelyEqual(violet, Vector3(0.0799f, 0, 1), 1e-3f));
}
