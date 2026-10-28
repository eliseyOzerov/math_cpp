// The sRGB transfer kernel (Kernels/SRGBTransfer.h) run on the CPU: the same values as
// EditorTests/Math/Optics/SRGBTests.swift, called straight on the kernel and with double as well
// as float to show the template is not tied to one number type. The same header is compiled as
// Metal by `make kernels` (Tests/KernelChecks/SRGBTransfer.metal).
#include <cmath>

#include "../Kernels/SRGBTransfer.h"
#include "Testing.h"

using editor::kernels::SRGBTransfer;
using editor::testing::isApproximatelyEqual;

TEST(srgbKernelEndpointsAreFixed) {
    EXPECT(SRGBTransfer::encodedFromLinear(0.0f) == 0);
    EXPECT(isApproximatelyEqual(SRGBTransfer::encodedFromLinear(1.0f), 1));
    EXPECT(SRGBTransfer::linearFromEncoded(0.0f) == 0);
    EXPECT(isApproximatelyEqual(SRGBTransfer::linearFromEncoded(1.0f), 1));
}

TEST(srgbKernelMatchesTheHandWorkedValues) {
    // 1.055·0.5^(1/2.4) − 0.055 = 0.7353570 (see SRGBTests.halfTheLightEncodesToAboutSeventyFourPercent)
    EXPECT(isApproximatelyEqual(SRGBTransfer::encodedFromLinear(0.5f), 0.735357f, 1e-6f));
    // ((0.5 + 0.055)/1.055)^2.4 = 0.214041
    EXPECT(isApproximatelyEqual(SRGBTransfer::linearFromEncoded(0.5f), 0.214041f, 1e-6f));
    // The straight line: 12.92 × 0.002 = 0.02584, and back.
    EXPECT(isApproximatelyEqual(SRGBTransfer::encodedFromLinear(0.002f), 0.02584f, 1e-7f));
    EXPECT(isApproximatelyEqual(SRGBTransfer::linearFromEncoded(0.02584f), 0.002f, 1e-7f));
}

TEST(srgbKernelClampsOutOfRangeValues) {
    EXPECT(SRGBTransfer::encodedFromLinear(-0.5f) == 0);
    EXPECT(isApproximatelyEqual(SRGBTransfer::encodedFromLinear(3.0f), 1));
    EXPECT(SRGBTransfer::linearFromEncoded(-0.5f) == 0);
    EXPECT(isApproximatelyEqual(SRGBTransfer::linearFromEncoded(3.0f), 1));
}

TEST(srgbKernelConstantsAreTheStandards) {
    EXPECT(SRGBTransfer::linearThreshold<float>() == 0.0031308f);
    EXPECT(SRGBTransfer::encodedThreshold<float>() == 0.04045f);
    EXPECT(SRGBTransfer::linearSegmentSlope<float>() == 12.92f);
    // scale − offset = 1, so the power piece passes through 1 at 1.
    EXPECT(isApproximatelyEqual(
        SRGBTransfer::powerSegmentScale<float>() - SRGBTransfer::powerSegmentOffset<float>(), 1));
    EXPECT(SRGBTransfer::gamma<float>() == 2.4f);
}

TEST(srgbKernelRoundTripWorksForFloatAndDouble) {
    for (int step = 0; step <= 200; step += 1) {
        float linear = static_cast<float>(step) / 200;
        float roundTrip = SRGBTransfer::linearFromEncoded(SRGBTransfer::encodedFromLinear(linear));
        EXPECT(isApproximatelyEqual(roundTrip, linear, 1e-6f));
        double linearDouble = static_cast<double>(step) / 200;
        double roundTripDouble = SRGBTransfer::linearFromEncoded(SRGBTransfer::encodedFromLinear(linearDouble));
        EXPECT(std::fabs(roundTripDouble - linearDouble) < 1e-6);
    }
}
