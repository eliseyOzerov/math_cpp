// The first half of the optics module with values worked out by hand: light directions, Beer–Lambert
// absorption, refraction, the Fresnel equations, dispersion and the microfacet terms
// (EditorTests/Math/Optics/ LightDirectionsTests, BeerLambertTests, RefractionTests,
// DielectricFresnelTests, DispersionTests and MicrofacetTests). The Schlick tests live with the
// kernel, in KernelTests.cpp.
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include "../Kernels/SchlickFresnel.h"
#include "../Linear/Angle.h"
#include "../Color/Wavelength.h"
#include "../Numerics/Integration.h"
#include "../Optics/BeerLambert.h"
#include "../Optics/DielectricFresnel.h"
#include "../Optics/Dispersion.h"
#include "../Optics/LightDirections.h"
#include "../Optics/Microfacet.h"
#include "../Optics/Refraction.h"
#include "Testing.h"

using namespace editor;

namespace {

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float positiveInfinity = std::numeric_limits<float>::infinity();
constexpr float pi = 0x1.921fb4p1f;

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

/// Swift's `stride(from:to:by:)`: the same, stopping before `end`.
void strideTo(float start, float end, float by, const std::function<void(float)>& body) {
    for (int step = 0; start + static_cast<float>(step) * by < end; step += 1) body(start + static_cast<float>(step) * by);
}

/// A unit ray heading down toward a surface whose normal is +z, `angle` away from the
/// normal, in the x–z plane: (sin θ, 0, −cos θ).
Vector3 downwardRay(Angle angle) { return Vector3(angle.sine(), 0, -angle.cosine()); }

constexpr float airToGlass = 1 / 1.5f;
constexpr float glassToAir = 1.5f;

} // namespace

// MARK: LightDirections

TEST(sineSquaredMatchesTheHandWorkedValues) {
    // cos 1 → 1 − 1 = 0; cos 0 → 1; cos 0.6 → 1 − 0.36 = 0.64; cos −0.8 → 1 − 0.64 = 0.36
    EXPECT(sineSquaredFromCosine(1) == 0);
    EXPECT(sineSquaredFromCosine(0) == 1);
    EXPECT(isApproximatelyEqual(sineSquaredFromCosine(0.6f), 0.64f));
    EXPECT(isApproximatelyEqual(sineSquaredFromCosine(-0.8f), 0.36f));
}

TEST(sineSquaredNeverGoesNegativeForACosinePastOne) {
    // A unit vector's dot product can round to 1.0000001; the result must be 0, not −2e-7.
    EXPECT(sineSquaredFromCosine(1.0000001f) == 0);
    EXPECT(sineSquaredFromCosine(-1.5f) == 0);
}

TEST(sineSquaredAndCosineSquaredAddToOne) {
    strideThrough(0, 180, 7, [](float degrees) {
        float cosine = Angle::degrees(degrees).cosine();
        EXPECT(isApproximatelyEqual(sineSquaredFromCosine(cosine) + cosine * cosine, 1));
    });
}

TEST(tangentSquaredMatchesTheHandWorkedValues) {
    // cos 0.6 → 0.64/0.36 = 1.7777778; cos 0.70710678 → 0.5/0.5 = 1; cos 1 → 0
    EXPECT(isApproximatelyEqual(tangentSquaredFromCosine(0.6f), 1.7777778f));
    EXPECT(isApproximatelyEqual(tangentSquaredFromCosine(0.70710678f), 1, 1e-6f));
    EXPECT(tangentSquaredFromCosine(1) == 0);
    // Sign of the cosine does not matter: cos −0.6 → same 1.7777778.
    EXPECT(isApproximatelyEqual(tangentSquaredFromCosine(-0.6f), 1.7777778f));
}

TEST(tangentSquaredIsInfiniteAtTheHorizon) { EXPECT(tangentSquaredFromCosine(0) == positiveInfinity); }

TEST(tangentSquaredMatchesTheTangentOfTheAngle) {
    strideThrough(5, 80, 15, [](float degrees) {
        Angle angle = Angle::degrees(degrees);
        float expected = std::tan(angle.radians) * std::tan(angle.radians);
        EXPECT(isApproximatelyEqual(tangentSquaredFromCosine(angle.cosine()), expected, expected * 1e-5f));
    });
}

TEST(halfVectorBisectsTwoDirectionsAndIsZeroForOpposites) {
    // (0.6, 0, 0.8) + (−0.6, 0, 0.8) = (0, 0, 1.6), normalized (0, 0, 1)
    EXPECT(isApproximatelyEqual(halfVector(Vector3(0.6f, 0, 0.8f), Vector3(-0.6f, 0, 0.8f)), Vector3(0, 0, 1)));
    EXPECT(halfVector(Vector3(0, 0, 1), Vector3(0, 0, -1)) == Vector3(0, 0, 0));
    EXPECT(isApproximatelyEqual(cosineOfAngle(Vector3(0, 0, 1), Vector3(0.6f, 0, 0.8f)), 0.8f));
}

// MARK: BeerLambert

TEST(transmittanceMatchesTheHandWorkedValues) {
    // σ = (1, 2, 0.5) per unit length, thickness 1:
    // exp(−1) = 0.36787944, exp(−2) = 0.13533528, exp(−0.5) = 0.60653066
    Vector3 result = BeerLambert::transmittance(Vector3(1, 2, 0.5f), 1);
    EXPECT(isApproximatelyEqual(result, Vector3(0.36787944f, 0.13533528f, 0.60653066f), 1e-6f));
    // Twice as thick: the square of the above, 0.13533528, 0.01831564, 0.36787944.
    Vector3 doubled = BeerLambert::transmittance(Vector3(1, 2, 0.5f), 2);
    EXPECT(isApproximatelyEqual(doubled, Vector3(0.13533528f, 0.01831564f, 0.36787944f), 1e-6f));
}

TEST(noThicknessPassesEverythingAndNaNStaysNaN) {
    Vector3 sigma(1, 2, 0.5f);
    EXPECT(BeerLambert::transmittance(sigma, 0) == Vector3(1, 1, 1));
    EXPECT(BeerLambert::transmittance(sigma, -1) == Vector3(1, 1, 1));
    // A NaN thickness is not "no material": it must show up as NaN, not as clear glass.
    Vector3 unknown = BeerLambert::transmittance(sigma, notANumber);
    EXPECT(std::isnan(unknown.x) && std::isnan(unknown.y) && std::isnan(unknown.z));
    Vector3 unknownCoefficient = BeerLambert::transmittance(Vector3(notANumber, 1, 1), 1);
    EXPECT(std::isnan(unknownCoefficient.x));
}

TEST(oneOverSigmaLeavesOneOverE) {
    for (float scale : {1e-3f, 1.0f, 1000.0f}) {
        // σ = 1/scale, d = scale: the product σd is 1 at every scale (a millimetre of
        // dense dye, a kilometre of clear water), so 36.79% is left.
        Vector3 result = BeerLambert::transmittance(Vector3(1, 1, 1) / scale, scale);
        EXPECT(isApproximatelyEqual(result, Vector3::repeating(0.36787944f), 1e-6f));
    }
}

TEST(thicknessesMultiply) {
    // T(a + b) = T(a)·T(b), because exp(−σ(a + b)) = exp(−σa)·exp(−σb).
    Vector3 coefficient(0.3f, 1.2f, 4);
    std::vector<std::pair<float, float>> pairs = {{0.1f, 0.7f}, {2, 3}, {0.001f, 0.002f}, {500, 700}};
    for (auto [first, second] : pairs) {
        Vector3 scaledCoefficient = coefficient / (first + second);
        Vector3 together = BeerLambert::transmittance(scaledCoefficient, first + second);
        Vector3 apart = BeerLambert::transmittance(scaledCoefficient, first) * BeerLambert::transmittance(scaledCoefficient, second);
        EXPECT(isApproximatelyEqual(together, apart, 1e-6f));
    }
}

TEST(transmittanceFallsWithThickness) {
    Vector3 coefficient(0.5f, 1, 2);
    Vector3 previous(1, 1, 1);
    strideThrough(0.25f, 5, 0.25f, [&](float thickness) {
        Vector3 result = BeerLambert::transmittance(coefficient, thickness);
        EXPECT(result.x < previous.x && result.y < previous.y && result.z < previous.z);
        EXPECT(result.x > 0);
        previous = result;
    });
}

TEST(noMaterialOrNoAbsorptionLosesNothing) {
    EXPECT(BeerLambert::transmittance(Vector3(1, 2, 3), 0) == Vector3(1, 1, 1));
    // A negative thickness would amplify; it is treated as no thickness.
    EXPECT(BeerLambert::transmittance(Vector3(1, 2, 3), -4) == Vector3(1, 1, 1));
    EXPECT(BeerLambert::transmittance(Vector3(0, 0, 0), 10) == Vector3(1, 1, 1));
    // A negative coefficient (a material that makes light) is treated as 0.
    EXPECT(isApproximatelyEqual(BeerLambert::transmittance(Vector3(-1, 0, 1), 1), Vector3(1, 1, 0.36787944f), 1e-6f));
    EXPECT(BeerLambert::transmittance(Vector3(positiveInfinity, 1, 1), 1).x == 0);
}

TEST(absorptionCoefficientGivesTheChosenColourAtTheChosenDistance) {
    // T = (0.5, 0.25, 1) after 2 units: σ = −ln T/2 = (ln 2/2, ln 4/2, 0)
    //   = (0.34657359, 0.69314718, 0).
    Vector3 coefficient = BeerLambert::absorptionCoefficient(Vector3(0.5f, 0.25f, 1), 2);
    EXPECT(isApproximatelyEqual(coefficient, Vector3(0.34657359f, 0.69314718f, 0), 1e-6f));
    // And back: transmittance through exactly that distance is the colour we asked for.
    Vector3 colour(0.9f, 0.35f, 0.05f);
    for (float distance : {0.001f, 0.1f, 1.0f, 1000.0f}) {
        Vector3 sigma = BeerLambert::absorptionCoefficient(colour, distance);
        Vector3 back = BeerLambert::transmittance(sigma, distance);
        EXPECT(isApproximatelyEqual(back, colour, 1e-6f));
    }
}

TEST(blackAndOverbrightColoursGiveFiniteCoefficients) {
    // A channel of 0 is raised to 10⁻⁶: σ·d = −ln 10⁻⁶ = 13.815511.
    Vector3 black = BeerLambert::absorptionCoefficient(Vector3(0, -1, 1e-9f), 1);
    EXPECT(isApproximatelyEqual(black, Vector3(13.815511f, 13.815511f, 13.815511f), 1e-4f));
    // A value above 1 would need a material that makes light: no absorption instead.
    EXPECT(BeerLambert::absorptionCoefficient(Vector3(1, 2, 100), 3) == Vector3(0, 0, 0));
}

// MARK: Refraction

TEST(straightDownPassesStraightThrough) {
    auto result = Refraction::refractedDirection(Vector3(0, 0, -1), Vector3::zAxis(), airToGlass);
    EXPECT(result.has_value());
    if (result) EXPECT(isApproximatelyEqual(*result, Vector3(0, 0, -1)));
}

TEST(airToGlassAt45DegreesMatchesTheHandWorkedResult) {
    // sin θ₂ = (1/1.5)·sin 45° = 0.70710678/1.5 = 0.47140452
    // cos θ₂ = √(1 − 0.47140452²) = √(1 − 0.22222222) = √0.77777778 = 0.88191710
    auto result = Refraction::refractedDirection(downwardRay(Angle::degrees(45)), Vector3::zAxis(), airToGlass);
    EXPECT(result.has_value());
    if (result) EXPECT(isApproximatelyEqual(*result, Vector3(0.47140452f, 0, -0.88191710f)));
}

TEST(ratioOfOnePassesTheDirectionUnchanged) {
    strideThrough(0, 89, 11, [](float degrees) {
        Vector3 incoming = downwardRay(Angle::degrees(degrees));
        auto result = Refraction::refractedDirection(incoming, Vector3::zAxis(), 1);
        EXPECT(result.has_value());
        if (result) EXPECT(isApproximatelyEqual(*result, incoming));
    });
}

TEST(snellsLawHoldsOverAGridOfAnglesAndIndices) {
    // n₁ sin θ₁ = n₂ sin θ₂. With ratio η = n₁/n₂ and n₂ = 1: η sin θ₁ = sin θ₂.
    // Also checks: unit length, on the far side (z < 0), refracted ray in the plane of
    // incoming ray and normal (y stays 0).
    for (float ratio : {airToGlass, 1 / 1.33f, 1.0f, 1.2f}) {
        strideThrough(0, 85, 5, [ratio](float degrees) {
            Angle angle = Angle::degrees(degrees);
            auto result = Refraction::refractedDirection(downwardRay(angle), Vector3::zAxis(), ratio);
            if (!result) {
                // Only allowed past the critical angle.
                auto critical = Refraction::criticalAngle(ratio);
                EXPECT(critical.has_value());
                if (critical) EXPECT(angle.radians > critical->radians - 1e-4f);
                return;
            }
            float sineTransmitted = std::sqrt(result->x * result->x + result->y * result->y);
            EXPECT(isApproximatelyEqual(sineTransmitted, ratio * angle.sine(), 1e-5f));
            EXPECT(isApproximatelyEqual(result->length(), 1, 1e-5f));
            EXPECT(result->z < 0);
            EXPECT(result->y == 0);
        });
    }
}

TEST(snellsLawHoldsForATiltedNormal) {
    // Measure the angles with the cross product: |a × b| = sin(angle between a and b).
    Vector3 normal = Vector3(1, 2, 3).normalized();
    Vector3 incoming = Vector3(-0.3f, 0.1f, -1).normalized();
    float ratio = 1 / 1.5f;
    auto result = Refraction::refractedDirection(incoming, normal, ratio);
    EXPECT(result.has_value());
    if (!result) return;
    float sineIncident = incoming.cross(normal).length();
    float sineTransmitted = result->cross(normal).length();
    EXPECT(isApproximatelyEqual(sineTransmitted, ratio * sineIncident));
    EXPECT(isApproximatelyEqual(result->length(), 1));
    // Far side: it continues against the normal, like the incoming ray.
    EXPECT(result->dot(normal) < 0);
    // Coplanar with the incoming ray and the normal: the triple product is zero.
    EXPECT(isApproximatelyEqual(incoming.cross(normal).dot(*result), 0));
}

TEST(totalInternalReflectionBeginsAtTheCriticalAngle) {
    // Glass to air: sin θ_c = 1/1.5 = 0.666667, θ_c = asin(0.666667) = 41.8103°.
    auto critical = Refraction::criticalAngle(glassToAir);
    EXPECT(critical.has_value());
    if (critical) EXPECT(isApproximatelyEqual(critical->radians, Angle::degrees(41.8103149f).radians));
    // Just inside: light escapes. Just outside: totally reflected.
    EXPECT(Refraction::refractedDirection(downwardRay(Angle::degrees(41.5f)), Vector3::zAxis(), glassToAir).has_value());
    EXPECT(!Refraction::refractedDirection(downwardRay(Angle::degrees(42.1f)), Vector3::zAxis(), glassToAir).has_value());
    // 45°: sin²θ₂ = 1.5²·0.5 = 1.125 > 1.
    EXPECT(!Refraction::refractedDirection(downwardRay(Angle::degrees(45)), Vector3::zAxis(), glassToAir).has_value());
}

TEST(lightEnteringADenserMediumNeverReflectsTotally) {
    EXPECT(!Refraction::criticalAngle(airToGlass).has_value());
    EXPECT(Refraction::refractedDirection(downwardRay(Angle::degrees(89.9f)), Vector3::zAxis(), airToGlass).has_value());
}

TEST(glassToAirBendsAwayFromTheNormal) {
    // sin θ₂ = 1.5·sin 20° = 1.5·0.34202014 = 0.51303021
    auto result = Refraction::refractedDirection(downwardRay(Angle::degrees(20)), Vector3::zAxis(), glassToAir);
    EXPECT(result.has_value());
    if (result) EXPECT(isApproximatelyEqual(result->x, 0.51303021f));
}

TEST(aNormalPointingAlongTheRayIsFlipped) {
    // A ray inside a ball hits the wall; the outward normal points along the ray.
    Vector3 incoming = downwardRay(Angle::degrees(30));
    auto normalFacingRay = Refraction::refractedDirection(incoming, Vector3::zAxis(), airToGlass);
    auto normalAlongRay = Refraction::refractedDirection(incoming, -Vector3::zAxis(), airToGlass);
    EXPECT(normalFacingRay.has_value() && normalAlongRay.has_value());
    if (normalFacingRay && normalAlongRay) EXPECT(isApproximatelyEqual(*normalFacingRay, *normalAlongRay));
}

TEST(directionsBuiltFromMillimetreAndKilometreVectorsRefractTheSame) {
    // The same 30° ray, built from a 1 mm, a 1 m and a 1 km long vector before normalizing.
    // sin θ₂ = 0.5/1.5 = 1/3, cos θ₂ = √(8/9) = 0.94280904
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Vector3 incoming = Vector3(0.5f * scale, 0, -0.8660254f * scale).normalized();
        Vector3 normal = Vector3(0, 0, 2 * scale).normalized();
        auto result = Refraction::refractedDirection(incoming, normal, airToGlass);
        EXPECT(result.has_value());
        if (result) EXPECT(isApproximatelyEqual(*result, Vector3(1.0f / 3.0f, 0, -0.94280904f), 1e-5f));
    }
}

TEST(nanInputsReturnNothing) {
    EXPECT(!Refraction::refractedDirection(downwardRay(Angle::degrees(30)), Vector3::zAxis(), notANumber).has_value());
    EXPECT(!Refraction::refractedDirection(Vector3(notANumber, 0, -1), Vector3::zAxis(), airToGlass).has_value());
}

TEST(grazingRayEntersAtTheCriticalSineOfTheRatio) {
    // incoming · normal = 0 and η = 1/1.5: sin θ₂ = 1/1.5 = 0.6666667, cos θ₂ = √(5/9) = 0.74535599.
    // Which side the result lies on is arbitrary (documented); its angle from the surface
    // normal is not, so only the magnitude of the z component is checked.
    auto result = Refraction::refractedDirection(Vector3::xAxis(), Vector3::zAxis(), airToGlass);
    EXPECT(result.has_value());
    if (!result) return;
    EXPECT(isApproximatelyEqual(std::fabs(result->z), 0.74535599f, 1e-6f));
    EXPECT(isApproximatelyEqual(result->x, 0.6666667f, 1e-6f));
}

// MARK: DielectricFresnel

TEST(normalIncidenceAirToGlassReflectsFourPercent) {
    // η = 1/1.5 = 2/3, cos θᵢ = cos θₜ = 1:
    // rs = rp magnitude = (η − 1)/(η + 1) = (−1/3)/(5/3) = −0.2, and R = 0.2² = 0.04.
    FresnelReflectance result = DielectricFresnel::reflectance(1, airToGlass);
    EXPECT(isApproximatelyEqual(result.perpendicular, 0.04f));
    EXPECT(isApproximatelyEqual(result.parallel, 0.04f));
    EXPECT(isApproximatelyEqual(result.unpolarized(), 0.04f));
    EXPECT(isApproximatelyEqual(result.transmittance(), 0.96f));
}

TEST(sixtyDegreesAirToGlassMatchesTheHandWorkedValues) {
    // cos θᵢ = 0.5, η = 2/3.   sin²θₜ = η²(1 − 0.25) = (4/9)(0.75) = 1/3,
    // cos θₜ = √(2/3) = 0.81649658.
    // rs = (η cos θᵢ − cos θₜ)/(η cos θᵢ + cos θₜ) = (0.33333333 − 0.81649658)/(0.33333333 + 0.81649658)
    //    = −0.48316325/1.14982991 = −0.42020  →  Rs = 0.17657149
    // rp = (cos θᵢ − η cos θₜ)/(cos θᵢ + η cos θₜ) = (0.5 − 0.54433105)/(0.5 + 0.54433105)
    //    = −0.04433105/1.04433105 = −0.04245  →  Rp = 0.00180194
    FresnelReflectance result = DielectricFresnel::reflectance(0.5f, airToGlass);
    EXPECT(isApproximatelyEqual(result.perpendicular, 0.17657149f, 1e-6f));
    EXPECT(isApproximatelyEqual(result.parallel, 0.00180194f, 1e-6f));
    EXPECT(isApproximatelyEqual(result.unpolarized(), 0.08918671f, 1e-6f));
}

TEST(glassToAirThirtyDegreesMatchesTheHandWorkedValues) {
    // η = 1.5, cos θᵢ = 0.8660254. sin²θₜ = 2.25 × 0.25 = 0.5625, cos θₜ = √0.4375 = 0.6614378.
    // rs = (1.5×0.8660254 − 0.6614378)/(1.5×0.8660254 + 0.6614378) = (1.2990381 − 0.6614378)/(1.2990381 + 0.6614378)
    //    = 0.6376003/1.9604759 = 0.32523  →  Rs = 0.10577279
    // rp = (0.8660254 − 1.5×0.6614378)/(0.8660254 + 1.5×0.6614378) = (0.8660254 − 0.9921567)/(0.8660254 + 0.9921567)
    //    = −0.1261313/1.8581821 = −0.06788  →  Rp = 0.00460754
    FresnelReflectance result = DielectricFresnel::reflectance(Angle::degrees(30).cosine(), glassToAir);
    EXPECT(isApproximatelyEqual(result.perpendicular, 0.10577279f, 1e-6f));
    EXPECT(isApproximatelyEqual(result.parallel, 0.00460754f, 1e-6f));
}

TEST(brewstersAngleReflectsNoParallelLight) {
    // tan θ_B = 1.5, so θ_B = 56.3099° and cos θ_B = 1/√(1 + 2.25) = 0.5547002.
    // There cos θᵢ = η cos θₜ, so the numerator of rp is 0. Rp measured in Float: under 1e-8.
    Angle brewster = DielectricFresnel::brewsterAngle(airToGlass);
    EXPECT(isApproximatelyEqual(brewster.degrees(), 56.309933f, 1e-4f));
    FresnelReflectance result = DielectricFresnel::reflectance(brewster.cosine(), airToGlass);
    EXPECT(result.parallel < 1e-8f);
    // s-polarized light is still reflected there: Rs = ((n² − 1)/(n² + 1))² = (1.25/3.25)² = 0.147929.
    EXPECT(isApproximatelyEqual(result.perpendicular, 0.147929f, 1e-5f));
    // From the glass side, Brewster's angle is the one whose tangent is 1/1.5: 33.69°.
    Angle inside = DielectricFresnel::brewsterAngle(glassToAir);
    EXPECT(isApproximatelyEqual(inside.degrees(), 33.690068f, 1e-4f));
    EXPECT(DielectricFresnel::reflectance(inside.cosine(), glassToAir).parallel < 1e-8f);
}

TEST(aNegativeCosineIsClampedAndReflectsEverything) {
    // The documented trap: a cosine from the far side passed without its absolute value
    // clamps to 0, the grazing case, and gives a mirror. The absolute value gives the
    // real answer (60° air to glass: Rs 0.17737, Rp 0.00177, as in the test above).
    float unsignedCosine = Angle::degrees(60).cosine();
    EXPECT(DielectricFresnel::reflectance(-unsignedCosine, airToGlass).unpolarized() == 1);
    FresnelReflectance correct = DielectricFresnel::reflectance(std::fabs(-unsignedCosine), airToGlass);
    EXPECT(correct.unpolarized() < 0.1f);
}

TEST(grazingIncidenceReflectsEverything) {
    EXPECT(DielectricFresnel::reflectance(0, airToGlass) == (FresnelReflectance{1, 1}));
    // At cos θᵢ = 10⁻⁴ (89.994°), the reflectance is within 1 − 10⁻³.
    FresnelReflectance nearlyGrazing = DielectricFresnel::reflectance(1e-4f, airToGlass);
    EXPECT(nearlyGrazing.unpolarized() > 0.999f);
    // Exactly grazing with equal indices has no boundary at all, but must not divide 0 by 0.
    EXPECT(DielectricFresnel::reflectance(0, 1).unpolarized() == 1);
}

TEST(totalInternalReflectionBeyondTheCriticalAngle) {
    auto critical = Refraction::criticalAngle(glassToAir);
    EXPECT(critical.has_value());
    if (!critical) return;
    for (float degrees : {42.0f, 45.0f, 60.0f, 80.0f, 89.9f}) {
        FresnelReflectance result = DielectricFresnel::reflectance(Angle::degrees(degrees).cosine(), glassToAir);
        EXPECT(result == (FresnelReflectance{1, 1}));
        EXPECT(result.transmittance() == 0);
    }
    // Just inside the critical angle (41.81°) the reflectance climbs steeply to 1 (cos θₜ
    // goes as the square root of the distance to it): 0.01° below, the exact value is
    // 0.89235; and at 41° Rs = 0.53098, Rp = 0.22853 (computed with the same formulas in
    // double precision).
    FresnelReflectance nearCritical =
        DielectricFresnel::reflectance(Angle::degrees(critical->degrees() - 0.01f).cosine(), glassToAir);
    EXPECT(isApproximatelyEqual(nearCritical.unpolarized(), 0.89235f, 2e-3f));
    FresnelReflectance at41 = DielectricFresnel::reflectance(Angle::degrees(41).cosine(), glassToAir);
    EXPECT(isApproximatelyEqual(at41.perpendicular, 0.53097677f, 1e-4f));
    EXPECT(isApproximatelyEqual(at41.parallel, 0.22852576f, 1e-4f));
}

TEST(reflectanceStaysBetweenZeroAndOneAndConservesEnergy) {
    for (float ratio : {1 / 2.42f, airToGlass, 1 / 1.33f, 1.0f, 1.33f, glassToAir}) {
        strideThrough(0, 90, 1.5f, [ratio](float degrees) {
            FresnelReflectance result = DielectricFresnel::reflectance(Angle::degrees(degrees).cosine(), ratio);
            for (float value : {result.perpendicular, result.parallel, result.unpolarized()}) {
                EXPECT(value >= 0 && value <= 1);
            }
            // Reflected + transmitted = incoming: no absorption at the boundary.
            EXPECT(isApproximatelyEqual(result.unpolarized() + result.transmittance(), 1, 1e-6f));
        });
    }
}

TEST(reflectanceIsTheSameLookedAtFromTheOtherSide) {
    // Stokes: light going from glass to air at θ₁, that comes out at θ₂, reflects the
    // same share as light going from air to glass at θ₂ (the path reversed).
    strideTo(0, 41.5f, 2.5f, [](float degrees) {
        float cosineInside = Angle::degrees(degrees).cosine();
        Vector3 direction(Angle::degrees(degrees).sine(), 0, -cosineInside);
        auto refracted = Refraction::refractedDirection(direction, Vector3::zAxis(), glassToAir);
        EXPECT(refracted.has_value());
        if (!refracted) return;
        float cosineOutside = -refracted->z;
        FresnelReflectance from = DielectricFresnel::reflectance(cosineInside, glassToAir);
        FresnelReflectance back = DielectricFresnel::reflectance(cosineOutside, airToGlass);
        EXPECT(isApproximatelyEqual(from.perpendicular, back.perpendicular, 1e-5f));
        EXPECT(isApproximatelyEqual(from.parallel, back.parallel, 1e-5f));
    });
}

TEST(equalIndicesReflectNothing) {
    strideThrough(0, 85, 5, [](float degrees) {
        FresnelReflectance result = DielectricFresnel::reflectance(Angle::degrees(degrees).cosine(), 1);
        EXPECT(result.unpolarized() < 1e-12f);
    });
}

TEST(schlickApproximatesTheExactReflectanceForGlass) {
    // Air to glass, F0 = 0.04. Measured over 0…90° in steps of 0.01°: the largest
    // difference between Schlick and the unpolarized exact reflectance is 0.0357, at about
    // 85°; below 70° it is at most 0.0196. The bounds below leave a margin of about 12%.
    float normalIncidence = kernels::SchlickFresnel::reflectanceAtNormalIncidence(1.0f, 1.5f);
    float largestDifference = 0;
    float largestDifferenceBelowSeventy = 0;
    for (int hundredths = 0; hundredths < 9000; hundredths += 1) {
        Angle angle = Angle::degrees(static_cast<float>(hundredths) / 100);
        float exact = DielectricFresnel::reflectance(angle.cosine(), airToGlass).unpolarized();
        float schlick = kernels::SchlickFresnel::reflectance(angle.cosine(), normalIncidence);
        largestDifference = largerOf(largestDifference, std::fabs(exact - schlick));
        if (angle.degrees() < 70) {
            largestDifferenceBelowSeventy = largerOf(largestDifferenceBelowSeventy, std::fabs(exact - schlick));
        }
    }
    EXPECT(largestDifference < 0.040f);
    EXPECT(largestDifference > 0.030f);
    EXPECT(largestDifferenceBelowSeventy < 0.022f);
}

// MARK: Dispersion

TEST(cauchyMatchesTheHandWorkedValues) {
    CauchyDispersion glass = CauchyDispersion::borosilicateCrown();
    // λ = 0.5876 µm: λ² = 0.34527376, B/λ² = 0.0042/0.34527376 = 0.01216 → 1.5046 + 0.01216 = 1.51676
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(nanometres(587.6f)), 1.516764f, 1e-5f));
    // 400 nm: 0.0042/0.16 = 0.02625 → 1.53085
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(nanometres(400)), 1.53085f, 1e-5f));
    // 700 nm: 0.0042/0.49 = 0.0085714 → 1.5131714
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(nanometres(700)), 1.5131714f, 1e-5f));
}

TEST(nBK7MatchesTheSchottDataSheet) {
    SellmeierDispersion glass = SellmeierDispersion::schottNBK7();
    // Schott N-BK7 data sheet: n_d = 1.51680, n_F = 1.52238, n_C = 1.51432, Abbe 64.17.
    // Float measurement of this code: n_d 1.5168000, n_F 1.5223764, n_C 1.5143223, V_d 64.1658.
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(Wavelength::dLine()), 1.5168f, 1e-5f));
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(Wavelength::fLine()), 1.52238f, 1e-5f));
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(Wavelength::cLine()), 1.51432f, 1e-5f));
    EXPECT(isApproximatelyEqual(glass.abbeNumber(), 64.17f, 0.01f));
    // The catalog lists 587.6 nm as n = 1.5168 too; 656.3 nm → 1.5143.
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(nanometres(587.6f)), 1.5168f, 2e-5f));
}

TEST(nSF11MatchesTheSchottDataSheet) {
    SellmeierDispersion glass = SellmeierDispersion::schottNSF11();
    // Schott N-SF11: n_d = 1.78472, Abbe 25.68. Double-precision value of the formula:
    // n_d 1.784720, n_F 1.806513, n_C 1.775955, V_d 25.680.
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(Wavelength::dLine()), 1.78472f, 1e-5f));
    EXPECT(isApproximatelyEqual(glass.abbeNumber(), 25.68f, 0.02f));
    // Flint disperses more than crown: n_F − n_C is 0.0306 against 0.0080 for N-BK7.
    float spread = glass.refractiveIndex(Wavelength::fLine()) - glass.refractiveIndex(Wavelength::cLine());
    EXPECT(isApproximatelyEqual(spread, 0.030557f, 1e-5f));
}

TEST(fusedSilicaMatchesMalitson) {
    SellmeierDispersion glass = SellmeierDispersion::fusedSilica();
    // Malitson (1965) tabulates n = 1.45846 at 587.6 nm; 1.45042 at 1.0 µm is his formula
    // evaluated in double precision (not a tabulated value). The Abbe number is 67.8.
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(nanometres(587.6f)), 1.45846f, 2e-5f));
    EXPECT(isApproximatelyEqual(glass.refractiveIndex(Wavelength::micrometres(1)), 1.45042f, 2e-5f));
    EXPECT(isApproximatelyEqual(glass.abbeNumber(), 67.82f, 0.02f));
}

TEST(indexFallsWithWavelengthAcrossTheVisibleRange) {
    // Normal dispersion: blue bends more than red, in all three models.
    CauchyDispersion cauchy = CauchyDispersion::borosilicateCrown();
    SellmeierDispersion boroSilicate = SellmeierDispersion::schottNBK7();
    SellmeierDispersion flint = SellmeierDispersion::schottNSF11();
    SellmeierDispersion silica = SellmeierDispersion::fusedSilica();
    std::vector<const DispersiveMaterial*> materials = {&cauchy, &boroSilicate, &flint, &silica};
    for (const DispersiveMaterial* material : materials) {
        float previous = material->refractiveIndex(nanometres(380));
        strideThrough(390, 780, 10, [&](float value) {
            float index = material->refractiveIndex(nanometres(value));
            EXPECT(index < previous);
            previous = index;
        });
    }
}

TEST(cauchyAgreesWithSellmeierWhereItWasFitted) {
    // Measured over 400–700 nm: the largest difference is 1.1e-4 (in n), so the cheap
    // law is good for drawing. Bound with margin 3×.
    float largestDifference = 0;
    strideThrough(400, 700, 1, [&](float value) {
        float cauchy = CauchyDispersion::borosilicateCrown().refractiveIndex(nanometres(value));
        float sellmeier = SellmeierDispersion::schottNBK7().refractiveIndex(nanometres(value));
        largestDifference = largerOf(largestDifference, std::fabs(cauchy - sellmeier));
    });
    EXPECT(largestDifference < 3.3e-4f);
}

TEST(abbeNumberOfCauchyGlassFollowsFromItsCoefficients) {
    // n_d − 1 = 1.5046 + 0.0042/0.5875618² − 1 = 0.51677
    // n_F − n_C = B (1/0.4861327² − 1/0.6562725²) = 0.0042 × (4.2316 − 2.3219) = 0.0080204
    // V = 0.51677/0.0080204 = 64.43
    EXPECT(isApproximatelyEqual(CauchyDispersion::borosilicateCrown().abbeNumber(), 64.43f, 0.02f));
}

// MARK: Microfacet roughness

TEST(alphaIsRoughnessSquared) {
    // 0.5² = 0.25, 1² = 1
    EXPECT(isApproximatelyEqual(MicrofacetRoughness::alpha(0.5f), 0.25f));
    EXPECT(MicrofacetRoughness::alpha(1) == 1);
}

TEST(zeroRoughnessIsRaisedToTheMinimumAndOutOfRangeIsClamped) {
    EXPECT(MicrofacetRoughness::alpha(0) == MicrofacetRoughness::minimumAlpha);
    EXPECT(MicrofacetRoughness::alpha(-3) == MicrofacetRoughness::minimumAlpha);
    EXPECT(MicrofacetRoughness::alpha(7) == 1);
    EXPECT(MicrofacetRoughness::alpha(notANumber) == MicrofacetRoughness::minimumAlpha);
}

// MARK: GGX distribution

TEST(peakIsOneOverPiAlphaSquared) {
    // roughness 0.5 → α = 0.25, c = 1: α²/(π·(α²)²) = 1/(π·α²) = 1/(π·0.0625) = 5.0929582
    float peak = GGXDistribution::density(1.0f, 0.5f);
    EXPECT(isApproximatelyEqual(peak, 5.0929582f, 5.0929582f * 1e-5f));
}

TEST(fullyRoughSurfaceIsUniform) {
    // roughness 1 → α = 1: the denominator is (c²·0 + 1)² = 1, so D = 1/π for every c.
    for (float cosine : {1.0f, 0.8f, 0.5f, 0.1f}) {
        EXPECT(isApproximatelyEqual(GGXDistribution::density(cosine, 1.0f), 0.31830989f));
    }
}

TEST(offPeakValueMatchesTheHandWorkedResult) {
    // roughness √0.5 → α = 0.5, c = 0.5: c²(α² − 1) + 1 = 0.25·(−0.75) + 1 = 0.8125
    // D = 0.25/(π·0.8125²) = 0.25/(π·0.66015625) = 0.25/2.0739420 = 0.1205434
    float value = GGXDistribution::density(0.5f, 0.70710678f);
    EXPECT(isApproximatelyEqual(value, 0.1205434f, 0.1205434f * 1e-5f));
}

TEST(vectorFormMatchesTheCosineForm) {
    Vector3 normal = Vector3::zAxis();
    Vector3 half(0.6f, 0, 0.8f);
    EXPECT(GGXDistribution::density(normal, half, 0.4f) == GGXDistribution::density(0.8f, 0.4f));
}

TEST(densityFallsAsTheHalfVectorLeavesTheNormal) {
    float previous = positiveInfinity;
    strideThrough(0, 89, 1, [&](float degrees) {
        float value = GGXDistribution::density(Angle::degrees(degrees).cosine(), 0.4f);
        EXPECT(value <= previous);
        previous = value;
    });
}

TEST(belowTheHorizonIsZero) {
    EXPECT(GGXDistribution::density(0.0f, 0.5f) == 0);
    EXPECT(GGXDistribution::density(-0.3f, 0.5f) == 0);
}

TEST(zeroRoughnessStaysFinite) {
    // α is raised to 1e-4, so the peak is 1/(π·1e-8) ≈ 3.18e7, not positiveInfinity or NaN.
    float peak = GGXDistribution::density(1.0f, 0.0f);
    EXPECT(std::isfinite(peak));
    EXPECT(isApproximatelyEqual(peak, 3.1830989e7f, 3.1830989e7f * 1e-4f));
}

namespace {

/// ∫ D(h)(n·h) dω = 2π ∫₀^{π/2} D(cos θ)·cos θ·sin θ dθ, which is exactly 1 for GGX.
float projectedAreaIntegral(float roughness) {
    return GaussLegendre::integral(
               [roughness](float theta) {
                   return GGXDistribution::density(std::cos(theta), roughness) * std::cos(theta) * std::sin(theta);
               },
               0, pi / 2, 128)
        * 2 * pi;
}

} // namespace

TEST(projectedAreaIntegratesToOneOverTheHemisphere) {
    // Pieces of 1/128 of the range resolve even the narrow α = 0.09 peak. Measured error
    // with 5-point Gauss–Legendre × 128 pieces: at most 1.7e-6 for roughness 0.3–1.
    for (float roughness : {0.3f, 0.5f, 0.7f, 1.0f}) {
        EXPECT(isApproximatelyEqual(projectedAreaIntegral(roughness), 1, 1e-5f));
    }
}

TEST(projectedAreaOfVeryNarrowPeaksIsStillOne) {
    // The same integral for the narrower peaks α = 0.04 and α = 0.01. Measured error
    // with the same rule: 1.0e-5 at roughness 0.2 and 1.7e-4 at 0.1, where the peak is
    // about 0.01 rad wide and 128 pieces of 0.012 rad only just resolve it. Bounds are
    // about twice the measurement.
    EXPECT(isApproximatelyEqual(projectedAreaIntegral(0.2f), 1, 2e-5f));
    EXPECT(isApproximatelyEqual(projectedAreaIntegral(0.1f), 1, 3.5e-4f));
}

TEST(nanRoughnessActsAsTheMinimumAndNanCosineGivesZero) {
    EXPECT(GGXDistribution::density(1.0f, notANumber) == GGXDistribution::density(1.0f, 0.0f));
    EXPECT(GGXDistribution::density(notANumber, 0.5f) == 0);
}

// MARK: Smith masking and shadowing

namespace {

const float roughnessValues[] = {0, 0.2f, 0.5f, 0.8f, 1};

} // namespace

TEST(lambdaIsZeroAtNormalIncidenceForAnyRoughness) {
    for (float roughness : roughnessValues) EXPECT(SmithMasking::lambda(1, roughness) == 0);
}

TEST(lambdaMatchesTheHandWorkedValue) {
    // α = 1, θ = 45°: tan² = 1; Λ = (√(1 + 1) − 1)/2 = (1.41421356 − 1)/2 = 0.20710678
    EXPECT(isApproximatelyEqual(SmithMasking::lambda(Angle::degrees(45).cosine(), 1), 0.20710678f));
}

TEST(belowTheHorizonIsFullyHidden) {
    EXPECT(SmithMasking::lambda(0, 0.5f) == positiveInfinity);
    EXPECT(SmithMasking::singleDirectionVisibility(-0.2f, 0.5f) == 0);
    EXPECT(SmithMasking::maskingShadowing(0.5f, 0, 0.5f) == 0);
    EXPECT(SmithMasking::visibility(0.5f, 0, 0.5f) == 0);
}

TEST(singleDirectionVisibilityMatchesTheHandWorkedValue) {
    // 1/(1 + 0.20710678) = 0.82842712
    EXPECT(isApproximatelyEqual(SmithMasking::singleDirectionVisibility(Angle::degrees(45).cosine(), 1), 0.82842712f));
}

TEST(maskingShadowingMatchesTheHandWorkedValue) {
    // Both directions at 45°, α = 1: 1/(1 + 2·0.20710678) = 1/1.41421356 = 0.70710678
    float cosine = Angle::degrees(45).cosine();
    EXPECT(isApproximatelyEqual(SmithMasking::maskingShadowing(cosine, cosine, 1), 0.70710678f));
}

TEST(visibilityMatchesTheHandWorkedValue) {
    // G₂ / (4·cos·cos) = 0.70710678 / (4·0.5) = 0.35355339
    float cosine = Angle::degrees(45).cosine();
    EXPECT(isApproximatelyEqual(SmithMasking::visibility(cosine, cosine, 1), 0.35355339f));
}

TEST(visibilityStaysFiniteAndContinuousAtGrazingAngles) {
    // roughness 0.5 → α = 0.25, α² = 0.0625, viewer along the normal (cv = 1, sᵥ = 1).
    // cl → 0: sₗ → α = 0.25, so V → 0.5/(1·0.25 + cl·1) = 0.5/0.25 = 2.
    // At cl = 1e-20, cl² = 1e-40 is added to α² = 0.0625 and disappears; V = 2 to Float precision.
    float tiny = SmithMasking::visibility(1e-20f, 1, 0.5f);
    EXPECT(std::isfinite(tiny));
    EXPECT(isApproximatelyEqual(tiny, 2, 2e-6f));
    // Continuous: a cosine ten orders larger gives the same value, and the limit is not 0.
    float nearby = SmithMasking::visibility(1e-10f, 1, 0.5f);
    EXPECT(isApproximatelyEqual(nearby, tiny, 2e-6f));
}

TEST(visibilityEqualsMaskingShadowingOverFourCosines) {
    // The cancelled form must agree with G₂/(4·cl·cv) wherever that is safe to compute.
    for (float roughness : {0.2f, 0.5f, 1.0f}) {
        strideThrough(0, 80, 10, [roughness](float lightDegrees) {
            strideThrough(0, 80, 10, [roughness, lightDegrees](float viewerDegrees) {
                float lightCosine = Angle::degrees(lightDegrees).cosine();
                float viewerCosine = Angle::degrees(viewerDegrees).cosine();
                float direct = SmithMasking::maskingShadowing(lightCosine, viewerCosine, roughness)
                    / (4 * lightCosine * viewerCosine);
                float cancelled = SmithMasking::visibility(lightCosine, viewerCosine, roughness);
                EXPECT(isApproximatelyEqual(cancelled, direct, direct * 1e-5f));
            });
        });
    }
}

TEST(nanCosinesGiveZero) {
    EXPECT(SmithMasking::maskingShadowing(notANumber, 0.5f, 0.5f) == 0);
    EXPECT(SmithMasking::visibility(notANumber, 0.5f, 0.5f) == 0);
    EXPECT(SmithMasking::singleDirectionVisibility(notANumber, 0.5f) == 0);
}

TEST(bothDirectionsAlongTheNormalLoseNothing) {
    // Λ(1) = 0 for every roughness, so G₂ = 1/(1 + 0 + 0) = 1.
    for (float roughness : roughnessValues) EXPECT(SmithMasking::maskingShadowing(1, 1, roughness) == 1);
}

TEST(oneDirectionAlongTheNormalLeavesTheOthersSingleVisibility) {
    // G₂ = 1/(1 + 0 + Λ) = G₁ of the other direction.
    float cosine = Angle::degrees(70).cosine();
    EXPECT(isApproximatelyEqual(
        SmithMasking::maskingShadowing(1, cosine, 0.6f), SmithMasking::singleDirectionVisibility(cosine, 0.6f)));
}

TEST(maskingShadowingIsBetweenZeroAndOneAndSymmetric) {
    for (float roughness : roughnessValues) {
        strideThrough(0, 89, 8, [roughness](float lightDegrees) {
            strideThrough(0, 89, 8, [roughness, lightDegrees](float viewerDegrees) {
                float lightCosine = Angle::degrees(lightDegrees).cosine();
                float viewerCosine = Angle::degrees(viewerDegrees).cosine();
                float forward = SmithMasking::maskingShadowing(lightCosine, viewerCosine, roughness);
                float backward = SmithMasking::maskingShadowing(viewerCosine, lightCosine, roughness);
                EXPECT(forward > 0 && forward <= 1);
                EXPECT(forward == backward);
            });
        });
    }
}

TEST(heightCorrelatedIsAtLeastSeparableAndAtMostEitherSingleTerm) {
    for (float roughness : roughnessValues) {
        strideThrough(0, 85, 17, [roughness](float lightDegrees) {
            strideThrough(0, 85, 17, [roughness, lightDegrees](float viewerDegrees) {
                float lightCosine = Angle::degrees(lightDegrees).cosine();
                float viewerCosine = Angle::degrees(viewerDegrees).cosine();
                float lightSingle = SmithMasking::singleDirectionVisibility(lightCosine, roughness);
                float viewerSingle = SmithMasking::singleDirectionVisibility(viewerCosine, roughness);
                float correlated = SmithMasking::maskingShadowing(lightCosine, viewerCosine, roughness);
                EXPECT(correlated >= lightSingle * viewerSingle * (1 - 1e-6f));
                EXPECT(correlated <= smallerOf(lightSingle, viewerSingle) * (1 + 1e-6f));
            });
        });
    }
}

TEST(shadowingFallsTowardTheHorizon) {
    float previous = positiveInfinity;
    strideThrough(0, 89, 1, [&](float degrees) {
        float cosine = Angle::degrees(degrees).cosine();
        float value = SmithMasking::maskingShadowing(cosine, cosine, 0.6f);
        EXPECT(value <= previous);
        previous = value;
    });
}

TEST(nearlyMirrorSurfaceHidesNothing) {
    // roughness 0 → α = 1e-4: Λ ≈ α² tan²θ/4 ≈ 1e-8·(tan 80°)²/4 = 8e-8
    float cosine = Angle::degrees(80).cosine();
    EXPECT(isApproximatelyEqual(SmithMasking::maskingShadowing(cosine, cosine, 0), 1, 1e-5f));
}

TEST(cosinesFromMillimetreAndKilometreDirectionsAgree) {
    // The same 60° light direction built at three scales gives the same G₂.
    // α = 1, cos 60° = 0.5: tan² = 3, Λ = (√4 − 1)/2 = 0.5, G₂ = 1/(1 + 0.5) = 0.6666667
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Vector3 towardLight = Vector3(0.8660254f * scale, 0, 0.5f * scale).normalized();
        float value = SmithMasking::maskingShadowing(cosineOfAngle(Vector3::zAxis(), towardLight), 1, 1);
        EXPECT(isApproximatelyEqual(value, 0.6666667f, 1e-5f));
    }
}

// MARK: Cook–Torrance assembly

namespace {

float specular(Vector3 light, Vector3 viewer) {
    Vector3 half = halfVector(light, viewer);
    float cosineLight = cosineOfAngle(Vector3::zAxis(), light);
    float cosineViewer = cosineOfAngle(Vector3::zAxis(), viewer);
    return GGXDistribution::density(Vector3::zAxis(), half, 0.5f)
        * kernels::SchlickFresnel::reflectance(cosineOfAngle(viewer, half), 0.04f)
        * SmithMasking::visibility(cosineLight, cosineViewer, 0.5f);
}

} // namespace

TEST(theThreeTermsMultiplyIntoAPositiveSymmetricSpecularLobe) {
    // f = D · F · V for light at 30° and viewer at −30°: h = normal, the peak of D.
    Vector3 towardLight(Angle::degrees(30).sine(), 0, Angle::degrees(30).cosine());
    Vector3 towardViewer(-Angle::degrees(30).sine(), 0, Angle::degrees(30).cosine());
    float forward = specular(towardLight, towardViewer);
    EXPECT(forward > 0);
    EXPECT(isApproximatelyEqual(forward, specular(towardViewer, towardLight), forward * 1e-5f));
}
