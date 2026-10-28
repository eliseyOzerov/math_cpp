// The shader-library ray marching (Kernels/Raymarching.h) on the CPU, against scenes whose answers
// are worked out by hand: a unit sphere at the origin (a ray from (0, 0, -3) along +z hits it at
// distance 2, at (0, 0, -1), where the normal is (0, 0, -1)), a floor, and constant fields. The
// scenes are C++ lambdas here; Metal's compiler has no lambdas, so the kernels' Metal check and the
// GPU agreement cases use a struct with an operator() instead. The same header is compiled as Metal
// by `make kernels` and compared on the GPU by `make gpu`.
#include <cmath>

#include "../Kernels/Raymarching.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;
using Kernel3 = kernels::Vector3Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }
bool nearly(Kernel3 first, Kernel3 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance);
}

/// The distance to the unit sphere at the origin: |p| − 1.
float unitSphere(Kernel3 p) { return kernels::lengthOf(p) - 1.0f; }
} // namespace

TEST(sphereTraceHitsTheUnitSphereAtDistanceTwo) {
    auto sphere = [](Kernel3 p) { return unitSphere(p); };
    kernels::SphereTraceResult result = kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(0, 0, 1), 20.0f, 64.0f, 0.001f);
    EXPECT(result.hit);
    EXPECT(nearly(result.travel, 2.0f));
    EXPECT(nearly(result.position, Kernel3(0, 0, -1)));
    // The first step (distance 2) lands on the surface, so one step of 64 was spent before the hit test passed.
    EXPECT(nearly(result.stepsUsed, 1.0f / 64.0f));
    // The direction need not be a unit vector, and a zero direction counts as +z.
    EXPECT(nearly(kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(0, 0, 5), 20.0f, 64.0f, 0.001f).travel, 2.0f));
    EXPECT(nearly(kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(0, 0, 0), 20.0f, 64.0f, 0.001f).travel, 2.0f));
}

TEST(sphereTraceFindsTheSurfaceOfAnOffCentreRay) {
    auto sphere = [](Kernel3 p) { return unitSphere(p); };
    // At x = 0.5 the sphere's near surface is at z = −√(1 − 0.25) = −0.8660254, travel 3 − 0.8660254 = 2.1339746.
    // The hit test accepts a distance under 0.001 (1 + 0.2 t) ≈ 0.0014, so the answer is within about that.
    kernels::SphereTraceResult result = kernels::sphereTrace(sphere, Kernel3(0.5f, 0, -3), Kernel3(0, 0, 1), 20.0f, 64.0f, 0.001f);
    EXPECT(result.hit);
    EXPECT(nearly(result.travel, 2.1339746f, 3e-3f));
    EXPECT(std::fabs(kernels::lengthOf(result.position) - 1.0f) < 2e-3f);
}

TEST(sphereTraceMissesReportTheLimitAndTheOrigin) {
    auto sphere = [](Kernel3 p) { return unitSphere(p); };
    // A ray passing the sphere 3 away never gets near it: travel is the distance limit, the position the origin.
    kernels::SphereTraceResult miss = kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(1, 0, 0), 20.0f, 64.0f, 0.001f);
    EXPECT(!miss.hit);
    EXPECT(miss.travel == 20.0f);
    EXPECT(nearly(miss.position, Kernel3(0, 0, -3)));
    EXPECT(miss.stepsUsed == 1.0f);
    // A single step is not enough to reach a surface 2 away (the second step would be the hit test).
    EXPECT(!kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(0, 0, 1), 20.0f, 1.0f, 0.001f).hit);
    // A limit that stops the ray short: 1.5 < 2.
    EXPECT(!kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(0, 0, 1), 1.5f, 64.0f, 0.001f).hit);
    // A ray starting inside the surface hits at once at t = 0, since the distance is negative.
    kernels::SphereTraceResult inside = kernels::sphereTrace(sphere, Kernel3(0, 0, 0), Kernel3(0, 0, 1), 20.0f, 64.0f, 0.001f);
    EXPECT(inside.hit && inside.travel == 0.0f && inside.stepsUsed == 0.0f);
}

TEST(sphereTraceWorksWithAFloorAndACapturingLambda) {
    // A floor at y = −1 (distance p.y + 1), seen from (0, 2, 0) straight down: hit at distance 3.
    auto floorScene = [](Kernel3 p) { return p.y + 1.0f; };
    kernels::SphereTraceResult down = kernels::sphereTrace(floorScene, Kernel3(0, 2, 0), Kernel3(0, -1, 0), 20.0f, 64.0f, 0.001f);
    EXPECT(down.hit && nearly(down.travel, 3.0f));
    // A sphere of radius 0.5 centred at (0, 0, 2), captured by value.
    Kernel3 centre(0, 0, 2);
    auto small = [centre](Kernel3 p) { return kernels::lengthOf(p - centre) - 0.5f; };
    kernels::SphereTraceResult result = kernels::sphereTrace(small, Kernel3(0, 0, 0), Kernel3(0, 0, 1), 20.0f, 64.0f, 0.001f);
    EXPECT(result.hit && nearly(result.travel, 1.5f, 2e-3f));
}

TEST(surfaceNormalPointsOutOfTheSurface) {
    auto sphere = [](Kernel3 p) { return unitSphere(p); };
    EXPECT(nearly(kernels::surfaceNormal(sphere, Kernel3(0, 0, -1), 0.001f), Kernel3(0, 0, -1), 1e-4f));
    EXPECT(nearly(kernels::surfaceNormal(sphere, Kernel3(1, 0, 0), 0.001f), Kernel3(1, 0, 0), 1e-4f));
    // On the diagonal the normal is (1, 1, 1)/√3 = 0.57735027 per component.
    float diagonal = 0.57735027f;
    EXPECT(nearly(kernels::surfaceNormal(sphere, Kernel3(diagonal, diagonal, diagonal), 0.001f), Kernel3(diagonal, diagonal, diagonal), 1e-3f));
    // The normal of a floor is +y; a constant field has no gradient, which gives +y too (no NaN).
    auto floorScene = [](Kernel3 p) { return p.y + 1.0f; };
    EXPECT(nearly(kernels::surfaceNormal(floorScene, Kernel3(3, -1, 2), 0.001f), Kernel3(0, 1, 0), 1e-4f));
    auto constant = [](Kernel3) { return 5.0f; };
    EXPECT(nearly(kernels::surfaceNormal(constant, Kernel3(0, 0, 0), 0.001f), Kernel3(0, 1, 0), 1e-4f));
    // Unit length, at points all over the sphere.
    for (int index = 0; index < 50; index += 1) {
        float angle = static_cast<float>(index) * 0.37f;
        Kernel3 point(std::cos(angle), std::sin(angle) * 0.6f, std::sin(angle) * 0.8f);
        EXPECT(nearly(kernels::lengthOf(kernels::surfaceNormal(sphere, point, 0.001f)), 1.0f, 1e-5f));
    }
}

TEST(shadedSurfaceBlendsAmbientAndDiffuse) {
    auto sphere = [](Kernel3 p) { return unitSphere(p); };
    kernels::SphereTraceResult hit = kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(0, 0, 1), 20.0f, 64.0f, 0.001f);
    Kernel4 albedo(0.8f, 0.4f, 0.2f, 0.5f);
    Kernel4 background(0.1f, 0.2f, 0.3f, 1.0f);
    // The light towards the camera (0, 0, -1) is aligned with the normal: diffuse 1 → the albedo.
    Kernel4 lit = kernels::shadedSurface(sphere, hit, albedo, Kernel3(0, 0, -1), 0.25f, background, 0.001f);
    EXPECT(nearly(lit.x, 0.8f, 1e-4f) && nearly(lit.y, 0.4f, 1e-4f) && nearly(lit.z, 0.2f, 1e-4f) && lit.w == 0.5f);
    // A light behind the sphere: diffuse 0 → albedo × ambient.
    Kernel4 dark = kernels::shadedSurface(sphere, hit, albedo, Kernel3(0, 0, 1), 0.25f, background, 0.001f);
    EXPECT(nearly(dark.x, 0.2f, 1e-4f) && nearly(dark.y, 0.1f, 1e-4f) && nearly(dark.z, 0.05f, 1e-4f));
    // Light at 60° to the normal: diffuse 0.5 → ambient + (1 − ambient) 0.5 = 0.25 + 0.375 = 0.625.
    Kernel4 slanted = kernels::shadedSurface(sphere, hit, albedo, Kernel3(0.8660254f, 0, -0.5f), 0.25f, background, 0.001f);
    EXPECT(nearly(slanted.x, 0.8f * 0.625f, 1e-4f));
    // A miss gives the background.
    kernels::SphereTraceResult miss = kernels::sphereTrace(sphere, Kernel3(0, 0, -3), Kernel3(1, 0, 0), 20.0f, 64.0f, 0.001f);
    Kernel4 missed = kernels::shadedSurface(sphere, miss, albedo, Kernel3(0, 0, -1), 0.25f, background, 0.001f);
    EXPECT(missed.x == 0.1f && missed.y == 0.2f && missed.z == 0.3f && missed.w == 1.0f);
}

TEST(lambertIsTheCosineOfTheAngleToTheLight) {
    EXPECT(nearly(kernels::lambertDiffuse(Kernel3(0, 0, 1), Kernel3(0, 0, 1)), 1.0f));
    EXPECT(nearly(kernels::lambertDiffuse(Kernel3(0, 0, 5), Kernel3(0, 0, 2)), 1.0f));
    EXPECT(nearly(kernels::lambertDiffuse(Kernel3(0, 0, 1), Kernel3(1, 0, 0)), 0.0f));
    EXPECT(kernels::lambertDiffuse(Kernel3(0, 0, 1), Kernel3(0, 0, -1)) == 0.0f);
    EXPECT(nearly(kernels::lambertDiffuse(Kernel3(0, 0, 1), Kernel3(0, 1, 1)), 0.70710678f));
}

TEST(phongAndBlinnPhongPeakWhenTheHighlightFacesTheViewer) {
    Kernel3 up(0, 1, 0);
    Kernel3 viewDirection(0, -1, 0);
    // Light, normal and viewer aligned: the reflection returns straight to the viewer → 1.
    EXPECT(nearly(kernels::phongSpecular(up, viewDirection, up, 16.0f), 1.0f));
    EXPECT(nearly(kernels::blinnPhongSpecular(up, viewDirection, up, 16.0f), 1.0f));
    // Light at 45°: the Phong reflection is at 45° to the viewer, cos 45° = 0.70710678, squared 0.5.
    Kernel3 slanted(1, 1, 0);
    EXPECT(nearly(kernels::phongSpecular(up, viewDirection, slanted, 2.0f), 0.5f));
    // Blinn–Phong's half vector is at 22.5° to the normal: cos² 22.5° = (1 + cos 45°)/2 = 0.85355339.
    EXPECT(nearly(kernels::blinnPhongSpecular(up, viewDirection, slanted, 2.0f), 0.85355339f));
    // A light under the surface gives none; shininess under 1 counts as 1.
    EXPECT(kernels::phongSpecular(up, viewDirection, Kernel3(0, -1, 0), 8.0f) == 0.0f);
    EXPECT(kernels::blinnPhongSpecular(up, viewDirection, Kernel3(0, -1, 0), 8.0f) == 0.0f);
    EXPECT(nearly(kernels::phongSpecular(up, viewDirection, slanted, 0.0f), 0.70710678f));
}

TEST(fresnelRimGrowsTowardsTheSilhouette) {
    // Facing the viewer: 0. Edge-on: 1. At 60° (cos 0.5) with power 2: 0.5² = 0.25.
    EXPECT(nearly(kernels::fresnelRim(Kernel3(0, 0, -1), Kernel3(0, 0, 1), 3.0f), 0.0f));
    EXPECT(nearly(kernels::fresnelRim(Kernel3(0, 1, 0), Kernel3(0, 0, 1), 3.0f), 1.0f));
    EXPECT(nearly(kernels::fresnelRim(Kernel3(0.8660254f, 0, -0.5f), Kernel3(0, 0, 1), 2.0f), 0.25f));
}

TEST(ambientOcclusionIsOneInTheOpenAndDarkInsideSolids) {
    // A floor: every sample along the normal is as far from the surface as it is high → sum 0 → 1.
    auto floorScene = [](Kernel3 p) { return p.y; };
    EXPECT(nearly(kernels::ambientOcclusion(floorScene, Kernel3(0, 0, 0), Kernel3(0, 1, 0), 0.5f, 1.0f), 1.0f));
    // A scene that is 0 everywhere: h = 0.2…1 weighted 1, 0.7, 0.49, 0.343, 0.2401 sums to 1.2885,
    // so the result is 1 − 0.6 × 1.2885 = 0.2269 for radius 1, strength 1.
    auto solid = [](Kernel3) { return 0.0f; };
    EXPECT(nearly(kernels::ambientOcclusion(solid, Kernel3(0, 0, 0), Kernel3(0, 1, 0), 1.0f, 1.0f), 0.2269f, 1e-5f));
    // Twice the strength darkens it twice as much (until the clamp at 0): 1 − 1.2 × 1.2885 < 0 → 0.
    EXPECT(kernels::ambientOcclusion(solid, Kernel3(0, 0, 0), Kernel3(0, 1, 0), 1.0f, 2.0f) == 0.0f);
}

TEST(softShadowIsOneInTheOpenZeroBlockedAndSoftBetween) {
    Kernel3 point(0, 0, 0);
    Kernel3 normal(0, 1, 0);
    Kernel3 light(0, 1, 0);
    auto open = [](Kernel3) { return 10.0f; };
    EXPECT(nearly(kernels::softShadow(open, point, normal, light, 8.0f, 10.0f), 1.0f));
    auto blocked = [](Kernel3) { return 0.0f; };
    EXPECT(kernels::softShadow(blocked, point, normal, light, 8.0f, 10.0f) == 0.0f);
    // A constant distance 0.01: the step is 0.01 and the ratio softness d/t is smallest at the last
    // of the 48 steps, t = 0.02 + 47 × 0.01 = 0.49 → 8 × 0.01/0.49 = 0.16326531; smoothed s²(3 − 2s) = 0.0712628.
    auto narrow = [](Kernel3) { return 0.01f; };
    EXPECT(nearly(kernels::softShadow(narrow, point, normal, light, 8.0f, 10.0f), 0.0712628f, 1e-4f));
    // Shadow under a sphere: the sphere centred 2 above the point shades it from a light straight up.
    auto sphere = [](Kernel3 p) { return kernels::lengthOf(p - Kernel3(0, 2, 0)) - 0.5f; };
    EXPECT(kernels::softShadow(sphere, point, normal, light, 8.0f, 10.0f) < 0.01f);
    EXPECT(kernels::softShadow(sphere, point, normal, Kernel3(1, 0.2f, 0), 8.0f, 10.0f) > 0.99f);
}

TEST(distanceFogBlendsTowardsTheFogColour) {
    Kernel4 color(1.0f, 0.0f, 0.5f, 0.25f);
    Kernel4 fog(0.0f, 1.0f, 0.5f, 1.0f);
    // No travel: unchanged, whatever the mode.
    for (int mode = 0; mode < 3; mode += 1) {
        Kernel4 same = kernels::distanceFog(color, 0.0f, fog, 1.0f, mode);
        EXPECT(nearly(same.x, 1.0f) && nearly(same.y, 0.0f) && nearly(same.z, 0.5f) && same.w == 0.25f);
    }
    // Half way: exponential 1 − e^(−d t) = 0.5 at t = ln 2; squared at t = √(ln 2); linear d t 0.1 = 0.5 at t = 5.
    float exponentialTravel = 0.6931472f;
    float squaredTravel = 0.8325546f;
    EXPECT(nearly(kernels::distanceFog(color, exponentialTravel, fog, 1.0f, 0).x, 0.5f));
    EXPECT(nearly(kernels::distanceFog(color, squaredTravel, fog, 1.0f, 1).y, 0.5f));
    EXPECT(nearly(kernels::distanceFog(color, 5.0f, fog, 1.0f, 2).x, 0.5f));
    // Far away it is all fog, and the alpha is kept.
    Kernel4 far = kernels::distanceFog(color, 1000.0f, fog, 1.0f, 0);
    EXPECT(nearly(far.x, 0.0f) && nearly(far.y, 1.0f) && far.w == 0.25f);
    // A negative density counts as 0.
    EXPECT(nearly(kernels::distanceFog(color, 5.0f, fog, -3.0f, 0).x, 1.0f));
}

TEST(skyRunsFromGroundThroughHorizonToZenithWithASun) {
    Kernel4 horizon(0.9f, 0.8f, 0.7f, 1.0f);
    Kernel4 zenith(0.1f, 0.2f, 0.9f, 1.0f);
    Kernel4 ground(0.3f, 0.3f, 0.3f, 1.0f);
    Kernel3 sunAway(0, -1, 0);
    // Straight up: the zenith; straight down: the ground; level: the horizon. The sun is elsewhere.
    Kernel4 up = kernels::skyColor(Kernel3(0, 1, 0), horizon, zenith, ground, Kernel3(0, 0, 1), 0.1f);
    EXPECT(nearly(up.x, 0.1f) && nearly(up.y, 0.2f) && nearly(up.z, 0.9f) && up.w == 1.0f);
    Kernel4 down = kernels::skyColor(Kernel3(0, -1, 0), horizon, zenith, ground, Kernel3(0, 0, 1), 0.1f);
    EXPECT(nearly(down.x, 0.3f) && nearly(down.y, 0.3f) && nearly(down.z, 0.3f));
    Kernel4 level = kernels::skyColor(Kernel3(1, 0, 0), horizon, zenith, ground, sunAway, 0.1f);
    EXPECT(nearly(level.x, 0.9f) && nearly(level.y, 0.8f) && nearly(level.z, 0.7f));
    // Looking into the sun adds (1, 0.9, 0.7) at full strength (cos 0 = 1, 1^anything = 1).
    Kernel4 sun = kernels::skyColor(Kernel3(1, 0, 0), horizon, zenith, ground, Kernel3(1, 0, 0), 0.1f);
    EXPECT(nearly(sun.x, 1.9f) && nearly(sun.y, 1.7f) && nearly(sun.z, 1.4f));
}

TEST(lookAtCameraRayShootsThroughTheScreenPoint) {
    Kernel3 origin(0, 0, -3);
    Kernel3 target(0, 0, 0);
    EXPECT(nearly(kernels::lookAtCameraRay(Kernel2(0, 0), origin, target, 60.0f), Kernel3(0, 0, 1)));
    // A 90° field of view: focal = 0.5 / tan 45° = 0.5; the screen's right edge (x = 0.5) is 45° off.
    EXPECT(nearly(kernels::lookAtCameraRay(Kernel2(0.5f, 0), origin, target, 90.0f), Kernel3(0.70710678f, 0, 0.70710678f)));
    EXPECT(nearly(kernels::lookAtCameraRay(Kernel2(0, 0.5f), origin, target, 90.0f), Kernel3(0, 0.70710678f, 0.70710678f)));
    // Looking straight up the right vector falls back to +x and the middle of the screen still looks up.
    EXPECT(nearly(kernels::lookAtCameraRay(Kernel2(0, 0), Kernel3(0, 0, 0), Kernel3(0, 5, 0), 60.0f), Kernel3(0, 1, 0), 1e-5f));
    // Unit length everywhere.
    for (int index = 0; index < 40; index += 1) {
        Kernel2 screen(static_cast<float>(index % 8) * 0.14f - 0.5f, static_cast<float>(index / 8) * 0.2f - 0.5f);
        EXPECT(nearly(kernels::lengthOf(kernels::lookAtCameraRay(screen, Kernel3(2, 1, -4), target, 50.0f)), 1.0f, 1e-5f));
    }
}

TEST(orbitCameraOriginSitsOnASphereAroundTheTarget) {
    Kernel3 target(1, 2, 3);
    // Yaw 0, pitch 0: radius 4 behind the target (−z), as the look-at camera expects.
    EXPECT(nearly(kernels::orbitCameraOrigin(target, 0.0f, 0.0f, 4.0f), Kernel3(1, 2, -1)));
    EXPECT(nearly(kernels::orbitCameraOrigin(target, 90.0f, 0.0f, 4.0f), Kernel3(5, 2, 3), 1e-5f));
    // Pitch 30°: height 4 sin 30° = 2, the ground distance 4 cos 30° = 3.4641016.
    EXPECT(nearly(kernels::orbitCameraOrigin(target, 0.0f, 30.0f, 4.0f), Kernel3(1, 4, 3.0f - 3.4641016f)));
    // Pitch is held to ±89°: 90° gives the same as 89°.
    EXPECT(nearly(kernels::orbitCameraOrigin(target, 20.0f, 90.0f, 4.0f), kernels::orbitCameraOrigin(target, 20.0f, 89.0f, 4.0f)));
    for (int index = 0; index < 40; index += 1) {
        Kernel3 origin = kernels::orbitCameraOrigin(target, static_cast<float>(index) * 31.0f, static_cast<float>(index) * 7.0f - 140.0f, 4.0f);
        EXPECT(nearly(kernels::lengthOf(origin - target), 4.0f, 1e-4f));
    }
}

TEST(densityMarchIntegratesTheDensityAlongTheRay) {
    // A constant density 2 over 5 units: optical depth 10 whatever the step count; transmittance e^(−0.1 × 10).
    auto constant = [](Kernel3) { return 2.0f; };
    Kernel2 thick = kernels::densityMarch(constant, Kernel3(0, 0, 0), Kernel3(0, 0, 1), 5.0f, 10.0f, 0.1f);
    EXPECT(nearly(thick.x, 10.0f, 1e-4f));
    EXPECT(nearly(thick.y, 0.36787944f, 1e-5f));
    Kernel2 coarse = kernels::densityMarch(constant, Kernel3(0, 0, 0), Kernel3(0, 0, 1), 5.0f, 0.0f, 0.1f);
    EXPECT(nearly(coarse.x, 10.0f, 1e-4f));
    // Density = z along +z over 4 units in 4 steps: midpoints 0.5, 1.5, 2.5, 3.5 sum to 8 (the exact integral).
    auto ramp = [](Kernel3 p) { return p.z; };
    EXPECT(nearly(kernels::densityMarch(ramp, Kernel3(0, 0, 0), Kernel3(0, 0, 1), 4.0f, 4.0f, 1.0f).x, 8.0f, 1e-4f));
    // Nothing (or a negative density) gives optical depth 0 and full transmittance.
    auto none = [](Kernel3) { return -1.0f; };
    Kernel2 clear = kernels::densityMarch(none, Kernel3(0, 0, 0), Kernel3(0, 0, 1), 5.0f, 8.0f, 1.0f);
    EXPECT(clear.x == 0.0f && clear.y == 1.0f);
}

TEST(reflectedAboutMirrorsTheIncomingDirection) {
    EXPECT(nearly(kernels::reflectedAbout(Kernel3(1, -1, 0), Kernel3(0, 1, 0)), Kernel3(1, 1, 0)));
    EXPECT(nearly(kernels::reflectedAbout(Kernel3(0, 0, 1), Kernel3(0, 0, -1)), Kernel3(0, 0, -1)));
    EXPECT(nearly(kernels::tangent(0.7853982f), 1.0f, 1e-6f));
}
