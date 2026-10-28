// The second half of the optics module with values worked out by hand: a ray at a glass surface
// and through a glass solid, caustic splatting under a wavy surface, and Lambert and Blinn–Phong
// shading (EditorTests/Math/Optics/ RefractingSolidTests, CausticSplattingTests and
// DiffuseAndSpecularTests; the RefractingInterface tests live in RefractingSolidTests).
#include <cmath>
#include <vector>

#include "../Linear/Angle.h"
#include "../Numerics/Integration.h"
#include "../Optics/CausticSplatting.h"
#include "../Optics/DielectricFresnel.h"
#include "../Optics/DiffuseAndSpecular.h"
#include "../Optics/Dispersion.h"
#include "../Optics/LightDirections.h"
#include "../Optics/RefractingInterface.h"
#include "../Optics/RefractingSolid.h"
#include "Testing.h"

using namespace editor;

namespace {

bool isApproximatelyEqual(float first, float second, float tolerance = 1e-5f) {
    return testing::isApproximatelyEqual(first, second, tolerance);
}

bool isApproximatelyEqual(Vector3 first, Vector3 second, float tolerance = 1e-5f) {
    return testing::isApproximatelyEqual(first, second, tolerance);
}

Vector3 zero() { return Vector3(0, 0, 0); }

float squareRootOfThree() { return std::sqrt(3.0f); }

// MARK: - One surface (RefractingSolidTests)

Vector3 downward(Angle angle) { return Vector3(angle.sine(), 0, -angle.cosine()); }

} // namespace

TEST(normalIncidenceKeepsTheDirectionAndTransmitsNinetySixPercent) {
    WeightedRay incoming{Ray(Vector3(0, 0, 1), Vector3(0, 0, -1)), 1};
    InterfaceContinuation result = RefractingInterface::continuation(incoming, zero(), Vector3::zAxis(), 1, 1.5f);
    EXPECT(!result.wasReflected);
    EXPECT(result.continued.ray.origin == zero());
    EXPECT(isApproximatelyEqual(result.continued.ray.direction, Vector3(0, 0, -1)));
    // 1 − R = 1 − 0.04
    EXPECT(isApproximatelyEqual(result.continued.weight, 0.96f));
}

TEST(airToGlassAtFortyFiveDegreesMatchesTheHandWorkedValues) {
    // Direction: sin θₜ = 0.7071068/1.5 = 0.4714045, cos θₜ = 0.8819171.
    // Weight: Rs = 0.0920134, Rp = 0.0084665 → R = 0.0502399, 1 − R = 0.9497601.
    WeightedRay incoming{Ray(Vector3(-1, 0, 1), downward(Angle::degrees(45))), 0.5f};
    InterfaceContinuation result = RefractingInterface::continuation(incoming, zero(), Vector3::zAxis(), 1, 1.5f);
    EXPECT(isApproximatelyEqual(result.continued.ray.direction, Vector3(0.4714045f, 0, -0.8819171f)));
    EXPECT(isApproximatelyEqual(result.continued.weight, 0.5f * 0.9497601f, 1e-6f));
}

TEST(theNormalMayPointToEitherSide) {
    WeightedRay incoming{Ray(Vector3(-1, 0, 1), downward(Angle::degrees(30))), 1};
    InterfaceContinuation up = RefractingInterface::continuation(incoming, zero(), Vector3::zAxis(), 1, 1.33f);
    InterfaceContinuation down = RefractingInterface::continuation(incoming, zero(), -Vector3::zAxis(), 1, 1.33f);
    EXPECT(isApproximatelyEqual(up.continued.ray.direction, down.continued.ray.direction));
    EXPECT(isApproximatelyEqual(up.continued.weight, down.continued.weight));
}

TEST(totalInternalReflectionMirrorsTheRayAndKeepsAllItsPower) {
    // Inside glass at 45° (beyond the critical angle 41.81°): direction (sin45, 0, +cos45)
    // heading up at the surface from below; the mirror flips z.
    WeightedRay incoming{Ray(Vector3(-1, 0, -1), Vector3(0.70710678f, 0, 0.70710678f)), 0.8f};
    InterfaceContinuation result = RefractingInterface::continuation(incoming, zero(), Vector3::zAxis(), 1.5f, 1);
    EXPECT(result.wasReflected);
    EXPECT(isApproximatelyEqual(result.continued.ray.direction, Vector3(0.70710678f, 0, -0.70710678f)));
    EXPECT(result.continued.weight == 0.8f);
}

// MARK: - A slab of glass (RefractingSolidTests)

namespace {

RefractingSolid slab(float thickness, float refractiveIndex = 1.5f) {
    return RefractingSolid{
        {Plane(zero(), Vector3::zAxis()), Plane(Vector3(0, 0, -thickness), -Vector3::zAxis())}, refractiveIndex, 1};
}

} // namespace

TEST(aRayThroughAParallelSlabLeavesParallelAndShifted) {
    // n = 1.5, θ = 45°: sin θₜ = 0.4714045, cos θₜ = 0.8819171, θₜ = 28.1255°,
    // lateral shift = t·sin(θ − θₜ)/cos θₜ = t·sin(16.8745°)/0.8819171 = t·0.2902846/0.8819171 = 0.3291423·t.
    // Millimetre, metre and kilometre slabs; tolerances are relative to the thickness
    // (measured error: under 1e-7 of t, so a tolerance of 1e-6 of t).
    for (float thickness : {0.001f, 1.0f, 1000.0f}) {
        Vector3 direction = downward(Angle::degrees(45));
        Ray incoming(Vector3(0, 0, 0) - direction * thickness, direction);
        std::vector<RayLeg> legs = slab(thickness).path(incoming);
        EXPECT(legs.size() == 3);
        if (legs.size() != 3) return;
        Ray exiting = legs[2].start.ray;
        EXPECT(isApproximatelyEqual(exiting.direction, direction, 1e-5f));
        // Distance from the exit point to the straight line the ray would have followed.
        float offset = (exiting.origin - incoming.origin).cross(direction).length();
        EXPECT(isApproximatelyEqual(offset, 0.3291423f * thickness, 1e-6f * thickness));
        // Inside, the ray crosses the slab: its length is t/cos θₜ.
        EXPECT(legs[1].length.has_value());
        if (!legs[1].length) return;
        EXPECT(isApproximatelyEqual(*legs[1].length, thickness / 0.8819171f, 1e-6f * thickness));
        // Entering and leaving: each surface passes the unpolarized 1 − R. This ignores the
        // polarization the first surface introduces (as PBRT does); the exact value,
        // (Ts² + Tp²)/2, is 0.90379, 0.2% above this product.
        EXPECT(isApproximatelyEqual(legs[2].start.weight, 0.9497601f * 0.9497601f, 1e-5f));
        EXPECT(!legs[2].length.has_value());
    }
}

TEST(aRayInsideJustPastAFaceStillFindsTheFaceAhead) {
    // Rounding at an edge can leave a ray that is inside a hair past a face it is moving
    // away through (x = 1 + 10⁻⁶). That face's crossing is behind it (t < 0), so the ray
    // must leave through the face ahead, the floor at z = −1, at t = 0.5·√2 = 0.70711.
    RefractingSolid wedge{
        {Plane(zero(), Vector3::zAxis()),
         Plane(Vector3(0, 0, -1), -Vector3::zAxis()),
         Plane(Vector3(1, 0, 0), Vector3::xAxis())},
        1.5f,
        1};
    Ray ray(Vector3(1 + 1e-6f, 0, -0.5f), Vector3(1, 0, -1).normalized());
    std::optional<BoundaryHit> hit = wedge.firstBoundaryHit(ray, true);
    EXPECT(hit.has_value());
    if (!hit) return;
    EXPECT(isApproximatelyEqual(hit->distance, 0.70710678f, 1e-6f));
    EXPECT(hit->faceNormal == -Vector3::zAxis());
}

TEST(straightThroughASlabLosesEightPercent) {
    std::vector<RayLeg> legs = slab(2).path(Ray(Vector3(0.3f, 0, 5), Vector3(0, 0, -1)));
    EXPECT(legs.size() == 3);
    if (legs.size() != 3) return;
    EXPECT(legs[0].length.has_value() && isApproximatelyEqual(*legs[0].length, 5, 1e-5f));
    EXPECT(legs[1].length.has_value() && isApproximatelyEqual(*legs[1].length, 2, 1e-5f));
    // (1 − 0.04)² = 0.9216
    EXPECT(isApproximatelyEqual(legs[2].start.weight, 0.9216f, 1e-6f));
    EXPECT(isApproximatelyEqual(legs[2].start.ray.direction, Vector3(0, 0, -1)));
}

TEST(aRayPointingAwayOrParallelMissesTheSlab) {
    std::vector<RayLeg> away = slab(1).path(Ray(Vector3(0, 0, 2), Vector3(0, 0, 1)));
    EXPECT(away.size() == 1 && !away[0].length.has_value());
    std::vector<RayLeg> parallelAbove = slab(1).path(Ray(Vector3(0, 0, 2), Vector3(1, 0, 0)));
    EXPECT(parallelAbove.size() == 1 && !parallelAbove[0].length.has_value());
}

// MARK: - A prism (RefractingSolidTests)

namespace {

/// A prism with a triangular cross-section in the x–y plane, endless along z: base from
/// (−½, 0) to (½, 0), apex at (0, √3/2). All three faces make 60° with each other.
RefractingSolid equilateralPrism(float refractiveIndex) {
    float apexHeight = squareRootOfThree() / 2;
    return RefractingSolid{
        {Plane(zero(), Vector3(0, -1, 0)),
         Plane(Vector3(0.5f, 0, 0), Vector3(apexHeight, 0.5f, 0)),
         Plane(Vector3(-0.5f, 0, 0), Vector3(-apexHeight, 0.5f, 0))},
        refractiveIndex,
        1};
}

} // namespace

TEST(aPrismAtMinimumDeviationTurnsTheRayBySymmetricPassage) {
    // n = 1.5, apex 60°. Inside, the ray runs parallel to the base: it meets each side
    // face at 30°. Outside, Snell: sin θ₁ = 1.5 sin 30° = 0.75, θ₁ = 48.590378°.
    // The face normal points 30° above −x; the incoming ray makes θ₁ with its inward
    // side, so it climbs at θ₁ − 30° = 18.590378° above +x. By symmetry it leaves 18.59°
    // below +x: total deviation 2θ₁ − 60° = 37.180756°.
    Angle climb = Angle::degrees(18.590378f);
    Vector3 incomingDirection(climb.cosine(), climb.sine(), 0);
    Vector3 entryPoint(-0.25f, squareRootOfThree() / 4, 0);
    Ray incoming(entryPoint - incomingDirection, incomingDirection);
    std::vector<RayLeg> legs = equilateralPrism(1.5f).path(incoming);
    EXPECT(legs.size() == 3);
    if (legs.size() != 3) return;
    // Inside: parallel to the base, from the left face midpoint to the right face midpoint (length 0.5).
    EXPECT(isApproximatelyEqual(legs[1].start.ray.direction, Vector3(1, 0, 0), 1e-5f));
    EXPECT(legs[1].length.has_value() && isApproximatelyEqual(*legs[1].length, 0.5f, 1e-5f));
    EXPECT(isApproximatelyEqual(legs[2].start.ray.direction, Vector3(climb.cosine(), -climb.sine(), 0), 1e-5f));
    EXPECT(isApproximatelyEqual(legs[2].start.ray.origin, Vector3(0.25f, squareRootOfThree() / 4, 0), 1e-5f));
}

TEST(aRightAnglePrismTurnsLightAroundByTotalInternalReflection) {
    // Glass triangle x ≥ 0, y ≥ 0, x + y ≤ 1, n = 1.5. A ray going up (+y) enters the
    // bottom face straight (no bend), meets the slanted face at 45° from its normal,
    // beyond the 41.81° critical angle. Mirror of (0,1) in the normal (1,1)/√2:
    // (0,1) − 2(0.7071)(0.7071, 0.7071) = (−1, 0). It leaves the left face straight on.
    RefractingSolid prism{
        {Plane(zero(), Vector3(0, -1, 0)),
         Plane(zero(), Vector3(-1, 0, 0)),
         Plane(Vector3(1, 0, 0), Vector3(1, 1, 0))},
        1.5f,
        1};
    std::vector<RayLeg> legs = prism.path(Ray(Vector3(0.25f, -1, 0), Vector3(0, 1, 0)));
    EXPECT(legs.size() == 4);
    if (legs.size() != 4) return;
    std::optional<Vector3> endPoint = legs[1].endPoint();
    EXPECT(endPoint.has_value() && isApproximatelyEqual(*endPoint, Vector3(0.25f, 0.75f, 0), 1e-5f));
    EXPECT(isApproximatelyEqual(legs[2].start.ray.direction, Vector3(-1, 0, 0), 1e-5f));
    EXPECT(isApproximatelyEqual(legs[3].start.ray.direction, Vector3(-1, 0, 0), 1e-5f));
    // Two surface crossings at normal incidence (0.96 each); the reflection loses nothing.
    EXPECT(isApproximatelyEqual(legs[3].start.weight, 0.9216f, 1e-6f));
}

TEST(violetBendsMoreThanRedInGlass) {
    // Same ray, same prism, indices from the Sellmeier law at 400 nm and 700 nm.
    Vector3 direction(Angle::degrees(18.59f).cosine(), Angle::degrees(18.59f).sine(), 0);
    Vector3 origin = Vector3(-0.25f, squareRootOfThree() / 4, 0) - direction;
    std::vector<float> deviations;
    for (float nanometres : {400.0f, 700.0f}) {
        float index = SellmeierDispersion::schottNBK7().refractiveIndex(Wavelength::fromNanometres(nanometres));
        std::vector<RayLeg> legs = equilateralPrism(index).path(Ray(origin, direction));
        EXPECT(!legs.empty());
        if (legs.empty()) return;
        Vector3 exitDirection = legs.back().start.ray.direction;
        deviations.push_back(Angle::fromRadians(std::acos(cosineOfAngle(direction, exitDirection))).degrees());
    }
    // Hand check with δ = θ₁ + θ₄ − 60°, where sin θ₂ = sin θ₁/n, θ₃ = 60° − θ₂,
    // sin θ₄ = n sin θ₃, at θ₁ = 48.59° and n(400 nm) = 1.530849, n(700 nm) = 1.513064:
    //   400 nm: θ₂ = 29.335°, θ₃ = 30.665°, θ₄ = 51.329° → δ = 39.919°
    //   700 nm: δ = 38.324°
    EXPECT(isApproximatelyEqual(deviations[0], 39.919f, 0.02f));
    EXPECT(isApproximatelyEqual(deviations[1], 38.324f, 0.02f));
    EXPECT(deviations[0] > deviations[1]);
}

// MARK: - Caustic splatting (CausticSplattingTests)

namespace {

/// z = height everywhere.
class FlatSurface : public HeightField {
public:
    float level = 0;
    float height(float, float) const override { return level; }
    Vector2 slope(float, float) const override { return Vector2(0, 0); }
};

/// z = −curvature·x²: higher in the middle, like a convex lens of water.
class Ridge : public HeightField {
public:
    float curvature;
    explicit Ridge(float curvature) : curvature(curvature) {}
    float height(float x, float) const override { return -curvature * x * x; }
    Vector2 slope(float x, float) const override { return Vector2(-2 * curvature * x, 0); }
};

/// A few gentle sine waves in different directions.
class Ripples : public HeightField {
public:
    float height(float x, float y) const override {
        return 0.012f * std::sin(9 * x + 2 * y) + 0.010f * std::sin(-5 * x + 11 * y + 1)
            + 0.008f * std::sin(13 * x - 7 * y + 2);
    }
    Vector2 slope(float x, float y) const override {
        return Vector2(
            0.012f * 9 * std::cos(9 * x + 2 * y) - 0.010f * 5 * std::cos(-5 * x + 11 * y + 1)
                + 0.008f * 13 * std::cos(13 * x - 7 * y + 2),
            0.012f * 2 * std::cos(9 * x + 2 * y) + 0.010f * 11 * std::cos(-5 * x + 11 * y + 1)
                - 0.008f * 7 * std::cos(13 * x - 7 * y + 2));
    }
};

constexpr float waterTransmittanceAtNormalIncidence = 0.97994069f;   // 1 − ((1 − 1.33)/(1 + 1.33))²

/// Light of irradiance 1 over [−1,1]², 120 rays each way, onto a receiver at `depth`
/// below the surface covering [−1,1]², with 120 × 120 cells.
CausticResult simulate(const HeightField& surface, float depth = 1, int receiverCells = 120) {
    return CausticSplatting::receiverIrradiance(
        surface, 1, 1.33f, 1, Vector2(-1, -1), Vector2(1, 1), 120, 120, -depth,
        IrradianceGrid(Vector2(-1, -1), Vector2(1, 1), receiverCells, receiverCells));
}

} // namespace

TEST(normalOfAHeightFieldPointsUpAndLeansAgainstTheSlope) {
    EXPECT(isApproximatelyEqual(FlatSurface().normal(3, -2), Vector3(0, 0, 1)));
    // Slope 1 along x (45°): (−1, 0, 1)/√2 = (−0.70710678, 0, 0.70710678)
    Vector3 tilted = Ridge(-0.5f).normal(1, 0);   // h = 0.5x², slope = x = 1
    EXPECT(isApproximatelyEqual(tilted, Vector3(-0.70710678f, 0, 0.70710678f)));
    EXPECT(isApproximatelyEqual(tilted.length(), 1));
}

TEST(splatKeepsAllThePowerAndSharesItByCloseness) {
    // 4×4 cells of size 1 over [0,4]²; cell area 1, so irradiance = power.
    IrradianceGrid grid(Vector2(0, 0), Vector2(4, 4), 4, 4);
    grid.splat(8, Vector2(1.5f, 2.5f));   // exactly the centre of cell (1, 2)
    EXPECT(grid(1, 2) == 8);
    // The corner shared by cells (1,1), (2,1), (1,2), (2,2): a quarter each (2).
    grid.splat(8, Vector2(2, 2));
    EXPECT(grid(1, 1) == 2 && grid(2, 1) == 2 && grid(2, 2) == 2);
    EXPECT(grid(1, 2) == 10);
    // A quarter of a cell right of a centre: 0.75 to this cell, 0.25 to the next.
    grid.splat(4, Vector2(2.75f, 0.5f));
    EXPECT(grid(2, 0) == 3 && grid(3, 0) == 1);
    // Total: 8 + 8 + 4 = 20, all kept.
    EXPECT(isApproximatelyEqual(grid.totalPower(), 20, 1e-5f));
}

TEST(splatOnTheEdgeStillKeepsAllThePower) {
    IrradianceGrid grid(Vector2(0, 0), Vector2(4, 2), 4, 2);
    grid.splat(8, Vector2(0, 0));      // the corner of the rectangle
    grid.splat(2, Vector2(4, 2));
    EXPECT(grid(0, 0) == 8 && grid(3, 1) == 2);
    EXPECT(isApproximatelyEqual(grid.totalPower(), 10, 1e-5f));
}

TEST(cellAreaFollowsTheRectangle) {
    IrradianceGrid grid(Vector2(-1, 0), Vector2(1, 3), 4, 6);
    EXPECT(grid.cellSize() == Vector2(0.5f, 0.5f));
    EXPECT(grid.cellArea() == 0.25f);
}

TEST(aFlatSurfaceGivesUniformLight) {
    // Straight down through a flat surface, nothing bends: every cell gets irradiance 1
    // times the normal-incidence transmittance 0.97994069, exactly one ray per cell.
    // Measured largest difference: 7.5e-6 (Float rounding of 1/1.33); tolerance 2e-5.
    FlatSurface surface;
    surface.level = 0.3f;
    CausticResult result = simulate(surface);
    EXPECT(result.emittedPower == 4);
    for (int row = 0; row < 120; row += 1) {
        for (int column = 0; column < 120; column += 1) {
            EXPECT(isApproximatelyEqual(result.irradiance(column, row), waterTransmittanceAtNormalIncidence, 2e-5f));
        }
    }
    EXPECT(result.powerMissingReceiver == 0);
}

TEST(aFlatSurfaceIsUniformAlsoOnACoarserReceiver) {
    // 60 × 60 cells: two rays per cell along each axis, split between neighbouring
    // cells by the splatting. The sum over a cell is uniform; the edge cells are left out
    // only because their outer half-cell of rays is clamped in, which this test is not about. Measured largest difference: 2.0e-6; tolerance 1e-5.
    CausticResult result = simulate(FlatSurface(), 1, 60);
    for (int row = 1; row < 59; row += 1) {
        for (int column = 1; column < 59; column += 1) {
            EXPECT(isApproximatelyEqual(result.irradiance(column, row), waterTransmittanceAtNormalIncidence, 1e-5f));
        }
    }
}

TEST(energyIsConservedUnderWavySurface) {
    Ripples surface;
    CausticResult result = simulate(surface, 0.8f);
    // 1. Everything that went in the water ends in the grid or in the part that missed.
    EXPECT(isApproximatelyEqual(
        result.irradiance.totalPower(), result.transmittedPower - result.powerMissingReceiver,
        result.transmittedPower * 1e-4f));
    // 2. What went in is emitted power times the average Fresnel transmittance, computed
    //    here one ray at a time from the surface slope, independently of the simulation.
    double expectedTransmitted = 0.0;
    for (int row = 0; row < 120; row += 1) {
        for (int column = 0; column < 120; column += 1) {
            float x = -1 + (static_cast<float>(column) + 0.5f) / 60;
            float y = -1 + (static_cast<float>(row) + 0.5f) / 60;
            float cosine = surface.normal(x, y).z;   // light comes straight down
            float transmittance = DielectricFresnel::reflectance(cosine, 1 / 1.33f).transmittance();
            expectedTransmitted += static_cast<double>(transmittance) * static_cast<double>(4.0f / 14400);
        }
    }
    EXPECT(isApproximatelyEqual(
        result.transmittedPower, static_cast<float>(expectedTransmitted), static_cast<float>(expectedTransmitted) * 1e-4f));
    // 3. The surface reflects a few percent: the average transmittance is just under 1.
    float averageTransmittance = result.transmittedPower / result.emittedPower;
    EXPECT(averageTransmittance > 0.97f && averageTransmittance < 0.98f);
    // 4. Nearly all of it lands in the receiver, which is as wide as the light.
    EXPECT(result.powerMissingReceiver < result.transmittedPower * 0.05f);
}

TEST(wavesMakeBrightAndDarkPatches) {
    CausticResult result = simulate(Ripples(), 0.8f);
    float mean = result.irradiance.totalPower() / 4;
    float brightest = *std::max_element(result.irradiance.values().begin(), result.irradiance.values().end());
    float darkest = *std::min_element(result.irradiance.values().begin(), result.irradiance.values().end());
    EXPECT(brightest > 2 * mean);
    EXPECT(darkest < 0.5f * mean);
}

TEST(theLensFocusesTheSameAtAMillimetreAndAKilometre) {
    // The same lens as below with every length times `scale`: curvature c/scale, depth and
    // corners times scale. 2c·d is unchanged, so the light is squeezed into the same
    // 0.50376 of its width, and the irradiance (light per area) is the same 1.9453.
    for (float scale : {0.001f, 1000.0f}) {
        CausticResult result = CausticSplatting::receiverIrradiance(
            Ridge(1 / scale), 1, 1.33f, 1,
            Vector2(-0.1f, -0.05f) * scale, Vector2(0.1f, 0.05f) * scale, 1600, 8, -scale,
            IrradianceGrid(Vector2(-0.1f, -0.05f) * scale, Vector2(0.1f, 0.05f) * scale, 100, 1));
        float centre = (result.irradiance(49, 0) + result.irradiance(50, 0)) / 2;
        EXPECT(isApproximatelyEqual(centre, 1.9453f, 1.9453f * 0.005f));
        EXPECT(result.powerMissingReceiver == 0);
    }
}

TEST(aConvexLensFocusesLightByTheParaxialAmount) {
    // h = −c x² with c = 1 and water n = 1.33. Near the axis the slope is −2cx, the ray
    // is bent towards the axis by (1 − 1/n)·slope, and at depth d it has moved to
    //     x' = x·(1 − 2c d (1 − 1/n)) = x·(1 − 2×1×1×0.2481) = 0.50376·x.
    // The rays near the middle are squeezed into 0.50376 of their width, so the light per
    // area there is T₀/0.50376 = 0.97994069/0.50376 = 1.9453 (the line focus is at
    // d = 1/(2c(1 − 1/n)) = 2.01).
    CausticResult result = CausticSplatting::receiverIrradiance(
        Ridge(1), 1, 1.33f, 1, Vector2(-0.1f, -0.05f), Vector2(0.1f, 0.05f), 1600, 8, -1,
        IrradianceGrid(Vector2(-0.1f, -0.05f), Vector2(0.1f, 0.05f), 100, 1));
    float centre = (result.irradiance(49, 0) + result.irradiance(50, 0)) / 2;
    // Measured: 1.94509 (0.01% below the paraxial value); tolerance 0.5%.
    EXPECT(isApproximatelyEqual(centre, 1.9453f, 1.9453f * 0.005f));
    // The lens is symmetric: mirrored cells match (measured difference 3e-6).
    for (int column = 0; column < 50; column += 1) {
        EXPECT(isApproximatelyEqual(result.irradiance(column, 0), result.irradiance(99 - column, 0), 1e-4f));
    }
    // All the light lands within ±0.0504 (0.1 × 0.50376): cells at x = −0.048 are lit,
    // cells at x = −0.056 get nothing.
    EXPECT(result.irradiance(26, 0) > 1);
    EXPECT(result.irradiance(22, 0) == 0);
}

// MARK: - Lambert diffuse (DiffuseAndSpecularTests)

TEST(brdfIsAlbedoOverPi) {
    // 1/π = 0.31830989; 0.5/π = 0.15915494
    EXPECT(isApproximatelyEqual(LambertDiffuse::brdf(1), 0.31830989f));
    EXPECT(isApproximatelyEqual(LambertDiffuse::brdf(0.5f), 0.15915494f));
    EXPECT(LambertDiffuse::brdf(0) == 0);
}

TEST(cosineFactorFollowsTheAngleOfTheLight) {
    // cos 0° = 1, cos 60° = 0.5, cos 90° = 0
    Vector3 overhead = Vector3::zAxis();
    Vector3 sixtyDegrees(Angle::degrees(60).sine(), 0, Angle::degrees(60).cosine());
    EXPECT(isApproximatelyEqual(LambertDiffuse::cosineFactor(Vector3::zAxis(), overhead), 1));
    EXPECT(isApproximatelyEqual(LambertDiffuse::cosineFactor(Vector3::zAxis(), sixtyDegrees), 0.5f));
    EXPECT(isApproximatelyEqual(LambertDiffuse::cosineFactor(Vector3::zAxis(), Vector3::xAxis()), 0));
}

TEST(lightBehindTheSurfaceGivesZeroNotNegative) {
    Vector3 below = Vector3(1, 0, -1).normalized();
    EXPECT(LambertDiffuse::cosineFactor(Vector3::zAxis(), below) == 0);
    EXPECT(LambertDiffuse::reflectance(1, Vector3::zAxis(), -Vector3::zAxis()) == 0);
}

TEST(reflectanceIsBrdfTimesCosine) {
    // albedo 0.8 at 60°: 0.8/π · 0.5 = 0.12732395
    Vector3 light(Angle::degrees(60).sine(), 0, Angle::degrees(60).cosine());
    EXPECT(isApproximatelyEqual(LambertDiffuse::reflectance(0.8f, Vector3::zAxis(), light), 0.12732395f));
}

TEST(energyConservationIntegratesToTheAlbedo) {
    // ∫ f·cos θ dω over the hemisphere, with dω = sin θ dθ dφ. The φ integral gives 2π,
    // the rest depends only on θ: 2π ∫₀^{π/2} (albedo/π)·cos θ·sin θ dθ
    //   = 2·albedo·(1/2) = albedo.
    for (float albedo : {0.1f, 0.5f, 0.8f, 1.0f}) {
        float integral = GaussLegendre::integral(
            [albedo](float theta) {
                Vector3 light(std::sin(theta), 0, std::cos(theta));
                return LambertDiffuse::reflectance(albedo, Vector3::zAxis(), light) * std::sin(theta);
            },
            0, Angle::pi / 2, 4) * 2 * Angle::pi;
        EXPECT(isApproximatelyEqual(integral, albedo, albedo * 1e-4f));
    }
}

// MARK: - Blinn–Phong normalization (DiffuseAndSpecularTests)

namespace {

/// The directional albedo at normal incidence (viewer along +z): the BRDF times the cosine
/// factor, summed over the light hemisphere. With dω = sin θ dθ dφ the φ integral is 2π.
float albedoAtNormalIncidence(float shininess) {
    return GaussLegendre::integral(
        [shininess](float theta) {
            Vector3 towardLight(std::sin(theta), 0, std::cos(theta));
            return BlinnPhong::brdf(Vector3::zAxis(), towardLight, Vector3::zAxis(), shininess)
                * std::cos(theta) * std::sin(theta);
        },
        0, Angle::pi / 2, 128) * 2 * Angle::pi;
}

} // namespace

TEST(albedoMatchesTheClosedFormAndStaysNearOne) {
    // Closed form from the derivation in `normalizationFactor`:
    //   (m+8)(m + 2^(−m/2)) / ((m+2)(m+4))
    // m = 0:    8·(0 + 1) / (2·4)                   = 1
    // m = 1:    9·(1 + 0.70710678) / (3·5)          = 9·1.70710678/15 = 1.0242641
    // m = 10:   18·(10 + 0.03125) / (12·14)         = 180.5625/168   = 1.0747768
    // m = 100:  108·(100 + 7.9e-16) / (102·104)     = 10800/10608    = 1.0181000
    // m = 1000: 1008·1000 / (1002·1004)             = 1008000/1006008 = 1.0019801
    // The albedo stays ≤ 1 plus 7.5%: the approximation (m+8)/(8π) overshoots slightly
    // (maximum about 1.075 near m = 8…10), as the doc of `normalizationFactor` states.
    struct Case {
        float shininess;
        float albedo;
    };
    for (Case testCase : {Case{0, 1}, Case{1, 1.0242641f}, Case{10, 1.0747768f}, Case{100, 1.0181000f}, Case{1000, 1.0019801f}}) {
        float albedo = albedoAtNormalIncidence(testCase.shininess);
        EXPECT(isApproximatelyEqual(albedo, testCase.albedo, testCase.albedo * 1e-3f));
        EXPECT(albedo <= 1.08f);
    }
}

// MARK: - Blinn–Phong (DiffuseAndSpecularTests)

namespace {

/// A unit direction `degrees` from +z in the x–z plane; negative degrees lean toward −x.
Vector3 direction(float degrees) {
    Angle angle = Angle::degrees(degrees);
    return Vector3(angle.sine(), 0, angle.cosine());
}

} // namespace

TEST(halfVectorBisectsTheTwoDirections) {
    // (1,0,0) + (0,0,1) = (1,0,1), length √2 → (0.70710678, 0, 0.70710678)
    Vector3 half = halfVector(Vector3::xAxis(), Vector3::zAxis());
    EXPECT(isApproximatelyEqual(half, Vector3(0.70710678f, 0, 0.70710678f)));
    // Opposite directions have no halfway direction.
    EXPECT(halfVector(Vector3::xAxis(), -Vector3::xAxis()) == Vector3(0, 0, 0));
}

TEST(halfVectorOfDirectionsBuiltAtAnyScaleHasLengthOne) {
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Vector3 towardLight = Vector3(scale, 0, scale).normalized();
        Vector3 towardViewer = Vector3(-scale, 0, scale).normalized();
        Vector3 half = halfVector(towardLight, towardViewer);
        EXPECT(isApproximatelyEqual(half, Vector3::zAxis()));
        EXPECT(isApproximatelyEqual(half.length(), 1));
    }
}

TEST(lobePeaksAtOneInTheMirrorDirection) {
    // Light and viewer at equal angles on opposite sides: h = normal, so (n·h)^s = 1.
    for (float degrees : {0.0f, 20.0f, 45.0f, 70.0f, 85.0f}) {
        float value = BlinnPhong::lobe(Vector3::zAxis(), direction(degrees), direction(-degrees), 50);
        EXPECT(isApproximatelyEqual(value, 1));
    }
}

TEST(lobeMatchesTheHandWorkedValue) {
    // Light at 60°, viewer at 0°: h is at 30°, n·h = cos 30° = 0.8660254.
    // shininess 2 → 0.8660254² = 0.75; shininess 0 → 1.
    Vector3 towardLight = direction(60);
    EXPECT(isApproximatelyEqual(BlinnPhong::lobe(Vector3::zAxis(), towardLight, Vector3::zAxis(), 2), 0.75f));
    EXPECT(isApproximatelyEqual(BlinnPhong::lobe(Vector3::zAxis(), towardLight, Vector3::zAxis(), 0), 1));
}

TEST(higherShininessGivesANarrowerLobe) {
    Vector3 towardLight = direction(40);
    Vector3 towardViewer = direction(-20);
    float wide = BlinnPhong::lobe(Vector3::zAxis(), towardLight, towardViewer, 5);
    float narrow = BlinnPhong::lobe(Vector3::zAxis(), towardLight, towardViewer, 200);
    EXPECT(narrow < wide);
}

TEST(swappingLightAndViewerGivesTheSameValue) {
    Vector3 first = direction(35);
    Vector3 second = Vector3(-0.3f, 0.4f, 0.8f).normalized();
    float forward = BlinnPhong::brdf(Vector3::zAxis(), first, second, 30);
    float backward = BlinnPhong::brdf(Vector3::zAxis(), second, first, 30);
    EXPECT(forward > 0);
    EXPECT(forward == backward);
}

TEST(belowTheSurfaceGivesZero) {
    Vector3 below = Vector3(1, 0, -0.2f).normalized();
    EXPECT(BlinnPhong::lobe(Vector3::zAxis(), below, Vector3::zAxis(), 10) == 0);
    EXPECT(BlinnPhong::lobe(Vector3::zAxis(), Vector3::zAxis(), below, 10) == 0);
}

TEST(normalizationFactorMatchesTheFormula) {
    // (8 + 8)/(8π) = 2/π = 0.63661977; (100 + 8)/(8π) = 108/25.132741 = 4.2971
    EXPECT(isApproximatelyEqual(BlinnPhong::normalizationFactor(8), 0.63661977f));
    EXPECT(isApproximatelyEqual(BlinnPhong::normalizationFactor(100), 4.2971f, 1e-3f));
}

TEST(brdfAtThePeakIsTheNormalizationFactor) {
    EXPECT(isApproximatelyEqual(
        BlinnPhong::brdf(Vector3::zAxis(), Vector3::zAxis(), Vector3::zAxis(), 8), 0.63661977f));
}
