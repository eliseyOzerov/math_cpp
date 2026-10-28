// Semi-implicit Euler and velocity Verlet, with values worked out by hand
// (EditorTests/Math/Physics/ParticleIntegratorsTests.swift).
#include <algorithm>
#include <cmath>
#include <limits>

#include "../Numerics/ParticleIntegrators.h"
#include "../Physics/DampedSpring.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();

/// The unit oscillator: mass 1, stiffness 1, so ω = 1 and the acceleration is `−position`.
/// Started at x = 1 with no velocity its exact motion is x = cos t, v = −sin t, and its
/// energy `½|v|² + ½|x|²` is exactly 0.5 forever.
const ParticleState unitOscillatorStart(Vector3(1, 0, 0), Vector3(0, 0, 0));

float oscillatorEnergy(const ParticleState& state) {
    return state.kineticEnergy(1) + 0.5f * state.position.dot(state.position);
}

Vector3 pullToOrigin(Vector3 position) { return -position; }

using Step = std::function<ParticleState(const ParticleState&, float)>;

ParticleState semiImplicitEulerOnOscillator(const ParticleState& state, float timeStep) {
    return state.steppedBySemiImplicitEuler(timeStep, pullToOrigin);
}

ParticleState velocityVerletOnOscillator(const ParticleState& state, float timeStep) {
    return state.steppedByVelocityVerlet(timeStep, pullToOrigin);
}

} // namespace

// MARK: One step worked by hand

TEST(semiImplicitEulerOneStepOfTheOscillator) {
    // a = −x = −1, dt = 0.1.
    // v₁ = 0 + (−1)(0.1) = −0.1;  x₁ = 1 + (−0.1)(0.1) = 0.99 (uses the NEW velocity;
    // explicit Euler would give 1.0).
    ParticleState result = unitOscillatorStart.steppedBySemiImplicitEuler(0.1f, pullToOrigin);
    EXPECT(isApproximatelyEqual(result.velocity, Vector3(-0.1f, 0, 0), 1e-7f));
    EXPECT(isApproximatelyEqual(result.position, Vector3(0.99f, 0, 0), 1e-7f));
}

TEST(velocityVerletOneStepOfTheOscillator) {
    // a₀ = −1, dt = 0.1.
    // x₁ = 1 + 0·0.1 + ½(−1)(0.01) = 0.995;  a₁ = −0.995
    // v₁ = 0 + ½(−1 + −0.995)(0.1) = −0.09975.   (Exact: cos 0.1 = 0.995004, −sin 0.1 = −0.099833.)
    ParticleState result = unitOscillatorStart.steppedByVelocityVerlet(0.1f, pullToOrigin);
    EXPECT(isApproximatelyEqual(result.position, Vector3(0.995f, 0, 0), 1e-7f));
    EXPECT(isApproximatelyEqual(result.velocity, Vector3(-0.09975f, 0, 0), 1e-7f));
}

TEST(oneStepOfFallingUnderGravity) {
    // a = (0, 0, −9.81), from z = 10 at rest, dt = 0.5.
    // Euler: v = −9.81·0.5 = −4.905;  z = 10 + (−4.905)(0.5) = 7.5475.
    // Verlet: z = 10 − ½·9.81·0.25 = 8.77375, equal to the exact fall because the
    // acceleration is constant; v = ½(−9.81 − 9.81)(0.5) = −4.905.
    ParticleState start(Vector3(0, 0, 10), Vector3(0, 0, 0));
    auto gravity = [](Vector3) { return Vector3(0, 0, -9.81f); };
    ParticleState euler = start.steppedBySemiImplicitEuler(0.5f, gravity);
    EXPECT(isApproximatelyEqual(euler.position, Vector3(0, 0, 7.5475f), 1e-5f));
    EXPECT(isApproximatelyEqual(euler.velocity, Vector3(0, 0, -4.905f), 1e-5f));
    ParticleState verlet = start.steppedByVelocityVerlet(0.5f, gravity);
    EXPECT(isApproximatelyEqual(verlet.position, Vector3(0, 0, 8.77375f), 1e-5f));
    EXPECT(isApproximatelyEqual(verlet.velocity, Vector3(0, 0, -4.905f), 1e-5f));
}

TEST(oneStepWithVelocityDependentDrag) {
    // a = −v (drag, c/m = 1), start x = 0, v = (2,0,0), dt = 0.1. Exact after 0.1 s:
    // v = 2e^{−0.1} = 1.80967, x = 2(1 − e^{−0.1}) = 0.190325.
    ParticleState start(Vector3(0, 0, 0), Vector3(2, 0, 0));
    auto drag = [](Vector3, Vector3 velocity) { return -velocity; };
    // Euler: v₁ = 2 + (−2)(0.1) = 1.8;  x₁ = 0 + 1.8·0.1 = 0.18.
    ParticleState euler = start.steppedBySemiImplicitEuler(0.1f, drag);
    EXPECT(isApproximatelyEqual(euler.velocity, Vector3(1.8f, 0, 0), 1e-6f));
    EXPECT(isApproximatelyEqual(euler.position, Vector3(0.18f, 0, 0), 1e-6f));
    // Verlet: a₀ = −2;  x₁ = 0 + 0.2 + ½(−2)(0.01) = 0.19;  predicted v̂ = 2 − 0.2 = 1.8,
    // a₁ = −1.8;  v₁ = 2 + ½(−2 − 1.8)(0.1) = 1.81.
    ParticleState verlet = start.steppedByVelocityVerlet(0.1f, drag);
    EXPECT(isApproximatelyEqual(verlet.position, Vector3(0.19f, 0, 0), 1e-6f));
    EXPECT(isApproximatelyEqual(verlet.velocity, Vector3(1.81f, 0, 0), 1e-6f));
}

TEST(kineticEnergyIsHalfMassTimesSpeedSquared) {
    // ½ · 3 · (1² + 2² + 2²) = ½ · 3 · 9 = 13.5
    ParticleState state(Vector3(0, 0, 0), Vector3(1, 2, 2));
    EXPECT(isApproximatelyEqual(state.kineticEnergy(3), 13.5f, 1e-6f));
}

TEST(stepsOfNoTimeOrNegativeTimeChangeNothingAndNaNPropagates) {
    ParticleState start(Vector3(1, 2, 3), Vector3(4, 5, 6));
    EXPECT(start.steppedBySemiImplicitEuler(0, pullToOrigin) == start);
    EXPECT(start.steppedBySemiImplicitEuler(-0.1f, pullToOrigin) == start);
    EXPECT(start.steppedByVelocityVerlet(0, pullToOrigin) == start);
    EXPECT(start.steppedByVelocityVerlet(-0.1f, pullToOrigin) == start);
    // A NaN step is not "no time": it must show up as NaN.
    EXPECT(std::isnan(start.steppedBySemiImplicitEuler(notANumber, pullToOrigin).position.x));
    EXPECT(std::isnan(start.steppedByVelocityVerlet(notANumber, pullToOrigin).position.x));
}

// MARK: Energy stays bounded

namespace {

/// Runs `steps` steps of `timeStep` seconds on the unit oscillator and returns the largest
/// departure of the energy from its exact value 0.5, as a fraction of 0.5.
float largestRelativeEnergyDrift(int steps, float timeStep, const Step& step) {
    ParticleState state = unitOscillatorStart;
    float largest = 0;
    for (int index = 0; index < steps; index += 1) {
        state = step(state, timeStep);
        largest = std::max(largest, std::fabs(oscillatorEnergy(state) - 0.5f) / 0.5f);
    }
    return largest;
}

} // namespace

TEST(semiImplicitEulerEnergyStaysBoundedOver10000Steps) {
    // dt = 0.05, so 10 000 steps are 500 s, about 80 periods. Measured largest relative
    // energy error: 0.02565 (the shadow-energy wobble, ≈ dt/2 = 0.025); it does not grow
    // with the number of steps. Bound 0.03 = 17% margin.
    float drift = largestRelativeEnergyDrift(10000, 0.05f, semiImplicitEulerOnOscillator);
    EXPECT(drift < 0.03f);
    EXPECT(drift > 0.01f); // the wobble is real: it is bounded, not zero
    // Bounded means it stops growing: the worst error over the first 1000 steps is the
    // worst over all 10 000 (both measured 0.02564).
    float earlyDrift = largestRelativeEnergyDrift(1000, 0.05f, semiImplicitEulerOnOscillator);
    EXPECT(isApproximatelyEqual(drift, earlyDrift, 1e-3f));
}

TEST(velocityVerletEnergyStaysBoundedOver10000Steps) {
    // Same run. Measured largest relative energy error: 0.000627 (≈ dt²/4).
    // Bound 0.00075 = 20% margin.
    float drift = largestRelativeEnergyDrift(10000, 0.05f, velocityVerletOnOscillator);
    EXPECT(drift < 0.00075f);
}

TEST(explicitEulerEnergyGrowsWhichIsWhyItIsNotOffered) {
    // Plain explicit Euler multiplies the energy by 1 + dt² each step: after 100 steps of
    // 0.05 it is (1.0025)^100 = 1.28 times too big, and after 10 000 steps e^{25}. The
    // test writes the rejected scheme out by hand.
    ParticleState state = unitOscillatorStart;
    for (int index = 0; index < 100; index += 1) {
        Vector3 acceleration = -state.position;
        state = ParticleState(state.position + state.velocity * 0.05f, state.velocity + acceleration * 0.05f);
    }
    float growth = oscillatorEnergy(state) / 0.5f;
    EXPECT(isApproximatelyEqual(growth, 1.2836f, 1e-3f)); // (1.0025)^100 = 1.28362
}

// MARK: Order of accuracy

namespace {

/// The distance in (position, velocity) from the exact state cos t, −sin t after one second
/// of the unit oscillator, for a step count.
float errorAfterOneSecond(float timeStep, const Step& step) {
    ParticleState state = unitOscillatorStart;
    int steps = static_cast<int>(std::round(1 / timeStep));
    for (int index = 0; index < steps; index += 1) state = step(state, timeStep);
    float exactPosition = static_cast<float>(std::cos(1.0));
    float exactVelocity = static_cast<float>(-std::sin(1.0));
    float positionError = state.position.x - exactPosition;
    float velocityError = state.velocity.x - exactVelocity;
    return std::sqrt(positionError * positionError + velocityError * velocityError);
}

} // namespace

TEST(semiImplicitEulerIsFirstOrder) {
    // Halving dt halves the error. Measured error ratios for dt = 0.1, 0.05, 0.025, 0.0125:
    // 2.011, 2.005, 2.002 (errors 4.25e-2, 2.11e-2, 1.05e-2, 5.26e-3; the limit is 2).
    // Band 1.95…2.05.
    std::vector<float> errors;
    for (float timeStep : {0.1f, 0.05f, 0.025f, 0.0125f}) {
        errors.push_back(errorAfterOneSecond(timeStep, semiImplicitEulerOnOscillator));
    }
    for (size_t index = 1; index < errors.size(); index += 1) {
        float ratio = errors[index - 1] / errors[index];
        EXPECT(ratio > 1.95f && ratio < 2.05f);
    }
}

TEST(velocityVerletIsSecondOrder) {
    // Halving dt quarters the error. Measured error ratios for dt = 0.2, 0.1, 0.05, 0.025:
    // 4.011, 4.001, 3.991 (errors 3.61e-3, 8.99e-4, 2.25e-4, 5.63e-5; limit 4). Band 3.95…4.05.
    // (Smaller steps would push the error toward Float's own rounding, about 1e-7, and blur the
    // ratio.)
    std::vector<float> errors;
    for (float timeStep : {0.2f, 0.1f, 0.05f, 0.025f}) {
        errors.push_back(errorAfterOneSecond(timeStep, velocityVerletOnOscillator));
    }
    for (size_t index = 1; index < errors.size(); index += 1) {
        float ratio = errors[index - 1] / errors[index];
        EXPECT(ratio > 3.95f && ratio < 4.05f);
    }
}

TEST(velocityVerletStaysSecondOrderWithDamping) {
    // a = −x − 0.5v, the damped oscillator m = 1, k = 1, c = 0.5. With a velocity-dependent
    // force Verlet predicts v + a·dt for the new acceleration; that keeps it second order.
    // Measured error ratios for dt = 0.2, 0.1, 0.05, 0.025:
    // 4.134, 4.065, 4.028 (errors 1.37e-3, 3.37e-4, 8.36e-5; limit 4). Band 3.9…4.2.
    DampedOscillator exact(1, 1, 0.5f, 1, 0);
    std::vector<float> errors;
    for (float timeStep : {0.2f, 0.1f, 0.05f, 0.025f}) {
        ParticleState state = unitOscillatorStart;
        int steps = static_cast<int>(std::round(1 / timeStep));
        for (int index = 0; index < steps; index += 1) {
            state = state.steppedByVelocityVerlet(
                timeStep, [](Vector3 position, Vector3 velocity) { return -position - velocity * 0.5f; });
        }
        float positionError = state.position.x - exact.positionAt(1);
        float velocityError = state.velocity.x - exact.velocityAt(1);
        errors.push_back(std::sqrt(positionError * positionError + velocityError * velocityError));
    }
    for (size_t index = 1; index < errors.size(); index += 1) {
        float ratio = errors[index - 1] / errors[index];
        EXPECT(ratio > 3.9f && ratio < 4.2f);
    }
}

TEST(velocityVerletRunsBackwardsToItsStart) {
    // Time-reversible: 50 steps forward, flip the velocity, 50 steps more, flip it back,
    // and the particle is where it started. Measured: 1.8e-7 in position and 3.0e-7 in
    // velocity (Float rounding only); tolerance 1e-6.
    ParticleState state(Vector3(1, 0.5f, 0), Vector3(0, 0.8f, -0.3f));
    for (int index = 0; index < 50; index += 1) state = state.steppedByVelocityVerlet(0.05f, pullToOrigin);
    state = ParticleState(state.position, -state.velocity);
    for (int index = 0; index < 50; index += 1) state = state.steppedByVelocityVerlet(0.05f, pullToOrigin);
    float back = (state.position - Vector3(1, 0.5f, 0)).length();
    float backVelocity = (-state.velocity - Vector3(0, 0.8f, -0.3f)).length();
    EXPECT(back < 1e-6f && backVelocity < 1e-6f);
}

// MARK: Scale

TEST(theMotionScalesWithTheLengthUnit) {
    // The same oscillator started at x = scale (a millimetre or a kilometre away) must move
    // exactly `scale` times as far: the trajectory is the base one multiplied by `scale`.
    // Tolerance: 4e-6 relative to the scale (measured worst case 1.2e-6 at 1 mm and 1.0e-6
    // at 1 km over 200 steps; margin 3.4x).
    for (float scale : {1e-3f, 1.0f, 1000.0f}) {
        ParticleState base = unitOscillatorStart;
        ParticleState scaled(base.position * scale, base.velocity * scale);
        for (int index = 0; index < 200; index += 1) {
            base = base.steppedByVelocityVerlet(0.05f, pullToOrigin);
            scaled = scaled.steppedByVelocityVerlet(0.05f, pullToOrigin);
        }
        EXPECT(isApproximatelyEqual(scaled.position, base.position * scale, 4e-6f * scale));
        EXPECT(isApproximatelyEqual(scaled.velocity, base.velocity * scale, 4e-6f * scale));
        // The same holds for semi-implicit Euler. Measured: 3.5e-7 at 1 mm, 0 at 1, 6.1e-8 at
        // 1 km, relative to the scale; tolerance 1.5e-6.
        ParticleState baseEuler = unitOscillatorStart;
        ParticleState scaledEuler(baseEuler.position * scale, baseEuler.velocity * scale);
        for (int index = 0; index < 200; index += 1) {
            baseEuler = baseEuler.steppedBySemiImplicitEuler(0.05f, pullToOrigin);
            scaledEuler = scaledEuler.steppedBySemiImplicitEuler(0.05f, pullToOrigin);
        }
        float eulerDifference = std::max((scaledEuler.position - baseEuler.position * scale).length(),
                                         (scaledEuler.velocity - baseEuler.velocity * scale).length())
                              / scale;
        EXPECT(eulerDifference < 1.5e-6f);
    }
}
