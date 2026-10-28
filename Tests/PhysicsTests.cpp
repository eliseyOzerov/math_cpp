// Springs, the exact damped oscillator, mass and inertia tensors, with values worked out by
// hand (EditorTests/Math/Physics/DampedSpringTests.swift and MassPropertiesTests.swift).
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "../Numerics/ParticleIntegrators.h"
#include "../Physics/DampedSpring.h"
#include "../Physics/MassProperties.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
const Vector3 origin(0, 0, 0);

} // namespace

// MARK: Spring force worked by hand

TEST(stretchedSpringPullsTheEndsTogether) {
    // k = 100, rest 1, first at (3,0,0), second at the origin: L = 3, d̂ = (1,0,0).
    // F = −100·(3 − 1)·d̂ = (−200, 0, 0): toward the second point.
    DampedSpring spring(100, 0, 1);
    Vector3 force = spring.forceOnFirst(Vector3(3, 0, 0), origin, origin, origin);
    EXPECT(isApproximatelyEqual(force, Vector3(-200, 0, 0), 1e-4f));
}

TEST(compressedSpringPushesTheEndsApart) {
    // L = 0.5: F = −100·(0.5 − 1)·d̂ = +50·(1,0,0), away from the second point.
    DampedSpring spring(100, 0, 1);
    Vector3 force = spring.forceOnFirst(Vector3(0.5f, 0, 0), origin, origin, origin);
    EXPECT(isApproximatelyEqual(force, Vector3(50, 0, 0), 1e-4f));
}

TEST(obliqueSpringActsAlongTheLine) {
    // first (3,4,0), second origin: L = 5, d̂ = (0.6, 0.8, 0). k = 10, rest 2:
    // F = −10·(5 − 2)·d̂ = (−18, −24, 0).
    DampedSpring spring(10, 0, 2);
    Vector3 force = spring.forceOnFirst(Vector3(3, 4, 0), origin, origin, origin);
    EXPECT(isApproximatelyEqual(force, Vector3(-18, -24, 0), 1e-4f));
}

TEST(dampingActsOnlyAlongTheSpring) {
    // k = 0, c = 4. first (2,0,0) with velocity (3,5,0), second at the origin with velocity
    // (1,0,0): relative velocity (2,5,0); along d̂ = (1,0,0) it is 2, so the lengthening
    // speed is 2 and F = −4·2·d̂ = (−8, 0, 0). The sideways 5 m/s is not damped.
    DampedSpring spring(0, 4, 1);
    Vector3 force = spring.forceOnFirst(Vector3(2, 0, 0), Vector3(3, 5, 0), origin, Vector3(1, 0, 0));
    EXPECT(isApproximatelyEqual(force, Vector3(-8, 0, 0), 1e-5f));
}

TEST(springAndDamperAddUp) {
    // k = 100, c = 4, rest 1, first at (3,0,0) moving at (2,0,0), second at rest:
    // F = −(100·2 + 4·2)·d̂ = (−208, 0, 0).
    DampedSpring spring(100, 4, 1);
    Vector3 force = spring.forceOnFirst(Vector3(3, 0, 0), Vector3(2, 0, 0), origin, origin);
    EXPECT(isApproximatelyEqual(force, Vector3(-208, 0, 0), 1e-3f));
}

TEST(theTwoEndsFeelOppositeForces) {
    DampedSpring spring(37, 3, 1.5f);
    Vector3 firstPosition(1, -2, 0.5f), secondPosition(-0.5f, 1, 2);
    Vector3 firstVelocity(0.3f, 1, -2), secondVelocity(-1, 0.2f, 0.4f);
    Vector3 onFirst = spring.forceOnFirst(firstPosition, firstVelocity, secondPosition, secondVelocity);
    Vector3 onSecond = spring.forceOnFirst(secondPosition, secondVelocity, firstPosition, firstVelocity);
    EXPECT(isApproximatelyEqual(onFirst + onSecond, origin, 1e-4f));
    EXPECT(onFirst.length() > 1); // the check above is not about two zeros
}

TEST(coincidentPointsHaveNoDirectionAndNoForce) {
    DampedSpring spring(100, 4, 1);
    Vector3 point(2, 3, 4);
    EXPECT(spring.forceOnFirst(point, Vector3(1, 0, 0), point, origin) == origin);
}

TEST(aNaNPositionGivesNaNForce) {
    DampedSpring spring(100, 4, 1);
    Vector3 force = spring.forceOnFirst(Vector3(notANumber, 0, 0), origin, origin, origin);
    EXPECT(std::isnan(force.x));
}

TEST(storedEnergyIsHalfKTimesStretchSquared) {
    // k = 100, rest 1, length 1.2: ½·100·0.2² = 2. Compressed by the same 0.2: also 2.
    DampedSpring spring(100, 0, 1);
    EXPECT(isApproximatelyEqual(spring.storedEnergy(1.2f), 2, 1e-5f));
    EXPECT(isApproximatelyEqual(spring.storedEnergy(0.8f), 2, 1e-5f));
    EXPECT(spring.storedEnergy(1) == 0);
}

TEST(theForceIsTheNegativeSlopeOfTheStoredEnergy) {
    // F = −dU/dL for the spring part: U(L) = ½k(L − rest)², so −dU/dL = −k(L − rest).
    // Central difference with h = 1e-2 around L = 2.3 (k = 50, rest 1): −50·1.3 = −65.
    DampedSpring spring(50, 0, 1);
    float step = 1e-2f;
    float slope = (spring.storedEnergy(2.3f + step) - spring.storedEnergy(2.3f - step)) / (2 * step);
    Vector3 force = spring.forceOnFirst(Vector3(2.3f, 0, 0), origin, origin, origin);
    EXPECT(isApproximatelyEqual(force.x, -slope, 2e-3f));
    EXPECT(isApproximatelyEqual(force.x, -65, 1e-4f));
}

// MARK: The damper only removes energy

TEST(theDamperAloneNeverDoesPositiveWork) {
    // With k = 0 the power on the pair is F·(v_first − v_second) = −c·(lengthening speed)²,
    // never positive. Checked over a grid of 5³ relative velocities and 3 directions.
    DampedSpring spring(0, 2.5f, 1);
    std::array<float, 5> speeds = {-3, -1, 0, 0.5f, 2};
    int checked = 0;
    for (Vector3 secondPosition : {Vector3(1, 0, 0), Vector3(-2, 1, 0.5f), Vector3(0, 0, -3)}) {
        for (float x : speeds) {
            for (float y : speeds) {
                for (float z : speeds) {
                    Vector3 relativeVelocity(x, y, z);
                    Vector3 force = spring.forceOnFirst(origin, relativeVelocity, secondPosition, origin);
                    EXPECT(force.dot(relativeVelocity) <= 1e-5f);
                    checked += 1;
                }
            }
        }
    }
    EXPECT(checked == 375);
}

namespace {

struct EnergyRun {
    float start;
    float end;
    float largestIncrease;
};

/// Steps a unit mass on a spring (k = 20, c = 2, rest 1) fixed to the origin, 5000 times with
/// dt = 0.002 (10 s) by velocity Verlet, and returns the starting energy, the final energy
/// and the largest step-to-step increase of the total (kinetic + stored) energy.
EnergyRun simulate(ParticleState state) {
    DampedSpring spring(20, 2, 1);
    auto energy = [&spring](const ParticleState& current) {
        return current.kineticEnergy(1) + spring.storedEnergy(current.position.length());
    };
    float start = energy(state);
    float previous = start;
    float largestIncrease = 0;
    for (int index = 0; index < 5000; index += 1) {
        state = state.steppedByVelocityVerlet(0.002f, [&spring](Vector3 position, Vector3 velocity) {
            return spring.forceOnFirst(position, velocity, Vector3(0, 0, 0), Vector3(0, 0, 0));
        });
        float current = energy(state);
        largestIncrease = std::max(largestIncrease, current - previous);
        previous = current;
    }
    return {start, previous, largestIncrease};
}

} // namespace

TEST(aSimulatedDampedSpringNeverGainsEnergy) {
    // Started at (1.5, 0.5, 0) moving radially at (0.3, 0.1, 0): the mass slides along one
    // line, so the damper acts on all of its motion and the energy runs down to nothing.
    // Measured: the largest step-to-step increase is 2.4e-7 (7e-8 of the starting energy 3.43,
    // pure rounding noise) and the final energy is 1.9e-9 of the start. Allow an increase of
    // up to 1e-6 of the start (7x margin) and require the final energy under 1e-8 of it.
    EnergyRun run = simulate(ParticleState(Vector3(1.5f, 0.5f, 0), Vector3(0.3f, 0.1f, 0)));
    EXPECT(run.largestIncrease <= 1e-6f * run.start);
    EXPECT(run.end < 1e-8f * run.start);
}

TEST(aSidewaysSwingStillNeverGainsEnergy) {
    // Started at the same point but moving at (0, 1, 0): the swing around the origin keeps
    // its angular momentum (the damper only brakes the stretching), so some energy stays
    // behind as a slow circle (measured: 3.88 falls to 1.03); the total must never rise
    // (measured largest increase 4.8e-7 = 1.2e-7 of the start).
    EnergyRun run = simulate(ParticleState(Vector3(1.5f, 0.5f, 0), Vector3(0, 1, 0)));
    EXPECT(run.largestIncrease <= 1e-6f * run.start);
    EXPECT(run.end < run.start);
}

// MARK: Spring scale

TEST(theForceScalesWithTheLengthUnit) {
    // Multiplying every length and velocity by `scale` (rest length too) multiplies the
    // stretch, the lengthening speed and so the force by `scale`. Base force worked by hand:
    // first (3,4,0) moving (1,2,0), second origin at rest, k = 10, c = 2, rest 2: L = 5,
    // d̂ = (0.6, 0.8, 0), lengthening speed = 0.6 + 1.6 = 2.2;
    // F = −(10·3 + 2·2.2)·d̂ = −34.4·(0.6, 0.8, 0) = (−20.64, −27.52, 0).
    for (float scale : {1e-3f, 1.0f, 1000.0f}) {
        DampedSpring spring(10, 2, 2 * scale);
        Vector3 force = spring.forceOnFirst(Vector3(3, 4, 0) * scale, Vector3(1, 2, 0) * scale, origin, origin);
        // Measured error relative to |F|: 5.4e-8 at 1 mm, 7.8e-8 at 1 m, 0 at 1 km; tolerance
        // 3e-7 relative (3.8x margin).
        EXPECT(isApproximatelyEqual(force, Vector3(-20.64f, -27.52f, 0) * scale, 3e-7f * 34.4f * scale));
    }
}

TEST(zeroCoefficientsAreAllowed) {
    // Negative stiffness, damping or rest length would add energy, so the constructor asserts
    // on them (which a test cannot catch). The boundary itself is legal.
    DampedSpring boundary(0, 0, 0);
    EXPECT(boundary.storedEnergy(5) == 0);
}

// MARK: Damped oscillator

namespace {

// The reference oscillator: m = 4, k = 16 gives ω₀ = √(16/4) = 2 and 2√(k·m) = 2·8 = 16,
// so ζ = c/16: c = 8 is ζ = 0.5, c = 16 is ζ = 1, c = 32 is ζ = 2.
DampedOscillator oscillator(float damping, float position = 1, float velocity = 0) {
    return DampedOscillator(4, 16, damping, position, velocity);
}

/// The derivative of `function` at `time` by the five-point stencil
/// `(−f(t + 2h) + 8f(t + h) − 8f(t − h) + f(t − 2h)) / 12h`, whose error is of order `h⁴`,
/// so a step of 0.05 is accurate far beyond what Float can show in the values themselves.
float derivative(float time, const std::function<float(float)>& function, float step = 0.05f) {
    return (-function(time + 2 * step) + 8 * function(time + step) - 8 * function(time - step)
            + function(time - 2 * step))
         / (12 * step);
}

/// Swift's `stride(from: 0.5, through: 3, by: 0.05)`: each value is `start + index × step`.
std::vector<float> stride(float start, float through, float step) {
    std::vector<float> values;
    for (int index = 0; start + static_cast<float>(index) * step <= through; index += 1) {
        values.push_back(start + static_cast<float>(index) * step);
    }
    return values;
}

} // namespace

TEST(naturalFrequencyAndDampingRatioMatchTheirFormulas) {
    EXPECT(isApproximatelyEqual(oscillator(8).naturalFrequency(), 2, 1e-6f));
    EXPECT(isApproximatelyEqual(oscillator(8).dampingRatio(), 0.5f, 1e-6f));
    EXPECT(isApproximatelyEqual(oscillator(16).dampingRatio(), 1, 1e-6f));
    EXPECT(isApproximatelyEqual(oscillator(32).dampingRatio(), 2, 1e-6f));
    EXPECT(oscillator(0).dampingRatio() == 0);
    EXPECT(oscillator(8).regime() == DampingRegime::underdamped);
    EXPECT(oscillator(16).regime() == DampingRegime::criticallyDamped);
    EXPECT(oscillator(32).regime() == DampingRegime::overdamped);
}

TEST(undampedOscillatorIsACosine) {
    // c = 0, x₀ = 1, v₀ = 0, ω₀ = 2: x = cos 2t. At t = π/4: cos(π/2) = 0; at t = π/2: −1.
    DampedOscillator free = oscillator(0);
    EXPECT(isApproximatelyEqual(free.positionAt(0.78539816f), 0, 1e-6f));
    EXPECT(isApproximatelyEqual(free.positionAt(1.5707963f), -1, 1e-6f));
    EXPECT(isApproximatelyEqual(free.velocityAt(0.78539816f), -2, 1e-6f)); // −2 sin(π/2)
}

TEST(underdampedValueMatchesTheHandWorkedOne) {
    // ζ = 0.5: ω_d = 2√(1 − 0.25) = 1.7320508. x(1) = e^{−1}(cos 1.7320508 + sin 1.7320508 / 1.7320508)
    // = 0.36787944 · (−0.16055654 + 0.98702664/1.7320508) = 0.36787944 · 0.40931 = 0.15057437.
    EXPECT(isApproximatelyEqual(oscillator(8).positionAt(1), 0.15057437f, 1e-6f));
}

TEST(criticallyDampedValueMatchesTheHandWorkedOnes) {
    // x₀ = 1, v₀ = 0: x = e^{−2t}(1 + 2t); x(1) = e^{−2}·3 = 0.13533528 · 3 = 0.40600585.
    EXPECT(isApproximatelyEqual(oscillator(16).positionAt(1), 0.40600585f, 1e-6f));
    // x₀ = 0, v₀ = 1: x = t·e^{−2t}; x(0.5) = 0.5 · e^{−1} = 0.18393972.
    DampedOscillator kicked = oscillator(16, 0, 1);
    EXPECT(isApproximatelyEqual(kicked.positionAt(0.5f), 0.18393972f, 1e-6f));
}

TEST(overdampedValueMatchesTheHandWorkedOne) {
    // ζ = 2: s = 2√3 = 3.4641016, r₁ = −4 + s = −0.5358984, r₂ = −4 − s = −7.4641016.
    // x₀ = 1, v₀ = 0: A = (0 − r₂)/(r₁ − r₂) = 7.4641016/6.9282032 = 1.0773503, B = r₁/(r₁ − r₂) = −0.0773503.
    // x(1) = 1.0773503·e^{−0.5358984} − 0.0773503·e^{−7.4641016}
    //      = 1.0773503 · 0.5851434 − 0.0773503 · 0.00057 = 0.63036 (more digits: 0.6303600).
    EXPECT(isApproximatelyEqual(oscillator(32).positionAt(1), 0.63036002f, 1e-6f));
}

TEST(startsWhereItWasLet) {
    // x(0) = x₀ and x′(0) = v₀ in every regime, including the two ratios next to critical.
    for (float damping : {0.0f, 8.0f, 16.0f, 32.0f, 15.999999f, 16.000001f}) {
        DampedOscillator start = oscillator(damping, 0.7f, -1.3f);
        EXPECT(isApproximatelyEqual(start.positionAt(0), 0.7f, 1e-6f));
        EXPECT(isApproximatelyEqual(start.velocityAt(0), -1.3f, 1e-6f));
    }
}

TEST(satisfiesTheEquationOfMotion) {
    // m·x″ + c·x′ + k·x = 0, with x′ from velocityAt and x″ the derivative of it. The
    // residual is measured relative to k·x₀ = 16 for t = 0.5…3. Measured worst: 5e-6 (c = 0),
    // 2.9e-6 (8), 6.3e-6 (16), 3.3e-5 (32, the fast mode e^{−7.5t} is the hardest for the
    // stencil and for Float). Bound 6e-5.
    for (float damping : {0.0f, 8.0f, 16.0f, 32.0f}) {
        DampedOscillator system = oscillator(damping, 1, 0.5f);
        float worst = 0;
        for (float time : stride(0.5f, 3, 0.05f)) {
            float acceleration = derivative(time, [&system](float at) { return system.velocityAt(at); });
            float residual = 4 * acceleration + damping * system.velocityAt(time) + 16 * system.positionAt(time);
            worst = std::max(worst, std::fabs(residual) / 16);
        }
        EXPECT(worst < 6e-5f);
    }
}

TEST(velocityIsTheDerivativeOfPosition) {
    // Worst difference between the stencil slope of position and velocityAt, t = 0.5…3:
    // 9e-6 (c = 0), 1.7e-6 (8), 2.0e-5 (16), 1.8e-5 (32). Bound 3e-5.
    for (float damping : {0.0f, 8.0f, 16.0f, 32.0f}) {
        DampedOscillator system = oscillator(damping, 1, 0.5f);
        float worst = 0;
        for (float time : stride(0.5f, 3, 0.05f)) {
            float slope = derivative(time, [&system](float at) { return system.positionAt(at); });
            worst = std::max(worst, std::fabs(slope - system.velocityAt(time)));
        }
        EXPECT(worst < 3e-5f);
    }
}

TEST(aRatioWithinRoundingOfCriticalIsTreatedAsCritical) {
    // m = 1, k = 50, c = Float(2√50) = 14.142135620117188: ζ − 1 = −2.6e-10, closer to 1
    // than the under- and overdamped formulas can be evaluated without cancelling. It must
    // take the critical branch: ω₀ = √50 = 7.0710678, x = e^{−ω₀t}(1 + ω₀t);
    // x(0.2) = e^{−1.4142136}·2.4142136 = 0.24311673·2.4142136 = 0.58693572.
    DampedOscillator nearlyCritical(1, 50, 14.142135620117188f, 1, 0);
    EXPECT(nearlyCritical.regime() == DampingRegime::criticallyDamped);
    EXPECT(isApproximatelyEqual(nearlyCritical.positionAt(0.2f), 0.58693572f, 1e-6f));
}

TEST(theRegimesAgreeAcrossTheirBoundary) {
    // The formulas on both sides of ζ = 1 meet continuously: ζ = 1 ± 1e-6 differ from the
    // critical curve by about 3e-7 at most (measured 3.9e-7 over t = 0.3, 1, 2.5); tolerance 1e-6.
    DampedOscillator critical = oscillator(16);
    DampedOscillator under = oscillator(15.999984f); // ζ = 0.999999
    DampedOscillator over = oscillator(16.000016f);  // ζ = 1.000001
    for (float time : {0.3f, 1.0f, 2.5f}) {
        EXPECT(isApproximatelyEqual(under.positionAt(time), critical.positionAt(time), 1e-6f));
        EXPECT(isApproximatelyEqual(over.positionAt(time), critical.positionAt(time), 1e-6f));
    }
}

TEST(energyIsConstantWithoutDampingAndFallsWithIt) {
    // Undamped: ½·4·v² + ½·16·x² = 8 (x₀ = 1, v₀ = 0) at every time.
    DampedOscillator free = oscillator(0);
    for (float time : {0.0f, 0.37f, 1.0f, 4.0f, 20.0f}) {
        EXPECT(isApproximatelyEqual(free.energyAt(time), 8, 2e-5f));
    }
    for (float damping : {8.0f, 16.0f, 32.0f}) {
        DampedOscillator system = oscillator(damping);
        float previous = system.energyAt(0);
        EXPECT(isApproximatelyEqual(previous, 8, 1e-5f));
        for (float time : stride(0.05f, 10, 0.05f)) {
            float current = system.energyAt(time);
            EXPECT(current <= previous + 1e-6f);
            previous = current;
        }
    }
}

TEST(aScaledOscillatorMovesScaledDistances) {
    // Same motion with the start position (and velocity) multiplied by `scale`: x scales
    // linearly because the equation is linear. Relative tolerance 2e-7 (measured 0: the closed
    // form runs in Double and the only rounding is the final Float, identical at every scale;
    // 2e-7 is 3 ulp).
    for (float scale : {1e-3f, 1.0f, 1000.0f}) {
        DampedOscillator base = oscillator(8, 1, 0.5f);
        DampedOscillator scaled = oscillator(8, scale, 0.5f * scale);
        for (float time : {0.4f, 1.7f, 3.0f}) {
            EXPECT(isApproximatelyEqual(scaled.positionAt(time), base.positionAt(time) * scale, 2e-7f * scale));
        }
    }
}

// MARK: Mass properties

namespace {

/// The diagonal of a tensor, `(Iₓₓ, I_yy, I_zz)`.
Vector3 diagonalOf(const Matrix3& matrix) {
    return Vector3(matrix.entry(0, 0), matrix.entry(1, 1), matrix.entry(2, 2));
}

bool isSymmetric(const Matrix3& matrix, float tolerance) {
    return isApproximatelyEqual(matrix, matrix.transposed(), tolerance);
}

/// Sylvester's criterion: a symmetric matrix is positive definite when its leading minors
/// (top-left 1×1, 2×2 and the whole determinant) are all positive.
bool isPositiveDefinite(const Matrix3& matrix) {
    float first = matrix.entry(0, 0);
    float second = first * matrix.entry(1, 1) - matrix.entry(0, 1) * matrix.entry(1, 0);
    return first > 0 && second > 0 && matrix.determinant() > 0;
}

/// The solids used by the property tests, each of mass 12, at a size of `scale`.
std::vector<Matrix3> tensors(float scale) {
    float mass = 12 * scale * scale * scale;
    return {
        MassProperties::solidSphereInertia(mass, 1.5f * scale),
        MassProperties::solidBoxInertia(mass, Vector3(2, 3, 4) * scale),
        MassProperties::solidBoxInertia(mass, Vector3(0.1f, 5, 7) * scale),
        MassProperties::solidCylinderInertia(mass, scale, 3 * scale),
        MassProperties::solidCylinderInertia(mass, 2 * scale, 0.2f * scale),
    };
}

Matrix3 rowSum(const Matrix3& first, const Matrix3& second) {
    auto firstRows = first.rows();
    auto secondRows = second.rows();
    return Matrix3(firstRows[0] + secondRows[0], firstRows[1] + secondRows[1], firstRows[2] + secondRows[2]);
}

Matrix3 rowDifference(const Matrix3& first, const Matrix3& second) {
    auto firstRows = first.rows();
    auto secondRows = second.rows();
    return Matrix3(firstRows[0] - secondRows[0], firstRows[1] - secondRows[1], firstRows[2] - secondRows[2]);
}

} // namespace

// MARK: Volumes and mass

TEST(volumesMatchTheHandWorkedOnes) {
    // Sphere r = 3: 4/3·π·27 = 36π = 113.097336.
    EXPECT(isApproximatelyEqual(MassProperties::sphereVolume(3), 113.097336f, 1e-4f));
    // Box 2 × 3 × 4 = 24.
    EXPECT(isApproximatelyEqual(MassProperties::boxVolume(Vector3(2, 3, 4)), 24, 1e-6f));
    // Cylinder r = 2, h = 3: π·4·3 = 12π = 37.699112.
    EXPECT(isApproximatelyEqual(MassProperties::cylinderVolume(2, 3), 37.699112f, 1e-4f));
    // A flat solid has no volume.
    EXPECT(MassProperties::boxVolume(Vector3(2, 0, 4)) == 0);
    EXPECT(MassProperties::sphereVolume(0) == 0);
}

TEST(massIsDensityTimesVolume) {
    // Water (1000 kg/m³) filling 0.002 m³ (two litres) weighs 2 kg.
    EXPECT(isApproximatelyEqual(MassProperties::mass(1000, 0.002f), 2, 1e-6f));
    // A 1 m radius ball of 7800 kg/m³ steel: 7800 · 4.1887902 = 32672.564 kg.
    float steelBall = MassProperties::mass(7800, MassProperties::sphereVolume(1));
    EXPECT(isApproximatelyEqual(steelBall, 32672.564f, 0.01f));
    EXPECT(MassProperties::mass(0, 5) == 0);
}

// MARK: Inertia tensors worked by hand

TEST(solidSphere) {
    // m = 5, r = 2: 2/5 · 5 · 4 = 8 about every axis.
    Matrix3 inertia = MassProperties::solidSphereInertia(5, 2);
    EXPECT(isApproximatelyEqual(inertia, Matrix3::diagonal(Vector3(8, 8, 8)), 1e-5f));
}

TEST(boxOf2By3By4WithMass12) {
    // m/12 = 1. Iₓₓ = 3² + 4² = 25, I_yy = 4² + 2² = 20, I_zz = 2² + 3² = 13.
    Matrix3 inertia = MassProperties::solidBoxInertia(12, Vector3(2, 3, 4));
    EXPECT(isApproximatelyEqual(inertia, Matrix3::diagonal(Vector3(25, 20, 13)), 1e-4f));
    // The cube of edge 1 and mass 6: m/12·(1 + 1) = 1 about every axis (= m s²/6).
    Matrix3 cube = MassProperties::solidBoxInertia(6, Vector3(1, 1, 1));
    EXPECT(isApproximatelyEqual(cube, Matrix3::diagonal(Vector3(1, 1, 1)), 1e-6f));
}

TEST(cylinderAlongZ) {
    // m = 12, r = 2, h = 3: I_zz = ½·12·4 = 24; Iₓₓ = I_yy = 12/12·(3·4 + 9) = 21.
    Matrix3 inertia = MassProperties::solidCylinderInertia(12, 2, 3);
    EXPECT(isApproximatelyEqual(inertia, Matrix3::diagonal(Vector3(21, 21, 24)), 1e-4f));
    // m = 6, r = 1, h = 2: I_zz = 3; Iₓₓ = 6/12·(3 + 4) = 3.5.
    Matrix3 second = MassProperties::solidCylinderInertia(6, 1, 2);
    EXPECT(isApproximatelyEqual(second, Matrix3::diagonal(Vector3(3.5f, 3.5f, 3)), 1e-5f));
}

TEST(aMasslessOrZeroSizeSolidHasNoInertia) {
    Matrix3 zero = Matrix3::diagonal(Vector3(0, 0, 0));
    EXPECT(isApproximatelyEqual(MassProperties::solidSphereInertia(0, 2), zero, 0));
    EXPECT(isApproximatelyEqual(MassProperties::solidBoxInertia(0, Vector3(2, 3, 4)), zero, 0));
    EXPECT(isApproximatelyEqual(MassProperties::solidCylinderInertia(0, 1, 1), zero, 0));
    EXPECT(isApproximatelyEqual(MassProperties::solidSphereInertia(5, 0), zero, 0));
}

// MARK: Properties every physical tensor has

TEST(tensorsAreSymmetricAndPositiveDefinite) {
    for (const Matrix3& tensor : tensors(1)) {
        EXPECT(isSymmetric(tensor, 0));
        EXPECT(isPositiveDefinite(tensor));
    }
}

TEST(principalMomentsObeyTheTriangleInequality) {
    // Iₓₓ + I_yy ≥ I_zz (and the other two): Iₓₓ + I_yy − I_zz = 2∫ρz² dV ≥ 0. Equality is
    // a flat plate, so the slack is checked to be positive for these solid shapes.
    for (const Matrix3& tensor : tensors(1)) {
        Vector3 moments = diagonalOf(tensor);
        EXPECT(moments.x + moments.y > moments.z);
        EXPECT(moments.y + moments.z > moments.x);
        EXPECT(moments.z + moments.x > moments.y);
    }
    // The thin plate (box 4 × 5 × 0): equality, the limit case: m/12·(25, 16, 41); 25 + 16 = 41.
    Vector3 plate = diagonalOf(MassProperties::solidBoxInertia(12, Vector3(4, 5, 0)));
    EXPECT(isApproximatelyEqual(plate.x + plate.y, plate.z, 1e-4f));
}

TEST(aLongerBoxEdgeSpinsMoreEasilyAboutItsOwnAxis) {
    // Shape sanity: the axis along the longest edge has the smallest moment.
    Vector3 moments = diagonalOf(MassProperties::solidBoxInertia(12, Vector3(2, 3, 4)));
    EXPECT(moments.z < moments.y && moments.y < moments.x);
}

TEST(inertiaGrowsWithTheFifthPowerOfSize) {
    // Same density, every length ×s: mass ×s³ and the squared lengths ×s², so I ×s⁵.
    // 1 mm, 1 m and 1 km, relative tolerance 3e-7 (measured worst 8.8e-8 at 1 mm, exactly 0 at
    // 1 km; margin 3.4x).
    std::vector<Matrix3> base = tensors(1);
    for (float scale : {1e-3f, 1000.0f}) {
        std::vector<Matrix3> scaled = tensors(scale);
        float factor = scale * scale * scale * scale * scale;
        for (size_t index = 0; index < scaled.size(); index += 1) {
            float difference = (diagonalOf(scaled[index]) / factor - diagonalOf(base[index])).length();
            EXPECT(difference <= 3e-7f * diagonalOf(base[index]).length());
        }
    }
}

// MARK: Parallel axis

TEST(parallelAxisAlongAnAxisOfASphere) {
    // m = 5, I = 8·1, offset (3,0,0): |r|² = 9. I' = 8·1 + 5·(9·1 − r rᵀ):
    // xx: 8 + 5·(9 − 9) = 8; yy: 8 + 5·9 = 53; zz: 53; off-diagonals −5·0 = 0.
    Matrix3 sphere = MassProperties::solidSphereInertia(5, 2);
    Matrix3 moved = MassProperties::parallelAxis(sphere, 5, Vector3(3, 0, 0));
    EXPECT(isApproximatelyEqual(moved, Matrix3::diagonal(Vector3(8, 53, 53)), 1e-4f));
}

TEST(parallelAxisInAnObliqueDirection) {
    // I = 2·1, m = 1, r = (1,2,2): |r|² = 9, r rᵀ rows (1,2,2), (2,4,4), (2,4,4).
    // I' = 2·1 + 9·1 − r rᵀ:
    //   row x: (2 + 9 − 1, −2, −2) = (10, −2, −2)
    //   row y: (−2, 2 + 9 − 4, −4) = (−2, 7, −4)
    //   row z: (−2, −4, 7)
    Matrix3 moved = MassProperties::parallelAxis(Matrix3::diagonal(Vector3(2, 2, 2)), 1, Vector3(1, 2, 2));
    Matrix3 expected(Vector3(10, -2, -2), Vector3(-2, 7, -4), Vector3(-2, -4, 7));
    EXPECT(isApproximatelyEqual(moved, expected, 1e-5f));
    // Same trace identity: trace' = trace + 2·m·|r|² = 6 + 18 = 24 = 10 + 7 + 7.
    Vector3 moments = diagonalOf(moved);
    EXPECT(isApproximatelyEqual(moments.x + moments.y + moments.z, 24, 1e-5f));
    // The axis through the new point parallel to r has the same moment as before:
    // I'·r = I·r = 2·r = (2, 4, 4).
    EXPECT(isApproximatelyEqual(moved.transform(Vector3(1, 2, 2)), Vector3(2, 4, 4), 1e-5f));
}

TEST(parallelAxisWithZeroOffsetChangesNothing) {
    for (const Matrix3& tensor : tensors(1)) {
        EXPECT(isApproximatelyEqual(MassProperties::parallelAxis(tensor, 12, Vector3(0, 0, 0)), tensor, 0));
    }
}

TEST(parallelAxisKeepsTheTensorSymmetricPositiveAndGrowing) {
    std::vector<Vector3> offsets = {Vector3(1, 0, 0), Vector3(0.5f, -2, 3), Vector3(-1, -1, -1)};
    for (const Matrix3& tensor : tensors(1)) {
        for (Vector3 offset : offsets) {
            Matrix3 moved = MassProperties::parallelAxis(tensor, 12, offset);
            EXPECT(isSymmetric(moved, 0));
            EXPECT(isPositiveDefinite(moved));
            // The moment about every axis through the new point is at least the old one:
            // checked on the three coordinate axes and the offset direction.
            for (Vector3 axis : {Vector3::xAxis(), Vector3::yAxis(), Vector3::zAxis(), offset.normalized()}) {
                EXPECT(axis.dot(moved.transform(axis)) >= axis.dot(tensor.transform(axis)) - 1e-4f);
            }
        }
    }
}

TEST(parallelAxisTermCanBeSubtractedToGoBack) {
    // Moving by r and then by −r does NOT return (both add m(|r|²1 − r rᵀ)); the tensor about
    // the centre is recovered by subtracting the term. Here the term is the parallel-axis
    // result for a zero tensor. Oblique example (m = 1, r = (1,2,2)): subtract (9·1 − r rᵀ).
    Matrix3 centre = Matrix3::diagonal(Vector3(2, 2, 2));
    Matrix3 moved = MassProperties::parallelAxis(centre, 1, Vector3(1, 2, 2));
    Matrix3 term = MassProperties::parallelAxis(Matrix3::diagonal(Vector3(0, 0, 0)), 1, Vector3(1, 2, 2));
    EXPECT(isApproximatelyEqual(rowDifference(moved, term), centre, 1e-5f));
}

TEST(aBoxMadeOfTwoHalvesGivesTheWholeBox) {
    // A 2 × 3 × 4 box of mass 12 cut in two halves at x = 0: each is a 1 × 3 × 4 box of
    // mass 6 centred at x = ±0.5. Parallel-axis each half to the common centre and add:
    // the sum must equal the whole box's (25, 20, 13).
    Matrix3 half = MassProperties::solidBoxInertia(6, Vector3(1, 3, 4));
    Matrix3 left = MassProperties::parallelAxis(half, 6, Vector3(-0.5f, 0, 0));
    Matrix3 right = MassProperties::parallelAxis(half, 6, Vector3(0.5f, 0, 0));
    Matrix3 sum = rowSum(left, right);
    // Iₓₓ: 2·(6/12·(9 + 16)) = 25. I_yy: 2·(6/12·(16 + 1) + 6·0.25) = 2·(8.5 + 1.5) = 20.
    // I_zz: 2·(6/12·(1 + 9) + 6·0.25) = 2·(5 + 1.5) = 13.
    EXPECT(isApproximatelyEqual(sum, Matrix3::diagonal(Vector3(25, 20, 13)), 1e-4f));
}

TEST(parallelAxisScalesWithTheFifthPower) {
    // The oblique example with mass ×s³ and offset ×s: every entry ×s⁵.
    // Tolerance 3e-7 relative to the largest entry (measured worst 8.5e-8 at 1 mm, 0 at 1 m and
    // 1 km; margin 3.5x).
    for (float scale : {1e-3f, 1.0f, 1000.0f}) {
        float mass = scale * scale * scale;
        Matrix3 centre = Matrix3::diagonal(Vector3(2, 2, 2) * (mass * scale * scale));
        Matrix3 moved = MassProperties::parallelAxis(centre, mass, Vector3(1, 2, 2) * scale);
        float factor = mass * scale * scale;
        Matrix3 expected(Vector3(10, -2, -2) * factor, Vector3(-2, 7, -4) * factor, Vector3(-2, -4, 7) * factor);
        EXPECT(isApproximatelyEqual(moved, expected, 3e-7f * 10 * factor));
    }
}
