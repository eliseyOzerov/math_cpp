// Springs: the force between two points that wants to keep them a fixed distance apart, and
// the exact motion of one mass on one spring, for checking a simulation against the truth
// (Engine/Math/Physics/DampedSpring.swift).
#pragma once

#include <cassert>

#include "../Linear/Vector3.h"

namespace editor {

/// A spring with damping between two points: Hooke's law along the line joining them, plus a
/// brake on how fast that line is lengthening or shortening.
///
/// Sources: Hooke (1678) for the spring; standard mechanics, e.g. Goldstein, *Classical
/// Mechanics* (3rd ed., 2002) §1 and any engineering table for the viscous damper.
///
/// Conventions: Z-up, right-handed; metres, newtons and seconds. `stiffness` `k` is in
/// newtons per metre, `damping` `c` in newton-seconds per metre, `restLength` in metres.
/// Both coefficients and the rest length are zero or positive (a negative one would add
/// energy; the constructor rejects them, and NaN, with an assert).
///
/// Use when: connecting two particles of a cloth, rope, soft body or suspension
/// Tags: spring, Hooke's law, damper, spring damper, dashpot, constraint, cloth spring, rope
/// Node: none (simulation)
struct DampedSpring {
    /// How hard the spring pulls per metre of stretch, `k` in newtons per metre.
    const float stiffness;
    /// How hard the damper resists per metre per second of lengthening, `c` in N·s/m.
    const float damping;
    /// The length at which the spring pulls with no force, in metres.
    const float restLength;

    /// A spring with the given coefficients; each must be zero or positive.
    DampedSpring(float stiffness, float damping, float restLength)
        : stiffness(stiffness), damping(damping), restLength(restLength) {
        assert(stiffness >= 0 && "The stiffness must be zero or positive.");
        assert(damping >= 0 && "The damping must be zero or positive.");
        assert(restLength >= 0 && "The rest length must be zero or positive.");
    }

    /// The force the spring puts on its **first** end point; the second end point feels the
    /// exact opposite (Newton's third law), so the pair never changes the total momentum.
    ///
    /// Formula: let `d = firstPosition − secondPosition`, `L = |d|`, `d̂ = d / L` (the
    /// direction from the second point toward the first) and
    /// `lengtheningSpeed = (firstVelocity − secondVelocity)·d̂`, the rate at which `L`
    /// grows. Then
    ///
    ///     F = −( k·(L − restLength) + c·lengtheningSpeed ) · d̂
    ///
    /// The first term is Hooke's law: stretched (`L > rest`) pulls the first point toward
    /// the second, compressed pushes it away. The second is the damper: it acts only along
    /// the spring, against the lengthening. Sideways motion (a pendulum swing, a spin) is not
    /// damped, as in a real dashpot.
    ///
    ///     second ●─────────── ● first  ──▶ d̂
    ///                 F points back along −d̂ when stretched
    ///
    /// Edge cases: exactly coincident points have no direction `d̂`, so the force is zero
    /// (a spring of rest length 0 is then at rest, which is correct; a spring of positive
    /// rest length gets no push to separate, which is the caller's to handle). A NaN position
    /// gives NaN; a NaN velocity gives NaN unless the points coincide, when the force is 0
    /// before the velocity is read.
    ///
    /// Use when: computing what two cloth particles or a suspension arm pull on each other
    /// Tags: Hooke's law, spring force, damper force, spring-damper, dashpot, -kx, cloth
    ///   spring, damping along the spring
    /// Node: none (simulation)
    Vector3 forceOnFirst(Vector3 firstPosition, Vector3 firstVelocity, Vector3 secondPosition, Vector3 secondVelocity) const;

    /// The energy stored in the spring at a given length, `½ k (L − restLength)²`, in joules.
    ///
    /// Formula: the work done stretching from rest, `∫ k·x dx = ½ k x²` with
    /// `x = length − restLength`. It is the same for stretching and for compressing by the
    /// same amount, and 0 at the rest length.
    ///
    /// Use when: checking that a spring simulation does not gain energy; showing strain
    /// Tags: spring energy, potential energy, elastic energy, half k x squared, strain energy
    /// Node: none (simulation)
    float storedEnergy(float atLength) const;
};

/// How a damped oscillator returns to rest, told apart by the damping ratio `ζ`.
///
/// Use when: telling a ringing spring from a creeping one; choosing how to tune a damper
/// Tags: damping regime, underdamped, critically damped, overdamped, damping ratio class
enum class DampingRegime {
    /// `ζ < 1`: it rings, swinging back and forth with a shrinking amplitude.
    underdamped,
    /// `ζ = 1`: the fastest return to rest without swinging past it.
    criticallyDamped,
    /// `ζ > 1`: a slow creep back to rest, no swinging.
    overdamped,
};

/// The **exact** motion of one mass on one damped spring, `m·x″ + c·x′ + k·x = 0`, from a
/// given start. It is what a simulation of that spring should converge to, so tests and
/// pictures compare against it.
///
/// Source: Thornton & Marion, *Classical Dynamics of Particles and Systems* (5th ed., 2004)
/// §3.5 (damped oscillations); any differential equations text (the characteristic equation `m r² + c r + k = 0`).
///
/// Derivation: try `x = e^{rt}`. Then `m r² + c r + k = 0`, so
/// `r = −ζω₀ ± ω₀√(ζ² − 1)` where `ω₀ = √(k/m)` and `ζ = c / (2√(k·m))`. Three cases:
///
///     ζ < 1  (under):    x = e^{−ζω₀t} · ( x₀ cos ω_d t + (v₀ + ζω₀x₀)/ω_d · sin ω_d t ),
///                        ω_d = ω₀√(1 − ζ²)
///     ζ = 1  (critical): x = e^{−ω₀t} · ( x₀ + (v₀ + ω₀x₀) t )
///     ζ > 1  (over):     x = A e^{r₁t} + B e^{r₂t},  r₁,₂ = −ζω₀ ± ω₀√(ζ² − 1),
///                        A = (v₀ − r₂x₀)/(r₁ − r₂),  B = (r₁x₀ − v₀)/(r₁ − r₂)
///
/// Each satisfies `x(0) = x₀` and `x′(0) = v₀`. The values are computed in `double` and
/// rounded to `float` at the end, because near `ζ = 1` the over-damped form subtracts two
/// nearly equal exponentials; a ratio within `10⁻⁹` of 1 is treated as critical.
///
/// Conventions: metres, kilograms, seconds, newtons per metre. `mass` and `stiffness` must
/// be positive (with no spring there is no oscillator) and `damping` zero or positive; the
/// constructor enforces this with asserts. `time` is in seconds, and a negative time
/// extends the same solution backward.
///
/// Swift's `position(at:)`, `velocity(at:)` and `energy(at:)` are `positionAt`, `velocityAt`
/// and `energyAt`.
///
/// Use when: checking an integrator or a spring against the exact answer; animating a door
///   closer or a camera easing to a target without any simulation
/// Tags: damped harmonic oscillator, damped spring, damping ratio, critical damping,
///   underdamped, overdamped, natural frequency, analytic solution, spring animation
/// Node: none (simulation)
struct DampedOscillator {
    /// The moving mass, in kilograms.
    const float mass;
    /// The spring constant `k`, in newtons per metre.
    const float stiffness;
    /// The damping coefficient `c`, in newton-seconds per metre.
    const float damping;
    /// The displacement from rest at time 0, in metres.
    const float initialPosition;
    /// The velocity at time 0, in metres per second.
    const float initialVelocity;

    /// An oscillator of `mass` kg on a spring of `stiffness` N/m with `damping` N·s/m, let go
    /// at `initialPosition` with `initialVelocity`.
    DampedOscillator(float mass, float stiffness, float damping, float initialPosition, float initialVelocity)
        : mass(mass), stiffness(stiffness), damping(damping), initialPosition(initialPosition),
          initialVelocity(initialVelocity) {
        assert(mass > 0 && "The mass must be positive.");
        assert(stiffness > 0 && "The stiffness must be positive.");
        assert(damping >= 0 && "The damping must be zero or positive.");
    }

    /// The natural (undamped) angular frequency `ω₀ = √(k / m)` in radians per second: how
    /// fast the mass would swing if there were no damping. One swing takes `2π/ω₀` seconds.
    ///
    /// Use when: picking a spring that swings at a wanted rate; choosing a stable time step
    /// Tags: natural frequency, angular frequency, omega zero, resonance, sqrt(k/m), period
    /// Node: none (simulation)
    float naturalFrequency() const;

    /// The damping ratio `ζ = c / (2√(k·m))`: damping as a fraction of the amount that
    /// just stops the swinging. Below 1 it rings, at 1 it is critically damped, above 1 it
    /// creeps.
    ///
    /// Use when: tuning a spring-damper so it settles without bouncing; classifying a system
    /// Tags: damping ratio, zeta, critical damping, c over 2 sqrt(km), Q factor, damper
    ///   tuning
    /// Node: none (simulation)
    float dampingRatio() const;

    /// Which of the three behaviours the damping ratio gives (within `10⁻⁹` of 1 counts as
    /// critical, see the type's notes).
    ///
    /// Use when: choosing how to draw or describe the motion; picking the matching formula
    /// Tags: underdamped, overdamped, critically damped, damping regime, classification
    /// Node: none (simulation)
    DampingRegime regime() const;

    /// The displacement from rest at `time` seconds, in metres, by the closed forms in the
    /// type's notes.
    ///
    /// Use when: drawing the true curve next to a simulated one; animating a spring without
    ///   stepping it
    /// Tags: damped oscillator position, x(t), analytic spring, spring animation, closed form
    /// Node: none (simulation)
    float positionAt(float time) const;

    /// The velocity at `time` seconds, in metres per second: the derivative of `positionAt`.
    ///
    /// Use when: computing the exact energy of the oscillator; checking the velocity of a
    ///   simulation
    /// Tags: damped oscillator velocity, x'(t), analytic spring velocity, closed form
    /// Node: none (simulation)
    float velocityAt(float time) const;

    /// The total energy `½ m v² + ½ k x²` at `time` seconds, in joules. It is constant when
    /// `damping` is 0 and falls steadily when it is not.
    ///
    /// Use when: showing that damping removes energy; the reference for an integrator's drift
    /// Tags: oscillator energy, total energy, mechanical energy, energy decay, conservation
    /// Node: none (simulation)
    float energyAt(float time) const;
};

} // namespace editor
