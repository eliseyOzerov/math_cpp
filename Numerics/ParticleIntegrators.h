// Moving a point mass through time: one small step of a numerical integrator at a time
// (Engine/Math/Physics/ParticleIntegrators.swift). Cloth, ropes, soft bodies and falling props
// all advance this way, frame after frame.
#pragma once

#include <functional>

#include "../Linear/Vector3.h"

namespace editor {

/// A point mass at one instant: where it is and how fast it is moving.
///
/// Conventions: Z-up, right-handed. Positions are in metres, velocities in metres per
/// second, accelerations in metres per second squared, and every time step is a duration in
/// **seconds** (a `float`).
///
/// Swift's two `acceleration` overloads (one closure taking position and velocity, one taking
/// position alone) are two overloads taking a `std::function` each.
///
/// Use when: simulating a falling object, a bouncing spring or a piece of cloth, one point at
///   a time
/// Tags: particle, point mass, state, position and velocity, phase space, rigid body state
/// Node: none (simulation)
struct ParticleState {
    /// Where the particle is.
    Vector3 position;
    /// How fast, and in which direction, the particle is moving.
    Vector3 velocity;

    /// A particle at the origin at rest.
    constexpr ParticleState() = default;
    /// A particle at `position` moving with `velocity`.
    constexpr ParticleState(Vector3 position, Vector3 velocity) : position(position), velocity(velocity) {}

    constexpr bool operator==(const ParticleState&) const = default;

    /// The energy of the motion, `½ m |v|²`, in joules for a mass in kilograms.
    ///
    /// Formula: half the mass times the squared speed (`v·v`). It is never negative and is 0
    /// at rest.
    ///
    /// Use when: checking that a simulation conserves (or loses) energy
    /// Tags: kinetic energy, energy, half m v squared, speed squared, conservation
    /// Node: none (simulation)
    float kineticEnergy(float mass) const;

    // MARK: Semi-implicit Euler

    /// The state one `timeStep` later, by **semi-implicit (symplectic) Euler**: the velocity
    /// is updated first from the acceleration, and the new velocity then moves the position.
    ///
    /// Source: Hairer, Lubich & Wanner, *Geometric Numerical Integration* (2003), §I.1.2
    /// (the symplectic Euler method).
    ///
    /// Formula, with `a` the acceleration at the current position and velocity:
    ///
    ///     v₁ = v₀ + a(x₀, v₀)·dt
    ///     x₁ = x₀ + v₁·dt          ← uses the NEW velocity; this is the whole trick
    ///
    /// Why not plain explicit Euler (`x₁ = x₀ + v₀·dt`, `v₁ = v₀ + a·dt`)? On an oscillator
    /// it multiplies the energy by `1 + (ω·dt)²` every step, so it grows without bound and
    /// a spring explodes. Swapping the order of the two lines makes the step *symplectic*:
    /// it preserves area in the (position, velocity) plane exactly, and as a result the
    /// energy never drifts; it only wobbles by a bounded amount around the true value. That
    /// is why this engine does not offer plain explicit Euler at all.
    ///
    /// Accuracy: first order, global error ∝ `dt`.
    ///
    /// Velocity-dependent forces (drag, damping) are evaluated at the *old* velocity
    /// `v₀`, as in explicit Euler. That is stable for `c·dt/m < 2`; beyond that, damping
    /// itself overshoots. Position-only forces need no such care.
    ///
    /// Edge cases: `timeStep ≤ 0` returns the state unchanged (run time forward only); a NaN
    /// step or a NaN acceleration gives NaN.
    ///
    /// Use when: stepping a game-style simulation cheaply and stably; springs, cloth, particles
    /// Tags: symplectic Euler, semi-implicit Euler, Euler-Cromer, explicit Euler, time
    ///   integration, physics step, Rigidbody step
    /// Node: none (simulation)
    ParticleState steppedBySemiImplicitEuler(
        float timeStep, const std::function<Vector3(Vector3 position, Vector3 velocity)>& acceleration) const;

    /// The same step as the overload above for a force that depends only on position, such
    /// as gravity or a spring to a fixed point.
    ///
    /// Use when: stepping a simulation whose forces do not depend on velocity
    /// Tags: symplectic Euler, semi-implicit Euler, position-only force, gravity step,
    ///   Euler-Cromer
    /// Node: none (simulation)
    ParticleState steppedBySemiImplicitEuler(float timeStep, const std::function<Vector3(Vector3 position)>& acceleration) const;

    // MARK: Velocity Verlet

    /// The state one `timeStep` later, by **velocity Verlet** (the leapfrog method written
    /// with positions and velocities at the same instant).
    ///
    /// Source: Hairer, Lubich & Wanner, *Geometric Numerical Integration* (2003), §I.1.4
    /// and §VI.1 (Störmer–Verlet); Swope, Andersen, Berens & Wilson (1982) for the
    /// velocity form.
    ///
    /// Formula, with `a₀ = a(x₀)` and `a₁ = a(x₁)`:
    ///
    ///     x₁ = x₀ + v₀·dt + ½·a₀·dt²        (the Taylor series of position)
    ///     v₁ = v₀ + ½·(a₀ + a₁)·dt          (the average acceleration over the step)
    ///
    /// It is symplectic and time-reversible, so the energy of an undamped system stays
    /// within a bounded wobble forever, and it is second order: halving `dt` divides the
    /// error by 4, where semi-implicit Euler divides it by 2. It costs two acceleration
    /// evaluations per call (the `a₁` of one step is the `a₀` of the next, but this stateless
    /// function recomputes it).
    ///
    /// Velocity-dependent forces: `a₁` needs the velocity `v₁`, which needs `a₁`. This
    /// function breaks the circle by predicting `v̂ = v₀ + a₀·dt` and evaluating
    /// `a₁ = a(x₁, v̂)`. The prediction is off by O(dt²), which moves `a₁` by O(dt²) and the
    /// step by O(dt³), so the scheme stays **second order** (it becomes Heun's predictor–
    /// corrector method in the velocity). What it loses is the structure: with
    /// velocity-dependent forces it is no longer symplectic or time-reversible.
    ///
    /// Edge cases: `timeStep ≤ 0` returns the state unchanged; a NaN step or a NaN
    /// acceleration gives NaN.
    ///
    /// Use when: stepping a simulation where energy drift must stay small, such as orbits,
    ///   pendulums and stiff springs
    /// Tags: velocity Verlet, Verlet integration, leapfrog, Störmer-Verlet, symplectic, time
    ///   integration, physics step
    /// Node: none (simulation)
    ParticleState steppedByVelocityVerlet(
        float timeStep, const std::function<Vector3(Vector3 position, Vector3 velocity)>& acceleration) const;

    /// The same step as the overload above for a force that depends only on position. This
    /// is the case where the method is truly second order.
    ///
    /// Use when: stepping gravity, springs to fixed points or any conservative force
    /// Tags: velocity Verlet, leapfrog, Verlet integration, position-only force, conservative
    ///   force, second order
    /// Node: none (simulation)
    ParticleState steppedByVelocityVerlet(float timeStep, const std::function<Vector3(Vector3 position)>& acceleration) const;
};

} // namespace editor
