// How much a solid weighs and how hard it is to spin: mass and the inertia tensor of the
// basic shapes, which a rigid-body simulation needs before it can move anything
// (Engine/Math/Physics/MassProperties.swift). `mass` also serves the `measureMesh` node.
#pragma once

#include <cassert>

#include "../Linear/Matrix3.h"
#include "../Linear/Vector3.h"

namespace editor {

/// Mass and **inertia tensors** of uniform (constant-density) solids.
///
/// Sources: Goldstein, *Classical Mechanics* (3rd ed., 2002) §5.3 for the inertia tensor and
/// the parallel-axis theorem (Steiner, 1840s); the shape formulas are the standard
/// engineering tables, each re-derived in its doc comment.
///
/// Conventions: kilograms, metres, so inertia is in kg·m². Z-up, right-handed. Every tensor is
/// taken about the solid's **centre of mass** in the solid's own frame (origin at the centre,
/// axes along the symmetry axes), where it comes out diagonal; the diagonal entries
/// `Iₓₓ, I_yy, I_zz` are the **principal moments**. A tensor `I` turns an angular velocity `ω`
/// into an angular momentum `L = I·ω`. Masses, densities and sizes are zero or positive;
/// the functions reject a negative or NaN one with an assert (Swift's preconditions).
///
/// Use when: preparing a solid for a rigid-body simulation; weighing a model from its
///   density; spinning a thrown ball, crate or wheel realistically
/// Tags: mass properties, moment of inertia, inertia tensor, rigid body, density, volume,
///   center of mass, Rigidbody.inertiaTensor, btCollisionShape.calculateLocalInertia
/// Node: measureMesh
struct MassProperties {
    // MARK: Mass and volume

    /// The mass of a uniform solid: `density × volume`, in kilograms for a density in kg/m³
    /// and a volume in m³ (water is 1000 kg/m³).
    ///
    /// Conventions: density and volume must be zero or positive (Swift's preconditions are
    /// asserts here).
    ///
    /// Use when: getting the mass of a model from its material
    /// Tags: mass from density, density times volume, weight of solid, material density
    /// Node: measureMesh
    static float mass(float density, float volume) {
        assert(density >= 0 && "The density must be zero or positive.");
        assert(volume >= 0 && "The volume must be zero or positive.");
        return density * volume;
    }

    /// The volume of a ball, `4/3 · π · r³`.
    ///
    /// Derivation: stack discs of radius `√(r² − z²)` for `z` from `−r` to `r` and
    /// integrate: `∫ π (r² − z²) dz = π (2r³ − 2r³/3) = 4π r³ / 3`.
    ///
    /// Use when: finding the mass of a ball; sizing a sphere collider
    /// Tags: sphere volume, ball volume, four thirds pi r cubed, volume of sphere
    /// Node: none (closed-form physics for rigid bodies, not a modelling step)
    static float sphereVolume(float radius);

    /// The volume of a box, `x × y × z` of its full edge lengths.
    ///
    /// Use when: finding the mass of a crate, a wall or a bounding box
    /// Tags: box volume, cuboid volume, cube volume, rectangular prism, volume of box
    /// Node: none (closed-form physics for rigid bodies, not a modelling step)
    static float boxVolume(Vector3 size);

    /// The volume of a solid cylinder, `π · r² · h`: the area of the circular end times
    /// the height.
    ///
    /// Use when: finding the mass of a barrel, a wheel, a pillar or a tree trunk
    /// Tags: cylinder volume, pi r squared h, volume of cylinder, wheel volume
    /// Node: none (closed-form physics for rigid bodies, not a modelling step)
    static float cylinderVolume(float radius, float height);

    // MARK: Inertia tensors about the centre of mass

    /// The inertia tensor of a solid sphere of `mass` and `radius`: `2/5 · m · r²` about any
    /// axis through the centre, so the tensor is that number times the identity.
    ///
    /// Derivation: `I = ∫ ρ (x² + y²) dV`. By symmetry `∫ρx² = ∫ρy² = ∫ρz² = (1/3)∫ρ r² dV`,
    /// so `I = (2/3)∫ρ r² dV`. With shells of area `4π r²`:
    /// `∫ρ r² dV = ρ ∫₀ᴿ 4π r⁴ dr = 4πρR⁵/5 = (3m/R³)·R⁵/5 = 3mR²/5`, giving `I = 2mR²/5`.
    ///
    /// Use when: spinning a ball, a planet or a marble in a rigid-body simulation
    /// Tags: sphere inertia, moment of inertia of a sphere, 2/5 m r squared, solid ball,
    ///   inertia tensor
    /// Node: none (closed-form physics for rigid bodies, not a modelling step)
    static Matrix3 solidSphereInertia(float mass, float radius);

    /// The inertia tensor of a solid box of `mass` whose full edge lengths along x, y and z
    /// are `size`, about its centre.
    ///
    /// Formula: about the x axis `Iₓₓ = m/12 · (size.y² + size.z²)`, and cyclically
    /// `I_yy = m/12 · (size.z² + size.x²)`, `I_zz = m/12 · (size.x² + size.y²)`.
    ///
    /// Derivation: for a uniform rod of length `s` about its middle, `∫ρ u² du` over
    /// `−s/2…s/2` gives `m s²/12`. A box is three such rods in x, y, z, and the moment about
    /// the x axis adds the y and z contributions (`I = ∫ρ (y² + z²) dV`).
    ///
    ///           ┌────────┐
    ///          ╱        ╱│ z      the axis along the longest edge
    ///         ┌────────┐ │        has the SMALLEST moment
    ///         │        │ ╱ y
    ///         └────────┘
    ///              x
    ///
    /// Use when: spinning a crate, a book, a car body or a wall in a rigid-body simulation
    /// Tags: box inertia, cuboid moment of inertia, m/12 (b squared + c squared), rectangular
    ///   prism, inertia tensor
    /// Node: none (closed-form physics for rigid bodies, not a modelling step)
    static Matrix3 solidBoxInertia(float mass, Vector3 size);

    /// The inertia tensor of a solid cylinder of `mass`, `radius` and `height` whose axis is
    /// the Z axis, about its centre.
    ///
    /// Formula: about the axis `I_zz = ½ · m · r²`; about a line across the middle
    /// `Iₓₓ = I_yy = m/12 · (3r² + h²)`.
    ///
    /// Derivation: a thin disc of mass `m` spins about its axis with `½ m r²` (integrate
    /// `ρ r² · 2π r dr` from 0 to `R`). Across a diameter it has half of that, `¼ m r²`;
    /// stacking the discs along the height adds the rod term `m h²/12` (parallel axis, see
    /// `parallelAxis`), giving `m r²/4 + m h²/12 = m/12 (3r² + h²)`.
    ///
    /// Use when: spinning a wheel, a barrel, a pillar or a log in a rigid-body simulation
    /// Tags: cylinder inertia, moment of inertia of a cylinder, 1/2 m r squared, m/12 (3r2 +
    ///   h2), solid cylinder, inertia tensor
    /// Node: none (closed-form physics for rigid bodies, not a modelling step)
    static Matrix3 solidCylinderInertia(float mass, float radius, float height);

    // MARK: Moving the reference point

    /// The inertia tensor about a point displaced by `offset` from the centre of mass, the
    /// **parallel-axis theorem** (generalized to a full tensor).
    ///
    /// Formula: `I' = I + m · ( |r|²·1 − r·rᵀ )`, with `r = offset`, `1` the identity and
    /// `r·rᵀ` the 3×3 matrix with entries `rᵢ·rⱼ`. In components:
    ///
    ///     I'ₓₓ = Iₓₓ + m (r_y² + r_z²)        I'ₓᵧ = Iₓᵧ − m rₓ r_y     (and so on)
    ///
    /// Derivation: a mass element at `ρ` from the centre sits at `ρ + r` from the new point,
    /// and `I' = ∫ dm ( |ρ + r|²·1 − (ρ + r)(ρ + r)ᵀ )`. The cross terms contain `∫ρ dm`,
    /// which is 0 about the centre of mass, leaving `I` plus the point-mass term for `m` at
    /// `r`.
    ///
    /// The tensor only grows: `I'` is never smaller than `I`, and the smallest moments are
    /// always about the centre of mass. An offset of zero returns `inertia` unchanged. The
    /// tensor stays in the same axes (only the reference point moves).
    ///
    /// Use when: spinning a body about a hinge or a pivot; building a compound body from
    ///   parts that are not at the centre
    /// Tags: parallel axis theorem, Steiner's theorem, Huygens-Steiner, inertia about a point,
    ///   offset inertia, compound body
    /// Node: none (closed-form physics for rigid bodies, not a modelling step)
    static Matrix3 parallelAxis(const Matrix3& inertia, float mass, Vector3 offset);
};

} // namespace editor
