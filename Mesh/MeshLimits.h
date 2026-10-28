// The smallest and largest counts the mesh builders accept, and the settings of smoothing
// (Engine/Geometry/Mesh+Primitives.swift, Mesh+Operations.swift, Mesh+Smoothing.swift).
#pragma once

namespace editor {

/// The smallest counts the primitives accept: fewer slices and the shape collapses to a flat
/// or empty one. Both the primitives' checks and the validation of a loaded shape use these
/// numbers, so the two cannot disagree.
///
/// Use when: checking that a primitive's parameters are buildable before building it
/// Tags: primitive limits, minimum segments, minimum divisions, parameter bounds, validation
/// Node: none (limits)
struct PrimitiveLimits {
    /// A grid has at least one square per side.
    static constexpr int minimumGridDivisions = 1;
    /// A sphere has at least a triangle's worth of slices around its axis.
    static constexpr int minimumSphereSegments = 3;
    /// A sphere has at least the two polar caps (an octahedron with 4 segments).
    static constexpr int minimumSphereRings = 2;
    /// A cylinder has at least a triangle's worth of slices around its axis.
    static constexpr int minimumCylinderSegments = 3;
    /// A cone has at least a triangle's worth of slices around its axis.
    static constexpr int minimumConeSegments = 3;
    /// A torus has at least 3 slices around the Z axis and around the tube.
    static constexpr int minimumTorusSegments = 3;
};

/// The smallest and largest counts the mesh operations accept. Like `PrimitiveLimits`, one
/// place for the numbers, so the checks of the operations and any later validation of a stored
/// recipe cannot disagree.
///
/// Use when: checking that the parameters of a sweep, revolve or icosphere are buildable
/// Tags: operation limits, minimum samples, maximum subdivisions, parameter bounds, validation
/// Node: none (limits)
struct MeshOperationLimits {
    /// A sweep needs a ring at each end of the curve.
    static constexpr int minimumSweepSamples = 2;
    /// The most rings a sweep makes. With `maximumSweepProfilePoints` that is about a million
    /// points (1024 × 1024), six times the finest icosphere.
    static constexpr int maximumSweepSamples = 1024;
    /// A sweep's profile is a ring of at least a triangle's corners.
    static constexpr int minimumSweepProfilePoints = 3;
    /// The most corners a sweep's profile may have (see `maximumSweepSamples`).
    static constexpr int maximumSweepProfilePoints = 1024;
    /// A revolved profile has at least one edge to spin.
    static constexpr int minimumRevolveProfilePoints = 2;
    /// The most points a revolved profile may have.
    static constexpr int maximumRevolveProfilePoints = 1024;
    /// A full turn needs at least a triangle's worth of slices; a partial turn needs one.
    static constexpr int minimumRevolveSegmentsForFullTurn = 3;
    /// The most slices a revolve makes around the axis (see `maximumRevolveProfilePoints`).
    static constexpr int maximumRevolveSegments = 1024;
    /// An icosphere with 7 subdivisions has 163 842 points and 327 680 triangles. One more
    /// would be four times that, more than a model needs and enough to stall an editor.
    static constexpr int maximumIcosphereSubdivisions = 7;
    /// A refined sphere (`geodesicSphere`, `cubeSphere`) with 128 divisions has as many points
    /// as the finest icosphere (163 842 for the geodesic sphere, 98 306 for the cube sphere).
    static constexpr int maximumSphereDivisions = 128;
};

/// The settings of `Mesh::taubinSmoothed`.
///
/// Use when: reading the default amounts of the smoothing, or checking a recipe's iteration count
/// Tags: smoothing limits, taubin defaults, lambda mu, iteration cap
/// Node: none (limits)
struct MeshSmoothing {
    /// The most iterations `Mesh::taubinSmoothed` accepts. Each iteration costs two passes over
    /// every edge; beyond this many, the surface has long stopped changing.
    static constexpr int maximumIterations = 200;
    /// The shrinking step λ: each point moves this fraction of the way to the average of its
    /// neighbours.
    static constexpr float shrinkFactor = 0.5f;
    /// The inflating step μ, negative: each point moves this fraction of the way *away* from the
    /// average of its neighbours. With λ = 0.5 and μ = −0.53 the pass band is
    /// `1/λ + 1/μ ≈ 0.113`.
    static constexpr float inflateFactor = -0.53f;
};

} // namespace editor
