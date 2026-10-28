// A block of glass with flat faces (a slab, a prism), and the path a ray takes through it
// (Engine/Math/Optics/RefractingSolid.swift).
#pragma once

#include <optional>
#include <vector>

#include "../Geometry/Plane.h"
#include "../Geometry/Ray.h"
#include "../Linear/Vector3.h"
#include "RefractingInterface.h"

namespace editor {

/// One straight piece of a ray's journey, in a single medium.
///
/// Use when: storing the stretches of a light ray inside and outside a glass solid
/// Tags: ray leg, ray segment, ray path piece, light path, segment of ray
/// Node: none (value type)
struct RayLeg {
    /// The ray at the start of the leg, with the power it carries there.
    WeightedRay start;
    /// How far the leg runs before it meets a surface, or no value if it goes on forever
    /// (the last leg, after the ray has left the glass).
    std::optional<float> length;

    bool operator==(const RayLeg&) const = default;

    /// The point where the leg ends, or no value for a leg that goes on forever.
    ///
    /// Use when: drawing a light path or finding where a ray leaves a solid
    /// Tags: leg end, ray end point, segment end, exit point
    /// Node: none (renderer shading)
    std::optional<Vector3> endPoint() const;
};

/// Where a ray meets a face of a solid: how far along the ray, and the face's outward
/// normal (Swift's tuple `(distance:, faceNormal:)`).
///
/// Use when: reading the result of `RefractingSolid::firstBoundaryHit`
/// Tags: boundary hit, face hit, hit distance, face normal
/// Node: none (value type)
struct BoundaryHit {
    float distance;
    Vector3 faceNormal;

    bool operator==(const BoundaryHit&) const = default;
};

/// A convex solid bounded by flat faces, filled with a transparent material.
///
/// The solid is the set of points behind **every** face plane: each `Plane` normal points
/// out of the glass, so `signedDistance ≤ 0` means inside. Two parallel planes make an
/// endless slab; three planes in a triangle make an endless prism (a triangle seen from the
/// side, running along the axis that no normal has a part along); more planes make a
/// polyhedron.
///
/// Use when: tracing light through a glass prism, a gemstone or a crystal
/// Tags: glass solid, prism, gemstone, polyhedron, crystal, refracting solid, convex solid
/// Node: none (renderer shading)
struct RefractingSolid {
    /// The faces, with unit normals pointing out of the glass.
    std::vector<Plane> faces;
    /// The refractive index of the material inside.
    float refractiveIndexInside;
    /// The refractive index of the surroundings (1 for air).
    float refractiveIndexOutside;

    /// The first face a ray meets and how far along the ray that is, or no value if it never
    /// reaches the solid's boundary in that direction.
    ///
    /// Source: Cyrus & Beck, "Generalized Two- and Three-Dimensional Clipping" (1978); the
    /// same idea as the slab test for boxes. Each face plane cuts the ray's line at
    /// `t = (distance − normal·origin) / (normal·direction)`. A face with
    /// `normal·direction < 0` is one the ray *enters* through, one with `> 0` one it *leaves*
    /// through, and the ray is inside the convex solid between the last entering `t` and the
    /// first leaving `t`:
    ///
    ///           enter plane          leave plane
    ///     ray ─────┃──────────────────────┃────▶   inside for  tEnter ≤ t ≤ tLeave
    ///
    /// For a ray starting outside, the boundary it meets is where it enters: the largest
    /// entering `t`, valid if it is not after the smallest leaving `t`. For a ray starting
    /// inside (`startsInside`), it is the smallest leaving `t` greater than 0. A ray parallel to a face
    /// (`|normal·direction| ≤ Tolerance.geometric`) never crosses that plane: if it starts in
    /// front of it, it misses the solid. Hits at `t ≤ 0` are ignored, so a ray that starts
    /// exactly on a face, having just crossed it, does not hit it again.
    ///
    /// Use when: finding where a ray hits the faces of a glass prism or crystal
    /// Tags: ray hit, ray face intersection, convex polyhedron, slab test, boundary hit,
    ///   entry exit point
    /// Node: none (optics simulation)
    std::optional<BoundaryHit> firstBoundaryHit(Ray ray, bool startsInside) const;

    /// The legs of the path of `ray` (starting in the surroundings) through the solid, each
    /// with the power it carries. It follows the refracted ray at every face, or the
    /// reflected one on total internal reflection, until the ray leaves for good.
    ///
    /// Each leg ends at a face where `RefractingInterface.continuation` decides what comes
    /// next. The last leg, after the ray has left, has no length. A ray that misses the solid
    /// gives one infinite leg. Because the solid is convex, a ray that has left cannot come
    /// back, so the path ends there.
    ///
    /// `maximumLegs` stops a ray trapped inside by repeated total internal reflection (a
    /// glass rod can trap light for a long way); the last leg returned is then cut short and
    /// has a length. A ray's origin must not lie inside the solid (it starts outside).
    ///
    /// Use when: drawing the route of a light ray through a prism or gemstone, including
    ///   reflections inside
    /// Tags: ray path, light path, ray tracing, internal reflection, prism path,
    ///   refraction through solid
    /// Node: none (renderer shading)
    std::vector<RayLeg> path(Ray ray, int maximumLegs = 16) const;
};

} // namespace editor
