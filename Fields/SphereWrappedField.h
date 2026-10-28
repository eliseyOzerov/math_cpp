// Bending a flat shape onto a ball: a plate built flat (an outline, extruded, rounded) is laid
// onto a sphere like a sticker, so a shoulder blade, a skull plate or a curved shield needs
// no shape of its own (Engine/Math/Fields/SphereWrappedField.swift).
#pragma once

#include <optional>

#include "../Linear/Angle.h"
#include "../Geometry/Bounds.h"
#include "../Linear/Vector3.h"
#include "DistanceField.h"

namespace editor {

/// A flat shape bent onto a sphere of radius `radius`: the sheet z = 0 of the shape's own frame
/// is laid onto the sphere, and z becomes the height above it.
///
/// **The map.** The sphere's centre is `(0, 0, −radius)`, so the origin stays where it is and
/// the top of the sphere touches the sheet there. A flat point at distance ρ from the origin,
/// in the direction φ, goes to the point of the sphere an arc ρ away from the top, in the same
/// direction, then `z` further out along the radius (the **azimuthal equidistant** map, Snyder,
/// *Map Projections: A Working Manual* (1987) §25, read backwards):
///
///     flat (ρ cos φ, ρ sin φ, z)   →   centre + (radius + z)·(sin θ cos φ, sin θ sin φ, cos θ),
///                                      θ = ρ / radius
///
///          flat sheet  ───────●───────            ρ along the sheet
///                         ╭───●───╮               = the same arc length along the sphere
///                       ╱     │     ╲
///                      │      ● centre
///
/// The field asks the inner field at the flat point of each world point (`flatPoint`):
/// with `q = point − centre`, `r = |q|` and θ the angle between `q` and +z,
///
///     flat = (radius·θ · (q.x, q.y) / |(q.x, q.y)|,  r − radius)
///
/// **Why the value is divided by L.** The map stretches some directions: along the radius by
/// 1, along a meridian by `radius / r`, along a parallel by `radius·θ / (r·sin θ)`. A distance
/// in the world can become up to `L` times longer in the flat frame, so the inner value is
/// divided by the largest stretch `L` to stay conservative. The stretch grows without limit
/// near the centre (r → 0) and the far pole (θ → 180°), so it is bounded only on a **guard
/// region** G: `r ≥ guardRadius` and `θ ≤ guardAngle`, which holds the bent shape with room to
/// spare. There `L = max(1, (radius / guardRadius) · guardAngle / sin(guardAngle))`
/// (`largestStretch`), since θ / sin θ grows with θ.
///
/// **The value**, with `margin` the room between the shape and the edge of G:
///
///     inside G:   min(innerBox.raising(inner(flat)) / L, margin),  flat = flatPoint(point)
///     outside G:  margin
///     then raised to at least the distance to `bounds` (`raising`)
///
/// Why it is safe. (1) Inside G each straight piece of a path is stretched at most `L` times,
/// so `inner / L` is 1-Lipschitz there; raising by the inner box's distance keeps it so (the
/// larger of two 1-Lipschitz functions is 1-Lipschitz). (2) On the edge of G the raised inner
/// value is at least `margin · L`, because the inner box is that far from the edge in the flat
/// frame; the raise makes this true even for an inner field that only bounds its distance
/// (a displacement, a smooth blend). So the two cases meet at `margin` and the whole is
/// continuous; a continuous function that is 1-Lipschitz on each piece of every segment is
/// 1-Lipschitz. (3) Outside G the shape is at least `margin` away.
/// (4) The sign is right: inside G the map is one-to-one, and the shape is exactly where the
/// inner value is negative. 1-Lipschitz with the right sign is conservative (Hart, 1996).
/// The cap at `margin` loses nothing near the shape; far from it the box takes over.
///
/// The guard: `guardRadius = (1 − guardDepthShare) · r₀` and `guardAngle = θₛ + min(widening,
/// (180° − θₛ) / 2)`, with `r₀ = radius + lowest z` the inner box's lowest reach and
/// `θₛ = ρₘₐₓ / radius` the widest arc its corners reach. For a hand-sized plate on a
/// hand-sized ball `L` is about 1.6 to 1.7: blends next to a bent plate are about that much
/// wider.
///
/// The margin is the smallest of three gaps between the shape and the edge of G, each
/// measured so that it is a safe value there:
///
///     (r₀ − guardRadius) / L      the flat z-gap down to the inner sphere r = guardRadius,
///                                 shrunk by the stretch (the value is inner / L)
///     radius · widening / L       the flat ρ-gap out to the cone θ = guardAngle, shrunk
///                                 the same way
///     r₀ · sin(widening)          the world distance from outside the cone to the shape,
///                                 which sits at r ≥ r₀ and θ ≤ θₛ (widening < 90°)
///
/// Points closer to the centre than `guardRadius` are covered by the first gap, since L ≥ 1.
///
/// Conventions: metres, Z-up; `radius` positive. The inner field must be bounded, must not reach
/// below the sphere's centre (`radius + lowest z > 0`), and must reach less than halfway round
/// (`ρₘₐₓ < π · radius`). Bend toward −z only; turn the result over (`TransformedField`) to
/// bend the other way. Anything else throws a `MathProblem`.
///
/// Use when: bending a flat plate into a shell (a shoulder blade, a skull plate, a knee cap, a
///   shield, a curved panel); laying a flat pattern onto a ball
/// Tags: bend, wrap, curve onto sphere, spherical bend, Simple Deform bend, cast to sphere,
///   azimuthal equidistant, domain warp, shell
/// Node: sphereWrapField
struct SphereWrappedField {
    DistanceField inner;
    /// The inner field's box, in the flat frame.
    Bounds innerBounds;
    /// The radius of the sphere the sheet z = 0 is laid onto.
    float radius;
    /// The guard region's inner radius, from the sphere's centre.
    float guardRadius;
    /// The guard region's widest angle from the top.
    Angle guardAngle;
    /// The largest stretch of the map on the guard region (`largestStretch`).
    float stretch;
    /// The value's cap: a lower bound on the field's value at the edge of the guard region
    /// (see the three gaps in the type's comment).
    float margin;

    /// The share of the room below the shape the guard region keeps: it reaches down to
    /// `(1 − share) · r₀` from the centre.
    ///
    /// Use when: tuning how deep below a bent shape its field stays exact
    /// Tags: guard depth, guard region, bend margin
    /// Node: inside sphereWrapField
    static float guardDepthShare() { return 0.25f; }

    /// The most the guard region reaches past the shape's widest arc.
    ///
    /// Use when: tuning how far round the sphere a bent shape's field stays exact
    /// Tags: guard widening, guard angle, bend margin
    /// Node: inside sphereWrapField
    static Angle largestGuardWidening() { return Angle::fromRadians(0.5f); }

    /// `inner` bent onto a sphere of `radius` (see the type's comment for the limits).
    SphereWrappedField(DistanceField inner, float radius);

    /// The largest factor by which the flat-frame map lengthens a short step, anywhere in the
    /// guard region `r ≥ guardRadius`, `θ ≤ guardAngle`.
    ///
    /// How: a step along the radius keeps its length; along a meridian it grows by
    /// `sphereRadius / r` (the arc `r dθ` becomes `sphereRadius dθ`); along a parallel by
    /// `sphereRadius·θ / (r sin θ)` (the circle of radius `r sin θ` becomes one of radius
    /// `sphereRadius·θ`). θ / sin θ is at least 1 and grows with θ, so the largest is at the
    /// region's inner, widest corner:
    ///
    ///     L = max(1, (sphereRadius / guardRadius) · guardAngle / sin(guardAngle))
    ///
    /// Conventions: `guardRadius` positive; `guardAngle` between 0 and 180°, exclusive of 180°.
    /// A `guardAngle` of 0 uses θ / sin θ → 1.
    ///
    /// Use when: the Lipschitz factor of a spherical bend
    /// Tags: Lipschitz constant, stretch factor, metric distortion, scale factor of map
    /// Node: inside sphereWrapField
    static float largestStretch(float sphereRadius, float guardRadius, Angle guardAngle);

    /// The farthest a point of the box is from the z axis: the distance of its farthest corner,
    /// seen from above.
    ///
    /// Use when: the widest arc a flat shape reaches when it is bent round the z axis
    /// Tags: farthest corner, radial extent, distance from axis
    /// Node: inside sphereWrapField
    static float widestDistanceFromAxis(const Bounds& box);

    /// The box around the part of a spherical shell between `nearest` and `farthest` from
    /// `centre`, within `widestArc` of the top (+z).
    ///
    /// How: across, the cap reaches `farthest · sin(min(widestArc, 90°))` either way. Up, it
    /// reaches `farthest` (the top). Down, `nearest · cos(widestArc)` when the arc is at most
    /// 90° (the lowest point is the inner rim), `farthest · cos(widestArc)` past 90° (the
    /// outer rim hangs below the equator).
    ///
    /// Use when: the bounds of a shape bent onto a sphere
    /// Tags: spherical cap bounds, shell box, bounding box of cap
    /// Node: inside sphereWrapField
    static Bounds boundsOfCap(Vector3 centre, float nearest, float farthest, Angle widestArc);

    /// Whether `point` (in the world) lies in the guard region (see the type's comment).
    ///
    /// Use when: deciding whether the flat-frame value can be trusted at a point
    /// Tags: guard region, valid region, domain of map
    /// Node: inside sphereWrapField
    bool isInGuardRegion(Vector3 point) const;

    /// The flat-frame point that the bend carries to `point`: `(radius·θ · direction across,
    /// r − radius)` (see the type's comment).
    ///
    /// Conventions: a point on the z axis above the centre maps to `(0, 0, r − radius)`.
    ///
    /// Use when: asking the flat shape about a world point; unwrapping a point of a bent plate
    /// Tags: unwrap, inverse bend, world to flat, azimuthal equidistant inverse
    /// Node: inside sphereWrapField
    Vector3 flatPoint(Vector3 point) const;

    /// The world point a flat-frame point is carried to: the inverse of `flatPoint`.
    ///
    /// How: `centre + (radius + z)·(sin θ cos φ, sin θ sin φ, cos θ)`, θ = ρ / radius.
    ///
    /// Use when: placing something on the bent plate by its flat coordinates; drawing the bend
    /// Tags: bend point, wrap point, flat to world, azimuthal equidistant
    /// Node: sphereWrapField
    Vector3 wrappedPoint(Vector3 flat) const;

    /// The inner value at the flat point, raised to the inner box's distance there, divided by
    /// the stretch and capped at `margin` in the guard region; `margin` outside it; then raised
    /// to the world box's distance (see the type's comment).
    ///
    /// Use when: sampling the bent shape
    /// Tags: bent sdf value, wrapped field value, warped distance
    /// Node: inside sphereWrapField
    float signedDistance(Vector3 point) const;

    /// The world box around the bent inner box.
    std::optional<Bounds> bounds() const { return worldBox; }

private:
    Bounds worldBox;
};

} // namespace editor
