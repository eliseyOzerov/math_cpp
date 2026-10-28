// An ellipsoid is a sphere stretched by a different amount along three perpendicular axes: the
// shape of a muscle belly, an eyeball socket, a fat pad. Its exact distance has no closed
// form (the nearest point solves a degree-6 equation), so the field is a proven bound
// (Engine/Math/Fields/Ellipsoid.swift).
#pragma once

#include <optional>

#include "../Linear/Quaternion.h"
#include "../Geometry/Bounds.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A solid ellipsoid: the points `centre + rotation(q)` with
/// `(qₓ/rₓ)² + (q_y/r_y)² + (q_z/r_z)² ≤ 1`, where `r = radii` are the half lengths along its
/// own x, y and z axes before the rotation.
///
/// Conventions: Z-up, right-handed; `radii` are positive lengths; `rotation` is a unit
/// quaternion that turns the ellipsoid's own axes into world directions.
///
/// Use when: modelling a muscle belly, a fat pad, a head or an eye; a squashed or stretched
///   ball
/// Tags: ellipsoid, sdEllipsoid, stretched sphere, spheroid, ovoid, egg, muscle shape
/// Node: ellipsoidField
struct Ellipsoid {
    Vector3 centre;
    /// The half lengths along the ellipsoid's own x, y and z axes.
    Vector3 radii;
    /// Turns the ellipsoid's own axes into world directions.
    Quaternion rotation;

    /// The ellipsoid around `centre` with half lengths `radii` along its own axes, turned by
    /// `rotation` (a unit quaternion). Every radius must be positive and finite.
    Ellipsoid(Vector3 centre, Vector3 radii, Quaternion rotation = Quaternion::identity());

    bool operator==(const Ellipsoid&) const = default;

    /// A conservative signed distance from `point` to the ellipsoid's surface: never more
    /// than the true distance, with the right sign. Exact on the three axes outside the
    /// ellipsoid, and inside along its shortest axis; a bound elsewhere.
    ///
    /// How: in the ellipsoid's own frame, `q = rotation⁻¹(point − centre)` (the centre is
    /// subtracted first, so the value keeps its precision far from the origin). Three
    /// conservative estimates are computed and the largest is returned (the largest of
    /// conservative values is conservative, see `raising`):
    ///
    /// 1. The **stretched-sphere bound** `(|q / r| − 1) × min(r)` (`q / r` componentwise).
    ///    Proof: `g(q) = min(r) × |q / r|` is the length of `q` after scaling each axis by
    ///    `min(r)/rᵢ ≤ 1`, a map that never lengthens a vector, so `g` is 1-Lipschitz. `g − min(r)`
    ///    is zero exactly on the ellipsoid, negative inside, and 1-Lipschitz, so it never
    ///    overstates the distance to the surface (Hart, "Sphere tracing", 1996). It is the
    ///    "simple" bound of Quílez, "ellipsoid SDF" (iquilezles.org/articles/ellipsoids);
    ///    his "improved" `k₀(k₀ − 1)/k₁` is not conservative everywhere, so it is not used.
    /// 2. The distance to the box `−r … r` in the own frame, which holds the ellipsoid.
    ///    Outside, on an axis, it is the exact distance (`|qₓ| − rₓ` on the x axis), which the
    ///    stretched-sphere bound underestimates on every axis but the shortest.
    /// 3. The distance to `bounds` (the world box), so the field keeps the `DistanceField`
    ///    contract outside its box.
    ///
    ///          ┌──── own box ────┐
    ///          │   ╭─────────╮   │      outside on an axis: the box face is the nearest
    ///       q ●┼───┤ ellipse ├───┼      point, so the box distance is exact there
    ///          │   ╰─────────╯   │
    ///          └─────────────────┘
    ///
    /// Inside on the shortest axis the bound is exact: there the nearest surface point is
    /// the end of that axis (the centre of curvature of that end lies beyond the centre).
    ///
    /// Edge cases: equal radii give the exact sphere distance. A NaN point gives NaN.
    ///
    /// Use when: sampling a muscle or fat pad of a character; any squashed ball
    /// Tags: sdEllipsoid, ellipsoid sdf, ellipsoid bound, distance to ellipsoid, stretched sphere distance
    /// Node: ellipsoidField
    float signedDistance(Vector3 point) const;

    /// The smallest axis-aligned box around the turned ellipsoid.
    ///
    /// How: the ellipsoid is `centre + M·u` for `|u| ≤ 1`, with `M = rotation × diag(r)`, whose
    /// columns are the three turned semi-axes `aᵢ = rᵢ × rotation(axisᵢ)`. Its reach along a
    /// world axis `ê` is the largest `ê·M·u`, which is `|Mᵀê|` (Cauchy–Schwarz, at `u` along
    /// `Mᵀê`). On each world axis:
    ///
    ///     half size = √(a₁² + a₂² + a₃²)       (squares taken componentwise)
    ///
    /// This is tighter than the box around the turned own box (Arvo), which sums the
    /// absolute values instead.
    ///
    /// Use when: culling an ellipsoid; sizing the region to mesh it in
    /// Tags: ellipsoid bounds, ellipsoid AABB, rotated ellipsoid box, support function
    /// Node: ellipsoidField
    std::optional<Bounds> bounds() const { return worldBox; }

private:
    /// The tight world box (see `bounds`), worked out once.
    Bounds worldBox;

    /// The box `−radii … radii` in the ellipsoid's own frame: the ellipsoid touches it at the
    /// six ends of its axes.
    Bounds localBox() const;

    /// The smallest axis-aligned box around the ellipsoid with these parts, as derived in
    /// `bounds`.
    static Bounds tightBox(Vector3 centre, Vector3 radii, const Quaternion& rotation);
};

} // namespace editor
