// An endless flat sheet (Engine/Math/Primitives/Plane.swift).
#pragma once

#include "../Linear/Vector3.h"

namespace editor {

/// The flat surface of all points p with `normal·p = distance`.
///
/// `normal` has length 1 and points to the plane's "front" side. `distance` is how far the plane
/// sits from the origin, measured along `normal`: the plane z = 2 has normal (0, 0, 1) and
/// distance 2.
///
/// Use when: describing a floor, a wall, a mirror or a cut
/// Tags: plane, flat surface, normal and distance, ground plane, mirror plane, clipping plane
/// Node: mirrorField
struct Plane {
    Vector3 normal;
    float distance = 0;

    /// The plane through `point` that is at right angles to `normal` (Swift
    /// `Plane(point:normal:)`). `normal` is normalized here, so any length works, however small;
    /// only an exactly zero normal throws a `GeometryProblem`. (A tolerance would be wrong here:
    /// the cross product of two 1 mm edges is 1e-6 long and is a perfectly good direction.)
    ///
    /// How: distance = normal·point. Every point of the plane has the same dot product with the
    /// normal, and `point` is one of them.
    Plane(Vector3 point, Vector3 normal);

    /// The plane through three corners listed **counterclockwise** as seen from the front (Swift
    /// `Plane(first:second:third:)`).
    ///
    /// How: normal = (second − first) × (third − first), normalized (Ericson §3.6). By the
    /// right-hand rule, counterclockwise corners give a normal pointing at the viewer. Corners in
    /// a straight line (or equal corners) do not define a plane and throw. The test is
    /// `Triangle::isDegenerate`, which is relative to the corners' spacing, so a valid triangle
    /// with 1 mm sides is accepted.
    Plane(Vector3 first, Vector3 second, Vector3 third);

    bool operator==(const Plane&) const = default;

    /// How far `point` is from the plane, with a sign: positive on the side the normal points
    /// to, negative behind it, zero on the plane.
    ///
    /// How: normal·point − distance (Ericson §3.6). With a unit normal the result is the true
    /// perpendicular distance in world units.
    ///
    /// Use when: telling which side of a floor or mirror a point is on; measuring height above a plane
    /// Tags: signed distance to plane, point plane distance, side test, GetDistanceToPoint, height above plane
    /// Node: projectOntoPlane
    float signedDistance(Vector3 point) const;

    /// The point on the plane nearest to `point`: `point` slid along the normal until it lands on
    /// the plane.
    ///
    /// How: point − normal × signedDistance(point) (Ericson §3.6).
    ///
    /// Use when: dropping a point onto a floor or a wall; snapping an object to a plane
    /// Tags: project onto plane, closest point on plane, flatten, snap to plane, ClosestPointOnPlane
    /// Node: projectOntoPlane
    Vector3 projection(Vector3 point) const;
};

} // namespace editor
