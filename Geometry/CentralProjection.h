// Central (gnomonic) projection between a plane and the sphere about the origin
// (Engine/Math/Primitives/CentralProjection.swift). Swift adds it to `Plane` as a method; here it
// is a free function beside `Plane`, which this file leaves alone.
#pragma once

#include <cmath>
#include <optional>

#include "../Linear/Vector3.h"
#include "Plane.h"

namespace editor {

/// Where the line from the origin along `direction` meets `plane`: the **central projection** of
/// a direction onto the plane, or nothing when the line runs parallel to the plane or meets it
/// behind the origin.
///
/// Source: Snyder, *Map Projections: A Working Manual* (1987), the gnomonic projection.
///
/// How: the points of the line are `t·direction`. The plane holds the points with
/// `normal·p = distance`, so `t·(normal·direction) = distance` and
///
///     point = direction · distance / (normal·direction)
///
/// Going back is `point / |point|`. The two undo each other, and both keep straight lines
/// straight in this sense: a segment on a plane that misses the origin, together with the
/// origin, spans a plane through the centre; that plane cuts the unit sphere in a great
/// circle, so the segment projects onto an arc of it shorter than 180°.
///
///        origin ●───────────▶ direction
///                ╲    │
///                 ╲   │ plane (distance along normal)
///                  ╲  ● point
///
/// The projection is not even: a small area of the plane shrinks by cos³θ on the unit
/// sphere, θ the angle between the direction and the normal, so a grid on the plane gives
/// smaller cells away from the plane's foot.
///
/// Conventions: `direction` need not have length 1. The plane must not pass through the
/// origin (`distance` not 0). Nothing when `distance / (normal·direction)` is not positive and
/// finite: the line is parallel to the plane, or meets it on the far side of the origin.
///
/// Use when: carrying a point drawn on one panel's plane onto another's along the line
///   through the ball's centre; moving a motif from its tangent plane onto a face's plane
/// Tags: central projection, gnomonic projection, project direction onto plane, ray from
///   origin to plane, radial projection, great circle as straight line
/// Node: inside sphericalPanels, tileSphere
inline std::optional<Vector3> centralProjection(const Plane& plane, Vector3 direction) {
    float alongNormal = plane.normal.dot(direction);
    if (alongNormal == 0) return std::nullopt;
    float scale = plane.distance / alongNormal;
    if (!(std::isfinite(scale) && scale > 0)) return std::nullopt;
    return direction * scale;
}

} // namespace editor
