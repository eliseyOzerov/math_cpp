// A direction tilted away from a parent (Engine/Math/Algebra/Vector3+Tilt.swift). Swift's
// `Vector3.direction(tiltedAwayFrom:by:toward:)` is a free function here, because Vector3.h
// cannot include Quaternion.h.
#pragma once

#include "Angle.h"
#include "Quaternion.h"

namespace editor {

/// The unit direction that leans `tilt` away from `parent`, on the side given by `turn`: the
/// direction a branch grows from its parent limb, or a spike leaves a surface.
///
/// How: rotate the horizontal direction at `turn`, (cos turn, sin turn, 0), by the rotation that
/// takes +Z to `parent` (`Quaternion::rotation`). That gives a unit vector `side` perpendicular to
/// `parent`. Then
///
///     direction = parent·cos(tilt) + side·sin(tilt)
///
/// is unit length (the two parts are perpendicular unit vectors) and makes the angle `tilt` with
/// `parent`:
///
///          parent
///            ↑   ↗ direction
///            |  /
///            | / tilt
///            |/____→ side (at angle `turn` around parent)
///
/// Source: spherical coordinates about an arbitrary axis; the frame around the axis comes from
/// the shortest-arc rotation (Melax, "The Shortest Arc Quaternion", 2000).
///
/// Conventions: Z-up, right-handed. `parent` need not be unit length; it is reduced to a
/// direction first (`unitDirection`), so any finite non-zero length works. `turn` is measured
/// counterclockwise around `parent`, starting from where +X goes under the rotation from +Z; for
/// `parent` = +Z it is the usual angle from +X in the XY plane. For `parent` = −Z exactly, the
/// rotation from +Z is a half turn about +Y, so `turn` 0 points along −X and 90° along +Y. Near
/// −Z the starting direction depends on which side `parent` approaches from and can jump by up to
/// 180°; the turn still runs counterclockwise around `parent`. Recipes that need a steady turn near
/// straight down should use a reference direction instead. Any tilt works: a negative tilt equals
/// |tilt| toward `turn` + 180°, and `tilt` and 360° − `tilt` give the same direction, so the angle
/// with `parent` is the tilt folded into 0…180°. A tilt of 0 returns the direction of `parent`, 90°
/// returns `side`. A zero-length or non-finite `parent` has no direction, so the result has NaN
/// components.
///
/// Use when: pointing the branches of a tree around their parent limb; scattering spikes or petals in a cone around a normal
/// Tags: cone direction, tilt away, spherical coordinates around axis, branch angle, azimuth and inclination, direction in cone, phyllotaxis direction
/// Node: coneDirection
Vector3 directionTiltedAwayFrom(Vector3 parent, Angle tilt, Angle turn);

} // namespace editor
