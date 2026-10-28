// A flat outline pushed out into a slab: the field behind plates, blades, shoulder blades, a
// hip bone before it is bent, a cookie-cutter shape. Round its edges with `OffsetField`, bend
// it with `SphereWrappedField` (Engine/Math/Fields/ExtrudedOutline.swift).
#pragma once

#include <optional>

#include "../Curves/Path2D.h"
#include "../Curves/Polygon2D.h"
#include "../Geometry/Bounds.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A closed 2D outline in the xy plane, extruded `thickness` along z and centred on z = 0: a
/// flat plate with the outline's shape and straight walls.
///
///        z
///        ▲   ┌──────────────┐  ▲
///        │   │    outline    │  thickness
///     ───┼───┼───────────────┼──▶ x
///        │   │   (extruded)  │  ▼
///            └──────────────┘
///
/// How: with `outline` the signed distance in the plane (`Polygon2D::signedDistance`) and
/// `height = |z| − thickness / 2` the signed distance to the slab between the two faces,
///
///     w = (outline(x, y), height)
///     value = min(max(w.x, w.y), 0) + |(max(w.x, 0), max(w.y, 0))|
///
/// Outside both, the nearest point is on the rim and the distance is the length of `w`; outside
/// one only, it is that one; inside both, it is the nearer of the two (the larger, less negative,
/// value). This is the textbook extrusion (Quílez, *opExtrusion*).
///
/// Exact for the flattened polygon when it does not cross itself; a bound (and 1-Lipschitz)
/// when it does, as `Polygon2D::signedDistance`. The polygon follows the path to within
/// `tolerance`, so the plate's walls are within `tolerance` of the path.
///
/// Rounded edges: `OffsetField(ExtrudedOutline(outline shrunk by r, thickness − 2r), by: r)`,
/// or fillet the outline's corners first (`Path2D::filleted`) for rounded corners seen
/// from above.
///
/// Conventions: metres, Z-up. The outline's units are the plate's. The path must be closed.
/// Swift traps on a broken precondition; the core throws a `GeometryProblem` with the same
/// sentence.
///
/// Use when: a plate bone (shoulder blade, hip bone, skull plate) before it is bent; a blade, a
///   fin, a sign, a cookie-cutter shape; any part that is a flat outline with a thickness
/// Tags: extrusion, extrude 2D shape, opExtrusion, slab, plate, prism, linear extrude,
///   Solidify, outline to solid
/// Node: extrudedOutlineField
struct ExtrudedOutline {
    /// The flattened outline in the xy plane.
    Polygon2D outline;
    /// The plate's thickness along z, in the outline's units. Positive.
    float thickness;

    /// The default distance the flattened outline may stray from the path's curves: 1 mm.
    static constexpr float defaultTolerance = 0.001f;

    /// `path` flattened to within `tolerance` and extruded `thickness` along z. The path must be
    /// closed; `thickness` and `tolerance` must be positive and finite.
    ExtrudedOutline(const Path2D& path, float thickness, float tolerance = defaultTolerance);

    /// `outline` extruded `thickness` along z. `thickness` must be positive and finite.
    ExtrudedOutline(const Polygon2D& outline, float thickness);

    /// The extrusion of the outline's 2D distance (see the type's comment).
    ///
    /// Use when: sampling the plate
    /// Tags: extrusion sdf value, slab distance, plate distance
    /// Node: inside extrudedOutlineField
    float signedDistance(Vector3 point) const;

    /// The outline's box, `thickness` deep, centred on z = 0.
    std::optional<Bounds> bounds() const { return boundingBox; }

private:
    Bounds boundingBox;
};

} // namespace editor
