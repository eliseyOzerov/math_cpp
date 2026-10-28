// Clipping a polygon by a half-plane (graphics_cpp curves/fit `clipPolygon`).
#include "Polygon2D.h"

namespace editor {

/// The polygon trimmed to the kept side of `halfPlane`, or nothing when fewer than 3 corners
/// remain (it was all on the cut side, or only touched the line).
///
/// How: `HalfPlane2::clipped` (Sutherland–Hodgman against one line).
///
/// Conventions: y-up plane; exact for a convex polygon; a concave polygon cut into several
/// pieces comes back joined along the line (see `HalfPlane2::clipped`).
///
/// Use when: cutting the end of a stroke square; slicing a convex plate with a line
/// Tags: clip polygon, half plane, cut polygon, trim shape, Sutherland-Hodgman
/// Node: planned (clipByLine)
std::optional<Polygon2D> Polygon2D::clippedBy(const HalfPlane2& halfPlane) const {
    std::vector<Vector2> kept = halfPlane.clipped(cornerList);
    if (kept.size() < 3) return std::nullopt;
    return Polygon2D(std::move(kept));
}

} // namespace editor
