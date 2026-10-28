// A place on a 2D path or a list of curves, found by a search: which piece, where on it, the
// point and how far it is from what was searched for. Ported from graphics_cpp's `Nearest`,
// `Pick` and `PathProjection`, which all carried these four values.
#pragma once

#include "../Linear/Vector2.h"

namespace editor {

/// A place on a path: the piece it lies on, the parameter on that piece (0…1), the point, and
/// its distance from the point the search started from (0 when the search had no such point).
///
/// Use when: picking the curve under the cursor; snapping a point onto an outline
/// Tags: nearest point on path, closest point, projection, hit test result, pick
/// Node: planned (nearestPointOnPath)
struct PathPoint2 {
    /// Which piece, counting from 0 in drawing order.
    int pieceIndex = 0;
    /// Where on that piece, 0 at its start and 1 at its end.
    float parameter = 0;
    /// The point itself.
    Vector2 point;
    /// How far `point` is from the point searched from.
    float distance = 0;

    bool operator==(const PathPoint2&) const = default;
};

} // namespace editor
