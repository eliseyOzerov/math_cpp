// A filled region of the plane: closed outlines, where some may be holes in others. Booleans,
// corners, warps and ink outlines read and build these. Replaces graphics_cpp's
// `std::vector<Contour>` (Curves2D).
#pragma once

#include <vector>

#include "Path2D.h"

namespace editor {

/// Which points a set of overlapping outlines fills.
///
///     even-odd: inside when a ray from the point crosses the outlines an odd number of times
///     nonzero:  inside when the outlines wind round the point a nonzero number of times
///
/// Use when: deciding whether a hole drawn the same way round as its outline is a hole
/// Tags: fill rule, even-odd, evenodd, nonzero, winding rule, SVG fill-rule
/// Node: planned (fillShape)
enum class FillRule { evenOdd, nonzero };

/// A region of the plane bounded by closed paths: outlines and the holes in them, filled by
/// `fillRule`. An open path in `outlines` is read as closed by a straight line, as a fill reads it.
///
///     ┌──────────┐      one outline counterclockwise, one hole clockwise:
///     │  ┌────┐  │      both fill rules give a ring
///     │  └────┘  │
///     └──────────┘
///
/// Conventions: y-up plane; outlines counterclockwise and holes clockwise is the tidy form the
/// booleans return, but any directions are accepted.
///
/// Use when: holding the result of a union or difference; outlining a letter with holes
/// Tags: shape, region, compound path, contours, polygon with holes, SVG path with subpaths
/// Node: planned (booleanShapes)
struct Shape2D {
    /// The closed paths that bound the region.
    std::vector<Path2D> outlines;
    /// Which points the outlines fill.
    FillRule fillRule = FillRule::nonzero;

    bool operator==(const Shape2D&) const = default;
};

} // namespace editor
