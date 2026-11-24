// Polygon triangulation by ear clipping, a faithful port of mapbox/earcut 3.0.2
// (Vladimir Agafonkin, ISC; the copy bundled in three.js r187) so a host gets exactly the
// triangles the JavaScript gives.
#pragma once

#include <span>
#include <vector>

namespace editor {

/// The triangles that tile a polygon with holes, as corner indices in threes.
///
/// Port of mapbox/earcut 3.0.2 (Vladimir Agafonkin, ISC), which clips "ears" (a corner whose
/// triangle holds no other corner) off a circular linked list of the corners; holes are first
/// bridged into the outer ring (David Eberly's method), and rings of more than 80 corners use
/// a z-order curve hash to find the corners near an ear. When no ear is left it filters
/// collinear points, cures small self-intersections, and finally splits the ring in two.
///
/// This function uses `double`, unlike the rest of Curves, on purpose: JavaScript numbers are
/// doubles, and the result must match earcut's bit for bit (the z-order hash and the signs of
/// the areas depend on every rounding). Arithmetic keeps earcut's order; build without fused
/// multiply-add (the Makefile's `-ffp-contract=off`).
///
/// Conventions: `coordinates` is x0, y0, x1, y1, ...: the outer ring first, then each hole.
/// `holeStarts` gives the corner number (not the coordinate number) where each hole begins,
/// earcut's `holeIndices` with dim fixed at 2; empty means no holes. Winding of the inputs is
/// free (each ring is re-wound inside). Indices count corners from 0 and are returned in
/// earcut's order, not reoriented and not deduplicated beyond what earcut does itself. Fewer
/// than three usable corners, or a fully collinear ring, give an empty result.
///
/// Use when: filling a building footprint or a land polygon with holes with triangles;
///   cutting a flattened outline for a mesh
/// Tags: triangulation, earcut, ear clipping, polygon with holes, tessellation, fill
/// Node: none (a triangulation step, used by hosts)
std::vector<int> earcutTriangles(std::span<const double> coordinates, std::span<const int> holeStarts);

}  // namespace editor
