// The curve tools of bezier.hpp, intersect.hpp and offset.hpp on whole paths, with places as progress: the Path
// methods of the same names call these (path.project(p), path.mold(at, to), path.offset(d)…).
//
// Use-cases, by function:
// - project: the place on a stroke nearest a point (where a pointer grabs a path, where a dot or serif attaches).
// - extrema, inflections: where a stroke is level or upright, and where it changes its bend; cut there with
//   path.cut(path.extrema()).
// - side: the place a path reaches furthest left, right, up or down.
// - turns: the sharp nodes of a path, how much each turns and which way its point faces.
// - nearness: how near each place of a path is to some places, along it, eased by a falloff.
// - bbox: a path's exact bounds.
// - curvature, normal, osculating: how tight a stroke turns at a place, the direction across it, the circle that
//   matches it there.
// - mold: drag the curve itself, not a handle (the Primer's molding).
// - offset, outline: a path's parallel at a distance, and a monoline stroke round it.
// - lineCrossings, circleCrossings, selfCrossings: intersections in progress, ready for slice and cut.
// - arcs: the path as circular arcs.
// - selectAt: a soft selection centered on a place, the start of every brush.
//
// Ported from fountain-ts/src/curves/analysis.ts (its header has the use-cases at length). Tools that measure
// distances or angles work in the path's metric; affine ones work on the plain coordinates.
//
// Decisions (C++ mapping):
// - The result records are declared in path.hpp (PathProjection, Turn, PathLineCrossing, PathCircleCrossing,
//   PathSelfCrossing, PathArc), where the Path forwarders need them; osculating answers with bezier.hpp's
//   Osculating (center none where straight).
// - `outline(path, d1)` is the TS's `d2 = d1`; nearness takes a FalloffSpec (a name or a function).
// - selectAt takes SelectAtOptions (falloff, along, cut), each left out as the TS leaves it out.
#pragma once

#include "curves/bezier.hpp"
#include "curves/geometry.hpp"
#include "curves/offset.hpp"
#include "curves/path.hpp"

#include <functional>
#include <vector>

namespace curves {

/** The place on `path` nearest `p`: its progress, the point and the distance (in the metric). */
PathProjection project(const Path& path, Pt p);
/** Where `path` is level or upright inside its segments, as progress in order. */
std::vector<Progress> extrema(const Path& path);
/** Where `path` changes the way it bends inside its segments, as progress in order (corners at nodes aren't). */
std::vector<Progress> inflections(const Path& path);
/** Where `path` reaches furthest `way` (its extremes, nodes and ends considered), as progress; of points as far, the
 *  first. */
Progress side(const Path& path, Way way);
/** The nodes of `path` where it turns by at least `minDeg` degrees (inner nodes, and a closed path's start), in
 *  order. Directions are taken in the metric, so a turn is the angle seen on the page. */
std::vector<Turn> turns(const Path& path, double minDeg = 20);
/** How near each progress of `path` is to the nearest of `ats`, along it: `falloff` of the distance as a share of
 *  `reach` (in the metric's units), 1 at a place and 0 from `reach` on; round a closed path's start. */
std::function<double(Progress)> nearness(const Path& path, const std::vector<Progress>& ats, double reach,
                                         const FalloffSpec& falloff = FalloffName::smooth);
/** `path`'s exact bounds. */
Box bbox(const Path& path);
/** How tight `path` turns at progress `at`, in the metric: 1 / radius, positive turning left. */
double curvature(const Path& path, Progress at);
/** The unit normal of `path` at `at`, in the metric: its direction turned a quarter to the left. */
Pt normal(const Path& path, Progress at);
/** The circle that matches `path` at `at`: its center (none where the path is straight), radius in the metric and
 *  curvature. */
Osculating osculating(const Path& path, Progress at);
/** `path` with its point at progress `at` moved to `to` and its segment there bent through it (molded). */
Path mold(const Path& path, Progress at, Pt to);
/** The path at distance `d` to its left (negative: its right), in the metric, corners closed by a straight bevel. */
Path offset(const Path& path, const Reach& d);
/** A monoline stroke round `path`: `d1` to its left and `d2` (default `d1`) to its right (in the metric). */
std::vector<Contour> outline(const Path& path, double d1);
std::vector<Contour> outline(const Path& path, double d1, double d2);
/** Where `path` crosses the line from `a` to `b` (only between them unless `segment` is false), in order along the
 *  path. */
std::vector<PathLineCrossing> lineCrossings(const Path& path, Pt a, Pt b, bool segment = true);
/** Where `path` crosses the circle round `o` of radius `r` (in the metric). */
std::vector<PathCircleCrossing> circleCrossings(const Path& path, Pt o, double r);
/** Where `path` crosses itself: the two places, earlier first, and the point. */
std::vector<PathSelfCrossing> selfCrossings(const Path& path);
/** `path` as circular arcs within `threshold` (in the metric), each with the progress it spans. */
std::vector<PathArc> arcs(const Path& path, double threshold = 0.5);
/** A soft selection about the place at progress `at`: a node cut in there, selected wholly, and softened over
 *  `reach` with the falloff. With `cut` false the nearest node is the center and the path keeps its nodes. */
Selection selectAt(const Path& path, Progress at, double reach, const SelectAtOptions& o = {});

}  // namespace curves
