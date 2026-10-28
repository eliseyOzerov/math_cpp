// The ink: a nib stamped along a stroke's centreline, shaped by the stroke's curves and trimmed at its ends.
//
// `ink` runs in steps: sample the centreline into points (sweeping round sharp corners), run the cut ends on and set
// the planes that trim them, head each point along the stroke, stamp the nib at each, and join neighbouring stamps by
// their hulls. The floats follow the TS step for step, so arithmetic keeps its order.
//
// Ported from fountain-ts/src/ink/ink.ts.
//
// Decisions:
// - InkEnd (the TS End; path.hpp has an End of its own) is an enum; Cut's mode, InkOptions' caps and model are enums too.
// - Keys left out are std::nullopt; a heading not set is std::nullopt.
#pragma once

#include "curves/fit.hpp"
#include "curves/geometry.hpp"
#include "curves/nib.hpp"
#include "curves/profile.hpp"

#include <optional>
#include <vector>

namespace curves {

enum class InkEnd { start, end };
/** A terminal's kind: 'auto' cut level on the given levels and otherwise as the caps say; 'flat' trimmed square
 *  across the stroke; 'nib' the nib's own end, untrimmed; 'round' a disc as wide as the ink; 'level' along the
 *  horizontal; 'angle' at `deg` from the horizontal. */
enum class CutMode { auto_, flat, nib, round, level, angle };
/** A terminal: its kind, and `taper` scaling the nib by 1 + taper at the end, easing in over `reach` units (150 by
 *  default). */
struct Cut {
  CutMode mode = CutMode::auto_;
  std::optional<double> deg, taper, reach;
};
/** Each end's cut, either left out. */
struct Cuts {
  std::optional<Cut> start, end;
};
/** A stroke as the ink reads it: its chord S→E (by default its first segment's start and last segment's end), its
 *  cubic segments, whether it closes on itself, a weight scale, its end cuts and the nib's curves along it: `size`,
 *  `rotate` (degrees), `follow`, `offset` (fraction of the ink's half-width) and `shapes`. */
struct InkStroke {
  std::optional<Pt> S, E;
  std::vector<Cubic> segments;
  bool closed = false;
  std::optional<double> weight;
  std::optional<Cuts> cuts;
  std::optional<Keys<double>> size, rotate, follow, offset;
  std::optional<Keys<Shape>> shapes;
};
enum class Caps { butt, nib };
enum class Model { broad, pointed };
/** How to ink: `caps` for ends with no cut, whether `cuts` are on, the `levels` that cut a bare end level, the stamps
 *  per curve segment (or, with `spacing`, as many as keep them at most that far apart), and the contrast `model`. */
struct InkOptions {
  Caps caps = Caps::butt;
  bool cuts = true;
  std::vector<double> levels;
  std::optional<double> perSpan, spacing;
  Model model = Model::broad;
};
/** A plane that trims the hulls near one end of the stroke. */
struct EndPlane : Plane {
  InkEnd end;
};
/** The ink of a stroke: `segments` the centreline (with run-ons), `pts` it as a polyline with each point's place `ts`
 *  along the stroke, `widths` the ink's width across there, `stamps` the nib placed there and `frames` the same
 *  exactly, `polys` the convex hulls of neighbouring stamps (their union is the ink), `nodes` the indices where the
 *  skeleton's segments meet and `heads` the headings set for a sharp corner's sweep. */
struct InkResult {
  std::vector<Cubic> segments;
  double weight;
  std::vector<EndPlane> planes;
  std::vector<Pt> pts;
  std::vector<double> ts, widths;
  std::vector<std::vector<Pt>> stamps;
  std::vector<Frame> frames;
  std::vector<std::vector<Pt>> polys;
  std::vector<int> nodes;
  std::vector<std::optional<double>> heads;
  Caps caps;
  bool closed;
};

/** The ink of an evaluated stroke under a nib: the nib stamped along the stroke's centreline, shaped, sized, turned
 *  and moved across by the stroke's curves at each point, and each pair of neighbouring stamps joined by their convex
 *  hull. See the TS for the ends. */
InkResult ink(const InkStroke& st, const NibInput& nib = {}, const InkOptions& opts = {});

}  // namespace curves
