// Ready-made paths. Soft polylines: each side bowed out or in, each inner corner rounded by a circular-ish cubic that
// meets the sides along their tangents, so the result is smooth wherever it is rounded; they work in plain units (the
// same across and up), and the builders map points into a metric and back. On them: `soft`, a soft polyline as a
// path; lobes, soft polygons bowed out from a line, from one end round to the other, their character set by how many
// corners they have, how round the corners are and how the sides bow (D, B, P and R are lobes off a stem); and ovals.
//
// Why a lobe hangs off a chord: a lobe is generic, so it hangs off the line between its own ends rather than off a
// vertical stem; one off a leaning stem (an italic D) leans with it for free. Where the ends don't lie on the line
// the lobe should hang off (B's lower lobe starts on the upper lobe, not on the stem), `axis` names that line instead.
// `reach` is measured from the line by default so lobes off one stem line up however their starts lie;
// `reachFrom: start` measures it from the start, for a lobe that should reach a set distance past the point it
// grows from.
//
// Ported from fountain-ts/src/curves/shape.ts.
//
// Decisions:
// - Softness's bulge, one number or one per side, is a variant; the option objects are structs with the TS defaults,
//   and a lobe's shape and place (LobeShape & LobePlace) are one struct, LobeOptions.
#pragma once

#include "curves/geometry.hpp"
#include "curves/path.hpp"

#include <optional>
#include <variant>
#include <vector>

namespace curves {

/** How to soften a polyline: `radius` (0…1) is how far each corner is rounded, as a share of the room its sides
 *  allow (half a side shared with another rounded corner, all of a side that ends the polyline); `bulge` is each
 *  side's sagitta as a share of its length, to the left of the direction of travel (negative bows it right): one
 *  number for every side or one per side. */
struct Softness {
  double radius = 0;
  std::variant<double, std::vector<double>> bulge = 0.0;
};

/** A side from a to b as a cubic, bowed `bulge` of its length to its left. */
Cubic bowed(Pt a, Pt b, double bulge = 0);

/** Each side's bulge, scaled down where needed so that no inner corner turns past MAX_TURN (11/12 of a half turn).
 *  A bowed side's ends turn atan(4·bulge) towards the side it bows to, so a corner between sides a and b turns
 *  φ + atan(4·bₐ) + atan(4·b_b): bowing in sharpens a corner, and too far makes the sides cross and the line loop
 *  round the tip. */
std::vector<double> fitBulges(const std::vector<Pt>& pts, const std::vector<double>& bulges);

/** `path` with every segment bowed `amount` of its chord away from `from` (below 0 towards it): both handles pushed
 *  along the chord's normal, as `bowed`, so the nodes and the turns at them hold while the runs between swell. A
 *  segment that comes back to its start (no chord) is kept. */
Path bend(const Path& path, double amount, Pt from);

/** A knob's shape: `size` halves across, `length` times that along, set `side` of the curve. */
struct KnobShape {
  double size = 1.6, length = 1, side = 0;
};

/** A closed knob on the end at `p` that leaves along `t` (outward), the curve `half` wide there: an ellipse `size`
 *  halves across and `length` times that along `t`, its centre set `side` of the curve (+1 its left, −1 its right, 0
 *  on it) by its radius less `half`, so one edge runs on from the curve's edge on the other side and it swells
 *  towards `side`, as a c's ball hangs into its counter. The curve's end, rounded, lies inside it for `size` ≥ 1. */
std::vector<Cubic> knob(Pt p, Pt t, double half, const KnobShape& k = {});

/** A curve run on from `p` along `t` for `length`: a circular arc turning `turn` degrees on the way (positive to
 *  the left of `t`), straight for 0. One cubic, so keep `turn` within about 120°. */
Cubic runOn(Pt p, Pt t, double length, double turn = 0);

/** The polyline through `pts` (open, its ends kept sharp) as cubics, its sides bowed (as far as its corners allow,
 *  see fitBulges) and inner corners rounded. */
std::vector<Cubic> softPolyline(const std::vector<Pt>& pts, const Softness& softness = {});

/** The polyline through `pts` made soft (see softPolyline): each inner corner a circular arc meeting its sides along
 *  their tangents, `radius` of the room the sides allow, and the sides bowed by `bulge`. Worked in `metric`, so a
 *  corner in a narrow box is round in font units.
 *
 *  Why not poly then smooth: smoothing a polyline's corner part of the way leaves a kink in it, and all the way
 *  turns it on handles a third of each side long, so the sides between bow and a long upright leans; an arc sized
 *  by its own radius keeps the turn smooth and the sides straight. */
Path soft(const std::vector<Pt>& pts, const Softness& softness = {}, Pt metric = {1, 1});

/** Where an oval starts. */
enum class Side { bottom, right, top, left };

struct OvalOptions {
  double tension = KAPPA;
  Side start = Side::bottom;
};

/** An ellipse in the box from (x0, y0) to (x1, y1), closed, running counter-clockwise from its bottom or `start`. */
Path oval(double x0, double y0, double x1, double y1, const OvalOptions& o = {});

enum class ReachFrom { line, start };

/** A lobe's character and where it sits. Character: `nodes` corners out from the line (1 a point, 2 a box, 3 a half
 *  hexagon, …), `radius` (0…1) how round they are (2 nodes at 1 is a half circle), `bulge` how far each side bows
 *  out (above 0) or in, as a share of its length (as a Softness's); sides along the lobe's top and bottom stay
 *  straight. Place: `axis` the direction in degrees of the line it is off, pointing from its end to its start (the
 *  chord from `b` to `a` by default; 90 is a stem straight up), and `reach` how far out it goes, in units across
 *  (the metric's first), from that line (`ReachFrom::line`) or from its start (`ReachFrom::start`). */
struct LobeOptions {
  double reach = 1;
  double nodes = 2;
  double radius = 1;
  double bulge = 0;
  std::optional<double> axis;
  ReachFrom reachFrom = ReachFrom::line;
};

/** A lobe from `a` round to `b`, bowing out to the right of the line from `b` to `a`, measured with `metric` (so
 *  corners stay round in its units). The line runs at `axis` through whichever end lies further back; the lobe's
 *  box runs out from it to `reach` and between the ends along it, and its corners sit where rays from the box's
 *  middle on the line, spread evenly from along the axis round to against it, meet the box. Throws when the lobe
 *  has no room. */
Path lobe(Pt a, Pt b, const LobeOptions& o = {}, Pt metric = {1, 1});

}  // namespace curves
