// Corners of closed outlines: where two pieces meet at an angle instead of running on smoothly. They come in two
// collections: pits (concave corners, where one stroke runs into another: Z's diagonal leaving its bar, b's bowl
// leaving its stem, E's arms, under A's bar), for ink traps and ink wells, and peaks (convex ones: the corners of
// square ends, A's apex). Corners are rounded by a radius (a font's softness) and shaped in or out against the ink
// (shapeCorners). And caps: the end of a stroke's outline reshaped by an open curve (a triangle, a rectangle, an arc,
// or any curve drawn across the stroke), done as a boolean operation on the outline.
//
// Outlines are taken as a boolean operation leaves them, the inside on the right of each piece (outer contours
// clockwise, holes counter-clockwise, y up), so a turn to the right is a convex corner and a turn to the left a
// concave one. A corner is rounded with a circular arc as a cubic tangent to both sides, each side cut back by
// r·tan(θ/2) and at most `share` of the side (the contour to the next corner). See corners.ts for the full reasoning
// on rounding, shaping and caps, and their known limits.
//
// Ported from fountain-ts/src/curves/corners.ts. Decisions:
// - Option objects ({ minDeg, share }) are CornerOpts with the TS defaults.
// - `radius: number | (k) => number` is two overloads of roundCorners. `shape: (k) => number | CornerShape` returns a
//   CornerShape, which a plain angle converts to; `profile` is a std::optional (an empty one still counts as given,
//   as `[]` is truthy in the TS).
// - `cap.align ?? 1` is a std::optional; `caps` is a namespace of functions with the TS defaults.
// - Contour lists are std::span<const Contour>; the TS `cs.slice()` copies are std::vector<Contour>.
#pragma once

#include "curves/geometry.hpp"

#include <functional>
#include <optional>

namespace curves {

/** A corner: on contour `contour`, where piece `at` starts (and the piece before it ends), at point `p`; `in` the
 *  direction the outline arrives by and `out` the one it leaves by (unit vectors); `turn` the angle between them in
 *  radians, negative to the right (convex, with the inside on the right) and positive to the left (concave); and
 *  `outside`, the angle of the wedge outside the outline at the corner, in degrees. */
struct Corner {
  int contour = 0, at = 0;
  Pt p, in, out;
  double turn = 0, outside = 0;
};

/** Every corner of the contours that turns by at least `minDeg` degrees. */
std::vector<Corner> corners(std::span<const Contour> cs, double minDeg = 1);

/** The pits of the contours: concave corners whose outside wedge is narrower than `maxDeg` degrees, straight or
 *  curved sides alike. Where strokes meet the ink pools, and the sharper the wedge (`outside`) the more; these are
 *  the spots for ink traps and ink wells. */
std::vector<Corner> pits(std::span<const Contour> cs, double maxDeg = 170);

/** The peaks of the contours: convex corners turning by more than `minDeg` degrees (the corners of square ends, A's
 *  apex, a flat end cut at a slant), the other collection beside the pits. */
std::vector<Corner> peaks(std::span<const Contour> cs, double minDeg = 10);

/** The options of replaceCorners and roundCorners. */
struct CornerOpts {
  double minDeg = 1, share = 0.5;
};

/** What goes in a corner's place: from `a` leaving along `ta` to `b` arriving along `tb`. */
using MakeCorner = std::function<std::vector<Piece>(Pt a, Pt ta, Pt b, Pt tb, const Corner& k)>;

/** The contours with corners replaced: each corner of at least `minDeg` that `reach` gives a length (0 or less leaves
 *  it) has both its sides cut back that far, at most `share` of each (a side the contour to the next corner, over
 *  pieces that run on smoothly), and `make` draws what goes between the cut points, from `a` leaving along `ta` to
 *  `b` arriving along `tb`. Rounding, fills and notches are all this. */
std::vector<Contour> replaceCorners(std::span<const Contour> cs, const std::function<double(const Corner&)>& reach,
                                    const MakeCorner& make, CornerOpts opts = {});

/** The contours with each corner of at least `minDeg` rounded by `radius` (0 leaves the corner sharp), each side cut
 *  back at most `share` of it (to the next corner). */
std::vector<Contour> roundCorners(std::span<const Contour> cs, double radius, CornerOpts opts = {});
/** roundCorners with a radius per corner. */
std::vector<Contour> roundCorners(std::span<const Contour> cs, const std::function<double(const Corner&)>& radius,
                                  CornerOpts opts = {});

/** How a corner is shaped (see `shapeCorners`): the angle, −1…1, and optionally a profile drawn in its place, or a
 *  curve drawn in the corner's own frame (see `cornerFrame`). */
struct CornerShape {
  double angle = 0;
  std::optional<std::vector<Piece>> profile, curve;
  CornerShape() = default;
  CornerShape(double angle) : angle(angle) {}
  CornerShape(double angle, std::vector<Piece> profile) : angle(angle), profile(std::move(profile)) {}
};

/** The frame a corner's replacement is drawn in: x from −1 at the first cut `a` to 1 at the second `b`, and y from 0
 *  on the chord between them to 1 at the corner `p` itself, an affine map either way (`to` into the plane, `from`
 *  back). Past 1 is beyond the corner: into the ink at a pit (a trap), out of it at a peak; below 1 is the other way
 *  (a well, a cove). A corner on its chord has no height: a unit square to the chord stands in. */
struct CornerFrame {
  Pt m, ex, ey;
  double det = 0;
  Pt to(Pt q) const;
  Pt from(Pt w) const;
};
CornerFrame cornerFrame(Pt a, Pt b, Pt p);

/** The contours with their corners shaped: each corner `size` gives a radius (0 or less leaves it) is replaced by a
 *  circular arc from where a fillet of that radius would start to where it would end, and the shape's `angle` sets
 *  the angle the arc meets the sides at, against the ink: 0 runs on along them (the fillet); outset, towards 1,
 *  tilts it out of the ink, adding ink; inset, towards −1, tilts it into the ink, taking ink away. At ±1 the arc
 *  leaves the sides square, at 90°; beyond, up to the arc closing on itself. A `profile` draws any open curve in the
 *  arc's place, across a box from x −1 at the corner's first cut to 1 at its second, and up, its bounding box's top
 *  as far off the chord as the arc for that angle bulges. A `curve` is drawn in the corner's frame instead
 *  (cornerFrame), its ends pinned to the cuts. `seen` hears each corner's cuts as they are made. */
std::vector<Contour> shapeCorners(std::span<const Contour> cs, const std::function<double(const Corner&)>& size,
                                  const std::function<CornerShape(const Corner&)>& shape,
                                  const std::function<void(const Corner&, Pt, Pt)>& seen = {});

/** A cap: its curve, open, across the stroke, and where the stroke's end sits in its box (1 bottom … −1 top). */
struct Cap {
  std::vector<Piece> shape;
  std::optional<double> align;
};

/** An end of a stroke: where its centreline ends, the direction it leaves by, outward, and the ink's half-width there
 *  (used where the outline cannot be measured). */
struct EndFrame {
  Pt p, t;
  double r = 0;
};

/** Standard cap curves across a box two units wide (x −1…1), `h` units high: their bounding box is what counts. */
namespace caps {
/** A point: up from one side to the tip and down to the other. */
std::vector<Piece> triangle(double h = 1);
/** A square end raised by `h`. */
std::vector<Piece> rect(double h = 0.5);
/** A half ellipse `h` high (a half circle at 1). */
std::vector<Piece> arc(double h = 1);
}  // namespace caps

/** The contours of a stroke with the end at `end` capped by `cap`. Where the booleans' loops do not close the end is
 *  left as it was. */
std::vector<Contour> capped(std::span<const Contour> cs, const EndFrame& end, const Cap& cap);

}  // namespace curves
