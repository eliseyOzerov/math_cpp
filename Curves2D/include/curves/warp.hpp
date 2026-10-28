// Warps: any map of the plane applied to curves and to nodes, and the warps made of it: bows across a height (a
// spine), shifts by a function of height, wobble, a smooth noise field, and a mesh of nine points over a box.
//
// Decisions: a cubic is warped by mapping its ends and carrying each handle by the warp's derivative there (a
// numerical Jacobian), so the curve leaves and arrives in the warped directions and joins stay smooth where they
// were. That is exact for affine maps, and for a shift that is quadratic in height applied to a cubic rising evenly
// (a straight stroke under a bow), as the Swift pose was; for anything else a cubic is first cut into `pieces` so the
// warped curve follows the map closely. Cutting adds nodes, so warp after the nodes have been picked and moved.
//
// Wobble is a field over the plane, not a function along each curve, so two curves through the same place move
// the same way there: strokes that overlap still meet, and a ring still closes.
//
// Ported from fountain-ts/src/curves/warp.ts.
//
// Decisions (C++ mapping):
// - Warp is a std::function<Pt(Pt)>; the TS tuple [Pt, Pt] of jacobian is std::array<Pt, 2>.
// - warpNode is a template over any node type with members `p` (Pt) and `in`, `out` (std::optional<Pt>); it is
//   copied, so any other members are kept, as the TS object spread does.
// - WarpOptions' optional numbers are std::optional (pieces, step) and eps defaults to 1e-3.
// - spine's `dx` is a std::variant<double, Wave>: a bare number bows.
// - Wobble is a struct with the TS defaults (frequency 8, octaves 1, seed 0). noise2 returns a std::function.
// - warpContours returns Contours whose pieces are all cubics.
#pragma once

#include "curves/geometry.hpp"
#include "curves/profile.hpp"

#include <array>
#include <functional>
#include <optional>
#include <variant>
#include <vector>

namespace curves {

/** A map of the plane. */
using Warp = std::function<Pt(Pt)>;

/** The warp's derivative at p, as the images of the unit steps (columns of the Jacobian). */
std::array<Pt, 2> jacobian(const Warp& f, Pt p, double eps = 1e-3);

/** A node (its place and its handles as offsets from it) warped by `f`: its place mapped, its handles carried by the
 *  derivative there, so it keeps its nodes and its joins stay smooth (exact for affine maps). */
template <class N>
N warpNode(N n, const Warp& f, double eps = 1e-3) {
  std::array<Pt, 2> j = jacobian(f, n.p, eps);
  Pt jx = j[0], jy = j[1];
  auto by = [&](Pt v) -> Pt { return {jx[0] * v[0] + jy[0] * v[1], jx[1] * v[0] + jy[1] * v[1]}; };
  Pt q = f(n.p);
  if (n.in) n.in = by(*n.in);
  if (n.out) n.out = by(*n.out);
  n.p = q;
  return n;
}

/** The nine points of a 3×3 mesh at rest over the box `lo`…`hi`: rows from the bottom, x first in each. */
std::vector<Pt> meshGrid(Pt lo, Pt hi);

/** The warp of a 3×3 mesh over the box `lo`…`hi` with its nine points moved to `grid` (as meshGrid orders them):
 *  each point is the grid's blend by quadratic Lagrange weights across and up, so the plane bends smoothly through
 *  the nine points, is the identity at rest and is affine while the grid is. A box with no width or height is taken
 *  as one unit that way. */
Warp mesh(Pt lo, Pt hi, std::vector<Pt> grid);

/** How finely to warp: `pieces` per cubic (1 keeps the nodes), or one piece per `step` of a cubic's control polygon
 *  (at least one), and the step of the numerical derivative. */
struct WarpOptions {
  std::optional<double> pieces, step;
  double eps = 1e-3;
};

/** A cubic warped by `f`, as one cubic or cut into pieces first (see WarpOptions). */
std::vector<Cubic> warpCubic(const Cubic& c, const Warp& f, const WarpOptions& o = {});

/** Cubics warped by `f`, in order. */
std::vector<Cubic> warpCubics(const std::vector<Cubic>& cs, const Warp& f, const WarpOptions& o = {});

/** Closed contours warped by `f`: lines become cubics, so they can bend. */
std::vector<Contour> warpContours(const std::vector<Contour>& cs, const Warp& f, const WarpOptions& o = {});

/** A shift across by `dx` of the share of the way from `y0` up to `y1` (beyond them too): a spine. A bare number
 *  bows: that far across halfway up, none at y0 and y1 (quadratic, so a straight upright stroke bows exactly). */
Warp spine(std::variant<double, Wave> dx, double y0, double y1);

/** A warp whose push up or down fades out within `reach` of any of the lines y = `ys` (smoothly, none on them), so
 *  what sits on those lines stays there while it moves along them. Across it moves as before. */
Warp pinned(Warp f, std::vector<double> ys, double reach);

/** A smooth noise field: its `amplitude`, how far it moves a point each way at most, in units; its `frequency`, how
 *  many bends it makes in a thousand units (an em in most fonts; 8 puts them about 125 apart); `octaves`, finer
 *  layers each at twice the frequency and half the amplitude; and a `seed` for another field of the same kind. */
struct Wobble {
  double amplitude = 0, frequency = 8, octaves = 1, seed = 0;
};

/** How far apart a wobble's bends are, in units. */
double wavelength(const Wobble& w);

/** A wobble's displacement field as a warp. */
Warp wobble(const Wobble& w);

/** How finely a wobble should cut cubics for its warp to follow the field: a piece per half bend of its finest
 *  octave. At amplitude 12 and frequency 8 that strays at most 0.25 units from the warped points (a quarter bend:
 *  0.03, but twice the pieces, and every piece of a centerline is inked and merged; a whole bend: 2 units). */
double wobbleStep(const Wobble& w);

/** A wobble along a line `length` units long, as a wave of the place along it (0…1) around 1: 1 ± `amplitude` at most
 *  (so here amplitude is a share, 0.3 for ±30%), bending `frequency` times per thousand units. Read along a straight
 *  run of the field, or round a circle as long as the line when it is `closed`, so a ring's ends meet. */
Wave wobbleWave(const Wobble& w, double length, bool closed = false);

/** A seeded value for `key`, −1…1, the same for the same seed and key on every machine and unrelated between keys
 *  (mulberry32 on the two mixed): for a jitter per item, as a glyph's bounce by its code point. */
double scatter(double seed, double key);

/** Seeded 2D gradient noise (Perlin's), about −1…1, smooth, the same for the same seed on every machine. */
std::function<double(double, double)> noise2(double seed);

}  // namespace curves
