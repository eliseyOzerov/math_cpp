// Attaching a path to what is drawn already, by pairs: each pair says that the path's point at progress `from`
// lies on a target, B(s) = A(t). A target is a place on another path (progress `at`, within some of its segments),
// a fixed point, or an anchor found on the drawing, such as where two paths cross.
//
// How many pairs decide how the path moves, always as little as they need:
//   1  shifted onto the target;
//   2  turned and stretched along the line between the targets, its depth across that line kept (attach below);
//   3  mapped by the one affine map that takes the three points onto their targets (a shear joins the turn);
//   4+ mapped by the affine map nearest to all of them (least squares), then bent node by node: a node is cut in at
//      each pair's point and moved, with its handles, the rest of the way onto its target, and the nodes between two
//      pairs move by a blend of theirs by progress. A pair's progress is the path's before it is bent.
// Everything is affine up to three pairs, so the curves move exactly. Lengths and angles are taken in the metric.
//
// An alignment turns the path about its first pair's point (its start when there is none) so it runs along the
// tangent of another path at a progress, or along its normal, turned `angle` degrees more. It goes with at most one
// pair, since a second already decides the turn.
//
// Attachments go one way, in drawing order: B is placed on A as A is now, so a change to A shows in B when B is
// placed again after it.
//
// Ported from fountain-ts/src/curves/attach.ts.
//
// Decisions (C++ mapping):
// - Mount is a struct with optionals; `b` of attach is std::optional<Mount>.
// - Target is a std::variant of TargetOn, TargetPoint and TargetCross (the TS's three object shapes); a Pair is a
//   Target with an optional `from`. Align's `to` is AlignTo (tangent by default).
// - The TS's `Path.attach` (AttachEnd, AttachOptions) is declared in path.hpp and defined in attach.cpp.
#pragma once

#include "curves/geometry.hpp"
#include "curves/path.hpp"

#include <optional>
#include <variant>
#include <vector>

namespace curves {

/** Where a path is attached: the point at progress `from` along it (0 its start) goes to progress `at` along `on`,
 *  or along `on`'s segments `segs` only (see Segs). */
struct Mount {
  Path on;
  Progress at;
  std::optional<Progress> from;
  std::optional<Segs> segs;
};

/** `p` placed by mount `a` (by default its start), and by `b` as well when given (by default its end): shifted onto
 *  one point, or turned and stretched along the line between two, its depth across it kept. Two mounts that land on
 *  one point, or take two points of `p` that coincide, place it by the first alone. */
Path attach(const Path& p, const Mount& a, const std::optional<Mount>& b = {});

/** A place on another path. */
struct TargetOn {
  Path on;
  Progress at;
  std::optional<Segs> segs;
};
/** A fixed point. */
struct TargetPoint {
  Pt point;
};
/** Where `cross` crosses `with` (the first crossing along `cross`, or `which`). */
struct TargetCross {
  Path cross, with;
  std::optional<Which> which;
};
/** Where a point of the attached path goes. */
using Target = std::variant<TargetOn, TargetPoint, TargetCross>;

/** One pair: the attached path's point at progress `from` goes to the target. By default the pairs take the path's
 *  start, its end, and places evenly between for more. */
struct Pair {
  Target target;
  std::optional<Progress> from;
};

enum class AlignTo { tangent, normal };
/** The path turned to run along `on`'s tangent at `at` (or its normal), plus `angle` degrees, as measured at the
 *  path's progress `from` (by default its first pair's, or 0). */
struct Align {
  Path on;
  Progress at;
  std::optional<Segs> segs;
  AlignTo to = AlignTo::tangent;
  std::optional<double> angle;
  std::optional<Progress> from;
};

/** The point a target names; throws where an anchor isn't there, so a script fails where it is wrong. */
Pt target(const Target& t);

/** `p` placed by `pairs` and then turned by `align`. */
Path place(const Path& p, const std::vector<Pair>& pairs, const std::optional<Align>& align = {});

}  // namespace curves
