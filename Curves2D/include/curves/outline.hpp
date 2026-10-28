// The outline: an inked stroke as a closed contour of cubics and lines. The two edges of the ink (the rails) are
// found where the stamps' envelope touches each stamp, trimmed by the end planes, untangled, and fitted.
//
// The anchors of each rail follow a plan (pieces per skeleton span). A trimmed end is a straight line along its
// plane; an untrimmed one is the end stamp's far side, in pieces of equal length. The contour runs along the left
// rail, across the end, back along the right rail and across the start: clockwise with y up.
//
// Ported from fountain-ts/src/ink/outline.ts.
//
// Decisions:
// - A plan's end is `std::nullopt` for 'line', else its piece count.
// - The TS's `outline` is `outlineInk` here: geometry and analysis already have outlines of their own.
#pragma once

#include "curves/geometry.hpp"
#include "curves/ink.hpp"

#include <optional>
#include <string>
#include <vector>

namespace curves {

/** The fitting's settings: the `tolerance` in font units and the `most` pieces a span or run may take. */
struct OutlineSettings {
  double tolerance = 1;
  int most = 6;
};
extern const OutlineSettings OUTLINE;

/** Each end's runs between the nib's sharp corners ('l' straight, 'c' rounded). */
struct Runs {
  std::string start, end;
};
/** The plan: pieces per skeleton span on each rail, and per rounded run of each end (nullopt: a line, where a plane
 *  trims it). */
struct OutlinePlan {
  std::vector<int> left, right;
  std::optional<int> start, end;
  std::optional<Runs> runs;
};
/** The worst fit error in each skeleton span of each rail, and at each end. */
struct Errs {
  std::vector<double> left, right;
  double start, end;
};
/** An outline: `contour` (the first of `contours`, which hold a closed stroke's two rings), the plan it was fitted
 *  to, its anchor count, and its worst error. */
struct OutlineResult {
  Contour contour;
  std::vector<Contour> contours;
  OutlinePlan plan;
  int anchors;
  double err;
  Errs errs;
};

/** The plan an inked stroke needs alone. */
OutlinePlan outlinePlan(const InkResult& r, double tolerance = 1);
/** One plan for several inks of the same stroke: the most pieces any of them needs in each place. Nullopt when they
 *  can't share one. */
std::optional<OutlinePlan> sharedPlan(const std::vector<OutlinePlan>& plans);
/** An inked stroke as a closed contour of cubics and lines, fitted to `plan` (its own if none). */
OutlineResult outlineInk(const InkResult& r, const std::optional<OutlinePlan>& plan = std::nullopt, double tolerance = 1);
/** A plan's shape as text: equal for outlines that interpolate. */
std::string planKey(const OutlinePlan& p);

}  // namespace curves
