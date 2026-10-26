// Selections: some of the nodes of one or more paths, moved together. A node is a point where two segments meet
// (and a path's ends); its handles go with it, while the handles of the nodes left out stay, so the curves between
// a moved node and a still one are pulled into shape. Moves are made in the paths' metric, so a turn stays a turn
// however a path is scaled.
//
// Why selections: every operation on a glyph is a generic function on a selection of curves, so it works on any
// path (a stem, a lobe, an oval, a part of another glyph) and becomes a slider in a panel, rather than a feature of
// one kind of curve. Before adding something for one kind of curve, try a parameter that makes it work on all.
//
// Weights: every node of a selection has a weight, 0…1, and every move takes it that share of the way (a turn by
// that share of its angle, an inflation by that share of its radius, smoothing and fairing by that share of their
// amount), so a selection can be soft from the start: picked by a function giving weights, spread with `soften`
// (proportional editing, as in Blender), tapered with `weigh`. A whole selection is the case of weights 1.
//
// Fairing (fairNodes): continuous radii where segments meet along a tangent. Where a line meets a curve, a
// transition cubic is put between them that leaves the line straight and reaches the curve `reach` of the way along
// it with the curve's own curvature there (LEAN = 0.3 of the way from its start to the corner, solved from the
// curvature); where two curves meet along a tangent, their handles at the node are resized so both have the
// geometric mean of the two curvatures. Corners and lines meeting lines are left. See selection.ts for the reasons
// at length, including the known issue that at reach 1 one cubic overshoots the curvature by about 40%.
//
// Ported from fountain-ts/src/curves/selection.ts.
//
// Decisions (C++ mapping):
// - Selection::and is `and_` (`and` is a C++ keyword); it takes one selection or a vector of them.
// - Pivot is std::optional<Pt>: none is the TS 'center'. Place (geometry.ts) is a struct of two optional coordinates,
//   defined here since geometry.hpp has none; p(x, y) is Place{x, y}.
// - Option objects are structs with the TS defaults (SoftenOptions, HandlesOptions, InflateOptions, ShearOptions).
//   `soften`'s falloff and along are optionals, left out for the TS defaults (smooth, path).
// - smoothNodes' `amount` (a number or a function of the node) is two overloads. scale takes a number or a Pt.
// - Selection::of takes a NodePick or a Pick (overloads); NodePick's function returns the weight (true is 1).
// - Picked has public fields; Selection is an immutable value like Path.
#pragma once

#include "curves/geometry.hpp"
#include "curves/path.hpp"
#include "curves/profile.hpp"

#include <functional>
#include <optional>
#include <vector>

namespace curves {

/** Nodes `i…` (0 the start, then each cubic's end; -1 the last node). */
Pick nodeAt(std::vector<int> i);
/** Whole cubics `i…` (cubic i runs from node i to node i + 1; -1 the last), as their end nodes, so the cubics on
 *  either side bend to follow. */
Pick cubicAt(std::vector<int> i);
/** The stretch of a path from progress `from` to `to` (0…1 along its length): nodes are cut in at both ends where
 *  there are none (by de Casteljau, so the shape is kept), and every node from one to the other is taken. */
Pick section(Progress from, Progress to);

/** The falloff a name or a function stands for (smooth, linear, sphere, root, sharp, constant: Blender's names). */
Falloff falloff(const FalloffSpec& f);

/** What a selection turns or shears about: none is the middle of its nodes' bounds, or a point. */
using Pivot = std::optional<Pt>;

/** Where to move something, either coordinate left open to keep it. */
struct Place {
  std::optional<double> x, y;
};

/** Segments `segs` with nodes `nodes` and their handles moved by `f` (given each point and the index of the node it
 *  belongs to); a closed run's end moves with its start. Node i is the end of segment i − 1 and the start of
 *  segment i; its in-handle is segment i − 1's second control point, its out-handle segment i's first. Handles move
 *  with their node so that an affine move keeps the curve's shape at the node exactly; only the segments between a
 *  moved and a still node change shape. */
std::vector<Cubic> moveNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes,
                             const std::function<Pt(Pt, int)>& f);

/** Segments `segs` with the handles of nodes `nodes` turned and sized to run smoothly through the nodes, `amount` of
 *  the way (0 leaves them, 1 is fully smooth), measured with `metric`; `amount` can differ node by node. */
std::vector<Cubic> smoothNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes,
                               const std::function<double(int)>& amount, Pt metric = {1, 1});
std::vector<Cubic> smoothNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes,
                               double amount = 1, Pt metric = {1, 1});

/** How far out `p` goes when inflated by `r` from `from`: `r` along the way from `from` to it (in, when `r` is below
 *  0); (0, 0) when it lies on `from`. */
Pt inflation(Pt p, Pt from, double r);

/** What fairNodes gives back: the segments, the selected nodes renumbered and their weights by the new numbers. */
struct Faired {
  std::vector<Cubic> segs;
  std::vector<int> nodes;
  std::vector<double> weights;
};
/** Segments `segs` with continuous radii at the nodes `nodes` where they meet along a tangent, worked in the units
 *  of `metric`; a line eases into a curve over `reach` times the node's weight (`weights`, by node, 1 if left out). */
Faired fairNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes, double reach = 0.5,
                 Pt metric = {1, 1}, const std::optional<std::vector<double>>& weights = {});

/** A path and its nodes' weights (one per node, 0 for a node left out), with the nodes that have any. */
struct Picked {
  Path path;
  std::vector<int> nodes;
  std::vector<double> weights;
};

/** How soften spreads: the falloff (smooth) and along the path or the plane (path). */
struct SoftenOptions {
  std::optional<FalloffSpec> falloff;
  std::optional<Along> along;
};
/** How handles turns and sizes: `angle` aims (degrees), `turn` turns from where they point, `length` scales. */
struct HandlesOptions {
  std::optional<double> angle;
  double turn = 0, length = 1;
  HandleLead side = HandleLead::both;
};
/** Where inflate reads a node's place for its strength: round the ring about the point, or along its path. */
enum class InflateAlong { ring, path };
struct InflateOptions {
  Pt align = {0.5, 0.5};
  double handles = 1, smooth = 0;
  Strength strength = 1.0;
  InflateAlong along = InflateAlong::ring;
};
struct ShearOptions {
  double axis = 0;
  Pivot about;
};

/** A map for each selected node: given the node, its path and its index, the map its handles and it go through. */
using NodeMap = std::function<std::function<Pt(Pt)>(Pt, const Path&, int)>;

class Selection {
 public:
  std::vector<Picked> picked;

  explicit Selection(std::vector<Picked> picked) : picked(std::move(picked)) {}

  /** The nodes of `path` that `pick` takes, and how much; all of them wholly by default. A closed path's end is its
   *  start, so picking node `segs.length` picks node 0. */
  static Selection of(Path path);
  static Selection of(Path path, const NodePick& pick);
  static Selection of(Path path, const Pick& pick);

  /** This selection and `others` together, moved as one. */
  Selection and_(const Selection& other) const;
  Selection and_(const std::vector<Selection>& others) const;

  /** Each path's weights, by node. */
  std::vector<std::vector<double>> weights() const;
  /** The points the pivot and bounds are worked from: the wholly selected nodes (weight 1), or every node with any
   *  weight when none is whole. */
  std::vector<Pt> points() const;
  /** The middle of the selected nodes' bounds. */
  Pt center() const;
  /** The bounds of the selected nodes. */
  Box bounds() const;
  /** The metric the moves are made in: the first path's. */
  Pt metric() const;

  /** The selection spread to the nodes within `reach` (in the metric) of its nodes, each taking a nearer node's
   *  weight times `falloff` of how far it lies as a share of the reach: proportional editing. */
  Selection soften(double reach, const SoftenOptions& o = {}) const;
  /** The weights times `strength` at each node's progress along its path, clamped to 0…1. */
  Selection weigh(const Strength& strength) const;
  /** Every node of the paths weighted the other way round: 1 − its weight. */
  Selection invert() const;

  /** The paths with their selected nodes moved by `m`, made in the metric, each `w` of the way for its weight `w`. */
  Selection transform(const Affine& m) const;
  /** The paths with each selected node and its handles moved by the map `f` gives for that node, each point taken
   *  its node's weight of the way to where the map puts it. */
  Selection moveEach(const NodeMap& f) const;
  /** Moved so its middle sits at `to`, in the paths' own coordinates. */
  Selection put(const Place& to) const;
  /** The selected nodes set to `kind` (a corner made smooth is first smoothed); the weights kept. */
  Selection kind(NodeKind kind) const;
  /** The selected nodes' handles turned and sized, in the metric, each its node's weight of the way. */
  Selection handles(const HandlesOptions& o) const;
  /** Inflated by `r` (shrunk, below 0), in the metric, straight away from the point `align` of the way across and up
   *  the selection's bounds. */
  Selection inflate(double r, const InflateOptions& o = {}) const;
  /** Handles turned and sized to run smoothly through the nodes, `amount` times each node's weight of the way. */
  Selection smooth(double amount = 1) const;
  /** Continuous radii at the selected nodes where segments meet along a tangent; easing adds a node, so the
   *  selection takes both ends of each transition. */
  Selection fair(double reach = 0.5) const;

  /** Turned `deg` counter-clockwise about `about`, each node by its weight's share of the angle. */
  Selection rotate(double deg, const Pivot& about = {}) const;
  /** Sheared along the axis at `axis` degrees through `about`, so lines across that axis lean `deg`. */
  Selection shear(double deg, const ShearOptions& o = {}) const;
  /** Moved `dist` (in the metric) along the axis at `axis` degrees: 0 across, 90 up. */
  Selection shift(double dist, double axis = 0) const;
  /** Scaled by `k` (or {across, up}) about `about`, each node by its weight's share of the change. */
  Selection scale(double k, const Pivot& about = {}) const;
  Selection scale(Pt k, const Pivot& about = {}) const;
  /** Mirrored in the line at `axis` degrees through `about` (90, the default, flips left and right). */
  Selection mirror(double axis = 90, const Pivot& about = {}) const;

  /** The moved paths, in the order they were selected. */
  std::vector<Path> paths() const;
  /** Other nodes of the moved paths, so edits chain; the same pick on every path, the weights start afresh. */
  Selection select() const;
  Selection select(const NodePick& pick) const;
  Selection select(const Pick& pick) const;
  /** The moved path, when only one path was selected; throws otherwise. */
  Path path() const;

 private:
  Selection moveBy(const std::function<Affine(double)>& at) const;
  Selection reshape(const std::function<std::function<Pt(Pt)>(Pt, const Path&, int, double)>& f) const;
  Pt pivot(const Pivot& about) const;
};

/** Several paths as one, every node taken: one pivot and one bounds. Pick within it with `.select(pick)`. */
Selection group(const std::vector<Path>& of);
Selection group(const std::vector<Selection>& of);

}  // namespace curves
