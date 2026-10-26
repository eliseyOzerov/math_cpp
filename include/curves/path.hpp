// A path: cubic segments end to end, built from a start point by lines, curves and elliptical quarters, and measured
// with a metric (what a unit across and a unit up are worth). Paths are values: every step returns a new path, so a
// path can be reused and grown elsewhere without changing the first.
//
// Attachments place a path on another at a progress along it (see attach.hpp). Node kinds (see NodeKind) say how a
// node's two handles hang together, so editing one handle of a smooth node turns the other with it. A built path
// reads its kinds off its handles; a kind set with `withKind` is kept by every step that keeps the nodes (map, warp,
// moves of a selection, reverse, startAt, join). Known limit: steps that cut or add nodes (slice, aperture, divide,
// fair) read the kinds afresh.
//
// Ported from fountain-ts/src/curves/path.ts (its header has the reasons at length). path, selection, analysis and
// attach call each other, as in the TS; this header holds the small types they share so each can include it alone.
//
// Decisions (C++ mapping):
// - Path is an immutable value class with the TS's public fields as const accessors (start(), segs(), closed(),
//   arrives(), metric()); getters (length, end, kinds, nodeCount, nodes, distances, angle) are methods.
// - Segment lengths are measured lazily and cached, as the TS does; the cache is not synchronised, so measure a Path
//   from one thread at a time (copies are independent).
// - Heading, NodeKind and HandleSide are enum classes; Which is std::variant<End, int>; Segs is a struct built from
//   one index or two (`one` remembers which, for the error message).
// - The methods that answer with a Selection (select, selectAt) and the analysis forwarders are defined in
//   selection.cpp and analysis.cpp; include selection.hpp to call select.
// - NodePick's predicate returns a weight (a TS `true` is 1, `false` 0). Falloffs are std::function<double(double)>,
//   or named by FalloffName; options with a TS default left out are std::optional.
// - attach's `end` (a number or a Mount-like object) is an AttachEnd with an optional `on`; a bare number is
//   AttachEnd{.at = x}.
#pragma once

#include "curves/bezier.hpp"
#include "curves/geometry.hpp"
#include "curves/offset.hpp"
#include "curves/warp.hpp"

#include <functional>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace curves {

/** Which way a quarter leaves its start: along the horizontal or the vertical. It arrives along the other. */
enum class Heading { h, v };

/** Progress along a path: a share of its length, 0 at its start and 1 at its end, by length in the metric. */
using Progress = double;

/** Some of a path's segments, by index from 0 (below 0 from its end, -1 the last): one, or a run from the first to
 *  the second, both included. */
struct Segs {
  int a, b;
  bool one;
  Segs(int i) : a(i), b(i), one(true) {}
  Segs(int a, int b) : a(a), b(b), one(false) {}
};

/** How a node's two handles hang together: free (a corner), on one line through the node (smooth), or on one line
 *  and as long as each other (symmetric). An open path's ends have one handle and are corners. */
enum class NodeKind { corner, smooth, symmetric };

/** Which of a node's handles: the one arriving (the end of the segment before it) or the one leaving. */
enum class HandleSide { in, out };
/** A handle side, or both of a node's handles. */
enum class HandleLead { in, out, both };

/** The first or last of several crossings. */
enum class End { first, last };
/** Which crossing to take where a path crosses a level more than once: the first, the last, or by index. */
using Which = std::variant<End, int>;

class Path;
class Selection;
struct Pick;

/** Which nodes to take, and how much: their indices (0 is the start, then each segment's end), each wholly, or a
 *  function of each node's point, index and progress along the path giving its weight, 0…1. */
using NodePick = std::variant<std::vector<int>, std::function<double(Pt, int, Progress)>>;

/** How a softened selection's weight falls off with a node's distance, as a share of the reach: 1 at 0, 0 at 1. */
using Falloff = std::function<double(double)>;
/** The falloffs, as Blender's proportional editing names them. */
enum class FalloffName { smooth, linear, sphere, root, sharp, constant };
/** A falloff by name or as a function. */
using FalloffSpec = std::variant<FalloffName, Falloff>;
/** How a softening measures distance: along the path, or straight across the plane. */
enum class Along { path, plane };

/** Which side of a path to find: furthest left, right, up or down. */
enum class Way { left, right, top, bottom };

/** The place on a path nearest a point: its progress, the point and the distance. */
struct PathProjection {
  Progress at;
  Pt p;
  double d;
};
/** A sharp node: where, the point, how far the path turns there (degrees, positive left) and the tip's direction. */
struct Turn {
  Progress at;
  Pt p;
  double turn;
  Pt tip;
};
/** Where a path crosses a line: progress on the path, `s` along the line, the point. */
struct PathLineCrossing {
  Progress at;
  double s;
  Pt p;
};
/** Where a path crosses a circle. */
struct PathCircleCrossing {
  Progress at;
  Pt p;
};
/** Where a path crosses itself: the progress of each pass and the point. */
struct PathSelfCrossing {
  Progress a, b;
  Pt p;
};
/** A circular arc of a path, with the progress it spans. */
struct PathArc : Arc {
  Progress at0, at1;
};
/** Where a path crosses another: the point, its progress `t` on this path and `u` on the other. */
struct PathCrossing {
  Pt p;
  Progress t, u;
};
/** A node's segment and parameter: what locate gives. */
struct Located {
  int i;
  double t;
};

/** How selectAt softens: the falloff (smooth), along the path or the plane (path), and whether to cut a node in at
 *  the place first. */
struct SelectAtOptions {
  std::optional<FalloffSpec> falloff;
  std::optional<Along> along;
  bool cut = true;
};

struct QuarterOptions {
  std::optional<Heading> from;
  double tension = KAPPA;
};

struct AttachOptions;

class Path {
 public:
  explicit Path(Pt start, std::vector<Cubic> segs = {}, bool closed = false, std::optional<Heading> arrives = {},
                Pt metric = {1, 1}, std::optional<std::vector<std::optional<NodeKind>>> kinds = {});

  /** A path along segments `segs` (at least one), starting where the first starts. */
  static Path of(std::vector<Cubic> segs, bool closed = false, Pt metric = {1, 1});

  Pt start() const { return start_; }
  const std::vector<Cubic>& segs() const { return segs_; }
  bool closed() const { return closed_; }
  /** The heading the last quarter arrived along, so the next quarter can carry on smoothly. */
  std::optional<Heading> arrives() const { return arrives_; }
  /** What a unit across and a unit up are worth when measuring lengths and angles. */
  Pt metric() const { return metric_; }

  /** The same path measured with `metric`. */
  Path measured(Pt metric) const;
  /** The same path on segments `segs`, its kinds kept when they have as many nodes, read afresh otherwise. */
  Path reshaped(std::vector<Cubic> segs) const;

  /** Node `i`'s kind: the one set on it, or else read off its handles, in the metric. */
  NodeKind kind(int i) const;
  /** Every node's kind, in order. */
  std::vector<NodeKind> kinds() const;
  /** The path with the nodes `pick` takes (all by default) set to `kind`, their handles lined up to it. */
  Path withKind(NodeKind kind) const;
  Path withKind(NodeKind kind, const NodePick& pick) const;
  /** The path with node `i`'s handle on `side` moved to `to`, and its other handle following as its kind says. */
  Path handle(int i, HandleSide side, Pt to) const;

  /** The path's length, measured with the metric. */
  double length() const;
  /** The segment and its parameter at progress `p`. */
  Located locate(Progress p) const;
  /** The progress at parameter `t` of segment `i`: locate's inverse. */
  Progress progress(int i, double t) const;
  /** Where the path ends now. */
  Pt end() const;

  /** A straight line on to `to`. */
  Path line(Pt to) const;
  /** A cubic on to `to` through handles `c1` and `c2`. */
  Path curve(Pt c1, Pt c2, Pt to) const;
  /** A quarter of an ellipse on to `to`, leaving along `from` (by default the way the last quarter arrived, else
   *  vertically) and arriving along the other; `tension` is the handles' share of the radius. */
  Path quarter(Pt to, const QuarterOptions& o = {}) const;
  /** An arch on to `to` over a peak at height `peak` halfway across: two elliptical quarters. */
  Path arch(Pt to, double peak, double tension = KAPPA) const;
  /** The path running on into `next` (an open path), with a line to its start when it doesn't start here. */
  Path join(const Path& next) const;
  /** The path closed on itself, with a line back to the start when it doesn't end there. */
  Path close() const;

  /** The point at progress `p` along the path. */
  Pt at(Progress p) const;
  /** The points where the path crosses height `y`, along the path. */
  std::vector<Pt> crossY(double y) const;
  /** The points where the path crosses `x`, along the path. */
  std::vector<Pt> crossX(double x) const;
  /** The point where the path crosses height `y`; throws when it doesn't. */
  Pt atY(double y, Which which = End::first) const;
  /** The point where the path crosses `x`. */
  Pt atX(double x, Which which = End::first) const;
  /** The progress at which the path crosses height `y`. */
  Progress tAtY(double y, Which which = End::first) const;
  /** The progress at which the path crosses `x`. */
  Progress tAtX(double x, Which which = End::first) const;
  /** The straight line across the path at height `y`, from its first crossing to its last. */
  Path chordY(double y) const;
  /** The straight line across the path at `x`, from its first crossing to its last. */
  Path chordX(double x) const;

  /** The path from progress `p0` to `p1` (open). */
  Path slice(Progress p0, Progress p1) const;
  /** A closed path cut open on the side facing `angle` (degrees in the metric), `offset` of the way from its middle
   *  out to its edge that way; the longest piece behind the cut is kept. */
  Path aperture(double offset, double angle = 0) const;
  /** The same path with a node added at every `1 / n` of its length. */
  Path divide(int n) const;
  /** The unit direction the path runs at progress `p`, in the metric. */
  Pt direction(Progress p) const;
  /** Where the path crosses `other`, in order along this path. */
  std::vector<PathCrossing> crossings(const Path& other) const;
  /** The same path with nodes added at progress `ps`; places on a node already are skipped. */
  Path cut(const std::vector<Progress>& ps) const;
  /** The direction from start to end, in degrees from the horizontal. */
  double angle() const;

  /** How many nodes the path has: where its segments meet, and its ends when open. */
  int nodeCount() const;
  /** Node `i`: 0 is the start, then the end of each segment in turn. */
  Pt node(int i) const;
  /** Progress `p` along segments `segs` only, as progress along the whole path. */
  Progress within(Segs segs, Progress p) const;
  /** The progress at node `i`. */
  Progress nodeProgress(int i) const;
  /** The nodes from start to end; a closed path's start comes again at its end. */
  std::vector<Pt> nodes() const;
  /** Each node's length along the path from its start, measured with the metric. */
  std::vector<double> distances() const;

  /** The path run backwards. */
  Path reverse() const;
  /** A closed path started at its node `r`. */
  Path startAt(int r) const;
  /** Some of the path's nodes (see Selection); all of them by default. Defined in selection.cpp. */
  Selection select() const;
  Selection select(const NodePick& pick) const;
  Selection select(const Pick& pick) const;
  /** The path with every point moved by `f`, which should be affine so the curves move exactly. */
  Path map(const std::function<Pt(Pt)>& f) const;

  /** The path attached to `on` at progress `at` (see attach.hpp). Defined in attach.cpp. */
  Path attach(const Path& on, Progress at) const;
  Path attach(const Path& on, Progress at, const AttachOptions& o) const;

  // Curve tools on the whole path (analysis.hpp says what each is for). Defined in analysis.cpp.
  PathProjection project(Pt p) const;
  std::vector<Progress> extrema() const;
  std::vector<Progress> inflections() const;
  Box bbox() const;
  Progress side(Way way) const;
  std::vector<Turn> turns(double minDeg = 20) const;
  std::function<double(Progress)> nearness(const std::vector<Progress>& ats, double reach,
                                           const FalloffSpec& falloff = FalloffName::smooth) const;
  double curvature(Progress at) const;
  Pt normal(Progress at) const;
  Osculating osculating(Progress at) const;
  Path mold(Progress at, Pt to) const;
  Path offset(const Reach& d) const;
  std::vector<Contour> outline(double d1) const;
  std::vector<Contour> outline(double d1, double d2) const;
  std::vector<PathLineCrossing> lineCrossings(Pt a, Pt b, bool segment = true) const;
  std::vector<PathCircleCrossing> circleCrossings(Pt o, double r) const;
  std::vector<PathSelfCrossing> selfCrossings() const;
  std::vector<PathArc> arcs(double threshold = 0.5) const;
  Selection selectAt(Progress at, double reach, const SelectAtOptions& o = {}) const;

  /** The path bent by any map of the plane (see warp.hpp), each segment cut first as `o` says. */
  Path warp(const Warp& f, const WarpOptions& o = {}) const;

  /** The kinds set on its nodes, or none (for the files that rebuild paths: selection, attach). */
  const std::optional<std::vector<std::optional<NodeKind>>>& setKinds() const { return kinds_; }

 private:
  Pt start_;
  std::vector<Cubic> segs_;
  bool closed_;
  std::optional<Heading> arrives_;
  Pt metric_;
  std::optional<std::vector<std::optional<NodeKind>>> kinds_;
  mutable std::optional<std::vector<double>> lens_;

  std::vector<std::optional<NodeKind>> explicitKinds() const;
  Cubic iso(int i) const;
  const std::vector<double>& lens() const;
  Path then(const Cubic& c, std::optional<Heading> arrives = {}) const;
  Path chord(Pt a, Pt b) const;
  struct Cross {
    Pt p;
    Progress t;
  };
  std::vector<Cross> cross(int coord, double v) const;
};

/** A pick that reads a path first: by its own numbering (counted from its end when negative) or its own progress,
 *  and that may cut nodes into it (a section's ends). See selection.hpp for nodeAt, cubicAt and section. */
struct PickOn {
  Path path;
  NodePick pick;
};
struct Pick {
  std::function<PickOn(const Path&)> on;
};

/** Where an attachment's second point goes: progress `at` along `on` (the first path by default), its own point at
 *  `from` (its end by default), within segments `segs`. */
struct AttachEnd {
  std::optional<Path> on;
  Progress at = 0;
  std::optional<Progress> from;
  std::optional<Segs> segs;
};
/** Path::attach's options: which point of the path goes on (`from`, its start by default), within which segments of
 *  `on`, and the second point, if any. */
struct AttachOptions {
  std::optional<Progress> from;
  std::optional<Segs> segs;
  std::optional<AttachEnd> end;
};

/** Segments `segs` with node `i`'s handles lined up to `kind`, led by the handle on side `lead` (or both), in the
 *  metric. A corner, an open end, and a node with a handle on it are left. */
std::vector<Cubic> aligned(const std::vector<Cubic>& segs, bool closed, int i, NodeKind kind, HandleLead lead, Pt metric);

/** A path starting at `p`. */
Path from(Pt p);
/** A straight line from `a` to `b`. */
Path line(Pt a, Pt b);
/** Straight lines through the points in turn. */
Path poly(Pt p, const std::vector<Pt>& ps);

}  // namespace curves
