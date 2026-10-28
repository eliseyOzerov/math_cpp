// Paths as their nodes, the way a drawing program holds them: each node an anchor with its two handles as offsets
// from it and its kind, so moving an anchor carries its handles, and a handle left on its anchor (an offset of zero)
// is a node whose curve leaves or arrives straight from the next control point. What a hand editor reads and writes.
// Handle locks hold any handles together (those of one node, of two nodes, or across lists of nodes): a drag turns and
// scales the others by as much as the dragged one turned and stretched, so a lock keeps no angles or lengths of its own.
//
// Ported from fountain-ts/src/curves/nodes.ts (its header has the reasons at length).
//
// Decisions (C++ mapping):
// - Node's optional fields (`in`, `out`, `kind`) are std::optional; the TS's `undefined` is nullopt.
// - The TS generics over `N extends Node` (withLockedHandle, lockedKinds, kindLocks) take and give plain Node lists.
// - `lockOf` answers a pointer into `locks` (null for none), so `lockedKind` can compare locks by identity as the TS does.
// - HandleLock's `angle`/`length` (TS: `true` or absent) are bools; `flip` likewise. `locked`'s `how` is LockHow.
// - Callbacks (`pick`, `to`) are std::function; shiftNodes' default pick is an empty function (all nodes).
#pragma once

#include "curves/geometry.hpp"
#include "curves/path.hpp"

#include <functional>
#include <optional>
#include <vector>

namespace curves {

/** A node: its anchor `p`, its handles as offsets from it (left out: on the anchor) and its kind (left out: read off
 *  its handles, as a Path does). An open path's first node has no `in`, its last no `out`. */
struct Node {
  Pt p;
  std::optional<Pt> in, out;
  std::optional<NodeKind> kind;
};

/** The nodes of `path`, with their kinds as the path reads them. */
std::vector<Node> nodesOf(const Path& path);

/** The cubic from node `a` to node `b`, exactly as drawn (handles on their anchors left there). */
Cubic segmentOf(const Node& a, const Node& b);

/** Every segment of `nodes`, exactly as drawn: the closing one too when `closed`. */
std::vector<Cubic> segmentsOf(const std::vector<Node>& nodes, bool closed);

/** The path through `nodes` (at least two, or one closed on itself), measured by `metric`, with their kinds. Throws
 *  when there are no segments. */
Path pathOf(const std::vector<Node>& nodes, bool closed, Pt metric = {1, 1});

/** The point of the path through `nodes` on segment `seg` (from node `seg` to the next) at its parameter `t`, as
 *  `pathOf` draws it (a handle on its anchor nudged). */
Pt pointOn(const std::vector<Node>& nodes, bool closed, int seg, double t);

/** The place on the path through `nodes` nearest a point: its segment, its parameter there, the point and its distance. */
struct NodesNearest {
  int seg;
  double t;
  Pt p;
  double d;
};
NodesNearest nearestOn(const std::vector<Node>& nodes, bool closed, Pt p);

/** Where the path through `nodes` crosses the line where coordinate `j` (0 x, 1 y) is `v`: each segment, parameter
 *  and point, as `pathOf` draws it. */
struct NodesCrossing {
  int seg;
  double t;
  Pt p;
};
std::vector<NodesCrossing> crossingsOn(const std::vector<Node>& nodes, bool closed, int j, double v);

/** `nodes` with segment `seg` bent so its point at `t` is at `to` (molded): its two nodes stay and take new handles on
 *  that segment, their other handles following their kinds. No node is added. */
std::vector<Node> moldNodes(const std::vector<Node>& nodes, bool closed, int seg, double t, Pt to);

/** Node `n`'s kind: its own, or read off its handles (lined up: smooth, and as long: symmetric). */
NodeKind nodeKind(const Node& n);

/** Node `n` with its handle on `side` at offset `to`, its other handle following its kind: a smooth node's turned
 *  opposite, its length kept; a symmetric one's mirrored; a corner's left. A handle that doesn't exist isn't added,
 *  and one on its anchor stays there under a smooth node. */
Node withHandle(const Node& n, HandleSide side, Pt to);

/** Node `n` set to `kind`, its handles lined up to it. */
Node withNodeKind(const Node& n, NodeKind kind);

/** `nodes` with a node put on segment `seg` at its parameter `t`, the curve kept. The new node is smooth. */
std::vector<Node> insertNode(const std::vector<Node>& nodes, int seg, double t);

/** The nodes moved by `d`, those `pick` takes (all when `pick` is empty), their handles carried. */
std::vector<Node> shiftNodes(const std::vector<Node>& nodes, Pt d, const std::function<bool(int)>& pick = {});

/** `nodes` refitted to the fewest cubics within `tol` of the curve, its corners kept where they are. */
std::vector<Node> simplified(const std::vector<Node>& nodes, bool closed, double tol);

// ---- Handle locks ----

/** A handle among lists of nodes: list `s`, node `i`, its side. */
struct HandleRef {
  int s, i;
  HandleSide side;
};
/** A handle of a lock; `flip`ped ones turn the other way, as a mirror image turns. */
struct LockHandle : HandleRef {
  bool flip = false;
  LockHandle() = default;
  LockHandle(int s, int i, HandleSide side, bool flip = false) : HandleRef{s, i, side}, flip(flip) {}
  LockHandle(const HandleRef& r) : HandleRef(r) {}
};
/** Handles held together: dragging one turns the others by as much (`angle`) and scales their lengths alike (`length`). */
struct HandleLock {
  std::vector<LockHandle> handles;
  bool angle = false, length = false;
};
struct LockHow {
  bool angle = false, length = false;
};

/** The lock handle `r` is in, or null. */
const HandleLock* lockOf(const std::vector<HandleLock>& locks, const HandleRef& r);

/** `lists` with handle `ref` at offset `to`, the handles locked with it following from where they are in `lists`. */
std::vector<std::vector<Node>> withLockedHandle(const std::vector<std::vector<Node>>& lists,
                                                const std::vector<HandleLock>& locks, const HandleRef& ref, Pt to);

/** `locks` with `refs` locked together by `how`, the locks any of them was in merged into it. */
std::vector<HandleLock> locked(const std::vector<HandleLock>& locks, const std::vector<HandleRef>& refs, LockHow how);

/** `locks` without handles `refs`; a lock left with one handle goes. */
std::vector<HandleLock> unlocked(const std::vector<HandleLock>& locks, const std::vector<HandleRef>& refs);

/** `locks` after list `s`'s nodes changed: node `i` is now node `to(i)`, or gone (nullopt); a lock left with one
 *  handle goes. */
std::vector<HandleLock> remapLocks(const std::vector<HandleLock>& locks, int s,
                                   const std::function<std::optional<int>(int)>& to);

/** Node `i` of list `s`'s kind as its locks make it. */
NodeKind lockedKind(const std::vector<HandleLock>& locks, const Node& n, int s, int i);

/** `lists` with each node's kind read off `locks`. */
std::vector<std::vector<Node>> lockedKinds(const std::vector<std::vector<Node>>& lists, const std::vector<HandleLock>& locks);

/** The locks `lists`' node kinds stand for, and the lists without kinds. */
struct KindLocks {
  std::vector<std::vector<Node>> lists;
  std::vector<HandleLock> locks;
};
KindLocks kindLocks(const std::vector<std::vector<Node>>& lists);

}  // namespace curves
