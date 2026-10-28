// Morphing one path into another. The two are first made alike, node for node: each node of the path with fewer is
// matched to the closest node of the other, in order along both; where the path being morphed has too few nodes, the
// missing ones are added on it without changing its shape, and where it has too many, each extra one is sent to the
// closer of its matched neighbours, so it merges there. Then every point goes the same share of the way.
//
// Decisions: matching is in order along both paths and pins both ends (a dynamic program), so the morph never
// crosses itself by sending nodes backwards. `b` is tried both ways round and, when closed, from every start, and the
// cheapest fit wins; costs within `eps` count as equal and the first found wins, so the same paths always match the
// same way. Missing nodes are added at the share of the segment's length that they lie at along the target, so they
// travel straight to their partners. Extra nodes merge rather than vanish, so at k = 1 the morph is the target, and
// between it every segment exists. Known issue: a closed morph can show a small notch at its start halfway through
// (seen morphing O into a squarer O at k = 0.5); not yet looked into.
//
// Ported from fountain-ts/src/curves/morph.ts.
//
// Decisions of the port:
// - The `{ eps }` option objects are a plain `eps` argument.
#pragma once

#include "curves/geometry.hpp"
#include "curves/path.hpp"

#include <vector>

namespace curves {

/** Two paths made alike for morphing: as many segments each, node `i` of one going to node `i` of the other.
 *  `matched` holds, for each node of `from`, whether it was matched to a node of the target (rather than added or
 *  merging). */
struct Alike {
  Path from, to;
  std::vector<bool> matched;
};

/** `a` and `b` made alike, keeping `a`'s start and direction: `b` is turned round and, when closed, started at the
 *  node that fits best. Throws when one is closed and the other not, or either has no segment. */
Alike alike(const Path& a, const Path& b, double eps = 1e-6);

/** `a` morphed `k` of the way into `b` (0 is `a`, 1 is `b`). Segments that have shrunk to within `eps` (in units of
 *  `a`'s metric) are dropped, so nodes that met merge. */
Path morph(const Path& a, const Path& b, double k, double eps = 1e-6);

/** The path `k` of the way from `from` to `to`, two paths already alike; segments shrunk to within `eps` are
 *  dropped. */
Path between(const Path& from, const Path& to, double k, double eps = 1e-6);

}  // namespace curves
