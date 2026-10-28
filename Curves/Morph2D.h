// Morphing one path into another. The two are first made alike, node for node: each node of the
// path with fewer is matched to the closest node of the other, in order along both; where the path
// being morphed has too few nodes, the missing ones are added on it without changing its shape,
// and where it has too many, each extra one is sent to the closer of its matched neighbours, so it
// merges there. Then every point goes the same share of the way.
//
// Decisions: matching is in order along both paths and pins both ends (a dynamic program), so the
// morph never crosses itself by sending nodes backwards. The target is tried both ways round and,
// when closed, from every start, and the cheapest fit wins; costs within `tolerance` count as equal
// and the first found wins, so the same paths always match the same way. Missing nodes are added at
// the share of the segment's length that they lie at along the target, so they travel straight to
// their partners. Extra nodes merge rather than vanish, so at amount 1 the morph is the target, and
// between it every segment exists. Known issue (inherited): a closed morph can show a small notch
// at its start halfway through.
//
// Ported from graphics_cpp's morph.hpp/morph.cpp (fountain-ts/src/curves/morph.ts).
// Differences: float; a path has no metric of its own, so `metric` is an argument; lines are
// treated as cubics with handles at their thirds, and a closed path counts its closing line as a
// piece; results are paths of cubics.
#pragma once

#include <vector>

#include "../Linear/Vector2.h"
#include "Path2D.h"

namespace editor {

/// Two paths made alike for morphing: as many pieces each, node `i` of one going to node `i` of the
/// other. `matched` holds, for each node of `from`, whether it was matched to a node of the target
/// (rather than added or merging).
///
/// Use when: inspecting or animating a morph step by step
/// Tags: morph correspondence, matched nodes, alike paths
/// Node: planned (morphPath)
struct MorphCorrespondence {
    Path2D from;
    Path2D to;
    std::vector<bool> matched;
};

/// Morphing between paths with different numbers of nodes.
///
/// Use when: animating one outline into another; blending two letter shapes
/// Tags: morph, shape blending, path interpolation, tween
/// Node: planned (morphPath)
struct Morph2D {
    /// `first` and `second` made alike, keeping `first`'s start and direction: `second` is turned
    /// round and, when closed, started at the node that fits best.
    ///
    /// How: nodes are matched in order with both ends pinned by a dynamic program over the sum of
    /// node distances in `metric` units; the path with more nodes is the "long" one. If `first` has
    /// too few nodes, extra ones are cut into its pieces at the share of length their partners have
    /// on `second`; if too many, each extra node moves onto the closer matched neighbour.
    ///
    /// Conventions: both paths need a piece, and both closed or both open, or a `MathProblem` is
    /// thrown. A closed path's closing line is a piece. `tolerance` is in `metric` units.
    ///
    /// Use when: preparing two outlines for `between`
    /// Tags: make paths alike, node matching, correspondence, dynamic programming
    /// Node: planned (morphPath)
    static MorphCorrespondence alike(
        const Path2D& first, const Path2D& second, float tolerance = 1e-4f, Vector2 metric = Vector2(1, 1));

    /// `first` morphed `amount` of the way into `second` (0 is `first`, 1 is `second`). Pieces that
    /// have shrunk to within `tolerance` (in `metric` units) are dropped, so nodes that met merge.
    ///
    /// Use when: one-call morphing
    /// Tags: morph, tween, blend shapes
    /// Node: planned (morphPath)
    static Path2D morph(
        const Path2D& first, const Path2D& second, float amount, float tolerance = 1e-4f,
        Vector2 metric = Vector2(1, 1));

    /// The path `amount` of the way from `from` to `to`, two paths already alike (same piece count,
    /// otherwise a `MathProblem` is thrown); pieces shrunk to within `tolerance` are dropped.
    ///
    /// How: every control point is the linear blend of its two partners; a dropped piece's end
    /// is within `tolerance` of its start, and the next kept piece starts where the last kept one ended.
    ///
    /// Use when: animating a morph prepared once with `alike`
    /// Tags: interpolate paths, in-between, tween frame
    /// Node: planned (morphPath)
    static Path2D between(
        const Path2D& from, const Path2D& to, float amount, float tolerance = 1e-4f, Vector2 metric = Vector2(1, 1));
};

} // namespace editor
