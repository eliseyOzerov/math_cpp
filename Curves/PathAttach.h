// Attaching a path to what is drawn already, by pairs: each pair says that the path's point at
// some progress lies on a target. A target is a place on another path, a fixed point, or where two
// paths cross. Ported from graphics_cpp's attach.cpp (Curves2D), `attach`, `target` and `place`.
//
// How many pairs decide how the path moves, always as little as they need:
//   1  shifted onto the target;
//   2  turned and stretched along the line between the targets, its depth across that line kept;
//   3  mapped by the one affine map that takes the three points onto their targets;
//   4+ mapped by the affine map nearest to all of them (least squares), then bent node by node: a
//      node is cut in at each pair's point and moved, with its handles, the rest of the way onto
//      its target, and the nodes between two pairs move by a blend of theirs by progress.
// Everything is affine up to three pairs, so the curves move exactly. Lengths and angles are taken
// in a metric (what one unit across and one unit up are worth, as in `MeasuredPath2D`); the same
// metric measures every path named in a call.
//
// An alignment turns the path about its first pair's point (its start when there is none) so it
// runs along the tangent of another path at a progress, or along its normal, turned a further
// angle. It goes with at most one pair, since a second already decides the turn.
//
// Declared here, defined in PathAttach.cpp (attach, target, place) and PathAttach+Affine.cpp (the
// affine maps).
#pragma once

#include <optional>
#include <variant>
#include <vector>

#include "../Linear/AffineTransform2.h"
#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"
#include "MeasuredPath2D.h"
#include "Path2D.h"

namespace editor {

/// A run of pieces of a path, first to last, counted from 0 (negative counts from the last piece).
/// A progress given "within" it is a share of just that run.
///
/// Use when: naming a place on one stretch of a long path, such as "halfway along the third curve"
/// Tags: piece range, segment range, stretch of path, within segments
/// Node: planned (attachPath)
struct PieceRange {
    int first = 0;
    int last = 0;

    bool operator==(const PieceRange&) const = default;

    /// Just piece `index`.
    static PieceRange single(int index) { return PieceRange{index, index}; }
    /// Pieces `first` to `last`.
    static PieceRange between(int first, int last) { return PieceRange{first, last}; }
};

/// Where a path is attached: the point at progress `from` along it (0, its start, by default) goes
/// to progress `progress` along `on`, or along its pieces `pieces` only.
///
/// Use when: pinning the start of a stroke onto the middle of another
/// Tags: mount, attach point, pin to path
/// Node: planned (attachPath)
struct AttachMount {
    Path2D on;
    float progress = 0;
    std::optional<float> from;
    std::optional<PieceRange> pieces;
};

/// A place on another path to land on.
///
/// Use when: a target that follows a path as it is redrawn
/// Tags: target on path, anchor on path
/// Node: planned (placePath)
struct TargetOnPath {
    Path2D on;
    float progress = 0;
    std::optional<PieceRange> pieces;
};

/// A fixed point to land on.
///
/// Use when: pinning a path to a known point
/// Tags: target point, fixed anchor
/// Node: planned (placePath)
struct TargetPoint {
    Vector2 point;
};

/// Where `crossing` crosses `with`: the first crossing along `crossing`, or the one `which` names.
///
/// Use when: landing on an anchor found on the drawing, such as where two strokes meet
/// Tags: target crossing, intersection anchor
/// Node: planned (placePath)
struct TargetCrossing {
    Path2D crossing;
    Path2D with;
    std::optional<CrossingChoice> which;
};

/// Where a point of the attached path goes.
using AttachTarget = std::variant<TargetOnPath, TargetPoint, TargetCrossing>;

/// One pair: the attached path's point at progress `from` goes to the target. By default the pairs
/// take the path's start, its end, and places evenly between for more.
///
/// Use when: placing a path by two or more of its points
/// Tags: attach pair, correspondence, control point pair
/// Node: planned (placePath)
struct AttachPair {
    AttachTarget target;
    std::optional<float> from;
};

/// Whether an alignment follows the tangent of the other path or its normal.
enum class AlignTo { tangent, normal };

/// Turns a path to run along `on`'s tangent at `progress` (or its normal), plus `extraAngle`, as
/// measured at the path's progress `from` (by default its first pair's, or 0).
///
/// Use when: standing a serif square to a stroke; laying a leaf along a stem
/// Tags: align to tangent, align to normal, orient on path
/// Node: planned (placePath)
struct AttachAlign {
    Path2D on;
    float progress = 0;
    std::optional<PieceRange> pieces;
    AlignTo to = AlignTo::tangent;
    Angle extraAngle;
    std::optional<float> from;
};

/// Placing a path by pairs of points (a namespace of statics; there is nothing to construct).
struct PathAttachment {
    static Path2D attached(const Path2D& path, const AttachMount& first, const std::optional<AttachMount>& second = std::nullopt,
                           Vector2 metric = Vector2(1, 1));
    static Vector2 targetPoint(const AttachTarget& target, Vector2 metric = Vector2(1, 1));
    static Path2D placed(const Path2D& path, const std::vector<AttachPair>& pairs,
                         const std::optional<AttachAlign>& align = std::nullopt, Vector2 metric = Vector2(1, 1));

    // MARK: Affine maps (PathAttach+Affine.cpp)

    static std::optional<AffineTransform2> affineThroughTriangle(
        const std::vector<Vector2>& sources, const std::vector<Vector2>& destinations);
    static std::optional<AffineTransform2> affineLeastSquares(
        const std::vector<Vector2>& sources, const std::vector<Vector2>& destinations);
};

} // namespace editor
