// The five named booleans on two shapes, and a shape's own overlaps removed.
#include "Boolean2D.h"

namespace editor {

namespace {

/// The rule of one of the five operations over two operands (entry 0 is A, entry 1 is B). `cut` is
/// handled apart: it is two rules.
BooleanRule ruleOf(BooleanOperation operation) {
    switch (operation) {
    case BooleanOperation::unite: return [](const std::vector<bool>& inside) { return inside[0] || inside[1]; };
    case BooleanOperation::intersect: return [](const std::vector<bool>& inside) { return inside[0] && inside[1]; };
    case BooleanOperation::subtract: return [](const std::vector<bool>& inside) { return inside[0] && !inside[1]; };
    case BooleanOperation::exclude: return [](const std::vector<bool>& inside) { return inside[0] != inside[1]; };
    case BooleanOperation::cut: break;
    }
    return [](const std::vector<bool>& inside) { return inside[0]; };
}

} // namespace

/// The region produced by `operation` on `first` and `second`, or nothing where a loop of the
/// result does not close (see `traceBoolean`).
///
/// How: one of five rules over "inside A" and "inside B": unite a∨b, intersect a∧b, subtract a∧¬b,
/// exclude a⊕b. `cut` is A split along B's outline: the pieces of A inside B and the pieces outside
/// it, side by side, so the shared edge is in both.
///
///     A ┌──────┐          unite       one outline round both
///       │  ┌───┼──┐ B     intersect   the overlap
///       └──┼───┘  │       subtract    A with the overlap bitten out
///          └──────┘      exclude     two L-shaped pieces; cut: the overlap and A's L
///
/// Conventions: y-up plane; outer outlines of the result counterclockwise, holes clockwise,
/// nonzero rule. Each operand is filled by its own fill rule. Edges that two operands share (two
/// squares that touch along a side) are handled: touching squares unite into one outline and
/// intersect into nothing. `tolerance` as in `traceBoolean`.
///
/// Use when: cutting a window out of a wall profile; joining two outlines into one; keeping only
///   what two shapes share
/// Tags: boolean, union, intersection, difference, xor, subtract shapes, clip, pathfinder
/// Node: planned (booleanShapes)
std::optional<Shape2D> tryBooleanShapes(
    const Shape2D& first, const Shape2D& second, BooleanOperation operation, std::optional<float> tolerance) {
    std::vector<Shape2D> groups{first, second};
    if (operation != BooleanOperation::cut) return tryBooleanByRule(groups, ruleOf(operation), tolerance);
    std::optional<Shape2D> outside = tryBooleanByRule(groups, ruleOf(BooleanOperation::subtract), tolerance);
    std::optional<Shape2D> inside = tryBooleanByRule(groups, ruleOf(BooleanOperation::intersect), tolerance);
    if (!outside || !inside) return std::nullopt;
    Shape2D pieces = *outside;
    pieces.outlines.insert(pieces.outlines.end(), inside->outlines.begin(), inside->outlines.end());
    return pieces;
}

/// `tryBooleanShapes`, or both shapes' outlines together unchanged where a loop does not close.
///
/// Use when: a boolean in a pipeline that should degrade to "no change" rather than stop
/// Tags: boolean, union, difference, intersection, fallback
/// Node: planned (booleanShapes)
Shape2D booleanShapes(
    const Shape2D& first, const Shape2D& second, BooleanOperation operation, std::optional<float> tolerance) {
    if (std::optional<Shape2D> result = tryBooleanShapes(first, second, operation, tolerance)) return *result;
    Shape2D unchanged = first;
    unchanged.fillRule = FillRule::nonzero;
    unchanged.outlines.insert(unchanged.outlines.end(), second.outlines.begin(), second.outlines.end());
    return unchanged;
}

/// The shape with its overlaps removed: one outline where its outlines fill under their fill rule,
/// the form that fills the same under either rule. Two overlapping squares in one shape become
/// the one outline round both; a hole drawn the same way round as its outline is filled.
///
/// How: a boolean over one group with the rule "inside the group"; where it fails, the shape unchanged.
///
/// Use when: overlapping strokes of one glyph or logo that must become one outline; removing the
///   self-overlap a stroked or offset path leaves
/// Tags: remove overlaps, simplify path, union self, flatten winding, merge shapes
/// Node: planned (booleanShapes)
Shape2D united(const Shape2D& shape, std::optional<float> tolerance) {
    return booleanByRule({shape}, [](const std::vector<bool>& inside) { return inside[0]; }, tolerance);
}

} // namespace editor
