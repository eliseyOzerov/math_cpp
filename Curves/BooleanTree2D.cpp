// The boolean operand tree (graphics_cpp bool.cpp).
#include <cmath>

#include "../Linear/MathProblem.h"
#include "BooleanTree2D.h"
#include "Shape2D+Pieces.h"

namespace editor {

namespace {

bool anyInside(const std::vector<bool>& inside, size_t from) {
    for (size_t index = from; index < inside.size(); index += 1) {
        if (inside[index]) return true;
    }
    return false;
}

/// The rule of `operation` over any number of operands (entry 0 is the first). `cut` is the rule of
/// the parts inside the others' outlines; the parts outside are `subtract`.
BooleanRule ruleOf(BooleanOperation operation) {
    switch (operation) {
    case BooleanOperation::unite: return [](const std::vector<bool>& inside) { return anyInside(inside, 0); };
    case BooleanOperation::intersect:
        return [](const std::vector<bool>& inside) {
            for (bool one : inside) {
                if (!one) return false;
            }
            return true;
        };
    case BooleanOperation::subtract: return [](const std::vector<bool>& inside) { return inside.at(0) && !anyInside(inside, 1); };
    case BooleanOperation::exclude:
        return [](const std::vector<bool>& inside) {
            int count = 0;
            for (bool one : inside) count += one ? 1 : 0;
            return count % 2 == 1;
        };
    case BooleanOperation::cut: return [](const std::vector<bool>& inside) { return inside.at(0) && anyInside(inside, 1); };
    }
    return [](const std::vector<bool>& inside) { return inside.at(0); };
}

/// The region an operand stands for: a boolean's own outline, a path as one closed outline, or the
/// shape as given.
Shape2D regionOf(const BooleanOperand& operand) {
    if (const std::shared_ptr<const BooleanTree2D>* nested = std::get_if<std::shared_ptr<const BooleanTree2D>>(&operand)) {
        Shape2D region;
        region.outlines = (*nested)->contours();
        region.fillRule = FillRule::nonzero;
        return region;
    }
    if (const Path2D* path = std::get_if<Path2D>(&operand)) {
        Path2D closedPath = *path;
        closedPath.close();
        return Shape2D{{closedPath}, FillRule::nonzero};
    }
    return std::get<Shape2D>(operand);
}

/// The corner between the piece `arriving` and the piece `leaving`.
///
/// How: the filled side is on the left of the way the outline runs, so outward is to the right; at
/// a corner the right normals of the two pieces are added to halve it. Where they cancel (the
/// outline doubles back) outward is the way back along the arriving piece.
BooleanCorner cornerBetween(int contourIndex, int nodeIndex, const OutlinePiece& arriving, const OutlinePiece& leaving, std::array<int, 2> between) {
    constexpr float cancelled = 1e-5f;
    Vector2 incoming = arriving.endDirection().normalized();
    Vector2 outgoing = leaving.startDirection().normalized();
    Vector2 sum = Vector2(incoming.y, -incoming.x) + Vector2(outgoing.y, -outgoing.x);
    BooleanCorner corner;
    corner.contourIndex = contourIndex;
    corner.nodeIndex = nodeIndex;
    corner.point = leaving.start();
    corner.between = between;
    corner.incomingDirection = incoming;
    corner.outgoingDirection = outgoing;
    corner.outward = sum.length() > cancelled ? sum.normalized() : incoming * -1.0f;
    corner.isConvex = incoming.cross(outgoing) > 0;
    return corner;
}

} // namespace

BooleanTree2D::BooleanTree2D(BooleanOperation operation, std::vector<BooleanOperand> operands, std::optional<float> tolerance)
    : operationValue(operation), operandList(std::move(operands)), toleranceValue(tolerance) {
    if (operandList.empty()) throw MathProblem("A boolean needs at least one operand.");
    if (operation == BooleanOperation::cut && operandList.size() < 2) throw MathProblem("A cut needs at least two operands.");
}

/// The same operation over other operands, as when one of them has been redrawn.
///
/// Use when: redoing a boolean after an operand changed
/// Tags: update boolean, redraw boolean group, replace operands
/// Node: planned (booleanShapes)
BooleanTree2D BooleanTree2D::with(std::vector<BooleanOperand> operands) const {
    return BooleanTree2D(operationValue, std::move(operands), toleranceValue);
}

/// The traced outline, worked out on first use: nothing where a loop does not close.
///
/// How: the operands' regions are the groups of one `traceBoolean` with the operation's rule; `cut`
/// is two traces, the first operand's parts inside the others and then those outside, joined side
/// by side.
const std::optional<TracedBoolean>& BooleanTree2D::traced() const {
    if (tracedValue) return *tracedValue;
    std::vector<Shape2D> groups;
    for (const BooleanOperand& operand : operandList) groups.push_back(regionOf(operand));
    if (operationValue != BooleanOperation::cut) {
        return tracedValue.emplace(traceBoolean(groups, ruleOf(operationValue), toleranceValue));
    }
    std::optional<TracedBoolean> inside = traceBoolean(groups, ruleOf(BooleanOperation::cut), toleranceValue);
    std::optional<TracedBoolean> outside = traceBoolean(groups, ruleOf(BooleanOperation::subtract), toleranceValue);
    if (!inside || !outside) return tracedValue.emplace(std::nullopt);
    TracedBoolean both = *inside;
    both.shape.outlines.insert(both.shape.outlines.end(), outside->shape.outlines.begin(), outside->shape.outlines.end());
    both.sourceGroups.insert(both.sourceGroups.end(), outside->sourceGroups.begin(), outside->sourceGroups.end());
    return tracedValue.emplace(both);
}

/// Whether the outline could be traced; where a loop does not close, `contours` gives the operands
/// unchanged.
///
/// Use when: warning that a boolean failed and its operands are shown as they are
/// Tags: boolean succeeded, trace ok, boolean failed
/// Node: planned (booleanShapes)
bool BooleanTree2D::ok() const { return traced().has_value(); }

/// The outline: outer contours counterclockwise, holes clockwise (y up); the operands' own outlines
/// together where the boolean could not be traced.
///
/// Use when: filling or stroking the result of a boolean group
/// Tags: boolean outline, result contours, compound path
/// Node: planned (booleanShapes)
std::vector<Path2D> BooleanTree2D::contours() const {
    if (const std::optional<TracedBoolean>& result = traced()) return result->shape.outlines;
    std::vector<Path2D> outlines;
    for (const BooleanOperand& operand : operandList) {
        for (const Path2D& outline : regionOf(operand).outlines) outlines.push_back(outline);
    }
    return outlines;
}

/// For each piece of each contour, the operand it was cut from (−1 where the outline could not be
/// traced).
///
/// Use when: telling which operand a piece of the outline belongs to
/// Tags: boolean provenance, piece origin, source operand
/// Node: planned (booleanShapes)
std::vector<std::vector<int>> BooleanTree2D::sources() const {
    if (const std::optional<TracedBoolean>& result = traced()) return result->sourceGroups;
    std::vector<std::vector<int>> unknown;
    for (const Path2D& outline : contours()) unknown.push_back(std::vector<int>(outlinePieces(outline).size(), -1));
    return unknown;
}

/// The corners where the outline turns from a piece of one operand to a piece of another.
///
/// How: for each contour and each node, the operands of the piece arriving and of the piece
/// leaving. Corners within one operand are left out unless `includeSameOperand`. With `first` and
/// `second` given only the corners where those two operands meet (in either order); with `first`
/// alone the corners where it meets any other; with neither, where any two different operands meet.
///
/// Source: graphics_cpp curves/bool.cpp `Bool::corners`.
///
/// Use when: rounding the join between a stem and a bowl but not the corners of the stem itself
/// Tags: boolean corners, junction corners, seam corners, where operands meet
/// Node: planned (booleanShapes)
std::vector<BooleanCorner> BooleanTree2D::corners(std::optional<int> first, std::optional<int> second, bool includeSameOperand) const {
    std::vector<BooleanCorner> found;
    std::vector<Path2D> outlines = contours();
    std::vector<std::vector<int>> origins = sources();
    for (size_t contourIndex = 0; contourIndex < outlines.size(); contourIndex += 1) {
        std::vector<OutlinePiece> pieces = outlinePieces(outlines[contourIndex]);
        const std::vector<int>& pieceSources = origins[contourIndex];
        int count = static_cast<int>(pieces.size());
        if (count == 0 || pieceSources.size() != pieces.size()) continue;
        for (int node = 0; node < count; node += 1) {
            int previous = (node + count - 1) % count;
            std::array<int, 2> between{pieceSources[static_cast<size_t>(previous)], pieceSources[static_cast<size_t>(node)]};
            if (!includeSameOperand && between[0] == between[1]) continue;
            if (first) {
                bool meets = !second ? (between[0] == *first || between[1] == *first)
                                     : ((between[0] == *first && between[1] == *second) || (between[0] == *second && between[1] == *first));
                if (!meets) continue;
            }
            found.push_back(cornerBetween(
                static_cast<int>(contourIndex), node, pieces[static_cast<size_t>(previous)], pieces[static_cast<size_t>(node)], between));
        }
    }
    return found;
}

/// The unit direction out of the outline at a node of a contour: the direction halving the corner
/// there, pointing away from the filled side.
///
/// Conventions: throws a `MathProblem` for a contour or node that does not exist.
///
/// Use when: adding a shape or making a cut at a corner, in the direction that points out of the
///   outline
/// Tags: outward direction, corner bisector, outside normal at node
/// Node: planned (booleanShapes)
Vector2 BooleanTree2D::outward(int contourIndex, int nodeIndex) const {
    std::vector<Path2D> outlines = contours();
    if (contourIndex < 0 || contourIndex >= static_cast<int>(outlines.size())) throw MathProblem("The boolean has no such outline.");
    std::vector<OutlinePiece> pieces = outlinePieces(outlines[static_cast<size_t>(contourIndex)]);
    int count = static_cast<int>(pieces.size());
    if (nodeIndex < 0 || nodeIndex >= count) throw MathProblem("The outline has no such node.");
    int previous = (nodeIndex + count - 1) % count;
    return cornerBetween(contourIndex, nodeIndex, pieces[static_cast<size_t>(previous)], pieces[static_cast<size_t>(nodeIndex)], {-1, -1}).outward;
}

} // namespace editor
