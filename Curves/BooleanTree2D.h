// The boolean object: an operation over outlines that keeps its operands, as a boolean group does
// in a drawing app. Its outline is worked out from them when asked for (Boolean2D.h), so an
// operand can be changed and the boolean made again, and an operand can be a boolean itself. Each
// piece of the outline knows the operand it was cut from, which is what makes the places where
// operands meet selectable: a corner between a stem and a bowl is the corner whose pieces come
// from the stem's operand and the bowl's, wherever the curves happen to cross. Ported from
// graphics_cpp's `Bool` (graphics_cpp bool.hpp/bool.cpp).
//
// Operations, over the operands in order: unite (inside any), intersect (inside all), subtract
// (inside the first and none of the rest), exclude (inside an odd number) and cut (the first
// divided along the others' outlines: its parts inside them and outside, as separate outlines).
// Each reduces to a rule over "inside" flags, so `traceBoolean` serves them all.
//
// The outline is traced once, on first use, and kept in the object (not thread-safe while untraced).
#pragma once

#include <array>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

#include "Boolean2D.h"

namespace editor {

class BooleanTree2D;

/// What a boolean is made of: a filled region, a closed path (filled by the nonzero rule), or
/// another boolean (shared, so one boolean can be an operand of several).
///
/// Use when: building a boolean group out of shapes, paths and other groups
/// Tags: boolean operand, boolean group member, compound shape member
/// Node: planned (booleanShapes)
using BooleanOperand = std::variant<Shape2D, Path2D, std::shared_ptr<const BooleanTree2D>>;

/// Where the outline turns from a piece of one operand to a piece of another (or of the same, see
/// `BooleanTree2D::corners`): node `nodeIndex` of outline `contourIndex` (the start of its piece
/// `nodeIndex`), `between` the operands of the piece arriving and the piece leaving, the unit
/// directions they arrive and leave in, and `outward`, the unit direction out of the filled side
/// that halves the corner.
///
///      outward ↑            the filled side is on the left of the way the outline runs
///        in ──●── out
///
/// `isConvex` is true where the outline turns toward its inside (a peak), false at a pit.
///
/// Use when: styling the corners where two operands meet differently from the corners of one;
///   rounding only the join between a stem and a bowl
/// Tags: boolean corner, junction, seam corner, operand meeting
/// Node: planned (booleanShapes)
struct BooleanCorner {
    int contourIndex = 0;
    int nodeIndex = 0;
    Vector2 point;
    std::array<int, 2> between{-1, -1};
    Vector2 incomingDirection;
    Vector2 outgoingDirection;
    Vector2 outward;
    bool isConvex = false;

    bool operator==(const BooleanCorner&) const = default;
};

/// An operation over operands that keeps them, and traces its outline on demand.
///
/// Conventions: y-up plane; the outline's outer contours run counterclockwise and its holes
/// clockwise (the nonzero rule), so the filled side is on the left of the way they run. `tolerance`
/// is as in `traceBoolean` (nothing: a thousandth of the biggest extent of the operands). `cut`
/// needs at least two operands and `subtract` at least one; other operations need one.
///
/// Use when: a drawing's boolean group (union of strokes, a letter with a window cut out) that
///   stays editable
/// Tags: boolean group, live boolean, compound shape, pathfinder, non-destructive boolean
/// Node: planned (booleanShapes)
class BooleanTree2D {
public:
    BooleanTree2D(BooleanOperation operation, std::vector<BooleanOperand> operands, std::optional<float> tolerance = std::nullopt);

    BooleanOperation operation() const { return operationValue; }
    const std::vector<BooleanOperand>& operands() const { return operandList; }
    std::optional<float> tolerance() const { return toleranceValue; }

    BooleanTree2D with(std::vector<BooleanOperand> operands) const;
    bool ok() const;
    std::vector<Path2D> contours() const;
    std::vector<std::vector<int>> sources() const;
    std::vector<BooleanCorner> corners(
        std::optional<int> first = std::nullopt, std::optional<int> second = std::nullopt, bool includeSameOperand = false) const;
    Vector2 outward(int contourIndex, int nodeIndex) const;

private:
    BooleanOperation operationValue;
    std::vector<BooleanOperand> operandList;
    std::optional<float> toleranceValue;
    /// Not traced yet / traced and failed / traced.
    mutable std::optional<std::optional<TracedBoolean>> tracedValue;

    const std::optional<TracedBoolean>& traced() const;
};

} // namespace editor
