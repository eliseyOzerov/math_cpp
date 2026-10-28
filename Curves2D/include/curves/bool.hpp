// The boolean object: an operation over outlines that keeps its operands, as a boolean group does in a drawing app.
// Its outline is worked out from them when asked for (boolean.hpp), so an operand can be changed and the boolean made
// again, and an operand can be a boolean itself. Each piece of the outline knows the operand it was cut from, which
// is what makes the places where operands meet selectable: a corner between a stem and a bowl is the corner whose
// pieces come from the stem's operand and the bowl's, wherever the curves happen to cross. There a shape can be
// added or a cut made, in the direction that points out of the outline (see BoolCorner).
//
// Operations, over the operands in order: union (inside any), intersect (inside all), subtract (inside the first and
// none of the rest), exclude (inside an odd number) and cut (the first divided along the others' outlines: its parts
// inside them and outside, as separate contours). Each reduces to a rule over winding numbers, so one algorithm
// serves them all.
//
// Ported from fountain-ts/src/curves/bool.ts.
//
// Decisions:
// - An Operand is a variant of contours, a Path, or a shared Bool (so a Bool can be an operand of several).
// - The outline is traced once, on first use, and cached in the object (not thread-safe while untraced).
#pragma once

#include "curves/boolean.hpp"
#include "curves/geometry.hpp"
#include "curves/path.hpp"

#include <memory>
#include <optional>
#include <variant>
#include <vector>

namespace curves {

/** The operations a Bool can be. */
enum class Op { union_, intersect, subtract, exclude, cut };

class Bool;

/** What a Bool is made of: outlines, closed paths, or other Bools. */
using Operand = std::variant<std::vector<Contour>, Path, std::shared_ptr<const Bool>>;

/** Where the outline turns from a piece of one operand to a piece of another (or of the same, see corners): node
 *  `node` of contour `contour` (the start of its piece `node`), `between` the operands of the piece arriving and the
 *  piece leaving, the directions they arrive and leave in, and `outward`, the unit direction out of the filled side
 *  that halves the corner. `convex` is true where the outline turns away from its inside (a peak), false at a pit. */
struct BoolCorner {
  int contour, node;
  Pt p;
  std::array<int, 2> between;
  Pt in, out, outward;
  bool convex;
};

struct BoolCornersOptions {
  bool same = false;
};

class Bool {
 public:
  /** `op` over `operands`, points closer than `eps` units counting as one (see boolean.hpp). */
  Bool(Op op, std::vector<Operand> operands, double eps = 0.25);

  Op op() const { return op_; }
  const std::vector<Operand>& operands() const { return operands_; }
  double eps() const { return eps_; }

  /** The same operation over other operands, as when one of them has been redrawn. */
  Bool with(std::vector<Operand> operands) const;

  /** Whether the outline could be traced; where a loop doesn't close, `contours` gives the operands unchanged. */
  bool ok() const;

  /** The outline: outer contours clockwise, holes counter-clockwise (y up). */
  std::vector<Contour> contours() const;

  /** For each piece of each contour, the operand it was cut from (−1 where the outline couldn't be traced). */
  std::vector<std::vector<int>> sources() const;

  /** The corners where the operands `a` and `b` meet (in either order), where `a` meets any other when `b` is left
   *  out, or where any two different operands meet when both are; `same` includes the nodes between two pieces of
   *  one operand as well. */
  std::vector<BoolCorner> corners(std::optional<int> a = {}, std::optional<int> b = {}, const BoolCornersOptions& o = {}) const;

  /** The direction out of the outline at node `node` of contour `contour` (see BoolCorner). */
  Pt outward(int contour, int node) const;

 private:
  const std::optional<Traced>& traced() const;

  Op op_;
  std::vector<Operand> operands_;
  double eps_;
  mutable std::optional<std::optional<Traced>> traced_;
};

}  // namespace curves
