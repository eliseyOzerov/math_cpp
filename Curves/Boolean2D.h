// Booleans of filled regions: union, intersection, difference, exclusion and cut, on outlines
// made of lines and cubic curves. Ported from graphics_cpp's boolean.hpp and bool.hpp (Curves2D).
//
// One algorithm for every operation, in three steps (the declarations are here; Boolean2D.cpp
// holds the steps, Boolean2D+Operations.cpp the named operations on two shapes, and
// Boolean2D+Polylines.cpp the small polyline helpers the booleans' callers need):
//
//   cut    every piece is split where it crosses another piece; pieces that run along each other
//          are split at each other's ends, so their fragments match
//   keep   a fragment is kept where the rule holds on one side of it and not on the other, found
//          by testing a point a little to its left and a little to its right; a second copy of an
//          edge already kept is dropped
//   chain  the kept fragments are joined end to start into closed loops
//
// Known limits (as in graphics_cpp): curves that run along each other (not lines) are split only at
// each other's ends, so a partly shared curve may leave a sliver; a loop that does not close (a gap
// left by a missed crossing) makes the operation fail (`tryBoolean` returns nothing, `boolean`
// returns its inputs unchanged) rather than hand back a broken outline.
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "../Geometry/Segment2.h"
#include "Shape2D.h"

namespace editor {

/// Which boolean to take of two regions A and B.
///
///     unite     A ∪ B       everything either covers
///     intersect A ∩ B       only where both cover
///     subtract  A − B       A with B cut away
///     exclude   A ⊕ B       where exactly one covers
///     cut       A split along B's outline, every piece kept
///
/// Use when: combining two outlines into one; cutting a window out of a wall profile
/// Tags: boolean, union, intersection, difference, xor, exclusion, path ops, CSG 2D, pathfinder
/// Node: planned (booleanShapes)
enum class BooleanOperation { unite, intersect, subtract, exclude, cut };

/// Whether a point is inside the result, from whether it is inside each operand: entry `k` is true
/// when the point is filled by group `k` (by that group's own fill rule).
///
/// Use when: writing a rule the five named operations do not cover, such as "in the first group and
///   in exactly one of the others"
/// Tags: boolean rule, inside predicate, fill rule over groups
/// Node: planned (booleanShapes)
using BooleanRule = std::function<bool(const std::vector<bool>& insideGroups)>;

/// The result of a boolean together with where each piece came from: `sourceGroups[k][j]` is the
/// operand (group) that piece `j` of outline `k` was cut from, so a corner can be told by the two
/// operands that meet there.
///
/// Use when: styling the corners where two operands meet differently from the corners of one
/// Tags: boolean provenance, traced boolean, piece origin, source operand
/// Node: planned (booleanShapes)
struct TracedBoolean {
    /// Outer outlines counterclockwise, holes clockwise, nonzero rule.
    Shape2D shape;
    std::vector<std::vector<int>> sourceGroups;
};

std::optional<TracedBoolean> traceBoolean(
    const std::vector<Shape2D>& groups, const BooleanRule& rule, std::optional<float> tolerance = std::nullopt);

std::optional<Shape2D> tryBooleanByRule(
    const std::vector<Shape2D>& groups, const BooleanRule& rule, std::optional<float> tolerance = std::nullopt);

Shape2D booleanByRule(
    const std::vector<Shape2D>& groups, const BooleanRule& rule, std::optional<float> tolerance = std::nullopt);

std::optional<Shape2D> tryBooleanShapes(
    const Shape2D& first, const Shape2D& second, BooleanOperation operation,
    std::optional<float> tolerance = std::nullopt);

Shape2D booleanShapes(
    const Shape2D& first, const Shape2D& second, BooleanOperation operation,
    std::optional<float> tolerance = std::nullopt);

Shape2D united(const Shape2D& shape, std::optional<float> tolerance = std::nullopt);

// MARK: Polyline helpers (Boolean2D+Polylines.cpp)

/// Where a horizontal line crosses an edge: its x and its winding (+1 going up, −1 going down).
struct ScanCrossing {
    float x = 0;
    int winding = 0;

    bool operator==(const ScanCrossing&) const = default;
};

/// Two points and the distance between them.
struct ClosestPair {
    float distance = 0;
    Vector2 first;
    Vector2 second;

    bool operator==(const ClosestPair&) const = default;
};

std::vector<Segment2> scanlineEdges(const std::vector<std::vector<Vector2>>& closedPolylines);
std::vector<ScanCrossing> scanlineCrossings(const std::vector<Segment2>& edges, float height);
std::optional<ClosestPair> narrowestPoint(const std::vector<Vector2>& closedPolyline, float minimumRunApart);
ClosestPair closestPointsBetween(const std::vector<Vector2>& first, const std::vector<Vector2>& second);

} // namespace editor
