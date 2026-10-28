// Measurements of a filled region: area, winding, inside test, bounds, direction of its outlines.
#include "Shape2D.h"

#include <cmath>

#include "../Linear/MathProblem.h"
#include "Shape2D+Pieces.h"

namespace editor {

/// The signed area of the region as the sum of its outlines' signed areas: counterclockwise
/// outlines add, clockwise ones take away (`Path2D::signedArea`).
///
/// How: Σ over outlines of the shoelace/Green area, open outlines closed by a straight line.
///
/// Conventions: y-up plane. For outlines that overlap (nonzero rule) the sum counts the overlap
/// once per outline, so it is the area only for a tidy shape: outer outlines counterclockwise,
/// holes clockwise, no overlaps.
///
/// Use when: the area of the result of a boolean; finding the biggest outline
/// Tags: shape area, polygon with holes area, signed area, region area
/// Node: planned (shapeArea)
float Shape2D::signedArea() const {
    float total = 0;
    for (const Path2D& outline : outlines) total += outline.signedArea();
    return total;
}

/// How many times the outlines wind counterclockwise round `point`: the sum of the outlines'
/// winding numbers (`Path2D::windingNumber`).
///
/// Use when: telling an outline from a hole at a point; the nonzero fill rule
/// Tags: winding number, region winding, nonzero rule
/// Node: planned (shapeContains)
int Shape2D::windingNumber(Vector2 point) const {
    int total = 0;
    for (const Path2D& outline : outlines) total += outline.windingNumber(point);
    return total;
}

/// Whether `point` is filled: by the nonzero rule when the winding number is not 0, by the even-odd
/// rule when it is odd.
///
/// Conventions: a point exactly on an outline may come out either way.
///
/// Use when: picking a region; testing a hole; the inside test the booleans classify by
/// Tags: point in shape, inside test, hit test, fill rule, contains
/// Node: planned (shapeContains)
bool Shape2D::contains(Vector2 point) const {
    int winding = windingNumber(point);
    return fillRule == FillRule::nonzero ? winding != 0 : (winding % 2) != 0;
}

/// The smallest rectangle holding every outline. Throws a `MathProblem` for a shape with no outlines.
///
/// Use when: fitting a shape in view; a quick overlap test before a boolean
/// Tags: shape bounds, bounding box, extent
/// Node: none (2D outline bounds)
Bounds2 Shape2D::bounds() const {
    if (outlines.empty()) throw MathProblem("A shape with no outlines has no bounds.");
    Bounds2 total = outlines.front().bounds();
    for (const Path2D& outline : outlines) total = total.unionWith(outline.bounds());
    return total;
}

/// The shape with every outline run backwards (`Path2D::reversed`), the fill rule kept.
///
/// Use when: turning a whole shape inside out in direction; flipping a mirrored shape
/// Tags: reverse shape, reverse contours, flip winding
/// Node: planned (reversePath)
Shape2D Shape2D::reversed() const {
    Shape2D result;
    result.fillRule = fillRule;
    for (const Path2D& outline : outlines) result.outlines.push_back(outline.reversed());
    return result;
}

/// The outlines turned so that the one enclosing the most area runs counterclockwise and all
/// the others clockwise, which makes them holes of it under the nonzero rule. Graphics_cpp's
/// `orient`, mirrored: it has the biggest outline clockwise, as TrueType fills glyphs; here
/// counterclockwise is positive.
///
/// How: the signed area of each outline (`Path2D::signedArea`); the largest |area| is the outer
/// one; an outline is reversed when its sign is not the one its role needs.
///
/// Use when: preparing the outlines of one stroke so overlapping strokes add up under the
///   nonzero rule instead of cancelling
/// Tags: orient contours, winding order, outer and hole, truetype direction, fix direction
/// Node: planned (orientShape)
Shape2D Shape2D::orientedForFill() const {
    Shape2D result;
    result.fillRule = fillRule;
    size_t outer = 0;
    std::vector<float> areas;
    for (const Path2D& outline : outlines) areas.push_back(outline.signedArea());
    for (size_t index = 0; index < areas.size(); index += 1) {
        if (std::fabs(areas[index]) > std::fabs(areas[outer])) outer = index;
    }
    for (size_t index = 0; index < outlines.size(); index += 1) {
        bool shouldBePositive = index == outer;
        bool isPositive = areas[index] >= 0;
        result.outlines.push_back(isPositive == shouldBePositive ? outlines[index] : outlines[index].reversed());
    }
    return result;
}

} // namespace editor
