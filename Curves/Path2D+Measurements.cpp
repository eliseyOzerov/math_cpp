// Questions about the region a path encloses: how big it is, where it sits, and whether a point
// is inside. A fill treats an open path as closed by a straight line from its last point back to
// its first, and so do these (Engine/Math/Paths/Path2D+Measurements.swift).
#include <vector>

#include "../Linear/Scalar.h"
#include "Path2D.h"

namespace editor {

namespace {

/// The crossing of the ray from `point` with the straight line a→b, by the half-open rule.
int lineRayCrossing(Vector2 lineStart, Vector2 lineEnd, Vector2 point) {
    bool isStartAbove = lineStart.y > point.y;
    bool isEndAbove = lineEnd.y > point.y;
    if (isStartAbove == isEndAbove) return 0;
    float side = (lineEnd - lineStart).cross(point - lineStart);
    if (isEndAbove) return side > 0 ? 1 : 0;
    return side < 0 ? -1 : 0;
}

/// The parameter in `lower…upper` where a curve that is monotonic in y there passes
/// through `height`, by bisection. `startsAbove` says whether the curve is above `height`
/// at `lower` (and so below at `upper`, or the reverse).
float parameterOfHeightChange(
    const CubicBezier<Vector2>& curve, float lower, float upper, bool startsAbove, float height) {
    float low = lower;
    float high = upper;
    // 40 halvings shrink the interval far below Float resolution.
    for (int halving = 0; halving < 40; halving += 1) {
        float middle = (low + high) / 2;
        if ((curve.point(middle).y > height) == startsAbove) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return (low + high) / 2;
}

} // namespace

/// The smallest axis-aligned rectangle that contains the whole path, curves included.
///
/// How: the start, the end of every piece, and the extreme points of every curved
/// piece (`CubicBezier::bounds`). Straight lines are bounded by their ends. Returns a
/// single point for an empty path.
///
/// Conventions: the path is inside up to rounding, as for `CubicBezier::bounds`: a point
/// evaluated on a curve may lie outside by a few ulps of its coordinates.
///
/// Use when: framing a profile in a view; scaling a shape to fit; rejecting far points
/// Tags: path bounds, bounding box, extent, tight bounds, shape size
/// Node: none (2D outline bounds; boundingBox measures 3D geometry)
Bounds2 Path2D::bounds() const {
    std::vector<Vector2> pointsToEnclose{start};
    for (const Edge& edge : edges()) {
        pointsToEnclose.push_back(edge.segment.end());
        if (std::optional<CubicBezier<Vector2>> curve = edge.cubic()) {
            for (float parameter : curve->extremaParameters()) {
                pointsToEnclose.push_back(curve->point(parameter));
            }
        }
    }
    return Bounds2::enclosing(pointsToEnclose);
}

/// The signed area of the region the path encloses: positive when the path runs
/// counterclockwise, negative when clockwise; the area is counted with its winding, so
/// a figure-eight's two lobes cancel and a doubly wound circle counts twice.
///
/// How: Green's theorem. For a closed outline, `area = ½∮(x dy − y dx)`. Each piece
/// contributes the area of its fan from a pivot (the path's start):
///
///     straight line a→b:  ½ (a − pivot) × (b − pivot)       (the shoelace term)
///     cubic:               CubicBezier.signedAreaSwept(around: pivot), exact
///
/// and the fans add up to the area. The closing line contributes zero when the pivot is
/// the start, so it is left out. Measuring from the start, not the origin, keeps the
/// arithmetic accurate when the shape is far from the origin.
///
/// Conventions: y-up plane, counterclockwise positive. An open path is measured as if
/// closed by the straight line back to its start. A path with no pieces has area 0.
///
/// Source: Green's theorem; shoelace formula (Gauss, 1795; Meister, 1769).
///
/// Use when: knowing how large a filled profile is; telling whether an outline runs
///   clockwise or counterclockwise
/// Tags: polygon area, signed area, shoelace, Green's theorem, path area, orientation,
///   winding direction
/// Node: profileArea
float Path2D::signedArea() const {
    float total = 0;
    for (const Edge& edge : edges()) {
        total = total + edge.signedAreaSwept(start);
    }
    return total;
}

/// How many times the path winds counterclockwise around `point`: positive for
/// counterclockwise turns, negative for clockwise, 0 for a point outside.
///
/// How: shoot a ray from `point` toward +x and add up its crossings with the outline,
/// +1 where the outline crosses it going up (counterclockwise around the point) and −1
/// going down. That sum is the winding number.
///
///         ●─────────────▶ ray to +x
///      ╭──┼─────────╮     an upward crossing right of the point counts +1
///      │  ●         │
///
/// A line a→b crosses the ray's line when exactly one end is above the point's y
/// (`y > point.y`; an end at exactly the same height counts as *not above*, the
/// half-open rule of Hormann and Agathos, so a ray through a vertex counts once, not
/// twice or not at all). The crossing lies to the right of the point when
/// `(b − a) × (point − a)` is positive for an upward line and negative for a downward
/// one. A curve is cut at its vertical extremes into pieces that only rise or only fall;
/// each piece crosses at most once, by the same rule, and the crossing parameter is
/// found by bisection (guaranteed because the piece is monotonic).
///
/// Conventions: y-up plane. An open path is closed by the straight line back to the
/// start. A point exactly *on* the outline is not reliably inside or outside: this
/// function gives a consistent but arbitrary answer for it, since a crossing exactly at
/// the point is not counted. Test points near a boundary with a margin if it matters.
///
/// Source: Hormann & Agathos, "The point in polygon problem for arbitrary polygons"
/// (2001), the winding number algorithm; curves by monotone splitting.
///
/// Use when: filling a profile by the non-zero rule; telling inside from outside for a
///   self-crossing outline; picking a region
/// Tags: winding number, point in polygon, point in path, inside test, non-zero rule,
///   even-odd, fill rule, crossing number
/// Node: profileCrossings
int Path2D::windingNumber(Vector2 point) const {
    int total = 0;
    for (const Edge& edge : edgesWithClosingLine()) {
        total = total + edge.rayCrossings(point);
    }
    return total;
}

/// The signed area of the fan from `pivot` over this piece (see `Path2D::signedArea`).
///
/// How: a line a→b gives the triangle `½ (a − pivot) × (b − pivot)`; a curve gives
/// `CubicBezier::signedAreaSwept(pivot)`.
///
/// Use when: summing the area of a path piece by piece
/// Tags: edge area, fan area, segment area, shoelace term
/// Node: none (solver)
float Path2D::Edge::signedAreaSwept(Vector2 pivot) const {
    if (std::optional<CubicBezier<Vector2>> curve = cubic()) {
        return curve->signedAreaSwept(pivot);
    }
    return (start - pivot).cross(segment.end() - pivot) / 2;
}

/// The signed number of times this piece crosses the ray from `point` toward +x: +1 for
/// an upward crossing to the right of the point, −1 for a downward one, summed over
/// the piece. The rules are those of `Path2D::windingNumber`.
///
/// Use when: counting the winding of a path piece by piece
/// Tags: ray crossing, crossing number, edge crossings, winding contribution
/// Node: none (solver)
int Path2D::Edge::rayCrossings(Vector2 point) const {
    std::optional<CubicBezier<Vector2>> maybeCurve = cubic();
    if (!maybeCurve) return lineRayCrossing(start, segment.end(), point);
    const CubicBezier<Vector2>& curve = *maybeCurve;
    // Cut the curve where it turns around vertically; between cuts it is monotonic in y.
    std::vector<float> cuts{0};
    for (float parameter : curve.verticalExtremaParameters()) {
        if (parameter > 0 && parameter < 1) cuts.push_back(parameter);
    }
    cuts.push_back(1);
    // The heights at the two ends come from the end points themselves: evaluating at 0
    // or 1 can miss them by a rounding step, and then a vertex exactly at the ray's
    // height would be "above" for this piece but not for its neighbour, breaking the
    // half-open rule and counting one crossing too many or too few.
    std::vector<float> heights;
    for (size_t index = 0; index < cuts.size(); index += 1) {
        if (index == 0) {
            heights.push_back(curve.start.y);
        } else if (index == cuts.size() - 1) {
            heights.push_back(curve.end.y);
        } else {
            heights.push_back(curve.point(cuts[index]).y);
        }
    }
    int total = 0;
    for (size_t index = 0; index + 1 < cuts.size(); index += 1) {
        bool isLowerAbove = heights[index] > point.y;
        bool isUpperAbove = heights[index + 1] > point.y;
        if (isLowerAbove == isUpperAbove) continue;
        float crossingParameter = parameterOfHeightChange(curve, cuts[index], cuts[index + 1], isLowerAbove, point.y);
        if (!(curve.point(crossingParameter).x > point.x)) continue;
        total += isUpperAbove ? 1 : -1;
    }
    return total;
}

} // namespace editor
