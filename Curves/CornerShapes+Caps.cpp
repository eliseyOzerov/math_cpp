// Caps: the end of a stroke's outline reshaped by an open curve, done as two booleans. See
// CornerShapes.h. Ported from graphics_cpp curves/corners.cpp (`caps`, `capped`).
#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "CornerShapes.h"

namespace editor {

namespace {

float signOf(float value) { return value > 0 ? 1.0f : (value < 0 ? -1.0f : 0.0f); }

/// How far along a ray from `origin` in `direction` the shape's outlines are first met, within `far`.
std::optional<float> firstReach(const Shape2D& shape, Vector2 origin, Vector2 direction, float far) {
    std::optional<float> best;
    Vector2 rayEnd = origin + direction * far;
    for (const Path2D& outline : shape.outlines) {
        for (const OutlinePiece& piece : outlinePieces(outline)) {
            for (const CurveCrossing& crossing : piece.cubic.crossings(origin, rayEnd, true)) {
                float reach = crossing.secondParameter * far;
                if (!best || reach < *best) best = reach;
            }
        }
    }
    return best;
}

} // namespace

/// A cap that comes to a point: up from one side to the tip and down to the other, across a box two
/// units wide (x −1…1) and `height` high.
///
///        ●          tip at (0, height)
///       ╱ ╲
///     ●───── ●      (−1, 0) and (1, 0)
///
/// Use when: a pointed end on a stroke, a pen nib, an arrow-like terminal
/// Tags: triangle cap, pointed cap, spike end, arrow cap
/// Node: planned (capStroke)
CapShape CapShape::triangle(float height) {
    Path2D curve(Vector2(-1, 0));
    curve.addLine(Vector2(0, height));
    curve.addLine(Vector2(1, 0));
    return CapShape(curve);
}

/// A square end raised by `height`: up the left side, across, down the right, across a box two
/// units wide.
///
/// Use when: a flat extension beyond the stroke's end, a slab terminal
/// Tags: rectangle cap, square cap, flat cap, slab end, projecting cap
/// Node: planned (capStroke)
CapShape CapShape::rectangle(float height) {
    Path2D curve(Vector2(-1, 0));
    curve.addLine(Vector2(-1, height));
    curve.addLine(Vector2(1, height));
    curve.addLine(Vector2(1, 0));
    return CapShape(curve);
}

/// A half ellipse `height` high across a box two units wide: a half circle at height 1.
///
/// How: two cubics with the circle handle length 0.5522847 (the kappa of a quarter circle) scaled
/// by `height`: (−1,0) → (0,h) → (1,0).
///
/// Use when: a round end on a stroke
/// Tags: arc cap, round cap, half circle end, rounded terminal
/// Node: planned (capStroke)
CapShape CapShape::arc(float height) {
    constexpr float kappa = 0.5522847498f;
    Path2D curve(Vector2(-1, 0));
    curve.addCubic(Vector2(-1, kappa * height), Vector2(-kappa, height), Vector2(0, height));
    curve.addCubic(Vector2(kappa, height), Vector2(1, kappa * height), Vector2(1, 0));
    return CapShape(curve);
}

/// The stroke with the end at `end` reshaped by `cap`.
///
/// How: work in the stroke's own units, u across (−1…1, side to side) and v out along it (0 at the
/// end), both in half-widths measured on the outline (rays across the stroke a little behind the end).
/// A band across the stroke is laid a little behind where the cap starts, far enough that the stroke's
/// own end, cut square to its tangent, lies clear past the line. The stroke is first cut off at the
/// band (stroke minus band), then the cap, closed under the curve by lines out to the stroke's sides,
/// is added in the band (cap and band, joined to the cut stroke). It is two booleans on purpose: in one,
/// the stroke's sides inside the band, a hair from the cap's sides on a curved stroke, were both counted
/// as edges and the loops did not close. The cap narrows below the band so its sides cross the band's
/// bottom and never run along the stroke's.
///
///              cap curve
///            ╭──────────╮
///      ──────┤ band     ├──────   stroke runs left to right, ends in the band
///            ╰──────────╯
///
/// Conventions: y-up plane; `end.direction` points out of the stroke. `end.halfWidth` is used where
/// the outline cannot be measured. An empty cap curve, or a boolean whose loops do not close, leaves the
/// stroke unchanged.
///
/// Source: graphics_cpp curves/corners.cpp `capped`.
///
/// Use when: ending a stroke with a point, a square or a half round; terminals of letters
/// Tags: cap stroke, stroke end, line cap, terminal, pen end, end shape
/// Node: planned (capStroke)
Shape2D cappedStroke(const Shape2D& stroke, const StrokeEnd& end, const CapShape& cap) {
    std::vector<OutlinePiece> segments = drawnPieces(cap.curve);
    if (segments.empty()) return stroke;
    Vector2 endDirection = end.direction.normalized();
    Vector2 right = endDirection.rightNormal();
    Bounds2 curveBox = cap.curve.bounds();
    float x0 = curveBox.minimum.x, y0 = curveBox.minimum.y, x1 = curveBox.maximum.x, y1 = curveBox.maximum.y;
    float scale = 2 / ((x1 - x0) != 0 ? (x1 - x0) : 1.0f);
    float alignment = clamped(cap.align.value_or(1), -1, 1);
    float lift = (1 - alignment) / 2 * (y1 - y0) * scale;
    // The stroke's middle and half-width across it at a depth behind the end, measured on the outline.
    struct Across {
        Vector2 centre;
        float halfWidth;
    };
    auto across = [&](float depth) {
        Vector2 origin = end.point + endDirection * -depth;
        float toRight = firstReach(stroke, origin, right, 4 * end.halfWidth).value_or(end.halfWidth);
        float toLeft = firstReach(stroke, origin, right * -1, 4 * end.halfWidth).value_or(end.halfWidth);
        return Across{origin + right * ((toRight - toLeft) / 2), (toLeft + toRight) / 2};
    };
    // Local units: u across (−1…1 side to side), v out along the stroke (0 at its end), both in half-widths.
    // The cap takes over a little behind where it starts (the end, or as deep as it carves), at `bottom`.
    float bandHalfWidth = 1.5f;
    float tip = (y1 - y0) * scale - lift;
    float low = -lift;
    float bottom = low - 0.25f;
    float floorLevel = bottom - 0.1f;
    float top = largerOf(tip, 0) + 0.5f;
    float depth = -bottom * end.halfWidth;
    Across middle = across(depth);
    Across further = across(depth + end.halfWidth / 2);
    float halfWidth = middle.halfWidth;
    // The way the stroke runs, from its middle a little deeper still, so the cap's sides meet the stroke's
    // on the band's line and go on along it (a curved stroke's end tangent would aim them off).
    std::optional<Vector2> measured = (middle.centre - further.centre).unitDirection();
    Vector2 along = measured ? *measured : endDirection;
    Vector2 acrossDirection = along.rightNormal();
    auto local = [&](Vector2 point) { return Vector2((point.x - (x0 + x1) / 2) * scale, (point.y - y0) * scale - lift); };
    auto world = [&](Vector2 point) {
        return middle.centre + (acrossDirection * (point.x * halfWidth) + along * ((point.y - bottom) * halfWidth));
    };
    std::vector<OutlinePiece> capPieces =
        mappedPieces(segments, [&](Vector2 point) { return world(local(point)); });
    Vector2 startLocal = local(segments.front().start());
    Vector2 endLocal = local(segments.back().end());
    float side = signOf(endLocal.x - startLocal.x) != 0 ? signOf(endLocal.x - startLocal.x) : 1.0f;
    // The cap: under the curve, between the stroke's sides: the curve, then out level to the side (if it
    // ends inside it), down to the band's bottom, in a little and down past it, across and back up. One closed
    // region, so no edge of it lies on another input's.
    std::vector<Vector2> outline{endLocal,
                                 {side, endLocal.y},
                                 {side, bottom},
                                 {side * 0.8f, floorLevel},
                                 {-side * 0.8f, floorLevel},
                                 {-side, bottom},
                                 {-side, startLocal.y},
                                 startLocal};
    std::vector<OutlinePiece> region = capPieces;
    for (size_t index = 0; index + 1 < outline.size(); index += 1) {
        if (outline[index].distance(outline[index + 1]) > 1e-7f) {
            region.push_back(OutlinePiece::line(world(outline[index]), world(outline[index + 1])));
        }
    }
    std::vector<Vector2> bandCorners{{-bandHalfWidth, bottom}, {-bandHalfWidth, top}, {bandHalfWidth, top}, {bandHalfWidth, bottom}};
    std::vector<OutlinePiece> bandPieces;
    for (size_t index = 0; index < 4; index += 1) {
        bandPieces.push_back(OutlinePiece::line(world(bandCorners[index]), world(bandCorners[(index + 1) % 4])));
    }
    Shape2D capRegion{{pathOfPieces(region, true)}, FillRule::nonzero};
    Shape2D band{{pathOfPieces(bandPieces, true)}, FillRule::nonzero};
    // The stroke cut off at the band, then the cap in the band added. Where either fails to close, the
    // end is left as it was rather than half cut.
    std::optional<Shape2D> cut = tryBooleanByRule(
        {stroke, band}, [](const std::vector<bool>& inside) { return inside[0] && !inside[1]; });
    if (cut) {
        std::optional<Shape2D> result = tryBooleanByRule(
            {*cut, capRegion, band}, [](const std::vector<bool>& inside) { return inside[0] || (inside[1] && inside[2]); });
        if (result) return *result;
    }
    return stroke;
}

} // namespace editor
