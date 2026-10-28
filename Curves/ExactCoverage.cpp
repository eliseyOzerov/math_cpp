// The exact-coverage reference rasterizer: each pixel row cut into trapezoids between edge
// crossings, each trapezoid's area shared out to the pixels it covers.
#include "ExactCoverage.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "../Linear/MathProblem.h"

namespace editor {

namespace {

// The rasterizer works in double: it is the oracle, so its own rounding has to sit far below the
// float resolution of the coverages it is compared against.

/// A point of the plane in double precision.
struct PlanePoint {
    double x = 0;
    double y = 0;
};

/// One straight, non-horizontal edge of a flattened outline, stored bottom end first, with the
/// direction the outline runs along it: +1 up, −1 down.
struct RowEdge {
    PlanePoint lower;
    PlanePoint upper;
    int direction = 0;

    /// Where the edge is at `height`, between `lower.y` and `upper.y`: the linear blend of its
    /// ends, `lower.x + (upper.x − lower.x)·(height − lower.y)/(upper.y − lower.y)`.
    double xAtHeight(double height) const {
        double share = (height - lower.y) / (upper.y - lower.y);
        return lower.x + (upper.x - lower.x) * share;
    }

    /// Whether the edge reaches across all of `bottom…top`.
    bool spans(double bottom, double top) const { return lower.y <= bottom && upper.y >= top; }

    /// Whether some of the edge lies strictly inside the heights `bottom…top`.
    bool overlaps(double bottom, double top) const { return lower.y < top && upper.y > bottom; }
};

/// The edges of every outline of `shape`, flattened to `tolerance`, each outline closed by the
/// line from its last point back to its first (as a fill closes it). Horizontal edges are left
/// out: they bound no area between two heights and never change a winding number along a row.
std::vector<RowEdge> flattenedEdges(const Shape2D& shape, float tolerance) {
    std::vector<RowEdge> edges;
    for (const Path2D& outline : shape.outlines) {
        std::vector<Vector2> points = outline.flattened(tolerance);
        for (size_t index = 0; index < points.size(); index += 1) {
            Vector2 from = points[index];
            Vector2 to = points[(index + 1) % points.size()];
            if (from.y == to.y) continue;
            PlanePoint start{from.x, from.y};
            PlanePoint end{to.x, to.y};
            if (to.y > from.y) {
                edges.push_back(RowEdge{start, end, 1});
            } else {
                edges.push_back(RowEdge{end, start, -1});
            }
        }
    }
    return edges;
}

/// The height inside `bottom…top` where two edges that both span it cross, if they do.
///
/// How: the gap `first.x − second.x` is linear in height; when it has strictly opposite signs
/// at `bottom` and `top` it passes zero once, at `bottom + (top − bottom)·g₀/(g₀ − g₁)`. A gap
/// of zero at an end is a touch, not a crossing inside, and needs no cut.
std::optional<double> crossingHeight(const RowEdge& first, const RowEdge& second, double bottom, double top) {
    double gapAtBottom = first.xAtHeight(bottom) - second.xAtHeight(bottom);
    double gapAtTop = first.xAtHeight(top) - second.xAtHeight(top);
    if (!((gapAtBottom < 0 && gapAtTop > 0) || (gapAtBottom > 0 && gapAtTop < 0))) return std::nullopt;
    double height = bottom + (top - bottom) * gapAtBottom / (gapAtBottom - gapAtTop);
    if (!(height > bottom && height < top)) return std::nullopt;
    return height;
}

/// `value` held to `lower…upper`.
double clampedTo(double value, double lower, double upper) { return std::min(std::max(value, lower), upper); }

/// The integral, over a slab of `height`, of an edge's position held to the column
/// `columnLeft…columnRight`, where the edge runs straight from `startX` at the slab's bottom to
/// `endX` at its top.
///
/// How: with t = 0…1 up the slab, x(t) = startX + (endX − startX)·t. Held to the column,
/// clamp(x(t)) is linear in t except where x(t) meets a column side, so cut t there (at
/// t = (side − startX)/(endX − startX), only for a side strictly between startX and endX, so the
/// division is by a nonzero number); on each piece the trapezoid rule is exact:
///
///     ∫ clamp(x) dy = height · Σ (t₊ − t₋)·(clamp(x(t₋)) + clamp(x(t₊)))/2
///
/// The difference of this for an area's right and left edge is the area between them inside
/// the column, because clamp(right) − clamp(left) is the width of [left, right] ∩ column when
/// left ≤ right.
double integralOfClampedPosition(double startX, double endX, double height, double columnLeft, double columnRight) {
    double cuts[4] = {0, 1, 0, 0};
    int cutCount = 2;
    for (double side : {columnLeft, columnRight}) {
        bool isStrictlyBetween = (startX < side && side < endX) || (endX < side && side < startX);
        if (isStrictlyBetween) {
            cuts[cutCount] = (side - startX) / (endX - startX);
            cutCount += 1;
        }
    }
    std::sort(cuts, cuts + cutCount);
    double total = 0;
    for (int index = 0; index + 1 < cutCount; index += 1) {
        double lowerParameter = cuts[index];
        double upperParameter = cuts[index + 1];
        double lowerX = clampedTo(startX + (endX - startX) * lowerParameter, columnLeft, columnRight);
        double upperX = clampedTo(startX + (endX - startX) * upperParameter, columnLeft, columnRight);
        total += (upperParameter - lowerParameter) * (lowerX + upperX) / 2;
    }
    return total * height;
}

/// Adds to `rowCoverages` the area, column by column, of the trapezoid between `left` and
/// `right` over the slab `bottom…top` (the two edges span the slab and do not cross in it).
void addTrapezoid(const RowEdge& left, const RowEdge& right, double bottom, double top,
                  std::vector<double>& rowCoverages) {
    double leftAtBottom = left.xAtHeight(bottom);
    double leftAtTop = left.xAtHeight(top);
    double rightAtBottom = right.xAtHeight(bottom);
    double rightAtTop = right.xAtHeight(top);
    double height = top - bottom;
    int columns = static_cast<int>(rowCoverages.size());
    // Only the columns the trapezoid reaches; the formula gives 0 for the rest anyway. Held to
    // the grid before converting, so a shape far outside cannot overflow an int.
    double leftmost = std::floor(std::min(leftAtBottom, leftAtTop));
    double rightmost = std::ceil(std::max(rightAtBottom, rightAtTop)) - 1;
    int firstColumn = static_cast<int>(clampedTo(leftmost, 0, columns));
    int lastColumn = static_cast<int>(clampedTo(rightmost, -1, columns - 1));
    for (int column = firstColumn; column <= lastColumn; column += 1) {
        double columnLeft = column;
        double columnRight = column + 1;
        double rightIntegral = integralOfClampedPosition(rightAtBottom, rightAtTop, height, columnLeft, columnRight);
        double leftIntegral = integralOfClampedPosition(leftAtBottom, leftAtTop, height, columnLeft, columnRight);
        rowCoverages[column] += rightIntegral - leftIntegral;
    }
}

/// The heights where row `bottom…top` has to be cut so that inside each piece no edge ends and
/// no two edges cross: the row's own bottom and top, every edge end strictly inside, and every
/// crossing of two edges strictly inside. Sorted, without repeats.
std::vector<double> slabHeights(const std::vector<const RowEdge*>& rowEdges, double bottom, double top) {
    std::vector<double> heights{bottom, top};
    for (const RowEdge* edge : rowEdges) {
        if (edge->lower.y > bottom && edge->lower.y < top) heights.push_back(edge->lower.y);
        if (edge->upper.y > bottom && edge->upper.y < top) heights.push_back(edge->upper.y);
    }
    for (size_t first = 0; first < rowEdges.size(); first += 1) {
        for (size_t second = first + 1; second < rowEdges.size(); second += 1) {
            // Two edges can only cross where both are present.
            double sharedBottom = std::max({bottom, rowEdges[first]->lower.y, rowEdges[second]->lower.y});
            double sharedTop = std::min({top, rowEdges[first]->upper.y, rowEdges[second]->upper.y});
            if (!(sharedTop > sharedBottom)) continue;
            if (std::optional<double> height = crossingHeight(*rowEdges[first], *rowEdges[second], sharedBottom, sharedTop)) {
                heights.push_back(*height);
            }
        }
    }
    std::sort(heights.begin(), heights.end());
    heights.erase(std::unique(heights.begin(), heights.end()), heights.end());
    return heights;
}

} // namespace

float CoverageImage::coverage(int column, int row) const {
    if (column < 0 || column >= columns || row < 0 || row >= rows) {
        throw MathProblem("Pixel (" + std::to_string(column) + ", " + std::to_string(row) + ") is outside a "
                          + std::to_string(columns) + " × " + std::to_string(rows) + " coverage image.");
    }
    return coverages[static_cast<size_t>(row) * columns + column];
}

float CoverageImage::totalCoverage() const {
    double total = 0;
    for (float pixelCoverage : coverages) total += pixelCoverage;
    return static_cast<float>(total);
}

/// For each unit pixel square of a `columns` × `rows` grid, the exact area of the part of it
/// that `shape` fills under its fill rule: where the outlines' winding number is nonzero
/// (nonzero) or odd (even-odd). Curves are flattened to `flatteningTolerance` first.
///
/// How (one pixel row y = r…r+1 at a time):
///
/// 1. **Flatten.** Each outline becomes a closed polygon (`Path2D::flattened`). Every point of
///    a curve is within `flatteningTolerance` of its polyline at the same parameter.
/// 2. **Cut the row into slabs.** Cut at every height inside the row where an edge ends and
///    where two edges cross. Inside one slab no edge ends and none cross, so every edge present
///    runs from the slab's bottom to its top and their left-to-right order is the same at
///    every height: the slab is a stack of trapezoids side by side.
///
///         slab top    ─────╱────────╲─────╱──
///                         ╱  w = 1   ╲ 0 ╱      between neighbouring edges the winding
///         slab bottom ───╱────────────╲─╱────   number is one constant
///
/// 3. **Wind along the slab.** Order the edges by their position at the slab's middle. Far to
///    the left, a ray toward +x meets every edge, so the winding number there is Σ direction
///    (+1 for an edge the outline runs up, −1 down, as in `Path2D::windingNumber`); passing an
///    edge takes its direction away. Between edge i and i+1 the winding is fixed, so the
///    trapezoid between them is wholly filled or wholly empty (`isFilled`). This is what keeps
///    overlapping outlines exact: the fill rule is applied to the winding, never to a sum of
///    signed areas.
/// 4. **Share each filled trapezoid out to the columns.** For column [c, c+1], its area there is
///    ∫(clamp(right(y)) − clamp(left(y))) dy over the slab, each clamp to [c, c+1]; the clamped
///    position is piecewise linear in y, with a kink only where the edge meets a column side, so
///    the trapezoid rule over those pieces is exact (`integralOfClampedPosition`).
///
/// The pixel's coverage is the sum over every slab of its row. For a polygon this is exact up to
/// double rounding, overlaps and self-crossings included. For a curved outline the polygon's area
/// differs from the curve's: with B(t) the curve, P(t) the polyline and D = B − P (zero at each
/// polyline point), Green's theorem gives ΔA = ½∫(B×B′ − P×P′)dt = ½∫D×(P′ + B′)dt after
/// integrating P×D′ by parts, so |ΔA| ≤ ½·tolerance·(length of P + length of B) ≤ tolerance ×
/// the length of the curved pieces. That bounds the total; a single pixel's error is bounded by
/// the tolerance times the length of curve inside it.
///
/// Conventions: y-up; pixel (c, r) is the square [c, c+1] × [r, r+1] of the shape's own plane,
/// so a caller scales and moves the shape into pixel units first. The parts of the shape outside
/// the grid are cut off. Coverage is not clamped: it lies in 0…1 up to rounding. Throws a
/// `MathProblem` for a non-positive or non-finite tolerance, or a size outside 0…16384. The cost
/// is about rows × (edges + edges in a row²): slow on purpose, it is the reference.
///
/// Source: Green's theorem for the area of a polygon; the decomposition of a polygon into
/// trapezoids at every vertex and crossing height (Preparata & Shamos, *Computational Geometry*,
/// 1985, §7.2; Chazelle, "Triangulating a simple polygon in linear time", 1991, §2); exact-area
/// pixel coverage as the antialiasing reference (Catmull, "A hidden-surface algorithm with
/// anti-aliasing", SIGGRAPH 1978). Derived here; no code was copied.
///
/// Use when: checking a fast glyph rasterizer pixel by pixel; rendering a glyph sheet where
///   accuracy matters more than speed; measuring how far a rasterizer drifts where contours overlap
/// Tags: exact coverage, area coverage, antialiasing reference, scanline rasterizer, trapezoid
///   decomposition, glyph rendering, alpha mask, nonzero, even-odd, oracle
/// Node: none (text rendering reference, not a modelling step)
CoverageImage exactCoverage(const Shape2D& shape, int columns, int rows, float flatteningTolerance) {
    if (!(flatteningTolerance > 0) || !std::isfinite(flatteningTolerance)) {
        throw MathProblem("The flattening tolerance must be positive and finite; got "
                          + std::to_string(flatteningTolerance) + ".");
    }
    constexpr int largestSize = 16384;
    if (columns < 0 || rows < 0 || columns > largestSize || rows > largestSize) {
        throw MathProblem("A coverage image must be 0…16384 pixels each way; got " + std::to_string(columns)
                          + " × " + std::to_string(rows) + ".");
    }
    CoverageImage image{columns, rows, std::vector<float>(static_cast<size_t>(columns) * rows, 0)};
    std::vector<RowEdge> edges = flattenedEdges(shape, flatteningTolerance);
    std::vector<double> rowCoverages(columns);
    for (int row = 0; row < rows; row += 1) {
        double bottom = row;
        double top = row + 1;
        std::vector<const RowEdge*> rowEdges;
        for (const RowEdge& edge : edges) {
            if (edge.overlaps(bottom, top)) rowEdges.push_back(&edge);
        }
        if (rowEdges.empty()) continue;
        std::fill(rowCoverages.begin(), rowCoverages.end(), 0);
        std::vector<double> heights = slabHeights(rowEdges, bottom, top);
        for (size_t slab = 0; slab + 1 < heights.size(); slab += 1) {
            double slabBottom = heights[slab];
            double slabTop = heights[slab + 1];
            double middle = (slabBottom + slabTop) / 2;
            std::vector<const RowEdge*> slabEdges;
            int winding = 0;
            for (const RowEdge* edge : rowEdges) {
                if (!edge->spans(slabBottom, slabTop)) continue;
                slabEdges.push_back(edge);
                winding += edge->direction;
            }
            std::sort(slabEdges.begin(), slabEdges.end(), [middle](const RowEdge* first, const RowEdge* second) {
                return first->xAtHeight(middle) < second->xAtHeight(middle);
            });
            for (size_t index = 0; index + 1 < slabEdges.size(); index += 1) {
                winding -= slabEdges[index]->direction;
                if (!isFilled(winding, shape.fillRule)) continue;
                addTrapezoid(*slabEdges[index], *slabEdges[index + 1], slabBottom, slabTop, rowCoverages);
            }
        }
        for (int column = 0; column < columns; column += 1) {
            image.coverages[static_cast<size_t>(row) * columns + column] = static_cast<float>(rowCoverages[column]);
        }
    }
    return image;
}

} // namespace editor
