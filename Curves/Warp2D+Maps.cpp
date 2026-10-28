// The warps themselves: a mesh of nine points, a spine, and a pin that holds lines in place.
#include <cmath>
#include <limits>

#include "../Linear/Scalar.h"
#include "Warp2D.h"

namespace editor {

/// The nine points of a 3×3 mesh at rest over the box `lowerCorner`…`upperCorner`: rows from the
/// bottom, x first in each.
///
/// How: point (row, column) = lower + (column/2 · width, row/2 · height).
///
///     6 ● 7 ● 8          row 2 (top)
///     3 ● 4 ● 5          row 1
///     0 ● 1 ● 2          row 0 (bottom)
///
/// Use when: starting a free-form deformation by moving some of the nine points
/// Tags: mesh grid, lattice, control grid, FFD, 3x3 grid
/// Node: planned (meshWarp)
std::vector<Vector2> Warp2D::meshGrid(Vector2 lowerCorner, Vector2 upperCorner) {
    std::vector<Vector2> grid;
    for (float row : {0.0f, 0.5f, 1.0f}) {
        for (float column : {0.0f, 0.5f, 1.0f}) {
            grid.push_back(Vector2(
                linearBlend(lowerCorner.x, upperCorner.x, column), linearBlend(lowerCorner.y, upperCorner.y, row)));
        }
    }
    return grid;
}

/// The warp of a 3×3 mesh over the box with its nine points moved to `grid` (as `meshGrid` orders
/// them): each point is the grid's blend by quadratic Lagrange weights across and up, so the plane
/// bends smoothly through the nine points, is the identity at rest and is affine while the grid is.
///
/// How: with u = (x − lower.x)/width and v = (y − lower.y)/height, the weights over the nodes 0, ½, 1
/// are
///
///     w(u) = ( 2(u − ½)(u − 1),  −4u(u − 1),  2u(u − ½) )
///
/// and the result is Σ over rows r and columns c of w(v)[r] · w(u)[c] · grid[3r + c]. Outside the
/// box the same quadratics continue. A box with no width or height is taken as one unit that way.
///
/// Conventions: `grid` needs nine points, or a `MathProblem` is thrown.
///
/// Source: Lagrange interpolation on three nodes; graphics_cpp curves/warp `mesh`.
///
/// Use when: pulling a shape's middle out like a bulge; a quick free-form deformation
/// Tags: mesh warp, lattice deformation, FFD, lagrange grid, bulge, envelope distort
/// Node: planned (meshWarp)
Warp2D::PlaneMap Warp2D::mesh(Vector2 lowerCorner, Vector2 upperCorner, std::vector<Vector2> grid) {
    if (grid.size() != 9) throw MathProblem("A mesh needs nine points: three rows of three.");
    auto weights = [](float position) {
        return std::array<float, 3>{2 * (position - 0.5f) * (position - 1), -4 * position * (position - 1), 2 * position * (position - 0.5f)};
    };
    float width = upperCorner.x - lowerCorner.x;
    float height = upperCorner.y - lowerCorner.y;
    float widthOrOne = width != 0 ? width : 1;
    float heightOrOne = height != 0 ? height : 1;
    return [=](Vector2 point) {
        std::array<float, 3> across = weights((point.x - lowerCorner.x) / widthOrOne);
        std::array<float, 3> up = weights((point.y - lowerCorner.y) / heightOrOne);
        Vector2 result;
        for (int row = 0; row < 3; row += 1) {
            for (int column = 0; column < 3; column += 1) {
                result = result + grid[static_cast<size_t>(row * 3 + column)] * (up[static_cast<size_t>(row)] * across[static_cast<size_t>(column)]);
            }
        }
        return result;
    };
}

/// A shift across by `bow` halfway up between two heights: none at either height, `bow` in the
/// middle, parabolic in between (and beyond). A straight upright stroke bows exactly.
///
/// How: u = (y − lowerHeight)/(upperHeight − lowerHeight) (1 for equal heights) and x moves by
/// 4·bow·u·(1 − u); y stays.
///
/// Use when: curving a straight stem; a bent pole
/// Tags: spine, bow, bend, arch shift, parabolic shift
/// Node: planned (spineWarp)
Warp2D::PlaneMap Warp2D::spine(float bow, float lowerHeight, float upperHeight) {
    return spine([bow](float position) { return 4 * bow * position * (1 - position); }, lowerHeight, upperHeight);
}

/// A shift across by `shift` of the share of the way from `lowerHeight` up to `upperHeight`
/// (beyond them too): a spine of any shape.
///
/// How: x moves by shift((y − lowerHeight)/(upperHeight − lowerHeight)); equal heights count as one
/// unit apart. y stays.
///
/// Use when: leaning, waving or tapering a stroke by height
/// Tags: spine, shift by height, lean, sway, wave along height
/// Node: planned (spineWarp)
Warp2D::PlaneMap Warp2D::spine(Wave shift, float lowerHeight, float upperHeight) {
    float span = upperHeight - lowerHeight;
    float spanOrOne = span != 0 ? span : 1;
    return [=](Vector2 point) { return Vector2(point.x + shift((point.y - lowerHeight) / spanOrOne), point.y); };
}

/// A warp whose push up or down fades out within `reach` of any of the lines y = `heights` (smoothly,
/// none on them), so what sits on those lines stays there while it moves along them. Across it moves
/// as before.
///
/// How: with d the distance of the point's y to the nearest line, y' = y + (map(p).y − y) · smoothStep(d / reach);
/// x' = map(p).x. A `reach` of 0 or less pins nothing.
///
/// Use when: bending a glyph without lifting it off the baseline
/// Tags: pin, fade vertical warp, hold baseline, lock lines
/// Node: planned (pinWarp)
Warp2D::PlaneMap Warp2D::pinned(PlaneMap map, std::vector<float> heights, float reach) {
    return [=](Vector2 point) {
        Vector2 moved = map(point);
        float nearest = std::numeric_limits<float>::infinity();
        for (float height : heights) nearest = std::min(nearest, std::fabs(point.y - height));
        float freedom = reach > 0 ? smoothStep(nearest / reach) : 1;
        return Vector2(moved.x, point.y + (moved.y - point.y) * freedom);
    };
}

} // namespace editor
