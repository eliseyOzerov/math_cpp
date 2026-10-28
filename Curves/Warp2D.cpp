// Applying a map of the plane to nodes, cubics, paths and shapes.
#include <cmath>

#include "../Geometry/Polyline2.h"
#include "../Linear/MathProblem.h"
#include "Shape2D+Pieces.h"
#include "Warp2D.h"

namespace editor {

namespace {

/// `vector` carried by the derivative whose columns are the images of the unit steps.
Vector2 carriedBy(const std::array<Vector2, 2>& derivative, Vector2 vector) {
    return derivative[0] * vector.x + derivative[1] * vector.y;
}

/// One cubic warped as one: its ends mapped, its handles carried by the derivative at their ends.
CubicBezier<Vector2> warpedAsOne(const CubicBezier<Vector2>& cubic, const Warp2D::PlaneMap& map, float derivativeStep) {
    Vector2 start = map(cubic.start);
    Vector2 end = map(cubic.end);
    Vector2 startHandle = start + carriedBy(Warp2D::jacobian(map, cubic.start, derivativeStep), cubic.startHandle - cubic.start);
    Vector2 endHandle = end + carriedBy(Warp2D::jacobian(map, cubic.end, derivativeStep), cubic.endHandle - cubic.end);
    return CubicBezier<Vector2>(start, startHandle, endHandle, end);
}

} // namespace

/// The map of an affine transform: a point goes where `transform.applied` sends it.
///
/// Use when: warping with a move, turn, scale or shear (exact with `warpedCubic` and one piece)
/// Tags: affine warp, linear map, transform as map
/// Node: planned (transform2D)
Warp2D::PlaneMap Warp2D::affine(const AffineTransform2& transform) {
    return [transform](Vector2 point) { return transform.applied(point); };
}

/// The derivative of `map` at `at`: the images of the unit steps, as the columns of the Jacobian.
///
/// How: central differences, column x = (map(at + (h, 0)) − map(at − (h, 0))) / 2h and column y the
/// same upwards, with h = `derivativeStep`. Exact for affine maps up to rounding; for others the
/// error is about h² times the third derivative.
///
///     J = ┌ ∂x'/∂x  ∂x'/∂y ┐        result[0] = first column, result[1] = second
///         └ ∂y'/∂x  ∂y'/∂y ┘
///
/// Conventions: `derivativeStep` must be above 0, or a `MathProblem` is thrown.
///
/// Source: central difference formula; graphics_cpp curves/warp `jacobian`.
///
/// Use when: carrying a handle through a bending map; checking that a map does not fold
/// Tags: jacobian, derivative of map, numerical derivative, central difference, local linearization
/// Node: planned (warpPath)
std::array<Vector2, 2> Warp2D::jacobian(const PlaneMap& map, Vector2 at, float derivativeStep) {
    if (!(derivativeStep > 0)) throw MathProblem("The step of a derivative must be above 0.");
    Vector2 right = map(Vector2(at.x + derivativeStep, at.y));
    Vector2 left = map(Vector2(at.x - derivativeStep, at.y));
    Vector2 up = map(Vector2(at.x, at.y + derivativeStep));
    Vector2 down = map(Vector2(at.x, at.y - derivativeStep));
    return {(right - left) / (2 * derivativeStep), (up - down) / (2 * derivativeStep)};
}

/// A node warped by `map`: its place mapped, its handles carried by the derivative there, so it
/// keeps its nodes and its joins stay smooth (exact for affine maps).
///
/// Use when: moving the nodes of an editable outline through a warp
/// Tags: warp node, warp handles, carry handles, bend anchor
/// Node: planned (warpPath)
WarpNode Warp2D::warpedNode(const WarpNode& node, const PlaneMap& map, float derivativeStep) {
    std::array<Vector2, 2> derivative = jacobian(map, node.point, derivativeStep);
    WarpNode result = node;
    result.point = map(node.point);
    if (node.incoming) result.incoming = carriedBy(derivative, *node.incoming);
    if (node.outgoing) result.outgoing = carriedBy(derivative, *node.outgoing);
    return result;
}

/// A cubic warped by `map`, as one cubic or cut into pieces first (see `WarpSettings`).
///
/// How: the pieces count is `settings.pieces`, else ceil(control-polygon length ÷ `settings.step`),
/// else 1, rounded and at least 1; the cubic is cut at even parameters
/// (`CubicBezier::splitAtEvenParameters`) and each piece warped as one: ends mapped, handles
/// carried by the Jacobian at their ends.
///
///     start' = map(start)         startHandle' = start' + J(start) · (startHandle − start)
///     end'   = map(end)           endHandle'   = end'   + J(end)   · (endHandle − end)
///
/// Use when: bending a curve, with more pieces where the map bends sharply
/// Tags: warp cubic, bend bezier, deform curve, subdivide and warp
/// Node: planned (warpPath)
std::vector<CubicBezier<Vector2>> Warp2D::warpedCubic(
    const CubicBezier<Vector2>& cubic, const PlaneMap& map, const WarpSettings& settings) {
    float pieces = 1;
    if (settings.pieces) {
        pieces = *settings.pieces;
    } else if (settings.step && *settings.step != 0) {
        Polyline2 controlPolygon{{cubic.start, cubic.startHandle, cubic.endHandle, cubic.end}};
        pieces = std::ceil(controlPolygon.length() / *settings.step);
    }
    int count = std::max(1, static_cast<int>(std::lround(pieces)));
    std::vector<CubicBezier<Vector2>> warped;
    for (const CubicBezier<Vector2>& piece : cubic.splitAtEvenParameters(count)) {
        warped.push_back(warpedAsOne(piece, map, settings.derivativeStep));
    }
    return warped;
}

/// Cubics warped by `map`, in order (see `warpedCubic`).
///
/// Use when: warping the pieces of an outline already taken apart
/// Tags: warp cubics, bend curves
/// Node: planned (warpPath)
std::vector<CubicBezier<Vector2>> Warp2D::warpedCubics(
    const std::vector<CubicBezier<Vector2>>& cubics, const PlaneMap& map, const WarpSettings& settings) {
    std::vector<CubicBezier<Vector2>> warped;
    for (const CubicBezier<Vector2>& cubic : cubics) {
        for (const CubicBezier<Vector2>& piece : warpedCubic(cubic, map, settings)) warped.push_back(piece);
    }
    return warped;
}

/// A path warped by `map`: every piece, a line included, becomes cubics (so it can bend), joined
/// end to end from the mapped start. A closed path stays closed.
///
/// How: `warpedCubics` over the path's pieces, the closing line of a closed path included. Node marks
/// are kept when no piece was cut (the closing line of a closed path, if it was only implied, becomes
/// a piece of its own), else dropped; the nodes
/// are then read afresh from the new handles.
///
/// Conventions: a path with no pieces has only its start mapped.
///
/// Use when: bowing, waving or roughening an outline
/// Tags: warp path, bend path, deform outline, wobble outline
/// Node: planned (warpPath)
Path2D Warp2D::warpedPath(const Path2D& path, const PlaneMap& map, const WarpSettings& settings) {
    std::vector<OutlinePiece> pieces = path.isClosed() ? outlinePieces(path) : drawnPieces(path);
    std::vector<CubicBezier<Vector2>> cubics;
    for (const OutlinePiece& piece : pieces) cubics.push_back(piece.cubic);
    std::vector<CubicBezier<Vector2>> warped = warpedCubics(cubics, map, settings);
    if (warped.empty()) return Path2D(map(path.start));
    Path2D result(warped.front().start);
    for (const CubicBezier<Vector2>& cubic : warped) result.addCubic(cubic.startHandle, cubic.endHandle, cubic.end);
    if (path.isClosed()) result.close();
    if (warped.size() == cubics.size()) {
        for (int node = 0; node <= static_cast<int>(path.segments().size()); node += 1) result.setNodeKind(node, path.nodeKind(node));
    }
    return result;
}

/// A shape warped by `map`: every outline through `warpedPath`, the fill rule kept.
///
/// Use when: bending a letter or a filled region with its holes
/// Tags: warp shape, bend region, deform contours, warp outlines
/// Node: planned (warpShape)
Shape2D Warp2D::warpedShape(const Shape2D& shape, const PlaneMap& map, const WarpSettings& settings) {
    Shape2D result;
    result.fillRule = shape.fillRule;
    for (const Path2D& outline : shape.outlines) result.outlines.push_back(warpedPath(outline, map, settings));
    return result;
}

} // namespace editor
