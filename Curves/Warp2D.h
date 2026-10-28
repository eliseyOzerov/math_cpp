// Warps: any map of the plane applied to curves and paths, and the warps made of it: bows across a
// height (a spine), shifts by a function of height, wobble, a smooth noise field, and a mesh of nine
// points over a box. Ported from graphics_cpp's warp (warp.hpp, warp.cpp).
//
// A cubic is warped by mapping its ends and carrying each handle by the warp's derivative there (a
// numerical Jacobian), so the curve leaves and arrives in the warped directions and joins stay smooth
// where they were. That is exact for affine maps, and for a shift that is quadratic in height applied
// to a cubic rising evenly (a straight stroke under a bow); for anything else a cubic is first cut
// into `pieces` so the warped curve follows the map closely. Cutting adds nodes, so warp after the
// nodes have been picked and moved.
//
// Wobble is a field over the plane, not a function along each curve, so two curves through the same
// place move the same way there: strokes that overlap still meet, and a ring still closes.
//
// Affine warps need no cutting: `Warp2D::affine(transform)` is the map of an `AffineTransform2`, and
// `Path2D::mapped` carries a path through it exactly.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "../Linear/AffineTransform2.h"
#include "../Linear/Vector2.h"
#include "CubicBezier.h"
#include "Path2D.h"
#include "Profile.h"
#include "Shape2D.h"

namespace editor {

/// How finely to warp: `pieces` per cubic (1 keeps the nodes), or one piece per `step` of a cubic's
/// control polygon (at least one), and the step of the numerical derivative.
///
/// Use when: balancing how closely a warped curve follows a bending map against how many nodes it gets
/// Tags: warp options, subdivision, warp detail, derivative step
/// Node: planned (warpPath)
struct WarpSettings {
    /// Pieces per cubic, rounded and at least 1; wins over `step` when both are set.
    std::optional<float> pieces;
    /// Cut a cubic into one piece per `step` of control-polygon length (rounded up, at least 1).
    std::optional<float> step;
    /// The offset of the central differences that find the map's derivative, in plane units. 0.01 in
    /// floats: a smaller step drowns in rounding at coordinates of 100 (graphics_cpp used 1e-3 in doubles).
    float derivativeStep = 0.01f;

    bool operator==(const WarpSettings&) const = default;
};

/// A node of an editable outline: its place and its handles as offsets from it (absent at an end).
///
/// Use when: warping node and handle data kept apart from a path, as an editor holds them
/// Tags: node with handles, anchor with handles, bezier node
/// Node: none (editor data)
struct WarpNode {
    Vector2 point;
    std::optional<Vector2> incoming;
    std::optional<Vector2> outgoing;

    bool operator==(const WarpNode&) const = default;
};

/// A smooth noise field over the plane that moves each point a little in x and in y: its
/// `amplitude`, how far it moves a point each way at most, in units; its `frequency`, how many bends
/// it makes in a thousand units (an em in most fonts; 8 puts them about 125 apart); `octaves`, finer
/// layers each at twice the frequency and half the amplitude; and a `seed` for another field of the
/// same kind.
///
/// Use when: hand-drawn wobble of an outline; shaky ink; irregular edges
/// Tags: wobble, jitter, hand drawn, noise displacement, roughen
/// Node: planned (wobble)
struct Wobble {
    float amplitude = 0;
    float frequency = 8;
    float octaves = 1;
    float seed = 0;

    bool operator==(const Wobble&) const = default;
};

/// Maps of the plane and their application to curves, paths and shapes.
///
/// Conventions: y-up plane; a map must be defined on the whole plane near the curves it warps (the
/// derivative is read a `derivativeStep` either side of each end).
///
/// Use when: bending, bowing, shearing or roughening outlines while keeping them smooth
/// Tags: warp, deform, distort, bend, free-form deformation, plane map
/// Node: planned (warpPath)
struct Warp2D {
    /// A map of the plane.
    using PlaneMap = std::function<Vector2(Vector2)>;
    /// A one-variable wave (profile.h).
    using Wave = profile::Wave;

    // MARK: Maps and derivatives (Warp2D.cpp)

    static PlaneMap affine(const AffineTransform2& transform);
    static std::array<Vector2, 2> jacobian(const PlaneMap& map, Vector2 at, float derivativeStep = 0.01f);
    static WarpNode warpedNode(const WarpNode& node, const PlaneMap& map, float derivativeStep = 0.01f);

    // MARK: Curves, paths and shapes (Warp2D.cpp)

    static std::vector<CubicBezier<Vector2>> warpedCubic(
        const CubicBezier<Vector2>& cubic, const PlaneMap& map, const WarpSettings& settings = {});
    static std::vector<CubicBezier<Vector2>> warpedCubics(
        const std::vector<CubicBezier<Vector2>>& cubics, const PlaneMap& map, const WarpSettings& settings = {});
    static Path2D warpedPath(const Path2D& path, const PlaneMap& map, const WarpSettings& settings = {});
    static Shape2D warpedShape(const Shape2D& shape, const PlaneMap& map, const WarpSettings& settings = {});

    // MARK: Warps (Warp2D+Maps.cpp)

    static std::vector<Vector2> meshGrid(Vector2 lowerCorner, Vector2 upperCorner);
    static PlaneMap mesh(Vector2 lowerCorner, Vector2 upperCorner, std::vector<Vector2> grid);
    static PlaneMap spine(float bow, float lowerHeight, float upperHeight);
    static PlaneMap spine(Wave shift, float lowerHeight, float upperHeight);
    static PlaneMap pinned(PlaneMap map, std::vector<float> heights, float reach);

    // MARK: Wobble and noise (Warp2D+Wobble.cpp)

    static float wavelength(const Wobble& wobble);
    static PlaneMap wobbleMap(const Wobble& wobble);
    static float wobbleStep(const Wobble& wobble);
    static Wave wobbleWave(const Wobble& wobble, float length, bool closed = false);
    static float scatter(std::int64_t seed, std::int64_t key);
    static std::function<float(float, float)> noise2(float seed);
};

} // namespace editor
