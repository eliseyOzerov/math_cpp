// A tube along a curve whose radius changes along it: an arm, a finger, a tail, a tendon. The
// curve is cut into short straight pieces, each piece is a round cone, and the tube is their
// union (Engine/Math/Fields/TaperedTube.swift).
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "../Curves/CubicBezier.h"
#include "../Geometry/Bounds.h"
#include "../Linear/Vector3.h"
#include "SignedDistances.h"

namespace editor {

/// A tube of varying radius along a 3D cubic Bézier: a chain of `RoundCone`s between samples
/// of the curve.
///
/// The ideal shape is the **canal surface**: the union of the balls of radius `r(t)` centred
/// on the curve's points `c(t)`. The tube replaces the curve by the polyline through
/// `n + 1` samples at even parameter steps `tᵢ = i/n`, and the radius by its values at
/// those samples. A round cone is exactly the union of the balls whose centre and radius
/// blend linearly between its two end balls, so the tube is the union of the balls
/// `(ĉ(t), r̂(t))`, with `ĉ` and `r̂` the piecewise-linear blends in the parameter.
///
///     curve  ╭──────╮            samples ●, one round cone between each pair
///          ●─┼──●───┼──●──●      the radius is interpolated by parameter, not arc length
///
/// Conventions: Z-up, right-handed; radii are lengths in the curve's units, 0 or more.
///
/// Use when: modelling a limb, a finger, a tail or a horn along a curved centre line;
///   sweeping a tapering radius along a bone
/// Tags: tapered tube, canal surface, sweep sphere, curved capsule, tube along curve, limb,
///   generalized cylinder, pipe, sdBezier
/// Node: taperedTubeField
struct TaperedTube {
    /// The round cones from the start of the curve to its end, one per piece.
    std::vector<RoundCone> pieces;
    /// Whether the curve needed more than `mostPieces` pieces for the tolerance and was cut
    /// into `mostPieces` instead, so it is coarser than asked.
    bool reachedMostPieces = false;

    /// The most pieces one tube is cut into, so a tiny tolerance or a huge curve cannot
    /// allocate without limit.
    static constexpr int mostPieces = 1000;

    /// The largest coordinate a control point may have: 1e18, far beyond any scene, and small
    /// enough that squares and sums of a few coordinates stay below the largest finite float.
    static constexpr float largestCoordinate = 1e18f;

    /// The tube along `curve` whose radius at the curve's parameter `t` is `radius(t)`.
    ///
    /// How the pieces are chosen: the centre line must stay within `tolerance` of the curve.
    /// On a step `Δt = 1/n` the straight blend misses a cubic by at most
    /// `Δt²/8 × max|c″|` (the error of linear interpolation), and `|c″| ≤ 6M` with
    /// `M = max(|P₀ − 2P₁ + P₂|, |P₁ − 2P₂ + P₃|)`, so
    ///
    ///     n = ⌈√(0.75 × M / tolerance)⌉        (at least 1, at most `mostPieces`)
    ///
    /// Source: Wang's formula, Wang (1984); Filip, Magedson & Markot, "Surface algorithms
    /// using bounds on derivatives", *CAGD* 3 (1986), here in 3D.
    ///
    /// **Error against the ideal canal surface:** the value at any point differs from
    /// `min over t of (|p − c(t)| − r(t))` by at most `tolerance + E_r`, where `E_r` is the
    /// largest gap between `radius` and its straight blend between samples (0 when `radius`
    /// is linear in `t`, as for the constructor with `startRadius` and `endRadius`).
    /// **This bound holds only when `reachedMostPieces` is false**: when the piece count was
    /// capped at `mostPieces`, the centre line can be farther than `tolerance` from the curve.
    /// Why: each ball moves by at most `|c − ĉ|` and changes radius by at most `|r − r̂|`,
    /// and each changes `|p − centre| − radius` by at most that much.
    ///
    /// Conventions: `tolerance` must be positive and finite; the control points must be
    /// finite and no larger than `largestCoordinate` (1e18) in any component, so the curve
    /// arithmetic (squares, differences of piece centres) cannot overflow; `radius` must give a
    /// finite value of 0 or more on 0…1. The two ends of the centre line are the curve's own
    /// `start` and `end`, not evaluated points. Swift traps on a broken precondition; the core
    /// throws a `GeometryProblem` with the same sentence.
    ///
    /// Use when: building a tube whose radius is not a straight blend between its ends
    /// Tags: tube with radius function, variable radius pipe, canal surface
    /// Node: taperedTubeField
    TaperedTube(const CubicBezier<Vector3>& curve, const std::function<float(float)>& radius, float tolerance);

    /// The tube along `curve` whose radius blends linearly in the curve's parameter from
    /// `startRadius` to `endRadius`, with a tolerance of `relativeTolerance` times the larger
    /// end radius (1% by default). Both radii must be 0 or more and at least one positive.
    TaperedTube(const CubicBezier<Vector3>& curve, float startRadius, float endRadius, float relativeTolerance = 0.01f);

    /// The signed distance from `point` to the union of the round cones: exact for that
    /// union outside it, conservative inside (as `FieldUnion`), and within the error stated
    /// on the constructor of the ideal canal surface's value.
    ///
    /// How: the smallest of the pieces' exact distances. A piece's distance is at least the
    /// distance to its box (the box holds it), so a piece whose box is already as far as the
    /// best value so far cannot win and is skipped without evaluating it. The result is the
    /// same as without the skip.
    ///
    /// Edge cases: a point that is not finite gives NaN.
    ///
    /// Use when: sampling a limb or finger of a character
    /// Tags: tube sdf, canal surface distance, union of round cones, sdBezier tube, limb sdf
    /// Node: inside taperedTubeField
    float signedDistance(Vector3 point) const;

    /// The box around all the pieces.
    std::optional<Bounds> bounds() const { return boundingBox; }

private:
    /// Each piece's box, in the same order, worked out once for culling.
    std::vector<Bounds> pieceBoxes;
    std::optional<Bounds> boundingBox;

    /// The number of straight pieces that keep the centre line within `tolerance` of
    /// `curve` (Wang's formula, see the constructor), before the cap; the largest 64-bit
    /// number when that number is too large to hold.
    static std::int64_t neededPieceCount(const CubicBezier<Vector3>& curve, float tolerance);
};

} // namespace editor
