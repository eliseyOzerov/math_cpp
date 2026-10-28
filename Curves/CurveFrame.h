// A frame is a little coordinate system riding along a curve: it says which way is forward
// and which way is "up" and "sideways" at that spot. Sweeping a road profile along a curve
// places the profile using one frame per sample (Engine/Math/Curves/CurveFrame.swift).
#pragma once

#include <optional>
#include <vector>

#include "../Linear/NumberText.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "../Linear/Vector3.h"
#include "Curve.h"

namespace editor {

/// A set of three perpendicular unit directions at a point on a curve.
///
///     normal
///        ▲   ● position
///        │  ╱
///        │ ╱  tangent → along the curve
///        ●─────────────▶
///       ╱
///      ▼ binormal (out of the page for this view)
///
/// Conventions: `tangent`, `normal` and `binormal` all have length 1 and are at right angles
/// to each other. They are **right-handed**: `tangent × normal = binormal` (so, for a curve
/// heading along +Y with normal +Z, the binormal is +X). World space is Z-up, right-handed.
///
/// Use when: orienting a road cross-section, a tube or a camera along a path
/// Tags: Frenet frame, tangent normal binormal, curve frame, TNB, local axes, orientation
///   along path
/// Node: curveFrames
struct CurveFrame {
    Vector3 position;
    Vector3 tangent;
    Vector3 normal;
    Vector3 binormal;

    bool operator==(const CurveFrame&) const = default;
};

namespace curve_detail {

/// How close to parallel the up vector may get to the tangent before we pick another
/// direction, as the sine of the angle between them. 0.01 is about 0.6 degrees.
inline constexpr float minimumSineToUpVector = 0.01f;

/// How far from a point of zero velocity `unitTangent` looks for the direction the curve
/// moves in, as a fraction of the width of the parameter range.
inline constexpr float stationaryPointStep = 1e-4f;

/// The unit vector perpendicular to `tangent` that is closest to `preferredDirection`:
/// the preferred direction with its component along the tangent taken out. Falls back
/// to a world axis when the preferred direction is too close to parallel with the tangent.
inline Vector3 normalPerpendicularTo(Vector3 tangent, Vector3 preferredDirection) {
    for (Vector3 candidate : {preferredDirection, Vector3::xAxis(), Vector3::yAxis(), Vector3::zAxis()}) {
        Vector3 sideways = candidate - tangent * tangent.dot(candidate);
        if (sideways.length() > minimumSineToUpVector * candidate.length()) {
            return sideways.normalized();
        }
    }
    throw MathProblem("A unit tangent cannot be parallel to all three world axes.");
}

} // namespace curve_detail

/// The direction the curve moves in at `parameter`, length 1: the velocity's direction, or,
/// where the velocity is zero, the direction the curve leaves (or arrives) in.
///
/// How: normalize `firstDerivative(at: parameter)` (by `Vector3.unitDirection`, which works
/// at any scale). Where the velocity is zero, as at the end of a Bézier whose handle sits on
/// its end point (`P1 = P0`), the curve still has a direction: take the velocity a small step
/// (`10⁻⁴` of the parameter range) further inside the range instead. For that Bézier the
/// velocity at `t` is `6t(1 − t)(P2 − P0) + 3t²(P3 − P2)`, which points along `P2 − P0` as
/// `t → 0`; at `t = 10⁻⁴` it is off by about `10⁻⁴ · |P3 − P2| / (2|P2 − P0|)` radians.
/// At the end of the range the step goes backwards, but the velocity there still points
/// forwards, the way the curve arrives.
///
/// Conventions: the curve must move near `parameter` (a curve at rest over the whole step,
/// such as all four control points equal, has no direction); this is checked.
///
/// Use when: the tangent of a sweep path or road whose control handles may be pulled back
///   onto their end points
/// Tags: unit tangent, curve direction, tangent at cusp, zero velocity, stationary point,
///   handle on end point, safe tangent
/// Node: inside curveFrames, curveToMesh
template <typename Derived, CurvePoint PointType>
Vector3 Curve<Derived, PointType>::unitTangent(float parameter) const
    requires std::same_as<PointType, Vector3>
{
    if (std::optional<Vector3> tangent = derived().firstDerivative(parameter).unitDirection()) return *tangent;
    ParameterRange range = derived().parameterRange();
    float step = (range.upperBound - range.lowerBound) * curve_detail::stationaryPointStep;
    bool isInLowerHalf = parameter < linearBlend(range.lowerBound, range.upperBound, 0.5f);
    float nearby = isInLowerHalf ? parameter + step : parameter - step;
    std::optional<Vector3> tangent = derived().firstDerivative(nearby).unitDirection();
    if (!tangent) {
        throw MathProblem(
            "The curve does not move near parameter " + numberText(parameter) + ", so it has no tangent.");
    }
    return *tangent;
}

/// The frame at one parameter, with the normal as close to `up` as the tangent allows.
/// This is the frame the rotation-minimizing sequence starts from.
///
/// How: the tangent is `unitTangent(at:)`, the normalized velocity. The normal is `up` with its tangent
/// component removed, normalized:
///
///     normal = normalize(up − (up · tangent)·tangent)        binormal = tangent × normal
///
/// Conventions: if `up` is (nearly) parallel to the tangent, as for a vertical climb
/// with the default up, no unique direction exists, so the first of +X, +Y, +Z that is
/// not parallel to the tangent is used instead. Where the velocity is zero the tangent is
/// the direction the curve moves in just beside `parameter` (see `unitTangent(at:)`).
///
/// Use when: placing one object along a path with a sensible up direction, such as a
///   lamp post by a road
/// Tags: frame at point, tangent normal binormal, Frenet frame, curve orientation,
///   path frame
/// Node: inside curveFrames, curveToMesh
template <typename Derived, CurvePoint PointType>
CurveFrame Curve<Derived, PointType>::frame(float parameter, Vector3 up) const
    requires std::same_as<PointType, Vector3>
{
    Vector3 tangent = unitTangent(parameter);
    Vector3 normal = curve_detail::normalPerpendicularTo(tangent, up);
    return CurveFrame{derived().point(parameter), tangent, normal, tangent.cross(normal)};
}

/// Frames at the given parameters whose normals **do not twist** around the curve: from
/// one frame to the next, the frame turns only as much as the curve's direction forces.
///
/// Why not just use `up` at every point (or the Frenet frame): `up` flips and spins when
/// the curve points near `up`, and the Frenet normal flips at inflection points and is
/// undefined on straight stretches. A swept road would visibly twist. Rotation-minimizing
/// frames only rotate "around the bend", never around the tangent.
///
/// How: the **double reflection method**. The first frame comes from `frame(at:up:)`. For
/// each next sample, with positions `xᵢ, xᵢ₊₁`, tangents `tᵢ, tᵢ₊₁` and normal `rᵢ`, and
/// "reflect(vector, across v)" meaning `vector − 2·(vector·v)/(v·v)·v` (a mirror in the
/// plane perpendicular to `v`):
///
///     v₁ = xᵢ₊₁ − xᵢ                  mirror 1 swaps xᵢ and xᵢ₊₁
///     rL = reflect(rᵢ, across v₁)      tL = reflect(tᵢ, across v₁)
///     v₂ = tᵢ₊₁ − tL                  mirror 2 turns tL into the true tangent tᵢ₊₁
///     rᵢ₊₁ = reflect(rL, across v₂)
///
/// Two mirrors compose to a rotation, here the smallest one carrying the old frame onto
/// the new tangent. The binormal follows as `tᵢ₊₁ × rᵢ₊₁`.
///
/// Degenerate mirrors. A single mirror on its own would flip the frame's handedness, so
/// mirrors must come in pairs:
/// - Two samples at the same position (`v₁ = 0`, as when a closed curve is sampled only
///   at its ends): mirror 1 uses `tᵢ` instead. That sends `tᵢ` to `−tᵢ` and leaves `rᵢ`
///   alone, so `v₂ = tᵢ₊₁ + tᵢ`, and the pair composes to the smallest rotation about
///   `tᵢ × tᵢ₊₁`.
/// - `v₂ = 0` means mirror 1 already produced the new tangent, so mirror 2 is skipped
///   and `rL` is kept. (It is not the "unchanged tangent" case: an unchanged tangent
///   along a straight step gives `v₂ = 2tᵢ₊₁`, and the two mirrors cancel.)
///
/// Conventions: `parameters` should be in increasing order and dense enough that the
/// curve does not turn sharply between samples; the method is accurate to the fourth
/// order in the spacing, so a few dozen samples per bend are plenty. Where the speed is
/// zero the tangent comes from just beside the parameter (`unitTangent(at:)`). Each frame's normal is re-orthogonalized against its
/// tangent, so rounding errors do not accumulate. An empty list gives an empty result.
///
/// Source: Wang, Jüttler, Zheng & Liu (2008), "Computation of rotation minimizing
/// frames", ACM Transactions on Graphics 27(1).
///
/// Use when: extruding a road, rail or tube along a curve without the profile
///   twisting; sweeping a shape along a path
/// Tags: rotation minimizing frame, parallel transport, sweep, extrude along curve,
///   no twist, double reflection
/// Node: curveFrames
template <typename Derived, CurvePoint PointType>
std::vector<CurveFrame> Curve<Derived, PointType>::rotationMinimizingFrames(
    const std::vector<float>& parameters, Vector3 up) const
    requires std::same_as<PointType, Vector3>
{
    if (parameters.empty()) return {};
    std::vector<CurveFrame> frames = {frame(parameters[0], up)};

    for (size_t index = 1; index < parameters.size(); index += 1) {
        float parameter = parameters[index];
        CurveFrame previous = frames.back();
        Vector3 position = derived().point(parameter);
        Vector3 tangent = unitTangent(parameter);

        Vector3 positionStep = position - previous.position;
        // With no step to mirror across, mirror across the old tangent instead, so the
        // pair of mirrors still makes a rotation (see "Degenerate mirrors" above).
        Vector3 firstMirror = positionStep.definesReflectionPlane() ? positionStep : previous.tangent;
        Vector3 reflectedNormal = previous.normal.reflectedAcrossPlane(firstMirror);
        Vector3 reflectedTangent = previous.tangent.reflectedAcrossPlane(firstMirror);
        Vector3 tangentStep = tangent - reflectedTangent;
        Vector3 rawNormal = reflectedNormal.reflectedAcrossPlane(tangentStep);

        Vector3 normal = curve_detail::normalPerpendicularTo(tangent, rawNormal);
        frames.push_back(CurveFrame{position, tangent, normal, tangent.cross(normal)});
    }
    return frames;
}

} // namespace editor
