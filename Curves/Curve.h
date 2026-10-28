// What arc length and frames need from a curve, so they are written once and work for every
// kind of curve we have (Engine/Math/Curves/Curve.swift).
//
// One curve code serves two worlds. A **profile** (the cross-section of a road, the outline
// that a lathe spins) is a curve in 2D, made of `Vector2` points. A **path** (the road
// itself, a camera track) is a curve in 3D, made of `Vector3` points. The Bézier formulas,
// the speed, and the arc length are the same in both, so they are written once, over any
// type that is a `CurvePoint`: `CubicBezier<Vector2>` is a profile piece and
// `CubicBezier<Vector3>` is a path piece. Only the things that need a third dimension, such
// as a frame with a normal and a binormal, are 3D only.
//
// How Swift's protocol becomes C++: Swift's `protocol Curve` with its extensions (ArcLength.swift,
// CurveFrame.swift) is the class template `Curve<Derived, Point>`, used as a base class by
// the curve itself (the "curiously recurring template pattern"): `class CubicBezier : public
// Curve<CubicBezier<Point>, Point>`. A curve provides `parameterRange()`, `point(parameter)`
// and `firstDerivative(parameter)`; the base class adds everything built from those three.
// The base class only declares its members here; ArcLength.h and CurveFrame.h define them, and
// are included at the end of this file, so including Curve.h is enough.
#pragma once

#include <concepts>
#include <vector>

#include "../Linear/Vector2.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A kind of point a curve can pass through: a `Vector2` on a plane or a `Vector3` in space.
/// It lists only what the curve code needs, so adding a new kind of point stays easy. (Swift's
/// `CurvePoint` protocol.)
///
/// Use when: writing a curve or a function on curves that should work in 2D and in 3D
/// Tags: curve point, generic point, 2D and 3D curves, vector protocol, point type
template <typename Candidate>
concept CurvePoint = requires(Candidate first, Candidate second, float number) {
    // The sum and the difference of two points, component by component.
    { first + second } -> std::convertible_to<Candidate>;
    { first - second } -> std::convertible_to<Candidate>;
    // Every component multiplied by the same number.
    { first * number } -> std::convertible_to<Candidate>;
    // The length, as in `Vector3::length`.
    { first.length() } -> std::convertible_to<float>;
    // The point a fraction `amount` of the way from `start` to `end`, as in `Vector3::linearBlend`.
    { Candidate::linearBlend(first, second, number) } -> std::convertible_to<Candidate>;
    // Whether every component is an ordinary number: none infinite, none NaN.
    { first.isFinite() } -> std::convertible_to<bool>;
};

static_assert(CurvePoint<Vector2>);
static_assert(CurvePoint<Vector3>);

/// The numbers a closed range `lowerBound…upperBound` runs over (Swift's `ClosedRange<Float>`).
///
/// Use when: telling where a curve starts and ends in its parameter
/// Tags: closed range, parameter range, interval, domain
/// Node: none (value type)
struct ParameterRange {
    float lowerBound = 0;
    float upperBound = 0;

    constexpr bool operator==(const ParameterRange&) const = default;

    /// Whether `value` lies in the range, ends included.
    constexpr bool contains(float value) const { return lowerBound <= value && value <= upperBound; }
};

struct CurveFrame;

/// A path through 2D or 3D space, described by a point for every value of a parameter.
///
/// `Point` is `Vector2` for a profile and `Vector3` for a path in space.
///
/// A curve derives from `Curve<ItsOwnType, Point>` and provides three functions:
///
///     ParameterRange parameterRange() const;     where the curve starts and ends in its parameter
///     Point point(float parameter) const;        the position on the curve
///     Point firstDerivative(float parameter) const;   the velocity
///
/// Conventions: positions are in world units (Z-up, right-handed, in 3D). The parameter is a plain
/// number with no unit; it is *not* distance along the curve (see `ArcLength` for that).
/// Both functions must be smooth enough to evaluate anywhere in `parameterRange`; what
/// happens outside the range is up to each curve and documented there.
///
/// Use when: writing code that works with any path, such as a road builder that accepts both
///   Bézier and spline
/// Tags: curve protocol, path, parametric curve, spline interface, generic curve
template <typename Derived, CurvePoint PointType>
class Curve {
public:
    /// The kind of point the curve passes through: `Vector2` or `Vector3`.
    using Point = PointType;

    // The base holds no data, so two curves are equal when their own data is (Swift's synthesized ==).
    bool operator==(const Curve&) const = default;

    // MARK: ArcLength.h

    float length(float start, float end) const;
    float totalLength() const;
    float parameterAtDistance(float distance) const;
    float parameterAtDistance(float distance, float startParameter) const;
    std::vector<float> evenlySpacedParameters(int count) const;
    std::vector<Point> evenlySpacedPoints(int count) const;

    // MARK: CurveFrame.h (3D curves only)

    Vector3 unitTangent(float parameter) const
        requires std::same_as<PointType, Vector3>;
    CurveFrame frame(float parameter, Vector3 up = Vector3::zAxis()) const
        requires std::same_as<PointType, Vector3>;
    std::vector<CurveFrame> rotationMinimizingFrames(const std::vector<float>& parameters, Vector3 up = Vector3::zAxis()) const
        requires std::same_as<PointType, Vector3>;

protected:
    const Derived& derived() const { return static_cast<const Derived&>(*this); }
};

} // namespace editor

#include "ArcLength.h"
#include "CurveFrame.h"
