// Angles as their own type, so a number in degrees is never mistaken for one in radians
// (Engine/Math/Angle.swift, Engine/Geometry/Angle+Between.swift).
#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include "Scalar.h"
#include "Vector2.h"
#include "Vector3.h"

namespace editor {

/// An angle. It can be created from degrees or radians and read back in either unit.
///
/// Engine code never passes a bare number as an angle. It is always `Angle::degrees(90)`, so
/// the unit can't be mistaken.
///
/// Internally, angles are stored in **radians**, because sine, cosine and every rotation
/// formula expect radians. One full turn is 360° = 2π radians.
///
///              90° = π/2
///                 │
///     180° = π ───┼─── 0°
///                 │
///             270° = 3π/2
///
/// Positive angles turn **counterclockwise**, from the x axis toward the y axis.
///
/// Swift's `Angle.radians(_)` is `Angle::fromRadians` here, because the stored value is
/// already called `radians`; Swift's `degrees` property is the member function `degrees()`.
///
/// Use when: passing a turn, field of view or tilt around without mixing up degrees and radians
/// Tags: angle, degrees, radians, unit conversion, rotation amount, deg2rad
/// Node: none (value type)
struct Angle {
    /// The angle in radians.
    float radians = 0;

    constexpr bool operator==(const Angle&) const = default;

    /// An angle given in radians.
    ///
    /// Use when: making an angle from a value a formula already gives in radians
    /// Tags: radians, angle from radians, rad, construct angle
    /// Node: inside sine, cosine, rotate, coneDirection
    static constexpr Angle fromRadians(float radians) { return Angle{radians}; }

    /// An angle given in degrees.
    ///
    /// How: radians = degrees × π / 180, with π/180 rounded to a float once, as in the Swift
    /// engine. A full turn is 360° or 2π radians, so one degree is π/180 radians.
    ///
    /// Use when: writing a camera field of view or rotation the way people read it, such as 90 degrees
    /// Tags: degrees, angle from degrees, deg, radians conversion, deg2rad, Mathf.Deg2Rad
    /// Node: inside sine, cosine, rotate, coneDirection
    static constexpr Angle degrees(float degrees) { return Angle{degrees * radiansPerDegree}; }

    /// The angle in degrees. How: degrees = radians × 180 / π.
    ///
    /// Use when: showing an angle to a person or printing it in a test
    /// Tags: to degrees, degrees value, rad2deg, angle in degrees, Mathf.Rad2Deg
    /// Node: inside sine, cosine, rotate, coneDirection
    constexpr float degrees() const { return radians / radiansPerDegree; }

    /// The angle whose cosine is `cosine`, between 0° and 180°: the inverse of `cosine()`.
    ///
    /// How: arccos(clamp(cosine, −1, 1)). On the unit circle it is the angle, measured
    /// counterclockwise from the x axis, of the point whose horizontal position is `cosine`,
    /// taken in the upper half (0° at cosine 1, 90° at 0, 180° at −1).
    ///
    /// Conventions: the input is clamped into −1…1 first. A dot product of two unit vectors is
    /// mathematically in that range but rounding can push it to 1.0000001, where a bare `acos`
    /// returns NaN; clamping returns 0° instead. NaN in gives NaN out.
    ///
    /// Precision: near cosine 1 the inverse is badly conditioned. cos θ ≈ 1 − θ²/2, and the
    /// float just below 1 is 1 − 2⁻²⁴, so the smallest angle other than 0 it can return is
    /// √(2·2⁻²⁴) ≈ 3.5e-4 rad (0.02°); smaller angles come back as 0, and angles a little above
    /// it only to a few digits (the same near 180°). For the angle between two nearly parallel
    /// vectors use `Angle::between`, which takes atan2 of the cross and dot products and stays
    /// accurate all the way down.
    ///
    /// Use when: finding the angle between two unit vectors from their dot product; the arc between two unit quaternions
    /// Tags: acos, arccos, inverse cosine, angle between vectors, Mathf.Acos, safe acos
    /// Node: inside slerp
    static Angle arcCosine(float cosine) { return Angle{std::acos(clamped(cosine, -1, 1))}; }

    /// No rotation at all.
    static constexpr Angle zero() { return Angle{0}; }

    /// The angle between two directions, from 0 to π.
    ///
    /// How: atan2(|a × b|, a · b). Unlike acos of the dot product, this stays accurate for
    /// nearly parallel directions, and it needs no normalising. Two zero vectors give 0.
    ///
    /// Use when: how sharply the surface bends across an edge
    /// Tags: angle between vectors, unsigned angle, Vector3.Angle
    /// Node: inside subdivide
    static Angle between(Vector3 first, Vector3 second) {
        return Angle{std::atan2(first.cross(second).length(), first.dot(second))};
    }

    /// The signed angle that turns the direction of `first` onto the direction of `second`:
    /// positive counterclockwise, in −180°…180°.
    ///
    /// Formula: `atan2(first × second, first · second)`. The cross product is |a||b|·sin θ and
    /// the dot product |a||b|·cos θ, so their ratio fixes θ in all four quadrants; unlike
    /// `arccos` of a dot product it stays accurate for nearly parallel vectors.
    ///
    ///          ▲ second
    ///          │ ╲  θ > 0 (counterclockwise)
    ///          │  ╲
    ///          ●───▶ first
    ///
    /// Each vector is first reduced to its direction (`Vector2::unitDirection`, which divides
    /// by the largest component first), so the products neither overflow nor underflow at any
    /// scale. Conventions: y up. A zero or non-finite vector has no direction: the result is
    /// 0. Two opposite directions give +180° or −180° depending on the sign of a
    /// rounding-level cross product.
    ///
    /// Use when: how far, and which way, a path turns at a corner; the sweep of an arc from
    ///   one radius to another
    /// Tags: signed angle, atan2, turn angle, deflection, angle between 2D vectors, winding
    ///   direction, Vector2.SignedAngle
    /// Node: inside roundCorners, chamferCorners
    static Angle signedTurn(Vector2 first, Vector2 second) {
        std::optional<Vector2> firstDirection = first.unitDirection();
        std::optional<Vector2> secondDirection = second.unitDirection();
        if (!firstDirection || !secondDirection) return zero();
        return Angle{std::atan2(firstDirection->cross(*secondDirection), firstDirection->dot(*secondDirection))};
    }

    // MARK: Trigonometry

    /// The sine of the angle. On a circle of radius 1, it is the height (y) of the point
    /// reached by turning this angle from the x axis.
    ///
    /// Use when: getting the height of a point on a circle; building rotation matrices and wave shapes
    /// Tags: sin, sine, angle sine, Mathf.Sin, trigonometry
    /// Node: sine
    float sine() const { return std::sin(radians); }

    /// The cosine of the angle. On a circle of radius 1, it is the horizontal position (x) of
    /// the point reached by turning this angle from the x axis.
    ///
    /// Use when: getting the horizontal position of a point on a circle; the angle between two unit vectors
    /// Tags: cos, cosine, angle cosine, Mathf.Cos, trigonometry
    /// Node: cosine
    float cosine() const { return std::cos(radians); }

    /// The tangent of the angle: sine / cosine. In a right triangle it is the opposite side
    /// divided by the adjacent side. It grows without limit as the angle nears 90°.
    ///
    /// The perspective projection uses it: looking at a wall 1 unit away, you see
    /// tangent(half the field of view) units up and the same down.
    ///
    /// Use when: computing a field-of-view scale for a projection; a slope from an angle
    /// Tags: tan, tangent, angle tangent, Mathf.Tan, slope, trigonometry
    /// Node: tangent
    float tangent() const { return std::tan(radians); }

    // MARK: Arithmetic

    /// The sum of two angles: `Angle::degrees(30) + Angle::degrees(60)` is 90°.
    ///
    /// How: the radians are added. Not wrapped: 300° + 90° is 390°, not 30°, so a sum can count
    /// whole turns.
    ///
    /// Use when: stepping a heading by a turn; the angle of the i-th piece of an arc, start plus a step times i
    /// Tags: angle add, angle sum, angle plus, add rotation, accumulate turn
    /// Node: none (value type)
    constexpr Angle operator+(Angle other) const { return Angle{radians + other.radians}; }

    /// The difference of two angles: `Angle::degrees(90) - Angle::degrees(30)` is 60°.
    ///
    /// Not wrapped: 30° − 90° is −60°, not 300°. Positive angles turn counterclockwise, so a
    /// negative result is a clockwise turn.
    ///
    /// Use when: finding how far one heading is from another; the turn left after subtracting a step
    /// Tags: angle subtract, angle difference, angle minus, delta angle, turn between
    /// Node: none (value type)
    constexpr Angle operator-(Angle other) const { return Angle{radians - other.radians}; }

    /// The opposite turn: `-Angle::degrees(30)` is −30°, the same turn clockwise.
    ///
    /// Use when: undoing a rotation; turning the other way by the same amount
    /// Tags: negate angle, opposite angle, reverse rotation, clockwise, minus angle
    /// Node: none (value type)
    constexpr Angle operator-() const { return Angle{-radians}; }

    /// The angle multiplied by a plain number: `Angle::degrees(45) * 2` is 90°.
    ///
    /// The usual use is turning a speed into a distance turned: a speed (per second) times the
    /// seconds elapsed is how far the object has turned.
    ///
    /// Use when: doubling or scaling a rotation, such as a step angle times a count
    /// Tags: scale angle, angle multiply, angle times number, multiples of an angle
    /// Node: none (value type)
    constexpr Angle operator*(float factor) const { return Angle{radians * factor}; }

    /// The angle divided by a plain number: `Angle::degrees(90) / 2` is 45°.
    ///
    /// Use when: splitting a turn into equal steps, such as a circle into sectors
    /// Tags: divide angle, angle division, half angle, equal steps, angle step
    /// Node: none (value type)
    constexpr Angle operator/(float divisor) const { return Angle{radians / divisor}; }

    // MARK: Direction

    /// This angle folded into (−90°, 90°]: a direction without caring which way along its line it
    /// runs. Whole half turns (180°) are added or taken away.
    ///
    /// How: result = angle − n·π with n = ceil(angle/π − ½). The range is open at −90° and
    /// closed at 90°: exactly −90° becomes 90°. A NaN or infinite angle is returned unchanged.
    ///
    ///           90° (kept)
    ///            │
    ///     ───────●─────── 0°   190° → 10°, −100° → 80°, 90° → 90°, −90° → 90°
    ///
    /// Source: graphics_cpp curves/geometry `halfTurn`.
    ///
    /// Use when: comparing the slopes of lines whatever way they are drawn; the angle of a line
    ///   as opposed to a ray
    /// Tags: half turn, fold angle, line direction, modulo pi, undirected angle, wrap angle
    /// Node: planned (foldAngle)
    Angle halfTurn() const {
        if (!std::isfinite(radians)) return *this;
        constexpr float quarter = pi / 2;
        float folded = radians - std::ceil(radians / pi - 0.5f) * pi;
        // A quarter turn computed from degrees can differ from `quarter` by a few rounding steps either
        // way; that much counts as exactly 90°, so 90° stays 90° and -90° becomes 90°.
        float rounding = 4 * std::numeric_limits<float>::epsilon() * quarter;
        if (folded > quarter + rounding) folded -= pi;
        if (folded <= -quarter + rounding) folded += pi;
        return Angle{folded};
    }

    /// The unit vector at this angle counterclockwise from the x axis: (cos, sin).
    ///
    /// Use when: turning a heading into a direction
    /// Tags: unit vector, direction from angle, polar, heading vector
    /// Node: planned (directionFromAngle)
    Vector2 direction() const { return Vector2(cosine(), sine()); }

    /// π as Swift's `Float.pi` holds it: rounded toward zero (0x1.921fb4p1), one step below the
    /// nearest float, so the two engines convert degrees and measure circles identically. Every
    /// file of the core uses this one, never a π of its own.
    static constexpr float pi = 0x1.921fb4p1f;

private:
    static constexpr float radiansPerDegree = pi / 180;
};

inline Vector2 Vector2::fromAngle(Angle angle) { return angle.direction(); }

inline Vector2 Vector2::rotated(Angle angle, Vector2 pivot) const {
    Vector2 offset = *this - pivot;
    float cosine = angle.cosine();
    float sine = angle.sine();
    return pivot + Vector2(offset.x * cosine - offset.y * sine, offset.x * sine + offset.y * cosine);
}

inline Vector2 Vector2::rotated(Angle angle) const { return rotated(angle, Vector2(0, 0)); }

} // namespace editor
