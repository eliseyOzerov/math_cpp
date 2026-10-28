// Two lines of the plane and where they cross (graphics_cpp curves/geometry `meet`).
#pragma once

#include <optional>

#include "../Linear/Scalar.h"
#include "../Linear/Vector2.h"

namespace editor {

/// Where two lines cross, as how far along each: the point is `first.origin + first.direction ·
/// alongFirst`, which is also `second.origin + second.direction · alongSecond`.
struct LineMeeting {
    float alongFirst = 0;
    float alongSecond = 0;
};

/// A line in the plane given by a point and a direction vector that is not required to be unit
/// length: with `origin` and `direction` it is also the segment origin … origin + direction, so
/// "how far along" 0 is the origin and 1 is the segment's far end.
///
/// Use when: finding where two sides of an outline or two offset curves cross
/// Tags: line line intersection, 2D lines, point and direction, parametric line, crossing
/// Node: planned (lineCrossing)
struct Line2 {
    Vector2 origin;
    Vector2 direction;

    bool operator==(const Line2&) const = default;

    /// Where this line meets `other`, or nothing when they run parallel.
    ///
    /// How: with w = other.origin − origin and denominator = direction × other.direction
    /// (2D cross), alongThis = (w × other.direction)/denominator and alongOther = (w ×
    /// direction)/denominator solve origin + s·direction = other.origin + r·other.direction.
    ///
    ///     origin ●───── s ────→ ✕ ←──── r ───● other.origin
    ///
    /// Parallel means |denominator| < `flat`. With `inside`, the lines are read as the segments
    /// origin … origin + direction and other.origin … other.origin + other.direction, and a
    /// crossing is reported only if both parameters lie strictly between `inside` and 1 − `inside`
    /// (so `inside` = 0 keeps the open segments, and shared end points are rejected).
    ///
    /// Conventions: y-up plane; the parameters are in units of the direction vectors. `flat` is
    /// compared with the raw cross product, so it scales with the direction lengths; the float
    /// default is `Tolerance::geometric` where the double original used 1e-9.
    ///
    /// Source: graphics_cpp curves/geometry `meet`.
    ///
    /// Use when: cutting a corner where two offset sides cross; testing whether two sides of a
    ///   polygon intersect
    /// Tags: meet, line intersection, segment intersection, 2D intersect, ray cast, cross product
    /// Node: planned (lineCrossing)
    std::optional<LineMeeting> meets(const Line2& other, float flat = Tolerance::geometric,
                                     std::optional<float> inside = std::nullopt) const {
        float denominator = direction.cross(other.direction);
        if (std::fabs(denominator) < flat) return std::nullopt;
        Vector2 between = other.origin - origin;
        float alongFirst = between.cross(other.direction) / denominator;
        float alongSecond = between.cross(direction) / denominator;
        if (inside) {
            float margin = *inside;
            bool strictlyInside = alongFirst > margin && alongFirst < 1 - margin && alongSecond > margin && alongSecond < 1 - margin;
            if (!strictlyInside) return std::nullopt;
        }
        return LineMeeting{alongFirst, alongSecond};
    }
};

} // namespace editor
