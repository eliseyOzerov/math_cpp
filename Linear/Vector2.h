// Points, directions and sizes on a plane (the Vector2 parts of Engine/Math/Vector.swift).
#pragma once

#include <cmath>
#include <optional>

namespace editor {

/// Two numbers, x and y, for a point, a direction or a size on a plane.
///
/// Conventions: x points right, y points up, and positive angles turn counterclockwise from x
/// toward y, as in `Angle`. When a 2D profile is placed in 3D, the caller decides which plane
/// it lies in. Arithmetic is component by component, like Swift's `SIMD2<Float>`.
///
/// Use when: describing a point of a 2D profile, a height field's slope or the corner of a grid
/// Tags: vector2, float2, SIMD2, 2D point, 2D vector, plane position, point on plane
/// Node: none (a value the nodes pass around)
struct Vector2 {
    float x = 0;
    float y = 0;

    /// The origin, (0, 0).
    constexpr Vector2() = default;
    /// The vector (x, y).
    constexpr Vector2(float x, float y) : x(x), y(y) {}

    constexpr Vector2 operator+(Vector2 other) const { return Vector2(x + other.x, y + other.y); }
    constexpr Vector2 operator-(Vector2 other) const { return Vector2(x - other.x, y - other.y); }
    constexpr Vector2 operator*(Vector2 other) const { return Vector2(x * other.x, y * other.y); }
    constexpr Vector2 operator/(Vector2 other) const { return Vector2(x / other.x, y / other.y); }
    constexpr Vector2 operator*(float scale) const { return Vector2(x * scale, y * scale); }
    constexpr Vector2 operator/(float divisor) const { return Vector2(x / divisor, y / divisor); }
    constexpr Vector2 operator-() const { return Vector2(-x, -y); }
    constexpr Vector2& operator+=(Vector2 other) { return *this = *this + other; }
    constexpr bool operator==(const Vector2&) const = default;

    /// The x axis, (1, 0). It points right.
    ///
    /// Use when: naming the x direction on a plane instead of writing (1, 0)
    /// Tags: x axis 2D, unit x, right, Vector2.right, basis vector
    /// Node: none (constant)
    static constexpr Vector2 xAxis() { return Vector2(1, 0); }

    /// The y axis, (0, 1). It points up.
    ///
    /// Use when: naming the y direction on a plane instead of writing (0, 1)
    /// Tags: y axis 2D, unit y, up, Vector2.up, basis vector
    /// Node: none (constant)
    static constexpr Vector2 yAxis() { return Vector2(0, 1); }

    /// The **dot product** of two 2D vectors. It measures how much they point the same way.
    ///
    /// How: x·x' + y·y'. For two vectors of length 1, the result is the cosine of the angle
    /// between them: 1 for the same direction, 0 at right angles, −1 for opposite directions.
    ///
    /// Use when: measuring how much two directions on a plane agree; projecting a point onto a direction
    /// Tags: dot product 2D, inner product, scalar product, Vector2.Dot, cosine of angle
    /// Node: none (2D form of dot)
    constexpr float dot(Vector2 other) const { return x * other.x + y * other.y; }

    /// The **cross product** of two 2D vectors, as one number: the z component of the 3D cross
    /// product of (a.x, a.y, 0) and (b.x, b.y, 0). It is the **signed area** of the parallelogram
    /// the two vectors span.
    ///
    /// How: a.x·b.y − a.y·b.x. Positive when `other` is counterclockwise of this vector (turning
    /// left), negative when clockwise, zero when parallel or opposite. Swapping the inputs flips
    /// the sign. Its absolute value is |a|·|b|·sin(angle). xAxis × yAxis = +1.
    ///
    /// Use when: telling which side of a line a point is on; the sign of a turn in a polygon; the area of a triangle (half of this)
    /// Tags: cross product 2D, perp dot, signed area, wedge product, orientation test, left or right of line, Vector2.Cross
    /// Node: none (2D form of cross)
    constexpr float cross(Vector2 other) const { return x * other.y - y * other.x; }

    /// The squared length, x² + y²: the vector's length without the square root.
    ///
    /// Use when: comparing distances without paying for a square root; avoiding a root of a tiny number
    /// Tags: length squared, squared magnitude, sqrMagnitude, norm squared, dot with self
    /// Node: none (2D form of length)
    constexpr float lengthSquared() const { return dot(*this); }

    /// The vector's **length**: √(x² + y²), Pythagoras' theorem on a plane.
    ///
    /// Use when: measuring how long a 2D vector is or how far a point is from the origin
    /// Tags: length 2D, magnitude, norm, Vector2.magnitude, size of vector
    /// Node: none (2D form of length)
    float length() const { return std::sqrt(lengthSquared()); }

    /// The vector scaled to length 1, still pointing the same way: v / length.
    ///
    /// How: a zero-length vector has no direction, so it comes back unchanged, as (0, 0).
    ///
    /// Use when: turning a 2D direction into a unit direction, such as a tangent of a profile
    /// Tags: normalize 2D, unit vector, normalise, direction only, Vector2.normalized
    /// Node: none (2D form of normalize)
    Vector2 normalized() const {
        float size = length();
        return size > 0 ? *this / size : *this;
    }

    /// The vector turned **90° counterclockwise**, with the same length: (x, y) becomes (−y, x).
    ///
    /// How: `cross(perpendicular())` is |self|², positive, which is what "counterclockwise"
    /// means. For the tangent of a curve running left to right, the result points to the left of
    /// the direction of travel: the usual normal of a profile.
    ///
    /// Use when: finding the normal of a 2D curve from its tangent; offsetting a path to its left side
    /// Tags: perpendicular, rotate 90 degrees, normal 2D, orthogonal, Vector2.Perpendicular, left normal
    /// Node: inside roundCorners, chamferCorners
    constexpr Vector2 perpendicular() const { return Vector2(-y, x); }

    /// The normal on the right of this direction: the vector turned 90° clockwise, (y, −x), with
    /// the same length. It is −perpendicular().
    ///
    /// How: walking a counterclockwise closed path, the right side is the outside, so this is the
    /// outward normal there; it is the side a positive offset moves to (`CubicBezier.offset`).
    ///
    /// Use when: moving a point outward from a counterclockwise outline; the side a positive offset or stroke width goes to
    /// Tags: right normal, clockwise perpendicular, outward normal, rotate -90 degrees
    /// Node: inside roundCorners, chamferCorners
    constexpr Vector2 rightNormal() const { return Vector2(y, -x); }

    /// Whether both components are ordinary numbers: neither infinite nor NaN.
    ///
    /// Use when: checking a point or direction of a profile before building on it
    /// Tags: is finite, finite vector, NaN check, infinity check, valid 2D vector
    /// Node: none (validation)
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y); }

    /// The vector scaled to length 1, computed safely at every scale, or an empty optional when
    /// it has no direction: zero length, or a component that is infinite or NaN.
    ///
    /// How: divide by the largest component magnitude first, so the length is between 1 and √2,
    /// then normalize. Squaring the components directly (`normalized`) underflows to 0 below about
    /// 1e-19 and overflows to infinity above about 1.8e19. Both components are checked for
    /// finiteness first: `max` of a number and NaN can return the number, so testing only the
    /// largest component would let (1, NaN) through.
    ///
    /// Use when: turning a tangent or a chord of any size into a unit direction when it may be zero
    /// Tags: safe normalize, unit vector, direction, scale-safe normalization, normalize or nil, try normalize 2D
    /// Node: none (2D form of normalize)
    std::optional<Vector2> unitDirection() const {
        if (!isFinite()) return std::nullopt;
        float first = std::fabs(x);
        float second = std::fabs(y);
        float largestComponent = second >= first ? second : first; // Swift's max(first, second)
        if (!(largestComponent > 0)) return std::nullopt;
        return (*this / largestComponent).normalized();
    }

    /// The straight-line distance between two points, |other − self|.
    ///
    /// Use when: measuring how far apart two points of a profile are; checking a pick radius on a plane
    /// Tags: distance 2D, Vector2.Distance, length of difference, point distance
    /// Node: inside roundCorners, chamferCorners
    float distance(Vector2 other) const { return (other - *this).length(); }

    /// The point a fraction `amount` of the way from `start` to `end`, along the straight line
    /// between them: start + (end − start) × amount. `amount` is not clamped.
    ///
    /// Use when: sliding a 2D point from one position to another; one step of de Casteljau's algorithm on a profile curve
    /// Tags: lerp 2D, mix, linear interpolation, Vector2.Lerp, blend points, midpoint
    /// Node: none (2D form of blend)
    static constexpr Vector2 linearBlend(Vector2 start, Vector2 end, float amount) {
        return start + (end - start) * amount;
    }

    /// The area of a rectangle whose width is `x` and height is `y`: x · y. Read the vector as a size.
    ///
    /// Use when: turning the size of a grid cell or a window into its area
    /// Tags: area of rectangle, width times height, size area, cell area, x times y
    /// Node: none (rectangle area of a 2D size, not a step)
    constexpr float area() const { return x * y; }

    /// The centre of one cell in a regular grid of rectangles, given the grid's lower-left corner
    /// and the size of one cell.
    ///
    /// How: lowerCorner + (column + ½, row + ½) · cellSize. Cell (0, 0) touches the corner; the
    /// half-cell offset moves from a cell's corner to its middle. Columns count along x from the
    /// left, rows along y from the bottom.
    ///
    /// Use when: sending one ray through each cell of a grid; reading the position of a texel or a pixel centre
    /// Tags: cell centre, pixel centre, texel centre, grid cell middle, half cell offset
    /// Node: none (renderer shading)
    static constexpr Vector2 cellCentre(int column, int row, Vector2 lowerCorner, Vector2 cellSize) {
        return lowerCorner + Vector2(static_cast<float>(column) + 0.5f, static_cast<float>(row) + 0.5f) * cellSize;
    }
};

/// `vector` scaled by `scale` on both axes.
constexpr Vector2 operator*(float scale, Vector2 vector) { return vector * scale; }

} // namespace editor
