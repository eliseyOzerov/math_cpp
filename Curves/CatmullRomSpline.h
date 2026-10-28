// A Catmull–Rom spline is a smooth curve that passes *through* every point you give it,
// unlike a Bézier curve, whose handles pull the curve without touching it
// (Engine/Math/Curves/CatmullRomSpline.swift).
#pragma once

#include <vector>

#include "../Linear/Vector3.h"
#include "CubicBezier.h"
#include "Curve.h"

namespace editor {

/// A smooth curve through a list of points, using the **centripetal Catmull–Rom**
/// parameterization (alpha = 0.5).
///
/// Between each pair of neighbouring points the spline is one cubic piece. The pieces join
/// with matching position and direction, so the whole curve has no corners.
///
/// Why centripetal: the plain ("uniform") Catmull–Rom spline can overshoot or loop when the
/// points are unevenly spaced. Spending parameter time in proportion to the *square root* of
/// the distance between points (`distance^0.5`, "alpha = 0.5") provably avoids cusps and
/// self-intersections inside a piece, and the curve hugs the points more tightly.
///
/// Design choice: the spline is stored as a list of `CubicBezier` pieces. The paper gives
/// the curve as a blend of lines (Barry–Goldman); converting each piece to Bézier form once,
/// at construction, makes point, derivative and split reuse the already-proven Bézier code
/// instead of a second set of formulas.
///
/// Conventions:
/// - Parameter range is `0...segmentCount`. Piece `i` covers `i...i+1`, so the whole number
///   `i` is exactly the `i`-th point, and the fractional part is the position within the
///   piece. The parameter is *not* distance (use `ArcLength` for that). Each piece takes
///   one unit of parameter however long it is, so the speed can jump at a point even
///   though the direction stays continuous.
/// - Outside the range the first or last piece continues as a polynomial (not clamped).
/// - **End pieces:** the first and last point have no neighbour on one side. We invent a
///   phantom point by reflecting the nearest real neighbour through the end point:
///   `phantomBefore = 2·first − second`, `phantomAfter = 2·last − secondToLast`. The end
///   pieces then behave as if the curve continued straight out of the end, with no extra
///   bending.
/// - **Coincident points:** consecutive points closer than `Tolerance.geometric` are merged
///   into one when the spline is built, since a zero-length piece has no direction (the
///   formula would divide by zero). The curve still passes through every *distinct* point.
///   If all points coincide, the spline is a single motionless piece at that point.
///
/// Sources: Yuksel, Schaefer & Keyser (2011), "Parameterization and applications of
/// Catmull–Rom curves"; Barry & Goldman (1988), "A recursive evaluation algorithm for a
/// class of Catmull–Rom splines".
///
/// Use when: drawing a smooth road or river through points a user placed; a camera path
///   through waypoints
/// Tags: Catmull-Rom, spline, smooth path, curve through points, centripetal, interpolating
///   spline
/// Node: inside curveToMesh, sweep
class CatmullRomSpline : public Curve<CatmullRomSpline, Vector3> {
public:
    /// The points the curve passes through, after merging coincident neighbours.
    std::vector<Vector3> throughPoints;
    /// One cubic piece between each pair of neighbouring `throughPoints`.
    std::vector<CubicBezier<Vector3>> segments;

    /// Builds the spline through `points`, in order. Needs at least 2 points; with fewer it
    /// throws a `MathProblem` (Swift's precondition) saying "A spline needs at least two points.".
    explicit CatmullRomSpline(const std::vector<Vector3>& points);

    bool operator==(const CatmullRomSpline&) const = default;

    /// The parameter values of the spline: from 0 to the number of pieces, one unit per
    /// piece.
    ///
    /// Use when: sampling a road or camera path through its whole length; looping over
    ///   all pieces of a spline
    /// Tags: spline parameter range, spline domain, segment count
    /// Node: none (protocol plumbing)
    ParameterRange parameterRange() const { return ParameterRange{0, static_cast<float>(segments.size())}; }

    Vector3 point(float parameter) const;
    Vector3 firstDerivative(float parameter) const;

private:
    struct SegmentLocation {
        int index;
        float localParameter;
    };

    SegmentLocation segmentLocation(float parameter) const;
    static Vector3 phantomPoint(Vector3 endPoint, Vector3 neighbour);
    static float centripetalInterval(Vector3 start, Vector3 end);
    static CubicBezier<Vector3> segment(Vector3 previous, Vector3 current, Vector3 next, Vector3 following);
};

} // namespace editor
