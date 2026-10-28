// A path is an outline drawn with a pen: start somewhere, then draw straight lines, curves and
// arcs one after another, each starting where the last one ended. It is the 2D shape that
// profiles, sweeps, lathes and fills are made from
// (Engine/Math/Paths/Path2D.swift and its extensions +Flattening, +Measurements and +Crossings).
//
// Swift's `private(set) var segments` and `isClosed` are the accessors `segments()` and
// `isClosed()`; building goes through `addLine`, `addCubic`, `addArc` and `close`. Swift's
// `precondition` becomes a thrown `MathProblem` with the same message. The members are
// declared here and defined in Path2D.cpp, Path2D+Flattening.cpp, Path2D+Measurements.cpp,
// Path2D+Crossings.cpp, Path2D+Fitting.cpp, Path2D+Offset.cpp and Path2D+Corners.cpp, one file
// for each Swift file. Swift's `Path2D.OffsetJoin` enum is `Path2D::OffsetJoin`, a small struct
// with the three named constructors `miter(limit)`, `round()` and `bevel()`; closures are
// `std::function`.
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "../Linear/Angle.h"
#include "CubicBezier.h"
#include "../Geometry/Bounds2.h"
#include "../Linear/Vector2.h"

namespace editor {

struct PathCorner;

/// A 2D path: a start point followed by straight lines and cubic Bézier curves, each beginning
/// where the one before ended. Circle arcs are stored as cubics (`addArc`).
///
///           ╭────●────╮         start ● then a curve, a line, a curve
///          ╱          │
///     start●          ●
///          ╲_________╱
///
/// A path is **open** or **closed** (`isClosed`). Closed means a straight line joins the last
/// point back to the start. For the questions that need an inside (`signedArea`,
/// `windingNumber`) an open path is treated as closed by that same line, which is how a fill
/// treats it.
///
/// Conventions: y-up plane; counterclockwise is the positive direction. Build a path with the
/// `add…` functions; the pieces are then read-only.
///
/// Use when: describing a profile, an outline or a region on the plane before sweeping,
///   filling or measuring it
/// Tags: path, polyline, outline, Path2D, SVG path, pen tool, contour, shape, bezier path
/// Node: inside roundCorners, chamferCorners
class Path2D {
public:
    /// One piece of the path, starting at the end of the one before.
    ///
    /// Use when: reading back the pieces of a path; converting a path to another format
    /// Tags: path segment, line segment, curve segment, path element, SVG command
    struct Segment {
        enum class Kind { line, cubic };

        Kind kind = Kind::line;
        /// The two handles of a cubic. Zero for a line.
        Vector2 startHandle;
        Vector2 endHandle;
        /// Where the piece ends: the `to` of a line, the `end` of a cubic.
        Vector2 endPoint;

        bool operator==(const Segment&) const = default;

        /// A straight line to `to`.
        static Segment line(Vector2 to) { return Segment{Kind::line, Vector2(), Vector2(), to}; }

        /// A cubic Bézier curve with these two handles, ending at `end`.
        static Segment cubic(Vector2 startHandle, Vector2 endHandle, Vector2 end) {
            return Segment{Kind::cubic, startHandle, endHandle, end};
        }

        bool isLine() const { return kind == Kind::line; }

        /// Where this piece ends.
        ///
        /// Use when: following the pen along a path; chaining pieces end to start
        /// Tags: segment end, end point, pen position
        /// Node: inside roundCorners, chamferCorners
        Vector2 end() const { return endPoint; }
    };

    /// Where the path begins.
    Vector2 start;

    /// An empty path that begins at `start`.
    explicit Path2D(Vector2 start) : start(start) {}

    bool operator==(const Path2D&) const = default;

    /// The pieces in drawing order. Empty for a path that is just a start point.
    const std::vector<Segment>& segments() const { return segmentList; }

    /// Whether a straight line joins the last point back to `start`.
    bool isClosed() const { return closed; }

    /// Where the pen is now: the end of the last piece, or `start` for an empty path.
    ///
    /// Use when: continuing a path from where it stopped; checking that pieces join
    /// Tags: current point, pen position, last point, path end
    /// Node: inside roundCorners, chamferCorners
    Vector2 currentPoint() const { return segmentList.empty() ? start : segmentList.back().end(); }

    // MARK: Building (Path2D.cpp)

    void addLine(Vector2 to);
    void addCubic(Vector2 startHandle, Vector2 endHandle, Vector2 end);
    void addArc(Vector2 centre, float radius, Angle startAngle, Angle sweep);
    static float arcJoinTolerance(Vector2 centre, float radius);
    void close();

    // MARK: Pieces for the measurements (Path2D.cpp)

    /// A piece of the path together with the point it starts from, so it can be measured on
    /// its own.
    ///
    /// Use when: measuring or walking the pieces of a path one by one
    /// Tags: path edge, segment with start, piece of path
    /// Node: inside roundCorners, chamferCorners
    struct Edge {
        Vector2 start;
        Segment segment;

        bool operator==(const Edge&) const = default;

        std::optional<CubicBezier<Vector2>> cubic() const;
        /// Path2D+Offset.cpp.
        bool hasLength() const;
        float length() const;
        /// Path2D+Corners.cpp.
        Vector2 startDirection() const;
        Vector2 endDirection() const;
        float signedAreaSwept(Vector2 pivot) const;
        int rayCrossings(Vector2 point) const;
    };

    std::vector<Edge> edges() const;
    std::vector<Edge> edgesWithClosingLine() const;
    /// Path2D+Corners.cpp.
    std::vector<Edge> cornerPieces() const;

    // MARK: Flattening (Path2D+Flattening.cpp)

    std::vector<Vector2> flattened(float tolerance) const;

    // MARK: Measurements (Path2D+Measurements.cpp)

    Bounds2 bounds() const;
    float signedArea() const;
    int windingNumber(Vector2 point) const;

    // MARK: Crossings (Path2D+Crossings.cpp)

    std::optional<std::vector<struct PathSelfCrossing>> selfCrossings() const;
    static CubicBezier<Vector2> evenlyPacedCubic(Vector2 start, Vector2 end);

    // MARK: Fitting (Path2D+Fitting.cpp)

    static std::optional<Path2D> fitted(
        const std::vector<Vector2>& points, float tolerance, std::optional<Vector2> startDirection = std::nullopt,
        std::optional<Vector2> endDirection = std::nullopt, std::optional<bool> closed = std::nullopt);

    // MARK: Offset (Path2D+Offset.cpp)

    /// How the offset pieces meet at a corner of the path, on the side where a gap opens.
    ///
    /// Use when: choosing the look of the corners of an outline, a stroke or an inset
    /// Tags: line join, stroke join, miter join, round join, bevel join, corner style
    struct OffsetJoin {
        enum class Kind { miter, round, bevel };

        Kind kind = Kind::bevel;
        /// The miter limit; 0 for the other kinds.
        float limit = 0;

        bool operator==(const OffsetJoin&) const = default;

        /// The offset edges carried on in a straight line until they meet in a point, as long
        /// as that point is at most `limit` times the distance from the corner; a bevel past
        /// it. A square's corner needs √2; SVG's default limit is 4 (about 29°).
        static OffsetJoin miter(float limit) { return OffsetJoin{Kind::miter, limit}; }
        /// An arc of a circle around the corner, of radius the offset distance.
        static OffsetJoin round() { return OffsetJoin{Kind::round, 0}; }
        /// A straight line across the gap.
        static OffsetJoin bevel() { return OffsetJoin{Kind::bevel, 0}; }
    };

    std::optional<Path2D> offset(float distance, OffsetJoin join, float tolerance) const;
    std::optional<Path2D> offset(float startDistance, float endDistance, OffsetJoin join, float tolerance) const;
    void append(const Segment& segment);

    // MARK: Corners (Path2D+Corners.cpp)

    static Angle defaultMinimumCornerTurn();

    /// The share of a piece that one corner may trim away: at most half, so the corners at
    /// its two ends never overlap.
    ///
    /// Use when: keeping two fillets on the same side from overlapping
    /// Tags: fillet clamp, corner overlap, tangent length limit
    /// Node: none (validation)
    static constexpr float largestCornerShare = 0.5f;

    std::vector<PathCorner> corners(Angle minimumTurn = defaultMinimumCornerTurn()) const;
    Path2D filleted(float radius, Angle minimumTurn = defaultMinimumCornerTurn()) const;
    Path2D filleted(Angle minimumTurn, const std::function<float(const PathCorner&)>& radius) const;
    Path2D filleted(const std::function<float(const PathCorner&)>& radius) const;
    Path2D chamfered(float cut, Angle minimumTurn = defaultMinimumCornerTurn()) const;
    Path2D chamfered(Angle minimumTurn, const std::function<float(const PathCorner&)>& cut) const;
    Path2D chamfered(const std::function<float(const PathCorner&)>& cut) const;

private:
    std::vector<Segment> segmentList;
    bool closed = false;

    static bool isSamePointUpToRounding(Vector2 first, Vector2 second);
};

/// A point where a path crosses itself, with the two pieces and the parameter on each.
///
/// Use when: finding where an outline crosses itself before filling or offsetting it
/// Tags: self intersection, self crossing, path intersection, knot
/// Node: profileCrossings
struct PathSelfCrossing {
    /// The index of the first piece in `Path2D::cornerPieces`.
    int firstPieceIndex = 0;
    /// The parameter on the first piece, 0…1.
    float firstParameter = 0;
    /// The index of the second piece (equal to the first for a single cubic's loop, never
    /// smaller).
    int secondPieceIndex = 0;
    /// The parameter on the second piece, 0…1 (larger than the first for a loop).
    float secondParameter = 0;
    /// Where the path crosses itself.
    Vector2 point;

    bool operator==(const PathSelfCrossing&) const = default;
};

/// A corner of a path: a point where the direction jumps from one piece to the next.
///
/// Use when: deciding which corners of an outline to round; labelling the sharp points of
///   a shape
/// Tags: path corner, vertex, sharp point, kink, joint, turn angle
/// Node: inside roundCorners, chamferCorners
struct PathCorner {
    /// The index of the piece that leaves the corner, in `Path2D::cornerPieces` (the edges,
    /// plus the closing line of a closed path). The corner is at that piece's start.
    int outgoingPieceIndex = 0;
    /// Where the corner is.
    Vector2 point;
    /// The unit direction the path arrives in.
    Vector2 incomingDirection;
    /// The unit direction the path leaves in.
    Vector2 outgoingDirection;

    bool operator==(const PathCorner&) const = default;

    Angle turn() const;
    Angle interiorAngle() const;
};

} // namespace editor
