// A path with its lengths measured, so places on it can be named by how far along they are: the
// halfway point of a curve, the stretch between 20% and 60%, the node a third of the way round.
// Ported from graphics_cpp's `Path` (graphics_cpp path.cpp/path.hpp): its `locate`, `progress`, `at`,
// `direction`, `slice`, `divide`, `cut`, `aperture`, `crossY`… and `crossings`.
//
// Graphics_cpp's `Path` was one immutable type that both built and measured; here `Path2D` builds
// and `MeasuredPath2D` measures a copy of it. The pieces are `OutlinePiece`s (lines and cubics held
// as cubics), numbered from 0 in drawing order; a closed path has its closing line as a last
// piece, as graphics_cpp's `close()` added it.
//
// Members are declared here and defined in MeasuredPath2D.cpp (table, locate, progress, nodes),
// MeasuredPath2D+Levels.cpp (crossing a level or another path, aperture) and
// MeasuredPath2D+Cutting.cpp (slice, cut, divide).
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"
#include "Path2D.h"
#include "PathPoint2.h"
#include "Shape2D+Pieces.h"

namespace editor {

/// Which of several crossings to take where a path crosses a level more than once.
///
/// Use when: asking for the first or last place a profile reaches a height
/// Tags: first crossing, last crossing, nth crossing, which intersection
/// Node: planned (pathAtHeight)
struct CrossingChoice {
    enum class Kind { first, last, indexed };

    Kind kind = Kind::first;
    /// The position when `kind` is `indexed`, counting from 0 along the path.
    int index = 0;

    bool operator==(const CrossingChoice&) const = default;

    static CrossingChoice first() { return CrossingChoice{Kind::first, 0}; }
    static CrossingChoice last() { return CrossingChoice{Kind::last, 0}; }
    /// The `position`th crossing along the path, from 0.
    static CrossingChoice at(int position) { return CrossingChoice{Kind::indexed, position}; }
};

/// A place where one path crosses another: the point, and how far along each path it is (shares of
/// length, 0…1).
///
/// Use when: cutting two outlines at each other; finding where a stroke meets a boundary
/// Tags: path intersection, path crossing, curve crossing
/// Node: planned (pathCrossings)
struct PathCrossing2 {
    Vector2 point;
    /// The share of length along this path.
    float progress = 0;
    /// The share of length along the other path.
    float otherProgress = 0;

    bool operator==(const PathCrossing2&) const = default;
};

/// A path with the length of each piece worked out. A **progress** is a share of the whole length,
/// 0 at the start and 1 at the end, so 0.5 is the point halfway along, however the parameters of
/// the pieces are spaced.
///
///     start ●━━━━━━━●━━━━━━━━━━━━━━━●━━━━●  end
///           0     0.2             0.7    1        progress at the nodes; piece lengths 0.2, 0.5, 0.3
///
/// Lengths are measured in a **metric**: what one unit across and one unit up are worth. (1, 1) is
/// true length; (2, 1) makes horizontal runs count double, which is how a stretched drawing is
/// measured in the units it will be shown in. Points and parameters stay in the path's own
/// coordinates.
///
/// Conventions: y-up plane. The table is built once, so the type is a snapshot: change the path
/// by making a new `MeasuredPath2D`. A path of zero length has every progress at its start.
///
/// Use when: placing things at 25% along a road outline; cutting a profile into equal lengths;
///   finding where a shape's side reaches a height
/// Tags: arc length table, progress, normalized length, path parameter by length, measured path
/// Node: planned (measurePath)
class MeasuredPath2D {
public:
    /// Measures `path` in `metric`. Both components of `metric` must be above 0 and finite.
    explicit MeasuredPath2D(Path2D path, Vector2 metric = Vector2(1, 1));

    bool operator==(const MeasuredPath2D& other) const { return pathValue == other.pathValue && metricValue == other.metricValue; }

    const Path2D& path() const { return pathValue; }
    Vector2 metric() const { return metricValue; }
    /// The pieces in drawing order (the closing line last, for a closed path).
    const std::vector<OutlinePiece>& pieces() const { return pieceList; }
    /// The length of each piece in the metric.
    const std::vector<float>& pieceLengths() const { return lengthList; }

    // MARK: Table and places (MeasuredPath2D.cpp)

    float length() const;
    PathPoint2 locate(float progress) const;
    float progress(int pieceIndex, float parameter) const;
    float progress(const PathPoint2& place) const { return progress(place.pieceIndex, place.parameter); }
    Vector2 pointAt(float progress) const;
    Vector2 directionAt(float progress) const;
    Angle chordAngle() const;

    // MARK: Nodes (MeasuredPath2D.cpp)

    std::vector<Vector2> nodes() const;
    float nodeProgress(int nodeIndex) const;
    std::vector<float> nodeDistances() const;
    float progressWithin(int firstPiece, int lastPiece, float progress) const;

    // MARK: Levels and crossings (MeasuredPath2D+Levels.cpp)

    std::vector<PathPoint2> crossingsAtHeight(float height) const;
    std::vector<PathPoint2> crossingsAtX(float x) const;
    PathPoint2 crossingAtHeight(float height, CrossingChoice which = CrossingChoice::first()) const;
    PathPoint2 crossingAtX(float x, CrossingChoice which = CrossingChoice::first()) const;
    Path2D chordAtHeight(float height) const;
    Path2D chordAtX(float x) const;
    std::vector<PathCrossing2> crossings(const MeasuredPath2D& other) const;
    Path2D aperture(float offset, Angle angle = Angle()) const;

    // MARK: Cutting (MeasuredPath2D+Cutting.cpp)

    Path2D slice(float startProgress, float endProgress) const;
    Path2D cut(const std::vector<float>& progresses) const;
    Path2D divided(int count) const;

    // MARK: Same measure, new path (MeasuredPath2D.cpp)

    MeasuredPath2D reversed() const { return MeasuredPath2D(pathValue.reversed(), metricValue); }
    MeasuredPath2D startingAtNode(int nodeIndex) const { return MeasuredPath2D(pathValue.startingAtNode(nodeIndex), metricValue); }
    MeasuredPath2D mapped(const std::function<Vector2(Vector2)>& pointMap) const {
        return MeasuredPath2D(pathValue.mapped(pointMap), metricValue);
    }

private:
    Path2D pathValue;
    Vector2 metricValue;
    std::vector<OutlinePiece> pieceList;
    std::vector<float> lengthList;

    float lengthOfPieceUpTo(int pieceIndex, float parameter) const;
    float parameterAtDistanceOnPiece(int pieceIndex, float distance) const;
    CubicBezier<Vector2> pieceInMetric(int pieceIndex) const;
    float lengthBefore(int pieceIndex) const;
};

} // namespace editor
