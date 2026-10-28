// Where a measured path crosses a horizontal or vertical level, another path, and the aperture cut.
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "../Linear/MathProblem.h"
#include "../Linear/NumberText.h"
#include "MeasuredPath2D.h"

namespace editor {

namespace {

/// Crossings closer than this on the plane are one place (a level through a node meets two
/// pieces there).
constexpr float sameCrossingDistance = 1e-5f;

/// Progresses closer than this are one place on the loop.
constexpr float sameProgressDistance = 1e-5f;

/// Pieces shorter than this share of a parameter are dropped from a cut-out stretch.
constexpr float shortestStretch = 1e-6f;

PathPoint2 chosen(const std::vector<PathPoint2>& crossings, CrossingChoice which, const std::string& what) {
    const PathPoint2* place = nullptr;
    if (!crossings.empty()) {
        if (which.kind == CrossingChoice::Kind::first) place = &crossings.front();
        else if (which.kind == CrossingChoice::Kind::last) place = &crossings.back();
        else if (which.index >= 0 && which.index < static_cast<int>(crossings.size())) place = &crossings[static_cast<size_t>(which.index)];
    }
    if (!place) throw MathProblem("The path does not cross " + what + ".");
    return *place;
}

} // namespace

/// Where the path crosses the horizontal line y = `height`, in order along the path.
///
/// How: the roots of (y(t) − height) on each piece (`CubicBezier::parametersAtCoordinate`), in
/// drawing order; a root at a node meeting the previous crossing within 1e-5 is one crossing.
/// `PathPoint2::distance` is 0.
///
/// Conventions: a piece lying along the line crosses it nowhere.
///
/// Use when: finding the width of a vase profile at a height; where a road reaches a level
/// Tags: crossY, horizontal line intersection, level crossing, scanline, path at height
/// Node: planned (pathAtHeight)
std::vector<PathPoint2> MeasuredPath2D::crossingsAtHeight(float height) const {
    std::vector<PathPoint2> crossings;
    for (size_t index = 0; index < pieceList.size(); index += 1) {
        for (float parameter : pieceList[index].cubic.parametersAtCoordinate(CurveAxis::y, height)) {
            Vector2 point = pieceList[index].cubic.point(parameter);
            if (crossings.empty() || point.distance(crossings.back().point) > sameCrossingDistance) {
                crossings.push_back(PathPoint2{static_cast<int>(index), parameter, point, 0});
            }
        }
    }
    return crossings;
}

/// Where the path crosses the vertical line x = `x`, in order along the path (see
/// `crossingsAtHeight`).
///
/// Use when: slicing a profile at a station; the height of a hill at a given distance
/// Tags: crossX, vertical line intersection, station crossing
/// Node: planned (pathAtHeight)
std::vector<PathPoint2> MeasuredPath2D::crossingsAtX(float x) const {
    std::vector<PathPoint2> crossings;
    for (size_t index = 0; index < pieceList.size(); index += 1) {
        for (float parameter : pieceList[index].cubic.parametersAtCoordinate(CurveAxis::x, x)) {
            Vector2 point = pieceList[index].cubic.point(parameter);
            if (crossings.empty() || point.distance(crossings.back().point) > sameCrossingDistance) {
                crossings.push_back(PathPoint2{static_cast<int>(index), parameter, point, 0});
            }
        }
    }
    return crossings;
}

/// The crossing of y = `height` that `which` names (the first by default). Throws a `MathProblem`
/// when the path does not cross it that often.
///
/// Use when: the point where a profile first reaches a height
/// Tags: atY, point at height, first crossing, last crossing
/// Node: planned (pathAtHeight)
PathPoint2 MeasuredPath2D::crossingAtHeight(float height, CrossingChoice which) const {
    return chosen(crossingsAtHeight(height), which, "height " + numberText(height));
}

/// The crossing of x = `x` that `which` names (the first by default). Throws a `MathProblem` when
/// the path does not cross it that often.
///
/// Use when: the point where a profile first reaches a station
/// Tags: atX, point at x, first crossing, last crossing
/// Node: planned (pathAtHeight)
PathPoint2 MeasuredPath2D::crossingAtX(float x, CrossingChoice which) const {
    return chosen(crossingsAtX(x), which, "x " + numberText(x));
}

/// The straight line across the path at y = `height`, from its first crossing to its last, as an
/// open path of one line.
///
/// Use when: the width of a shape at a height, as something to measure or to cut along
/// Tags: chordY, chord at height, width line, cross section line
/// Node: planned (pathAtHeight)
Path2D MeasuredPath2D::chordAtHeight(float height) const {
    Path2D chord(crossingAtHeight(height, CrossingChoice::first()).point);
    chord.addLine(crossingAtHeight(height, CrossingChoice::last()).point);
    return chord;
}

/// The straight line across the path at x = `x`, from its first crossing to its last, as an open
/// path of one line.
///
/// Use when: the height of a shape at a station, as something to measure or to cut along
/// Tags: chordX, chord at x, height line, cross section line
/// Node: planned (pathAtHeight)
Path2D MeasuredPath2D::chordAtX(float x) const {
    Path2D chord(crossingAtX(x, CrossingChoice::first()).point);
    chord.addLine(crossingAtX(x, CrossingChoice::last()).point);
    return chord;
}

/// Where this path crosses `other`, in order along this path.
///
/// How: every pair of pieces is intersected (`CubicBezier::crossings(other)`); a crossing within
/// 1e-5 of one already found is dropped; each is given as a share of length on both paths. Pieces
/// that overlap along a stretch give no crossings (the curves' answer for coincident pieces).
///
/// Use when: cutting a shape with an outline; finding where a road crosses a river
/// Tags: path intersection, crossings, curve intersect, where paths meet
/// Node: planned (pathCrossings)
std::vector<PathCrossing2> MeasuredPath2D::crossings(const MeasuredPath2D& other) const {
    std::vector<PathCrossing2> found;
    for (size_t index = 0; index < pieceList.size(); index += 1) {
        for (size_t otherIndex = 0; otherIndex < other.pieceList.size(); otherIndex += 1) {
            const CubicBezier<Vector2>& first = pieceList[index].cubic;
            std::optional<std::vector<CurveCrossing>> pairs = first.crossings(other.pieceList[otherIndex].cubic);
            if (!pairs) continue;
            for (const CurveCrossing& pair : *pairs) {
                Vector2 point = first.point(pair.firstParameter);
                bool isKnown = std::any_of(found.begin(), found.end(), [&](const PathCrossing2& old) {
                    return old.point.distance(point) < sameCrossingDistance;
                });
                if (isKnown) continue;
                found.push_back(PathCrossing2{
                    point, progress(static_cast<int>(index), pair.firstParameter),
                    other.progress(static_cast<int>(otherIndex), pair.secondParameter)});
            }
        }
    }
    std::stable_sort(found.begin(), found.end(), [](const PathCrossing2& first, const PathCrossing2& second) {
        return first.progress < second.progress;
    });
    return found;
}

/// A closed path cut open on the side facing `angle`: the longest run of it that lies behind a
/// cutting line, as an open path. `offset` is how far the line is from the middle of the path
/// towards its edge, as a share of the half-extent.
///
/// How: with d = (cos angle, sin angle)·metric, the extent of the path along d is lo…hi (sampled,
/// 33 points per piece), and the cut sits at c = (lo + hi)/2 + offset·(hi − lo)/2. The crossings of
/// d·point = c on each piece give the places where the loop meets the cut; between neighbouring
/// crossings (the last wrapping round the start) the run whose middle has d·point < c and that
/// is longest is kept. Offset −1 is the far edge, 0 the middle, 1 the near edge.
///
///           ╭───────╮ ← cut line facing up (angle 90°, offset 0.5)
///         ╱           ╲       the kept run is the part of the loop below the line
///         ╲___________╱
///
/// Conventions: y-up plane; angle counterclockwise from +x. Throws a `MathProblem` for an open
/// path, a cut that meets the path fewer than twice, or one that leaves nothing behind it.
///
/// Use when: opening a ring to make a C-shaped clip; cutting the lid off a closed profile
/// Tags: aperture, open a loop, cut ring, slit, gap in outline
/// Node: planned (pathAperture)
Path2D MeasuredPath2D::aperture(float offset, Angle angle) const {
    if (!pathValue.isClosed()) throw MathProblem("Only a closed path has an aperture.");
    Vector2 direction = Vector2(std::cos(angle.radians), std::sin(angle.radians)) * metricValue;
    auto along = [&](Vector2 point) { return point.dot(direction); };
    float lowest = std::numeric_limits<float>::infinity();
    float highest = -lowest;
    constexpr int samples = 32;
    for (const OutlinePiece& piece : pieceList) {
        for (int step = 0; step <= samples; step += 1) {
            float value = along(piece.pointAt(static_cast<float>(step) / samples));
            lowest = std::min(lowest, value);
            highest = std::max(highest, value);
        }
    }
    float cutAt = (lowest + highest) / 2 + offset * (highest - lowest) / 2;
    std::vector<float> meetings;
    for (size_t index = 0; index < pieceList.size(); index += 1) {
        const CubicBezier<Vector2>& cubic = pieceList[index].cubic;
        for (float parameter : CubicBezier<Vector2>::coordinateRoots(
                 along(cubic.start), along(cubic.startHandle), along(cubic.endHandle), along(cubic.end), cutAt)) {
            float share = progress(static_cast<int>(index), parameter);
            bool isKnown = std::any_of(meetings.begin(), meetings.end(), [&](float old) {
                return std::fabs(old - share) < sameProgressDistance || std::fabs(std::fabs(old - share) - 1) < sameProgressDistance;
            });
            if (!isKnown) meetings.push_back(share);
        }
    }
    if (meetings.size() < 2) throw MathProblem("The cut at offset " + numberText(offset) + " does not open the path.");
    std::stable_sort(meetings.begin(), meetings.end());
    std::optional<std::pair<float, float>> best;
    float longest = 0;
    for (size_t index = 0; index < meetings.size(); index += 1) {
        float from = meetings[index];
        float to = meetings[(index + 1) % meetings.size()];
        float span = to > from ? to - from : 1 - from + to;
        float middle = from + span / 2;
        if (along(pointAt(middle > 1 ? middle - 1 : middle)) < cutAt && span > longest) {
            best = std::make_pair(from, to);
            longest = span;
        }
    }
    if (!best) throw MathProblem("The cut at offset " + numberText(offset) + " leaves nothing.");
    if (best->second > best->first) return slice(best->first, best->second);
    // The run wraps round the start: its end part, then its beginning part.
    bool hasEnd = best->first < 1 - shortestStretch;
    bool hasBeginning = best->second > shortestStretch;
    if (hasEnd && hasBeginning) return slice(best->first, 1).joined(slice(0, best->second));
    return hasEnd ? slice(best->first, 1) : slice(0, best->second);
}

} // namespace editor
