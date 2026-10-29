// Recognising a drawn stroke as a line, arc, circle, ellipse, polyline or chain of lines and arcs
// (see ShapeRecognition.h).
#include "ShapeRecognition.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "../Geometry/CircleConstruction.h"
#include "../Geometry/Line2.h"
#include "../Geometry/Polyline2.h"
#include "ShapeFits.h"

namespace editor {

namespace {

/// How many even points a stroke is respaced to before it is read: enough for a corner window of
/// a few percent of the length to hold ten points, few enough that the greedy matching stays fast.
constexpr int recognitionPointCount = 256;

/// How many points the greedy matching steps back by when a run is too long to fit.
constexpr int matchingStride = 4;

/// The fewest points a part covers, unless the stretch it comes from is shorter.
constexpr int fewestPartPoints = 6;

/// The largest sweep an arc part may have: just under a whole turn, which is a circle.
constexpr float largestArcSweepDegrees = 350;

/// The direction of the point on a circle at angle φ: (cos φ, sin φ).
Vector2 radialDirection(Angle angle) { return Vector2(angle.cosine(), angle.sine()); }

/// The angle of `point` seen from `centre`, counterclockwise from x.
Angle angleAround(Vector2 centre, Vector2 point) {
    Vector2 offset = point - centre;
    return Angle::fromRadians(std::atan2(offset.y, offset.x));
}

/// `angle` moved by whole turns into −180°…180°.
float wrappedRadians(float radians) {
    float turn = 2 * std::numbers::pi_v<float>;
    return radians - turn * std::round(radians / turn);
}

/// The points from index `first` to `last` inclusive.
std::vector<Vector2> pointsBetween(const std::vector<Vector2>& points, int first, int last) {
    return std::vector<Vector2>(points.begin() + first, points.begin() + last + 1);
}

/// A part matched to a run of points, with how well it fits.
struct MatchedPart {
    ShapePart part;
    float rmsResidual = 0;
    int pointCount = 0;
};

/// The arc through a run of points on `circle`: from the first point's angle, turning by the
/// sum of the small turns between neighbouring points (so a sweep past 180° is kept).
ShapeArc arcAlong(const Circle2& circle, const std::vector<Vector2>& run) {
    float sweep = 0;
    for (size_t index = 1; index < run.size(); index += 1) {
        sweep += Angle::signedTurn(run[index - 1] - circle.centre, run[index] - circle.centre).radians;
    }
    return ShapeArc{circle, angleAround(circle.centre, run.front()), Angle::fromRadians(sweep)};
}

/// How far an arc bows out from its chord, its sagitta: r·(1 − cos(|sweep|/2)), the distance from
/// the chord's midpoint to the arc's middle (more than r once the arc passes a half turn).
float arcBow(const ShapeArc& arc) {
    return arc.circle.radius * (1 - std::cos(std::fabs(arc.sweep.radians) / 2));
}

/// A line, or else an arc, that fits `run` within `allowed` (root mean square), or nothing. An
/// arc that bows less than `straightness` from its chord is no arc to the person drawing (it is a
/// line with a bend at an end, fitted loosely), so it is not taken.
std::optional<MatchedPart> matchedPart(const std::vector<Vector2>& run, float allowed, float straightness) {
    int count = static_cast<int>(run.size());
    std::optional<FittedLine> line = fittedLine(run);
    if (line && line->rmsResidual <= allowed) {
        ShapePart part;
        part.kind = ShapePart::Kind::line;
        part.line = line->span;
        return MatchedPart{part, line->rmsResidual, count};
    }
    std::optional<FittedCircle> circle = fittedCircle(run);
    if (!circle || !(circle->rmsResidual <= allowed)) return std::nullopt;
    ShapeArc arc = arcAlong(circle->circle, run);
    if (std::fabs(arc.sweep.degrees()) > largestArcSweepDegrees || arcBow(arc) < straightness) return std::nullopt;
    ShapePart part;
    part.kind = ShapePart::Kind::arc;
    part.arc = arc;
    return MatchedPart{part, circle->rmsResidual, count};
}

/// Whether two parts are arcs that bend opposite ways (an S-bend).
bool bendOppositeWays(const ShapePart& first, const ShapePart& second) {
    return first.kind == ShapePart::Kind::arc && second.kind == ShapePart::Kind::arc
        && first.arc.sweep.radians * second.arc.sweep.radians < 0;
}

/// Whether `next` can follow `parts` inside one stretch between corners. Inside a stretch the
/// stroke has no corner, so it bends smoothly one way: two lines in a row would meet at a bend
/// that is no corner and no arc (a smooth curve being cut into lines), and an arc bending the
/// other way from the stretch's earlier arcs makes an S. Either is a curve no chain of lines and
/// arcs reads well.
bool followsSmoothly(const std::vector<MatchedPart>& parts, const ShapePart& next) {
    if (parts.empty()) return true;
    if (parts.back().part.kind == ShapePart::Kind::line && next.kind == ShapePart::Kind::line) return false;
    if (next.kind != ShapePart::Kind::arc) return true;
    for (const MatchedPart& earlier : parts) {
        if (bendOppositeWays(earlier.part, next)) return false;
    }
    return true;
}

/// The parts of the stretch `first`…`last` (inclusive) of `points`, matched greedily: from the
/// start, the longest run a line or an arc fits becomes a part, and the next starts where it
/// ends. Nothing when some run fits no part, or a part does not follow smoothly
/// (`followsSmoothly`).
std::optional<std::vector<MatchedPart>> matchedStretch(const std::vector<Vector2>& points, int first, int last,
                                                       float allowed, float straightness) {
    std::vector<MatchedPart> parts;
    int start = first;
    while (start < last) {
        std::optional<MatchedPart> found;
        int foundEnd = last;
        int shortestEnd = std::min(last, start + fewestPartPoints - 1);
        for (int end = last; end >= shortestEnd; end -= matchingStride) {
            found = matchedPart(pointsBetween(points, start, end), allowed, straightness);
            if (found) {
                foundEnd = end;
                break;
            }
        }
        if (!found) return std::nullopt;
        // A short tail left after this part joins it if the two still fit as one.
        if (foundEnd < last && last - foundEnd < fewestPartPoints) {
            std::optional<MatchedPart> whole = matchedPart(pointsBetween(points, start, last), allowed, straightness);
            if (whole) {
                found = whole;
                foundEnd = last;
            }
        }
        if (!followsSmoothly(parts, found->part)) return std::nullopt;
        parts.push_back(*found);
        start = foundEnd;
    }
    return parts;
}

/// The part with its start moved to `point` (an arc's start angle moves to the point's angle,
/// keeping its end).
ShapePart withStart(ShapePart part, Vector2 point) {
    if (part.kind == ShapePart::Kind::line) {
        part.line.start = point;
        return part;
    }
    float endAngle = part.arc.start.radians + part.arc.sweep.radians;
    float newStart = angleAround(part.arc.circle.centre, point).radians;
    float change = wrappedRadians(newStart - part.arc.start.radians);
    part.arc.start = Angle::fromRadians(part.arc.start.radians + change);
    part.arc.sweep = Angle::fromRadians(endAngle - part.arc.start.radians);
    return part;
}

/// The part with its end moved to `point`.
ShapePart withEnd(ShapePart part, Vector2 point) {
    if (part.kind == ShapePart::Kind::line) {
        part.line.end = point;
        return part;
    }
    float endAngle = part.arc.start.radians + part.arc.sweep.radians;
    float change = wrappedRadians(angleAround(part.arc.circle.centre, point).radians - endAngle);
    part.arc.sweep = Angle::fromRadians(part.arc.sweep.radians + change);
    return part;
}

/// The point nearest `target` among `candidates`, if one is within `reach`.
std::optional<Vector2> nearestWithin(const std::vector<Vector2>& candidates, Vector2 target, float reach) {
    std::optional<Vector2> best;
    for (Vector2 candidate : candidates) {
        if (candidate.distance(target) > reach) continue;
        if (!best || candidate.distance(target) < best->distance(target)) best = candidate;
    }
    return best;
}

Line2 lineThrough(const Segment2& segment) { return Line2{segment.start, segment.end - segment.start}; }

/// Where the lines or circles of two parts cross, nearest `drawn` and within `reach` of it.
std::optional<Vector2> crossingNear(const ShapePart& first, const ShapePart& second, Vector2 drawn, float reach) {
    bool firstIsLine = first.kind == ShapePart::Kind::line;
    bool secondIsLine = second.kind == ShapePart::Kind::line;
    std::vector<Vector2> candidates;
    if (firstIsLine && secondIsLine) {
        Line2 firstLine = lineThrough(first.line);
        std::optional<LineMeeting> meeting = firstLine.meets(lineThrough(second.line));
        if (meeting) candidates.push_back(firstLine.origin + firstLine.direction * meeting->alongFirst);
    } else if (firstIsLine) {
        candidates = circleLineCrossings(second.arc.circle, lineThrough(first.line));
    } else if (secondIsLine) {
        candidates = circleLineCrossings(first.arc.circle, lineThrough(second.line));
    } else {
        candidates = circleCircleCrossings(first.arc.circle, second.arc.circle);
    }
    return nearestWithin(candidates, drawn, reach);
}

/// The foot of `point` on the infinite line through `segment`.
Vector2 footOnLine(const Segment2& segment, Vector2 point) {
    Vector2 direction = segment.end - segment.start;
    float squaredLength = direction.dot(direction);
    if (!(squaredLength > 0)) return segment.start;
    return segment.start + direction * ((point - segment.start).dot(direction) / squaredLength);
}

/// Where two smoothly meeting parts join: the foot of the arc's centre on the line (the tangent
/// point of a line and a circle), or halfway between the two ends.
Vector2 smoothJointPoint(const ShapePart& first, const ShapePart& second, float reach) {
    Vector2 halfway = (first.end() + second.start()) * 0.5f;
    if (first.kind == ShapePart::Kind::line && second.kind == ShapePart::Kind::arc) {
        Vector2 foot = footOnLine(first.line, second.arc.circle.centre);
        if (foot.distance(halfway) <= reach) return foot;
    }
    if (first.kind == ShapePart::Kind::arc && second.kind == ShapePart::Kind::line) {
        Vector2 foot = footOnLine(second.line, first.arc.circle.centre);
        if (foot.distance(halfway) <= reach) return foot;
    }
    return halfway;
}

/// The diagonal of the points' bounding box.
float sizeOf(const std::vector<Vector2>& points) {
    Vector2 lowest = points.front();
    Vector2 highest = points.front();
    for (Vector2 point : points) {
        lowest = Vector2(std::fmin(lowest.x, point.x), std::fmin(lowest.y, point.y));
        highest = Vector2(std::fmax(highest.x, point.x), std::fmax(highest.y, point.y));
    }
    return lowest.distance(highest);
}

/// The point at `index` of a chain, wrapping round when closed (the index is then taken modulo
/// the count).
int wrappedIndex(int index, int count) { return ((index % count) + count) % count; }

} // namespace

Vector2 ShapeArc::point(float share) const {
    return circle.centre + radialDirection(Angle::fromRadians(start.radians + sweep.radians * share)) * circle.radius;
}

Vector2 ShapePart::start() const { return kind == Kind::line ? line.start : arc.point(0); }

Vector2 ShapePart::end() const { return kind == Kind::line ? line.end : arc.point(1); }

Vector2 ShapePart::startDirection() const {
    if (kind == Kind::line) return (line.end - line.start).normalized();
    Vector2 tangent = radialDirection(arc.start).perpendicular();
    return arc.sweep.radians < 0 ? tangent * -1.0f : tangent;
}

Vector2 ShapePart::endDirection() const {
    if (kind == Kind::line) return (line.end - line.start).normalized();
    Vector2 tangent = radialDirection(Angle::fromRadians(arc.start.radians + arc.sweep.radians)).perpendicular();
    return arc.sweep.radians < 0 ? tangent * -1.0f : tangent;
}

std::vector<Vector2> evenlySpacedPoints(const std::vector<Vector2>& points, int count, bool closed) {
    if (points.size() < 2 || count < 2) return points;
    Polyline2 chain{points};
    if (closed) chain.points.push_back(points.front());
    std::vector<float> runs = chain.runLengths();
    float total = runs.back();
    if (!(total > 0)) return points;
    float spacing = total / static_cast<float>(closed ? count : count - 1);
    std::vector<Vector2> spaced;
    spaced.reserve(static_cast<size_t>(count));
    for (int index = 0; index < count; index += 1) spaced.push_back(chain.pointAlong(runs, spacing * static_cast<float>(index)));
    if (!closed) spaced.back() = points.back();
    return spaced;
}

std::vector<float> chordTurns(const std::vector<Vector2>& points, int window, bool closed) {
    int count = static_cast<int>(points.size());
    std::vector<float> turns(points.size(), 0);
    if (window < 1) return turns;
    for (int index = 0; index < count; index += 1) {
        int before = index - window;
        int after = index + window;
        if (closed) {
            if (count < 3) break;
            before = wrappedIndex(before, count);
            after = wrappedIndex(after, count);
        } else if (before < 0 || after >= count) {
            continue;
        }
        Vector2 point = points[static_cast<size_t>(index)];
        turns[static_cast<size_t>(index)] = std::fabs(
            Angle::signedTurn(point - points[static_cast<size_t>(before)], points[static_cast<size_t>(after)] - point).radians);
    }
    return turns;
}

std::vector<int> strokeCorners(const std::vector<Vector2>& points, int window, Angle minimumTurn, float sharpness,
                               bool closed) {
    int count = static_cast<int>(points.size());
    std::vector<int> corners;
    if (window < 1 || count < 3) return corners;
    std::vector<float> turns = chordTurns(points, window, closed);
    std::vector<float> halfTurns = chordTurns(points, std::max(1, window / 2), closed);
    for (int index = 0; index < count; index += 1) {
        float turn = turns[static_cast<size_t>(index)];
        if (!(turn >= minimumTurn.radians) || turn <= 0) continue;
        bool largest = true;
        for (int offset = -window; offset <= window && largest; offset += 1) {
            if (offset == 0) continue;
            int neighbour = index + offset;
            if (closed) {
                neighbour = wrappedIndex(neighbour, count);
            } else if (neighbour < 0 || neighbour >= count) {
                continue;
            }
            float other = turns[static_cast<size_t>(neighbour)];
            if (other > turn || (other == turn && offset < 0)) largest = false;
        }
        if (largest && halfTurns[static_cast<size_t>(index)] >= sharpness * turn) corners.push_back(index);
    }
    return corners;
}

namespace {

/// The drawn points of a stroke and how far along it each lies, so that a part found on the
/// respaced points can be refitted on the points that were drawn: respaced points sit on the
/// chords between drawn ones, a little inside a curve, and would shrink every circle.
struct DrawnStroke {
    std::vector<Vector2> points;
    /// How far along the stroke each point lies (`Polyline2::runLengths`).
    std::vector<float> runs;
    /// The stroke's length, back round to the first point when closed.
    float length = 0;
    bool closed = false;

    /// The drawn points from `from` to `to` along the stroke, in order; on a closed stroke the
    /// distances wrap round (a negative `from` starts before the first point).
    std::vector<Vector2> between(float from, float to) const {
        float slack = 1e-4f * length;
        std::vector<std::pair<float, Vector2>> found;
        for (size_t index = 0; index < points.size(); index += 1) {
            for (int turn = closed ? -1 : 0; turn <= (closed ? 1 : 0); turn += 1) {
                float along = runs[index] + static_cast<float>(turn) * length;
                if (along >= from - slack && along <= to + slack) found.push_back({along, points[index]});
            }
        }
        std::sort(found.begin(), found.end(), [](const auto& first, const auto& second) { return first.first < second.first; });
        std::vector<Vector2> run;
        for (const auto& entry : found) run.push_back(entry.second);
        return run;
    }
};

/// The distance from `point` to the line or circle a part lies on (not just to the part). Near
/// a joint this is what tells the parts' points apart: past a tangent point the drawn points lie
/// on the line and off the circle by about d²/2r, and past a corner on the next side's line.
float distanceToCarrier(const ShapePart& part, Vector2 point) {
    if (part.kind == ShapePart::Kind::line) return point.distance(footOnLine(part.line, point));
    return std::fabs(point.distance(part.arc.circle.centre) - part.arc.circle.radius);
}

/// The part refitted on `drawn`, the drawn points of its stretch, when there are enough of them
/// and the same kind of part still fits them within `allowed`; otherwise the part as matched.
MatchedPart refitted(const MatchedPart& matched, const std::vector<Vector2>& drawn, float allowed) {
    MatchedPart result = matched;
    if (matched.part.kind == ShapePart::Kind::line) {
        std::optional<FittedLine> line = drawn.size() >= 2 ? fittedLine(drawn) : std::nullopt;
        if (line && line->rmsResidual <= allowed) {
            result.part.line = line->span;
            result.rmsResidual = line->rmsResidual;
        }
        return result;
    }
    std::optional<FittedCircle> circle = drawn.size() >= 3 ? fittedCircle(drawn) : std::nullopt;
    if (!circle || !(circle->rmsResidual <= allowed)) return result;
    ShapeArc arc = arcAlong(circle->circle, drawn);
    if (arc.sweep.radians * matched.part.arc.sweep.radians <= 0) return result;
    result.part.arc = arc;
    result.rmsResidual = circle->rmsResidual;
    return result;
}

/// A stroke read as a chain of parts: the parts, whether the joint after each is a corner, where
/// each starts and ends in the chain of respaced points, and the root mean square distance of
/// those points from their parts.
struct ChainReading {
    std::vector<MatchedPart> parts;
    std::vector<bool> cornerAfter;
    std::vector<int> startIndex;
    std::vector<int> endIndex;
    float rmsResidual = 0;
};

/// The stretches between `boundaries` of `chain` matched to parts (`matchedStretch`), or nothing
/// when one cannot be. A closed chain cut where the drawing started rather than at a corner
/// (`cutAtCorner` false) has its last and first parts joined into one when one part fits both.
std::optional<ChainReading> readChain(const std::vector<Vector2>& chain, const std::vector<int>& boundaries, bool closed,
                                      bool cutAtCorner, float allowed, float straightness) {
    ChainReading reading;
    for (size_t stretch = 0; stretch + 1 < boundaries.size(); stretch += 1) {
        std::optional<std::vector<MatchedPart>> parts = matchedStretch(chain, boundaries[stretch], boundaries[stretch + 1], allowed, straightness);
        if (!parts) return std::nullopt;
        int start = boundaries[stretch];
        for (size_t index = 0; index < parts->size(); index += 1) {
            const MatchedPart& part = (*parts)[index];
            reading.parts.push_back(part);
            reading.cornerAfter.push_back(index + 1 == parts->size());
            reading.startIndex.push_back(start);
            start += part.pointCount - 1;
            reading.endIndex.push_back(start);
        }
    }
    if (closed && !cutAtCorner && !reading.parts.empty()) {
        // The joint where the drawing started is smooth, and its two parts are one if they fit
        // as one.
        reading.cornerAfter.back() = false;
        int loop = boundaries.back();
        if (reading.parts.size() >= 2 && reading.parts.back().part.kind == reading.parts.front().part.kind) {
            std::vector<Vector2> around = pointsBetween(chain, reading.startIndex.back(), loop);
            std::vector<Vector2> firstRun = pointsBetween(chain, 1, reading.endIndex.front());
            around.insert(around.end(), firstRun.begin(), firstRun.end());
            std::optional<MatchedPart> merged = matchedPart(around, allowed, straightness);
            if (merged && !bendOppositeWays(reading.parts[reading.parts.size() - 2].part, merged->part)) {
                reading.parts.front() = *merged;
                reading.startIndex.front() = reading.startIndex.back() - loop;
                reading.parts.pop_back();
                reading.cornerAfter.pop_back();
                reading.startIndex.pop_back();
                reading.endIndex.pop_back();
            }
        }
    }
    double squaredSum = 0;
    int pointSum = 0;
    for (const MatchedPart& part : reading.parts) {
        squaredSum += static_cast<double>(part.rmsResidual) * part.rmsResidual * part.pointCount;
        pointSum += part.pointCount;
    }
    reading.rmsResidual = static_cast<float>(std::sqrt(squaredSum / std::max(1, pointSum)));
    return reading;
}

/// How much a reading costs, to choose between a whole circle or ellipse and chains matched at
/// different tolerances: one for each part, plus `fitWeight` times its root mean square distance
/// over the tolerance. An extra part is worth it only when it brings the points a quarter of the
/// tolerance closer, so a stadium is read as its two lines and two arcs, not as the ellipse round
/// it or as two loose arcs, while a noisy triangle stays three lines.
constexpr float fitWeight = 4;

/// How many times the parts are refitted on the drawn points nearest to them.
constexpr int refitRounds = 4;

/// The shares of the tolerance the stretches are matched at, loosest first.
constexpr float matchingShares[] = {1.0f, 0.5f, 0.25f, 0.125f};

float readingCost(size_t partCount, float rmsResidual, float allowed) {
    return static_cast<float>(partCount) + fitWeight * rmsResidual / allowed;
}

RecognisedShape freeformShape(bool closed) {
    RecognisedShape shape;
    shape.closed = closed;
    return shape;
}

} // namespace

RecognisedShape recognisedShape(const std::vector<Vector2>& points, const ShapeTolerances& tolerances) {
    DrawnStroke drawn;
    for (Vector2 point : points) {
        if (!point.isFinite()) continue;
        if (!drawn.points.empty() && drawn.points.back() == point) continue;
        drawn.points.push_back(point);
    }
    if (drawn.points.size() < 3) return freeformShape(false);
    float size = sizeOf(drawn.points);
    if (!(size > 0)) return freeformShape(false);
    Polyline2 drawnChain{drawn.points};
    drawn.runs = drawnChain.runLengths();
    float openLength = drawn.runs.back();
    float gap = drawn.points.front().distance(drawn.points.back());
    drawn.closed = gap <= tolerances.closingShare * size && openLength >= 1.5f * size;
    drawn.length = drawn.closed ? openLength + gap : openLength;
    bool closed = drawn.closed;

    std::vector<Vector2> spaced = evenlySpacedPoints(drawn.points, recognitionPointCount, closed);
    int count = static_cast<int>(spaced.size());
    float spacing = drawn.length / static_cast<float>(closed ? count : count - 1);
    int window = std::max(2, static_cast<int>(std::lround(tolerances.cornerWindowShare * static_cast<float>(count))));
    float reach = tolerances.cornerWindowShare * drawn.length;
    float allowed = tolerances.fitShare * size;
    std::vector<int> corners = strokeCorners(spaced, window, tolerances.cornerTurn, tolerances.cornerSharpness, closed);

    // A whole circle or ellipse, for a closed stroke.
    std::optional<RecognisedShape> whole;
    if (closed) {
        float wholeAllowed = corners.empty() ? allowed : allowed / 2;
        std::optional<FittedCircle> circle = fittedCircle(spaced);
        std::optional<FittedEllipse> ellipse;
        if (!(circle && circle->rmsResidual <= wholeAllowed)) ellipse = fittedEllipse(spaced);
        if (circle && circle->rmsResidual <= wholeAllowed) {
            std::optional<FittedCircle> drawnCircle = fittedCircle(drawn.points);
            if (drawnCircle) circle = drawnCircle;
            RecognisedShape shape = freeformShape(true);
            shape.kind = RecognisedKind::circle;
            shape.circle = circle->circle;
            shape.rmsResidual = circle->rmsResidual;
            whole = shape;
        } else if (ellipse && ellipse->rmsResidual <= wholeAllowed) {
            std::optional<FittedEllipse> drawnEllipse = fittedEllipse(drawn.points);
            if (drawnEllipse) ellipse = drawnEllipse;
            RecognisedShape shape = freeformShape(true);
            shape.kind = RecognisedKind::ellipse;
            shape.ellipse = ellipse->ellipse;
            shape.rmsResidual = ellipse->rmsResidual;
            whole = shape;
        }
    }

    // The chain the stretches are cut from: a closed stroke starts at its first corner and runs
    // back to it, so every stretch lies between two corners.
    std::vector<Vector2> chain = spaced;
    std::vector<int> boundaries;
    int offset = 0;
    if (closed) {
        offset = corners.empty() ? 0 : corners.front();
        std::rotate(chain.begin(), chain.begin() + offset, chain.end());
        chain.push_back(chain.front());
        for (int corner : corners) boundaries.push_back(corner - offset);
        if (boundaries.empty()) boundaries.push_back(0);
        boundaries.push_back(count);
    } else {
        boundaries.push_back(0);
        for (int corner : corners) boundaries.push_back(corner);
        boundaries.push_back(count - 1);
    }
    std::optional<ChainReading> reading;
    float readingCostSoFar = 0;
    for (float share : matchingShares) {
        std::optional<ChainReading> candidate = readChain(chain, boundaries, closed, !corners.empty(), allowed * share, allowed);
        if (!candidate || static_cast<int>(candidate->parts.size()) > tolerances.mostParts) continue;
        float cost = readingCost(candidate->parts.size(), candidate->rmsResidual, allowed);
        if (!reading || cost < readingCostSoFar) {
            reading = candidate;
            readingCostSoFar = cost;
        }
    }
    if (whole) {
        float wholeOnSpaced = whole->kind == RecognisedKind::circle ? fittedCircle(spaced)->rmsResidual
                                                                    : fittedEllipse(spaced)->rmsResidual;
        if (!reading || !(readingCostSoFar < readingCost(1, wholeOnSpaced, allowed))) return *whole;
    }
    if (!reading) return freeformShape(closed);

    // Each part refitted on the drawn points nearest to it: those of its stretch, reaching a
    // corner window into its neighbours, that lie no farther from it than from either neighbour.
    // A few rounds, since each fit moves the parts the next assigns points by.
    std::vector<MatchedPart> matched = reading->parts;
    size_t partCount = matched.size();
    for (int round = 0; round < refitRounds; round += 1) {
        std::vector<MatchedPart> refits;
        for (size_t index = 0; index < partCount; index += 1) {
            float from = static_cast<float>(reading->startIndex[index] + offset) * spacing - reach;
            float to = static_cast<float>(reading->endIndex[index] + offset) * spacing + reach;
            std::vector<const ShapePart*> neighbours;
            if (index > 0 || closed) neighbours.push_back(&matched[(index + partCount - 1) % partCount].part);
            if (index + 1 < partCount || closed) neighbours.push_back(&matched[(index + 1) % partCount].part);
            std::vector<Vector2> own;
            for (Vector2 point : drawn.between(from, to)) {
                float distance = distanceToCarrier(matched[index].part, point);
                bool nearest = true;
                for (const ShapePart* neighbour : neighbours) {
                    if (neighbour != &matched[index].part && distanceToCarrier(*neighbour, point) < distance) nearest = false;
                }
                if (nearest) own.push_back(point);
            }
            refits.push_back(refitted(reading->parts[index], own, allowed));
        }
        matched = refits;
    }

    RecognisedShape result = freeformShape(closed);
    for (const MatchedPart& part : matched) result.parts.push_back(part.part);
    size_t jointCount = closed ? result.parts.size() : result.parts.size() - 1;
    std::vector<Vector2> jointPoints;
    for (size_t index = 0; index < jointCount; index += 1) {
        const ShapePart& first = result.parts[index];
        const ShapePart& second = result.parts[(index + 1) % result.parts.size()];
        Vector2 drawnCorner = chain[static_cast<size_t>(wrappedIndex(reading->endIndex[index], static_cast<int>(chain.size())))];
        if (reading->cornerAfter[index]) {
            jointPoints.push_back(crossingNear(first, second, drawnCorner, reach).value_or(drawnCorner));
        } else {
            jointPoints.push_back(smoothJointPoint(first, second, reach));
        }
    }
    for (size_t index = 0; index < jointCount; index += 1) {
        size_t next = (index + 1) % result.parts.size();
        result.parts[index] = withEnd(result.parts[index], jointPoints[index]);
        result.parts[next] = withStart(result.parts[next], jointPoints[index]);
    }
    for (size_t index = 0; index < jointCount; index += 1) {
        const ShapePart& first = result.parts[index];
        const ShapePart& second = result.parts[(index + 1) % result.parts.size()];
        Angle turn = Angle::signedTurn(first.endDirection(), second.startDirection());
        result.joints.push_back(ShapeJoint{jointPoints[index], turn, std::fabs(turn.radians) < tolerances.smoothJointTurn.radians});
    }

    double squaredSum = 0;
    int pointSum = 0;
    bool allLines = true;
    for (const MatchedPart& part : matched) {
        squaredSum += static_cast<double>(part.rmsResidual) * part.rmsResidual * part.pointCount;
        pointSum += part.pointCount;
        if (part.part.kind != ShapePart::Kind::line) allLines = false;
    }
    result.rmsResidual = static_cast<float>(std::sqrt(squaredSum / std::max(1, pointSum)));
    if (result.parts.size() == 1 && !closed) {
        result.kind = allLines ? RecognisedKind::line : RecognisedKind::arc;
    } else {
        result.kind = allLines ? RecognisedKind::polyline : RecognisedKind::linesAndArcs;
    }
    return result;
}

} // namespace editor
