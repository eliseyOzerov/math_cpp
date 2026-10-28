// The boolean algorithm: cut every piece at its crossings, keep the fragments where the rule changes
// across them, chain the kept fragments into loops. Ported from graphics_cpp curves/boolean.cpp
// (itself from fountain-ts), in float, on Shape2D; see Boolean2D.h for the overview.
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "Boolean2D.h"
#include "Polygon2D.h"
#include "Shape2D+Pieces.h"

namespace editor {

namespace {

/// A fragment of an input piece: which piece, the parameters it spans on it, and its ends (snapped
/// to the crossings) in its own direction. A reversed fragment starts at the far end of its span.
struct Fragment {
    int group = 0;
    int outline = 0;
    int piece = 0;
    float lowerParameter = 0;
    float upperParameter = 1;
    Vector2 first;
    Vector2 last;
    bool isReversed = false;
};

/// The identity of an input piece.
struct PieceId {
    int group = 0;
    int outline = 0;
    int piece = 0;
};

using GroupPieces = std::vector<std::vector<std::vector<OutlinePiece>>>;

/// Everything the three steps read.
struct BooleanContext {
    GroupPieces pieces;
    /// Each group rebuilt from its straightened pieces, so winding counts and cutting see the same
    /// geometry.
    std::vector<Shape2D> shapes;
    const BooleanRule* rule = nullptr;
    float tolerance = 0;

    const OutlinePiece& source(const Fragment& fragment) const {
        return pieces[fragment.group][fragment.outline][fragment.piece];
    }
};

/// How far outside a line two straight pieces may cross and still count as meeting on it, in
/// tolerances.
constexpr float lineParameterSlackInTolerances = 1;

/// The parameters at which cutting is worth doing: not at the very ends of a piece.
constexpr float smallestCutParameter = 1e-5f;

/// Two cut parameters closer than this are one cut.
constexpr float sameCutParameter = 1e-6f;

/// Lines whose directions differ by less than this sine are parallel for the crossing test.
constexpr float parallelSine = 1e-5f;

/// A fragment shorter than this has no direction to probe sideways from.
constexpr float smallestKeptLength = 1e-6f;

/// Where two straight pieces cross, as the parameter on each; nothing where they are parallel
/// (the caller then cuts each where the other ends on it) or meet outside both by more than the
/// tolerance.
///
/// How: first = a.start + t·r, second = b.start + u·s; t = ((q − p) × s) / (r × s) and
/// u = ((q − p) × r) / (r × s) (Gareth Rees' answer to the line segment intersection problem).
///
/// Conventions: parameters are clamped into 0…1 after the slack test.
///
/// Use when: cutting two straight pieces of an outline
/// Tags: line intersection, segment crossing, cross product method
/// Node: none (internal boolean step)
std::optional<std::pair<float, float>> straightCrossing(const OutlinePiece& first, const OutlinePiece& second, float tolerance) {
    Vector2 firstStart = first.start();
    Vector2 firstSpan = first.end() - firstStart;
    Vector2 secondStart = second.start();
    Vector2 secondSpan = second.end() - secondStart;
    float spread = firstSpan.cross(secondSpan);
    if (std::fabs(spread) < parallelSine * firstSpan.length() * secondSpan.length()) return std::nullopt;
    float firstParameter = (secondStart - firstStart).cross(secondSpan) / spread;
    float secondParameter = (secondStart - firstStart).cross(firstSpan) / spread;
    float firstSlack = lineParameterSlackInTolerances * tolerance / firstSpan.length();
    float secondSlack = lineParameterSlackInTolerances * tolerance / secondSpan.length();
    if (firstParameter < -firstSlack || firstParameter > 1 + firstSlack) return std::nullopt;
    if (secondParameter < -secondSlack || secondParameter > 1 + secondSlack) return std::nullopt;
    return std::make_pair(clamped(firstParameter, 0, 1), clamped(secondParameter, 0, 1));
}

/// The piece a fragment stands for, in its direction of travel. Built along the source piece from
/// the lower parameter to the upper, with its ends moved to the snapped ones, and turned round if
/// the fragment is reversed (its `first` and `last` are already swapped, so they are swapped back
/// first).
OutlinePiece fragmentPiece(const Fragment& fragment, const OutlinePiece& source) {
    Vector2 alongStart = fragment.isReversed ? fragment.last : fragment.first;
    Vector2 alongEnd = fragment.isReversed ? fragment.first : fragment.last;
    OutlinePiece forward = source.isLine ? OutlinePiece::line(alongStart, alongEnd)
                                         : source.portion(fragment.lowerParameter, fragment.upperParameter)
                                               .withEnds(alongStart, alongEnd);
    return fragment.isReversed ? forward.reversed() : forward;
}

/// The pieces cut into fragments wherever they cross each other. Pieces that run along each other
/// are cut at each other's ends, so their fragments match and `keepFragments` can drop the second
/// copy. Fragments shorter than the tolerance are left out.
std::vector<Fragment> cutIntoFragments(const BooleanContext& context, const std::vector<PieceId>& ids) {
    float tolerance = context.tolerance;
    size_t count = ids.size();
    auto pieceOf = [&](size_t index) -> const OutlinePiece& {
        return context.pieces[ids[index].group][ids[index].outline][ids[index].piece];
    };
    std::vector<Bounds2> boxes;
    for (size_t index = 0; index < count; index += 1) boxes.push_back(pieceOf(index).cubic.bounds());
    struct Cut {
        float parameter;
        Vector2 point;
    };
    std::vector<std::vector<Cut>> cuts(count);
    auto addCut = [&](size_t index, float parameter, Vector2 point) {
        if (parameter > smallestCutParameter && parameter < 1 - smallestCutParameter) {
            cuts[index].push_back({parameter, point});
        }
    };
    for (size_t firstIndex = 0; firstIndex < count; firstIndex += 1) {
        for (size_t secondIndex = firstIndex + 1; secondIndex < count; secondIndex += 1) {
            if (!boxes[firstIndex].overlaps(boxes[secondIndex], tolerance)) continue;
            const OutlinePiece& first = pieceOf(firstIndex);
            const OutlinePiece& second = pieceOf(secondIndex);
            bool bothLines = first.isLine && second.isLine;
            std::vector<std::pair<float, float>> hits;
            bool runAlong = false;
            if (bothLines) {
                // Lines through each other's ends meet there, so each is also cut where the other ends on it.
                if (std::optional<std::pair<float, float>> hit = straightCrossing(first, second, tolerance)) {
                    hits.push_back(*hit);
                }
                runAlong = true;
            } else if (std::optional<std::vector<CurveCrossing>> found = first.cubic.crossings(second.cubic)) {
                for (const CurveCrossing& crossing : *found) {
                    hits.emplace_back(crossing.firstParameter, crossing.secondParameter);
                }
                runAlong = hits.empty();
            } else {
                runAlong = true;
            }
            if (runAlong) {
                // Along each other, or only touching at an end: each is cut where the other's ends lie on it.
                for (int side = 0; side < 2; side += 1) {
                    const OutlinePiece& target = side == 0 ? first : second;
                    const OutlinePiece& other = side == 0 ? second : first;
                    size_t targetIndex = side == 0 ? firstIndex : secondIndex;
                    for (Vector2 end : {other.start(), other.end()}) {
                        PathPoint2 nearest = target.nearestTo(end);
                        if (nearest.distance < tolerance) addCut(targetIndex, nearest.parameter, end);
                    }
                }
            }
            for (const auto& [firstParameter, secondParameter] : hits) {
                Vector2 point = Vector2::linearBlend(first.pointAt(firstParameter), second.pointAt(secondParameter), 0.5f);
                addCut(firstIndex, firstParameter, point);
                addCut(secondIndex, secondParameter, point);
            }
        }
    }
    std::vector<Fragment> fragments;
    for (size_t index = 0; index < count; index += 1) {
        const OutlinePiece& piece = pieceOf(index);
        std::vector<Cut>& sorted = cuts[index];
        std::stable_sort(sorted.begin(), sorted.end(), [](const Cut& a, const Cut& b) { return a.parameter < b.parameter; });
        std::vector<Cut> stops{{0, piece.start()}};
        for (size_t cutIndex = 0; cutIndex < sorted.size(); cutIndex += 1) {
            if (cutIndex == 0 || sorted[cutIndex].parameter - sorted[cutIndex - 1].parameter > sameCutParameter) {
                stops.push_back(sorted[cutIndex]);
            }
        }
        stops.push_back({1, piece.end()});
        for (size_t stop = 0; stop + 1 < stops.size(); stop += 1) {
            const Cut& from = stops[stop];
            const Cut& to = stops[stop + 1];
            // A fragment shorter than the tolerance is a crossing seen twice, or a sliver between ends that nearly meet.
            bool isLongEnough = from.point.distance(to.point) > tolerance
                || (!piece.isLine && piece.portion(from.parameter, to.parameter).length() > tolerance);
            if (isLongEnough) {
                fragments.push_back(
                    {ids[index].group, ids[index].outline, ids[index].piece, from.parameter, to.parameter, from.point, to.point,
                     false});
            }
        }
    }
    return fragments;
}

/// Whether the rule holds at `point`: the point's insideness in each group, through the rule.
bool isInsideResult(const BooleanContext& context, Vector2 point) {
    std::vector<bool> insideGroups;
    for (const Shape2D& shape : context.shapes) insideGroups.push_back(shape.contains(point));
    return (*context.rule)(insideGroups);
}

/// The fragments where the rule changes across them, each turned so the inside is on its left (so
/// outer outlines come out counterclockwise). The rule is tested at a point just to the left and a
/// point just to the right of the middle of the fragment. A copy of an edge already kept is dropped.
std::vector<Fragment> keepFragments(const BooleanContext& context, const std::vector<Fragment>& fragments) {
    std::vector<Fragment> kept;
    for (const Fragment& fragment : fragments) {
        // Probed on the piece itself, not its snapped ends: an end snapped to a crossing up to the
        // tolerance away tilts a short fragment enough to put both probes on one side.
        OutlinePiece part = context.source(fragment).portion(fragment.lowerParameter, fragment.upperParameter);
        float length = part.length();
        if (length < smallestKeptLength) continue;
        Vector2 middle = part.pointAt(0.5f);
        Vector2 direction = part.directionAt(0.5f);
        float probeDistance = smallerOf(context.tolerance / 5, length / 8);
        bool isLeftInside = isInsideResult(context, middle + direction.perpendicular() * probeDistance);
        bool isRightInside = isInsideResult(context, middle + direction.rightNormal() * probeDistance);
        if (isLeftInside == isRightInside) continue;
        Fragment turned = fragment;
        if (isRightInside) {
            turned.isReversed = true;
            turned.first = fragment.last;
            turned.last = fragment.first;
        }
        Vector2 turnedMiddle = fragmentPiece(turned, context.source(turned)).pointAt(0.5f);
        bool isCopy = false;
        for (const Fragment& other : kept) {
            if (other.first.distance(turned.first) < context.tolerance && other.last.distance(turned.last) < context.tolerance
                && fragmentPiece(other, context.source(other)).pointAt(0.5f).distance(turnedMiddle) < context.tolerance) {
                isCopy = true;
                break;
            }
        }
        if (!isCopy) kept.push_back(turned);
    }
    return kept;
}

/// A loop of fragments as a closed list of pieces, with the group each came from: neighbours cut
/// from the same piece are put back together, straight pieces that run on in one direction become
/// one line, and pieces that stay within four tolerances of their start (they would not show but
/// read as two corners) are dropped; each piece then starts exactly where the one before ends.
std::pair<std::vector<OutlinePiece>, std::vector<int>> rejoinLoop(const BooleanContext& context, const std::vector<Fragment>& loop) {
    float tolerance = context.tolerance;
    std::vector<Fragment> merged;
    for (const Fragment& fragment : loop) {
        if (!merged.empty()) {
            Fragment& previous = merged.back();
            float previousEnd = previous.isReversed ? previous.lowerParameter : previous.upperParameter;
            float nextStart = fragment.isReversed ? fragment.upperParameter : fragment.lowerParameter;
            if (previous.group == fragment.group && previous.outline == fragment.outline && previous.piece == fragment.piece
                && previous.isReversed == fragment.isReversed && std::fabs(previousEnd - nextStart) < sameCutParameter) {
                if (previous.isReversed) {
                    previous.lowerParameter = fragment.lowerParameter;
                } else {
                    previous.upperParameter = fragment.upperParameter;
                }
                previous.last = fragment.last;
                continue;
            }
        }
        merged.push_back(fragment);
    }
    std::vector<OutlinePiece> pieces;
    std::vector<int> groups;
    float joinTolerance = 0.04f * tolerance;
    for (const Fragment& fragment : merged) {
        OutlinePiece piece = fragmentPiece(fragment, context.source(fragment));
        if (!pieces.empty() && pieces.back().isLine && piece.isLine
            && Segment2{pieces.back().start(), piece.end()}.isNear(pieces.back().end(), joinTolerance, 0.0f)) {
            pieces.back() = OutlinePiece::line(pieces.back().start(), piece.end());
        } else {
            pieces.push_back(piece);
            groups.push_back(fragment.group);
        }
    }
    if (pieces.size() > 2) {
        const OutlinePiece& last = pieces.back();
        const OutlinePiece& first = pieces.front();
        if (last.isLine && first.isLine && Segment2{last.start(), first.end()}.isNear(last.end(), joinTolerance, 0.0f)) {
            OutlinePiece joined = OutlinePiece::line(last.start(), first.end());
            pieces.pop_back();
            pieces[0] = joined;
            groups[0] = groups.back();
            groups.pop_back();
        }
    }
    for (int index = static_cast<int>(pieces.size()) - 1; index >= 0 && pieces.size() > 2; index -= 1) {
        const OutlinePiece& piece = pieces[static_cast<size_t>(index)];
        bool isSmall = piece.start().distance(piece.end()) < 4 * tolerance
            && piece.cubic.startHandle.distance(piece.start()) < 4 * tolerance
            && piece.cubic.endHandle.distance(piece.start()) < 4 * tolerance;
        if (isSmall) {
            pieces.erase(pieces.begin() + index);
            groups.erase(groups.begin() + index);
        }
    }
    return {stitchedPieces(pieces), groups};
}

/// Whether a loop is a sliver: twice its area over its length is under the tolerance, as where two
/// edges graze each other. Four points of every fragment are sampled for the area.
bool isSliver(const BooleanContext& context, const std::vector<Fragment>& loop) {
    std::vector<Vector2> points;
    for (const Fragment& fragment : loop) {
        OutlinePiece piece = fragmentPiece(fragment, context.source(fragment));
        for (float parameter : {0.0f, 0.25f, 0.5f, 0.75f}) points.push_back(piece.pointAt(parameter));
    }
    float length = 0;
    for (size_t index = 0; index < points.size(); index += 1) {
        length += points[index].distance(points[(index + 1) % points.size()]);
    }
    float area = Polygon2D(points).signedArea();
    return 2 * std::fabs(area) / largerOf(length, 1e-9f) < context.tolerance;
}

/// The kept fragments chained end to start into loops, or nothing where a loop does not close. A
/// sliver loop is left out rather than failing the whole outline.
std::optional<TracedBoolean> chainFragments(const BooleanContext& context, const std::vector<Fragment>& kept) {
    float tolerance = context.tolerance;
    std::vector<bool> used(kept.size(), false);
    TracedBoolean result;
    result.shape.fillRule = FillRule::nonzero;
    for (size_t seed = 0; seed < kept.size(); seed += 1) {
        if (used[seed]) continue;
        std::vector<Fragment> loop{kept[seed]};
        used[seed] = true;
        bool isOpen = false;
        for (size_t guard = 0; guard <= kept.size(); guard += 1) {
            Vector2 end = loop.back().last;
            if (loop.size() > 1 && end.distance(loop.front().first) < tolerance) break;
            int best = -1;
            float bestDistance = tolerance;
            for (size_t candidate = 0; candidate < kept.size(); candidate += 1) {
                float distance = kept[candidate].first.distance(end);
                if (!used[candidate] && distance < bestDistance) {
                    best = static_cast<int>(candidate);
                    bestDistance = distance;
                }
            }
            if (best < 0) {
                if (end.distance(loop.front().first) >= tolerance) isOpen = true;
                break;
            }
            used[static_cast<size_t>(best)] = true;
            loop.push_back(kept[static_cast<size_t>(best)]);
        }
        if (isOpen || loop.back().last.distance(loop.front().first) >= tolerance) {
            if (isSliver(context, loop)) continue;
            return std::nullopt;
        }
        if (isSliver(context, loop)) continue;
        auto [pieces, groups] = rejoinLoop(context, loop);
        result.shape.outlines.push_back(pathOfPieces(pieces, true));
        result.sourceGroups.push_back(groups);
    }
    return result;
}

/// The tolerance used when none is given: a thousandth of the largest side of the inputs' bounds,
/// and never under a hundred thousandth of the biggest coordinate, which floats cannot tell apart.
float automaticTolerance(const std::vector<Shape2D>& groups) {
    float extent = 0;
    float magnitude = 0;
    for (const Shape2D& group : groups) {
        if (group.outlines.empty()) continue;
        Bounds2 bounds = group.bounds();
        extent = largerOf(extent, largerOf(bounds.size().x, bounds.size().y));
        for (Vector2 corner : {bounds.minimum, bounds.maximum}) {
            magnitude = largerOf(magnitude, largerOf(std::fabs(corner.x), std::fabs(corner.y)));
        }
    }
    return largerOf(largerOf(1e-3f * extent, 1e-5f * magnitude), 1e-6f);
}

} // namespace

/// The outline where `rule` holds, with where each piece came from, or nothing where a loop does
/// not close.
///
/// How: the pieces of every outline of every group (nearly straight curves, within twice the
/// tolerance, become lines) are cut at their crossings; a fragment is kept when the rule differs
/// between a point just left of it and a point just right of it, and is turned so the inside is on
/// its left; kept fragments are chained into loops, and each loop rejoined into as few pieces as
/// it needs. Each group's insideness at a point is its own fill rule applied to its winding number.
///
///     A ∪ B:  rule = (a || b)           the boundary is every fragment with one side in either
///     A − B:  rule = (a && !b)
///
/// Conventions: y-up plane; outer outlines of the result run counterclockwise and holes clockwise,
/// nonzero rule. `tolerance` (default a thousandth of the biggest extent of the inputs) is the
/// distance under which points count as one: ends are snapped together, and fragments and loops
/// thinner than it are dropped. A shape with no outlines is nothing. Throws a `MathProblem` for a
/// tolerance that is not positive.
///
/// Source: Greiner–Hormann-style clipping is avoided on purpose; this is graphics_cpp's
/// split-classify-chain scheme (curves/boolean.cpp), after fountain-ts.
///
/// Use when: any boolean of outlines the five named operations do not cover; telling which
///   operand each piece of the result came from
/// Tags: boolean operation, path ops, union, difference, clip, CSG 2D, winding rule, trace
/// Node: planned (booleanShapes)
std::optional<TracedBoolean> traceBoolean(
    const std::vector<Shape2D>& groups, const BooleanRule& rule, std::optional<float> tolerance) {
    if (tolerance && !(*tolerance > 0)) throw MathProblem("A boolean's tolerance must be greater than zero.");
    BooleanContext context;
    context.rule = &rule;
    context.tolerance = tolerance ? *tolerance : automaticTolerance(groups);
    std::vector<PieceId> ids;
    for (size_t group = 0; group < groups.size(); group += 1) {
        context.pieces.emplace_back();
        Shape2D straightened;
        straightened.fillRule = groups[group].fillRule;
        for (size_t outline = 0; outline < groups[group].outlines.size(); outline += 1) {
            std::vector<OutlinePiece> pieces = outlinePieces(groups[group].outlines[outline]);
            for (OutlinePiece& piece : pieces) {
                if (piece.isStraightWithin(2 * context.tolerance)) piece = OutlinePiece::line(piece.start(), piece.end());
            }
            // A degenerate piece (a point) leaves nothing to cut or keep.
            std::erase_if(pieces, [](const OutlinePiece& piece) { return piece.isLine && piece.start() == piece.end(); });
            for (size_t index = 0; index < pieces.size(); index += 1) {
                ids.push_back({static_cast<int>(group), static_cast<int>(outline), static_cast<int>(index)});
            }
            if (!pieces.empty()) straightened.outlines.push_back(pathOfPieces(pieces, true));
            context.pieces.back().push_back(pieces);
        }
        context.shapes.push_back(straightened);
    }
    return chainFragments(context, keepFragments(context, cutIntoFragments(context, ids)));
}

/// `traceBoolean` without the sources: the result outlines, or nothing where a loop does not close.
///
/// Use when: a boolean over any number of operands where failing must be noticed
/// Tags: try boolean, boolean or nothing, safe boolean
/// Node: planned (booleanShapes)
std::optional<Shape2D> tryBooleanByRule(const std::vector<Shape2D>& groups, const BooleanRule& rule, std::optional<float> tolerance) {
    std::optional<TracedBoolean> traced = traceBoolean(groups, rule, tolerance);
    if (!traced) return std::nullopt;
    return traced->shape;
}

/// `tryBoolean`, or all the outlines of all the groups unchanged (nonzero rule) where a loop does
/// not close, rather than a broken outline.
///
/// Use when: a boolean in a pipeline that should degrade to "no change" rather than stop
/// Tags: boolean, path ops, fallback
/// Node: planned (booleanShapes)
Shape2D booleanByRule(const std::vector<Shape2D>& groups, const BooleanRule& rule, std::optional<float> tolerance) {
    if (std::optional<Shape2D> result = tryBooleanByRule(groups, rule, tolerance)) return *result;
    Shape2D unchanged;
    unchanged.fillRule = FillRule::nonzero;
    for (const Shape2D& group : groups) {
        unchanged.outlines.insert(unchanged.outlines.end(), group.outlines.begin(), group.outlines.end());
    }
    return unchanged;
}

} // namespace editor
