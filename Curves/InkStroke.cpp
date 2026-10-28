// The ink of a stroke (InkStroke.h). Ported from graphics_cpp curves/ink (ink.cpp), in floats.
// The steps keep the original's order: sample, run the ends on, head, stamp, join.
#include "InkStroke.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "../Geometry/ConvexHull2D.h"
#include "../Geometry/Polyline2.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "Path2D.h"

namespace editor {

namespace {

constexpr float pi = Angle::pi;

/// A cut closer than this to the stroke's own direction would run on without end; it is cut square instead.
const Angle squareCut = Angle::degrees(15);
/// How far a taper or flare eases in from its end, in font units, when the cut gives no `reach`.
constexpr float defaultTaperReach = 150;
/// The run-on's points lie at most this far apart (font units): close enough that the hulls follow the line.
constexpr float runOnStep = 8;
/// The nib's heading turns at most this much between copies when it sweeps round a sharp corner.
const Angle sweepStep = Angle::degrees(6);
/// How near (font units) an end must lie to one of the `levels` to be cut level there.
constexpr float onHeight = 0.5f;

constexpr InkEnd bothEnds[2] = {InkEnd::start, InkEnd::end};
constexpr int slot(InkEnd end) { return end == InkEnd::start ? 0 : 1; }

/// The stroke's curves along it, sorted.
struct Curves {
    std::optional<profile::Keys<float>> size, rotate, follow, offset;
    std::optional<profile::Keys<NibShape>> shapes;
};

/// What the ends ask for: each end's own terminal (none for `automatic` or none), its taper and how far that reaches.
struct Ends {
    std::optional<Cut> terminal[2];
    float taperOf[2] = {0, 0};
    float reachOf[2] = {defaultTaperReach, defaultTaperReach};
    float taper(InkEnd end) const { return taperOf[slot(end)]; }
    float reach(InkEnd end) const { return reachOf[slot(end)]; }
};

Ends endsOf(const InkStroke& stroke) {
    Ends ends;
    for (InkEnd end : bothEnds) {
        std::optional<Cut> cut;
        if (stroke.cuts) cut = end == InkEnd::start ? stroke.cuts->start : stroke.cuts->end;
        int index = slot(end);
        if (cut && cut->mode != CutMode::automatic) ends.terminal[index] = cut;
        ends.taperOf[index] = cut && cut->taper ? *cut->taper : 0;
        // A reach of 0 (or none) means the default.
        ends.reachOf[index] = cut && cut->reach && *cut->reach != 0 ? *cut->reach : defaultTaperReach;
    }
    return ends;
}

/// Furthest a stamp reaches from its centre, for the run-ons and how far a plane trims.
float reachOfStamp(const Nib& nib, const Curves& curves, float scale, const Ends& ends) {
    float diagonal;
    if (curves.shapes) {
        diagonal = -std::numeric_limits<float>::infinity();
        for (const profile::Key<NibShape>& key : *curves.shapes) {
            diagonal = std::max(diagonal, Vector2(key.value.width * nib.width, key.value.height * nib.height).length());
        }
    } else {
        diagonal = Vector2(nib.width, nib.height).length();
    }
    float flare = 1;
    for (InkEnd end : bothEnds) flare = std::max(flare, 1 + ends.taper(end));
    float sizeFactor = 1;
    float offset = 0;
    if (curves.size) for (const profile::Key<float>& key : *curves.size) sizeFactor = std::max(sizeFactor, key.value);
    if (curves.offset) for (const profile::Key<float>& key : *curves.offset) offset = std::max(offset, std::fabs(key.value));
    return diagonal / 2 * scale * sizeFactor * (1 + offset) * flare;
}

/// The angle of a vector, from the x axis.
Angle headingOf(Vector2 direction) { return Angle::fromRadians(std::atan2(direction.y, direction.x)); }

/// An angle wrapped to −π…π.
Angle wrapped(Angle angle) { return Angle::fromRadians(std::atan2(angle.sine(), angle.cosine())); }

struct Sampled {
    std::vector<Vector2> points;
    std::vector<std::optional<Angle>> headings;
    std::vector<int> corners;
};

/// The centreline as points, `stampsPerSpan` per segment (or by `spacing`), and the indices where segments meet.
/// Where the skeleton turns sharply between segments (a pointed or boxed bowl), the nib sweeps round the corner in
/// place: copies of the corner point whose `headings` (directions along the stroke) turn from the one segment's to
/// the next's, so the outer edge is the nib's own arc. The sweep is a span of its own, from one corner index to the
/// next.
Sampled sample(const std::vector<CubicBezier<Vector2>>& segments, int stampsPerSpan, std::optional<float> spacing) {
    Sampled sampled;
    // The end heading of a segment is its last handle's, or its chord's when the handle sits on the end (within 1e-6);
    // not `endDirection`, which falls back to the next handle: that moved the corners found in fonts.
    auto headingAt = [](const CubicBezier<Vector2>& segment, bool atEnd) {
        Vector2 handle = atEnd ? segment.end - segment.endHandle : segment.startHandle - segment.start;
        Vector2 chord = segment.end - segment.start;
        return handle.length() > 1e-6f ? headingOf(handle) : headingOf(chord);
    };
    for (size_t index = 0; index < segments.size(); index += 1) {
        const CubicBezier<Vector2>& segment = segments[index];
        if (index > 0) {
            Angle arriving = headingAt(segments[index - 1], true);
            Angle turn = wrapped(headingAt(segment, false) - arriving);
            int copies = std::fabs(turn.radians) > Nib::cornerTurn().radians ? static_cast<int>(std::ceil(std::fabs(turn.radians) / sweepStep.radians)) : 0;
            int last = static_cast<int>(sampled.points.size()) - 1;
            if (copies > 0) sampled.headings[last] = arriving;
            sampled.corners.push_back(last);
            if (copies > 0) sampled.corners.push_back(last + copies);
            for (int step = 1; step <= copies; step += 1) {
                sampled.points.push_back(sampled.points.back());
                sampled.headings.push_back(arriving + turn * (static_cast<float>(step) / static_cast<float>(copies)));
            }
        }
        int parts = spacing && *spacing != 0 ? std::max(2, static_cast<int>(std::ceil(segment.totalLength() / *spacing))) : stampsPerSpan;
        for (int step = sampled.points.empty() ? 0 : 1; step <= parts; step += 1) {
            sampled.points.push_back(segment.point(static_cast<float>(step) / static_cast<float>(parts)));
            sampled.headings.push_back(std::nullopt);
        }
    }
    return sampled;
}

/// Direction of a cut line, or none for the stroke's own end (square, or round with round ends).
std::optional<Vector2> cutDirection(const std::optional<Cut>& cut) {
    if (!cut || (cut->mode != CutMode::level && cut->mode != CutMode::angle)) return std::nullopt;
    if (cut->mode == CutMode::level) return Vector2(1, 0);
    if (!cut->angle) throw MathProblem("An angled cut needs an angle.");
    return Vector2::fromAngle(*cut->angle);
}

struct RunOns {
    std::vector<EndPlane> planes;
    float reach[2] = {0, 0};
    int lead = 0;
};

/// The ends' run-ons and planes, in place on `segments`, `points`, `shares` and `headings`. A cut end (its own, or
/// bare on one of the `levels`; none when `levels` is null, cuts off) runs the centreline on until the stamps reach
/// the cut line, as a line segment of its own, and a plane trims there; any other end is trimmed square across the
/// stroke unless it is `nib` or `round`, or bare with nib caps. `reach` is how far from its end each plane trims;
/// `lead` the run-on points put before the start. A closed stroke has no ends.
RunOns runOns(const InkStroke& stroke, std::vector<CubicBezier<Vector2>>& segments, std::vector<Vector2>& points,
              std::vector<float>& shares, std::vector<std::optional<Angle>>& headings, const Ends& ends, float big,
              InkCaps caps, const std::optional<std::vector<float>>& levels) {
    Vector2 chordStart = stroke.chordStart ? *stroke.chordStart : segments[0].start;
    Vector2 chordEnd = stroke.chordEnd ? *stroke.chordEnd : segments.back().end;
    Vector2 chord = (chordEnd - chordStart).unitDirection().value_or(Vector2(1, 0));
    RunOns result;
    if (stroke.closed) return result;
    for (InkEnd end : bothEnds) {
        bool isStart = end == InkEnd::start;
        const CubicBezier<Vector2>& segment = isStart ? segments[0] : segments.back();
        Vector2 endPoint = isStart ? segment.start : segment.end;
        // Outward tangent at the end.
        Vector2 outward = (isStart ? segment.start - segment.startHandle : segment.end - segment.endHandle)
                              .unitDirection()
                              .value_or(isStart ? -chord : chord);
        const std::optional<Cut>& terminal = ends.terminal[slot(end)];
        bool hasMode = terminal.has_value();
        CutMode mode = hasMode ? terminal->mode : CutMode::automatic;
        std::optional<Vector2> direction;
        if (!levels) direction = std::nullopt;
        else if (hasMode) direction = cutDirection(terminal);
        else {
            for (float height : *levels) {
                if (std::fabs(endPoint.y - height) < onHeight) { direction = Vector2(1, 0); break; }
            }
        }
        float crossing = direction ? std::fabs(direction->cross(outward)) : 1;
        if (direction && crossing >= squareCut.sine()) {
            float more = big * std::sqrt(1 - crossing * crossing) / crossing + big;
            int steps = std::max(1, static_cast<int>(std::ceil(more / runOnStep)));
            std::vector<Vector2> on;
            for (int step = 1; step <= steps; step += 1) {
                on.push_back(endPoint + outward * (more * static_cast<float>(step) / static_cast<float>(steps)));
            }
            if (isStart) {
                std::reverse(on.begin(), on.end());
                points.insert(points.begin(), on.begin(), on.end());
                shares.insert(shares.begin(), on.size(), 0.0f);
                headings.insert(headings.begin(), on.size(), std::nullopt);
                result.lead = static_cast<int>(on.size());
            } else {
                points.insert(points.end(), on.begin(), on.end());
                shares.insert(shares.end(), on.size(), 1.0f);
                headings.insert(headings.end(), on.size(), std::nullopt);
            }
            // Normal of the cut line, pointing back into the stroke.
            Vector2 normal(-direction->y, direction->x);
            if (normal.dot(outward) > 0) normal = -normal;
            // `on` was reversed in place for a start, so its last is the run-on point nearest the end either way.
            Vector2 edge = on.back();
            if (isStart) segments.insert(segments.begin(), Path2D::evenlyPacedCubic(edge, endPoint));
            else segments.push_back(Path2D::evenlyPacedCubic(endPoint, edge));
            EndPlane plane;
            plane.point = endPoint;
            plane.normal = normal;
            plane.end = end;
            result.planes.push_back(plane);
            result.reach[slot(end)] = more + 2 * big;
        } else if ((hasMode && mode == CutMode::flat) || (!(hasMode && mode == CutMode::nib) && !(hasMode && mode == CutMode::round) && caps != InkCaps::nib)) {
            EndPlane plane;
            plane.point = endPoint;
            plane.normal = -outward;
            plane.end = end;
            result.planes.push_back(plane);
            result.reach[slot(end)] = 2 * big;
        }
    }
    return result;
}

struct StrokeHeading {
    Angle raw;
    Angle unwound;
};

/// Each point's heading along the stroke, `raw` (set by a sweep, or from its neighbours) and `unwound` from the one
/// before, so the nib turns smoothly with the stroke.
std::vector<StrokeHeading> headingsOf(const std::vector<Vector2>& points, const std::vector<std::optional<Angle>>& set, bool closed) {
    int count = static_cast<int>(points.size());
    std::vector<StrokeHeading> result;
    std::optional<float> previous;
    for (int index = 0; index < count; index += 1) {
        Vector2 before = points[index > 0 ? index - 1 : closed ? count - 2 : 0];
        Vector2 after = points[index < count - 1 ? index + 1 : closed ? 1 : count - 1];
        Angle raw = set[index] ? *set[index] : headingOf(after - before);
        float unwound = previous ? raw.radians : raw.halfTurn().radians;
        if (previous) {
            while (unwound - *previous > pi) unwound -= 2 * pi;
            while (unwound - *previous <= -pi) unwound += 2 * pi;
        }
        previous = unwound;
        result.push_back({raw, Angle::fromRadians(unwound)});
    }
    return result;
}

/// What placing a stamp reads.
struct Stamping {
    Nib nib;
    float scale;
    int pointsPerCorner;
    InkModel model;
    Curves curves;
    Ends ends;
    const std::vector<float>& shares;
    float total;
    const std::vector<Vector2>& points;
    const std::vector<StrokeHeading>& headings;
};

struct Placed {
    std::vector<Vector2> stamp;
    NibFrame frame;
    float width;
};

using Discs = std::array<std::optional<float>, 2>;

/// The nib at point `index`: shaped, sized, turned and moved across by the stroke's curves. Terminals along the
/// stroke: a taper (below zero) or flare scales the nib by 1 + `taper` at its end, easing in over its reach; a round
/// end eases the stamp into a disc as wide as the ink across the stroke there (`discs`), over that width. A pointed
/// pen stamps a disc as wide as the nib's ink across, or the `sweep` width round a corner.
Placed place(const Stamping& stamping, size_t index, const Discs& discs, const std::vector<std::optional<float>>& sweep) {
    const Nib& nib = stamping.nib;
    Angle raw = stamping.headings[index].raw;
    Angle unwound = stamping.headings[index].unwound;
    float share = stamping.shares[index];
    auto distanceFromEnd = [&](InkEnd end) { return (end == InkEnd::start ? share : 1 - share) * stamping.total; };
    float scale = profile::keysAt<float>(stamping.curves.size, share, 1.0f) * stamping.scale;
    for (InkEnd end : bothEnds) {
        if (stamping.ends.taper(end) != 0) scale *= 1 + stamping.ends.taper(end) * smoothStep(1 - distanceFromEnd(end) / stamping.ends.reach(end));
    }
    NibShape morph = profile::keysAt<NibShape>(stamping.curves.shapes, share, NibShape{1, 1, nib.radiusX, nib.radiusY},
                                               [](const NibShape& first, const NibShape& second, float amount) { return Nib::mixShape(first, second, amount); });
    NibShape shape{morph.width * nib.width * scale, morph.height * nib.height * scale, morph.radiusX, morph.radiusY};
    for (InkEnd end : bothEnds) {
        float disc = discs[slot(end)].value_or(0);
        if (disc != 0 && !std::isnan(disc)) {
            float amount = smoothStep(1 - distanceFromEnd(end) / disc);
            if (amount > 0) shape = Nib::mixShape(shape, NibShape{disc, disc, 1.0f, 1.0f}, amount);
        }
    }
    float follow = clamped(nib.follow + profile::keysAt<float>(stamping.curves.follow, share, 0.0f), 0, 1);
    Angle rotation = nib.angle + Angle::degrees(profile::keysAt<float>(stamping.curves.rotate, share, 0.0f)) +
                     (unwound - Angle::fromRadians(pi / 2)) * follow;
    std::vector<Vector2> polygon = Nib::stampPolygon(shape, rotation, stamping.pointsPerCorner);
    Vector2 across(-raw.sine(), raw.cosine());
    auto extent = [&](const std::vector<Vector2>& corners) {
        float lowest = std::numeric_limits<float>::infinity();
        float highest = -std::numeric_limits<float>::infinity();
        for (Vector2 corner : corners) {
            lowest = std::min(lowest, corner.dot(across));
            highest = std::max(highest, corner.dot(across));
        }
        return std::pair<float, float>(lowest, highest);
    };
    if (stamping.model == InkModel::pointed) {
        auto [low, high] = extent(polygon);
        float wide = sweep[index] ? *sweep[index] : high - low;
        shape = NibShape{wide, wide, 1.0f, 1.0f};
        rotation = Angle::zero();
        polygon = Nib::stampPolygon(shape, rotation, stamping.pointsPerCorner);
    }
    auto [lowest, highest] = extent(polygon);
    float offset = profile::keysAt<float>(stamping.curves.offset, share, 0.0f);
    // Offset −1 puts the left edge (highest) on the centreline, 1 the right edge (lowest).
    float shift = offset < 0 ? offset * highest : -offset * lowest;
    Vector2 centre = stamping.points[index] + across * shift;
    Placed placed;
    placed.frame = NibFrame{shape, centre, rotation};
    placed.width = highest - lowest;
    for (Vector2 corner : polygon) placed.stamp.push_back(corner + centre);
    return placed;
}

/// Hulls of neighbouring stamps, each trimmed by the planes within their `reach` of its end.
std::vector<std::vector<Vector2>> hullsOf(const std::vector<std::vector<Vector2>>& stamps, const std::vector<Vector2>& points,
                                          const std::vector<EndPlane>& planes, const float reach[2]) {
    size_t count = stamps.size();
    std::vector<float> along = Polyline2{points}.runLengths();
    float length = along[count - 1];
    std::vector<std::vector<Vector2>> hulls;
    for (size_t index = 0; index < std::max<size_t>(1, count - 1); index += 1) {
        std::vector<Vector2> hull = count > 1 ? ConvexHull2D::merged(stamps[index], stamps[index + 1]) : ConvexHull2D::of(stamps[0]);
        for (const EndPlane& plane : planes) {
            float from = plane.end == InkEnd::start ? along[index] : length - along[std::min(count - 1, index + 1)];
            if (from <= reach[slot(plane.end)]) hull = plane.clipped(hull);
            if (hull.size() < 3) break;
        }
        if (hull.size() >= 3) hulls.push_back(hull);
    }
    return hulls;
}

} // namespace

InkResult InkStroke::ink(const NibInput& nibInput, const InkOptions& options) const {
    if (segments.empty()) throw MathProblem("An ink stroke needs at least one segment.");
    Nib nib = Nib::from(nibInput);
    float scale = weight ? *weight : 1;
    int stampsPerSpan = std::max(1, options.stampsPerSpan ? *options.stampsPerSpan : nib.stampsPerSpan);
    int pointsPerCorner = std::max(1, nib.pointsPerCorner);
    Curves curves{profile::sorted<float>(size), profile::sorted<float>(rotate), profile::sorted<float>(follow),
                  profile::sorted<float>(offset), profile::sorted<NibShape>(shapes)};
    Ends ends = endsOf(*this);
    std::vector<CubicBezier<Vector2>> skeleton = segments;
    Sampled sampled = sample(skeleton, stampsPerSpan, options.spacing);
    std::vector<Vector2>& points = sampled.points;
    std::vector<std::optional<Angle>>& headings = sampled.headings;
    size_t sampledCount = points.size();
    std::vector<float> run = Polyline2{points}.runLengths();
    float total = run.back() > 0 ? run.back() : 1;
    std::vector<float> shares;
    for (float travelled : run) shares.push_back(travelled / total);
    std::optional<std::vector<float>> levels;
    if (options.cuts) levels = options.levels;
    RunOns runOn = runOns(*this, skeleton, points, shares, headings, ends, reachOfStamp(nib, curves, scale, ends), options.caps, levels);
    std::vector<StrokeHeading> directions = headingsOf(points, headings, closed);
    Stamping stamping{nib, scale, pointsPerCorner, options.model, curves, ends, shares, total, points, directions};
    size_t count = points.size();
    // A pointed pen sweeping round a sharp corner (a run of points with headings set) eases its disc from the width it
    // arrives with to the width it leaves with: taken from the turning heading, it would swell through the thick
    // directions in between and leave a bump on the corner.
    std::vector<std::optional<float>> sweep(count);
    if (options.model == InkModel::pointed) {
        for (size_t index = 0; index < count; index += 1) {
            if (!headings[index] || (index > 0 && headings[index - 1])) continue;
            size_t last = index;
            while (last + 1 < count && headings[last + 1]) last += 1;
            float arriving = place(stamping, index, {}, sweep).width;
            float leaving = place(stamping, last, {}, sweep).width;
            for (size_t step = index; step <= last; step += 1) {
                float amount = last > index ? static_cast<float>(step - index) / static_cast<float>(last - index) : 0;
                sweep[step] = linearBlend(arriving, leaving, amount);
            }
        }
    }
    Discs discs;
    for (InkEnd end : bothEnds) {
        if (ends.terminal[slot(end)] && ends.terminal[slot(end)]->mode == CutMode::round) {
            discs[slot(end)] = place(stamping, end == InkEnd::start ? 0 : count - 1, {}, sweep).width;
        }
    }
    InkResult result;
    for (size_t index = 0; index < count; index += 1) {
        Placed placed = place(stamping, index, discs, sweep);
        result.stamps.push_back(placed.stamp);
        result.frames.push_back(placed.frame);
        result.widths.push_back(placed.width);
    }
    result.hulls = hullsOf(result.stamps, points, runOn.planes, runOn.reach);
    // The skeleton's own nodes among the points, run-ons aside: where its segments meet.
    result.nodes = {runOn.lead};
    for (int corner : sampled.corners) result.nodes.push_back(runOn.lead + corner);
    result.nodes.push_back(runOn.lead + static_cast<int>(sampledCount) - 1);
    result.segments = skeleton;
    result.weight = nib.width * scale;
    result.planes = runOn.planes;
    result.points = points;
    result.shares = shares;
    result.headings = headings;
    result.caps = options.caps;
    result.closed = closed;
    return result;
}

} // namespace editor
