// Ready-made paths: soft polylines (each side bowed out or in, each inner corner rounded by a circular
// arc that meets its sides along their tangents), lobes (soft polygons bowed out from a line, from one
// end round to the other: the D, B, P and R of a typeface are lobes off a stem), ovals, knobs and runs.
//
// Why a lobe hangs off a chord: a lobe is generic, so it hangs off the line between its own ends
// rather than off a vertical stem; one off a leaning stem (an italic D) leans with it for free.
// Where the ends do not lie on the line the lobe should hang off (B's lower lobe starts on the upper
// lobe, not on the stem), `axis` names that line instead.
//
// Ported from graphics_cpp's Curves2D shape.hpp/shape.cpp (itself from fountain-ts/src/curves/shape.ts).
// Differences: float, `Path2D` has no metric, so builders that take a `metric` work in it and hand
// back plain coordinates; the TS option objects are the structs below.
#pragma once

#include <optional>
#include <vector>

#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"
#include "CubicBezier.h"
#include "Path2D.h"

namespace editor {

/// How to soften a polyline: `radius` (0…1) is how far each corner is rounded, as a share of the
/// room its sides allow (half a side shared with another rounded corner, all of a side that ends
/// the polyline); `bulge` is each side's sagitta as a share of its length, to the left of the
/// direction of travel (negative bows it right). `sideBulges`, when not empty, gives one bulge per
/// side instead (a missing one counts as 0).
///
/// Use when: describing how round and how bowed a polygon outline should be
/// Tags: softness, corner radius, bulge, soft polygon
/// Node: planned (softPolyline)
struct Softness {
    float radius = 0;
    float bulge = 0;
    std::vector<float> sideBulges;
};

/// A knob's shape: `size` halves across, `length` times that along, set `side` of the curve.
struct KnobShape {
    float size = 1.6f;
    float length = 1;
    float side = 0;
};

/// Where an oval starts.
enum class OvalStart { bottom, right, top, left };

/// An oval's settings: the handle share of its quarters and where it starts.
struct OvalSettings {
    float tension = Path2D::circleQuarterTension();
    OvalStart start = OvalStart::bottom;
};

/// Whether a lobe's `reach` is measured from the line it hangs off, or from the lobe's start.
enum class LobeReach { fromLine, fromStart };

/// A lobe's character and where it sits. Character: `nodes` corners out from the line (1 a point,
/// 2 a box, 3 a half hexagon, …), `radius` (0…1) how round they are (2 nodes at 1 is a half circle),
/// `bulge` how far each side bows out (above 0) or in, as a share of its length; sides along the
/// lobe's top and bottom stay straight. Place: `axis` the direction of the line it is off, pointing
/// from its end to its start (the chord from `b` to `a` by default; 90° is a stem straight up), and
/// `reach` how far out it goes, in units across (the metric's first), from that line or from its start.
struct LobeSettings {
    float reach = 1;
    float nodes = 2;
    float radius = 1;
    float bulge = 0;
    std::optional<Angle> axis;
    LobeReach reachFrom = LobeReach::fromLine;
};

/// Builders for ready-made 2D shapes, all static.
///
/// Use when: drawing letter bowls, pills, rounded polygons and balls on a stem
/// Tags: shapes, soft polygon, lobe, oval, knob, bowl, glyph parts
/// Node: planned (softPolyline)
struct Shapes2D {
    /// A side from `start` to `end` as a cubic, bowed `bulge` of its length to its left.
    ///
    /// How: both handles sit on the chord's thirds, pushed along the chord's left normal by
    /// h = bulge / 0.75 of the chord, which puts the middle 0.75·h = bulge of the chord out
    /// (the sagitta). A bulge of 0 is the straight chord.
    ///
    /// Use when: a gently bowed side of a pill, a bellied edge
    /// Tags: bowed line, sagitta, arc approximation, bulge
    /// Node: planned (bowedSide)
    static CubicBezier<Vector2> bowed(Vector2 start, Vector2 end, float bulge = 0);

    /// Each side's bulge, scaled down where needed so that no inner corner turns past 11/12 of a
    /// half turn.
    ///
    /// How: a bowed side's ends turn atan(4·bulge) towards the side it bows to, so a corner
    /// between sides a and b turns φ + atan(4·bₐ) + atan(4·b_b): bowing in sharpens a corner, and
    /// too far makes the sides cross and the line loop round the tip. Where that exceeds the limit
    /// the two neighbours are scaled by the largest share (found by 40 bisections) that stays within it.
    ///
    /// Conventions: `bulges` holds one entry per side, so `points.size() - 1` (none for no points);
    /// otherwise a `MathProblem` is thrown.
    ///
    /// Use when: bowing a polygon's sides without the line looping at sharp tips
    /// Tags: clamp bulge, corner turn limit, loop avoidance
    /// Node: inside softPolyline
    static std::vector<float> fitBulges(const std::vector<Vector2>& points, const std::vector<float>& bulges);

    /// `path` with every piece bowed `amount` of its chord away from `from` (below 0 towards it).
    ///
    /// How: both handles of a piece are pushed along the chord's normal by amount/0.75 of the
    /// chord (as `bowed`), the sign chosen so the piece swells away from `from`; nodes and turns
    /// at them hold while the runs between swell. Lines become cubics. A piece that comes back to
    /// its start (no chord) is kept as it is; so is the whole path for an amount of 0. A closed
    /// path's closing line is bowed too.
    ///
    /// Use when: fattening or hollowing a drawn outline; belly on a stem
    /// Tags: bend, bow path, inflate, bulge path
    /// Node: planned (bendPath)
    static Path2D bend(const Path2D& path, float amount, Vector2 from);

    /// A closed knob on the end at `point` that leaves along `direction` (outward), the curve
    /// `half` wide there: an ellipse `size` halves across and `length` times that along
    /// `direction`, its centre set `side` of the curve (+1 its left, -1 its right, 0 on it) by its
    /// radius less `half`, so one edge runs on from the curve's edge on the other side and it
    /// swells towards `side`, as a c's ball hangs into its counter.
    ///
    /// How: an oval of radii (length·r, r), r = size·half, drawn about the origin and moved into
    /// the frame (direction, left normal) at the centre point + normal·sign·(r − half).
    ///
    /// Conventions: `direction` is expected to be a unit vector.
    ///
    /// Use when: ball terminals of letters, finial of a stem
    /// Tags: knob, terminal, ball, ellipse at end
    /// Node: planned (knob)
    static std::vector<CubicBezier<Vector2>> knob(
        Vector2 point, Vector2 direction, float half, const KnobShape& shape = KnobShape());

    /// A curve run on from `point` along `direction` for `length`: a circular arc turning `turn`
    /// on the way (positive to the left of `direction`), straight for 0. One cubic, so keep `turn`
    /// within about 120°.
    ///
    /// How: radius R = length/θ; handle k = (4/3)·tan(θ/4)·R (Riškus 2006); the end lies at
    /// (R·sinθ, R·(1 - cosθ)) in the frame (direction, left normal). Under 1e-6 rad it is straight,
    /// with handles at the thirds.
    ///
    /// Use when: tails and exits of strokes that curve away
    /// Tags: run on, arc of given length, tail, exit stroke
    /// Node: planned (runOn)
    static CubicBezier<Vector2> runOn(Vector2 point, Vector2 direction, float length, Angle turn = Angle());

    /// The polyline through `points` (open, its ends kept sharp) as cubics, its sides bowed (as far
    /// as its corners allow, see `fitBulges`) and inner corners rounded. Fewer than two points give none.
    ///
    /// How: each side is `bowed`; a corner at node i takes radius·min(room) off both sides, the
    /// room being half a side (a whole side at the polyline's ends), and is filled by
    /// `CubicBezier::arcCubic` between the two cut points along the sides' tangents.
    ///
    /// Use when: rounded polygons, pills with bowed sides
    /// Tags: soft polyline, rounded polygon, rounded polyline, fillet polygon
    /// Node: planned (softPolyline)
    static std::vector<CubicBezier<Vector2>> softPolyline(
        const std::vector<Vector2>& points, const Softness& softness = Softness());

    /// The polyline through `points` made soft (see `softPolyline`) as an open path. Worked in
    /// `metric`, so a corner in a narrow box is round in font units. Throws a `MathProblem` for
    /// fewer than two points.
    ///
    /// Why not poly then smooth: smoothing a polyline's corner part of the way leaves a kink in
    /// it, and all the way turns it on handles a third of each side long, so the sides between bow
    /// and a long upright leans; an arc sized by its own radius keeps the turn smooth and the sides straight.
    ///
    /// Use when: rounded outlines whose axes are not equally scaled
    /// Tags: soft path, rounded outline, metric
    /// Node: planned (softPolyline)
    static Path2D soft(
        const std::vector<Vector2>& points, const Softness& softness = Softness(), Vector2 metric = Vector2(1, 1));

    /// An ellipse in the box from (firstX, firstY) to (secondX, secondY), closed, running counter-clockwise from its
    /// bottom or `settings.start`. Four quarters (`Path2D::addQuarter`), the first leaving
    /// horizontally from the bottom or top and vertically from the sides.
    ///
    /// Use when: ellipses and circles with a chosen seam
    /// Tags: oval, ellipse, circle, superellipse
    /// Node: planned (oval)
    static Path2D oval(float firstX, float firstY, float secondX, float secondY, const OvalSettings& settings = OvalSettings());

    /// A lobe from `start` round to `end`, bowing out to the right of the line from `end` to `start`, measured
    /// with `metric` (so corners stay round in its units). The line runs at `axis` through
    /// whichever end lies further back; the lobe's box runs out from it to `reach` and between the
    /// ends along it, and its corners sit where rays from the box's middle on the line, spread
    /// evenly from along the axis round to against it, meet the box. Throws a `MathProblem` when
    /// the lobe has no room.
    ///
    /// Use when: bowls of D, B, P, R; half-round ends
    /// Tags: lobe, bowl, half pill, soft polygon off a line
    /// Node: planned (lobe)
    static Path2D lobe(
        Vector2 start, Vector2 end, const LobeSettings& settings = LobeSettings(), Vector2 metric = Vector2(1, 1));

    /// An open or closed `Path2D` through these cubics, each starting where the last ended (the
    /// first cubic's start begins the path). Closing adds no piece. No cubics give a path at the origin.
    ///
    /// Use when: turning builder output into a path
    /// Tags: path from cubics, assemble path
    /// Node: none (plumbing)
    static Path2D pathOfCubics(const std::vector<CubicBezier<Vector2>>& cubics, bool closed);

    /// The pieces of `path` as cubics (a line as a cubic with handles at its thirds), with the
    /// closing line of a closed path whose pen is not back at the start.
    ///
    /// Use when: treating every piece of a path alike
    /// Tags: path pieces as cubics, lines as cubics
    /// Node: none (plumbing)
    static std::vector<CubicBezier<Vector2>> cubicsOf(const Path2D& path);
};

} // namespace editor
