// The ink: a nib stamped along a stroke's centreline, shaped by the stroke's curves and trimmed at its ends.
//
// `InkStroke::ink` runs in steps: sample the centreline into points (sweeping round sharp corners), run the cut ends
// on and set the planes that trim them, head each point along the stroke, stamp the nib at each, and join neighbouring
// stamps by their hulls (their union is the ink).
//
// Ported from graphics_cpp Curves2D curves/ink (ink.hpp, ink.cpp; fountain-ts/src/ink/ink.ts), in floats. Bit-level
// agreement with the double-precision goldens is not expected: the bisections and rail searches of the original
// decide on the last bit, so the ink's topology (the number of stamps in a sweep) can differ at a tangency.
//
// Decisions:
// - InkEnd (the TS End; Path2D has its own end names) is an enum; a Cut's mode, the caps and the model are enums too.
// - Keys left out are std::nullopt; a heading not set is std::nullopt.
// - The keys that turn the nib (`rotate`) are in degrees, as keys are plain numbers; every other angle is an `Angle`.
// - A plane that trims a hull is a `HalfPlane2` (point on the cut, normal pointing back into the stroke).
#pragma once

#include <optional>
#include <vector>

#include "../Geometry/HalfPlane2.h"
#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"
#include "CubicBezier.h"
#include "Nib.h"
#include "Profile.h"

namespace editor {

/// Which end of a stroke.
enum class InkEnd { start, end };

/// A terminal's kind. `automatic`: cut level on the given levels and otherwise as the caps say; `flat`: trimmed
/// square across the stroke; `nib`: the nib's own end, untrimmed; `round`: a disc as wide as the ink; `level`: cut
/// along the horizontal; `angle`: cut at `Cut::angle` from the horizontal.
enum class CutMode { automatic, flat, nib, round, level, angle };

/// A terminal: its kind, the `angle` of an angled cut, and `taper` scaling the nib by 1 + taper at the end, easing in
/// over `reach` units (150 by default).
///
/// Use when: shaping how a stroke starts or finishes (a flat top, a tapering tail, a level foot)
/// Tags: stroke end, terminal, cut, taper, flare, serif cut
/// Node: planned (inkStroke)
struct Cut {
    CutMode mode = CutMode::automatic;
    std::optional<Angle> angle;
    std::optional<float> taper;
    std::optional<float> reach;
};

/// Each end's cut, either left out.
struct Cuts {
    std::optional<Cut> start;
    std::optional<Cut> end;
};

/// Whether an end with no cut of its own takes the nib's shape (`nib`) or is trimmed square (`butt`).
enum class InkCaps { butt, nib };

/// The pen's contrast model: `broad` is the nib as shaped (thick and thin by direction); `pointed` stamps a disc as
/// wide as the nib's ink across the stroke, so width changes with direction but the end stays round.
enum class InkModel { broad, pointed };

/// How to ink: `caps` for ends with no cut, whether `cuts` are on, the `levels` (heights) that cut a bare end level,
/// the stamps per curve segment (or, with `spacing`, as many as keep them at most that far apart; at least 2) and the
/// contrast `model`.
///
/// Use when: choosing the quality and ends of an inked stroke
/// Tags: ink options, stamps per span, spacing, caps, levels
/// Node: planned (inkStroke)
struct InkOptions {
    InkCaps caps = InkCaps::butt;
    bool cuts = true;
    std::vector<float> levels;
    std::optional<int> stampsPerSpan;
    std::optional<float> spacing;
    InkModel model = InkModel::broad;
};

/// A plane that trims the hulls near one end of the stroke.
struct EndPlane : HalfPlane2 {
    InkEnd end = InkEnd::start;
};

/// The ink of a stroke: `segments` the centreline (with run-ons), `points` it as a polyline with each point's place
/// `shares` (0…1 by length) along the stroke, `widths` the ink's width across there, `stamps` the nib placed there and
/// `frames` the same exactly, `hulls` the convex hulls of neighbouring stamps (their union is the ink), `nodes` the
/// indices where the skeleton's segments meet and `headings` the headings set for a sharp corner's sweep.
///
/// Use when: drawing the ink, or reading its edges for an outline
/// Tags: ink result, stamps, hulls, ink polygons
/// Node: planned (inkStroke)
struct InkResult {
    std::vector<CubicBezier<Vector2>> segments;
    float weight = 0;
    std::vector<EndPlane> planes;
    std::vector<Vector2> points;
    std::vector<float> shares;
    std::vector<float> widths;
    std::vector<std::vector<Vector2>> stamps;
    std::vector<NibFrame> frames;
    std::vector<std::vector<Vector2>> hulls;
    std::vector<int> nodes;
    std::vector<std::optional<Angle>> headings;
    InkCaps caps = InkCaps::butt;
    bool closed = false;
};

/// A stroke as the ink reads it: its chord `chordStart`→`chordEnd` (by default its first segment's start and last
/// segment's end), its cubic `segments`, whether it `closed` on itself, a weight scale, its end `cuts` and the nib's
/// curves along it (keys over 0…1 of the stroke's length): `size` (a factor), `rotate` (degrees), `follow` (added to
/// the nib's), `offset` (fraction of the ink's half-width, −1 puts the left edge on the centreline, 1 the right) and
/// `shapes`.
///
/// Use when: describing a stroke to ink: a letter stem, a swash, a brush line
/// Tags: ink stroke, calligraphic stroke, nib along path, brush path
/// Node: planned (inkStroke)
struct InkStroke {
    std::optional<Vector2> chordStart;
    std::optional<Vector2> chordEnd;
    std::vector<CubicBezier<Vector2>> segments;
    bool closed = false;
    std::optional<float> weight;
    std::optional<Cuts> cuts;
    std::optional<profile::Keys<float>> size, rotate, follow, offset;
    std::optional<profile::Keys<NibShape>> shapes;

    /// The ink of this stroke under a nib: the nib stamped along the centreline, shaped, sized, turned and moved
    /// across by the stroke's curves at each point, and each pair of neighbouring stamps joined by their convex hull.
    ///
    /// How: (1) the centreline is sampled, `stampsPerSpan` points per segment (or by `spacing`); where two segments
    /// meet at a turn over `Nib::cornerTurn`, the corner point is repeated with headings turning from the one
    /// segment's to the next's, in steps of 6°, so the outer edge is the nib's own arc. (2) An end with a cut line
    /// (level, angled, or bare on one of the `levels`) is run on along its tangent as a line until the stamps reach
    /// the cut, and a half-plane trims there; any other end is trimmed square across the stroke unless it is `nib`
    /// or `round`, or bare with nib caps. (3) Each point's heading is the direction to its neighbours, unwound from
    /// the one before so the nib turns smoothly. (4) The nib is stamped at each point: scaled by the weight, `size`,
    /// the end tapers and the weight of its `shapes`, turned by its angle + `rotate` + `follow`·(heading − 90°),
    /// moved across by `offset`. (5) Neighbouring stamps are joined by `ConvexHull2D::merged` and trimmed by the
    /// planes within reach of their end.
    ///
    ///     ●━━━━●━━━━●━━━━●     centreline points; at each, the stamp (a rounded rectangle)
    ///     ▭▭▭▭▭▭▭▭▭▭▭▭▭▭▭     the hull of each pair of stamps is one piece of ink
    ///
    /// Conventions: y-up plane; font units. A stroke with no segments throws a `MathProblem`; so does an angled cut
    /// with no angle. A closed stroke has no ends.
    ///
    /// Use when: drawing a calligraphic or font stroke; getting the ink's polygons to merge into an outline
    /// Tags: ink, nib stroke, calligraphy, stamp and hull, brush stroke, pen
    /// Node: planned (inkStroke)
    InkResult ink(const NibInput& nib = {}, const InkOptions& options = {}) const;
};

} // namespace editor
