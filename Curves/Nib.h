// The nib: a rounded rectangle stamped along a stroke, as a pen leaves ink. This file holds the nib itself (its
// size, corners and turn), the shape it takes at one place along a stroke, the polygon one stamp makes, the width
// of the ink it leaves, and the weight compensation that keeps strokes of one weight looking it. How stamps join
// into ink is in InkStroke.h.
//
// Weight compensation: the nib's size along a stroke, so strokes of one weight look it. Rounds are made a little
// heavier and diagonals a little lighter, by segment: every curve is a round, every line straight or diagonal by its
// direction; where the factor changes at a node the size eases across `ramp` units centred on it.
//
// Ported from graphics_cpp Curves2D curves/nib (nib.hpp, nib.cpp; fountain-ts/src/ink/nib.ts), in floats.
//
// Decisions:
// - The functions are statics of `Nib`, named in full: `Nib::from`, `Nib::stemNib`, `Nib::penOf`, `Nib::radii`...
// - A corner radius is a variant of one number or one per corner; `radii` reads it as four.
// - `NibInput` is a variant: nothing (std::monostate), a number (a round nib that wide), a whole Nib, or a StemPen.
// - The shapes given in part are `PartialNibShape`, their missing fields std::nullopt.
// - Angles are `Angle`; the keys that turn a nib along a stroke (InkStroke) stay in degrees, as keys are numbers.
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <variant>
#include <vector>

#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"
#include "CubicBezier.h"
#include "Profile.h"

namespace editor {

struct Nib;

/// A corner radius as a fraction of the half-side: one number for every corner, or one per corner (counterclockwise
/// from the top right, see `Nib::cornerSigns`).
///
/// Use when: rounding a nib's corners all alike or each its own way
/// Tags: corner radius, rounded rectangle, per-corner radius
/// Node: none (value type)
using CornerRadius = std::variant<float, std::vector<float>>;

/// A pen by its stem: an elliptical nib leaving a vertical stem `weight` wide, `contrast` (0…1) thinner across, turned
/// `stress` counterclockwise.
///
/// Use when: describing a pen the way a type designer does, by stem, contrast and stress
/// Tags: stem pen, stem weight, contrast, stress, type design pen
/// Node: planned (nib)
struct StemPen {
    float weight = 72;
    float contrast = 0;
    Angle stress = Angle::zero();

    bool operator==(const StemPen&) const = default;
};

/// What a nib can be given as: nothing (the default nib), a bare number (a round nib that wide), a nib, or a pen.
///
/// Use when: an option that accepts any way of naming a nib
/// Tags: nib input, nib or number, pen or nib
/// Node: none (value type)
using NibInput = std::variant<std::monostate, float, Nib, StemPen>;

/// A nib's shape at one place: its size and corners (radii as fractions of the half-sides).
///
/// Use when: morphing a nib along a stroke (a calligraphic swell from round to square)
/// Tags: nib shape, stamp shape, rounded rectangle
/// Node: planned (nib)
struct NibShape {
    float width = 72;
    float height = 72;
    CornerRadius radiusX = 1.0f;
    CornerRadius radiusY = 1.0f;

    bool operator==(const NibShape&) const = default;
};

/// A nib placed: its centre and its shape turned `rotation`.
///
/// Use when: reading back exactly where and how big each stamp was
/// Tags: nib frame, placed stamp, stamp pose
/// Node: none (value type)
struct NibFrame : NibShape {
    Vector2 centre;
    Angle rotation = Angle::zero();
};

/// One corner of a shape's rounded rectangle: its signs `signs` along x and y, the centre `arcCentre` of its quarter
/// ellipse and that ellipse's radii `radii` (0 for a square corner).
///
/// Use when: stamping a nib's outline; finding where its straight sides end
/// Tags: corner arc, quarter ellipse, rounded corner
/// Node: none (value type)
struct NibCornerArc {
    Vector2 signs;
    Vector2 arcCentre;
    Vector2 radii;
};

/// A shape given in part: fields left out are 1 (width, height) or the nib's own (radii).
///
/// Use when: listing a few morph targets without repeating every field
/// Tags: partial shape, shape override
/// Node: none (value type)
struct PartialNibShape {
    std::optional<float> width, height;
    std::optional<CornerRadius> radiusX, radiusY;
};

/// Nib shapes given loosely: keys of shapes as they are, or partial shapes spread evenly along the stroke.
using NibShapesInput = std::variant<profile::Keys<NibShape>, std::vector<PartialNibShape>>;

/// How much heavier rounds are and how much lighter diagonals, as ratios of the weight (1 leaves them).
///
/// Use when: making a font's rounds and diagonals look as heavy as its straights
/// Tags: weight compensation, optical correction, overshoot weight
/// Node: planned (nib)
struct WeightCompensation {
    float round = 1;
    float diagonal = 1;
};

/// The nib: a rectangle `width` wide and `height` tall (font units) with corners rounded by `radiusX`, `radiusY`,
/// turned `angle` counterclockwise; `follow` (0…1) turns it with the stroke (0 keeps it at `angle`, 1 turns it
/// with the direction as a ruling pen). Sampling: `pointsPerCorner` points round each corner of the stamp,
/// `stampsPerSpan` stamps along each curve segment. The defaults are a 72 unit disc.
///
/// Use when: a pen for calligraphic or font strokes; a brush with a shape
/// Tags: nib, pen, brush tip, calligraphy, rounded rectangle stamp
/// Node: planned (nib)
struct Nib {
    float width = 72;
    float height = 72;
    CornerRadius radiusX = 1.0f;
    CornerRadius radiusY = 1.0f;
    Angle angle = Angle::zero();
    float follow = 0;
    int pointsPerCorner = 5;
    int stampsPerSpan = 16;

    bool operator==(const Nib&) const = default;

    /// The sharpest turn that still runs on smoothly (20°): past it the centreline's turn between segments is a
    /// corner the nib sweeps round in place (ink), and a rail's turn is a corner an outline keeps sharp.
    ///
    /// Use when: deciding whether two segments join smoothly or at a corner
    /// Tags: corner threshold, sharp turn, smooth join
    /// Node: none (constant)
    static Angle cornerTurn();

    /// The corners in the nib's own frame, counterclockwise from the top right: their signs along x and y.
    ///
    /// Use when: walking round a nib's corners in order
    /// Tags: corner signs, quadrants
    /// Node: none (constant)
    static const std::array<Vector2, 4>& cornerSigns();

    /// A nib input as a whole nib. A bare number is a round nib that wide; a pen is solved by its stem (`stemNib`);
    /// nothing is the default nib.
    ///
    /// Use when: accepting any way of naming a nib and needing the nib
    /// Tags: nib from input, normalize nib
    /// Node: planned (nib)
    static Nib from(const NibInput& input = {});

    /// A pen by its stem: an ellipse of width w and height h = w·(1 − contrast), turned by the stress θ, reaches
    /// sqrt(w²cos²θ + h²sin²θ)/2 each way across a vertical stroke; set equal to weight/2.
    ///
    /// How: with k = 1 − contrast, w = weight / sqrt(cos²θ + k²sin²θ). Contrast is held in 0…0.99.
    ///
    ///     ╭───────╮   w wide, h = k·w tall, turned θ; its upright
    ///     │   ⬭   │   width is exactly `weight`
    ///     ╰───────╯
    ///
    /// Use when: building a nib from a type designer's stem, contrast and stress
    /// Tags: stem nib, pen from stem, ellipse nib, contrast stress
    /// Node: planned (nib)
    static Nib stemNib(const StemPen& pen);

    /// The pen of a nib input by its stem, contrast and stress (the inverse of `stemNib`, its corners left out):
    /// weight is the upright ink width and contrast 1 − height/width, both rounded to 0.01.
    ///
    /// Use when: showing a nib the way a type designer reads it
    /// Tags: pen of nib, stem weight, contrast of nib
    /// Node: planned (nib)
    static StemPen penOf(const NibInput& input = {});

    /// A radius as one per corner. One number is every corner's; a list is read by position, a short list
    /// continues with its first entry, an empty one gives NaN.
    ///
    /// Use when: reading a radius without asking which form it came in
    /// Tags: radii, per corner radius
    /// Node: none (reader)
    static std::array<float, 4> radii(const CornerRadius& radius);

    /// `radii` given back as the one number when all four agree to within 1e-6.
    ///
    /// Use when: tidying radii after blending them
    /// Tags: uniform radius, collapse radii
    /// Node: none (reader)
    static CornerRadius uniform(const CornerRadius& radius);

    /// A shape part of the way (`amount`, 0…1, not clamped) from `first` to `second`: sizes and radii blended
    /// straight, first + (second − first)·amount. A radius given per corner in either is blended corner by corner.
    ///
    /// Use when: easing a nib from one shape to another along a stroke
    /// Tags: morph nib shape, blend shapes, shape mix
    /// Node: planned (nib)
    static NibShape mixShape(const NibShape& first, const NibShape& second, float amount);

    /// The corners of a shape's rounded rectangle, in the nib's own frame, counterclockwise from the top right.
    /// With half-sides a = width/2, b = height/2 and radii clamped to 0…1 (rx·a, ry·b), the corner at signs (sx, sy)
    /// has its quarter ellipse centred at (sx·(a − rx·a), sy·(b − ry·b)).
    ///
    /// Use when: drawing or measuring a rounded rectangle corner by corner
    /// Tags: corner arcs, rounded rectangle corners, quarter ellipses
    /// Node: planned (nib)
    static std::array<NibCornerArc, 4> cornerArcs(const NibShape& shape);

    /// The stamp: the nib's rounded rectangle as a convex polygon about the origin, counterclockwise, turned by
    /// `rotation`. Each rounded corner gives `pointsPerCorner + 1` points along its quarter ellipse (angle 90°·k
    /// to 90°·(k + 1)); a square corner (a radius under 1e-6) gives its one corner point. A corner with a full
    /// radius repeats the point where it meets the next, which the hull drops.
    ///
    ///        ╭────────╮      a rounded rectangle about the origin
    ///        │    ·   │
    ///        ╰────────╯
    ///
    /// Use when: stamping a nib along a stroke; measuring a nib's ink
    /// Tags: stamp polygon, nib outline, rounded rectangle polygon, brush stamp
    /// Node: planned (nib)
    static std::vector<Vector2> stampPolygon(const NibShape& shape, Angle rotation = Angle::zero(), int pointsPerCorner = 5);

    /// Width of the ink a nib leaves on a straight stroke travelling in `direction` (from the x axis): the extent of
    /// the stamp across the stroke, the stamp turned by `angle` plus `follow` of the way from upright to the
    /// stroke's direction (folded to a half turn: a stroke has no way round).
    ///
    /// Use when: how thick a stroke of this nib looks going this way; the thick and thin of calligraphy
    /// Tags: nib width, ink width, stroke width, thick thin, contrast
    /// Node: planned (nib)
    static float nibWidth(const NibInput& input, Angle direction, float scale = 1);

    /// The stem: the ink's width on an upright stroke.
    ///
    /// Use when: reading a pen's weight
    /// Tags: stem width, upright stroke width
    /// Node: planned (nib)
    static float stemWidth(const NibInput& input, float scale = 1);

    /// A size curve over a stroke that makes a broad nib's turn from thick to thin more abrupt (`transition` > 0, a
    /// Didone's hairline reached in a short run) or more gradual (< 0, old style), from −1 to 1; 0 leaves the nib's
    /// own. At each of `samples` shares u the ink's width w (the nib across `direction(u)`, the stroke's direction)
    /// is placed between the nib's thinnest `low` and thickest `high` at x = (w − low)/(high − low), that share pushed
    /// by a power g = 4^transition (toward the nearer extreme for g > 1, toward the middle for g < 1):
    /// y = ½(2x)^g for x < ½, else 1 − ½(2 − 2x)^g; the key is (low + (high − low)·y)/w, the factor taking the width
    /// there. Thick and thin stay as they are; only the run between them changes. nullopt for no transition or a nib
    /// as wide every way.
    ///
    /// Use when: a pen whose thick-thin change should be sharper or softer than its shape gives
    /// Tags: stroke contrast transition, didone, old style, thick thin ramp
    /// Node: planned (nib)
    static std::optional<profile::Keys<float>> transitionKeys(const NibInput& input, float transition,
                                                              const std::function<Angle(float)>& direction,
                                                              int samples = 129);

    /// Nib shapes from loose input: none stays none, keys stay as they are, and partial shapes are filled in (width
    /// and height 1 when left out, radii the nib's own) and spread evenly along the stroke. An empty list gives
    /// empty keys.
    ///
    /// Use when: reading the shapes option of a stroke
    /// Tags: shapes of nib, morph keys, spread shapes
    /// Node: planned (nib)
    static std::optional<profile::Keys<NibShape>> shapesOf(const std::optional<NibShapesInput>& input, const Nib& nib);

    /// The weight factor of one segment under `compensation`: a curve is a round (`round`); a line leaning d degrees
    /// off the nearest axis is 1 + (diagonal − 1)·smoothStep(min(d, 90 − d)/15), so upright and level lines are
    /// untouched and anything past 15° counts fully as diagonal.
    ///
    /// Use when: sizing the nib for each piece of a letter
    /// Tags: segment weight, round heavier, diagonal lighter, optical weight
    /// Node: planned (nib)
    static float segmentWeight(const CubicBezier<Vector2>& segment, const WeightCompensation& compensation);

    /// The nib's size along the stroke `segments` (closed or not) under `compensation`, as keys over its length, easing
    /// over `ramp` units (centred on each node, at most half of either neighbour) where neighbouring segments
    /// differ; nullopt where every segment is 1. A closed stroke eases across its seam to the mean of the two
    /// factors.
    ///
    /// Use when: a font stroke whose rounds and diagonals should look as heavy as its straights
    /// Tags: weight curve, weight compensation keys, optical size along stroke
    /// Node: planned (nib)
    static std::optional<profile::Keys<float>> weightCurve(const std::vector<CubicBezier<Vector2>>& segments, bool closed,
                                                           const WeightCompensation& compensation, float ramp);
};

} // namespace editor
