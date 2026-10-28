// The outline of an inked stroke: the ink as a closed contour of cubics and lines. The two edges of the ink (the
// rails) are found where the stamps' envelope touches each stamp, trimmed by the end planes, untangled, and fitted.
//
// The anchors of each rail follow a plan (pieces per skeleton span). A trimmed end is a straight line along its
// plane; an untrimmed one is the end stamp's far side, in pieces of equal length. The contour runs along the left
// rail, across the end, back along the right rail and across the start: clockwise with y up.
//
// Ported from graphics_cpp curves/outline (outline.hpp, outline.cpp; fountain-ts/src/ink/outline.ts), in
// floats. Bit-level agreement with the double-precision goldens is not expected: the rail searches decide on the last
// bit, so the plan (and so the number of pieces) can differ at a tangency, and `planKey` strings can differ with it.
//
// Decisions:
// - The functions are statics of `InkOutline`: `InkOutline::plan`, `sharedPlan`, `outline` and `planKey`.
// - A plan's end is `std::nullopt` for a line, else its piece count.
// - The contours come as `Path2D`s (closed) and a `Shape2D`; the original's `Contour` list is that shape's outlines.
// - Distances that were 1e-9 in double precision are 1e-4 here (font units are around a thousand to the em).
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "InkStroke.h"
#include "Path2D.h"
#include "Shape2D.h"

namespace editor {

/// The fitting's settings: the `tolerance` (font units) a piece may stray from its rail, and the `mostPieces` a span or
/// run may take.
///
/// Use when: trading an outline's size against its accuracy
/// Tags: outline tolerance, fit tolerance, most pieces, outline quality
/// Node: planned (inkOutline)
struct InkOutlineSettings {
    float tolerance = 1;
    int mostPieces = 6;
};

/// Each end's runs between the nib's sharp corners: 'l' for a straight run, 'c' for a rounded one.
struct InkOutlineRuns {
    std::string start;
    std::string end;

    bool operator==(const InkOutlineRuns&) const = default;
};

/// The plan: pieces per skeleton span on each rail, and per rounded run of each end (`std::nullopt`: a line, where a
/// plane trims it).
///
/// Use when: making several inks of one stroke (a weight family) outline with the same number of pieces
/// Tags: outline plan, pieces per span, anchors, interpolation compatible outlines
/// Node: planned (inkOutline)
struct InkOutlinePlan {
    std::vector<int> left;
    std::vector<int> right;
    std::optional<int> start;
    std::optional<int> end;
    std::optional<InkOutlineRuns> runs;

    bool operator==(const InkOutlinePlan&) const = default;
};

/// The worst fit error in each skeleton span of each rail, and at each end.
struct InkOutlineErrors {
    std::vector<float> left;
    std::vector<float> right;
    float start = 0;
    float end = 0;
};

/// An outline: `contour` (the first of `contours`, which hold a closed stroke's two rings), the `plan` it was fitted
/// to, its `anchors` count (pieces over all contours) and its worst `error` (`errors` has it by place).
///
/// Use when: filling the ink as a shape; merging it with other shapes; comparing fits
/// Tags: ink outline, stroke outline, outline result, fitted contour
/// Node: planned (inkOutline)
struct InkOutlineResult {
    Path2D contour{Vector2()};
    Shape2D contours;
    InkOutlinePlan plan;
    int anchors = 0;
    float error = 0;
    InkOutlineErrors errors;
};

/// Turning an `InkResult` into its outline.
struct InkOutline {
    /// The plan an inked stroke needs alone: pieces per skeleton span on each rail, and per rounded run of each end (a
    /// line where a plane trims it), split until every piece is within `tolerance` or has `mostPieces`.
    ///
    /// How: start with one piece per span (two per rounded end run), fit the outline, add a piece to every place
    /// whose worst error is over the tolerance, and repeat up to `mostPieces` rounds.
    ///
    /// Conventions: font units; spans are the stroke's skeleton segments (`InkResult::nodes`).
    ///
    /// Use when: knowing how many pieces an ink's outline needs; fitting several inks alike
    /// Tags: outline plan, pieces per span, fit to tolerance, adaptive split
    /// Node: planned (inkOutline)
    static InkOutlinePlan plan(const InkResult& ink, float tolerance = 1);

    /// One plan for several inks of the same stroke: the most pieces any of them needs in each place. Nullopt when they
    /// cannot share one (no plans, different span counts, an end a line in one and rounded in another, or different
    /// runs).
    ///
    /// Use when: outlines of one stroke at several weights must interpolate piece by piece
    /// Tags: shared plan, common plan, compatible outlines, weight family, variable font masters
    /// Node: planned (inkOutline)
    static std::optional<InkOutlinePlan> sharedPlan(const std::vector<InkOutlinePlan>& plans);

    /// An inked stroke as a closed contour of cubics and lines, fitted to `plan` (its own if none).
    ///
    /// How: (1) the rails: for each stamp, the direction near square across the stroke in which the nib reaches no
    /// further from the stamp before to the one after (halving 24 times), and the nib's support point there; round a
    /// sharp corner the direction is each heading in turn. (2) Each end plane trims the rails. (3) Loops a turn tighter
    /// than the nib makes on the inner side are collapsed to their crossing. (4) Each rail is cut at the plan's anchors
    /// and a cubic fitted through the rail points between them, smooth where its two sides agree on a direction within
    /// `Nib::cornerTurn`. (5) A trimmed end is a line; an untrimmed one is cubics fitted along the end stamp's far
    /// side. A closed stroke is two rings, the left rail forward and the right back.
    ///
    ///     ──left rail──────────╮
    ///     ●━━━━●━━━━●━━━━●     ╎ end (line, or the stamp's far side)
    ///     ──right rail─────────╯
    ///
    /// Conventions: y-up plane; the contour is clockwise; an ink with no points throws `MathProblem`.
    ///
    /// Use when: getting a font stroke's shape to fill, merge, or export
    /// Tags: ink outline, stroke to outline, calligraphy outline, offset curve fit, nib envelope
    /// Node: planned (inkOutline)
    static InkOutlineResult outline(const InkResult& ink, const std::optional<InkOutlinePlan>& plan = std::nullopt, float tolerance = 1);

    /// A plan's shape as text, equal for outlines that interpolate: "L1.2 Eline R1.1 S2:cc" is the left rail's pieces
    /// per span, the end ("line" or its count, then its runs), the right rail, and the start.
    ///
    /// Use when: grouping outlines by whether they can interpolate
    /// Tags: plan key, outline signature, compatibility key
    /// Node: none (debug text)
    static std::string planKey(const InkOutlinePlan& plan);
};

} // namespace editor
