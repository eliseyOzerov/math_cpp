// Recognising a drawn stroke as the shape it was meant to be: a line, an arc, a circle, an ellipse,
// a polyline, a chain of lines and arcs, or none of these (freeform). The stroke is measured with
// the fits of ShapeFits.h, cut at its corners, and each stretch between corners is matched to a
// line or an arc.
#pragma once

#include <optional>
#include <vector>

#include "../Geometry/Circle2.h"
#include "../Geometry/Ellipse2.h"
#include "../Geometry/Segment2.h"
#include "../Linear/Angle.h"

namespace editor {

/// The points of a polyline moved to even spacing along it: `count` points, the first at the
/// polyline's start. Open, the last is at its end (spacing length/(count − 1)); closed, the
/// polyline runs back to its first point and the points go round it once without repeating the
/// first (spacing length/count).
///
/// How: the running lengths of the chain (`Polyline2::runLengths`, with the first point appended
/// when closed), then `Polyline2::pointAlong` at each multiple of the spacing.
///
/// Conventions: consecutive repeated points add no length. Fewer than two points, a chain of no
/// length, or `count` below 2 gives the points unchanged.
///
/// Source: arc-length resampling, as in Rubine, "Specifying gestures by example", SIGGRAPH (1991),
/// and Wobbrock, Wilson & Li's $1 recogniser (2007).
///
/// Use when: weighing every part of a hand-drawn stroke alike before fitting it; measuring turns
///   over a fixed length
/// Tags: resample, even spacing, arc length resampling, equidistant points, $1 recogniser
/// Node: planned (resamplePolyline)
std::vector<Vector2> evenlySpacedPoints(const std::vector<Vector2>& points, int count, bool closed);

/// How sharply an evenly spaced chain turns at each point over `window` points either side: the
/// unsigned angle between (pᵢ − pᵢ₋ₖ) and (pᵢ₊ₖ − pᵢ), k = window, in radians, one per point.
///
/// How: the k-cosine of Rosenfeld and Johnston, read as an angle (`Angle::signedTurn` of the two
/// chords). On a circle of radius r with spacing h it is k·h/r, growing with the window; at a
/// corner of turn θ it is θ for every window that reaches past the corner's rounding.
///
///      pᵢ₋ₖ ●─────────● pᵢ        the turn at pᵢ is the angle between the incoming
///                      ╲ θ         chord and the outgoing one
///                       ● pᵢ₊ₖ
///
/// Conventions: closed chains wrap round; on an open chain the first and last `window` points,
/// which have no chord on one side, get 0.
///
/// Source: A. Rosenfeld & E. Johnston, "Angle detection on digital curves", IEEE Transactions on
/// Computers 22 (1973).
///
/// Use when: finding where a drawn stroke has a corner; how much a stroke bends over a length
/// Tags: k-cosine, turning angle, discrete curvature, corner strength, bend
/// Node: none (inside recognisedShape)
std::vector<float> chordTurns(const std::vector<Vector2>& points, int window, bool closed);

/// The indices of the corners of an evenly spaced chain, in order: the points where it turns by
/// at least `minimumTurn` over `window` points either side, sharply.
///
/// How: a point is a corner when its `chordTurns` value over `window` is at least `minimumTurn`,
/// is the largest within `window` points either side (the first of equal ones), and its turn over
/// half the window is at least `sharpness` times its turn over the whole window. The last test
/// separates a corner from a tight smooth bend (the end of a thin ellipse): a corner turns as much
/// over the short window as over the long one (a ratio near 1), a smooth bend of steady curvature
/// only half as much (near ½). That is the scale test of Rattarangsi and Chin.
///
/// Conventions: closed chains wrap round; the ends of an open chain are never corners.
///
/// Source: Rosenfeld & Johnston (1973); A. Rattarangsi & R. Chin, "Scale-based detection of
/// corners of planar curves", IEEE Transactions on Pattern Analysis and Machine Intelligence 14
/// (1992).
///
/// Use when: splitting a drawn polygon at its corners; finding the sharp points of a sketch
/// Tags: corner detection, dominant points, vertices of a sketch, turning point, sharp bend
/// Node: none (inside recognisedShape)
std::vector<int> strokeCorners(const std::vector<Vector2>& points, int window, Angle minimumTurn, float sharpness,
                               bool closed);

/// What `recognisedShape` read a stroke as.
enum class RecognisedKind {
    /// One straight part, open.
    line,
    /// One circular arc, open.
    arc,
    /// A whole circle (closed); see `RecognisedShape::circle`.
    circle,
    /// A whole ellipse (closed); see `RecognisedShape::ellipse`.
    ellipse,
    /// Straight parts only, two or more, open or closed.
    polyline,
    /// Lines and arcs, two or more parts with at least one arc, open or closed.
    linesAndArcs,
    /// None of these: the stroke stays as drawn.
    freeform,
};

/// A circular arc: from the angle `start` round `circle`, turning by `sweep` (counterclockwise
/// when positive).
struct ShapeArc {
    Circle2 circle;
    /// The angle of the arc's first point, seen from the centre, counterclockwise from x.
    Angle start;
    /// How far the arc turns, signed: positive counterclockwise.
    Angle sweep;

    /// The point a `share` of the way along the arc, 0 at its start and 1 at its end:
    /// centre + r·(cos φ, sin φ) with φ = start + share·sweep.
    ///
    /// Use when: drawing a recognised arc; sampling it
    /// Tags: arc point, point on arc, circular arc sample
    /// Node: planned (arc)
    Vector2 point(float share) const;
};

/// One part of a recognised stroke: a line or an arc.
struct ShapePart {
    /// Whether the part is straight or round.
    enum class Kind { line, arc };
    Kind kind = Kind::line;
    /// The straight part, when `kind` is line.
    Segment2 line;
    /// The round part, when `kind` is arc.
    ShapeArc arc;

    /// Where the part starts.
    Vector2 start() const;
    /// Where the part ends.
    Vector2 end() const;
    /// The unit direction the part leaves its start in.
    Vector2 startDirection() const;
    /// The unit direction the part arrives at its end in.
    Vector2 endDirection() const;
};

/// Where two neighbouring parts meet, and how.
struct ShapeJoint {
    /// The shared point: the end of one part and the start of the next.
    Vector2 point;
    /// How far the stroke turns there, signed (positive counterclockwise), from the arriving
    /// direction of the first part to the leaving direction of the second.
    Angle turn;
    /// Whether the parts meet tangentially (|turn| below `ShapeTolerances::smoothJointTurn`), as a
    /// line runs into an arc in a rounded rectangle.
    bool smooth = false;
};

/// The tolerances of `recognisedShape`, as shares of the stroke's size (the diagonal of its
/// bounding box) or as angles, so that the same stroke drawn larger reads the same.
struct ShapeTolerances {
    /// The root mean square distance of the points from a fitted part, as a share of the size,
    /// that still counts as that part: 2%.
    float fitShare = 0.02f;
    /// The least turn over the corner window that is a corner: 35°.
    Angle cornerTurn = Angle::degrees(35);
    /// The corner window either side of a point, as a share of the stroke's length: 4%.
    float cornerWindowShare = 0.04f;
    /// How much of the turn over the whole window a corner keeps over half of it: 0.7 (a smooth
    /// bend keeps about ½, a corner about 1).
    float cornerSharpness = 0.7f;
    /// The gap between the ends, as a share of the size, under which a stroke is closed: 12%.
    float closingShare = 0.12f;
    /// The largest turn at a joint that is still a smooth (tangent) joint: 20°.
    Angle smoothJointTurn = Angle::degrees(20);
    /// The most parts a stroke may be read as; more, and it is freeform: 12.
    int mostParts = 12;
};

/// What a stroke was recognised as: its kind, its parts and the joints between them.
struct RecognisedShape {
    RecognisedKind kind = RecognisedKind::freeform;
    /// The lines and arcs in drawing order: one for a line or an arc, none for a circle, an
    /// ellipse or freeform.
    std::vector<ShapePart> parts;
    /// The joint after each part: joints[i] joins parts[i] to parts[i + 1]; a closed stroke has
    /// one more, joining the last part to the first. Corner joints sit where the neighbouring
    /// parts' lines or circles cross.
    std::vector<ShapeJoint> joints;
    /// Whether the stroke closes on itself.
    bool closed = false;
    /// The circle, when `kind` is circle.
    std::optional<Circle2> circle;
    /// The ellipse, when `kind` is ellipse.
    std::optional<Ellipse2> ellipse;
    /// The root mean square distance of the drawn points from the recognised shape (each part's
    /// own points, weighted by how many respaced points the part covers); 0 for freeform.
    float rmsResidual = 0;
};

/// What `points`, a drawn stroke, most likely is: a line, an arc, a circle, an ellipse, a
/// polyline, a chain of lines and arcs with its corners and smooth joints, or freeform.
///
/// How:
/// 1. The stroke is closed when its ends are within `closingShare` of its size and it is at least
///    1.5 times as long as its size. It is respaced to 256 even points (`evenlySpacedPoints`),
///    so slow and fast parts of the drawing count alike.
/// 2. Its corners are found (`strokeCorners`, window `cornerWindowShare` of its length).
/// 3. Closed: a circle (`fittedCircle`) within `fitShare` of the size, else an ellipse
///    (`fittedEllipse`), is a candidate when there are no corners, or when it fits within half
///    that (a thin ellipse's ends can look like corners).
/// 4. Each stretch between corners (the whole stroke when there are none) is matched greedily:
///    from its start, the longest run of points that a line, or else an arc, fits becomes a part,
///    and the next part starts where it ends. An arc that bows less than the tolerance from its
///    chord is not taken (it is a line with a bend at its end). Inside a stretch the stroke must
///    bend smoothly one way: two lines in a row, or arcs bending opposite ways (an S, a wave),
///    make the stretch unreadable. This is done at the tolerance and at ½, ¼ and ⅛ of it.
/// 5. The reading with the lowest cost wins, a cost of one per part plus 4·rms/tolerance (a
///    description-length trade: a part must bring the points a quarter of the tolerance closer
///    to be worth adding); a whole circle or ellipse counts as one part. No reading, or only ones
///    of more than `mostParts` parts, is freeform.
/// 6. Each part is refitted on the drawn points (the respaced ones sit on chords, inside every
///    curve) nearest to its line or circle, four rounds. Neighbouring parts are then joined: at a
///    corner, at the crossing of their lines or circles nearest to the drawn corner (the drawn
///    point when they do not cross nearby); at a smooth joint, at the foot of the arc's centre on
///    the line, or halfway between the two ends. Each joint's turn is measured between the
///    parts' tangents.
///
/// Conventions: y-up plane; tolerances relative to the size (bounding box diagonal) and length,
/// so the result does not depend on scale. Non-finite points are dropped; a stroke of fewer than
/// three distinct points, or of no size, is freeform.
///
/// Source: the pipeline of T. M. Sezgin, T. Stahovich & R. Davis, "Sketch based interfaces: early
/// processing for sketch understanding", Workshop on Perceptive User Interfaces (2001): corners
/// from turning, then lines and arcs fitted between them; the fits are those of ShapeFits.h.
///
/// Use when: snapping a hand-drawn stroke to a clean line, circle or polygon (draw-and-hold);
///   turning a sketch into editable construction geometry
/// Tags: shape recognition, sketch recognition, beautification, snap to shape, QuickShape,
///   line arc segmentation, stroke classification
/// Node: planned (recogniseShape)
RecognisedShape recognisedShape(const std::vector<Vector2>& points, const ShapeTolerances& tolerances = {});

} // namespace editor
