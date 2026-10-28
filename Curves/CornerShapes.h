// Corners of filled shapes and what goes in their place: pits and peaks, corners replaced by any
// piece, corners shaped by a tilt, a profile or a curve drawn in the corner's own frame, and caps for
// the ends of strokes. Ported from graphics_cpp curves/corners.cpp (Curves2D). The fixed fillet and
// chamfer of a single path are `Path2D::filleted` and `Path2D::chamfered`; everything here works on a
// `Shape2D` (several outlines) and accepts any replacement.
//
// Conventions: y-up plane. Outlines are taken as a boolean operation leaves them, the inside on the
// LEFT of each piece (outer outlines counterclockwise, holes clockwise). A turn to the left is then a
// convex corner (a peak) and a turn to the right a concave one (a pit). graphics_cpp keeps the inside on
// the right, so every sign here is the mirror of its source.
//
//   peak (convex)              pit (concave)
//        ╱╲                     ───╲    ╱───
//       ╱  ╲  ink inside         ink  ╲╱  ink      the wedge OUTSIDE the ink is narrow at a pit
#pragma once

#include <functional>
#include <optional>
#include <vector>

#include "Boolean2D.h"
#include "Shape2D+Pieces.h"

namespace editor {

/// A corner of one outline of a shape: where one piece meets the next at an angle.
///
/// Conventions: `turn` is positive to the left (convex, with the inside on the left) and negative to
/// the right (concave); `pieceIndex` counts the pieces of the outline closed by its last line
/// (`Path2D::cornerPieces`), the piece that leaves the corner.
///
/// Use when: choosing which corners of an outline to round, trap or shape; reading the sharp points of a glyph
/// Tags: shape corner, vertex, peak, pit, turn angle, concave corner, convex corner
/// Node: planned (shapeCorners)
struct ShapeCorner {
    /// Which outline of the shape, counting from 0.
    int outlineIndex = 0;
    /// The piece that leaves the corner, in `Path2D::cornerPieces` of the closed outline.
    int pieceIndex = 0;
    Vector2 point;
    /// The unit direction the outline arrives by, and the one it leaves by.
    Vector2 incomingDirection;
    Vector2 outgoingDirection;
    /// The signed angle from `incomingDirection` to `outgoingDirection`.
    Angle turn;

    bool operator==(const ShapeCorner&) const = default;

    bool isConvex() const { return turn.radians > 0; }
    Angle outsideAngle() const;
};

std::vector<ShapeCorner> cornersOfShape(const Shape2D& shape, Angle minimumTurn = Angle::degrees(1));
std::vector<ShapeCorner> pitsOfShape(const Shape2D& shape, Angle largestOutsideAngle = Angle::degrees(170));
std::vector<ShapeCorner> peaksOfShape(const Shape2D& shape, Angle smallestTurn = Angle::degrees(10));

/// The options of `replacedCorners` and `shapedCorners`.
///
/// Use when: ignoring tiny kinks; letting corners cut back further along short sides
/// Tags: corner options, minimum turn, share of side
/// Node: planned (shapeCorners)
struct CornerOptions {
    /// Corners that turn by less than this are left alone.
    Angle minimumTurn = Angle::degrees(1);
    /// The most of a side (the run of the outline to the next corner) that a corner may cut back.
    float share = Path2D::largestCornerShare;
};

/// What goes in a corner's place: from `cutBefore` leaving along `directionBefore` to `cutAfter`
/// arriving along `directionAfter`, at `corner`.
using CornerReplacement = std::function<std::vector<OutlinePiece>(
    Vector2 cutBefore, Vector2 directionBefore, Vector2 cutAfter, Vector2 directionAfter, const ShapeCorner& corner)>;

Shape2D replacedCorners(
    const Shape2D& shape, const std::function<float(const ShapeCorner&)>& reach, const CornerReplacement& replacement,
    CornerOptions options = CornerOptions());

/// The frame a corner's replacement is drawn in (see `cornerFrame`).
///
///     y = 1 ·················● corner            x runs −1 at the first cut `a` to 1 at the second `b`
///           ╲              ╱                    y runs 0 on the chord ab to 1 at the corner itself
///        a ●───────────────● b     y = 0
///
/// Past 1 is beyond the corner: into the ink at a pit (a trap), out of it at a peak; below 1 is the
/// other way (a well, a cove). The map from frame to plane is affine: plane = middle + halfChord·x +
/// height·y.
///
/// Use when: drawing the shape of a corner once and having it fit any corner of any angle
/// Tags: corner frame, local coordinates, affine frame, ink trap frame, notch frame
/// Node: planned (shapeCorners)
struct CornerFrame {
    Vector2 middle;
    Vector2 halfChord;
    Vector2 height;
    float determinant = 0;

    bool operator==(const CornerFrame&) const = default;

    Vector2 toPlane(Vector2 local) const;
    Vector2 fromPlane(Vector2 plane) const;
};

CornerFrame cornerFrame(Vector2 firstCut, Vector2 secondCut, Vector2 corner);

/// How one corner is shaped by `shapedCorners`.
///
/// `tilt` (−1…1) sets the angle the replacing arc meets the sides at, against the ink: 0 runs on along
/// them (the plain fillet); towards 1 it tilts out of the ink at a peak (adding ink) and into it at a pit;
/// towards −1 the other way. At ±1 the arc leaves the sides square, at 90°. A `profile` is an open
/// path drawn in the arc's place, across a box from x −1 at the corner's first cut to 1 at its second,
/// and up, its bounding box's top as far off the chord as the arc for that tilt bulges. A `curve` is an open
/// path drawn in the corner's own frame instead (`cornerFrame`), its ends pinned to the cuts.
///
/// Use when: giving every corner of a glyph the same ink trap; softening corners by degrees
/// Tags: corner shape, ink trap, corner tilt, corner profile, notch, cove
/// Node: planned (shapeCorners)
struct CornerShape {
    float tilt = 0;
    std::optional<Path2D> profile;
    std::optional<Path2D> curve;

    CornerShape() = default;
    explicit CornerShape(float tilt) : tilt(tilt) {}
};

Shape2D shapedCorners(
    const Shape2D& shape, const std::function<float(const ShapeCorner&)>& size,
    const std::function<CornerShape(const ShapeCorner&)>& shapeOfCorner,
    const std::function<void(const ShapeCorner&, Vector2, Vector2)>& seen = {});

Path2D shapedCorners(
    const Path2D& path, const std::function<float(const ShapeCorner&)>& size,
    const std::function<CornerShape(const ShapeCorner&)>& shapeOfCorner);

// MARK: Caps (CornerShapes+Caps.cpp)

/// A cap: its curve, open, across the stroke, and where the stroke's end sits in the curve's box.
///
/// `align` is 1 for the end at the bottom of the curve's box (the default: the cap rises out of the
/// end), −1 for the end at the top (the cap carves into the stroke), and between for between.
///
/// Use when: ending a stroke with a point, a square end or a half round
/// Tags: cap, stroke end, line cap, terminal, butt cap, round cap
/// Node: planned (capStroke)
struct CapShape {
    Path2D curve;
    std::optional<float> align;

    explicit CapShape(Path2D curve, std::optional<float> align = std::nullopt) : curve(std::move(curve)), align(align) {}

    static CapShape triangle(float height = 1);
    static CapShape rectangle(float height = 0.5f);
    static CapShape arc(float height = 1);
};

/// An end of a stroke: where its centreline ends, the unit direction it leaves by (outward), and the
/// ink's half-width there (used where the outline cannot be measured).
///
/// Use when: telling `cappedStroke` which end of an outline to reshape
/// Tags: stroke end, end frame, terminal, end point and direction
/// Node: planned (capStroke)
struct StrokeEnd {
    Vector2 point;
    Vector2 direction;
    float halfWidth = 0;
};

Shape2D cappedStroke(const Shape2D& stroke, const StrokeEnd& end, const CapShape& cap);

} // namespace editor
