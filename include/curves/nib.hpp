// The nib: a rounded rectangle stamped along a stroke, its curves along the stroke and its stamps as convex polygons
// (the hulls and clips that join stamps into ink are in fit.hpp).
//
// Weight compensation: the nib's size along a stroke, so strokes of one weight look it. Rounds are made a little
// heavier and diagonals a little lighter, by segment: every curve is a round, every line straight or diagonal by its
// direction; where the factor changes at a node the size eases across `ramp` units centred on it.
//
// Ported from fountain-ts/src/ink/nib.ts.
//
// Decisions:
// - Radius is a variant of one number or one per corner; `radii` reads it as four.
// - NibInput is a variant: nothing (std::monostate), a number (a round nib that wide), a whole Nib, or a StemPen. The
//   TS's Partial<Nib> is a Nib the caller fills from NIB.
// - ShapesInput's partial shapes are PartialShape, its missing fields std::nullopt.
#pragma once

#include "curves/geometry.hpp"
#include "curves/profile.hpp"

#include <array>
#include <optional>
#include <variant>
#include <vector>

namespace curves {

/** A corner radius as a fraction of the half-side: one number for every corner, or one per corner (see CORNERS). */
using Radius = std::variant<double, std::vector<double>>;
/** The nib: a rectangle `w` wide and `h` tall (font units) with corners rounded by `rx`, `ry`, turned `angle` degrees
 *  counter-clockwise; `follow` (0…1) turns it with the stroke. Sampling: `perCorner` points round each corner of the
 *  stamp, `perSpan` stamps along each curve segment. */
struct Nib {
  double w = 72, h = 72;
  Radius rx = 1.0, ry = 1.0;
  double angle = 0, follow = 0, perCorner = 5, perSpan = 16;
};
/** A pen: an elliptical nib leaving a vertical stem `weight` wide, `contrast` (0…1) thinner across, turned `stress`
 *  degrees. */
struct StemPen {
  double weight, contrast = 0, stress = 0;
};
/** What a nib can be given as: nothing (the default), a bare number (a round nib that wide), a nib, or a pen. */
using NibInput = std::variant<std::monostate, double, Nib, StemPen>;
/** A nib's shape at one place: its size and corners. */
struct Shape {
  double w, h;
  Radius rx, ry;
};
/** A placed nib: its centre and its shape turned `rot` radians. */
struct Frame : Shape {
  Pt c;
  double rot;
};

/** The sharpest turn that still runs on smoothly: past it the centreline's turn between segments is a corner the
 *  nib sweeps round in place (ink), and a rail's turn is a corner the outline keeps sharp. */
extern const double CORNER;
extern const Nib NIB;

/** A nib input as a whole nib. A bare number is a round nib that wide; a pen is solved by its stem (stemNib). */
Nib nibOf(const NibInput& nib = {});
/** A pen by its stem: the horizontal reach of the turned ellipse, sqrt(w²cos²θ + h²sin²θ) / 2, set equal to
 *  weight / 2 with h = w·(1 − contrast). */
Nib stemNib(const StemPen& pen);
/** The pen of a nib input by its stem, contrast and stress (the inverse of stemNib, its corners left out). */
StemPen stemPenOf(const NibInput& nib = {});

/** The corners in the nib's own frame, counter-clockwise from the top right: their signs along x and y. */
extern const std::array<Pt, 4> CORNERS;
/** A radius as one per corner. */
std::array<double, 4> radii(const Radius& v);
/** `radii` given back as the one number when all four agree. */
Radius uniform(const Radius& v);
/** A shape part of the way (u) from a to b. */
Shape mixShape(const Shape& a, const Shape& b, double u);

/** One corner of a shape's rounded rectangle: its signs `s`, the centre `o` of its quarter ellipse and that ellipse's
 *  radii `r` (0 for a square corner). */
struct CornerArc {
  Pt s, o, r;
};
/** The corners of a shape's rounded rectangle, in the nib's own frame, counter-clockwise from the top right. */
std::array<CornerArc, 4> cornerArcs(const Shape& shape);
/** The stamp: the nib's rounded rectangle as a convex polygon about the origin, counter-clockwise, turned by `rot`
 *  radians. */
std::vector<Pt> stampPolygon(const Shape& shape, double rot = 0, double perCorner = 5);

/** Width of the ink a nib leaves on a straight stroke travelling at `angle` (radians from the horizontal). */
double nibWidth(const NibInput& nib, double angle, double scale = 1);
/** The stem: the ink's width on an upright stroke. */
double stemWidth(const NibInput& nib, double scale = 1);

/** A shape given in part: fields left out are 1 (w, h) or the nib's own (rx, ry). */
struct PartialShape {
  std::optional<double> w, h;
  std::optional<Radius> rx, ry;
};
/** Nib shapes given loosely: keys of shapes as they are, or partial shapes spread evenly along the stroke. */
using ShapesInput = std::variant<Keys<Shape>, std::vector<PartialShape>>;
std::optional<Keys<Shape>> shapesOf(const std::optional<ShapesInput>& v, const Nib& nib);

/** How much heavier rounds are and how much lighter diagonals, as ratios of the weight (1 leaves them). */
struct Compensation {
  double round = 1, diagonal = 1;
};
/** The weight factor of one segment under `comp`. */
double segmentWeight(const Cubic& c, const Compensation& comp);
/** The nib's size along the stroke `segs` (closed or not) under `comp`, as keys over its length, easing over `ramp`
 *  units where neighbouring segments differ; nullopt where every segment is 1. */
std::optional<Keys<double>> weightCurve(const std::vector<Cubic>& segs, bool closed, const Compensation& comp, double ramp);

}  // namespace curves
