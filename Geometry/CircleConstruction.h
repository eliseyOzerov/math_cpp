// Ruler-and-compass constructions with circles: where a circle meets a line or another circle,
// the lines from a point that touch a circle, and the lines that touch two circles at once. The
// circle through three points is `Circle2::through`.
#pragma once

#include <vector>

#include "Circle2.h"
#include "Line2.h"

namespace editor {

/// How close, as a share of the circles' size, two distances must be to count as equal when a
/// construction decides between "misses", "touches" and "crosses": 1e-5. A line exactly as far
/// from the centre as the radius touches at one point; a little nearer, it crosses at two points
/// so close together that float rounding cannot tell them apart, so they are reported as one.
///
/// Use when: knowing when a nearly tangent line is reported as touching
/// Tags: tangency tolerance, touching tolerance, grazing, coincident crossings
/// Node: none (tolerance of the circle constructions)
inline constexpr float touchingShare = 1e-5f;

/// The points where a circle meets a line: none when the line misses, one when it touches, two
/// when it crosses, ordered along the line's direction.
///
/// How: the foot of the perpendicular from the centre is f = origin + direction·t with
/// t = (centre − origin)·direction / |direction|², at distance h = |centre − f| from the centre.
/// The crossings are f ± û·√(r² − h²), with û the unit direction (Pythagoras in the right
/// triangle centre, foot, crossing).
///
///            ╭───╮
///       ────●─────●────→ line        f is the foot, the crossings are √(r² − h²) either side
///          │  f   │
///           ╰─○──╯        ○ centre, h = |centre − f|
///
/// Conventions: y-up plane; computed in double. The line is infinite (`Line2` read as a line,
/// not a segment). It touches when |h − r| ≤ `touchingShare`·max(r, 1), and then the one point
/// is the foot. A line with a zero direction, or a circle of negative or non-finite radius,
/// meets nothing.
///
/// Source: the line–circle intersection by substitution (Bowyer & Woodwark, *A Programmer's
/// Geometry*, 1983, §4).
///
/// Use when: snapping to where a guide line crosses a circle; trimming a line at a circle
/// Tags: circle line intersection, line circle crossing, secant, chord, intersect circle and line
/// Node: planned (circleLineCrossings)
std::vector<Vector2> circleLineCrossings(const Circle2& circle, const Line2& line);

/// The points where two circles meet: none when they are apart, nested or the same circle, one
/// when they touch, two when they cross. Two crossings come left of the line from the first
/// centre to the second, then right.
///
/// How: with d = |c₂ − c₁| and û = (c₂ − c₁)/d, the crossings lie on the radical line, at
/// a = (d² + r₁² − r₂²)/(2d) along û from c₁ (subtracting the two circle equations), and
/// h = √(r₁² − a²) to either side: c₁ + û·a ± û⊥·h.
///
///          ● crossing (left)
///        ╱ │h╲
///     c₁ ●──a──┼─────● c₂
///        ╲ │  ╱
///          ● crossing (right)
///
/// Conventions: y-up plane; computed in double. They touch when |r₁² − a²| ≤ (touchingShare ·
/// max(r₁, r₂, 1))², and then the one point is c₁ + û·a. Concentric circles (d below that size)
/// give none, also when they are the same circle (they share every point).
///
/// Source: the radical line of two circles (Coxeter, *Introduction to Geometry*, 1969, §6.1).
///
/// Use when: snapping to where two construction circles cross; building a lens or a Venn shape
/// Tags: circle circle intersection, two circles crossing, radical line, lens, intersect circles
/// Node: planned (circleCircleCrossings)
std::vector<Vector2> circleCircleCrossings(const Circle2& first, const Circle2& second);

/// The points where the lines through `point` that touch `circle` touch it: none when the point
/// is inside, one (the point itself) when it is on the circle, two when it is outside, the one
/// counterclockwise of the direction from the centre to the point first.
///
/// How: with d = |point − centre| and û = (point − centre)/d, the touch point T makes a right
/// angle at T (a tangent is perpendicular to its radius), so cos α = r/d, where α is the angle at
/// the centre between û and T − centre. T = centre + r·(û·cos α ± û⊥·sin α), sin α = √(1 − cos²α).
///
///        T ●
///         ╱│╲  right angle at T
///      r ╱ │ ╲
///  centre ●─α─────● point
///
/// Conventions: y-up plane; computed in double. On the circle means |d − r| ≤ touchingShare ·
/// max(r, 1). The tangent line is the line from `point` to each T; (T − centre)·(T − point) = 0.
///
/// Source: Euclid, *Elements*, Book III, Proposition 17 (the tangent from an outside point).
///
/// Use when: drawing a line from a point that just touches a circle; a belt round a pulley
/// Tags: tangent from point, tangent line to circle, touch point, point circle tangent
/// Node: planned (tangentsFromPoint)
std::vector<Vector2> tangentPointsFrom(const Circle2& circle, Vector2 point);

/// One line that touches two circles: where it touches the first and where it touches the
/// second (the same point when the circles touch there).
struct CircleTangent {
    /// Where the line touches the first circle.
    Vector2 onFirst;
    /// Where the line touches the second circle.
    Vector2 onSecond;
    /// Whether the circles lie on opposite sides of the line (an inner, or transverse, tangent:
    /// it passes between them) rather than the same side (an outer, or direct, tangent).
    bool crossesBetween = false;

    bool operator==(const CircleTangent&) const = default;
};

/// The lines that touch both circles: the outer tangents (both circles on the same side) then the
/// inner tangents (the line passes between them). Two circles apart have 4, touching from outside
/// 3, crossing 2, touching from inside 1, nested, concentric or equal 0.
///
/// How: write a line as n·x = k with n a unit normal; its signed distance from a centre c is
/// n·c − k. An outer tangent has n·c₁ − k = r₁ and n·c₂ − k = r₂, so n·û = (r₂ − r₁)/d; an inner
/// one has n·c₂ − k = −r₂, so n·û = −(r₁ + r₂)/d, with d = |c₂ − c₁| and û = (c₂ − c₁)/d. Each
/// gives n = û·cos θ ± û⊥·sin θ for cos θ that ratio (none when |cos θ| > 1, one when it is ±1),
/// and the touch points are c₁ − r₁·n and c₂ ∓ r₂·n (minus for outer, plus for inner).
///
///      ●─────────────●      outer: both circles below the line
///     ( c₁ )╲     ╱( c₂ )
///      ●─────╳───────●      inner: the line crosses between them
///
/// Conventions: y-up plane; computed in double. Within each pair, the tangent whose normal is
/// turned counterclockwise from û comes first. |cos θ| within touchingShare of 1 counts as one
/// tangent. Concentric circles (d ≤ touchingShare · max(r₁, r₂, 1)) have none.
///
/// Source: the common tangents of two circles (Johnson, *Advanced Euclidean Geometry*, 1929,
/// §§ 60–64, the centres of similitude); the normal-vector form is Bowyer & Woodwark (1983).
///
/// Use when: a belt round two pulleys; the hull of two round shapes; joining two circles in a logo
/// Tags: common tangents, bitangent, external tangent, internal tangent, belt, two circle tangent
/// Node: planned (commonTangents)
std::vector<CircleTangent> commonTangents(const Circle2& first, const Circle2& second);

} // namespace editor
