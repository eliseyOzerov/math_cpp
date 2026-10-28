// Quarters of an ellipse by end point and heading, arches over a peak, and joining two paths
// (graphics_cpp path.cpp `quarter`, `arch`, `join`).
#include <cmath>

#include "../Linear/MathProblem.h"
#include "Path2D.h"

namespace editor {

namespace {

/// How near the start of one path must be to the end of the other for them to count as meeting.
constexpr float joinDistance = 1e-6f;

} // namespace

/// The share of a quarter's side that its two handles reach: the handle length that makes a cubic
/// follow a quarter circle best.
///
/// How: 4/3 · tan(sweep / 4) with a sweep of 90°, which is 0.5522847 (Riškus, "Approximation of a
/// cubic Bézier curve by circular arcs", 2006; the same factor `CubicBezier::circularArc` uses).
///
/// Use when: rounding the corner of a rectangle by hand; the default tension of `addQuarter`
/// Tags: kappa, circle constant, 0.5523, quarter circle handle, bezier circle
/// Node: none (constant)
float Path2D::circleQuarterTension() { return CubicBezier<Vector2>::circularArcHandleFactor(Angle::degrees(90)); }

/// The axis the last piece arrives along, when it arrives exactly along one: a cubic whose end
/// handle lies level with its end is horizontal, one whose end handle lies straight below or
/// above is vertical. Nothing after a line, on an empty path, or for a slanted arrival.
///
/// How: the arrival direction is `end − endHandle`; one component being exactly 0 and the other
/// not decides it. A quarter built by `addQuarter` always arrives this way, along the axis
/// opposite to the one it left on.
///
/// Use when: continuing a run of quarters so each one leaves where the last one arrived
/// Tags: arrival axis, heading, previous quarter, smooth continuation
/// Node: none (helper of quarter)
std::optional<Heading> Path2D::arrivalHeading() const {
    if (segmentList.empty() || segmentList.back().isLine()) return std::nullopt;
    Vector2 arrival = segmentList.back().end() - segmentList.back().endHandle;
    if (arrival.y == 0 && arrival.x != 0) return Heading::horizontal;
    if (arrival.x == 0 && arrival.y != 0) return Heading::vertical;
    return std::nullopt;
}

/// A quarter of an ellipse from the current point on to `to`: one cubic that leaves along
/// `leaving` and arrives along the other axis.
///
/// How: for a vertical start (a = current point), the handles are
///
///     startHandle = (a.x, a.y + tension·(to.y − a.y))
///     endHandle   = (to.x + tension·(a.x − to.x), to.y)
///
/// so the piece leaves straight up or down and arrives sideways; a horizontal start swaps the
/// roles of x and y. With `tension` ≈ 0.5523 and equal horizontal and vertical distance the
/// piece is a quarter circle; with unequal distances, a quarter ellipse.
///
///     leaving vertically:      to ●──╮                a ●──╮ leaving horizontally
///                                    │                     │
///                                a ● ┘                  to ●
///
/// `leaving` empty continues the way the last piece arrived (`arrivalHeading`), else leaves
/// vertically. Graphics_cpp remembered the last heading; here it is read from the handles, so
/// a hand-drawn cubic that arrives level also steers the next quarter. A closed path throws.
///
/// Conventions: y-up plane; `tension` is the handles' share of the distance (0…1).
///
/// Use when: pills, rounded rectangles, S-bends, leaf outlines drawn by end points
/// Tags: quarter ellipse, ellipse arc, rounded corner, superellipse, quarterTo, pen tool
/// Node: planned (quarter)
void Path2D::addQuarter(Vector2 to, std::optional<Heading> leaving, float tension) {
    Vector2 from = currentPoint();
    Heading axis = leaving ? *leaving : arrivalHeading().value_or(Heading::vertical);
    if (axis == Heading::vertical) {
        addCubic(Vector2(from.x, from.y + tension * (to.y - from.y)), Vector2(to.x + tension * (from.x - to.x), to.y), to);
    } else {
        addCubic(Vector2(from.x + tension * (to.x - from.x), from.y), Vector2(to.x, to.y + tension * (from.y - to.y)), to);
    }
}

/// An arch from the current point on to `to`, rising or falling to a peak at height `peakHeight`
/// halfway across: two quarters, the first leaving vertically.
///
/// How: `addQuarter` to ((from.x + to.x)/2, peakHeight) leaving vertically, then `addQuarter` on
/// to `to`, which continues horizontally from the peak. Both use the same `tension`.
///
///            peak ●──────╮
///                 ╱         ╲        `peakHeight` is the y of the peak, not its rise
///           from ●           ● to
///
/// Use when: a window top, a bridge span, a tunnel profile
/// Tags: arch, archTo, bump, hump, dome, window top
/// Node: planned (arch)
void Path2D::addArch(Vector2 to, float peakHeight, float tension) {
    Vector2 from = currentPoint();
    addQuarter(Vector2((from.x + to.x) / 2, peakHeight), Heading::vertical, tension);
    addQuarter(to, std::nullopt, tension);
}

/// This path followed by `next`: its pieces appended, with a straight line first when `next` does
/// not start where this one ends (further than 1e-6).
///
/// How: copy this path, add a line to `next.start` unless already there, then every piece of
/// `next`. Node marks of both are kept; the node where they meet is cleared, since it is read
/// afresh from the handles on either side.
///
/// Conventions: both paths must be open, or a `MathProblem` is thrown. The result is open.
///
/// Use when: building an outline from separately drawn runs; adding a cap to a stroke side
/// Tags: join paths, concatenate paths, append path, connect, lineTo start
/// Node: planned (joinPaths)
Path2D Path2D::joined(const Path2D& next) const {
    if (closed || next.closed) throw MathProblem("Only open paths can be joined: a closed path has no end to continue from.");
    Path2D result = *this;
    int meetingNode = static_cast<int>(segmentList.size());
    bool needsLink = currentPoint().distance(next.start) >= joinDistance;
    if (needsLink) result.addLine(next.start);
    for (const Segment& segment : next.segmentList) {
        if (segment.isLine()) result.addLine(segment.end());
        else result.addCubic(segment.startHandle, segment.endHandle, segment.end());
    }
    result.setNodeKind(meetingNode, std::nullopt);
    int offset = needsLink ? meetingNode + 1 : meetingNode;
    if (needsLink) result.setNodeKind(offset, std::nullopt);
    for (int node = 1; node <= static_cast<int>(next.segmentList.size()); node += 1) {
        result.setNodeKind(offset + node, next.nodeKind(node));
    }
    return result;
}

} // namespace editor
