// The parts of the ink outline shared between its files (rails, caps and the plan): not a public interface.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "../Linear/Angle.h"
#include "../Linear/Vector2.h"
#include "InkOutline.h"

namespace editor::inkoutline {

/// The shortest distance two rail points must be apart to count as different places (font units).
constexpr float distinctDistance = 1e-4f;
/// How far on the wrong side of a plane a rail point may sit and still count as kept.
constexpr float planeSlack = 1e-4f;

/// A rail point and the stamp index it belongs to (fractional where a rail was cut).
struct RailPoint {
    Vector2 point;
    float at = 0;
};
using Rail = std::vector<RailPoint>;

/// The two rails, trimmed and untangled, and which ends a plane trims.
struct RailPair {
    Rail left;
    Rail right;
    bool trimmedStart = false;
    bool trimmedEnd = false;

    bool trimmed(InkEnd end) const { return end == InkEnd::start ? trimmedStart : trimmedEnd; }
};

/// A piece of the contour: a line (only `cubic.start` and `cubic.end` count) or a cubic.
struct Piece {
    CubicBezier<Vector2> cubic;
    bool isLine = false;

    static Piece line(Vector2 start, Vector2 end) { return Piece{CubicBezier<Vector2>(start, start, end, end), true}; }
    static Piece curve(const CubicBezier<Vector2>& cubic) { return Piece{cubic, false}; }
    Piece reversed() const { return Piece{cubic.reversed(), isLine}; }
};

/// The result of fitting an outline to a plan.
struct Fit {
    std::vector<Piece> contour;
    std::vector<std::vector<Piece>> contours;
    InkOutlineErrors errors;
    InkOutlineRuns runs;
};

/// The angle between two unit directions, 0…π.
float turnBetween(Vector2 first, Vector2 second);
/// The unit directions a piece leaves its start and arrives at its end with (zero where it has none).
std::pair<Vector2, Vector2> endTangents(const Piece& piece);

/// Trimmed and untangled rails of an ink.
RailPair outlineParts(const InkResult& ink);
/// The first index of a rail at or past `at` (`inclusive`), or past it.
int firstAt(const Rail& rail, float at, bool inclusive);
/// The rail's point at a stamp index, between its samples.
Vector2 railAt(const Rail& rail, float at);
/// The rail's direction leaving `at` forwards (side 1) or arriving there (−1).
Vector2 railDirection(const Rail& rail, float at, float side);

/// Cubics fitted to a rail by the plan, with the worst error in each span.
struct RailFit {
    std::vector<CubicBezier<Vector2>> cubics;
    std::vector<float> errors;
};
RailFit fitRail(const Rail& rail, const InkResult& ink, const std::vector<int>& plan);

/// An untrimmed end's pieces, worst error and runs.
struct Cap {
    std::vector<Piece> pieces;
    float error = 0;
    std::string runs;
};
Cap fitCap(const InkResult& ink, InkEnd end, Vector2 from, Vector2 to, Vector2 tangentFrom, Vector2 tangentTo, int pieces);

/// The whole contour by a plan.
Fit fitOutline(const InkResult& ink, const InkOutlinePlan& plan, const RailPair& parts);

} // namespace editor::inkoutline
