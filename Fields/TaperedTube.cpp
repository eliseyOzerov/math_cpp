// A tube of varying radius along a cubic Bézier (Engine/Math/Fields/TaperedTube.swift).
#include "TaperedTube.h"

#include <cmath>
#include <limits>

#include "../Linear/GeometryProblem.h"
#include "../Linear/Scalar.h"
#include "VectorComponents.h"

namespace editor {

namespace {

std::vector<Vector3> controlPoints(const CubicBezier<Vector3>& curve) {
    return {curve.start, curve.startHandle, curve.endHandle, curve.end};
}

bool allSatisfy(const std::vector<Vector3>& points, bool (*test)(Vector3)) {
    for (Vector3 point : points) {
        if (!test(point)) return false;
    }
    return true;
}

bool isWithinLargestCoordinate(Vector3 point) {
    return largerOf(std::fabs(point.x), largerOf(std::fabs(point.y), std::fabs(point.z))) <= TaperedTube::largestCoordinate;
}

bool isFinitePoint(Vector3 point) { return point.isFinite(); }

/// `relativeTolerance` times the larger radius; refuses a negative radius first, as Swift's
/// precondition does before it builds anything.
float toleranceForRadii(float startRadius, float endRadius, float relativeTolerance) {
    if (!(startRadius >= 0 && endRadius >= 0)) throw GeometryProblem("A tapered tube cannot have a negative radius.");
    return relativeTolerance * largerOf(startRadius, endRadius);
}

} // namespace

TaperedTube::TaperedTube(const CubicBezier<Vector3>& curve, const std::function<float(float)>& radius, float tolerance) {
    if (!(tolerance > 0 && std::isfinite(tolerance))) {
        throw GeometryProblem("A tapered tube needs a positive, finite tolerance.");
    }
    if (!allSatisfy(controlPoints(curve), isFinitePoint)) throw GeometryProblem("A tapered tube needs finite control points.");
    if (!allSatisfy(controlPoints(curve), isWithinLargestCoordinate)) {
        throw GeometryProblem("A tapered tube's control points must be at most 1e18 in every coordinate, or its arithmetic overflows.");
    }
    std::int64_t neededPieces = neededPieceCount(curve, tolerance);
    int pieceCount = static_cast<int>(neededPieces < mostPieces ? neededPieces : mostPieces);
    reachedMostPieces = neededPieces > mostPieces;

    std::vector<Vector3> centres = {curve.start};
    for (int step = 1; step < pieceCount; step += 1) {
        centres.push_back(curve.point(static_cast<float>(step) / static_cast<float>(pieceCount)));
    }
    centres.push_back(curve.end);
    std::vector<float> radii;
    for (int step = 0; step <= pieceCount; step += 1) radii.push_back(radius(static_cast<float>(step) / static_cast<float>(pieceCount)));
    for (float each : radii) {
        if (!(each >= 0 && std::isfinite(each))) {
            throw GeometryProblem("A tapered tube's radius must be finite and 0 or more along the whole curve.");
        }
    }

    for (int index = 0; index < pieceCount; index += 1) {
        pieces.emplace_back(centres[index], centres[index + 1], radii[index], radii[index + 1]);
    }
    for (const RoundCone& piece : pieces) pieceBoxes.push_back(piece.enclosingBox());
    Bounds total = pieceBoxes[0];
    for (size_t index = 1; index < pieceBoxes.size(); index += 1) total = total.unionWith(pieceBoxes[index]);
    boundingBox = total;
}

TaperedTube::TaperedTube(const CubicBezier<Vector3>& curve, float startRadius, float endRadius, float relativeTolerance)
    : TaperedTube(
          curve, [startRadius, endRadius](float parameter) { return linearBlend(startRadius, endRadius, parameter); },
          toleranceForRadii(startRadius, endRadius, relativeTolerance)) {}

std::int64_t TaperedTube::neededPieceCount(const CubicBezier<Vector3>& curve, float tolerance) {
    float firstBend = (curve.start - curve.startHandle * 2 + curve.endHandle).length();
    float secondBend = (curve.startHandle - curve.endHandle * 2 + curve.end).length();
    float needed = std::ceil(std::sqrt(0.75f * largerOf(firstBend, secondBend) / tolerance));
    // 2⁶³ is the first float above the largest 64-bit integer.
    if (!(needed < 9223372036854775808.0f)) return std::numeric_limits<std::int64_t>::max();
    std::int64_t whole = static_cast<std::int64_t>(needed);
    return whole > 1 ? whole : 1;
}

float TaperedTube::signedDistance(Vector3 point) const {
    if (!point.isFinite()) return NAN;
    float nearest = INFINITY;
    for (size_t index = 0; index < pieces.size(); index += 1) {
        if (signedDistanceOf(pieceBoxes[index], point) < nearest) nearest = smallerOf(nearest, pieces[index].signedDistance(point));
    }
    return nearest;
}

} // namespace editor
