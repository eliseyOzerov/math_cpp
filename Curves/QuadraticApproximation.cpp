// A cubic as a quadratic spline (see QuadraticApproximation.h).
#include "QuadraticApproximation.h"

#include <optional>

#include "../Geometry/Line2.h"

namespace editor {

namespace {

constexpr int deepestHalving = 24;

bool staysWithinAtDepth(Vector2 first, Vector2 second, Vector2 third, Vector2 fourth, float tolerance, int depth) {
    if (second.length() <= tolerance && third.length() <= tolerance) return true;
    if (depth >= deepestHalving) return false;
    Vector2 middle = (first + (second + third) * 3.0f + fourth) * 0.125f;
    if (middle.length() > tolerance) return false;
    Vector2 drift = ((fourth + third) - (second + first)) * 0.125f;
    return staysWithinAtDepth(first, (first + second) * 0.5f, middle - drift, middle, tolerance, depth + 1)
        && staysWithinAtDepth(middle, middle + drift, (third + fourth) * 0.5f, fourth, tolerance, depth + 1);
}

/// Where the end tangents of a cubic meet, ahead of both ends: the control of the quadratic
/// with the same end tangents.
std::optional<Vector2> tangentsMeeting(const CubicBezier<Vector2>& cubic) {
    Vector2 startDirection = (cubic.startHandle - cubic.start).normalized();
    Vector2 endDirection = (cubic.endHandle - cubic.end).normalized();
    float flat = 1e-6f;
    std::optional<LineMeeting> meeting = Line2{cubic.start, startDirection}.meets(Line2{cubic.end, endDirection}, flat);
    if (meeting && meeting->alongFirst > 0 && meeting->alongSecond > 0) return cubic.start + startDirection * meeting->alongFirst;
    return std::nullopt;
}

} // namespace

bool QuadraticApproximation::staysWithin(Vector2 first, Vector2 second, Vector2 third, Vector2 fourth, float tolerance) {
    return staysWithinAtDepth(first, second, third, fourth, tolerance, 0);
}

std::vector<Vector2> QuadraticApproximation::controlPoints(const CubicBezier<Vector2>& cubic, float tolerance) {
    Vector2 zero;
    auto control = [](float parameter, const CubicBezier<Vector2>& piece) {
        return Vector2::linearBlend(piece.start + (piece.startHandle - piece.start) * 1.5f,
                                    piece.end + (piece.endHandle - piece.end) * 1.5f, parameter);
    };
    auto raisedHandle = [](Vector2 end, Vector2 control) { return Vector2::linearBlend(end, control, 2.0f / 3); };
    for (int count = 1; count <= mostPieces; count += 1) {
        if (count == 1) {
            std::optional<Vector2> apex = tangentsMeeting(cubic);
            if (apex && staysWithin(zero, raisedHandle(cubic.start, *apex) - cubic.startHandle,
                                    raisedHandle(cubic.end, *apex) - cubic.endHandle, zero, tolerance)) {
                return {*apex};
            }
            continue;
        }
        std::vector<CubicBezier<Vector2>> pieces = cubic.splitAtEvenParameters(count);
        CubicBezier<Vector2> next = pieces[0];
        Vector2 nextControl = control(0, next);
        Vector2 join = cubic.start;
        Vector2 difference = zero;
        bool fits = true;
        std::vector<Vector2> controls{nextControl};
        for (int index = 1; index <= count && fits; index += 1) {
            Vector2 startHandle = next.startHandle;
            Vector2 endHandle = next.endHandle;
            Vector2 end = next.end;
            Vector2 previousJoin = join;
            Vector2 thisControl = nextControl;
            if (index < count) {
                next = pieces[index];
                nextControl = control(static_cast<float>(index) / static_cast<float>(count - 1), next);
                controls.push_back(nextControl);
                join = (thisControl + nextControl) * 0.5f;
            } else {
                join = end;
            }
            Vector2 previousDifference = difference;
            difference = join - end;
            // startHandle is the piece's own first handle; the raised quadratic's handles are 2/3 of the way to the control.
            fits = difference.length() <= tolerance
                && staysWithin(previousDifference, raisedHandle(previousJoin, thisControl) - startHandle,
                               raisedHandle(join, thisControl) - endHandle, difference, tolerance);
        }
        if (fits) return controls;
    }
    std::vector<Vector2> fallback;
    for (const CubicBezier<Vector2>& piece : cubic.splitAtEvenParameters(mostPieces)) {
        std::optional<Vector2> apex = tangentsMeeting(piece);
        fallback.push_back(apex ? *apex : (piece.startHandle + piece.endHandle) * 0.5f);
    }
    return fallback;
}

} // namespace editor
