// An ellipse's points and the nearest of them to a point (see Ellipse2.h).
#include "Ellipse2.h"

#include <cmath>

namespace editor {

namespace {

/// The root s of G(s) = (r·z₀/(s + r))² + (z₁/(s + 1))² − 1, by bisection (Eberly 2013, `GetRoot`).
/// `startingValue` is G(0).
double ellipseNormalRoot(double ratio, double scaledFirst, double scaledSecond, double startingValue) {
    double stretchedFirst = ratio * scaledFirst;
    double lower = scaledSecond - 1;
    double upper = startingValue < 0 ? 0 : std::hypot(stretchedFirst, scaledSecond) - 1;
    double middle = lower;
    // Each halving shrinks the bracket; in double it stops moving after about 1100 steps at most.
    for (int step = 0; step < 1200; step += 1) {
        middle = (lower + upper) / 2;
        if (middle == lower || middle == upper) break;
        double firstRatio = stretchedFirst / (middle + ratio);
        double secondRatio = scaledSecond / (middle + 1);
        double value = firstRatio * firstRatio + secondRatio * secondRatio - 1;
        if (value > 0) {
            lower = middle;
        } else if (value < 0) {
            upper = middle;
        } else {
            break;
        }
    }
    return middle;
}

} // namespace

Vector2 Ellipse2::point(Angle parameter) const {
    Vector2 majorAxis(rotation.cosine(), rotation.sine());
    return centre + majorAxis * (majorRadius * parameter.cosine()) + majorAxis.perpendicular() * (minorRadius * parameter.sine());
}

Vector2 Ellipse2::nearestPoint(Vector2 position) const {
    double cosine = std::cos(static_cast<double>(rotation.radians));
    double sine = std::sin(static_cast<double>(rotation.radians));
    double offsetX = static_cast<double>(position.x) - centre.x;
    double offsetY = static_cast<double>(position.y) - centre.y;
    // The position in the ellipse's own frame (major axis along the first coordinate), folded
    // into the first quadrant; the signs unfold the answer at the end.
    double alongMajor = offsetX * cosine + offsetY * sine;
    double alongMinor = -offsetX * sine + offsetY * cosine;
    double firstSign = alongMajor < 0 ? -1 : 1;
    double secondSign = alongMinor < 0 ? -1 : 1;
    double first = std::fabs(alongMajor);
    double second = std::fabs(alongMinor);
    double major = majorRadius;
    double minor = minorRadius;
    double nearestFirst = 0;
    double nearestSecond = 0;
    if (!(major > 0)) {
        nearestFirst = 0;
        nearestSecond = 0;
    } else if (!(minor > 0)) {
        nearestFirst = std::fmin(first, major);
        nearestSecond = 0;
    } else if (second > 0) {
        if (first > 0) {
            double scaledFirst = first / major;
            double scaledSecond = second / minor;
            double startingValue = scaledFirst * scaledFirst + scaledSecond * scaledSecond - 1;
            if (startingValue != 0) {
                double ratio = (major / minor) * (major / minor);
                double root = ellipseNormalRoot(ratio, scaledFirst, scaledSecond, startingValue);
                nearestFirst = ratio * first / (root + ratio);
                nearestSecond = second / (root + 1);
            } else {
                nearestFirst = first;
                nearestSecond = second;
            }
        } else {
            nearestFirst = 0;
            nearestSecond = minor;
        }
    } else {
        // On the major axis: the nearest point is off the axis only inside the evolute's cusp.
        double numerator = major * first;
        double denominator = major * major - minor * minor;
        if (numerator < denominator) {
            double share = numerator / denominator;
            nearestFirst = major * share;
            nearestSecond = minor * std::sqrt(1 - share * share);
        } else {
            nearestFirst = major;
            nearestSecond = 0;
        }
    }
    nearestFirst *= firstSign;
    nearestSecond *= secondSign;
    return Vector2(static_cast<float>(centre.x + nearestFirst * cosine - nearestSecond * sine),
                   static_cast<float>(centre.y + nearestFirst * sine + nearestSecond * cosine));
}

float Ellipse2::distance(Vector2 position) const { return position.distance(nearestPoint(position)); }

} // namespace editor
