// An affine map of the plane, defined (declared in AffineTransform2.h). Ported from graphics_cpp's
// `Affine` (Curves2D/src/geometry.cpp), in floats, with Angle for the angles.
#include "AffineTransform2.h"

#include <cmath>

namespace editor {

namespace {

/// The linear map with columns `xAxis` and `yAxis`, made to leave `pivot` where it is:
/// translation = pivot − linear(pivot).
AffineTransform2 linearAbout(Vector2 xAxis, Vector2 yAxis, Vector2 pivot) {
    AffineTransform2 linear{xAxis, yAxis, Vector2()};
    return AffineTransform2{xAxis, yAxis, pivot - linear.appliedToVector(pivot)};
}

} // namespace

AffineTransform2 AffineTransform2::translationBy(Vector2 offset) { return AffineTransform2{Vector2(1, 0), Vector2(0, 1), offset}; }

AffineTransform2 AffineTransform2::translationAlong(Angle axisAngle, float distance) {
    return translationBy(axisAngle.direction() * distance);
}

AffineTransform2 AffineTransform2::rotation(Angle angle, Vector2 pivot) {
    float cosine = angle.cosine();
    float sine = angle.sine();
    return linearAbout(Vector2(cosine, sine), Vector2(-sine, cosine), pivot);
}

AffineTransform2 AffineTransform2::scaling(Vector2 factors, Vector2 pivot) {
    return linearAbout(Vector2(factors.x, 0), Vector2(0, factors.y), pivot);
}

AffineTransform2 AffineTransform2::shear(Angle lean, Angle axisAngle, Vector2 pivot) {
    Vector2 along = axisAngle.direction();
    Vector2 across = along.perpendicular();
    float strength = -lean.tangent();
    // identity + strength · along · acrossᵀ, written column by column.
    Vector2 xAxis = Vector2(1, 0) + along * (strength * across.x);
    Vector2 yAxis = Vector2(0, 1) + along * (strength * across.y);
    return linearAbout(xAxis, yAxis, pivot);
}

Vector2 AffineTransform2::applied(Vector2 point) const { return appliedToVector(point) + translation; }

Vector2 AffineTransform2::appliedToVector(Vector2 vector) const { return xAxis * vector.x + yAxis * vector.y; }

AffineTransform2 AffineTransform2::then(const AffineTransform2& next) const {
    return AffineTransform2{next.appliedToVector(xAxis), next.appliedToVector(yAxis), next.applied(translation)};
}

float AffineTransform2::determinant() const { return xAxis.cross(yAxis); }

std::optional<AffineTransform2> AffineTransform2::inverse() const {
    float divisor = determinant();
    if (!(std::isfinite(divisor) && divisor != 0)) return std::nullopt;
    Vector2 inverseXAxis = Vector2(yAxis.y, -xAxis.y) / divisor;
    Vector2 inverseYAxis = Vector2(-yAxis.x, xAxis.x) / divisor;
    AffineTransform2 linear{inverseXAxis, inverseYAxis, Vector2()};
    return AffineTransform2{inverseXAxis, inverseYAxis, -linear.appliedToVector(translation)};
}

AffineTransform2 AffineTransform2::partly(float amount) const {
    AffineTransform2 none = identity();
    return AffineTransform2{Vector2::linearBlend(none.xAxis, xAxis, amount), Vector2::linearBlend(none.yAxis, yAxis, amount),
                            Vector2::linearBlend(none.translation, translation, amount)};
}

AffineTransform2 AffineTransform2::inMetric(Vector2 metric) const {
    float acrossPerUp = metric.x / metric.y;
    return AffineTransform2{Vector2(xAxis.x, xAxis.y * acrossPerUp), Vector2(yAxis.x / acrossPerUp, yAxis.y),
                            translation.fromMetric(metric)};
}

} // namespace editor
