// The nib itself: naming one, reading its corners, blending two shapes and stamping the polygon (Nib.h).
// Ported from graphics_cpp curves/nib (nib.cpp), in floats.
#include "Nib.h"

#include <cmath>
#include <limits>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

Angle Nib::cornerTurn() { return Angle::degrees(20); }

const std::array<Vector2, 4>& Nib::cornerSigns() {
    static const std::array<Vector2, 4> signs = {Vector2(1, 1), Vector2(-1, 1), Vector2(-1, -1), Vector2(1, -1)};
    return signs;
}

Nib Nib::from(const NibInput& input) {
    if (std::holds_alternative<std::monostate>(input)) return Nib{};
    if (const float* diameter = std::get_if<float>(&input)) {
        Nib round;
        round.width = *diameter;
        round.height = *diameter;
        return round;
    }
    if (const StemPen* pen = std::get_if<StemPen>(&input)) return stemNib(*pen);
    return std::get<Nib>(input);
}

Nib Nib::stemNib(const StemPen& pen) {
    float contrast = clamped(pen.contrast, 0, 0.99f);
    float cosine = pen.stress.cosine();
    float sine = pen.stress.sine();
    float thinness = 1 - contrast;
    Nib nib;
    nib.width = pen.weight / std::sqrt(cosine * cosine + thinness * thinness * (sine * sine));
    nib.height = nib.width * thinness;
    nib.angle = pen.stress;
    return nib;
}

StemPen Nib::penOf(const NibInput& input) {
    if (const StemPen* pen = std::get_if<StemPen>(&input)) return *pen;
    Nib nib = from(input);
    NibInput whole = nib;
    return StemPen{std::round(stemWidth(whole) * 100) / 100, std::round((1 - nib.height / nib.width) * 100) / 100, nib.angle};
}

std::array<float, 4> Nib::radii(const CornerRadius& radius) {
    if (const std::vector<float>* list = std::get_if<std::vector<float>>(&radius)) {
        std::array<float, 4> result{};
        for (size_t corner = 0; corner < 4; corner += 1) {
            if (corner < list->size()) result[corner] = (*list)[corner];
            else result[corner] = list->empty() ? std::numeric_limits<float>::quiet_NaN() : (*list)[0];
        }
        return result;
    }
    float value = std::get<float>(radius);
    return {value, value, value, value};
}

CornerRadius Nib::uniform(const CornerRadius& radius) {
    std::array<float, 4> corners = radii(radius);
    bool same = true;
    for (float value : corners) {
        if (!(std::fabs(value - corners[0]) < 1e-6f)) same = false;
    }
    if (same) return corners[0];
    return std::vector<float>(corners.begin(), corners.end());
}

NibShape Nib::mixShape(const NibShape& first, const NibShape& second, float amount) {
    auto blendRadius = [amount](const CornerRadius& start, const CornerRadius& end) -> CornerRadius {
        if (std::holds_alternative<std::vector<float>>(start) || std::holds_alternative<std::vector<float>>(end)) {
            std::array<float, 4> startRadii = radii(start);
            std::array<float, 4> endRadii = radii(end);
            std::vector<float> blended(4);
            for (size_t corner = 0; corner < 4; corner += 1) blended[corner] = linearBlend(startRadii[corner], endRadii[corner], amount);
            return uniform(blended);
        }
        return linearBlend(std::get<float>(start), std::get<float>(end), amount);
    };
    return NibShape{linearBlend(first.width, second.width, amount), linearBlend(first.height, second.height, amount),
                    blendRadius(first.radiusX, second.radiusX), blendRadius(first.radiusY, second.radiusY)};
}

std::array<NibCornerArc, 4> Nib::cornerArcs(const NibShape& shape) {
    float halfWidth = shape.width / 2;
    float halfHeight = shape.height / 2;
    std::array<float, 4> horizontalRadii = radii(shape.radiusX);
    std::array<float, 4> verticalRadii = radii(shape.radiusY);
    std::array<NibCornerArc, 4> arcs;
    for (size_t corner = 0; corner < 4; corner += 1) {
        Vector2 signs = cornerSigns()[corner];
        float radiusAlongX = clamped(horizontalRadii[corner], 0, 1) * halfWidth;
        float radiusAlongY = clamped(verticalRadii[corner], 0, 1) * halfHeight;
        arcs[corner] = NibCornerArc{signs, Vector2(signs.x * (halfWidth - radiusAlongX), signs.y * (halfHeight - radiusAlongY)),
                                    Vector2(radiusAlongX, radiusAlongY)};
    }
    return arcs;
}

std::vector<Vector2> Nib::stampPolygon(const NibShape& shape, Angle rotation, int pointsPerCorner) {
    if (pointsPerCorner < 1) throw MathProblem("A stamp needs at least one point per corner.");
    float halfWidth = shape.width / 2;
    float halfHeight = shape.height / 2;
    std::vector<Vector2> polygon;
    std::array<NibCornerArc, 4> arcs = cornerArcs(shape);
    for (size_t corner = 0; corner < 4; corner += 1) {
        const NibCornerArc& arc = arcs[corner];
        if (arc.radii.x < 1e-6f || arc.radii.y < 1e-6f) {
            polygon.push_back(Vector2(arc.signs.x * halfWidth, arc.signs.y * halfHeight));
            continue;
        }
        for (int step = 0; step <= pointsPerCorner; step += 1) {
            Angle sweep = Angle::degrees(90 * static_cast<float>(corner) + 90 * static_cast<float>(step) / static_cast<float>(pointsPerCorner));
            polygon.push_back(Vector2(arc.arcCentre.x + arc.radii.x * sweep.cosine(), arc.arcCentre.y + arc.radii.y * sweep.sine()));
        }
    }
    for (Vector2& point : polygon) point = point.rotated(rotation);
    return polygon;
}

} // namespace editor
