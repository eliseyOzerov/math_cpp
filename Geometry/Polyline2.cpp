// Measures along a chain of points in the plane.
#include "Polyline2.h"

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

std::vector<float> Polyline2::runLengths() const {
    std::vector<float> runs;
    if (points.empty()) return runs;
    runs.push_back(0);
    for (size_t index = 1; index < points.size(); index += 1) {
        runs.push_back(runs[index - 1] + points[index].distance(points[index - 1]));
    }
    return runs;
}

Vector2 Polyline2::pointAlong(const std::vector<float>& runs, float distance) const {
    if (points.empty()) throw MathProblem("A polyline with no points has no point along it.");
    if (runs.size() != points.size()) throw MathProblem("The running lengths must have one number per point.");
    if (points.size() == 1) return points[0];

    float held = clamped(distance, 0, runs.back());
    size_t found = 0;
    while (found < runs.size() && runs[found] < held) found += 1;
    size_t end = found < 1 ? 1 : found;
    if (end >= points.size()) end = points.size() - 1;

    float pieceLength = runs[end] - runs[end - 1];
    float amount = pieceLength > 0 ? (held - runs[end - 1]) / pieceLength : 1;
    return Vector2::linearBlend(points[end - 1], points[end], amount);
}

float Polyline2::length() const {
    float total = 0;
    for (size_t index = 1; index < points.size(); index += 1) total += points[index].distance(points[index - 1]);
    return total;
}

} // namespace editor
