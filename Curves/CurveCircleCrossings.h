// Where a cubic curve meets a circle, and where two cubics meet, as free functions beside the
// curve's own `crossings` (which meet lines and are in CubicBezier+Crossings.cpp). Ported from
// graphics_cpp curves/intersect.cpp (`circleCrossings`, `curveCrossings`); its `lineCrossings` and
// `selfCrossings` are `CubicBezier::crossings(lineStart, lineEnd)` and `Path2D::selfCrossings`.
#pragma once

#include <optional>
#include <vector>

#include "CubicBezier.h"

namespace editor {

std::vector<CircleCrossing> circleCrossings(
    const CubicBezier<Vector2>& curve, Vector2 centre, float radius, int samples = 64);

std::optional<std::vector<CurveCrossing>> curveCrossings(
    const CubicBezier<Vector2>& first, const CubicBezier<Vector2>& second);

} // namespace editor
