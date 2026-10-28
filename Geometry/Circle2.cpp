// The circle through three points (see Circle2.h).
#include "Circle2.h"

#include <algorithm>
#include <cmath>

namespace editor {

std::optional<Circle2> Circle2::through(Vector2 first, Vector2 second, Vector2 third) {
    Vector2 toSecond = second - first;
    Vector2 toThird = third - first;
    float twiceArea = toSecond.cross(toThird);
    float longestSide = std::max({toSecond.length(), toThird.length(), (third - second).length()});
    if (!(std::fabs(twiceArea) >= 1e-6f * longestSide * longestSide) || !(longestSide > 0)) return std::nullopt;
    float denominator = 2 * twiceArea;
    float secondSquared = toSecond.lengthSquared();
    float thirdSquared = toThird.lengthSquared();
    Vector2 centreOffset((toThird.y * secondSquared - toSecond.y * thirdSquared) / denominator,
                         (toSecond.x * thirdSquared - toThird.x * secondSquared) / denominator);
    return Circle2{first + centreOffset, centreOffset.length()};
}

} // namespace editor
