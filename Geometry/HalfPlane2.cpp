// Clipping a polygon by a half-plane (see HalfPlane2.h).
#include "HalfPlane2.h"

namespace editor {

std::vector<Vector2> HalfPlane2::clipped(const std::vector<Vector2>& corners) const {
    std::vector<Vector2> kept;
    for (size_t index = 0; index < corners.size(); index += 1) {
        Vector2 current = corners[index];
        Vector2 next = corners[(index + 1) % corners.size()];
        float currentSide = signedDistance(current);
        float nextSide = signedDistance(next);
        if (currentSide >= 0) kept.push_back(current);
        if ((currentSide >= 0) != (nextSide >= 0)) {
            kept.push_back(Vector2::linearBlend(current, next, currentSide / (currentSide - nextSide)));
        }
    }
    return kept;
}

} // namespace editor
