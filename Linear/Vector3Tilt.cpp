// A direction tilted away from a parent (Engine/Math/Algebra/Vector3+Tilt.swift).
#include "Vector3Tilt.h"

#include <limits>

namespace editor {

Vector3 directionTiltedAwayFrom(Vector3 parent, Angle tilt, Angle turn) {
    std::optional<Vector3> axis = parent.unitDirection();
    if (!axis) {
        float notANumber = std::numeric_limits<float>::quiet_NaN();
        return Vector3(notANumber, notANumber, notANumber);
    }
    Vector3 side = Quaternion::rotation(Vector3::zAxis(), *axis).rotate(Vector3(turn.cosine(), turn.sine(), 0));
    return *axis * tilt.cosine() + side * tilt.sine();
}

} // namespace editor
