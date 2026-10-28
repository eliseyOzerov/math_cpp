// Kernels/ShaderShapes.h: the GPU (ShaderShapes.metal) against the CPU, for the cases with the most
// steps that could differ: the Bézier distance (a cubic solved with a cube root or a cosine), the
// star (atan2 and a fold), the polygon (an array and a loop) and the heart (branches and roots).
#include "../../Kernels/ShaderShapes.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector2Of;

namespace {
// Points on a square grid of irregular steps with each of `parameterSets` appended to every point.
std::vector<float> pointsWithParameters(const std::vector<std::vector<float>>& parameterSets, int perAxis, float extent) {
    std::vector<float> inputs;
    for (const std::vector<float>& parameters : parameterSets) {
        for (int column = 0; column < perAxis; column += 1) {
            for (int row = 0; row < perAxis; row += 1) {
                inputs.push_back(extent * (2.0f * (static_cast<float>(column) + 0.4137f) / static_cast<float>(perAxis) - 1.0f));
                inputs.push_back(extent * (2.0f * (static_cast<float>(row) + 0.2719f) / static_cast<float>(perAxis) - 1.0f));
                for (float parameter : parameters) inputs.push_back(parameter);
            }
        }
    }
    return inputs;
}
} // namespace

AGREEMENT({
    // A symmetric parabola, a lopsided curve and a nearly straight one (the straight-line shortcut
    // and the cubic's one-root and three-root branches). The cubic is ill-conditioned when the curve is
    // nearly straight (its coefficients divide by |a − 2b + c|², 0.05 for the third curve): there
    // even the CPU is 1.2e-4 off the true distance (sampled 2 million points on the curve), and
    // Metal's fast pow/acos add up to 5.1e-4 more (measured). So the tolerance, 1e-3, is the
    // loosest in this file; it still catches a wrong formula, which is off by far more.
    "bezierDistanceAgreement", 8, 1,
    pointsWithParameters({{-1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f}, {-0.5f, -0.2f, 1.4f, 1.1f, 0.8f, -0.9f}, {-1.0f, 0.0f, 0.1f, 0.2f, 1.0f, 0.5f}}, 60,
        2.0f),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::bezierDistance(Vector2Of<float>(inputs[0], inputs[1]), Vector2Of<float>(inputs[2], inputs[3]),
            Vector2Of<float>(inputs[4], inputs[5]), Vector2Of<float>(inputs[6], inputs[7]));
    },
    1e-3f, 1e-4f,
});

AGREEMENT({
    "starDistanceAgreement", 5, 1,
    pointsWithParameters({{1.0f, 5.0f, 0.0f}, {1.0f, 5.0f, 0.5f}, {0.8f, 7.0f, 0.8f}, {1.2f, 3.0f, 0.3f}}, 60, 1.6f),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::starDistance(Vector2Of<float>(inputs[0], inputs[1]), inputs[2], inputs[3], inputs[4]);
    },
    3e-5f, 1e-5f,
});

AGREEMENT({
    // A triangle, a square and a concave L, each given with all six points (unused ones repeat).
    "polygonDistanceAgreement", 15, 1,
    pointsWithParameters(
        {{0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f},
         {0.0f, 0.0f, 2.0f, 0.0f, 2.0f, 2.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f},
         {0.0f, 0.0f, 2.0f, 0.0f, 2.0f, 1.0f, 1.0f, 1.0f, 1.0f, 2.0f, 0.0f, 2.0f, 6.0f}},
        60, 2.5f),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::polygonDistance(Vector2Of<float>(inputs[0], inputs[1]), Vector2Of<float>(inputs[2], inputs[3]),
            Vector2Of<float>(inputs[4], inputs[5]), Vector2Of<float>(inputs[6], inputs[7]), Vector2Of<float>(inputs[8], inputs[9]),
            Vector2Of<float>(inputs[10], inputs[11]), Vector2Of<float>(inputs[12], inputs[13]), inputs[14]);
    },
    2e-6f, 2e-6f,
});

AGREEMENT({
    "heartDistanceAgreement", 3, 1,
    pointsWithParameters({{1.0f}, {2.5f}}, 80, 2.0f),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::heartDistance(Vector2Of<float>(inputs[0], inputs[1]), inputs[2]);
    },
    2e-6f, 2e-6f,
});
