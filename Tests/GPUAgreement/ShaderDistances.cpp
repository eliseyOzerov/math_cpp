// Kernels/ShaderDistances.h: the GPU (ShaderDistances.metal) against the CPU, for a rounded box
// (square roots over a 3D grid), a cone (several branches and a sign), a three-axis rotation
// (sines and cosines) and a stroke mask (a smooth step of a folded distance).
#include "../../Kernels/ShaderDistances.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector3Of;

namespace {
// Points on a 3D grid of irregular steps with `parameters` appended to every point.
std::vector<float> pointsWithParameters(const std::vector<std::vector<float>>& parameterSets, int perAxis, float extent) {
    std::vector<float> inputs;
    for (const std::vector<float>& parameters : parameterSets) {
        for (int i = 0; i < perAxis; i += 1) {
            for (int j = 0; j < perAxis; j += 1) {
                for (int k = 0; k < perAxis; k += 1) {
                    inputs.push_back(extent * (2.0f * (static_cast<float>(i) + 0.3137f) / static_cast<float>(perAxis) - 1.0f));
                    inputs.push_back(extent * (2.0f * (static_cast<float>(j) + 0.2719f) / static_cast<float>(perAxis) - 1.0f));
                    inputs.push_back(extent * (2.0f * (static_cast<float>(k) + 0.4421f) / static_cast<float>(perAxis) - 1.0f));
                    for (float parameter : parameters) inputs.push_back(parameter);
                }
            }
        }
    }
    return inputs;
}
} // namespace

AGREEMENT({
    // Sharp and rounded boxes; points inside, on faces and outside.
    "boxDistanceAgreement", 7, 1,
    pointsWithParameters({{1.0f, 0.7f, 1.4f, 0.0f}, {1.0f, 0.7f, 1.4f, 0.4f}, {0.5f, 0.5f, 0.5f, 0.5f}}, 14, 3.0f),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::boxDistance(Vector3Of<float>(inputs[0], inputs[1], inputs[2]),
            Vector3Of<float>(inputs[3], inputs[4], inputs[5]), inputs[6]);
    },
    2e-6f, 2e-6f,
});

AGREEMENT({
    // The sign flips at the surface, but the distance itself is continuous there.
    "coneDistanceAgreement", 5, 1,
    pointsWithParameters({{1.0f, 2.0f}, {0.6f, 1.0f}, {1.5f, 0.5f}}, 14, 3.0f),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::coneDistance(Vector3Of<float>(inputs[0], inputs[1], inputs[2]), inputs[3], inputs[4]);
    },
    1e-5f, 1e-5f,
});

AGREEMENT({
    "rotatedPointAgreement", 6, 3,
    pointsWithParameters({{30.0f, 0.0f, 0.0f}, {0.0f, 75.0f, -40.0f}, {120.0f, -33.0f, 210.0f}}, 10, 2.0f),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> turned = editor::kernels::rotatedPoint(Vector3Of<float>(inputs[0], inputs[1], inputs[2]),
            Vector3Of<float>(inputs[3], inputs[4], inputs[5]));
        outputs[0] = turned.x;
        outputs[1] = turned.y;
        outputs[2] = turned.z;
    },
    3e-5f, 1e-5f,
});

AGREEMENT({
    // Distances across the band for all three alignments and a few widths and feathers.
    "distanceStrokeMaskAgreement", 5, 1,
    [] {
        std::vector<float> inputs;
        for (int alignment = 0; alignment < 3; alignment += 1) {
            for (float width : {0.0f, 0.1f, 0.4f}) {
                for (float feather : {1.0f, 3.0f}) {
                    for (float distance : evenlySpaced(-0.6f, 0.6f, 241)) {
                        inputs.insert(inputs.end(), {distance, width, static_cast<float>(alignment), 0.01f, feather});
                    }
                }
            }
        }
        return inputs;
    }(),
    [](const float* inputs, float* outputs) {
        outputs[0] = editor::kernels::distanceStrokeMask(inputs[0], inputs[1], static_cast<int>(inputs[2]), inputs[3], inputs[4]);
    },
    1e-6f, 1e-6f,
});
