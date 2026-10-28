// Kernels/ShaderCoordinates.h: the GPU (ShaderCoordinates.metal) against the CPU, for a kaleidoscope
// fold (atan2, modulo, sine and cosine), a twirl (rotation by a computed angle) and the hexagonal
// grid (floors and a nearest-centre choice).
#include "../../Kernels/ShaderCoordinates.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector2Of;
using editor::kernels::Vector4Of;

namespace {
// Points on a grid of irregular step, so none sits on a cell or wedge boundary where the GPU's
// faster division could land a rounding step on the other side.
std::vector<float> pointsWithParameters(const std::vector<float>& parameterSets, int parametersPerSet, float extent, int perAxis) {
    std::vector<float> inputs;
    for (size_t set = 0; set + parametersPerSet <= parameterSets.size(); set += parametersPerSet) {
        for (int column = 0; column < perAxis; column += 1) {
            for (int row = 0; row < perAxis; row += 1) {
                inputs.push_back(extent * (2.0f * (static_cast<float>(column) + 0.4137f) / static_cast<float>(perAxis) - 1.0f));
                inputs.push_back(extent * (2.0f * (static_cast<float>(row) + 0.2719f) / static_cast<float>(perAxis) - 1.0f));
                for (int parameter = 0; parameter < parametersPerSet; parameter += 1) {
                    inputs.push_back(parameterSets[set + parameter]);
                }
            }
        }
    }
    return inputs;
}

// Points -3 … 3 on a grid whose steps are not simple fractions of the cell size (1, √3).
std::vector<float> pointsOnAGrid() {
    std::vector<float> inputs;
    for (int column = 0; column < 70; column += 1) {
        for (int row = 0; row < 70; row += 1) {
            inputs.push_back(-3.0f + 0.0861f * static_cast<float>(column) + 0.0123f);
            inputs.push_back(-3.0f + 0.0877f * static_cast<float>(row) + 0.0231f);
        }
    }
    return inputs;
}
} // namespace

AGREEMENT({
    // 3, 6 and 7 segments with two start angles. The fold is continuous across wedge boundaries, so
    // a point near one cannot disagree wildly. atan2 is the least exact step on the GPU.
    "kaleidoscopeFoldedAgreement", 4, 2,
    pointsWithParameters({3.0f, 0.0f, 6.0f, 0.5f, 7.0f, -1.2f}, 2, 2.0f, 40),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> folded = editor::kernels::kaleidoscopeFolded(Vector2Of<float>(inputs[0], inputs[1]), inputs[2], inputs[3]);
        outputs[0] = folded.x;
        outputs[1] = folded.y;
    },
    3e-5f, 1e-5f,
});

AGREEMENT({
    // Inside and outside the radius, both turn directions.
    "twirledAgreement", 4, 2,
    pointsWithParameters({4.0f, 1.0f, -3.0f, 1.5f, 8.0f, 2.5f}, 2, 2.0f, 40),
    [](const float* inputs, float* outputs) {
        Vector2Of<float> twisted = editor::kernels::twirled(Vector2Of<float>(inputs[0], inputs[1]), inputs[2], inputs[3]);
        outputs[0] = twisted.x;
        outputs[1] = twisted.y;
    },
    3e-5f, 1e-5f,
});

AGREEMENT({
    "hexagonalGridAgreement", 2, 4,
    pointsOnAGrid(),
    [](const float* inputs, float* outputs) {
        Vector4Of<float> cell = editor::kernels::hexagonalGrid(Vector2Of<float>(inputs[0], inputs[1]));
        outputs[0] = cell.x;
        outputs[1] = cell.y;
        outputs[2] = cell.z;
        outputs[3] = cell.w;
    },
    2e-6f, 1e-6f,
});
