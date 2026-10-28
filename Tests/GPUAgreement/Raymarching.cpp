// Kernels/Raymarching.h: the GPU (Raymarching.metal) against the CPU, for a sphere trace with the
// normal at its end, ambient occlusion with a soft shadow, and the look-at camera. The scene is the
// unit sphere, a C++ lambda here and a struct with an operator() in Metal (Metal has no lambdas).
// Rays are chosen well away from grazing the sphere, where a hit and a miss would be a coin toss for
// the last bit; the sphere trace's steps have fast math rounding (a few 1e-7 each) added over about
// ten steps, and the normal differences two nearly equal distances 0.002 apart, which magnifies the
// rounding by about 500, so its tolerance is 1e-3 (measured differences: 2.7e-4 sphere trace and normal, 1.5e-6 occlusion and shadow, 1.2e-7 camera).
#include "../../Kernels/Raymarching.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::Vector2Of;
using editor::kernels::Vector3Of;

namespace {
float unitSphere(Vector3Of<float> p) { return editor::kernels::lengthOf(p) - 1.0f; }

/// Rays from z = −3 along +z with offsets that hit (inside radius 0.85) or miss (beyond 1.5) the sphere.
std::vector<float> rays() {
    std::vector<float> inputs;
    for (float x : {-1.6f, -0.6f, -0.2f, 0.3f, 0.6f, 1.7f}) {
        for (float y : {-1.8f, -0.55f, 0.1f, 0.45f, 1.6f}) {
            for (float directionX : {0.0f, 0.05f}) {
                float row[6] = {x, y, -3.0f, directionX, 0.0f, 1.0f};
                inputs.insert(inputs.end(), row, row + 6);
            }
        }
    }
    return inputs;
}

/// Points around the sphere, a little outside it, so the occlusion samples and the shadow ray start in the open.
std::vector<float> surroundings() {
    std::vector<float> inputs;
    for (float polar : evenlySpaced(0.3f, 2.8f, 9)) {
        for (float azimuth : evenlySpaced(0.1f, 6.1f, 13)) {
            float radius = 1.15f;
            float row[3] = {radius * std::sin(polar) * std::cos(azimuth), radius * std::cos(polar), radius * std::sin(polar) * std::sin(azimuth)};
            inputs.insert(inputs.end(), row, row + 3);
        }
    }
    return inputs;
}
} // namespace

AGREEMENT({
    "sphereTraceAgreement", 6, 6, rays(),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> origin(inputs[0], inputs[1], inputs[2]);
        Vector3Of<float> direction(inputs[3], inputs[4], inputs[5]);
        auto sphere = [](Vector3Of<float> p) { return unitSphere(p); };
        editor::kernels::SphereTraceResult result = editor::kernels::sphereTrace(sphere, origin, direction, 20.0f, 64.0f, 0.001f);
        Vector3Of<float> normal = editor::kernels::surfaceNormal(sphere, result.position, 0.001f);
        outputs[0] = result.hit ? 1.0f : 0.0f;
        outputs[1] = result.travel;
        outputs[2] = result.stepsUsed;
        outputs[3] = normal.x;
        outputs[4] = normal.y;
        outputs[5] = normal.z;
    },
    1e-3f, 1e-5f,
});

AGREEMENT({
    "occlusionAndShadowAgreement", 3, 2, surroundings(),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> point(inputs[0], inputs[1], inputs[2]);
        Vector3Of<float> normal = editor::kernels::normalized(point);
        auto sphere = [](Vector3Of<float> p) { return unitSphere(p); };
        outputs[0] = editor::kernels::ambientOcclusion(sphere, point, normal, 0.4f, 1.0f);
        outputs[1] = editor::kernels::softShadow(sphere, point, normal, Vector3Of<float>(1.0f, 2.0f, -1.0f), 8.0f, 10.0f);
    },
    1e-4f, 1e-5f,
});

AGREEMENT({
    "lookAtCameraRayAgreement", 2, 3, everyPair(evenlySpaced(-0.5f, 0.5f, 21), evenlySpaced(-0.5f, 0.5f, 21)),
    [](const float* inputs, float* outputs) {
        Vector3Of<float> ray = editor::kernels::lookAtCameraRay(Vector2Of<float>(inputs[0], inputs[1]),
                                                                 Vector3Of<float>(2.0f, 1.0f, -4.0f), Vector3Of<float>(0.0f), 50.0f);
        outputs[0] = ray.x;
        outputs[1] = ray.y;
        outputs[2] = ray.z;
    },
    2e-6f, 1e-6f,
});
