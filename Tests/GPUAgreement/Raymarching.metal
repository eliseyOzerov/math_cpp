// Kernels/Raymarching.h on the GPU, for Raymarching.cpp beside it. Metal has no lambdas, so the
// scene is a struct with an operator().
#include <metal_stdlib>

#include "../../Kernels/Raymarching.h"

using namespace editor::kernels;

/// The unit sphere at the origin: |p| − 1.
struct UnitSphereScene {
    float operator()(Vector3Of<float> p) const { return lengthOf(p) - 1.0f; }
};

/// Inputs per item: (originX, originY, originZ, directionX, directionY, directionZ). Outputs: hit (1 or 0),
/// travel, stepsUsed and the surface normal at the stopping point (x, y, z).
kernel void sphereTraceAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                 uint item [[thread_position_in_grid]]) {
    UnitSphereScene sphere;
    Vector3Of<float> origin(inputs[item * 6 + 0], inputs[item * 6 + 1], inputs[item * 6 + 2]);
    Vector3Of<float> direction(inputs[item * 6 + 3], inputs[item * 6 + 4], inputs[item * 6 + 5]);
    SphereTraceResult result = sphereTrace(sphere, origin, direction, 20.0f, 64.0f, 0.001f);
    Vector3Of<float> normal = surfaceNormal(sphere, result.position, 0.001f);
    outputs[item * 6 + 0] = result.hit ? 1.0f : 0.0f;
    outputs[item * 6 + 1] = result.travel;
    outputs[item * 6 + 2] = result.stepsUsed;
    outputs[item * 6 + 3] = normal.x;
    outputs[item * 6 + 4] = normal.y;
    outputs[item * 6 + 5] = normal.z;
}

/// Inputs per item: (x, y, z) of a point near the sphere. Outputs: ambientOcclusion and softShadow there,
/// the normal being the direction from the centre and the light (1, 2, -1).
kernel void occlusionAndShadowAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                        uint item [[thread_position_in_grid]]) {
    UnitSphereScene sphere;
    Vector3Of<float> point(inputs[item * 3 + 0], inputs[item * 3 + 1], inputs[item * 3 + 2]);
    Vector3Of<float> normal = normalized(point);
    outputs[item * 2 + 0] = ambientOcclusion(sphere, point, normal, 0.4f, 1.0f);
    outputs[item * 2 + 1] = softShadow(sphere, point, normal, Vector3Of<float>(1.0f, 2.0f, -1.0f), 8.0f, 10.0f);
}

/// Inputs per item: (screenX, screenY). Outputs: lookAtCameraRay from (2, 1, -4) to the origin, 50 degrees.
kernel void lookAtCameraRayAgreement(device const float* inputs [[buffer(0)]], device float* outputs [[buffer(1)]],
                                     uint item [[thread_position_in_grid]]) {
    Vector3Of<float> ray = lookAtCameraRay(Vector2Of<float>(inputs[item * 2 + 0], inputs[item * 2 + 1]),
                                           Vector3Of<float>(2.0f, 1.0f, -4.0f), Vector3Of<float>(0.0f), 50.0f);
    outputs[item * 3 + 0] = ray.x;
    outputs[item * 3 + 1] = ray.y;
    outputs[item * 3 + 2] = ray.z;
}
