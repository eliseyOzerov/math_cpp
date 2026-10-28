// Compiles Kernels/ShaderDistances.h as Metal Shading Language (`make kernels`). Templates are
// only checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/ShaderDistances.h"

using namespace editor::kernels;

kernel void shaderDistancesCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f - 0.5f;
    Vector2Of<float> plane(x, 1.5f);
    Vector3Of<float> space(x, 1.5f, -2.0f);
    Vector3Of<float> size(1.0f, 2.0f, 3.0f);

    results[index * 16 + 0] = signOf(x) + signOf(plane).y;
    results[index * 16 + 1] = distanceFillMask(x, 0.01f, 1.0f) + distanceStrokeMask(x, 0.2f, int(index % 3), 0.01f, 2.0f);
    results[index * 16 + 2] = distanceGlow(x, 0.5f, 1.0f) + distanceShadow(x, plane, plane, 0.1f, 0.5f, 0.01f);
    results[index * 16 + 3] = smoothUnion(x, 0.25f, 0.5f) + smoothSubtraction(x, 0.25f, 0.5f) + smoothIntersection(x, 0.25f, 0.5f);
    Vector4Of<float> cell = repeatedCell2(plane, 1.0f, 2.0f);
    results[index * 16 + 4] = cell.x + cell.y + cell.z + cell.w;
    results[index * 16 + 5] = sphereDistance(space, 1.0f) + boxDistance(space, size, 0.25f) + torusDistance(space, 2.0f, 0.5f);
    results[index * 16 + 6] = cylinderDistance(space, 1.0f, 2.0f) + coneDistance(space, 1.0f, 2.0f) + capsuleDistance(space, 0.5f, 2.0f);
    results[index * 16 + 7] = planeDistance(space, size, 1.0f);
    Vector3Of<float> placed = translatedPoint(space, size) + rotatedPoint(space, size * 10.0f)
        + repeatedPoint(space, 2.0f, 0.0f) + repeatedCell3(space, 2.0f, 3.0f) + twistedPoint(space, 45.0f);
    results[index * 16 + 8] = placed.x + placed.y + placed.z;
}
