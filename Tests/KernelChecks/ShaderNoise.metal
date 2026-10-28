// Compiles Kernels/ShaderNoise.h as Metal Shading Language (`make kernels`). Every function is called here.
#include <metal_stdlib>

#include "../../Kernels/ShaderNoise.h"

using namespace editor::kernels;

kernel void shaderNoiseCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f - 0.5f;
    Vector2Of<float> plane(x, 1.5f);
    Vector3Of<float> space(x, 1.5f, -2.0f);
    Vector4Of<float> spaceTime(x, 1.5f, -2.0f, 0.25f);
    int type = int(index % 3);

    results[index * 16 + 0] = quinticSmoothing(x) + quinticSmoothing(space).y + gradientAtCell(space).z;
    results[index * 16 + 1] = valueNoise3(space) + perlinNoise3(space) + simplexNoise3(space) + baseNoise(space, type);
    results[index * 16 + 2] = perlinNoise4(spaceTime) + loopingNoise(plane, x, 4.0f, 0.8f);
    results[index * 16 + 3] = fractalNoise(space, type, int(index % 4), 5.0f, 2.0f, 0.5f);
    Vector3Of<float> warped = domainWarpedNoise(plane, 0.5f, 0.6f, type, 3.0f, 2.0f, 0.5f);
    results[index * 16 + 4] = warped.x + warped.y + warped.z;
    Vector2Of<float> curl = curlNoise(plane, 0.5f);
    results[index * 16 + 5] = curl.x + curl.y + flowNoise(plane, x);
    results[index * 16 + 6] = gaborNoise(plane, 3.0f, 1.0f, 0.5f, 0.5f) + bandLimitedNoise(space);
    CellularResult cells = cellularNoise(plane, 1.0f, 0.5f, type);
    results[index * 16 + 7] = cells.nearest + cells.second + cells.id + cells.nearestPoint.x + cells.nearestPoint.y;
    results[index * 16 + 8] = cellularDistance(plane, type) + cellularFeaturePoint(plane, 0.5f, 0.5f).x
        + cellularBorderDistance(plane, 1.0f, 0.5f);
    results[index * 16 + 9] = whiteNoise(plane, plane, 8.0f, 3.0f, int(index % 2))
        + whiteNoiseVector3(plane, plane, 8.0f, 3.0f, int(index % 2)).y;
}
