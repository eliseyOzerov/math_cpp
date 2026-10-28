// Compiles Kernels/ShaderCoordinates.h as Metal Shading Language (`make kernels`). Templates are
// only checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/ShaderCoordinates.h"

using namespace editor::kernels;

kernel void shaderCoordinatesCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f - 0.5f;
    Vector2Of<float> plane(x, 1.5f);
    Vector3Of<float> column(x, 0.5f, 1.0f);

    Vector2Of<float> mirrored = mirroredUv(plane, int(index % 3)) + flippedUv(plane, int(index % 3));
    Vector2Of<float> folded = kaleidoscopeFolded(plane, 6.0f, x) + twirled(plane, 2.0f, 3.0f) + spherized(plane, 0.5f, 3.0f);
    Vector2Of<float> gridded = rowOffsetGrid(plane, plane, 0.5f, int(index % 2)) + pixelatedUv(plane, 8.0f, 1.5f);
    Vector4Of<float> hexagon = hexagonalGrid(plane);
    Vector4Of<float> triangle = triangularGrid(plane);
    Vector2Of<float> moved = transformedByMatrix3(plane, column, column * 2.0f, column + 1.0f);

    results[index * 8 + 0] = mirrored.x + mirrored.y + folded.x + folded.y;
    results[index * 8 + 1] = gridded.x + gridded.y + moved.x + moved.y;
    results[index * 8 + 2] = hexagon.x + hexagon.y + hexagon.z + hexagon.w;
    results[index * 8 + 3] = triangle.x + triangle.y + triangle.z + triangle.w;
    results[index * 8 + 4] = hexagonalEdgeDistance(plane);
}
