// Compiles Kernels/ShaderShapes.h as Metal Shading Language (`make kernels`). Templates are only
// checked once they are used, so each function is called here with `float`.
#include <metal_stdlib>

#include "../../Kernels/ShaderShapes.h"

using namespace editor::kernels;

kernel void shaderShapesCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f - 0.5f;
    Vector2Of<float> plane(x, 0.75f);
    Vector2Of<float> size(1.0f, 0.5f);
    Vector2Of<float> start(-1.0f, 0.0f);
    Vector2Of<float> middle(0.0f, 1.0f);
    Vector2Of<float> end(1.0f, 0.0f);

    results[index * 16 + 0] = regularPolygonDistance(plane, 6.0f, 1.0f, 0.1f) + polygonEdgeMask(x, 0.01f, 1.0f);
    results[index * 16 + 1] = rotatedShapePoint(plane, 30.0f).x + circleDistance(plane, 1.0f) + rectangleDistance(plane, size);
    results[index * 16 + 2] = roundedRectangleDistance(plane, size, 0.2f) + segmentDistance(plane, start, end) + ellipseDistance(plane, size);
    results[index * 16 + 3] = triangleDistance(plane, 1.0f) + ringDistance(plane, 1.0f, 0.2f);
    results[index * 16 + 4] = arcDistance(plane, 45.0f, 1.0f, 0.2f, false) + arcDistance(plane, 45.0f, 1.0f, 0.2f, true);
    results[index * 16 + 5] = starDistance(plane, 1.0f, 5.0f, 0.5f) + heartDistance(plane, 1.0f) + crossDistance(plane, 1.0f, 0.3f);
    results[index * 16 + 6] = moonDistance(plane, 1.0f, 1.0f, 0.8f) + vesicaDistance(plane, 1.0f, 0.6f) + eggDistance(plane, 1.0f, 0.5f);
    results[index * 16 + 7] = bezierDistance(plane, start, middle, end);
    results[index * 16 + 8] = polygonDistance(plane, start, middle, end, size, plane, start, 5.0f);
}
