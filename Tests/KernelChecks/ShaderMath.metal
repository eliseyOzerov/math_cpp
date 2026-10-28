// Compiles Kernels/ShaderMath.h as Metal Shading Language (`make kernels`). Templates are only
// checked once they are used, so each function is called here with `float`, and the generic ones
// (written once for a number and for vectors) also with `float3`.
#include <metal_stdlib>

#include "../../Kernels/ShaderMath.h"

using namespace editor::kernels;

kernel void shaderMathCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f - 0.5f;
    Vector2Of<float> plane(x, 1.5f);
    Vector3Of<float> v(x, 1.5f, -2.0f);
    Vector3Of<float> w(0.5f, -1.0f, 3.0f);
    Vector4Of<float> colour(x, 0.5f, 0.25f, 0.75f);
    Vector4Of<int> which(3, 2, 4, 5);

    // Constants and Core.metal arithmetic.
    results[index * 32 + 0] = pi<float>() + tau<float>() + eulerNumber<float>() + squareRootOfTwo<float>()
        + goldenRatio<float>() + namedConstant<float>(int(index % 6)) + smallestSafeDivisor<float>() + largestFiniteFloat<float>();
    results[index * 32 + 1] = flooredModulo(x, 0.75f) + flooredModulo(v, w).y + flooredModulo(v, 2.0f).z + degreesToRadians(x);
    results[index * 32 + 2] = safeDivide(x, 0.0f) + safeDivide(v, w).z + safeModulo(x, 0.7f) + safeModulo(v, w).x;
    results[index * 32 + 3] = luminance(v) + luminance(colour) + rotatedCounterClockwise(plane, x).y;

    // Ranges and smoothing.
    results[index * 32 + 4] = remapped(x, 0.0f, 1.0f, 10.0f, 20.0f) + remapped(v, w, w * 2.0f, v, w).x
        + remappedClamped(x, 0.0f, 1.0f, 20.0f, 10.0f) + remappedClamped(v, w, w * 2.0f, v, w).y;
    results[index * 32 + 5] = safeSmoothStep(0.0f, 1.0f, x) + safeSmoothStep(w, w * 2.0f, v).z
        + smootherStep(0.0f, 1.0f, x) + smootherStep(w, w * 2.0f, v).x;
    results[index * 32 + 6] = inverseLinearBlend(1.0f, 3.0f, x) + inverseLinearBlend(v, w, v).y
        + snappedToMultiple(x, 0.25f) + snappedToMultiple(v, w).z;
    results[index * 32 + 7] = pingPong(x, 2.0f) + pingPong(v, w).x + wrappedInto(x, 0.0f, 1.0f) + wrappedInto(v, v - 1.0f, w).y;
    results[index * 32 + 8] = roundedHalfUp(x) + roundedHalfUp(v).y + safeReciprocal(x) + safeReciprocal(v).z;
    results[index * 32 + 9] = safeLogarithm(x, 2.0f) + safeLogarithm(v, 10.0f).y + safePower(x, 2.0f) + safePower(v, w).z;
    results[index * 32 + 10] = smoothMinimum(x, 0.25f, 0.5f) + smoothMinimum(v, w, 0.5f).x
        + smoothMaximum(x, 0.25f, 0.5f) + smoothMaximum(v, w, 0.5f).y;

    // Comparison, logic and flags.
    results[index * 32 + 11] = comparisonResult(x, 0.0f, int(index % 6), 0.01f) + comparisonResult(v, w, 2, w).x;
    results[index * 32 + 12] = logicalAnd(x, 0.75f) + logicalOr(x, 0.0f) + logicalExclusiveOr(x, 1.0f) + logicalNot(x);
    results[index * 32 + 13] = logicalAnd(v, w).y + logicalOr(v, w).z + logicalExclusiveOr(v, w).x + logicalNot(v).y;
    results[index * 32 + 14] = flagNotANumber(x) + flagInfinity(x) + flagNotANumber(v).y + flagInfinity(v).z;

    // Easing and waves.
    results[index * 32 + 15] = bounceEaseOut(x + 0.5f) + easedCurve(x + 0.5f, int(index % 20));
    results[index * 32 + 16] = waveShape(x, int(index % 4)) + waveShape(v, int(index % 4)).y;
    results[index * 32 + 17] = keyframesOfFour(x, colour, 2.0f, int(index % 3));

    // Vector helpers that also take numbers.
    results[index * 32 + 18] = dotProductOfValues(x, 2.0f) + dotProductOfValues(v, w) + lengthOfValue(x) + lengthOfValue(v);
    results[index * 32 + 19] = distanceBetweenValues(x, 1.0f) + distanceBetweenValues(v, w)
        + normalizedValue(x) + normalizedValue(v).y + normalizedValue(plane).x;
    results[index * 32 + 20] = refractedValue(x, 1.0f, 0.7f) + refractedValue(normalizedValue(v), normalizedValue(w), 0.7f).z;
    results[index * 32 + 21] = projectedValue(x, 2.0f) + projectedValue(v, w).y
        + angleBetweenValues(x, 2.0f) + angleBetweenValues(v, w);
    results[index * 32 + 22] = componentOf(colour, int(index % 5)) + swizzled(colour, which).x + swizzled(colour, which).w
        + selectedByIndex(x * 4.0f, 1.0f, 2.0f, 3.0f, 4.0f) + selectedByIndex(x * 4.0f, v, w, v, w).z;

    // Debug colours.
    results[index * 32 + 23] = heatmap(x).y + debugColor(x, int(index % 3)).x + debugColor(plane, 2).x
        + debugColor(v, 2).y + debugColor(colour, 2).z;
    results[index * 32 + 24] = layerOver(colour, colour * 0.5f).x + layersOver(colour, colour, colour, colour).w;
}
