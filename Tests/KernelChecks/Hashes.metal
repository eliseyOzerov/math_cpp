// Compiles Kernels/Hashes.h as Metal Shading Language (`make kernels`). Every function is called
// here; the hashes are plain functions on float and unsigned integer vectors.
#include <metal_stdlib>

#include "../../Kernels/Hashes.h"

using namespace editor::kernels;

kernel void hashesCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float value = float(index) / 64.0f - 0.5f;
    Vector2Of<float> plane(value, 1.5f);
    Vector3Of<float> space(value, 1.5f, -2.0f);
    Vector4Of<float> colour(value, 0.5f, 0.25f, 1.0f);
    unsigned int whole = hashUint(index);
    whole ^= hashUint(Vector2Of<unsigned int>(index, 1u));
    whole ^= hashUint(Vector3Of<unsigned int>(index, 1u, 2u));
    whole ^= hashUint(Vector4Of<unsigned int>(index, 1u, 2u, 3u));
    results[index * 8 + 0] = unitFloatFromHash(whole) + float(hashCellNumber(value));
    results[index * 8 + 1] = hash11(value) + hash21(plane) + hash31(space) + hash41(colour);
    results[index * 8 + 2] = hash22(plane).x + hash22(plane).y;
    results[index * 8 + 3] = hash33(space).x + hash33(space).y + hash33(space).z;
    results[index * 8 + 4] = hash44(colour).x + hash44(colour).y + hash44(colour).z + hash44(colour).w;
    results[index * 8 + 5] = hashFloat(value) + hashFloat(plane) + hashFloat(space) + hashFloat(colour);
}
