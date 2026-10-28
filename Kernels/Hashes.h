// Hashes for shaders and procedural patterns, ported from graphics_metal's Core.metal: an integer
// hash that mixes bits (PCG style), so it is stable and free of the stripes a sine hash makes,
// and float hashes built on it that give 0…1. The results are bit-identical to graphics_metal's,
// so a pattern looks the same in both.
#pragma once

#include "KernelPrelude.h"
#include "KernelVectors.h"

namespace editor {
namespace kernels {

/// A well-mixed 32-bit number from a 32-bit number: nearby inputs give unrelated outputs.
///
/// Source: the PCG output permutation, Mark Jarzynski and Marc Olano, "Hash Functions for GPU
/// Rendering", JCGT 9(3), 2020 (the `pcg_hash` variant).
///
/// How: state = value × 747796405 + 2891336453 (a linear congruential step, modulo 2³²);
/// word = ((state >> ((state >> 28) + 4)) ^ state) × 277803737; result = (word >> 22) ^ word.
/// The top 4 bits of the state choose how far to shift. For 0: state = 0xAC564B05, shift
/// 10 + 4 = 14, state >> 14 = 0x2B159, ^ state = 0xAC54FA5C, × 277803737 mod 2³² = 0x07BB2FFC,
/// and 0x07BB2FFC ^ (0x07BB2FFC >> 22 = 0x1E) = 0x07BB2FE2.
///
/// Conventions: arithmetic is modulo 2³² on `unsigned int`; no input is special.
///
/// Use when: a random number per cell, per pixel or per particle that must not change between runs
/// Tags: hashUint, pcg hash, integer hash, random, noise seed, white noise
/// Node: none (shader library)
inline unsigned int hashUint(unsigned int value) {
    unsigned int state = value * 747796405u + 2891336453u;
    unsigned int word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

/// A hash of two numbers, made by hashing the second and mixing it into the first.
///
/// How: hashUint(x ^ hashUint(y)).
///
/// Use when: a random number per 2D cell
/// Tags: hashUint, hash2, 2D integer hash
/// Node: none (shader library)
inline unsigned int hashUint(Vector2Of<unsigned int> value) { return hashUint(value.x ^ hashUint(value.y)); }

/// A hash of three numbers: hashUint(x ^ hashUint(y ^ hashUint(z))).
///
/// Use when: a random number per 3D cell
/// Tags: hashUint, hash3, 3D integer hash
/// Node: none (shader library)
inline unsigned int hashUint(Vector3Of<unsigned int> value) {
    return hashUint(value.x ^ hashUint(value.y ^ hashUint(value.z)));
}

/// A hash of four numbers: hashUint(x ^ hashUint(y ^ hashUint(z ^ hashUint(w)))).
///
/// Use when: a random number per 4D cell, or per cell and per time step
/// Tags: hashUint, hash4, 4D integer hash
/// Node: none (shader library)
inline unsigned int hashUint(Vector4Of<unsigned int> value) {
    return hashUint(value.x ^ hashUint(value.y ^ hashUint(value.z ^ hashUint(value.w))));
}

/// A 32-bit hash as a float from 0 to 1.
///
/// How: hash / 4294967295, in float. The divisor rounds to 2³² in float, and a hash near 2³² rounds
/// up with it, so 1 can occur; 0 occurs for a hash of 0.
///
/// Use when: turning `hashUint` into a random number
/// Tags: hash to float, unit float, random 0 to 1
/// Node: none (shader library)
inline float unitFloatFromHash(unsigned int hash) { return static_cast<float>(hash) / 4294967295.0f; }

/// The grid cell a coordinate lies in, as an unsigned number: ⌊coordinate⌋ + 32768, so cells from
/// −32768 up are numbered 0 and up and every cell has its own number.
///
/// Conventions: a coordinate outside about ±2³¹ overflows the conversion to int; the caller keeps points near the origin.
///
/// Use when: hashing the cell a point is in
/// Tags: cell index, grid cell, floor to integer
/// Node: none (shader library)
inline unsigned int hashCellNumber(float coordinate) {
    return static_cast<unsigned int>(static_cast<int>(roundedDown(coordinate)) + 32768);
}

/// A random number from 0 to 1 per whole-number cell of `p`: every point of the cell [n, n + 1)
/// gives the same number, and neighbouring cells hash independently.
///
/// How: for a number, the bits of ⌊p⌋ + 0 (the + 0 turns −0 into 0) go through `hashUint`; for a
/// vector, `hashUint` of the cell numbers (`hashCellNumber`); then unitFloatFromHash.
///
/// Use when: white noise on a grid; a random value per tile
/// Tags: hash11, hash21, hash31, hash41, cell noise, random per cell, white noise
/// Node: none (shader library)
inline float hash11(float p) { return unitFloatFromHash(hashUint(bitsOf(roundedDown(p) + 0.0f))); }
inline float hash21(Vector2Of<float> p) {
    return unitFloatFromHash(hashUint(Vector2Of<unsigned int>(hashCellNumber(p.x), hashCellNumber(p.y))));
}
inline float hash31(Vector3Of<float> p) {
    return unitFloatFromHash(hashUint(Vector3Of<unsigned int>(hashCellNumber(p.x), hashCellNumber(p.y), hashCellNumber(p.z))));
}
inline float hash41(Vector4Of<float> p) {
    return unitFloatFromHash(hashUint(Vector4Of<unsigned int>(
        hashCellNumber(p.x), hashCellNumber(p.y), hashCellNumber(p.z), hashCellNumber(p.w))));
}

/// Two random numbers from 0 to 1 per cell of a 2D point.
///
/// How: h = hashUint(cell); the first number is h, the second hashUint(h), each as unitFloatFromHash.
///
/// Use when: a random 2D offset per cell, as in Voronoi and jittered grids
/// Tags: hash22, random vector, jitter, cell offset
/// Node: none (shader library)
inline Vector2Of<float> hash22(Vector2Of<float> p) {
    unsigned int h = hashUint(Vector2Of<unsigned int>(hashCellNumber(p.x), hashCellNumber(p.y)));
    return Vector2Of<float>(unitFloatFromHash(h), unitFloatFromHash(hashUint(h)));
}

/// Three random numbers from 0 to 1 per cell of a 3D point: h, hashUint(h), hashUint(hashUint(h)).
///
/// Use when: a random colour or direction per cell
/// Tags: hash33, random vector, random colour
/// Node: none (shader library)
inline Vector3Of<float> hash33(Vector3Of<float> p) {
    unsigned int h = hashUint(Vector3Of<unsigned int>(hashCellNumber(p.x), hashCellNumber(p.y), hashCellNumber(p.z)));
    unsigned int h2 = hashUint(h);
    return Vector3Of<float>(unitFloatFromHash(h), unitFloatFromHash(h2), unitFloatFromHash(hashUint(h2)));
}

/// Four random numbers from 0 to 1 per cell of a 4D point: h, then each the hash of the one before.
///
/// Use when: a random colour with alpha per cell
/// Tags: hash44, random vector, random rgba
/// Node: none (shader library)
inline Vector4Of<float> hash44(Vector4Of<float> p) {
    unsigned int h = hashUint(Vector4Of<unsigned int>(
        hashCellNumber(p.x), hashCellNumber(p.y), hashCellNumber(p.z), hashCellNumber(p.w)));
    unsigned int h2 = hashUint(h);
    unsigned int h3 = hashUint(h2);
    return Vector4Of<float>(unitFloatFromHash(h), unitFloatFromHash(h2), unitFloatFromHash(h3), unitFloatFromHash(hashUint(h3)));
}

/// A random number from 0 to 1 for a continuous value: different for every float, not only for every whole number.
///
/// How: the 32 bits of each float (`bitsOf`) go through `hashUint` (mixed as for vectors), then unitFloatFromHash.
///
/// Conventions: 0 and −0 hash differently, as they have different bits; NaNs hash by their bits.
///
/// Use when: a random value from a time or a position that varies smoothly
/// Tags: hashFloat, hash by bits, float hash, random from float
/// Node: none (shader library)
inline float hashFloat(float p) { return unitFloatFromHash(hashUint(bitsOf(p))); }
inline float hashFloat(Vector2Of<float> p) {
    return unitFloatFromHash(hashUint(Vector2Of<unsigned int>(bitsOf(p.x), bitsOf(p.y))));
}
inline float hashFloat(Vector3Of<float> p) {
    return unitFloatFromHash(hashUint(Vector3Of<unsigned int>(bitsOf(p.x), bitsOf(p.y), bitsOf(p.z))));
}
inline float hashFloat(Vector4Of<float> p) {
    return unitFloatFromHash(hashUint(Vector4Of<unsigned int>(bitsOf(p.x), bitsOf(p.y), bitsOf(p.z), bitsOf(p.w))));
}

} // namespace kernels
} // namespace editor
