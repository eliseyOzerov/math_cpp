// The hash kernels (Kernels/Hashes.h) on the CPU: the integer hash against values derived by hand
// from the PCG formula, the float hashes against those, and the properties every hash must have.
// The same header is compiled as Metal by `make kernels`; graphics_metal's results are these.
#include <cmath>
#include <limits>

#include "../Kernels/Hashes.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Cell2 = kernels::Vector2Of<float>;
using Cell3 = kernels::Vector3Of<float>;
using Cell4 = kernels::Vector4Of<float>;

// A 32-bit hash as a float the way the kernel does it: float(hash) / 4294967295, which is exact
// division by 2³² once the divisor rounds in float.
float unitFloat(unsigned int hash) { return static_cast<float>(hash) / 4294967296.0f; }
} // namespace

TEST(hashUintOfZeroFollowsThePcgFormulaStepByStep) {
    // state = 0 × 747796405 + 2891336453 = 0xAC564B05; state >> 28 = 0xA = 10, so the shift is 14.
    // state >> 14 = 0x2B159; 0x2B159 ^ 0xAC564B05 = 0xAC54FA5C;
    // × 277803737 (0x108EF2D9) mod 2³² = 0x07BB2FFC; (0x07BB2FFC >> 22 = 0x1E) ^ 0x07BB2FFC = 0x07BB2FE2.
    EXPECT(kernels::hashUint(0u) == 0x07BB2FE2u);
}

TEST(hashUintMatchesValuesDerivedFromTheFormula) {
    // Each is worked as for 0: state, shift, state >> shift, xor with state, × 277803737, final xor.
    // 1: state 0xD8E8C2BA, shift 13 + 4 = 17, >> 17 = 0x6C74, xor = 0xD8E8AECE, word = 0xA8BEE89E,
    //    word ^ (word >> 22 = 0x2A2) = 0xA8BEEA3C.
    EXPECT(kernels::hashUint(1u) == 0xA8BEEA3Cu);
    // 2: state 0x057B3A6F, shift 0 + 4 = 4, >> 4 = 0x57B3A6, xor = 0x052C89C9, word = 0x7A7ECD61,
    //    word ^ (word >> 22 = 0x1E9) = 0x7A7ECC88.
    EXPECT(kernels::hashUint(2u) == 0x7A7ECC88u);
    // 3: state 0x320DB224, shift 3 + 4 = 7, >> 7 = 0x641B64, xor = 0x3269A940, word = 0x7F0EF740,
    //    word ^ (word >> 22 = 0x1FC) = 0x7F0EF6BC.
    EXPECT(kernels::hashUint(3u) == 0x7F0EF6BCu);
    // The largest input wraps the state to 0x7FC3D350, shift 7 + 4 = 11, xor = 0x7FCC2B2A,
    // word = 0xE62A4A9A, final = 0xE62A4902.
    EXPECT(kernels::hashUint(0xFFFFFFFFu) == 0xE62A4902u);
}

TEST(hashUintOfVectorsHashesTheLastComponentFirst) {
    using kernels::hashUint;
    using Uint2 = kernels::Vector2Of<unsigned int>;
    using Uint3 = kernels::Vector3Of<unsigned int>;
    using Uint4 = kernels::Vector4Of<unsigned int>;
    // hash(1, 2) = hash(1 ^ hash(2)) = hash(1 ^ 0x7A7ECC88) = hash(0x7A7ECC89) = 0x7DEDA5D8
    EXPECT(hashUint(Uint2(1u, 2u)) == hashUint(1u ^ hashUint(2u)));
    EXPECT(hashUint(Uint2(1u, 2u)) == 0x7DEDA5D8u);
    // Three and four components nest the same way: 0x5900B9DA and 0xFDBF7848.
    EXPECT(hashUint(Uint3(1u, 2u, 3u)) == hashUint(1u ^ hashUint(2u ^ hashUint(3u))));
    EXPECT(hashUint(Uint3(1u, 2u, 3u)) == 0x5900B9DAu);
    EXPECT(hashUint(Uint4(1u, 2u, 3u, 4u)) == hashUint(1u ^ hashUint(2u ^ hashUint(3u ^ hashUint(4u)))));
    EXPECT(hashUint(Uint4(1u, 2u, 3u, 4u)) == 0xFDBF7848u);
    // The order matters: swapping the two components changes the hash.
    EXPECT(hashUint(Uint2(2u, 1u)) != hashUint(Uint2(1u, 2u)));
}

TEST(unitFloatFromHashSpansZeroToOne) {
    EXPECT(kernels::unitFloatFromHash(0u) == 0.0f);
    // 2³¹ / 2³² = 0.5 exactly.
    EXPECT(kernels::unitFloatFromHash(0x80000000u) == 0.5f);
    // The largest hash rounds up to 2³² in float, so the result is exactly 1.
    EXPECT(kernels::unitFloatFromHash(0xFFFFFFFFu) == 1.0f);
}

TEST(hashCellNumberOffsetsByThirtyTwoThousandSevenHundredSixtyEight) {
    EXPECT(kernels::hashCellNumber(0.0f) == 32768u);
    // ⌊2.9⌋ = 2; ⌊−0.1⌋ = −1; ⌊−32768⌋ = −32768 → 0.
    EXPECT(kernels::hashCellNumber(2.9f) == 32770u);
    EXPECT(kernels::hashCellNumber(-0.1f) == 32767u);
    EXPECT(kernels::hashCellNumber(-32768.0f) == 0u);
}

TEST(floatHashesMatchTheIntegerHashes) {
    // hash11(3.7): ⌊3.7⌋ + 0 = 3.0 = 0x40400000 bits; 0.33661610 from the integer hash of those bits.
    EXPECT(isApproximatelyEqual(kernels::hash11(3.7f), unitFloat(kernels::hashUint(kernels::bitsOf(3.0f))), 1e-7f));
    EXPECT(isApproximatelyEqual(kernels::hash11(3.7f), 0.33661610f, 1e-6f));
    // hash11(0.5): ⌊0.5⌋ = 0 → hashUint(0) = 0x07BB2FE2 → 0.0302.
    EXPECT(isApproximatelyEqual(kernels::hash11(0.5f), 0.03019999f, 1e-6f));
    // hash21(3, 4): cells 32771 and 32772 → 0x6C01CB11 → 0.42190236.
    EXPECT(isApproximatelyEqual(kernels::hash21(Cell2(3.5f, 4.25f)), 0.42190236f, 1e-6f));
    // hash31(1, 2, 3) → 0.87014902.
    EXPECT(isApproximatelyEqual(kernels::hash31(Cell3(1.1f, 2.2f, 3.3f)), 0.87014902f, 1e-6f));
    // hashFloat(1.0): bits 0x3F800000 → 0xF1BB4A13 → 0.94426405.
    EXPECT(isApproximatelyEqual(kernels::hashFloat(1.0f), 0.94426405f, 1e-6f));
}

TEST(vectorFloatHashesChainTheIntegerHash) {
    // hash22(2, 5): h = hash of cells (32770, 32773) → 0.74016142; second = hashUint(h) → 0.35653239.
    Cell2 pair = kernels::hash22(Cell2(2.0f, 5.0f));
    EXPECT(isApproximatelyEqual(pair.x, 0.74016142f, 1e-6f));
    EXPECT(isApproximatelyEqual(pair.y, 0.35653239f, 1e-6f));
    // hash33 and hash44 start with the same hash as hash31 and hash41 and then chain.
    Cell3 triple = kernels::hash33(Cell3(1.0f, 2.0f, 3.0f));
    EXPECT(triple.x == kernels::hash31(Cell3(1.0f, 2.0f, 3.0f)));
    Cell4 quad = kernels::hash44(Cell4(1.0f, 2.0f, 3.0f, 4.0f));
    EXPECT(quad.x == kernels::hash41(Cell4(1.0f, 2.0f, 3.0f, 4.0f)));
    EXPECT(quad.y == unitFloat(kernels::hashUint(kernels::hashUint(kernels::Vector4Of<unsigned int>(
        kernels::hashCellNumber(1.0f), kernels::hashCellNumber(2.0f), kernels::hashCellNumber(3.0f), kernels::hashCellNumber(4.0f))))));
    EXPECT(triple.y != triple.x && triple.z != triple.y);
}

TEST(cellHashesAreConstantInsideACellAndChangeBetweenCells) {
    // Every point of the cell [2, 3) × [5, 6) gives the same number.
    float reference = kernels::hash21(Cell2(2.0f, 5.0f));
    EXPECT(kernels::hash21(Cell2(2.999f, 5.5f)) == reference);
    EXPECT(kernels::hash11(2.0f) == kernels::hash11(2.75f));
    // The next cell differs, and −0 and +0 are one cell.
    EXPECT(kernels::hash21(Cell2(3.0f, 5.0f)) != reference);
    EXPECT(kernels::hash11(-0.0f) == kernels::hash11(0.0f));
    EXPECT(kernels::hash11(-0.5f) != kernels::hash11(0.5f));
}

TEST(hashFloatSeparatesNearbyFloatsThatCellHashesMerge) {
    EXPECT(kernels::hashFloat(1.0f) != kernels::hashFloat(1.0000001f));
    EXPECT(kernels::hash11(1.0f) == kernels::hash11(1.0000001f));
    // 0 and −0 have different bits.
    EXPECT(kernels::hashFloat(0.0f) != kernels::hashFloat(-0.0f));
    EXPECT(kernels::hashFloat(Cell2(1.0f, 2.0f)) != kernels::hashFloat(Cell2(2.0f, 1.0f)));
    EXPECT(kernels::hashFloat(Cell3(1.0f, 2.0f, 3.0f)) != kernels::hashFloat(Cell3(1.0f, 2.0f, 3.5f)));
    EXPECT(kernels::hashFloat(Cell4(1.0f, 2.0f, 3.0f, 4.0f)) != kernels::hashFloat(Cell4(1.0f, 2.0f, 3.0f, 4.5f)));
}

TEST(everyFloatHashIsInZeroToOneAndSpreadsEvenly) {
    int belowHalf = 0;
    const int count = 4000;
    for (int step = 0; step < count; step += 1) {
        float coordinate = static_cast<float>(step) * 0.37f - 700.0f;
        float values[4] = {
            kernels::hash11(coordinate),
            kernels::hash21(Cell2(coordinate, 3.0f)),
            kernels::hash31(Cell3(coordinate, 1.0f, -2.0f)),
            kernels::hashFloat(Cell4(coordinate, 0.5f, 7.0f, 1.0f)),
        };
        for (float value : values) EXPECT(value >= 0.0f && value <= 1.0f);
        Cell4 quad = kernels::hash44(Cell4(coordinate, 1.0f, 2.0f, 3.0f));
        EXPECT(quad.x >= 0 && quad.x <= 1 && quad.y >= 0 && quad.y <= 1 && quad.z >= 0 && quad.z <= 1 && quad.w >= 0 && quad.w <= 1);
        if (values[0] < 0.5f) belowHalf += 1;
    }
    // A fair hash puts about half below 0.5; 4000 draws stay within 10% of 2000 with a very large margin.
    EXPECT(belowHalf > 1800 && belowHalf < 2200);
}
