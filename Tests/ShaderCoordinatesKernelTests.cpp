// The coordinate-transform kernels (Kernels/ShaderCoordinates.h) on the CPU, against values worked
// out by hand from each function's formula, and the properties each must keep. The same header is
// compiled as Metal by `make kernels`.
#include <cmath>

#include "../Kernels/ShaderCoordinates.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;
using Kernel3 = kernels::Vector3Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }
bool nearly(Kernel2 first, Kernel2 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance);
}
bool nearly(Kernel4 first, Kernel4 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance)
        && nearly(first.w, second.w, tolerance);
}
float lengthOf(Kernel2 value) { return std::sqrt(value.x * value.x + value.y * value.y); }
} // namespace

TEST(mirroredUvDropsTheSignOnTheChosenAxes) {
    // mode 0: (|−1|, −2) = (1, −2); mode 1: (−1, |−2|) = (−1, 2); other modes: both
    EXPECT(nearly(kernels::mirroredUv(Kernel2(-1.0f, -2.0f), 0), Kernel2(1.0f, -2.0f)));
    EXPECT(nearly(kernels::mirroredUv(Kernel2(-1.0f, -2.0f), 1), Kernel2(-1.0f, 2.0f)));
    EXPECT(nearly(kernels::mirroredUv(Kernel2(-1.0f, -2.0f), 2), Kernel2(1.0f, 2.0f)));
    // Mirroring twice changes nothing more.
    Kernel2 once = kernels::mirroredUv(Kernel2(-0.3f, 0.8f), 2);
    EXPECT(nearly(kernels::mirroredUv(once, 2), once));
}

TEST(flippedUvTurnsTheUnitSquareAround) {
    // 1 − 0.25 = 0.75 on the flipped axis
    EXPECT(nearly(kernels::flippedUv(Kernel2(0.25f, 0.75f), 0), Kernel2(0.75f, 0.75f)));
    EXPECT(nearly(kernels::flippedUv(Kernel2(0.25f, 0.75f), 1), Kernel2(0.25f, 0.25f)));
    EXPECT(nearly(kernels::flippedUv(Kernel2(0.25f, 0.75f), 2), Kernel2(0.75f, 0.25f)));
    // Flipping twice is the identity.
    Kernel2 uv(0.1f, 0.9f);
    EXPECT(nearly(kernels::flippedUv(kernels::flippedUv(uv, 2), 2), uv));
}

TEST(kaleidoscopeFoldsIntoOneWedgeAndKeepsTheLength) {
    // 4 segments: sector π/2. (1, 1) is at π/4, the middle of the sector: |π/4 − π/4| = 0 → (√2, 0)
    EXPECT(nearly(kernels::kaleidoscopeFolded(Kernel2(1.0f, 1.0f), 4.0f, 0.0f), Kernel2(1.41421356f, 0.0f)));
    // (1, 0.0) at angle 0: |0 − π/4| = π/4 → (cos π/4, sin π/4)
    EXPECT(nearly(kernels::kaleidoscopeFolded(Kernel2(1.0f, 0.0f), 4.0f, 0.0f), Kernel2(0.70710678f, 0.70710678f)));
    EXPECT(kernels::kaleidoscopeFolded(Kernel2(0.0f, 0.0f), 6.0f, 0.3f).x == 0.0f);
    // Segments are rounded and at least one: 0.2 → 1 segment (sector τ).
    EXPECT(nearly(kernels::kaleidoscopeFolded(Kernel2(0.5f, 0.2f), 0.2f, 0.0f), kernels::kaleidoscopeFolded(Kernel2(0.5f, 0.2f), 1.0f, 0.0f)));
    // Property: the length is kept and the result's angle lies in 0 … sector / 2.
    for (int segments = 1; segments <= 9; segments += 1) {
        float halfSector = 3.14159265f / static_cast<float>(segments);
        for (int step = 0; step < 60; step += 1) {
            float angle = 0.1f + 0.1037f * static_cast<float>(step);
            Kernel2 point(1.7f * std::cos(angle), 1.7f * std::sin(angle));
            Kernel2 folded = kernels::kaleidoscopeFolded(point, static_cast<float>(segments), 0.37f);
            EXPECT(nearly(lengthOf(folded), 1.7f, 1e-4f));
            EXPECT(folded.y >= -1e-5f);
            EXPECT(std::atan2(folded.y, folded.x) <= halfSector + 1e-4f);
        }
    }
}

TEST(twirlRotatesByStrengthTimesFalloffSquared) {
    // radius 1, point (0.5, 0): r = 0.5, falloff = 0.5, angle = 4 × 0.25 = 1 rad → 0.5 (cos 1, sin 1)
    EXPECT(nearly(kernels::twirled(Kernel2(0.5f, 0.0f), 4.0f, 1.0f), Kernel2(0.27015115f, 0.42073549f)));
    // Not at all from the radius out, and not at the origin.
    EXPECT(nearly(kernels::twirled(Kernel2(2.0f, 0.0f), 4.0f, 1.0f), Kernel2(2.0f, 0.0f)));
    EXPECT(nearly(kernels::twirled(Kernel2(0.0f, 0.0f), 4.0f, 1.0f), Kernel2(0.0f, 0.0f)));
    // Property: the length is kept; a negative strength undoes a positive one.
    for (int step = 0; step < 50; step += 1) {
        Kernel2 point(0.9f * std::cos(0.37f * static_cast<float>(step)), 0.9f * std::sin(0.37f * static_cast<float>(step)));
        Kernel2 twisted = kernels::twirled(point, 3.0f, 1.5f);
        EXPECT(nearly(lengthOf(twisted), 0.9f, 1e-5f));
        EXPECT(nearly(kernels::twirled(twisted, -3.0f, 1.5f), point, 1e-5f));
    }
}

TEST(spherizeMovesPointsInsideTheDiscAndFixesTheRest) {
    // r = 0.5, strength 1: bent = 0.5² = 0.25 → (0.5, 0) × 0.25 / 0.5 = (0.25, 0)
    EXPECT(nearly(kernels::spherized(Kernel2(0.5f, 0.0f), 1.0f, 1.0f), Kernel2(0.25f, 0.0f)));
    // r = 0.25, strength −0.5: bent = 0.25^0.5 = 0.5 → (0.25, 0) × 0.5 / 0.25 = (0.5, 0)
    EXPECT(nearly(kernels::spherized(Kernel2(0.25f, 0.0f), -0.5f, 1.0f), Kernel2(0.5f, 0.0f)));
    // From the radius out, and with strength 0, nothing moves.
    EXPECT(nearly(kernels::spherized(Kernel2(1.5f, 0.5f), 2.0f, 1.0f), Kernel2(1.5f, 0.5f)));
    EXPECT(nearly(kernels::spherized(Kernel2(0.3f, 0.4f), 0.0f, 1.0f), Kernel2(0.3f, 0.4f)));
    // Property: the direction is kept and the relative radius becomes r^(1 + strength).
    Kernel2 result = kernels::spherized(Kernel2(0.3f, 0.4f), 1.0f, 2.0f);
    EXPECT(nearly(lengthOf(result), 0.25f * 0.25f * 2.0f, 1e-5f));
    EXPECT(nearly(result.y / result.x, 0.4f / 0.3f, 1e-5f));
}

TEST(rowOffsetGridSlidesEveryOtherRowOrColumn) {
    Kernel2 count(10.0f, 10.0f);
    // p = (3, 6): row 6 is even, no slide; row 7 is odd: x + 0.5
    EXPECT(nearly(kernels::rowOffsetGrid(Kernel2(0.3f, 0.6f), count, 0.5f, 0), Kernel2(3.0f, 6.0f)));
    EXPECT(nearly(kernels::rowOffsetGrid(Kernel2(0.3f, 0.7f), count, 0.5f, 0), Kernel2(3.5f, 7.0f)));
    // Columns: column 3 is odd → y + 0.5; column 4 is even → no slide
    EXPECT(nearly(kernels::rowOffsetGrid(Kernel2(0.3f, 0.7f), count, 0.5f, 1), Kernel2(3.0f, 7.5f)));
    EXPECT(nearly(kernels::rowOffsetGrid(Kernel2(0.4f, 0.7f), count, 0.5f, 1), Kernel2(4.0f, 7.0f)));
    // Negative rows alternate the same way (floored modulo): row −1 is odd.
    EXPECT(nearly(kernels::rowOffsetGrid(Kernel2(0.3f, -0.1f), count, 0.5f, 0), Kernel2(3.5f, -1.0f)));
}

TEST(hexagonalGridFindsTheNearestCentre) {
    // Centres at integer multiples of (1, √3) and at (0.5, √3/2) plus those.
    Kernel4 atOrigin = kernels::hexagonalGrid(Kernel2(0.0f, 0.0f));
    EXPECT(nearly(atOrigin, Kernel4(0.0f, 0.0f, 0.0f, 0.0f)));
    EXPECT(nearly(kernels::hexagonalGrid(Kernel2(1.0f, 0.0f)), Kernel4(0.0f, 0.0f, 1.0f, 0.0f)));
    EXPECT(nearly(kernels::hexagonalGrid(Kernel2(0.5f, 0.8660254f)), Kernel4(0.0f, 0.0f, 0.5f, 0.8660254f)));
    // A point 0.1 right of the centre (0.5, √3/2) is 0.1 from it.
    EXPECT(nearly(kernels::hexagonalGrid(Kernel2(0.6f, 0.8660254f)), Kernel4(0.1f, 0.0f, 0.5f, 0.8660254f)));
    // Property: xy is the point relative to the centre zw, and the centre is one of the lattice's
    // (so within one covering radius, 1/√3 = 0.5774, of the point).
    for (int column = -12; column <= 12; column += 1) {
        for (int row = -12; row <= 12; row += 1) {
            Kernel2 point(0.1337f * static_cast<float>(column) + 0.0173f, 0.1121f * static_cast<float>(row) + 0.0291f);
            Kernel4 cell = kernels::hexagonalGrid(point);
            EXPECT(nearly(cell.x, point.x - cell.z, 1e-5f));
            EXPECT(nearly(cell.y, point.y - cell.w, 1e-5f));
            EXPECT(lengthOf(Kernel2(cell.x, cell.y)) <= 0.5774f + 1e-3f);
        }
    }
}

TEST(hexagonalEdgeDistanceIsHalfAtTheCentreAndZeroOnTheEdge) {
    // Centre: 0.5 − max(0, 0)
    EXPECT(nearly(kernels::hexagonalEdgeDistance(Kernel2(0.0f, 0.0f)), 0.5f));
    // Vertical edge at x = 0.5: q·n = 0.25 < 0.5 → 0.5 − 0.5 = 0
    EXPECT(nearly(kernels::hexagonalEdgeDistance(Kernel2(0.5f, 0.0f)), 0.0f));
    // Slanted edge: q = (0.25, 0.4330127), n = (0.5, 0.8660254) → 0.125 + 0.375 = 0.5 → 0
    EXPECT(nearly(kernels::hexagonalEdgeDistance(Kernel2(0.25f, 0.4330127f)), 0.0f));
    // Outside is negative: q = (0, 0.8660254) → 0.75 → −0.25
    EXPECT(nearly(kernels::hexagonalEdgeDistance(Kernel2(0.0f, 0.8660254f)), -0.25f));
}

TEST(triangularGridGivesBarycentricCoordinatesAndAStableId) {
    // skewed (0.25, 0.25) → the lower triangle of cell (0, 0): barycentric (0.25, 0.25, 0.5), edge 0.25
    float lowerY = 0.25f * 1.7320508f / 2.0f;
    Kernel4 lower = kernels::triangularGrid(Kernel2(0.25f + lowerY / 1.7320508f, lowerY));
    EXPECT(nearly(lower.x, 0.25f, 1e-5f) && nearly(lower.y, 0.25f, 1e-5f) && nearly(lower.z, 0.25f, 1e-5f));
    EXPECT(lower.w == kernels::hash21(Kernel2(0.0f, 0.0f)));
    // skewed (0.75, 0.75) → the upper triangle: barycentric (0.25, 0.25), edge 0.25, id from cell × 2 + 1
    float upperY = 0.75f * 1.7320508f / 2.0f;
    Kernel4 upper = kernels::triangularGrid(Kernel2(0.75f + upperY / 1.7320508f, upperY));
    EXPECT(nearly(upper.x, 0.25f, 1e-5f) && nearly(upper.y, 0.25f, 1e-5f) && nearly(upper.z, 0.25f, 1e-5f));
    EXPECT(upper.w == kernels::hash21(Kernel2(1.0f, 1.0f)));
    // At a vertex the distance to an edge is 0.
    EXPECT(nearly(kernels::triangularGrid(Kernel2(0.0f, 0.0f)).z, 0.0f));
    // Property: the edge distance never exceeds the centre value 1/3 and is never negative.
    for (int step = 0; step < 400; step += 1) {
        Kernel4 cell = kernels::triangularGrid(Kernel2(0.0731f * static_cast<float>(step), 0.0529f * static_cast<float>(step % 17)));
        EXPECT(cell.z >= -1e-6f && cell.z <= 1.0f / 3.0f + 1e-5f);
        EXPECT(cell.w >= 0.0f && cell.w <= 1.0f);
    }
}

TEST(pixelatedUvSnapsToTheCentreOfACoarseCell) {
    // 4 × 4 cells: ⌊(1.04, 3.6)⌋ = (1, 3); (1.5, 3.5) / 4
    EXPECT(nearly(kernels::pixelatedUv(Kernel2(0.26f, 0.9f), 4.0f, 1.0f), Kernel2(0.375f, 0.875f)));
    // Aspect 2: 8 × 4 cells: ⌊(2.08, 3.6)⌋ = (2, 3) → (2.5 / 8, 3.5 / 4)
    EXPECT(nearly(kernels::pixelatedUv(Kernel2(0.26f, 0.9f), 4.0f, 2.0f), Kernel2(0.3125f, 0.875f)));
    // Fewer than one cell becomes one: the middle of the square.
    EXPECT(nearly(kernels::pixelatedUv(Kernel2(0.26f, 0.9f), 0.5f, 1.0f), Kernel2(0.5f, 0.5f)));
    // Property: idempotent (a cell centre snaps to itself).
    Kernel2 snapped = kernels::pixelatedUv(Kernel2(0.61f, 0.12f), 7.0f, 1.0f);
    EXPECT(nearly(kernels::pixelatedUv(snapped, 7.0f, 1.0f), snapped));
}

TEST(transformedByMatrix3AppliesColumnsAndDividesByW) {
    Kernel3 firstColumn(1.0f, 0.0f, 0.0f);
    Kernel3 secondColumn(0.0f, 1.0f, 0.0f);
    // Identity
    EXPECT(nearly(kernels::transformedByMatrix3(Kernel2(1.0f, 2.0f), firstColumn, secondColumn, Kernel3(0.0f, 0.0f, 1.0f)), Kernel2(1.0f, 2.0f)));
    // Translation by (3, −1): the third column is (3, −1, 1) → (4, 1)
    EXPECT(nearly(kernels::transformedByMatrix3(Kernel2(1.0f, 2.0f), firstColumn, secondColumn, Kernel3(3.0f, -1.0f, 1.0f)), Kernel2(4.0f, 1.0f)));
    // w = 2 halves: h = (1, 2, 2) → (0.5, 1)
    EXPECT(nearly(kernels::transformedByMatrix3(Kernel2(1.0f, 2.0f), firstColumn, secondColumn, Kernel3(0.0f, 0.0f, 2.0f)), Kernel2(0.5f, 1.0f)));
    // Rotation by 90°: columns (0, 1, 0) and (−1, 0, 0) send (1, 0) to (0, 1)
    EXPECT(nearly(kernels::transformedByMatrix3(Kernel2(1.0f, 0.0f), Kernel3(0.0f, 1.0f, 0.0f), Kernel3(-1.0f, 0.0f, 0.0f), Kernel3(0.0f, 0.0f, 1.0f)),
        Kernel2(0.0f, 1.0f)));
    // A vanishing w leaves x and y as they are: h = (1, 2, 0) → (1, 2)
    EXPECT(nearly(kernels::transformedByMatrix3(Kernel2(1.0f, 2.0f), firstColumn, secondColumn, Kernel3(0.0f, 0.0f, 0.0f)), Kernel2(1.0f, 2.0f)));
}
