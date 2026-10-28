// The shader-library patterns (Kernels/ShaderPatterns.h) on the CPU: masks at cells, edges and
// centres worked out by hand, the Bayer matrix's thresholds against the textbook 4x4 matrix, and the
// properties each pattern keeps (a mask stays in 0…1, the checkerboard inverts one cell over). The
// pixel footprint is always given, since there is no `fwidth` here; 0.01 is a small, sharp one.
// The same header is compiled as Metal by `make kernels` and compared on the GPU by `make gpu`.
#include <cmath>

#include "../Kernels/ShaderPatterns.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;
using Kernel4 = kernels::Vector4Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }

constexpr float sharp = 0.01f;
const Kernel2 sharp2(0.01f, 0.01f);

/// Whether `value` is in 0…1, give or take `slack`. The checkerboard divides a difference of two
/// nearly equal numbers by the footprint (0.01 here), so its rounding is magnified about 200 times.
bool isMask(float value, float slack = 1e-6f) { return value >= -slack && value <= 1.0f + slack; }

/// Points on a jittered grid that avoid cell borders.
Kernel2 samplePoint(int index) {
    float step = static_cast<float>(index);
    return Kernel2(step * 0.173f - 6.1f, step * 0.291f - 9.3f);
}
constexpr int sampleCount = 100;
} // namespace

TEST(coverageIsAHalfOnTheEdgeAndSmoothsAcrossOnePixel) {
    EXPECT(nearly(kernels::antialiasedCoverage(0.0f, sharp, 0.0f), 0.5f));
    EXPECT(kernels::antialiasedCoverage(1.0f, sharp, 0.0f) == 1.0f);
    EXPECT(kernels::antialiasedCoverage(-1.0f, sharp, 0.0f) == 0.0f);
    // Softness widens the edge: width 0.01 + 0.09 = 0.1; at 0.025 t = (0.025 + 0.05)/0.1 = 0.75,
    // and 3t² − 2t³ = 1.6875 − 0.84375 = 0.84375.
    EXPECT(nearly(kernels::antialiasedCoverage(0.025f, sharp, 0.09f), 0.84375f));
    // Symmetric: inside and outside by the same distance add up to 1.
    EXPECT(nearly(kernels::antialiasedCoverage(0.003f, sharp, 0.0f) + kernels::antialiasedCoverage(-0.003f, sharp, 0.0f), 1.0f));
    // A zero footprint is floored at 0.0001, so the edge stays finite.
    EXPECT(nearly(kernels::antialiasedCoverage(0.0f, 0.0f, 0.0f), 0.5f));
}

TEST(checkerboardAlternatesBySquare) {
    // 0 where ⌊x⌋ + ⌊y⌋ is even and 1 where it is odd, including the negative squares.
    EXPECT(nearly(kernels::checkerboardPattern(Kernel2(0.5f, 0.5f), 0.0f, sharp2), 0.0f));
    EXPECT(nearly(kernels::checkerboardPattern(Kernel2(1.5f, 0.5f), 0.0f, sharp2), 1.0f));
    EXPECT(nearly(kernels::checkerboardPattern(Kernel2(1.5f, 1.5f), 0.0f, sharp2), 0.0f));
    EXPECT(nearly(kernels::checkerboardPattern(Kernel2(0.5f, -0.5f), 0.0f, sharp2), 1.0f));
    EXPECT(nearly(kernels::checkerboardPattern(Kernel2(-0.5f, -0.5f), 0.0f, sharp2), 0.0f));
    // On the border between two squares the mask is halfway: i.x = 0 there.
    EXPECT(nearly(kernels::checkerboardPattern(Kernel2(1.0f, 0.5f), 0.0f, sharp2), 0.5f));
}

TEST(checkerboardInvertsOneSquareOverAndRepeatsEveryTwo) {
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel2 p = samplePoint(index);
        float here = kernels::checkerboardPattern(p, 0.0f, sharp2);
        EXPECT(isMask(here, 1e-3f));
        EXPECT(nearly(kernels::checkerboardPattern(Kernel2(p.x + 1.0f, p.y), 0.0f, sharp2), 1.0f - here, 1e-3f));
        EXPECT(nearly(kernels::checkerboardPattern(Kernel2(p.x + 2.0f, p.y), 0.0f, sharp2), here, 1e-3f));
        EXPECT(nearly(kernels::checkerboardPattern(Kernel2(p.x, p.y + 2.0f), 0.0f, sharp2), here, 1e-3f));
    }
}

TEST(stripesAreCentredInEachPeriodAndNumbered) {
    // count 1, angle 0: q = p.x; the stripe is centred at fract 0.5 and is 0.5 wide.
    Kernel2 centre = kernels::stripesPattern(Kernel2(0.5f, 0.0f), 1.0f, 0.0f, 0.5f, 0.0f, sharp);
    EXPECT(nearly(centre.x, 1.0f));
    EXPECT(centre.y == 0.0f);
    EXPECT(nearly(kernels::stripesPattern(Kernel2(0.95f, 0.0f), 1.0f, 0.0f, 0.5f, 0.0f, sharp).x, 0.0f));
    EXPECT(kernels::stripesPattern(Kernel2(1.5f, 0.0f), 1.0f, 0.0f, 0.5f, 0.0f, sharp).y == 1.0f);
    // The coordinate: count 4 at p.x = 0.75 is (0.75 − 0.5) × 4 + 0.5 = 1.5; turned by π/2 it follows y.
    EXPECT(nearly(kernels::stripeCoordinate(Kernel2(0.75f, 0.0f), 4.0f, 0.0f), 1.5f));
    EXPECT(nearly(kernels::stripeCoordinate(Kernel2(0.0f, 0.25f), 2.0f, kernels::pi<float>() * 0.5f), 1.0f));
}

TEST(gridDrawsLinesBetweenCellsAndHashesTheCell) {
    Kernel2 count(4.0f, 4.0f);
    Kernel2 inCell = kernels::gridPattern(Kernel2(0.125f, 0.125f), count, 0.2f, 0.0f, Kernel2(0.001f, 0.001f));
    EXPECT(nearly(inCell.x, 0.0f));
    EXPECT(nearly(inCell.y, kernels::hash21(Kernel2(0.0f, 0.0f))));
    // On the line between cells 0 and 1 (q.x = 1): the distance to the line is 0, the mask is 1.
    Kernel2 onLine = kernels::gridPattern(Kernel2(0.25f, 0.125f), count, 0.2f, 0.0f, Kernel2(0.001f, 0.001f));
    EXPECT(nearly(onLine.x, 1.0f, 1e-4f));
    EXPECT(nearly(onLine.y, kernels::hash21(Kernel2(1.0f, 0.0f))));
}

TEST(dotsFillTheirRadiusAndStaggerAlternateRows) {
    // Scale 1, radius 0.3: the centre of a cell is inside, its corner outside.
    EXPECT(nearly(kernels::dotsPattern(Kernel2(0.5f, 0.5f), 1.0f, 0.3f, 0.0f, false, sharp).x, 1.0f));
    EXPECT(nearly(kernels::dotsPattern(Kernel2(0.0f, 0.0f), 1.0f, 0.3f, 0.0f, false, sharp).x, 0.0f));
    EXPECT(nearly(kernels::dotsPattern(Kernel2(0.5f, 0.5f), 1.0f, 0.3f, 0.0f, false, sharp).y, kernels::hash21(Kernel2(0.0f, 0.0f))));
    // Row 1 slid by half a cell: its dots are centred at x = 1, so x = 0.5 is between two dots.
    EXPECT(nearly(kernels::dotsPattern(Kernel2(0.5f, 1.5f), 1.0f, 0.3f, 0.0f, true, sharp).x, 0.0f));
    EXPECT(nearly(kernels::dotsPattern(Kernel2(0.5f, 1.5f), 1.0f, 0.3f, 0.0f, false, sharp).x, 1.0f));
    EXPECT(nearly(kernels::dotsPattern(Kernel2(0.0f, 1.5f), 1.0f, 0.3f, 0.0f, true, sharp).x, 1.0f));
}

TEST(bricksLeaveMortarAndSlideEveryOtherRow) {
    Kernel2 count(2.0f, 2.0f);
    Kernel2 tiny(0.001f, 0.001f);
    // Row 0, offset 0.5: q = p × 2. The middle of a brick is brick; q.x = 1 (a vertical joint) is mortar.
    EXPECT(nearly(kernels::bricksPattern(Kernel2(0.25f, 0.25f), count, 0.5f, 0.2f, 0.0f, tiny).x, 1.0f));
    EXPECT(nearly(kernels::bricksPattern(Kernel2(0.5f, 0.25f), count, 0.5f, 0.2f, 0.0f, tiny).x, 0.0f));
    // Row 1 is slid by half a brick: the joint moves to p.x = 0.25 and p.x = 0.5 is brick.
    EXPECT(nearly(kernels::bricksPattern(Kernel2(0.25f, 0.75f), count, 0.5f, 0.2f, 0.0f, tiny).x, 0.0f));
    EXPECT(nearly(kernels::bricksPattern(Kernel2(0.5f, 0.75f), count, 0.5f, 0.2f, 0.0f, tiny).x, 1.0f));
    EXPECT(nearly(kernels::bricksPattern(Kernel2(0.25f, 0.25f), count, 0.5f, 0.2f, 0.0f, tiny).y, kernels::hash21(Kernel2(0.0f, 0.0f))));
}

TEST(hexagonTilesAreFullAtCentresAndHalfOnTheirEdges) {
    // The centre of the hexagon with centre (0.5, √3/2): the local position is 0 and the edge
    // distance is 0.5 (the largest).
    Kernel4 centre = kernels::hexagonTilesPattern(Kernel2(0.5f, 0.8660254f), 1.0f, 0.1f, 0.0f, sharp);
    EXPECT(nearly(centre.x, 1.0f));
    EXPECT(nearly(centre.y, 0.5f));
    EXPECT(nearly(centre.z, kernels::hash21(Kernel2(0.5f, 0.8660254f)), 1e-4f));
    EXPECT(nearly(centre.w, kernels::hash21(Kernel2(0.5f, 0.8660254f) + 41.0f), 1e-4f));
    // The mid-point of an edge, half a cell left of the centre: edge distance 0, mask 0.5 with no gap.
    Kernel4 edge = kernels::hexagonTilesPattern(Kernel2(0.0f, 0.8660254f), 1.0f, 0.0f, 0.0f, sharp);
    EXPECT(nearly(edge.y, 0.0f, 1e-6f));
    EXPECT(nearly(edge.x, 0.5f, 1e-4f));
}

TEST(truchetArcsJoinAtTheMidpointsOfCellEdges) {
    // Variant 0 passes through every edge midpoint whichever way the cell is mirrored.
    EXPECT(nearly(kernels::truchetPattern(Kernel2(0.0f, 0.5f), 1.0f, 0.2f, 0.0f, 0, sharp), 1.0f, 1e-4f));
    EXPECT(nearly(kernels::truchetPattern(Kernel2(0.5f, 0.0f), 1.0f, 0.2f, 0.0f, 0, sharp), 1.0f, 1e-4f));
    // The middle of the cell is √2/2 − 0.5 = 0.2071 from the arcs, more than the half width 0.1.
    EXPECT(nearly(kernels::truchetPattern(Kernel2(0.5f, 0.5f), 1.0f, 0.2f, 0.0f, 0, sharp), 0.0f));
    // Variant 1: the diagonal f.x = f.y (or its mirror) passes through the middle of the cell.
    EXPECT(nearly(kernels::truchetPattern(Kernel2(0.5f, 0.5f), 1.0f, 0.2f, 0.0f, 1, sharp), 1.0f));
}

TEST(ripplesFollowTheWaveAndFadeWithDistance) {
    // At the origin sin(0) = 0, so 0.5; at r = 0.25 with frequency 1 the sine is sin(τ/4) = 1.
    EXPECT(nearly(kernels::ripplesPattern(Kernel2(0.0f, 0.0f), 3.0f, 0.0f, 0.0f, 0), 0.5f));
    EXPECT(nearly(kernels::ripplesPattern(Kernel2(0.25f, 0.0f), 1.0f, 0.0f, 0.0f, 0), 1.0f));
    // With decay 4 at r = 0.25 the wave is scaled by e⁻¹ = 0.36787944: 0.5 + 0.5 × 0.36787944.
    EXPECT(nearly(kernels::ripplesPattern(Kernel2(0.25f, 0.0f), 1.0f, 0.0f, 4.0f, 0), 0.6839397f));
    // A square wave at r = 0.25 is +1 (fract 0.25 < 0.5); a sawtooth is 2 × 0.25 − 1 = −0.5.
    EXPECT(nearly(kernels::ripplesPattern(Kernel2(0.0f, 0.25f), 1.0f, 0.0f, 0.0f, 2), 1.0f));
    EXPECT(nearly(kernels::ripplesPattern(Kernel2(0.0f, 0.25f), 1.0f, 0.0f, 0.0f, 3), 0.25f));
}

TEST(spiralWithNoArmsIsRingsAndTheLogarithmicOneRingsInTheLogarithm) {
    // arms 0, frequency 1: phase = r; the stripe is centred at fract 0.5 (width 0.4).
    EXPECT(nearly(kernels::spiralPattern(Kernel2(0.5f, 0.0f), 0.0f, 1.0f, 0.0f, 0.4f, 0.0f, 0, sharp), 1.0f));
    EXPECT(nearly(kernels::spiralPattern(Kernel2(1.0f, 0.0f), 0.0f, 1.0f, 0.0f, 0.4f, 0.0f, 0, sharp), 0.0f));
    // Logarithmic: phase = ln r, so r = e^0.5 is centred.
    EXPECT(nearly(kernels::spiralPattern(Kernel2(1.6487213f, 0.0f), 0.0f, 1.0f, 0.0f, 0.4f, 0.0f, 1, sharp), 1.0f));
}

TEST(sunburstHasRaysCentredBetweenWholeTurnsOfThePhase) {
    // 4 rays, phase = 4 θ/τ: θ = 0 gives phase 0 (the gap), θ = π/4 gives 0.5 (the middle of a ray).
    EXPECT(nearly(kernels::sunburstPattern(Kernel2(1.0f, 0.0f), 4.0f, 0.0f, 0.5f, 0.0f, sharp), 0.0f));
    EXPECT(nearly(kernels::sunburstPattern(Kernel2(1.0f, 1.0f), 4.0f, 0.0f, 0.5f, 0.0f, sharp), 1.0f));
    // Rotating by π/4 swaps them.
    EXPECT(nearly(kernels::sunburstPattern(Kernel2(1.0f, 0.0f), 4.0f, kernels::pi<float>() * 0.25f, 0.5f, 0.0f, sharp), 1.0f));
}

TEST(zigzagBendsTheStripesByTheWave) {
    // phase = p.y rows + waveShape(p.x frequency) amplitude/2. At x = 0.125, frequency 1:
    // sine sin(τ/8) = 0.70710678 → bend 0.7071; triangle 1 − 4|fract(0.375) − 0.5| = 0.5 → bend 0.5.
    EXPECT(nearly(kernels::zigzagPhase(Kernel2(0.125f, 0.5f), 1.0f, 1.0f, 2.0f, 1), 1.2071068f));
    EXPECT(nearly(kernels::zigzagPhase(Kernel2(0.125f, 0.5f), 1.0f, 1.0f, 2.0f, 0), 1.0f));
    // With no bend (x = 0) the stripe is centred at fract 0.5.
    EXPECT(nearly(kernels::zigzagPattern(Kernel2(0.0f, 0.5f), 1.0f, 1.0f, 2.0f, 0.4f, 0.0f, 1, sharp), 1.0f));
    EXPECT(nearly(kernels::zigzagPattern(Kernel2(0.0f, 1.0f), 1.0f, 1.0f, 2.0f, 0.4f, 0.0f, 1, sharp), 0.0f));
}

TEST(herringboneLeansOppositeWaysInNeighbouringColumns) {
    // band = p.y rows + lean (fract(p.x columns) − 0.5) slant, lean +1 in even columns, −1 in odd.
    // rows 2, slant 0.4, p.y 0.5: even column at f = 0.25: 1 + (−0.25)(0.4) = 0.9; odd: 1 + 0.1 = 1.1.
    EXPECT(nearly(kernels::herringboneBand(Kernel2(0.25f, 0.5f), 1.0f, 2.0f, 0.4f), 0.9f));
    EXPECT(nearly(kernels::herringboneBand(Kernel2(1.25f, 0.5f), 1.0f, 2.0f, 0.4f), 1.1f));
    Kernel2 tiny(0.001f, 0.001f);
    Kernel2 middle = kernels::herringbonePattern(Kernel2(0.5f, 0.25f), 1.0f, 2.0f, 0.0f, 0.1f, 0.0f, tiny);
    EXPECT(nearly(middle.x, 1.0f));
    EXPECT(nearly(middle.y, kernels::hash21(Kernel2(0.0f, 0.0f))));
    // The vertical joint at p.x = 1 is mortar.
    EXPECT(nearly(kernels::herringbonePattern(Kernel2(1.0f, 0.25f), 1.0f, 2.0f, 0.0f, 0.1f, 0.0f, tiny).x, 0.0f));
}

TEST(basketweaveRoundsEachBarAndAlternatesDirection) {
    // Scale 1, 2 bars, gap 0.2. Cell (0,0) runs along x. Across = 2 f.y.
    Kernel2 middleOfBar = kernels::basketweavePattern(Kernel2(0.5f, 0.25f), 1.0f, 2.0f, 0.2f, 0.0f, 0.001f);
    EXPECT(nearly(middleOfBar.x, 1.0f));
    EXPECT(nearly(middleOfBar.y, 1.0f));
    // A quarter of the way across the bar fb = 0.25: shade = 1 − 0.5 (2 × 0.25 − 1)² = 0.875.
    Kernel2 side = kernels::basketweavePattern(Kernel2(0.5f, 0.125f), 1.0f, 2.0f, 0.2f, 0.0f, 0.001f);
    EXPECT(nearly(side.x, 1.0f));
    EXPECT(nearly(side.y, 0.875f));
    // The next cell runs along y; the middle of its bar is the same.
    EXPECT(nearly(kernels::basketweavePattern(Kernel2(1.25f, 0.5f), 1.0f, 2.0f, 0.2f, 0.0f, 0.001f).x, 1.0f));
    // The joint between the two bars of a cell (across = 1, fb = 0) is a gap.
    EXPECT(nearly(kernels::basketweavePattern(Kernel2(0.5f, 0.5f), 1.0f, 2.0f, 0.2f, 0.0f, 0.001f).x, 0.0f));
}

TEST(moireMultipliesTwoGratings) {
    // Equal gratings (angle 0): the product is the grating squared. At x = 0.25 the sine is 1 → 1;
    // at 0.75 it is −1 → 0; at 0 the sine is 0, so smoothstep(0.3, 0.7, 0.5) = 0.5 and 0.5² = 0.25.
    EXPECT(nearly(kernels::moirePattern(Kernel2(0.25f, 0.0f), 1.0f, 0.0f, 0.0f, 0), 1.0f));
    EXPECT(nearly(kernels::moirePattern(Kernel2(0.75f, 0.0f), 1.0f, 0.0f, 0.0f, 0), 0.0f));
    EXPECT(nearly(kernels::moirePattern(Kernel2(0.0f, 0.0f), 1.0f, 0.0f, 0.0f, 0), 0.25f));
    // Rings with centres 0.5 apart: at the origin both rings are 0.25 away, sin(τ/4) = 1.
    EXPECT(nearly(kernels::moirePattern(Kernel2(0.0f, 0.0f), 1.0f, 0.0f, 0.5f, 1), 1.0f));
    for (int index = 0; index < sampleCount; index += 1) {
        EXPECT(isMask(kernels::moirePattern(samplePoint(index), 3.0f, 0.4f, 0.6f, index % 2)));
    }
}

TEST(halftoneGrowsTheShapeWithTheValue) {
    // Screen cell centres are at q = k + 0.5; with scale 1, angle 0, q = p − 0.5, so p = (1, 1) is one.
    // Dots, value 0.25: inside = 0.7071 × 0.5 − |f| = 0.3536 − |f|; f = (0.3, 0) is inside, (0.4, 0) outside.
    EXPECT(nearly(kernels::halftonePattern(Kernel2(1.0f, 1.0f), 0.25f, 1.0f, 0.0f, 0, sharp), 1.0f));
    EXPECT(nearly(kernels::halftonePattern(Kernel2(1.3f, 1.0f), 0.25f, 1.0f, 0.0f, 0, sharp), 1.0f));
    EXPECT(nearly(kernels::halftonePattern(Kernel2(1.4f, 1.0f), 0.25f, 1.0f, 0.0f, 0, sharp), 0.0f));
    // Squares, value 0.25: half side 0.5 × 0.5 = 0.25; f = (0.2, 0.2) inside, (0.3, 0) outside.
    EXPECT(nearly(kernels::halftonePattern(Kernel2(1.2f, 1.2f), 0.25f, 1.0f, 0.0f, 1, sharp), 1.0f));
    EXPECT(nearly(kernels::halftonePattern(Kernel2(1.3f, 1.0f), 0.25f, 1.0f, 0.0f, 1, sharp), 0.0f));
    // Lines, value 0.5: half thickness 0.25 across y; f.y = 0.2 inside, 0.3 outside; x does not matter.
    EXPECT(nearly(kernels::halftonePattern(Kernel2(1.4f, 1.2f), 0.5f, 1.0f, 0.0f, 2, sharp), 1.0f));
    EXPECT(nearly(kernels::halftonePattern(Kernel2(1.4f, 1.3f), 0.5f, 1.0f, 0.0f, 2, sharp), 0.0f));
    // The screen coordinate is (p − 0.5) turned and scaled: (1.5, 0.5) at scale 2, π/2 → (0, 2).
    Kernel2 screen = kernels::halftoneCoordinate(Kernel2(1.5f, 0.5f), 2.0f, kernels::pi<float>() * 0.5f);
    EXPECT(nearly(screen.x, 0.0f) && nearly(screen.y, 2.0f));
}

TEST(bayerThresholdsMatchTheTextbookMatrices) {
    // One bit: the 2×2 matrix [0 2; 3 1], so thresholds (index + 0.5) / 4.
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(0.5f, 0.5f), 1.0f, 1), 0.125f));
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(1.5f, 0.5f), 1.0f, 1), 0.625f));
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(0.5f, 1.5f), 1.0f, 1), 0.875f));
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(1.5f, 1.5f), 1.0f, 1), 0.375f));
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(2.5f, 0.5f), 1.0f, 1), 0.125f));
    // Two bits: the 4×4 matrix, rows 0 8 2 10 / 12 4 14 6 / 3 11 1 9 / 15 7 13 5, thresholds (index + 0.5)/16.
    const int matrix[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    for (int row = 0; row < 4; row += 1) {
        for (int column = 0; column < 4; column += 1) {
            float expected = (static_cast<float>(matrix[row][column]) + 0.5f) / 16.0f;
            EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(static_cast<float>(column) + 0.5f, static_cast<float>(row) + 0.5f), 1.0f, 2), expected));
        }
    }
    // Zero bits: a 1×1 matrix, threshold 0.5. A cell size of 2 makes each cell 2×2 pixels wide.
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(3.0f, 3.0f), 1.0f, 0), 0.5f));
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(3.9f, 0.5f), 2.0f, 1), 0.625f));
    // A negative position is taken as 0.
    EXPECT(nearly(kernels::bayerMatrixThreshold(Kernel2(-3.0f, -1.0f), 1.0f, 2), 0.5f / 16.0f));
}

TEST(bayerThresholdsOfAMatrixAreDistinctAndAverageAHalf) {
    float sum = 0.0f;
    bool seen[64] = {};
    for (int row = 0; row < 8; row += 1) {
        for (int column = 0; column < 8; column += 1) {
            float threshold = kernels::bayerMatrixThreshold(Kernel2(static_cast<float>(column) + 0.5f, static_cast<float>(row) + 0.5f), 1.0f, 3);
            EXPECT(threshold > 0.0f && threshold < 1.0f);
            int index = static_cast<int>(threshold * 64.0f);
            EXPECT(!seen[index]);
            seen[index] = true;
            sum += threshold;
        }
    }
    EXPECT(nearly(sum / 64.0f, 0.5f));
}

TEST(ditheredLevelsRoundByTheThreshold) {
    // Two levels (steps 1): 0.4 + 0.5 floors to 0, 0.6 + 0.5 floors to 1.
    EXPECT(kernels::ditheredLevels(0.4f, 2.0f, 0.5f) == 0.0f);
    EXPECT(kernels::ditheredLevels(0.6f, 2.0f, 0.5f) == 1.0f);
    // Four levels (steps 3): 0.5 → ⌊1.5 + 0.5⌋ = 2 → 2/3; with threshold 0.1 → ⌊1.6⌋ = 1 → 1/3.
    EXPECT(nearly(kernels::ditheredLevels(0.5f, 4.0f, 0.5f), 2.0f / 3.0f));
    EXPECT(nearly(kernels::ditheredLevels(0.5f, 4.0f, 0.1f), 1.0f / 3.0f));
    // One level counts as two steps' worth (steps never under 1); out-of-range values clamp.
    EXPECT(kernels::ditheredLevels(2.0f, 1.0f, 0.5f) == 1.0f);
    EXPECT(kernels::ditheredLevels(-1.0f, 4.0f, 0.99f) == 0.0f);
}

TEST(gradientRampsRunFromZeroToOne) {
    Kernel2 centre(0.5f, 0.5f);
    // Linear: 0.5 at the centre, 0.75 a quarter canvas along +x; turned by π/2 it follows y, flipped.
    EXPECT(nearly(kernels::gradientRamp(centre, centre, 0.0f, 1.0f, 0), 0.5f));
    EXPECT(nearly(kernels::gradientRamp(Kernel2(0.75f, 0.5f), centre, 0.0f, 1.0f, 0), 0.75f));
    EXPECT(nearly(kernels::gradientRamp(Kernel2(0.5f, 0.75f), centre, kernels::pi<float>() * 0.5f, 1.0f, 0), 0.25f));
    // Radial: |d| / (size/2) = 0.5 / 1 for size 2 at distance 0.5; clamps at 1 beyond the reach.
    EXPECT(nearly(kernels::gradientRamp(Kernel2(1.0f, 0.5f), centre, 0.0f, 2.0f, 1), 0.5f));
    EXPECT(kernels::gradientRamp(Kernel2(9.0f, 0.5f), centre, 0.0f, 2.0f, 1) == 1.0f);
    // Conic: fract(atan2/τ + 1): +y is a quarter turn, −x a half turn, +x the start.
    EXPECT(nearly(kernels::gradientRamp(Kernel2(0.5f, 1.5f), centre, 0.0f, 1.0f, 2), 0.25f));
    EXPECT(nearly(kernels::gradientRamp(Kernel2(-0.5f, 0.5f), centre, 0.0f, 1.0f, 2), 0.5f));
    // Diamond and square, size 2 (reach 1) at d = (0.3, 0.2): 0.5 and 0.3.
    EXPECT(nearly(kernels::gradientRamp(Kernel2(0.8f, 0.7f), centre, 0.0f, 2.0f, 3), 0.5f));
    EXPECT(nearly(kernels::gradientRamp(Kernel2(0.8f, 0.7f), centre, 0.0f, 2.0f, 4), 0.3f));
}

TEST(everyPatternStaysInZeroToOne) {
    Kernel2 tiny(0.002f, 0.002f);
    for (int index = 0; index < sampleCount; index += 1) {
        Kernel2 p = samplePoint(index);
        EXPECT(isMask(kernels::checkerboardPattern(p, 0.0f, sharp2), 1e-3f));
        EXPECT(isMask(kernels::checkerboardPattern(p, 0.05f, sharp2), 1e-3f));
        EXPECT(isMask(kernels::stripesPattern(p, 5.0f, 0.3f, 0.4f, 0.0f, 0.02f).x));
        EXPECT(isMask(kernels::gridPattern(p, Kernel2(3.0f, 4.0f), 0.1f, 0.0f, tiny).x));
        EXPECT(isMask(kernels::dotsPattern(p, 3.0f, 0.3f, 0.0f, index % 2 == 0, sharp).x));
        EXPECT(isMask(kernels::bricksPattern(p, Kernel2(3.0f, 5.0f), 0.5f, 0.1f, 0.0f, tiny).x));
        EXPECT(isMask(kernels::hexagonTilesPattern(p, 2.0f, 0.1f, 0.0f, sharp).x));
        EXPECT(isMask(kernels::truchetPattern(p, 2.0f, 0.2f, 0.0f, index % 2, sharp)));
        EXPECT(isMask(kernels::ripplesPattern(p, 3.0f, 0.5f, 0.5f, index % 4)));
        EXPECT(isMask(kernels::spiralPattern(p, 3.0f, 2.0f, 0.1f, 0.3f, 0.0f, index % 2, sharp)));
        EXPECT(isMask(kernels::sunburstPattern(p, 8.0f, 0.2f, 0.4f, 0.0f, sharp)));
        EXPECT(isMask(kernels::zigzagPattern(p, 3.0f, 4.0f, 0.5f, 0.3f, 0.0f, index % 2, 0.02f)));
        EXPECT(isMask(kernels::herringbonePattern(p, 4.0f, 4.0f, 0.5f, 0.1f, 0.0f, tiny).x));
        EXPECT(isMask(kernels::basketweavePattern(p, 3.0f, 2.0f, 0.1f, 0.0f, sharp).x));
        EXPECT(isMask(kernels::halftonePattern(p, 0.5f, 6.0f, 0.4f, index % 3, sharp)));
        float ramp = kernels::gradientRamp(p, Kernel2(0.0f, 0.0f), 0.2f, 3.0f, index % 5);
        EXPECT(isMask(ramp));
    }
}
