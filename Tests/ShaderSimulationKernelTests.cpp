// The shader-library simulations (Kernels/ShaderSimulation.h) on the CPU: the Gray–Scott step, Conway's
// Life, the one-pass fluid and the stateless particle field, with the arithmetic of each worked out
// by hand. The texture reads that surround each step stay in the Metal backend, so these tests feed
// the functions the numbers a texture read would have produced. The same header is compiled as Metal by
// `make kernels` and compared on the GPU by `make gpu`.
#include <cmath>

#include "../Kernels/ShaderSimulation.h"
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
bool nearly(Kernel3 first, Kernel3 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance);
}
bool nearly(Kernel4 first, Kernel4 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance) && nearly(first.z, second.z, tolerance)
        && nearly(first.w, second.w, tolerance);
}
} // namespace

// ---- Gray–Scott -----------------------------------------------------------------------------------

TEST(grayScottLaplacianWeightsSumToZeroSoAFlatFieldHasNone) {
    // 4 × 0.2 + 4 × 0.05 − 1 = 0.8 + 0.2 − 1 = 0.
    EXPECT(nearly(4.0f * kernels::grayScottEdgeWeight() + 4.0f * kernels::grayScottCornerWeight(), 1.0f));
    Kernel2 centre(0.5f, 0.3f);
    EXPECT(nearly(kernels::grayScottLaplacian(centre, centre * 4.0f, centre * 4.0f), Kernel2(0.0f, 0.0f), 1e-6f));
}

TEST(grayScottLaplacianOfAPeakAndOfAMixedNeighbourhood) {
    // A peak of A with nothing around it: −centre = (−1, 0).
    EXPECT(nearly(kernels::grayScottLaplacian(Kernel2(1.0f, 0.0f), Kernel2(0.0f), Kernel2(0.0f)), Kernel2(-1.0f, 0.0f)));
    // x: 0.2 × 1.0 + 0.05 × 2.0 − 0.5 = 0.2 + 0.1 − 0.5 = −0.2; y: 0.2 × 0.4 + 0.05 × 0.8 − 0.2 = 0.08 + 0.04 − 0.2 = −0.08.
    EXPECT(nearly(kernels::grayScottLaplacian(Kernel2(0.5f, 0.2f), Kernel2(1.0f, 0.4f), Kernel2(2.0f, 0.8f)), Kernel2(-0.2f, -0.08f)));
}

TEST(grayScottReactionKeepsTheEmptyStateAndMatchesAHandWorkedStep) {
    // Only A, flat: nothing reacts and A is already full, so (1, 0) stays.
    EXPECT(nearly(kernels::grayScottReaction(Kernel2(1.0f, 0.0f), Kernel2(0.0f), 0.04f, 0.06f, 1.0f, 0.5f, 1.0f), Kernel2(1.0f, 0.0f)));
    // (a, b) = (0.5, 0.5), laplacian (0.1, −0.2), feed 0.04, kill 0.06, diffusion (1, 0.5), rate 1:
    // reaction = 0.5 × 0.25 = 0.125.
    // a' = 0.5 + (1 × 0.1 − 0.125 + 0.04 × 0.5) = 0.5 + (0.1 − 0.125 + 0.02) = 0.495.
    // b' = 0.5 + (0.5 × −0.2 + 0.125 − 0.1 × 0.5) = 0.5 + (−0.1 + 0.125 − 0.05) = 0.475.
    EXPECT(nearly(kernels::grayScottReaction(Kernel2(0.5f, 0.5f), Kernel2(0.1f, -0.2f), 0.04f, 0.06f, 1.0f, 0.5f, 1.0f),
                  Kernel2(0.495f, 0.475f)));
    // Rate scales the change: rate 0 changes nothing.
    EXPECT(nearly(kernels::grayScottReaction(Kernel2(0.5f, 0.5f), Kernel2(0.1f, -0.2f), 0.04f, 0.06f, 1.0f, 0.5f, 0.0f), Kernel2(0.5f, 0.5f)));
    // Without B, feeding refills A toward 1: a' = 0.2 + 0.1 × 0.8 = 0.28 and B stays 0.
    EXPECT(nearly(kernels::grayScottReaction(Kernel2(0.2f, 0.0f), Kernel2(0.0f), 0.1f, 0.06f, 1.0f, 0.5f, 1.0f), Kernel2(0.28f, 0.0f)));
}

TEST(grayScottReactionStaysInZeroToOne) {
    for (int stepA = 0; stepA <= 10; stepA += 1) {
        for (int stepB = 0; stepB <= 10; stepB += 1) {
            Kernel2 state(static_cast<float>(stepA) / 10.0f, static_cast<float>(stepB) / 10.0f);
            Kernel2 next = kernels::grayScottReaction(state, Kernel2(0.7f, -0.9f), 0.055f, 0.062f, 1.0f, 0.5f, 3.0f);
            EXPECT(next.x >= 0.0f && next.x <= 1.0f);
            EXPECT(next.y >= 0.0f && next.y <= 1.0f);
        }
    }
    // Rate 3 with a = b = 1: reaction 1, a' = 1 + (0 − 1 + 0) × 3 = −2, clamped to 0; b' = 1 + (0 + 1 − 0.115) × 3 clamped to 1.
    EXPECT(nearly(kernels::grayScottReaction(Kernel2(1.0f, 1.0f), Kernel2(0.0f), 0.055f, 0.06f, 1.0f, 0.5f, 3.0f), Kernel2(0.0f, 1.0f)));
}

TEST(grayScottBrushSetsBInsideItsDiscOnly) {
    Kernel2 texel(0.01f, 0.01f);
    Kernel2 brush(0.5f, 0.5f);
    Kernel2 state(0.3f, 0.2f);
    // The disc has radius 8 texels = 0.08. At 0.03 away the ratio is 0.375, inside; at 0.1 it is 1.25, outside.
    EXPECT(nearly(kernels::grayScottBrushed(state, Kernel2(0.53f, 0.5f), brush, 1.0f, texel), Kernel2(0.3f, 1.0f)));
    EXPECT(nearly(kernels::grayScottBrushed(state, Kernel2(0.6f, 0.5f), brush, 1.0f, texel), state));
    // Not brushing (the flag at 0.5 or less) leaves the state.
    EXPECT(nearly(kernels::grayScottBrushed(state, Kernel2(0.5f, 0.5f), brush, 0.0f, texel), state));
}

TEST(grayScottStartStateIsFullOfAWithBInTheMiddleDiscAndSomeSpecks) {
    Kernel2 texel(1.0f / 1024.0f, 1.0f / 1024.0f);
    // In the disc (|uv − 0.5| < 0.04): A = 1, B = 1, alpha 1.
    EXPECT(nearly(kernels::grayScottStartState(Kernel2(0.5f, 0.5f), texel), Kernel4(1.0f, 1.0f, 0.0f, 1.0f)));
    EXPECT(nearly(kernels::grayScottStartState(Kernel2(0.53f, 0.5f), texel), Kernel4(1.0f, 1.0f, 0.0f, 1.0f)));
    // Outside the disc and the specks are cells of 12 texels whose hash exceeds 0.985: about 1.5% of them.
    int specks = 0;
    int blocks = 0;
    for (int row = 0; row < 85; row += 1) {
        for (int column = 0; column < 85; column += 1) {
            Kernel2 uv((static_cast<float>(column) + 0.5f) * 12.0f / 1024.0f, (static_cast<float>(row) + 0.5f) * 12.0f / 1024.0f);
            if (kernels::lengthOf(uv - 0.5f) < 0.05f) continue;
            Kernel4 start = kernels::grayScottStartState(uv, texel);
            EXPECT(start.x == 1.0f && start.z == 0.0f && start.w == 1.0f);
            EXPECT(start.y == 0.0f || start.y == 1.0f);
            bool expected = kernels::hash21(kernels::roundedDown(uv / (texel * 12.0f))) > 0.985f;
            EXPECT((start.y == 1.0f) == expected);
            specks += start.y == 1.0f ? 1 : 0;
            blocks += 1;
        }
    }
    // About 1.5% of about 7000 blocks is about 105.
    EXPECT(specks > 60 && specks < 160);
    EXPECT(blocks > 6500);
}

TEST(grayScottViewBlendsFromLowToHighAsBGrows) {
    Kernel3 low(0.0f, 0.1f, 0.2f);
    Kernel3 high(1.0f, 0.9f, 0.8f);
    EXPECT(nearly(kernels::grayScottViewColor(Kernel4(1.0f, 0.0f, 0.0f, 1.0f), low, high), Kernel4(0.0f, 0.1f, 0.2f, 1.0f)));
    EXPECT(nearly(kernels::grayScottViewColor(Kernel4(0.0f, 0.5f, 0.0f, 0.3f), low, high), Kernel4(1.0f, 0.9f, 0.8f, 1.0f)));
    // B = 0.25 is halfway across 0…0.5, so the smooth step is 0.5 and the colour is the mean: (0.5, 0.5, 0.5).
    EXPECT(nearly(kernels::grayScottViewColor(Kernel4(0.0f, 0.25f, 0.0f, 1.0f), low, high), Kernel4(0.5f, 0.5f, 0.5f, 1.0f)));
}

// ---- Life -----------------------------------------------------------------------------------------

TEST(lifeRuleIsBirthOnThreeAndSurvivalOnTwoOrThree) {
    for (int neighbours = 0; neighbours <= 8; neighbours += 1) {
        // A dead cell is born with exactly 3 neighbours and stays dead otherwise.
        EXPECT(kernels::lifeRule(false, neighbours) == (neighbours == 3));
        // A live cell survives with 2 or 3 neighbours and dies of loneliness (< 2) or crowding (> 3).
        EXPECT(kernels::lifeRule(true, neighbours) == (neighbours == 2 || neighbours == 3));
    }
}

TEST(lifeNextStateLivesInWhiteAndLeavesAFadingBlueTrail) {
    // A live cell with 2 neighbours lives: white.
    EXPECT(nearly(kernels::lifeNextState(Kernel4(1.0f, 1.0f, 1.0f, 1.0f), 2, 0.5f), Kernel4(1.0f, 1.0f, 1.0f, 1.0f)));
    // A live cell with 4 neighbours dies and starts a full trail: (0.25, 0.6, 1, 1).
    EXPECT(nearly(kernels::lifeNextState(Kernel4(1.0f, 1.0f, 1.0f, 1.0f), 4, 0.5f), Kernel4(0.25f, 0.6f, 1.0f, 1.0f)));
    // A dead cell with a trail of 0.5, 1 neighbour, fade 0.5: trail = 0.25, colour (0.0625, 0.15, 0.25, 1).
    EXPECT(nearly(kernels::lifeNextState(Kernel4(0.125f, 0.3f, 0.5f, 1.0f), 1, 0.5f), Kernel4(0.0625f, 0.15f, 0.25f, 1.0f)));
    // A dead cell with exactly 3 neighbours is born.
    EXPECT(nearly(kernels::lifeNextState(Kernel4(0.0f, 0.0f, 0.0f, 1.0f), 3, 0.5f), Kernel4(1.0f, 1.0f, 1.0f, 1.0f)));
    // A trail keeps fading to nothing: fade 0.5 applied repeatedly.
    Kernel4 state(0.25f, 0.6f, 1.0f, 1.0f);
    for (int generation = 0; generation < 20; generation += 1) state = kernels::lifeNextState(state, 0, 0.5f);
    EXPECT(state.z < 1e-5f);
    EXPECT(!kernels::lifeCellIsAlive(state.x));
}

TEST(aBlinkerOnATorusOscillatesWithPeriodTwo) {
    // The classic oscillator: three cells in a column on a 5 × 5 wrapped grid turn into a row and back.
    constexpr int size = 5;
    bool alive[size][size] = {};
    alive[1][2] = alive[2][2] = alive[3][2] = true; // alive[row][column]
    Kernel4 states[size][size];
    for (int round = 0; round < 2; round += 1) {
        for (int row = 0; row < size; row += 1) {
            for (int column = 0; column < size; column += 1) {
                int neighbours = 0;
                for (int dy = -1; dy <= 1; dy += 1) {
                    for (int dx = -1; dx <= 1; dx += 1) {
                        if (dx == 0 && dy == 0) continue;
                        neighbours += alive[(row + dy + size) % size][(column + dx + size) % size] ? 1 : 0;
                    }
                }
                float red = alive[row][column] ? 1.0f : 0.0f;
                states[row][column] = kernels::lifeNextState(Kernel4(red, red, red, 1.0f), neighbours, 0.9f);
            }
        }
        for (int row = 0; row < size; row += 1) {
            for (int column = 0; column < size; column += 1) alive[row][column] = kernels::lifeCellIsAlive(states[row][column].x);
        }
        if (round == 0) {
            // After one generation the column is a row: row 2, columns 1…3.
            for (int row = 0; row < size; row += 1) {
                for (int column = 0; column < size; column += 1) EXPECT(alive[row][column] == (row == 2 && column >= 1 && column <= 3));
            }
        }
    }
    for (int row = 0; row < size; row += 1) {
        for (int column = 0; column < size; column += 1) EXPECT(alive[row][column] == (column == 2 && row >= 1 && row <= 3));
    }
}

TEST(lifeCellsAreLaidOutOnAGridAndTheirNeighboursWrap) {
    // 8-pixel cells on a 512 × 256 texture are 1/64 by 1/32 of the uv square; a cell of 0 pixels is 1.
    EXPECT(nearly(kernels::lifeCellStep(8.0f, Kernel2(512.0f, 256.0f)), Kernel2(1.0f / 64.0f, 1.0f / 32.0f)));
    EXPECT(nearly(kernels::lifeCellStep(0.0f, Kernel2(512.0f, 256.0f)), Kernel2(1.0f / 512.0f, 1.0f / 256.0f)));
    // Cells of 0.25: uv (0.3, 0.6) is in cell (⌊1.2⌋, ⌊2.4⌋) = (1, 2), whose centre is (1.5, 2.5) × 0.25 = (0.375, 0.625).
    EXPECT(nearly(kernels::lifeCellCentreUv(Kernel2(0.3f, 0.6f), Kernel2(0.25f, 0.25f)), Kernel2(0.375f, 0.625f)));
    // The neighbour to the left of the cell at 0.125 is at −0.125, which wraps to 0.875; above it is 0.375.
    EXPECT(nearly(kernels::lifeNeighbourPoint(Kernel2(0.125f, 0.125f), -1, 0, Kernel2(0.25f, 0.25f)), Kernel2(0.875f, 0.125f)));
    EXPECT(nearly(kernels::lifeNeighbourPoint(Kernel2(0.125f, 0.125f), 1, 1, Kernel2(0.25f, 0.25f)), Kernel2(0.375f, 0.375f)));
    EXPECT(nearly(kernels::lifeNeighbourPoint(Kernel2(0.875f, 0.875f), 1, 1, Kernel2(0.25f, 0.25f)), Kernel2(0.125f, 0.125f)));
}

TEST(lifeAdvancesOnceEveryIntervalFrames) {
    // Interval 3: frames 0, 3, 6 advance; 1, 2, 4, 5 do not.
    for (int frame = 0; frame < 9; frame += 1) EXPECT(kernels::lifeAdvances(static_cast<float>(frame), 3.0f) == (frame % 3 == 0));
    // An interval under 1 is 1: every frame.
    for (int frame = 0; frame < 5; frame += 1) EXPECT(kernels::lifeAdvances(static_cast<float>(frame), 0.0f));
}

TEST(lifeStartColorSeedsTheRequestedDensityOfLiveCells) {
    Kernel2 step(1.0f / 64.0f, 1.0f / 64.0f);
    int aliveAtThirty = 0;
    for (int row = 0; row < 64; row += 1) {
        for (int column = 0; column < 64; column += 1) {
            Kernel2 uv((static_cast<float>(column) + 0.5f) / 64.0f, (static_cast<float>(row) + 0.5f) / 64.0f);
            Kernel4 none = kernels::lifeStartColor(uv, step, 0.0f);
            Kernel4 all = kernels::lifeStartColor(uv, step, 2.0f);
            Kernel4 some = kernels::lifeStartColor(uv, step, 0.3f);
            EXPECT(nearly(none, Kernel4(0.0f, 0.0f, 0.0f, 1.0f)));
            EXPECT(nearly(all, Kernel4(1.0f, 1.0f, 1.0f, 1.0f)));
            EXPECT(some.x == some.y && some.y == some.z && some.w == 1.0f);
            EXPECT(some.x == 0.0f || some.x == 1.0f);
            aliveAtThirty += some.x == 1.0f ? 1 : 0;
        }
    }
    // 30% of 4096 cells is 1229; the hash is uniform, so allow a few standard deviations (about 29 each).
    EXPECT(aliveAtThirty > 1100 && aliveAtThirty < 1360);
    // The same cell always starts the same way, wherever in the cell the pixel is.
    EXPECT(nearly(kernels::lifeStartColor(Kernel2(0.301f, 0.402f), step, 0.5f), kernels::lifeStartColor(Kernel2(0.304f, 0.405f), step, 0.5f)));
}

TEST(lifeViewShowsLiveCellsAndABlueGlowOnTheTrail) {
    Kernel3 alive(1.0f, 1.0f, 1.0f);
    Kernel3 dead(0.1f, 0.1f, 0.2f);
    EXPECT(nearly(kernels::lifeViewColor(Kernel4(1.0f, 1.0f, 1.0f, 1.0f), alive, dead), Kernel4(1.0f, 1.0f, 1.0f, 1.0f)));
    // No trail: the dead colour. Full trail: dead + (0, 0.15, 0.4) = (0.1, 0.25, 0.6). Half trail: the mean.
    EXPECT(nearly(kernels::lifeViewColor(Kernel4(0.0f, 0.0f, 0.0f, 1.0f), alive, dead), Kernel4(0.1f, 0.1f, 0.2f, 1.0f)));
    EXPECT(nearly(kernels::lifeViewColor(Kernel4(0.25f, 0.6f, 1.0f, 1.0f), alive, dead), Kernel4(0.1f, 0.25f, 0.6f, 1.0f)));
    EXPECT(nearly(kernels::lifeViewColor(Kernel4(0.125f, 0.3f, 0.5f, 1.0f), alive, dead), Kernel4(0.1f, 0.175f, 0.4f, 1.0f)));
}

// ---- Fluid ----------------------------------------------------------------------------------------

TEST(fluidVelocityDecodesZeroPointFiveAsStill) {
    EXPECT(nearly(kernels::fluidVelocityOf(Kernel2(0.5f, 0.5f)), Kernel2(0.0f, 0.0f)));
    EXPECT(nearly(kernels::fluidVelocityOf(Kernel2(1.0f, 0.0f)), Kernel2(1.0f, -1.0f)));
    EXPECT(nearly(kernels::fluidVelocityOf(Kernel2(0.0f, 0.75f)), Kernel2(-1.0f, 0.5f)));
}

TEST(fluidDivergenceIsZeroForUniformFlowAndPositiveForExpansion) {
    // Uniform flow: every neighbour the same, nothing leaves.
    Kernel2 flow(0.3f, -0.2f);
    EXPECT(nearly(kernels::fluidDivergenceOf(flow, flow, flow, flow), 0.0f));
    // The field v = (x, y): right − left = 2 in x, up − down = 2 in y; ½ (2 + 2) = 2.
    EXPECT(nearly(kernels::fluidDivergenceOf(Kernel2(1.0f, 0.0f), Kernel2(-1.0f, 0.0f), Kernel2(0.0f, 1.0f), Kernel2(0.0f, -1.0f)), 2.0f));
    // Only the matching components count: ½ ((0.5 − (−0.5)) + (0.2 − (−0.2))) = ½ (1 + 0.4) = 0.7.
    EXPECT(nearly(kernels::fluidDivergenceOf(Kernel2(0.5f, 9.0f), Kernel2(-0.5f, 9.0f), Kernel2(9.0f, 0.2f), Kernel2(9.0f, -0.2f)), 0.7f));
    // A rotation (v = (−y, x)) has no divergence: right.x − left.x = 0, up.y − down.y = 0.
    EXPECT(nearly(kernels::fluidDivergenceOf(Kernel2(0.0f, 1.0f), Kernel2(0.0f, -1.0f), Kernel2(-1.0f, 0.0f), Kernel2(1.0f, 0.0f)), 0.0f));
}

TEST(fluidAdvectionLooksBackAlongTheVelocity) {
    // Velocity (0.5, −1), texel (0.01, 0.02): 20 texels back is (0.5 × 0.2, −1 × 0.4) = (0.1, −0.4) away from (0.5, 0.5): (0.4, 0.9).
    EXPECT(nearly(kernels::fluidAdvectionOrigin(Kernel2(0.5f, 0.5f), Kernel2(0.5f, -1.0f), Kernel2(0.01f, 0.02f)), Kernel2(0.4f, 0.9f)));
    // Still fluid comes from where it is.
    EXPECT(nearly(kernels::fluidAdvectionOrigin(Kernel2(0.3f, 0.7f), Kernel2(0.0f), Kernel2(0.01f, 0.02f)), Kernel2(0.3f, 0.7f)));
    EXPECT(nearly(kernels::fluidAspect(640.0f, 480.0f), Kernel2(4.0f / 3.0f, 1.0f)));
}

TEST(fluidViscosityBlendsTowardTheNeighbourAverage) {
    Kernel2 moving(1.0f, 0.0f);
    Kernel2 zero(0.0f, 0.0f);
    // Viscosity 0 keeps the velocity; 1 is the average of the four neighbours (here 0); 0.5 is halfway.
    EXPECT(nearly(kernels::fluidViscousVelocity(moving, zero, zero, zero, zero, 0.0f), moving));
    EXPECT(nearly(kernels::fluidViscousVelocity(moving, zero, zero, zero, zero, 1.0f), zero));
    EXPECT(nearly(kernels::fluidViscousVelocity(moving, zero, zero, zero, zero, 0.5f), Kernel2(0.5f, 0.0f)));
    // Above 1 is clamped: still the pure average. The average of (1,0), (3,0), (0,1), (0,3) is (1, 1).
    EXPECT(nearly(kernels::fluidViscousVelocity(moving, Kernel2(1.0f, 0.0f), Kernel2(3.0f, 0.0f), Kernel2(0.0f, 1.0f), Kernel2(0.0f, 3.0f), 5.0f),
                  Kernel2(1.0f, 1.0f)));
}

TEST(fluidDampingStepsAlongTheGradientOfTheDivergence) {
    // damping 0.5 × ½ × (0.4 − 0.2, 0.3 − 0.1) = 0.25 × (0.2, 0.2) = (0.05, 0.05).
    EXPECT(nearly(kernels::fluidDampedVelocity(Kernel2(0.0f), 0.4f, 0.2f, 0.3f, 0.1f, 0.5f), Kernel2(0.05f, 0.05f)));
    // No damping, or a flat divergence, leaves the velocity.
    EXPECT(nearly(kernels::fluidDampedVelocity(Kernel2(0.2f, -0.1f), 0.4f, 0.2f, 0.3f, 0.1f, 0.0f), Kernel2(0.2f, -0.1f)));
    EXPECT(nearly(kernels::fluidDampedVelocity(Kernel2(0.2f, -0.1f), 0.3f, 0.3f, 0.2f, 0.2f, 1.0f), Kernel2(0.2f, -0.1f)));
}

TEST(fluidFalloffIsAGaussianOfTheDistanceToTheSource) {
    Kernel2 source(0.5f, 0.5f);
    Kernel2 round(1.0f, 1.0f);
    EXPECT(nearly(kernels::fluidFalloff(source, source, round, 0.2f), 1.0f));
    // 0.1 away on a texture twice as wide as tall (aspect (2, 1)): the distance is 0.2 = the radius, so
    // exp(−0.04 / (0.04 + 0.000001)) = exp(−0.999975) = 0.367888.
    EXPECT(nearly(kernels::fluidFalloff(Kernel2(0.6f, 0.5f), source, Kernel2(2.0f, 1.0f), 0.2f), 0.367888f, 1e-4f));
    // The same offset along y is not stretched: distance 0.1, exp(−0.01 / 0.04) = exp(−0.25) = 0.778801.
    EXPECT(nearly(kernels::fluidFalloff(Kernel2(0.5f, 0.6f), source, Kernel2(2.0f, 1.0f), 0.2f), 0.778801f, 1e-4f));
    // A zero radius is a point: 1 at the source (0 / 0.000001), practically 0 anywhere else.
    EXPECT(nearly(kernels::fluidFalloff(source, source, round, 0.0f), 1.0f));
    EXPECT(kernels::fluidFalloff(Kernel2(0.51f, 0.5f), source, round, 0.0f) < 1e-6f);
    // It falls with distance and never leaves 0…1.
    float previous = 1.0f;
    for (int step = 1; step <= 10; step += 1) {
        float falloff = kernels::fluidFalloff(Kernel2(0.5f + 0.05f * static_cast<float>(step), 0.5f), source, round, 0.2f);
        EXPECT(falloff < previous && falloff > 0.0f);
        previous = falloff;
    }
}

TEST(fluidNextStatePushesPacksAndFeedsTheDye) {
    // moving (0.2, −0.4), push (3, 4) × force 0.1 × falloff 1 = (0.3, 0.4): velocity (0.5, 0), stored (0.75, 0.5).
    // Dye: 0.5 × 0.9 + 1 × min(|push| 5 × 0.1 × 4 = 2, 1) = 0.45 + 1 = 1.45, clamped to 1.
    EXPECT(nearly(kernels::fluidNextState(Kernel2(0.2f, -0.4f), 0.5f, Kernel2(3.0f, 4.0f), 0.1f, 1.0f, 0.9f), Kernel4(0.75f, 0.5f, 1.0f, 1.0f)));
    // No push: the velocity is kept ((0.2, −0.4) → stored (0.6, 0.3)) and the dye only fades: 0.5 × 0.9 = 0.45.
    EXPECT(nearly(kernels::fluidNextState(Kernel2(0.2f, -0.4f), 0.5f, Kernel2(0.0f), 0.1f, 1.0f, 0.9f), Kernel4(0.6f, 0.3f, 0.45f, 1.0f)));
    // A weak push: push (0.1, 0), force 1, falloff 0.5: velocity += (0.05, 0); dye = 0.5 × min(0.4, 1) = 0.2 (no old dye).
    EXPECT(nearly(kernels::fluidNextState(Kernel2(0.0f), 0.0f, Kernel2(0.1f, 0.0f), 1.0f, 0.5f, 0.9f), Kernel4(0.525f, 0.5f, 0.2f, 1.0f)));
    // A velocity beyond ±1 saturates when stored: (3, −3) → (1, 0).
    EXPECT(nearly(kernels::fluidNextState(Kernel2(3.0f, -3.0f), 0.0f, Kernel2(0.0f), 0.0f, 0.0f, 1.0f), Kernel4(1.0f, 0.0f, 0.0f, 1.0f)));
    // Still water with no push is the start state.
    EXPECT(nearly(kernels::fluidNextState(Kernel2(0.0f), 0.0f, Kernel2(0.0f), 1.0f, 1.0f, 1.0f), kernels::fluidStartState()));
    EXPECT(nearly(kernels::fluidStartState(), Kernel4(0.5f, 0.5f, 0.0f, 1.0f)));
}

TEST(fluidViewShowsDyeOverDarkBlueWithAGlowWhereItMoves) {
    Kernel3 dye(1.0f, 0.0f, 0.5f);
    // Still, no dye: the dark blue (0.02, 0.03, 0.06). Still, full dye: the dye colour.
    EXPECT(nearly(kernels::fluidViewColor(Kernel4(0.5f, 0.5f, 0.0f, 1.0f), dye), Kernel4(0.02f, 0.03f, 0.06f, 1.0f)));
    EXPECT(nearly(kernels::fluidViewColor(Kernel4(0.5f, 0.5f, 1.0f, 1.0f), dye), Kernel4(1.0f, 0.0f, 0.5f, 1.0f)));
    // Velocity (1, 0): speed = clamp(3, 0, 1) = 1, adding (0.15, 0.25, 0.4) to the dark blue.
    EXPECT(nearly(kernels::fluidViewColor(Kernel4(1.0f, 0.5f, 0.0f, 1.0f), dye), Kernel4(0.17f, 0.28f, 0.46f, 1.0f)));
    // Velocity (0.1, 0): speed 0.3.
    EXPECT(nearly(kernels::fluidViewColor(Kernel4(0.55f, 0.5f, 0.0f, 1.0f), dye),
                  Kernel4(0.02f + 0.3f * 0.15f, 0.03f + 0.3f * 0.25f, 0.06f + 0.3f * 0.4f, 1.0f), 1e-5f));
}

// ---- Particles ------------------------------------------------------------------------------------

TEST(particlePositionFollowsEachMotion) {
    Kernel2 random(0.3f, 0.6f);
    Kernel2 random2(0.8f, 0.1f);
    Kernel2 aspect(2.0f, 1.0f);
    // At time 0 the angle 0.6 τ = 216° has cos −0.809017 and sin −0.587785.
    // Drifting (0): fract(random) = (0.3, 0.6), life 1.
    EXPECT(nearly(kernels::particlePosition(random, random2, aspect, 1.0f, 0.0f, 1.0f, 0), Kernel3(0.3f, 0.6f, 1.0f)));
    // At time 5, speed 1: random + (random2 − 0.5) × 5 × 0.2 = (0.3 + 0.3 × 1, 0.6 + (−0.4) × 1) = (0.6, 0.2).
    EXPECT(nearly(kernels::particlePosition(random, random2, aspect, 1.0f, 5.0f, 1.0f, 0), Kernel3(0.6f, 0.2f, 1.0f), 1e-5f));
    // Rising (1) at time 0: x = 0.3 + 0.03 sin(216°) = 0.3 − 0.017634 = 0.282366; y = fract(0.6) = 0.6; life = 0.4.
    EXPECT(nearly(kernels::particlePosition(random, random2, aspect, 1.0f, 0.0f, 1.0f, 1), Kernel3(0.282366f, 0.6f, 0.4f), 1e-4f));
    // Orbiting (2): radius 0.05 + 0.4 × 0.3 = 0.17; position = 0.5 + 0.17 (−0.809017, −0.587785) / (2, 1) = (0.431234, 0.400077).
    EXPECT(nearly(kernels::particlePosition(random, random2, aspect, 1.0f, 0.0f, 1.0f, 2), Kernel3(0.431234f, 0.400077f, 1.0f), 1e-4f));
    // Outward (3): age = 0.3; position = 0.5 + (−0.809017, −0.587785) × 0.18 / (2, 1) = (0.427189, 0.394199); life 0.7.
    EXPECT(nearly(kernels::particlePosition(random, random2, aspect, 1.0f, 0.0f, 1.0f, 3), Kernel3(0.427189f, 0.394199f, 0.7f), 1e-4f));
    // Any other mode number is the outward one.
    EXPECT(nearly(kernels::particlePosition(random, random2, aspect, 1.0f, 0.0f, 1.0f, 9), kernels::particlePosition(random, random2, aspect, 1.0f, 0.0f, 1.0f, 3)));
    // Speed 0 freezes every motion except the sway of the rising one, which depends on the time directly.
    EXPECT(nearly(kernels::particlePosition(random, random2, aspect, 0.0f, 100.0f, 1.0f, 0), Kernel3(0.3f, 0.6f, 1.0f)));
}

TEST(orbitingParticlesStayOnTheirCircleAndOutwardOnesFade) {
    Kernel2 aspect(1.6f, 1.0f);
    for (int index = 0; index < 20; index += 1) {
        Kernel2 random = kernels::hash22(Kernel2(static_cast<float>(index), 7.0f));
        Kernel2 random2 = kernels::hash22(Kernel2(static_cast<float>(index) + 100.0f, 7.0f));
        for (int step = 0; step < 6; step += 1) {
            float time = static_cast<float>(step) * 1.7f;
            // Orbit: the distance from the middle, measured in aspect-corrected units, is the radius whatever the time.
            Kernel3 orbit = kernels::particlePosition(random, random2, aspect, 1.0f, time, 0.8f, 2);
            Kernel2 fromMiddle = (Kernel2(orbit.x, orbit.y) - 0.5f) * aspect;
            EXPECT(nearly(kernels::lengthOf(fromMiddle), 0.05f + 0.4f * random.x * 0.8f, 1e-4f));
            // Outward: life is 1 − age, within 0…1, and the particle is never farther than 0.6 × spread from the middle.
            Kernel3 outward = kernels::particlePosition(random, random2, aspect, 1.0f, time, 0.8f, 3);
            EXPECT(outward.z >= 0.0f && outward.z <= 1.0f);
            EXPECT(kernels::lengthOf((Kernel2(outward.x, outward.y) - 0.5f) * aspect) <= 0.6f * 0.8f + 1e-4f);
            // Rising: life is 1 minus its height.
            Kernel3 rising = kernels::particlePosition(random, random2, aspect, 1.0f, time, 0.8f, 1);
            EXPECT(nearly(rising.z, 1.0f - rising.y));
        }
    }
}

TEST(particleFieldDrawsOneSoftDotAtEachParticle) {
    Kernel2 aspect(1.0f, 1.0f);
    float size = 0.1f;
    // One drifting particle at time 0 sits at fract(random) of its index 0 and seed 3.
    Kernel2 random = kernels::hash22(Kernel2(0.0f, 3.0f));
    Kernel2 random2 = kernels::hash22(Kernel2(100.0f, 3.0f));
    Kernel2 where = kernels::fractionalPart(random);
    Kernel3 hue = kernels::rgbFromHSV(Kernel3(random2.y, 0.5f, 1.0f));
    // On the particle: distance 0, shape 1, density 1 and the hue colour.
    EXPECT(nearly(kernels::particleField(where, aspect, 1.0f, size, 1.0f, 0.0f, 3.0f, 0, 1.0f), Kernel4(hue.x, hue.y, hue.z, 1.0f), 1e-5f));
    // 0.6 × size away the smooth step from `size` down to 0.2 `size` is halfway: t = (0.06 − 0.1) / (0.02 − 0.1) = 0.5, shape 0.5.
    Kernel4 halfway = kernels::particleField(Kernel2(where.x + 0.6f * size, where.y), aspect, 1.0f, size, 1.0f, 0.0f, 3.0f, 0, 1.0f);
    EXPECT(nearly(halfway, Kernel4(hue.x * 0.5f, hue.y * 0.5f, hue.z * 0.5f, 0.5f), 1e-5f));
    // Farther than `size`: nothing.
    EXPECT(nearly(kernels::particleField(Kernel2(where.x + 1.5f * size, where.y), aspect, 1.0f, size, 1.0f, 0.0f, 3.0f, 0, 1.0f), Kernel4(0.0f), 1e-6f));
    // The dot is measured with the aspect, so a wide screen keeps it round: aspect (2, 1) halves the uv distance along x.
    Kernel2 wide(2.0f, 1.0f);
    Kernel2 wideWhere = kernels::fractionalPart(random);
    Kernel4 rounded = kernels::particleField(Kernel2(wideWhere.x + 0.3f * size, wideWhere.y), wide, 1.0f, size, 1.0f, 0.0f, 3.0f, 0, 1.0f);
    EXPECT(nearly(rounded.w, 0.5f, 1e-5f));
}

TEST(particleFieldCountIsBoundedAndDensityStaysInZeroToOne) {
    Kernel2 aspect(1.5f, 1.0f);
    Kernel2 uv(0.4f, 0.55f);
    // No particles, or a negative count: nothing.
    EXPECT(nearly(kernels::particleField(uv, aspect, 0.0f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f), Kernel4(0.0f)));
    EXPECT(nearly(kernels::particleField(uv, aspect, -5.0f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f), Kernel4(0.0f)));
    // More than 64 is cut to 64; a fractional count is truncated (64.9 is 64).
    EXPECT(kernels::particleMaximumCount() == 64);
    EXPECT(nearly(kernels::particleField(uv, aspect, 100.0f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f), kernels::particleField(uv, aspect, 64.0f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f)));
    EXPECT(nearly(kernels::particleField(uv, aspect, 3.9f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f), kernels::particleField(uv, aspect, 3.0f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f)));
    // Many large particles overlap: the density is capped at 1 while the colour keeps adding.
    for (int motion = 0; motion < 4; motion += 1) {
        for (int step = 0; step < 10; step += 1) {
            Kernel4 field = kernels::particleField(Kernel2(0.1f * static_cast<float>(step), 0.5f), aspect, 64.0f, 0.4f, 1.0f, 2.5f, 5.0f, motion, 1.0f);
            EXPECT(field.w >= 0.0f && field.w <= 1.0f);
            EXPECT(field.x >= 0.0f && field.y >= 0.0f && field.z >= 0.0f);
        }
    }
    // A different seed gives a different arrangement; the same seed the same one.
    Kernel4 first = kernels::particleField(uv, aspect, 40.0f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f);
    Kernel4 again = kernels::particleField(uv, aspect, 40.0f, 0.2f, 1.0f, 1.0f, 1.0f, 0, 1.0f);
    Kernel4 other = kernels::particleField(uv, aspect, 40.0f, 0.2f, 1.0f, 1.0f, 2.0f, 0, 1.0f);
    EXPECT(nearly(first, again, 0.0f));
    EXPECT(!nearly(first, other, 1e-4f));
}
