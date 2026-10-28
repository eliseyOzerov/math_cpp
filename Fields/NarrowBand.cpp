// Narrow-band sampling for surface nets (Engine/Math/Fields/NarrowBand.swift).
#include "NarrowBand.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "../Linear/Scalar.h"
#include "SurfaceNets.h"

namespace editor {

float NarrowBand::cubeHalfDiagonal(float side) { return side * std::sqrt(3.0f) / 2; }

bool NarrowBand::surfaceCannotReach(float centreValue, float side) {
    return std::fabs(centreValue) > roundingAllowance * cubeHalfDiagonal(side);
}

bool NarrowBand::roundingIsSmall(float farthestCoordinate, float cellSize) {
    return farthestCoordinate <= largestCoordinateInCells * cellSize;
}

namespace {

/// Runs `body(0)` … `body(iterations − 1)` on every core, the way Swift's
/// `DispatchQueue.concurrentPerform` does: the iterations are handed out one by one to a few
/// threads, and the call returns when all are done. A problem thrown inside is thrown again
/// here, after the others finish.
void performConcurrently(int iterations, const std::function<void(int)>& body) {
    if (iterations <= 0) return;
    unsigned workerCount = std::thread::hardware_concurrency();
    if (workerCount == 0) workerCount = 1;
    if (static_cast<int>(workerCount) > iterations) workerCount = static_cast<unsigned>(iterations);

    std::atomic<int> nextIteration{0};
    std::exception_ptr firstProblem;
    std::mutex problemLock;
    auto work = [&] {
        for (int iteration = nextIteration.fetch_add(1); iteration < iterations; iteration = nextIteration.fetch_add(1)) {
            try {
                body(iteration);
            } catch (...) {
                std::lock_guard<std::mutex> guard(problemLock);
                if (!firstProblem) firstProblem = std::current_exception();
            }
        }
    };
    std::vector<std::thread> workers;
    for (unsigned index = 1; index < workerCount; index += 1) workers.emplace_back(work);
    work();
    for (std::thread& worker : workers) worker.join();
    if (firstProblem) std::rethrow_exception(firstProblem);
}

/// The leaf blocks of a grid, numbered x fastest.
struct LeafGrid {
    /// Leaves along x, y and z.
    std::array<int, 3> counts;

    /// How many leaves the grid holds: the product of `counts`.
    ///
    /// Use when: sizing the per-leaf tables of the narrow band
    /// Tags: leaf count, block count, grid size
    /// Node: inside fieldToMesh
    std::size_t total() const { return static_cast<std::size_t>(counts[0]) * counts[1] * counts[2]; }

    /// The number of leaf (x, y, z) in a flat table, x fastest:
    /// `(z × counts.y + y) × counts.x + x`.
    ///
    /// Use when: reading or marking one leaf of the narrow band
    /// Tags: flat index, linear index, row-major, block index
    /// Node: inside fieldToMesh
    std::size_t index(int x, int y, int z) const {
        return (static_cast<std::size_t>(z) * counts[1] + y) * counts[0] + x;
    }
};

/// The one or two leaves a lattice corner touches along one axis.
struct LeafPair {
    /// The leaf a corner starts, and the leaf it ends when it lies on a leaf boundary.
    LeafPair(int first, std::optional<int> second)
        : first(first), second(second ? *second : first), count(second ? 2 : 1) {}

    int operator[](int side) const { return side == 0 ? first : second; }

    int first;
    int second;
    /// 1 or 2.
    int count;
};

} // namespace

NarrowBandSample SurfaceNetsGrid::narrowBandValues(Vector3 origin, float cellSize, CellCounts cellCounts,
                                                   const std::function<float(Vector3)>& field) {
    const int leafCells = NarrowBand::leafCells;
    LeafGrid leaves{{(cellCounts[0] + leafCells - 1) / leafCells, (cellCounts[1] + leafCells - 1) / leafCells,
                     (cellCounts[2] + leafCells - 1) / leafCells}};
    const int topLeaves = 1 << NarrowBand::levels;
    std::array<int, 3> topCounts = {(leaves.counts[0] + topLeaves - 1) / topLeaves,
                                    (leaves.counts[1] + topLeaves - 1) / topLeaves,
                                    (leaves.counts[2] + topLeaves - 1) / topLeaves};
    float farthestCoordinate = 0;
    for (int axis = 0; axis < 3; axis += 1) {
        float reach = largerOf(std::fabs(origin[axis]),
                               std::fabs(origin[axis] + static_cast<float>(cellCounts[axis]) * cellSize));
        if (axis == 0 || reach > farthestCoordinate) farthestCoordinate = reach;
    }
    bool canSkip = NarrowBand::roundingIsSmall(farthestCoordinate, cellSize);
    // Far from the origin every leaf counts as near, so step 1 is skipped. (Bytes, not
    // `vector<bool>`, so threads writing different leaves never share a word.)
    std::vector<std::uint8_t> nearSurface(leaves.total(), canSkip ? 0 : 1);
    std::vector<float> farValue(leaves.total(), 0);

    // Step 1: classify the leaves, one top block per iteration. Each top block writes only the
    // leaves inside it, so the writes never overlap.

    // Tests the block of `size` leaves along each axis whose lowest leaf is `first`, and marks
    // its leaves, splitting it while it may hold surface.
    std::function<void(const std::array<int, 3>&, int)> classify = [&](const std::array<int, 3>& first, int size) {
        float side = static_cast<float>(size * leafCells) * cellSize;
        // The centre in cells from the origin: the lowest corner plus half the side.
        Vector3 centreInCells = Vector3(static_cast<float>(first[0] * leafCells), static_cast<float>(first[1] * leafCells),
                                        static_cast<float>(first[2] * leafCells))
            + Vector3::repeating(static_cast<float>(size * leafCells) / 2);
        float centreValue = field(origin + centreInCells * cellSize);
        std::array<int, 3> last = {std::min(first[0] + size, leaves.counts[0]), std::min(first[1] + size, leaves.counts[1]),
                                   std::min(first[2] + size, leaves.counts[2])};
        if (NarrowBand::surfaceCannotReach(centreValue, side)) {
            for (int z = first[2]; z < last[2]; z += 1) {
                for (int y = first[1]; y < last[1]; y += 1) {
                    for (int x = first[0]; x < last[0]; x += 1) farValue[leaves.index(x, y, z)] = centreValue;
                }
            }
        } else if (size == 1) {
            nearSurface[leaves.index(first[0], first[1], first[2])] = 1;
        } else {
            int half = size / 2;
            for (int octant = 0; octant < 8; octant += 1) {
                std::array<int, 3> child = {first[0] + (octant & 1) * half, first[1] + ((octant >> 1) & 1) * half,
                                            first[2] + ((octant >> 2) & 1) * half};
                if (!(child[0] < leaves.counts[0] && child[1] < leaves.counts[1] && child[2] < leaves.counts[2])) continue;
                classify(child, half);
            }
        }
    };

    performConcurrently(canSkip ? topCounts[0] * topCounts[1] * topCounts[2] : 0, [&](int block) {
        int x = block % topCounts[0];
        int y = (block / topCounts[0]) % topCounts[1];
        int z = block / (topCounts[0] * topCounts[1]);
        classify({x * topLeaves, y * topLeaves, z * topLeaves}, topLeaves);
    });

    // The leaves a lattice corner touches along each axis: the leaf it starts, and the leaf
    // it ends when it lies on a leaf boundary.
    std::array<std::vector<LeafPair>, 3> touching;
    for (int axis = 0; axis < 3; axis += 1) {
        for (int corner = 0; corner <= cellCounts[axis]; corner += 1) {
            std::optional<int> starting;
            if (corner / leafCells < leaves.counts[axis]) starting = corner / leafCells;
            std::optional<int> ending;
            if (corner % leafCells == 0 && corner > 0) ending = corner / leafCells - 1;
            touching[axis].push_back(LeafPair(starting ? *starting : *ending, starting ? ending : std::nullopt));
        }
    }
    const std::vector<LeafPair>& touchingX = touching[0];
    const std::vector<LeafPair>& touchingY = touching[1];
    const std::vector<LeafPair>& touchingZ = touching[2];

    // Step 2: sample the corners, one lattice plane (constant z) per iteration.
    std::array<int, 3> cornerCounts = {cellCounts[0] + 1, cellCounts[1] + 1, cellCounts[2] + 1};
    std::size_t planeSize = static_cast<std::size_t>(cornerCounts[0]) * cornerCounts[1];
    std::vector<float> values(planeSize * cornerCounts[2], 0);
    std::vector<std::int64_t> sampledPerPlane(cornerCounts[2], 0);
    // Each plane writes only its own corners and its own count.
    performConcurrently(cornerCounts[2], [&](int z) {
        std::int64_t sampled = 0;
        const LeafPair& leavesZ = touchingZ[z];
        for (int y = 0; y < cornerCounts[1]; y += 1) {
            const LeafPair& leavesY = touchingY[y];
            for (int x = 0; x < cornerCounts[0]; x += 1) {
                const LeafPair& leavesX = touchingX[x];
                bool isNear = false;
                for (int zSide = 0; zSide < leavesZ.count; zSide += 1) {
                    for (int ySide = 0; ySide < leavesY.count; ySide += 1) {
                        for (int xSide = 0; xSide < leavesX.count; xSide += 1) {
                            if (nearSurface[leaves.index(leavesX[xSide], leavesY[ySide], leavesZ[zSide])]) isNear = true;
                        }
                    }
                }
                std::size_t index = z * planeSize + static_cast<std::size_t>(y) * cornerCounts[0] + x;
                if (isNear) {
                    values[index] = field(SurfaceNetsGrid::cornerPosition(x, y, z, origin, cellSize));
                    sampled += 1;
                } else {
                    values[index] = farValue[leaves.index(leavesX[0], leavesY[0], leavesZ[0])];
                }
            }
        }
        sampledPerPlane[z] = sampled;
    });
    std::int64_t sampledCorners = 0;
    for (std::int64_t count : sampledPerPlane) sampledCorners += count;
    return NarrowBandSample{std::move(values), sampledCorners};
}

} // namespace editor
