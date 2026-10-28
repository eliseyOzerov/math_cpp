// Naive surface nets (Engine/Math/Fields/SurfaceNets.swift and DistanceField+Meshing.swift).
#include "SurfaceNets.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"
#include "DistanceField.h"

namespace editor {

namespace {

/// The offsets of a cell's 8 corners from its lowest corner: bit 0 of the corner number is
/// x, bit 1 is y, bit 2 is z.
std::array<std::array<int, 3>, 8> makeCornerOffsets() {
    std::array<std::array<int, 3>, 8> offsets{};
    for (int corner = 0; corner < 8; corner += 1) {
        offsets[corner] = {corner & 1, (corner >> 1) & 1, (corner >> 2) & 1};
    }
    return offsets;
}

const std::array<std::array<int, 3>, 8> cornerOffsets = makeCornerOffsets();

/// The 12 edges of a cell as pairs of corner numbers (each pair differs in one bit).
const std::array<std::pair<int, int>, 12> cellEdges = {{
    {0, 1}, {2, 3}, {4, 5}, {6, 7},   // along x
    {0, 2}, {1, 3}, {4, 6}, {5, 7},   // along y
    {0, 4}, {1, 5}, {2, 6}, {3, 7},   // along z
}};

/// A whole-number double the way Swift prints it (`17000000000.0`), for the limit message.
std::string wholeNumberText(double value) {
    char buffer[64];
    if (value < 1e16) {
        std::snprintf(buffer, sizeof buffer, "%.1f", value);
    } else {
        std::snprintf(buffer, sizeof buffer, "%.17g", value);
    }
    return buffer;
}

} // namespace

// MARK: TriangleMesh

Triangle TriangleMesh::triangle(int index) const {
    return Triangle{
        positions[triangleIndices[3 * index]],
        positions[triangleIndices[3 * index + 1]],
        positions[triangleIndices[3 * index + 2]],
    };
}

float TriangleMesh::enclosedVolume() const {
    float sum = 0;
    for (int index = 0; index < triangleCount(); index += 1) {
        Triangle corners = triangle(index);
        // The tetrahedron of the triangle and the origin: first · (second × third) / 6.
        Vector3 apex(0, 0, 0);
        sum += (corners.first - apex).dot((corners.second - apex).cross(corners.third - apex)) / 6;
    }
    return sum;
}

Mesh TriangleMesh::mesh() const {
    Mesh result;
    result.positions = positions;
    for (int triangle = 0; triangle < triangleCount(); triangle += 1) {
        result.faces.push_back(Mesh::Face{{static_cast<int>(triangleIndices[3 * triangle]),
                                           static_cast<int>(triangleIndices[3 * triangle + 1]),
                                           static_cast<int>(triangleIndices[3 * triangle + 2])}});
    }
    return result;
}

// MARK: SurfaceNetsGrid

SurfaceNetsGrid::SurfaceNetsGrid(Vector3 origin, float cellSize, CellCounts cellCounts,
                                 const std::function<float(Vector3)>& field)
    : origin(origin), cellSize(cellSize), cellCounts(cellCounts) {
    values.reserve(static_cast<std::size_t>(cellCounts[0] + 1) * (cellCounts[1] + 1) * (cellCounts[2] + 1));
    for (int z = 0; z <= cellCounts[2]; z += 1) {
        for (int y = 0; y <= cellCounts[1]; y += 1) {
            for (int x = 0; x <= cellCounts[0]; x += 1) {
                values.push_back(field(cornerPosition(x, y, z)));
            }
        }
    }
    vertexOfCell.assign(static_cast<std::size_t>(cellCounts[0]) * cellCounts[1] * cellCounts[2], -1);
}

SurfaceNetsGrid::SurfaceNetsGrid(Vector3 origin, float cellSize, CellCounts cellCounts, std::vector<float> values)
    : origin(origin), cellSize(cellSize), cellCounts(cellCounts), values(std::move(values)) {
    if (this->values.size() != static_cast<std::size_t>(cellCounts[0] + 1) * (cellCounts[1] + 1) * (cellCounts[2] + 1)) {
        throw MathProblem("One value per lattice corner.");
    }
    vertexOfCell.assign(static_cast<std::size_t>(cellCounts[0]) * cellCounts[1] * cellCounts[2], -1);
}

Vector3 SurfaceNetsGrid::cornerPosition(int x, int y, int z, Vector3 origin, float cellSize) {
    return origin + Vector3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)) * cellSize;
}

// Where lattice corner (x, y, z) of this grid is.
Vector3 SurfaceNetsGrid::cornerPosition(int x, int y, int z) const {
    return cornerPosition(x, y, z, origin, cellSize);
}

float SurfaceNetsGrid::cornerValue(int x, int y, int z) const {
    return values[(static_cast<std::size_t>(z) * (cellCounts[1] + 1) + y) * (cellCounts[0] + 1) + x];
}

int SurfaceNetsGrid::cellIndex(int x, int y, int z) const { return (z * cellCounts[1] + y) * cellCounts[0] + x; }

SurfaceNetsGrid::CellCounts SurfaceNetsGrid::cellCountsForSize(const std::array<float, 3>& sizes, float cellSize) {
    std::array<float, 3> ratios{};
    for (int axis = 0; axis < 3; axis += 1) ratios[axis] = largerOf(sizes[axis] / cellSize, 0);
    for (float ratio : ratios) {
        if (!std::isfinite(ratio)) {
            throw MathProblem("The bounds divided by the cell size must be finite: check for infinite bounds or a cell size that is too small.");
        }
    }
    const float wholeTolerance = 1e-5f;
    std::array<float, 3> rounded{};
    for (int axis = 0; axis < 3; axis += 1) {
        float ratio = ratios[axis];
        float nearest = std::round(ratio);
        float count = std::fabs(ratio - nearest) <= wholeTolerance * nearest ? nearest : std::ceil(ratio);
        rounded[axis] = largerOf(count, 1);
    }
    // The product in Double: three counts below 2²⁴ each could overflow an Int product.
    double total = 1.0;
    for (float count : rounded) total = total * static_cast<double>(count);
    if (!(total <= static_cast<double>(maximumCellCount))) {
        throw MathProblem("The grid would have " + wholeNumberText(total) + " cells; the limit is "
                          + std::to_string(maximumCellCount) + ". Use a larger cell size or smaller bounds.");
    }
    return {static_cast<int>(rounded[0]), static_cast<int>(rounded[1]), static_cast<int>(rounded[2])};
}

// Whether a sample counts as inside the shape (value below 0). NaN is not inside.
bool SurfaceNetsGrid::isInside(float value) { return value < 0; }

// Whether the surface passes between two lattice corners: their insideness differs and
// neither value is NaN (a NaN end has no zero to interpolate to).
bool SurfaceNetsGrid::surfaceCrosses(float valueA, float valueB) {
    return !std::isnan(valueA) && !std::isnan(valueB) && isInside(valueA) != isInside(valueB);
}

TriangleMesh SurfaceNetsGrid::mesh() {
    TriangleMesh result;
    placeVertices(result);
    joinVertices(result);
    return result;
}

// Whether no edge of a cell can cross the surface, judged from its 8 corner values: all of
// them are numbers and all are on the same side. Most cells are like this, and the test is
// far cheaper than walking the 12 edges. A cell with a NaN corner is never ruled out here.
bool SurfaceNetsGrid::cellIsOnOneSide(const std::array<float, 8>& corners) {
    int insideCount = 0;
    for (float value : corners) {
        if (std::isnan(value)) return false;
        if (isInside(value)) insideCount += 1;
    }
    return insideCount == 0 || insideCount == static_cast<int>(corners.size());
}

// Step 2: one vertex in every cell with a sign change, at the mean of its edge crossings.
void SurfaceNetsGrid::placeVertices(TriangleMesh& mesh) {
    // How far apart neighbouring corners are in `values`, along y and along z.
    int rowStride = cellCounts[0] + 1;
    int planeStride = rowStride * (cellCounts[1] + 1);
    std::array<int, 8> cornerSteps{};
    for (int corner = 0; corner < 8; corner += 1) {
        cornerSteps[corner] = cornerOffsets[corner][0] + cornerOffsets[corner][1] * rowStride
            + cornerOffsets[corner][2] * planeStride;
    }
    std::array<float, 8> corners{};
    for (int z = 0; z < cellCounts[2]; z += 1) {
        for (int y = 0; y < cellCounts[1]; y += 1) {
            for (int x = 0; x < cellCounts[0]; x += 1) {
                std::size_t lowestCorner = (static_cast<std::size_t>(z) * (cellCounts[1] + 1) + y) * rowStride + x;
                for (int corner = 0; corner < 8; corner += 1) corners[corner] = values[lowestCorner + cornerSteps[corner]];
                if (cellIsOnOneSide(corners)) continue;
                Vector3 crossingSum(0, 0, 0);
                float crossingCount = 0;
                for (const auto& [cornerA, cornerB] : cellEdges) {
                    const std::array<int, 3>& offsetA = cornerOffsets[cornerA];
                    const std::array<int, 3>& offsetB = cornerOffsets[cornerB];
                    float valueA = cornerValue(x + offsetA[0], y + offsetA[1], z + offsetA[2]);
                    float valueB = cornerValue(x + offsetB[0], y + offsetB[1], z + offsetB[2]);
                    if (!surfaceCrosses(valueA, valueB)) continue;
                    // The zero of the line through (0, valueA) and (1, valueB).
                    float fraction = valueA / (valueA - valueB);
                    crossingSum += Vector3::linearBlend(
                        cornerPosition(x + offsetA[0], y + offsetA[1], z + offsetA[2]),
                        cornerPosition(x + offsetB[0], y + offsetB[1], z + offsetB[2]), fraction);
                    crossingCount += 1;
                }
                if (!(crossingCount > 0)) continue;
                vertexOfCell[cellIndex(x, y, z)] = static_cast<std::int32_t>(mesh.positions.size());
                mesh.positions.push_back(crossingSum / crossingCount);
            }
        }
    }
}

// Step 3: a quad for every lattice edge that crosses the surface and has four cells around it.
void SurfaceNetsGrid::joinVertices(TriangleMesh& mesh) const {
    int rowStride = cellCounts[0] + 1;
    int planeStride = rowStride * (cellCounts[1] + 1);
    // How far the far end of an edge along x, y or z is from its near end in `values`.
    const std::array<int, 3> edgeSteps = {1, rowStride, planeStride};
    // The four cells around an edge, counterclockwise about +axis, as offsets along the other two axes.
    const std::array<std::pair<int, int>, 4> aroundOffsets = {{{-1, -1}, {0, -1}, {0, 0}, {-1, 0}}};
    for (int z = 0; z <= cellCounts[2]; z += 1) {
        for (int y = 0; y <= cellCounts[1]; y += 1) {
            for (int x = 0; x <= cellCounts[0]; x += 1) {
                std::size_t nearIndex = (static_cast<std::size_t>(z) * (cellCounts[1] + 1) + y) * rowStride + x;
                float nearValue = values[nearIndex];
                bool nearIsInside = isInside(nearValue);
                for (int axis = 0; axis < 3; axis += 1) {
                    int coordinate = axis == 0 ? x : axis == 1 ? y : z;
                    if (!(coordinate < cellCounts[axis])) continue;
                    float farValue = values[nearIndex + edgeSteps[axis]];
                    if (!surfaceCrosses(nearValue, farValue)) continue;
                    std::array<int, 3> corner = {x, y, z};

                    // The other two axes, in right-handed cyclic order, so that
                    // sideB × sideC points along `axis`.
                    int sideB = (axis + 1) % 3;
                    int sideC = (axis + 2) % 3;
                    if (!(corner[sideB] >= 1 && corner[sideC] >= 1 && corner[sideB] < cellCounts[sideB]
                          && corner[sideC] < cellCounts[sideC])) {
                        continue;
                    }
                    std::array<std::uint32_t, 4> around{};
                    for (int index = 0; index < 4; index += 1) {
                        std::array<int, 3> cell = corner;
                        cell[sideB] += aroundOffsets[index].first;
                        cell[sideC] += aroundOffsets[index].second;
                        around[index] = static_cast<std::uint32_t>(vertexOfCell[cellIndex(cell[0], cell[1], cell[2])]);
                    }
                    // Counterclockwise about +axis faces +axis, right when the far end is
                    // the outside one. If the near end is outside, the order is reversed.
                    if (nearIsInside) {
                        addQuad(around, mesh);
                    } else {
                        addQuad({around[3], around[2], around[1], around[0]}, mesh);
                    }
                }
            }
        }
    }
}

// Splits a quad into two triangles along its shorter diagonal.
void SurfaceNetsGrid::addQuad(const std::array<std::uint32_t, 4>& quad, TriangleMesh& mesh) {
    Vector3 first = mesh.positions[quad[0]];
    Vector3 second = mesh.positions[quad[1]];
    Vector3 third = mesh.positions[quad[2]];
    Vector3 fourth = mesh.positions[quad[3]];
    if (first.distance(third) <= second.distance(fourth)) {
        mesh.triangleIndices.insert(mesh.triangleIndices.end(), {quad[0], quad[1], quad[2], quad[0], quad[2], quad[3]});
    } else {
        mesh.triangleIndices.insert(mesh.triangleIndices.end(), {quad[0], quad[1], quad[3], quad[1], quad[2], quad[3]});
    }
}

// MARK: DistanceField meshing

TriangleMesh DistanceField::extractSurface(const Bounds& bounds, float cellSize) const {
    if (!(cellSize > 0)) throw MathProblem("The cell size must be positive.");
    Vector3 size = bounds.size();
    SurfaceNetsGrid::CellCounts cellCounts = SurfaceNetsGrid::cellCountsForSize({size.x, size.y, size.z}, cellSize);
    NarrowBandSample sampled = SurfaceNetsGrid::narrowBandValues(
        bounds.minimum, cellSize, cellCounts, [this](Vector3 point) { return signedDistance(point); });
    SurfaceNetsGrid grid(bounds.minimum, cellSize, cellCounts, std::move(sampled.values));
    return grid.mesh();
}

TriangleMesh DistanceField::extractSurfaceSamplingEveryCorner(const Bounds& bounds, float cellSize) const {
    if (!(cellSize > 0)) throw MathProblem("The cell size must be positive.");
    Vector3 size = bounds.size();
    SurfaceNetsGrid::CellCounts cellCounts = SurfaceNetsGrid::cellCountsForSize({size.x, size.y, size.z}, cellSize);
    SurfaceNetsGrid grid(bounds.minimum, cellSize, cellCounts, [this](Vector3 point) { return signedDistance(point); });
    return grid.mesh();
}

Bounds DistanceField::paddedMeshingBox(float cellSize) const {
    if (!boundingBox) throw MathProblem("Meshing a field needs a bounded field.");
    return expanded(*boundingBox, 2 * cellSize);
}

std::int64_t DistanceField::meshingCellCount(float cellSize) const {
    Vector3 size = paddedMeshingBox(cellSize).size();
    double cells = 1.0;
    for (float length : {size.x, size.y, size.z}) {
        cells = cells * std::ceil(static_cast<double>(length) / static_cast<double>(cellSize));
    }
    const std::int64_t capped = std::numeric_limits<std::int64_t>::max() / 2;
    return cells >= static_cast<double>(capped) ? capped : static_cast<std::int64_t>(cells);
}

Mesh DistanceField::meshedSurface(float cellSize, int smoothingIterations) const {
    return extractSurface(paddedMeshingBox(cellSize), cellSize).mesh().taubinSmoothed(smoothingIterations);
}

} // namespace editor
