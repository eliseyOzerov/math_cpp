// Photon splatting for caustics (Engine/Math/Optics/CausticSplatting.swift).
#include "CausticSplatting.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "../Geometry/Plane.h"
#include "../Geometry/Ray.h"
#include "RefractingInterface.h"

namespace editor {

Vector3 HeightField::normal(float x, float y) const {
    Vector2 gradient = slope(x, y);
    return Vector3(-gradient.x, -gradient.y, 1).normalized();
}

IrradianceGrid::IrradianceGrid(Vector2 lowerCorner, Vector2 upperCorner, int columns, int rows)
    : lowerCorner(lowerCorner), upperCorner(upperCorner), columns(columns), rows(rows) {
    assert(columns > 0 && rows > 0 && "A grid needs at least one cell.");
    assert(upperCorner.x > lowerCorner.x && upperCorner.y > lowerCorner.y
        && "The rectangle must have positive width and height.");
    cellValues.assign(static_cast<size_t>(columns * rows), 0.0f);
}

Vector2 IrradianceGrid::cellSize() const {
    return (upperCorner - lowerCorner) / Vector2(static_cast<float>(columns), static_cast<float>(rows));
}

float IrradianceGrid::cellArea() const { return cellSize().area(); }

float IrradianceGrid::totalPower() const {
    double sum = 0.0;
    for (float value : cellValues) sum = sum + static_cast<double>(value);
    return static_cast<float>(sum * static_cast<double>(cellArea()));
}

void IrradianceGrid::splat(float power, Vector2 position) {
    Vector2 cell = cellSize();
    // Cell centres sit half a cell in from the cell corners, hence the 0.5.
    Vector2 cellUnits = (position - lowerCorner) / cell - Vector2(0.5f, 0.5f);
    int firstColumn = static_cast<int>(std::floor(cellUnits.x));
    int firstRow = static_cast<int>(std::floor(cellUnits.y));
    float columnFraction = cellUnits.x - static_cast<float>(firstColumn);
    float rowFraction = cellUnits.y - static_cast<float>(firstRow);
    float irradiance = power / cellArea();
    const float rowShares[2] = {1 - rowFraction, rowFraction};
    const float columnShares[2] = {1 - columnFraction, columnFraction};
    for (int rowOffset = 0; rowOffset < 2; rowOffset += 1) {
        for (int columnOffset = 0; columnOffset < 2; columnOffset += 1) {
            int column = std::min(std::max(firstColumn + columnOffset, 0), columns - 1);
            int row = std::min(std::max(firstRow + rowOffset, 0), rows - 1);
            cellValues[static_cast<size_t>(row * columns + column)] +=
                irradiance * rowShares[rowOffset] * columnShares[columnOffset];
        }
    }
}

bool IrradianceGrid::contains(Vector2 position) const {
    return position.x >= lowerCorner.x && position.x <= upperCorner.x
        && position.y >= lowerCorner.y && position.y <= upperCorner.y;
}

CausticResult CausticSplatting::receiverIrradiance(
    const HeightField& surface,
    float refractiveIndexAbove,
    float refractiveIndexBelow,
    float lightIrradiance,
    Vector2 emissionLowerCorner,
    Vector2 emissionUpperCorner,
    int rayColumns,
    int rayRows,
    float receiverHeight,
    const IrradianceGrid& receiver) {
    assert(rayColumns > 0 && rayRows > 0 && "At least one ray is needed.");
    Vector2 emissionSize = emissionUpperCorner - emissionLowerCorner;
    Vector2 rayCellSize = emissionSize / Vector2(static_cast<float>(rayColumns), static_cast<float>(rayRows));
    float powerPerRay = lightIrradiance * rayCellSize.area();
    Plane receiverPlane(Vector3(0, 0, receiverHeight), Vector3::zAxis());
    IrradianceGrid grid = receiver;
    double transmittedPower = 0.0;
    double powerMissing = 0.0;

    for (int row = 0; row < rayRows; row += 1) {
        for (int column = 0; column < rayColumns; column += 1) {
            Vector2 rayCentre = Vector2::cellCentre(column, row, emissionLowerCorner, rayCellSize);
            float x = rayCentre.x;
            float y = rayCentre.y;
            Vector3 hitPoint(x, y, surface.height(x, y));
            WeightedRay downward{Ray(hitPoint, -Vector3::zAxis()), 1};
            InterfaceContinuation continuation = RefractingInterface::continuation(
                downward, hitPoint, surface.normal(x, y), refractiveIndexAbove, refractiveIndexBelow);
            if (continuation.wasReflected) continue;
            float power = powerPerRay * continuation.continued.weight;
            transmittedPower = transmittedPower + static_cast<double>(power);
            Ray refracted = continuation.continued.ray;
            if (std::optional<float> distance = refracted.intersectionDistance(receiverPlane)) {
                Vector3 landing = refracted.point(*distance);
                Vector2 landingPosition(landing.x, landing.y);
                if (grid.contains(landingPosition)) {
                    grid.splat(power, landingPosition);
                    continue;
                }
            }
            powerMissing = powerMissing + static_cast<double>(power);
        }
    }
    return CausticResult{
        grid,
        lightIrradiance * emissionSize.area(),
        static_cast<float>(transmittedPower),
        static_cast<float>(powerMissing)};
}

} // namespace editor
