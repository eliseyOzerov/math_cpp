// Flat panels laid on a ball by central projection (Engine/Geometry/Mesh+SphericalPanels.swift).
#include "SphericalPanels.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <map>
#include <string>

#include "../Linear/GeometryProblem.h"

namespace editor {

std::optional<SphericalPanel> SphericalPanel::make(Polygon2D outline, Vector3 origin, Vector3 xAxis, Vector3 upAxis) {
    std::optional<Vector3> unitX = xAxis.unitDirection();
    if (!unitX) return std::nullopt;
    std::optional<Vector3> unitY = (upAxis - *unitX * upAxis.dot(*unitX)).unitDirection();
    if (!unitY) return std::nullopt;
    Vector3 normal = unitX->cross(*unitY);
    if (!(std::fabs(normal.dot(origin)) > 1e-4f * std::max(origin.length(), 1e-20f))) return std::nullopt;
    return SphericalPanel(std::move(outline), origin, *unitX, *unitY);
}

Vector3 SphericalPanel::point(Vector2 flat) const { return planeOrigin + unitX * flat.x + unitY * flat.y; }

float SphericalPanel::planeDistance() const { return unitX.cross(unitY).dot(planeOrigin); }

long long SphericalPanel::gridDivisions(float radius, float spacing) const {
    const std::vector<Vector2>& corners = outlinePolygon.corners();
    float longestSide = 0;
    for (const Polygon2D::CornerTriangle& triangle : outlinePolygon.triangulated()) {
        Vector2 first = corners[static_cast<size_t>(triangle.first)];
        Vector2 second = corners[static_cast<size_t>(triangle.second)];
        Vector2 third = corners[static_cast<size_t>(triangle.third)];
        longestSide = std::max({longestSide, first.distance(second), second.distance(third), third.distance(first)});
    }
    float divisions = std::ceil(longestSide * radius / (std::fabs(planeDistance()) * spacing));
    if (!std::isfinite(divisions)) return LLONG_MAX;
    return std::max(1LL, static_cast<long long>(std::min(divisions, static_cast<float>(INT_MAX))));
}

PanelledSphere sphericalPanels(const std::vector<SphericalPanel>& panels, const std::vector<int>& divisions, float radius) {
    if (!(std::isfinite(radius) && radius > 0)) throw GeometryProblem("Spherical panels need a positive radius.");
    if (divisions.size() != panels.size()) throw GeometryProblem("Spherical panels need one division count per panel.");
    std::vector<Vector3> positions;
    std::vector<Mesh::Face> faces;
    std::vector<int> faceNumbers;
    std::vector<std::vector<Vector3>> borders;
    for (size_t panelIndex = 0; panelIndex < panels.size(); ++panelIndex) {
        const SphericalPanel& panel = panels[panelIndex];
        int panelNumber = static_cast<int>(panelIndex);
        int gridSize = divisions[panelIndex];
        if (!(gridSize >= 1 && gridSize <= SphericalPanel::maximumDivisions)) {
            throw GeometryProblem("A spherical panel needs 1 to " + std::to_string(SphericalPanel::maximumDivisions)
                                  + " divisions, but got " + std::to_string(gridSize) + ".");
        }
        const std::vector<Vector2>& corners = panel.outline().corners();
        bool facesOutward = panel.planeDistance() > 0;
        // A grid point's name: its outline corners with a weight above 0, in corner order, as
        // [corner, weight, corner, weight, ...].
        std::map<std::vector<int>, int> pointNumbers;
        struct WeightedCorner {
            int corner;
            int weight;
        };
        auto pointNumber = [&](std::vector<WeightedCorner> weightedCorners) {
            std::vector<WeightedCorner> ordered;
            for (const WeightedCorner& entry : weightedCorners) {
                if (entry.weight > 0) ordered.push_back(entry);
            }
            std::sort(ordered.begin(), ordered.end(), [](const WeightedCorner& first, const WeightedCorner& second) {
                return first.corner < second.corner;
            });
            std::vector<int> name;
            for (const WeightedCorner& entry : ordered) {
                name.push_back(entry.corner);
                name.push_back(entry.weight);
            }
            auto existing = pointNumbers.find(name);
            if (existing != pointNumbers.end()) return existing->second;
            Vector2 flatSum;
            for (const WeightedCorner& entry : ordered) {
                flatSum = flatSum + corners[static_cast<size_t>(entry.corner)] * static_cast<float>(entry.weight);
            }
            Vector3 onPlane = panel.point(flatSum / static_cast<float>(gridSize));
            positions.push_back(onPlane.normalized() * radius);
            int number = static_cast<int>(positions.size()) - 1;
            pointNumbers[name] = number;
            return number;
        };
        auto addFace = [&](int first, int second, int third) {
            faces.push_back(Mesh::Face{facesOutward ? std::vector<int>{first, second, third} : std::vector<int>{third, second, first}});
            faceNumbers.push_back(panelNumber);
        };
        for (const Polygon2D::CornerTriangle& triangle : panel.outline().triangulated()) {
            // P(i, j): weight i on the second corner, j on the third, the rest on the first.
            auto gridPoint = [&](int i, int j) {
                return pointNumber({{triangle.first, gridSize - i - j}, {triangle.second, i}, {triangle.third, j}});
            };
            for (int j = 0; j < gridSize; ++j) {
                for (int i = 0; i < gridSize - j; ++i) {
                    addFace(gridPoint(i, j), gridPoint(i + 1, j), gridPoint(i, j + 1));
                    if (i + j < gridSize - 1) {
                        addFace(gridPoint(i + 1, j), gridPoint(i + 1, j + 1), gridPoint(i, j + 1));
                    }
                }
            }
        }
        std::vector<Vector3> border;
        for (size_t corner = 0; corner < corners.size(); ++corner) {
            int next = static_cast<int>((corner + 1) % corners.size());
            for (int step = 0; step < gridSize; ++step) {
                border.push_back(positions[static_cast<size_t>(
                    pointNumber({{static_cast<int>(corner), gridSize - step}, {next, step}}))]);
            }
        }
        border.push_back(border[0]);
        borders.push_back(border);
    }
    Mesh ball;
    ball.positions = positions;
    ball.faces = faces;
    return PanelledSphere{ball, faceNumbers, borders};
}

} // namespace editor
