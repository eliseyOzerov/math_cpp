// Perspective, lattice and puppet-pin deformations (see Deform2D.h for the sources).
#include "Deform2D.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#include "../Linear/MathProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

namespace {

using Matrix3x3 = std::array<double, 9>;

/// The matrix of the unit-square-to-quadrilateral map, last entry 1 (Heckbert 1989, section 2.2).
Matrix3x3 squareToQuadrilateralMatrix(const std::array<Vector2, 4>& corners) {
    const double x0 = corners[0].x, y0 = corners[0].y, x1 = corners[1].x, y1 = corners[1].y;
    const double x2 = corners[2].x, y2 = corners[2].y, x3 = corners[3].x, y3 = corners[3].y;
    const double sumX = x0 - x1 + x2 - x3;
    const double sumY = y0 - y1 + y2 - y3;
    if (sumX == 0 && sumY == 0) return {x1 - x0, x2 - x1, x0, y1 - y0, y2 - y1, y0, 0, 0, 1};
    const double deltaX1 = x1 - x2, deltaX2 = x3 - x2, deltaY1 = y1 - y2, deltaY2 = y3 - y2;
    const double determinant = deltaX1 * deltaY2 - deltaX2 * deltaY1;
    if (std::fabs(determinant) < 1e-12) throw MathProblem("Three corners of the quadrilateral lie in a line, so no perspective map fits it.");
    const double g = (sumX * deltaY2 - sumY * deltaX2) / determinant;
    const double h = (deltaX1 * sumY - deltaY1 * sumX) / determinant;
    return {x1 - x0 + g * x1, x3 - x0 + h * x3, x0, y1 - y0 + g * y1, y3 - y0 + h * y3, y0, g, h, 1};
}

/// The adjugate (transposed cofactors) of a 3×3 matrix: the inverse times the determinant.
Matrix3x3 adjugate(const Matrix3x3& m) {
    return {m[4] * m[8] - m[5] * m[7], m[2] * m[7] - m[1] * m[8], m[1] * m[5] - m[2] * m[4],
            m[5] * m[6] - m[3] * m[8], m[0] * m[8] - m[2] * m[6], m[2] * m[3] - m[0] * m[5],
            m[3] * m[7] - m[4] * m[6], m[1] * m[6] - m[0] * m[7], m[0] * m[4] - m[1] * m[3]};
}

Matrix3x3 product(const Matrix3x3& left, const Matrix3x3& right) {
    Matrix3x3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += left[row * 3 + k] * right[k * 3 + column];
            result[row * 3 + column] = sum;
        }
    }
    return result;
}

/// The homography a matrix stands for, scaled so its last entry is 1.
Homography homographyOf(const Matrix3x3& m) {
    if (std::fabs(m[8]) < 1e-12) throw MathProblem("This perspective map cannot be written with a last entry of 1.");
    Homography result;
    for (int index = 0; index < 8; ++index) result.entries[index] = static_cast<float>(m[index] / m[8]);
    return result;
}

Matrix3x3 matrixOf(const Homography& homography) {
    Matrix3x3 m{};
    for (int index = 0; index < 8; ++index) m[index] = homography.entries[index];
    m[8] = 1;
    return m;
}

} // namespace

Homography Homography::squareToQuadrilateral(const std::array<Vector2, 4>& corners) {
    return homographyOf(squareToQuadrilateralMatrix(corners));
}

Homography Homography::fromQuadrilaterals(const std::array<Vector2, 4>& from, const std::array<Vector2, 4>& to) {
    const Matrix3x3 towardsSquare = adjugate(squareToQuadrilateralMatrix(from));
    return homographyOf(product(squareToQuadrilateralMatrix(to), towardsSquare));
}

Vector2 Homography::applied(Vector2 point) const {
    const double x = point.x, y = point.y;
    const double w = entries[6] * x + entries[7] * y + 1;
    return Vector2(static_cast<float>((entries[0] * x + entries[1] * y + entries[2]) / w),
                   static_cast<float>((entries[3] * x + entries[4] * y + entries[5]) / w));
}

Homography Homography::inverse() const {
    const Matrix3x3 m = matrixOf(*this);
    const double determinant = m[0] * (m[4] - m[5] * m[7]) - m[1] * (m[3] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (std::fabs(determinant) < 1e-12) throw MathProblem("This perspective map flattens the plane, so it has no inverse.");
    return homographyOf(adjugate(m));
}

Warp2D::PlaneMap Homography::planeMap() const {
    Homography copy = *this;
    return [copy](Vector2 point) { return copy.applied(point); };
}

// MARK: Lattice

Lattice2D Lattice2D::undeformed(Vector2 lowerCorner, Vector2 upperCorner, int columns, int rows) {
    if (columns < 2 || rows < 2) throw MathProblem("A lattice needs at least 2 columns and 2 rows of points.");
    if (!(upperCorner.x > lowerCorner.x) || !(upperCorner.y > lowerCorner.y)) throw MathProblem("A lattice needs a box with some width and height.");
    Lattice2D lattice;
    lattice.lowerCorner = lowerCorner;
    lattice.upperCorner = upperCorner;
    lattice.columns = columns;
    lattice.rows = rows;
    for (int index = 0; index < columns * rows; ++index) lattice.points.push_back(lattice.restPoint(index));
    return lattice;
}

Vector2 Lattice2D::restPoint(int index) const {
    const int column = index % columns;
    const int row = index / columns;
    return Vector2(linearBlend(lowerCorner.x, upperCorner.x, static_cast<float>(column) / static_cast<float>(columns - 1)),
                   linearBlend(lowerCorner.y, upperCorner.y, static_cast<float>(row) / static_cast<float>(rows - 1)));
}

namespace {

/// The point at `amount` of the Bézier curve whose control points are `controls`, by de Casteljau:
/// blend neighbours until one point is left.
Vector2 deCasteljau(std::vector<Vector2> controls, float amount) {
    for (size_t level = controls.size() - 1; level > 0; --level) {
        for (size_t index = 0; index < level; ++index) controls[index] = Vector2::linearBlend(controls[index], controls[index + 1], amount);
    }
    return controls[0];
}

} // namespace

Vector2 Lattice2D::applied(Vector2 point) const {
    if (static_cast<int>(points.size()) != columns * rows) throw MathProblem("The lattice does not have columns × rows points.");
    const float u = (point.x - lowerCorner.x) / (upperCorner.x - lowerCorner.x);
    const float v = (point.y - lowerCorner.y) / (upperCorner.y - lowerCorner.y);
    std::vector<Vector2> rowResults;
    for (int row = 0; row < rows; ++row) {
        rowResults.push_back(deCasteljau(std::vector<Vector2>(points.begin() + row * columns, points.begin() + (row + 1) * columns), u));
    }
    return deCasteljau(std::move(rowResults), v);
}

Warp2D::PlaneMap Lattice2D::planeMap() const {
    Lattice2D copy = *this;
    return [copy](Vector2 point) { return copy.applied(point); };
}

// MARK: Puppet

namespace {

struct Point64 {
    double x = 0, y = 0;
};

Point64 toPoint64(Vector2 v) { return Point64{v.x, v.y}; }

/// The barycentric coordinates of `p` in the triangle abc: p = b₀·a + b₁·b + b₂·c, summing to 1.
std::array<double, 3> barycentric(Point64 a, Point64 b, Point64 c, Point64 p) {
    const double determinant = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    const double second = ((p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y)) / determinant;
    const double third = ((b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y)) / determinant;
    return {1 - second - third, second, third};
}

} // namespace

PuppetMesh::PuppetMesh(Bounds2 box, int cellsAlongLongerSide, const std::vector<Vector2>& pinRestPositions)
    : pinRestList(pinRestPositions), box(box) {
    buildGrid(cellsAlongLongerSide);
    for (Vector2 pin : pinRestPositions) insertPin(pin);
    buildNeighbours();
    buildCells();
}

void PuppetMesh::buildGrid(int cellsAlongLongerSide) {
    Vector2 size = box.size();
    if (!(size.x > 0)) size.x = size.y > 0 ? size.y : 1;
    if (!(size.y > 0)) size.y = size.x;
    box = Bounds2{box.minimum, box.minimum + size};
    const int cells = std::max(1, cellsAlongLongerSide);
    const float cell = std::max(size.x, size.y) / static_cast<float>(cells);
    cellColumns = std::max(1, static_cast<int>(std::lround(size.x / cell)));
    cellRows = std::max(1, static_cast<int>(std::lround(size.y / cell)));
    for (int row = 0; row <= cellRows; ++row) {
        for (int column = 0; column <= cellColumns; ++column) {
            vertexList.push_back(box.minimum + Vector2(size.x * static_cast<float>(column) / static_cast<float>(cellColumns),
                                                       size.y * static_cast<float>(row) / static_cast<float>(cellRows)));
        }
    }
    const auto vertexAt = [this](int column, int row) { return row * (cellColumns + 1) + column; };
    for (int row = 0; row < cellRows; ++row) {
        for (int column = 0; column < cellColumns; ++column) {
            const int lowerLeft = vertexAt(column, row), lowerRight = vertexAt(column + 1, row);
            const int upperLeft = vertexAt(column, row + 1), upperRight = vertexAt(column + 1, row + 1);
            // Alternating diagonals so no direction is favoured. Every triangle is counterclockwise.
            if ((column + row) % 2 == 0) {
                triangleList.push_back({lowerLeft, lowerRight, upperRight});
                triangleList.push_back({lowerLeft, upperRight, upperLeft});
            } else {
                triangleList.push_back({lowerLeft, lowerRight, upperLeft});
                triangleList.push_back({lowerRight, upperRight, upperLeft});
            }
        }
    }
}

void PuppetMesh::insertPin(Vector2 restPosition) {
    for (size_t index = 0; index < triangleList.size(); ++index) {
        const std::array<int, 3> triangle = triangleList[index];
        const std::array<double, 3> b = barycentric(toPoint64(vertexList[triangle[0]]), toPoint64(vertexList[triangle[1]]),
                                                    toPoint64(vertexList[triangle[2]]), toPoint64(restPosition));
        if (b[0] < -1e-9 || b[1] < -1e-9 || b[2] < -1e-9) continue;
        const int largest = static_cast<int>(std::max_element(b.begin(), b.end()) - b.begin());
        if (b[largest] >= 0.98) {
            pinVertices.push_back(triangle[largest]);
            return;
        }
        std::array<double, 3> placed = b;
        const double smallest = *std::min_element(b.begin(), b.end());
        if (smallest < 0.02) {
            const double share = (0.02 - smallest) / (1.0 / 3.0 - smallest);
            for (double& coordinate : placed) coordinate = (1 - share) * coordinate + share / 3.0;
        }
        const Vector2 position = vertexList[triangle[0]] * static_cast<float>(placed[0]) + vertexList[triangle[1]] * static_cast<float>(placed[1])
            + vertexList[triangle[2]] * static_cast<float>(placed[2]);
        const int added = static_cast<int>(vertexList.size());
        vertexList.push_back(position);
        pinVertices.push_back(added);
        triangleList[index] = {triangle[0], triangle[1], added};
        triangleList.push_back({triangle[1], triangle[2], added});
        triangleList.push_back({triangle[2], triangle[0], added});
        return;
    }
    throw MathProblem("A pin lies outside the puppet mesh.");
}

void PuppetMesh::buildNeighbours() {
    std::map<std::pair<int, int>, double> weights;
    for (const std::array<int, 3>& triangle : triangleList) {
        for (int corner = 0; corner < 3; ++corner) {
            const int apex = triangle[corner];
            const int first = triangle[(corner + 1) % 3];
            const int second = triangle[(corner + 2) % 3];
            const Point64 toFirst{vertexList[first].x - double(vertexList[apex].x), vertexList[first].y - double(vertexList[apex].y)};
            const Point64 toSecond{vertexList[second].x - double(vertexList[apex].x), vertexList[second].y - double(vertexList[apex].y)};
            const double cross = std::fabs(toFirst.x * toSecond.y - toFirst.y * toSecond.x);
            const double dot = toFirst.x * toSecond.x + toFirst.y * toSecond.y;
            // cot(angle at the apex) = dot ÷ |cross|; the weight of the opposite edge is half of it.
            weights[{std::min(first, second), std::max(first, second)}] += 0.5 * dot / std::max(cross, 1e-18);
        }
    }
    neighbours.assign(vertexList.size(), {});
    for (const auto& [edge, weight] : weights) {
        const double held = std::min(std::max(weight, 0.0), 100.0);
        if (held <= 0) continue;
        neighbours[edge.first].push_back({edge.second, held});
        neighbours[edge.second].push_back({edge.first, held});
    }
}

void PuppetMesh::buildCells() {
    cellTriangles.assign(static_cast<size_t>(cellColumns) * cellRows, {});
    const Vector2 size = box.size();
    for (size_t index = 0; index < triangleList.size(); ++index) {
        Vector2 lower = vertexList[triangleList[index][0]], upper = lower;
        for (int corner = 1; corner < 3; ++corner) {
            const Vector2 v = vertexList[triangleList[index][corner]];
            lower = Vector2(std::min(lower.x, v.x), std::min(lower.y, v.y));
            upper = Vector2(std::max(upper.x, v.x), std::max(upper.y, v.y));
        }
        const auto columnOf = [&](float x) { return std::clamp(static_cast<int>(std::floor((x - box.minimum.x) / size.x * cellColumns)), 0, cellColumns - 1); };
        const auto rowOf = [&](float y) { return std::clamp(static_cast<int>(std::floor((y - box.minimum.y) / size.y * cellRows)), 0, cellRows - 1); };
        for (int row = rowOf(lower.y); row <= rowOf(upper.y); ++row) {
            for (int column = columnOf(lower.x); column <= columnOf(upper.x); ++column) {
                cellTriangles[static_cast<size_t>(row) * cellColumns + column].push_back(static_cast<int>(index));
            }
        }
    }
}

int PuppetMesh::cellIndexOf(Vector2 point) const {
    const Vector2 size = box.size();
    const int column = std::clamp(static_cast<int>(std::floor((point.x - box.minimum.x) / size.x * cellColumns)), 0, cellColumns - 1);
    const int row = std::clamp(static_cast<int>(std::floor((point.y - box.minimum.y) / size.y * cellRows)), 0, cellRows - 1);
    return row * cellColumns + column;
}

bool PuppetMesh::locate(Vector2 point, int& triangle, std::array<double, 3>& coordinates) const {
    if (!(point.x >= box.minimum.x && point.x <= box.maximum.x && point.y >= box.minimum.y && point.y <= box.maximum.y)) return false;
    for (int candidate : cellTriangles[cellIndexOf(point)]) {
        const std::array<int, 3>& corners = triangleList[candidate];
        const std::array<double, 3> b = barycentric(toPoint64(vertexList[corners[0]]), toPoint64(vertexList[corners[1]]),
                                                    toPoint64(vertexList[corners[2]]), toPoint64(point));
        if (b[0] >= -1e-7 && b[1] >= -1e-7 && b[2] >= -1e-7) {
            triangle = candidate;
            coordinates = b;
            return true;
        }
    }
    return false;
}

std::vector<Vector2> PuppetMesh::solved(const std::vector<Vector2>& pinTargets, int iterations) const {
    if (pinTargets.size() != pinVertices.size()) throw MathProblem("A puppet needs one target for each pin.");
    if (pinVertices.empty()) return vertexList;
    const size_t count = vertexList.size();
    std::vector<Point64> rest(count);
    for (size_t index = 0; index < count; ++index) rest[index] = toPoint64(vertexList[index]);

    // Pinned vertices and their targets (the mean when pins share a vertex).
    std::vector<char> isPinned(count, 0);
    std::vector<Point64> held(count);
    std::vector<int> pinsOnVertex(count, 0);
    for (size_t pin = 0; pin < pinVertices.size(); ++pin) {
        const size_t vertex = pinVertices[pin];
        isPinned[vertex] = 1;
        // A pin moved or snapped onto a vertex drags the vertex by the pin's own displacement.
        held[vertex].x += pinTargets[pin].x + (vertexList[vertex].x - pinRestList[pin].x);
        held[vertex].y += pinTargets[pin].y + (vertexList[vertex].y - pinRestList[pin].y);
        pinsOnVertex[vertex] += 1;
    }
    for (size_t vertex = 0; vertex < count; ++vertex) {
        if (pinsOnVertex[vertex] > 1) {
            held[vertex].x /= pinsOnVertex[vertex];
            held[vertex].y /= pinsOnVertex[vertex];
        }
    }

    // The start: the rigid motion that best takes the pins to their targets (Procrustes).
    double angle = 0;
    Point64 pinCentre, targetCentre;
    const double pinTotal = static_cast<double>(pinRestList.size());
    for (size_t pin = 0; pin < pinRestList.size(); ++pin) {
        pinCentre.x += pinRestList[pin].x / pinTotal;
        pinCentre.y += pinRestList[pin].y / pinTotal;
        targetCentre.x += pinTargets[pin].x / pinTotal;
        targetCentre.y += pinTargets[pin].y / pinTotal;
    }
    double dotSum = 0, crossSum = 0;
    for (size_t pin = 0; pin < pinRestList.size(); ++pin) {
        const double ax = pinRestList[pin].x - pinCentre.x, ay = pinRestList[pin].y - pinCentre.y;
        const double bx = pinTargets[pin].x - targetCentre.x, by = pinTargets[pin].y - targetCentre.y;
        dotSum += ax * bx + ay * by;
        crossSum += ax * by - ay * bx;
    }
    if (dotSum != 0 || crossSum != 0) angle = std::atan2(crossSum, dotSum);
    const double rigidCos = std::cos(angle), rigidSin = std::sin(angle);

    std::vector<Point64> current(count);
    for (size_t index = 0; index < count; ++index) {
        const double dx = rest[index].x - pinCentre.x, dy = rest[index].y - pinCentre.y;
        current[index] = Point64{targetCentre.x + rigidCos * dx - rigidSin * dy, targetCentre.y + rigidSin * dx + rigidCos * dy};
        if (isPinned[index]) current[index] = held[index];
    }

    std::vector<double> cosines(count, rigidCos), sines(count, rigidSin);

    // The global step: positions for the rotations in `cosines`/`sines`, by conjugate gradients on the
    // free vertices (pinned neighbours move to the right-hand side).
    std::vector<double> rightX(count), rightY(count), solutionX(count), solutionY(count), residualX(count), residualY(count);
    std::vector<double> directionX(count), directionY(count), productX(count), productY(count), diagonal(count);
    for (size_t vertex = 0; vertex < count; ++vertex) {
        for (const Neighbour& neighbour : neighbours[vertex]) diagonal[vertex] += neighbour.weight;
    }
    const auto globalStep = [&]() {
        for (size_t vertex = 0; vertex < count; ++vertex) {
            rightX[vertex] = rightY[vertex] = 0;
            if (isPinned[vertex]) continue;
            for (const Neighbour& neighbour : neighbours[vertex]) {
                const size_t other = neighbour.vertex;
                const double ex = rest[vertex].x - rest[other].x, ey = rest[vertex].y - rest[other].y;
                const double half = neighbour.weight * 0.5;
                const double c = cosines[vertex] + cosines[other], s = sines[vertex] + sines[other];
                rightX[vertex] += half * (c * ex - s * ey);
                rightY[vertex] += half * (s * ex + c * ey);
                if (isPinned[other]) {
                    rightX[vertex] += neighbour.weight * held[other].x;
                    rightY[vertex] += neighbour.weight * held[other].y;
                }
            }
        }
        const auto multiply = [&](const std::vector<double>& inX, const std::vector<double>& inY, std::vector<double>& outX, std::vector<double>& outY) {
            for (size_t vertex = 0; vertex < count; ++vertex) {
                if (isPinned[vertex]) {
                    outX[vertex] = outY[vertex] = 0;
                    continue;
                }
                double sumX = diagonal[vertex] * inX[vertex], sumY = diagonal[vertex] * inY[vertex];
                for (const Neighbour& neighbour : neighbours[vertex]) {
                    if (isPinned[neighbour.vertex]) continue;
                    sumX -= neighbour.weight * inX[neighbour.vertex];
                    sumY -= neighbour.weight * inY[neighbour.vertex];
                }
                outX[vertex] = sumX;
                outY[vertex] = sumY;
            }
        };
        // Start from the current positions (free vertices only).
        for (size_t vertex = 0; vertex < count; ++vertex) {
            solutionX[vertex] = isPinned[vertex] ? 0 : current[vertex].x;
            solutionY[vertex] = isPinned[vertex] ? 0 : current[vertex].y;
        }
        multiply(solutionX, solutionY, productX, productY);
        double residualNorm = 0, rightNorm = 0;
        for (size_t vertex = 0; vertex < count; ++vertex) {
            residualX[vertex] = isPinned[vertex] ? 0 : rightX[vertex] - productX[vertex];
            residualY[vertex] = isPinned[vertex] ? 0 : rightY[vertex] - productY[vertex];
            directionX[vertex] = residualX[vertex];
            directionY[vertex] = residualY[vertex];
            residualNorm += residualX[vertex] * residualX[vertex] + residualY[vertex] * residualY[vertex];
            rightNorm += rightX[vertex] * rightX[vertex] + rightY[vertex] * rightY[vertex];
        }
        const double stop = 1e-24 * std::max(rightNorm, 1e-30);
        for (int step = 0; step < 4 * static_cast<int>(count) + 20 && residualNorm > stop; ++step) {
            multiply(directionX, directionY, productX, productY);
            double curvature = 0;
            for (size_t vertex = 0; vertex < count; ++vertex) curvature += directionX[vertex] * productX[vertex] + directionY[vertex] * productY[vertex];
            if (!(curvature > 0)) break;
            const double stepLength = residualNorm / curvature;
            double newNorm = 0;
            for (size_t vertex = 0; vertex < count; ++vertex) {
                solutionX[vertex] += stepLength * directionX[vertex];
                solutionY[vertex] += stepLength * directionY[vertex];
                residualX[vertex] -= stepLength * productX[vertex];
                residualY[vertex] -= stepLength * productY[vertex];
                newNorm += residualX[vertex] * residualX[vertex] + residualY[vertex] * residualY[vertex];
            }
            const double ratio = newNorm / residualNorm;
            residualNorm = newNorm;
            for (size_t vertex = 0; vertex < count; ++vertex) {
                directionX[vertex] = residualX[vertex] + ratio * directionX[vertex];
                directionY[vertex] = residualY[vertex] + ratio * directionY[vertex];
            }
        }
        for (size_t vertex = 0; vertex < count; ++vertex) {
            current[vertex] = isPinned[vertex] ? held[vertex] : Point64{solutionX[vertex], solutionY[vertex]};
        }
    };

    // The local step: the best rotation at each vertex for the current positions.
    const auto localStep = [&]() {
        for (size_t vertex = 0; vertex < count; ++vertex) {
            double m00 = 0, m01 = 0, m10 = 0, m11 = 0;
            for (const Neighbour& neighbour : neighbours[vertex]) {
                const size_t other = neighbour.vertex;
                const double ex = rest[vertex].x - rest[other].x, ey = rest[vertex].y - rest[other].y;
                const double fx = current[vertex].x - current[other].x, fy = current[vertex].y - current[other].y;
                m00 += neighbour.weight * ex * fx;
                m01 += neighbour.weight * ex * fy;
                m10 += neighbour.weight * ey * fx;
                m11 += neighbour.weight * ey * fy;
            }
            const double turn = std::atan2(m01 - m10, m00 + m11);
            if (m00 + m11 != 0 || m01 - m10 != 0) {
                cosines[vertex] = std::cos(turn);
                sines[vertex] = std::sin(turn);
            }
        }
    };

    globalStep();
    for (int round = 0; round < iterations; ++round) {
        localStep();
        globalStep();
    }

    std::vector<Vector2> result(count);
    for (size_t index = 0; index < count; ++index) result[index] = Vector2(static_cast<float>(current[index].x), static_cast<float>(current[index].y));
    return result;
}

Vector2 PuppetMesh::mapped(Vector2 restPoint, const std::vector<Vector2>& solvedVertices) const {
    int triangle = 0;
    std::array<double, 3> b{};
    if (locate(restPoint, triangle, b)) {
        const std::array<int, 3>& corners = triangleList[triangle];
        double x = 0, y = 0;
        for (int corner = 0; corner < 3; ++corner) {
            x += b[corner] * solvedVertices[corners[corner]].x;
            y += b[corner] * solvedVertices[corners[corner]].y;
        }
        return Vector2(static_cast<float>(x), static_cast<float>(y));
    }
    size_t nearest = 0;
    float nearestDistance = std::numeric_limits<float>::infinity();
    for (size_t index = 0; index < vertexList.size(); ++index) {
        const float distance = (vertexList[index] - restPoint).lengthSquared();
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearest = index;
        }
    }
    return restPoint + (solvedVertices[nearest] - vertexList[nearest]);
}

Warp2D::PlaneMap PuppetMesh::planeMap(const std::vector<Vector2>& pinTargets, int iterations) const {
    auto mesh = std::make_shared<PuppetMesh>(*this);
    auto vertices = std::make_shared<std::vector<Vector2>>(solved(pinTargets, iterations));
    return [mesh, vertices](Vector2 point) { return mesh->mapped(point, *vertices); };
}

} // namespace editor
