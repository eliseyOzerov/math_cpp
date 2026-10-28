// Gaussian elimination and the normal equations (see LinearSystem.h).
#include "LinearSystem.h"

#include <cmath>
#include <string>
#include <utility>

#include "../Linear/MathProblem.h"

namespace editor {

namespace {

/// A pivot smaller than this in size makes its column singular.
constexpr double smallestPivot = 1e-14;

} // namespace

std::vector<double> LinearSystem::solved(const Matrix& coefficients, const Row& constants) {
    size_t size = constants.size();
    if (coefficients.size() != size) {
        throw MathProblem("A linear system needs as many equations as constants, but got "
                          + std::to_string(coefficients.size()) + " rows and " + std::to_string(size) + " constants.");
    }
    Matrix augmented;
    for (size_t row = 0; row < size; row += 1) {
        if (coefficients[row].size() != size) {
            throw MathProblem("A linear system needs a square matrix, but row " + std::to_string(row) + " has "
                              + std::to_string(coefficients[row].size()) + " entries for " + std::to_string(size) + " unknowns.");
        }
        augmented.push_back(coefficients[row]);
        augmented[row].push_back(constants[row]);
    }
    for (size_t column = 0; column < size; column += 1) {
        size_t pivotRow = column;
        for (size_t row = column + 1; row < size; row += 1) {
            if (std::fabs(augmented[row][column]) > std::fabs(augmented[pivotRow][column])) pivotRow = row;
        }
        std::swap(augmented[column], augmented[pivotRow]);
        double pivot = augmented[column][column];
        if (std::fabs(pivot) < smallestPivot) continue;
        for (size_t row = 0; row < size; row += 1) {
            if (row == column) continue;
            double multiplier = augmented[row][column] / pivot;
            if (multiplier == 0) continue;
            for (size_t entry = column; entry <= size; entry += 1) augmented[row][entry] -= multiplier * augmented[column][entry];
        }
    }
    std::vector<double> solution;
    for (size_t row = 0; row < size; row += 1) {
        double diagonal = augmented[row][row];
        solution.push_back(std::fabs(diagonal) < smallestPivot ? 0.0 : augmented[row][size] / diagonal);
    }
    return solution;
}

std::vector<double> LinearSystem::leastSquares(const Matrix& rows, const Row& constants) {
    if (rows.size() != constants.size()) {
        throw MathProblem("A least-squares system needs one constant per row, but got " + std::to_string(rows.size())
                          + " rows and " + std::to_string(constants.size()) + " constants.");
    }
    if (rows.empty()) return {};
    size_t unknowns = rows[0].size();
    for (const Row& row : rows) {
        if (row.size() != unknowns) throw MathProblem("Every row of a least-squares system must have the same number of entries.");
    }
    Matrix normal(unknowns, Row(unknowns, 0.0));
    Row right(unknowns, 0.0);
    for (size_t first = 0; first < unknowns; first += 1) {
        for (size_t second = 0; second < unknowns; second += 1) {
            double sum = 0;
            for (const Row& row : rows) sum += row[first] * row[second];
            normal[first][second] = sum;
        }
        double sum = 0;
        for (size_t index = 0; index < rows.size(); index += 1) sum += rows[index][first] * constants[index];
        right[first] = sum;
    }
    return solved(normal, right);
}

} // namespace editor
