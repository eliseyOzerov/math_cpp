// Line, circle and ellipse fits to points (see ShapeFits.h). Everything is summed in double:
// a stroke of a few hundred float points still gives scatter sums with a dozen good digits.
#include "ShapeFits.h"

#include <array>
#include <cmath>
#include <numbers>

#include "../Numerics/LinearSystem.h"

namespace editor {

namespace {

using WideMatrix3 = std::array<std::array<double, 3>, 3>;
using WideVector3 = std::array<double, 3>;

/// The mean of the points, in double.
std::array<double, 2> centroidOf(const std::vector<Vector2>& points) {
    double sumX = 0;
    double sumY = 0;
    for (Vector2 point : points) {
        sumX += point.x;
        sumY += point.y;
    }
    double count = static_cast<double>(points.size());
    return {sumX / count, sumY / count};
}

bool allFinite(const std::vector<Vector2>& points) {
    for (Vector2 point : points) {
        if (!point.isFinite()) return false;
    }
    return true;
}

/// The determinant of a 3×3 matrix, by cofactors along the first row.
double determinantOf(const WideMatrix3& matrix) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

/// The product of two 3×3 matrices.
WideMatrix3 productOf(const WideMatrix3& left, const WideMatrix3& right) {
    WideMatrix3 product{};
    for (int row = 0; row < 3; row += 1) {
        for (int column = 0; column < 3; column += 1) {
            for (int inner = 0; inner < 3; inner += 1) product[row][column] += left[row][inner] * right[inner][column];
        }
    }
    return product;
}

/// The transpose of a 3×3 matrix.
WideMatrix3 transposeOf(const WideMatrix3& matrix) {
    WideMatrix3 transposed{};
    for (int row = 0; row < 3; row += 1) {
        for (int column = 0; column < 3; column += 1) transposed[row][column] = matrix[column][row];
    }
    return transposed;
}

/// The cross product of two 3-vectors: perpendicular to both, zero when they are parallel.
WideVector3 crossOf(const WideVector3& first, const WideVector3& second) {
    return {first[1] * second[2] - first[2] * second[1], first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0]};
}

double squaredLengthOf(const WideVector3& vector) { return vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]; }

/// The real eigenvalues of a 3×3 matrix: the real roots of its characteristic polynomial
/// λ³ − tr·λ² + m·λ − det, m the sum of the principal 2×2 minors.
///
/// How: λ = t + tr/3 gives the depressed cubic t³ + p·t + q = 0. For p < 0 the three roots are
/// Viète's t_k = 2√(−p/3)·cos(⅓·acos((3q/2p)·√(−3/p)) − 2πk/3), with the acos argument held in
/// −1…1 so that a double root rounded into a complex pair still gives both (the matrices here
/// are similar to symmetric ones, so their roots are real); for p ≥ 0 there is one real root,
/// Cardano's ∛(−q/2 + √D) + ∛(−q/2 − √D) with D = q²/4 + p³/27.
///
/// Source: F. Viète, *De aequationum recognitione et emendatione* (1615); Cardano, *Ars Magna*
/// (1545); as in Press et al., *Numerical Recipes* (2007), §5.6.
std::vector<double> realEigenvaluesOf(const WideMatrix3& matrix) {
    double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    double minors = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0] + matrix[0][0] * matrix[2][2]
        - matrix[0][2] * matrix[2][0] + matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1];
    double determinant = determinantOf(matrix);
    double quadratic = -trace;
    double shift = quadratic / 3;
    double depressedLinear = minors - quadratic * quadratic / 3;
    double depressedConstant = 2 * quadratic * quadratic * quadratic / 27 - quadratic * minors / 3 - determinant;
    if (depressedLinear < 0) {
        double amplitude = 2 * std::sqrt(-depressedLinear / 3);
        double argument = (3 * depressedConstant / (2 * depressedLinear)) * std::sqrt(-3 / depressedLinear);
        double angle = std::acos(std::fmax(-1.0, std::fmin(1.0, argument))) / 3;
        std::vector<double> roots;
        for (int index = 0; index < 3; index += 1) {
            roots.push_back(amplitude * std::cos(angle - 2 * std::numbers::pi * index / 3) - shift);
        }
        return roots;
    }
    double discriminant = depressedConstant * depressedConstant / 4 + depressedLinear * depressedLinear * depressedLinear / 27;
    double root = std::sqrt(std::fmax(0.0, discriminant));
    return {std::cbrt(-depressedConstant / 2 + root) + std::cbrt(-depressedConstant / 2 - root) - shift};
}

/// A unit vector v with (matrix − eigenvalue·I)·v ≈ 0: the longest cross product of two rows of
/// that matrix (each row is perpendicular to v, so two independent rows span the plane v is
/// normal to). Nothing when every pair of rows is parallel.
std::optional<WideVector3> eigenvectorOf(const WideMatrix3& matrix, double eigenvalue) {
    WideMatrix3 shifted = matrix;
    for (int index = 0; index < 3; index += 1) shifted[index][index] -= eigenvalue;
    std::array<WideVector3, 3> candidates = {crossOf(shifted[0], shifted[1]), crossOf(shifted[0], shifted[2]),
                                            crossOf(shifted[1], shifted[2])};
    WideVector3 best = candidates[0];
    for (const WideVector3& candidate : candidates) {
        if (squaredLengthOf(candidate) > squaredLengthOf(best)) best = candidate;
    }
    double length = std::sqrt(squaredLengthOf(best));
    if (!(length > 0)) return std::nullopt;
    return WideVector3{best[0] / length, best[1] / length, best[2] / length};
}

/// The root mean square of the points' distances from the circle.
float circleResidual(const std::vector<Vector2>& points, double centreX, double centreY, double radius) {
    double sum = 0;
    for (Vector2 point : points) {
        double gap = std::hypot(point.x - centreX, point.y - centreY) - radius;
        sum += gap * gap;
    }
    return static_cast<float>(std::sqrt(sum / static_cast<double>(points.size())));
}

/// Kåsa's circle in double: centre x, centre y, radius; nothing for points on a line.
std::optional<std::array<double, 3>> algebraicCircle(const std::vector<Vector2>& points) {
    if (points.size() < 3 || !allFinite(points)) return std::nullopt;
    auto [meanX, meanY] = centroidOf(points);
    double suu = 0, suv = 0, svv = 0, suuu = 0, svvv = 0, suvv = 0, suuv = 0;
    for (Vector2 point : points) {
        double u = point.x - meanX;
        double v = point.y - meanY;
        suu += u * u;
        suv += u * v;
        svv += v * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        suuv += u * u * v;
    }
    double determinant = suu * svv - suv * suv;
    // Relative to the scatter: points on a line make the 2×2 system singular.
    if (!(determinant > 1e-12 * (suu + svv) * (suu + svv))) return std::nullopt;
    double rightFirst = (suuu + suvv) / 2;
    double rightSecond = (svvv + suuv) / 2;
    double centreU = (rightFirst * svv - rightSecond * suv) / determinant;
    double centreV = (suu * rightSecond - suv * rightFirst) / determinant;
    double count = static_cast<double>(points.size());
    double radius = std::sqrt(centreU * centreU + centreV * centreV + (suu + svv) / count);
    return std::array<double, 3>{centreU + meanX, centreV + meanY, radius};
}

/// Σ (|pᵢ − c| − r)², the geometric error a circle fit minimises.
double circleSquaredError(const std::vector<Vector2>& points, const std::array<double, 3>& circle) {
    double sum = 0;
    for (Vector2 point : points) {
        double gap = std::hypot(point.x - circle[0], point.y - circle[1]) - circle[2];
        sum += gap * gap;
    }
    return sum;
}

FittedCircle narrowedCircle(const std::vector<Vector2>& points, const std::array<double, 3>& circle) {
    Circle2 result{Vector2(static_cast<float>(circle[0]), static_cast<float>(circle[1])), static_cast<float>(circle[2])};
    return FittedCircle{result, circleResidual(points, circle[0], circle[1], circle[2])};
}

} // namespace

std::optional<FittedLine> fittedLine(const std::vector<Vector2>& points) {
    if (points.size() < 2 || !allFinite(points)) return std::nullopt;
    auto [meanX, meanY] = centroidOf(points);
    double sxx = 0, sxy = 0, syy = 0;
    for (Vector2 point : points) {
        double u = point.x - meanX;
        double v = point.y - meanY;
        sxx += u * u;
        sxy += u * v;
        syy += v * v;
    }
    if (!(sxx + syy > 0)) return std::nullopt;
    double angle = std::atan2(2 * sxy, sxx - syy) / 2;
    double directionX = std::cos(angle);
    double directionY = std::sin(angle);
    double towardsLastX = points.back().x - points.front().x;
    double towardsLastY = points.back().y - points.front().y;
    if (directionX * towardsLastX + directionY * towardsLastY < 0) {
        directionX = -directionX;
        directionY = -directionY;
    }
    double sumSquared = 0;
    for (Vector2 point : points) {
        double across = -(point.x - meanX) * directionY + (point.y - meanY) * directionX;
        sumSquared += across * across;
    }
    auto foot = [&](Vector2 point) {
        double along = (point.x - meanX) * directionX + (point.y - meanY) * directionY;
        return Vector2(static_cast<float>(meanX + directionX * along), static_cast<float>(meanY + directionY * along));
    };
    FittedLine line;
    line.centroid = Vector2(static_cast<float>(meanX), static_cast<float>(meanY));
    line.direction = Vector2(static_cast<float>(directionX), static_cast<float>(directionY));
    line.span = Segment2{foot(points.front()), foot(points.back())};
    line.rmsResidual = static_cast<float>(std::sqrt(sumSquared / static_cast<double>(points.size())));
    return line;
}

std::optional<FittedCircle> fittedCircleAlgebraic(const std::vector<Vector2>& points) {
    std::optional<std::array<double, 3>> circle = algebraicCircle(points);
    if (!circle) return std::nullopt;
    return narrowedCircle(points, *circle);
}

std::optional<FittedCircle> fittedCircle(const std::vector<Vector2>& points) {
    std::optional<std::array<double, 3>> start = algebraicCircle(points);
    if (!start) return std::nullopt;
    std::array<double, 3> circle = *start;
    double error = circleSquaredError(points, circle);
    for (int step = 0; step < 50; step += 1) {
        // The normal equations JᵀJ·δ = −Jᵀe of one Gauss–Newton step.
        LinearSystem::Matrix normal(3, LinearSystem::Row(3, 0));
        LinearSystem::Row gradient(3, 0);
        for (Vector2 point : points) {
            double offsetX = point.x - circle[0];
            double offsetY = point.y - circle[1];
            double distance = std::hypot(offsetX, offsetY);
            if (!(distance > 0)) continue;
            std::array<double, 3> slope = {-offsetX / distance, -offsetY / distance, -1};
            double residual = distance - circle[2];
            for (int row = 0; row < 3; row += 1) {
                for (int column = 0; column < 3; column += 1) normal[static_cast<size_t>(row)][static_cast<size_t>(column)] += slope[static_cast<size_t>(row)] * slope[static_cast<size_t>(column)];
                gradient[static_cast<size_t>(row)] -= slope[static_cast<size_t>(row)] * residual;
            }
        }
        std::vector<double> stepBy = LinearSystem::solved(normal, gradient);
        std::array<double, 3> moved = {circle[0] + stepBy[0], circle[1] + stepBy[1], circle[2] + stepBy[2]};
        double movedError = circleSquaredError(points, moved);
        if (!(movedError <= error) || !(moved[2] > 0)) break;
        double stepSize = std::fabs(stepBy[0]) + std::fabs(stepBy[1]) + std::fabs(stepBy[2]);
        circle = moved;
        error = movedError;
        if (stepSize <= 1e-12 * circle[2]) break;
    }
    return narrowedCircle(points, circle);
}

std::optional<FittedEllipse> fittedEllipse(const std::vector<Vector2>& points) {
    if (points.size() < 5 || !allFinite(points)) return std::nullopt;
    auto [meanX, meanY] = centroidOf(points);
    double spread = 0;
    for (Vector2 point : points) spread += (point.x - meanX) * (point.x - meanX) + (point.y - meanY) * (point.y - meanY);
    spread = std::sqrt(spread / static_cast<double>(points.size()));
    if (!(spread > 0)) return std::nullopt;
    // The scatter blocks S₁ = D₁ᵀD₁, S₂ = D₁ᵀD₂, S₃ = D₂ᵀD₂ of the normalised points.
    WideMatrix3 quadraticScatter{}, mixedScatter{}, linearScatter{};
    for (Vector2 point : points) {
        double x = (point.x - meanX) / spread;
        double y = (point.y - meanY) / spread;
        WideVector3 quadraticTerms = {x * x, x * y, y * y};
        WideVector3 linearTerms = {x, y, 1};
        for (int row = 0; row < 3; row += 1) {
            for (int column = 0; column < 3; column += 1) {
                quadraticScatter[row][column] += quadraticTerms[row] * quadraticTerms[column];
                mixedScatter[row][column] += quadraticTerms[row] * linearTerms[column];
                linearScatter[row][column] += linearTerms[row] * linearTerms[column];
            }
        }
    }
    double linearDeterminant = determinantOf(linearScatter);
    if (!(std::fabs(linearDeterminant) > 1e-12 * std::pow(static_cast<double>(points.size()), 3))) return std::nullopt;
    // T = −S₃⁻¹S₂ᵀ, one column of S₂ᵀ solved at a time.
    WideMatrix3 mixedTransposed = transposeOf(mixedScatter);
    LinearSystem::Matrix linearRows(3, LinearSystem::Row(3, 0));
    for (int row = 0; row < 3; row += 1) {
        for (int column = 0; column < 3; column += 1) linearRows[static_cast<size_t>(row)][static_cast<size_t>(column)] = linearScatter[row][column];
    }
    WideMatrix3 linearFromQuadratic{};
    for (int column = 0; column < 3; column += 1) {
        LinearSystem::Row constants = {mixedTransposed[0][column], mixedTransposed[1][column], mixedTransposed[2][column]};
        std::vector<double> solution = LinearSystem::solved(linearRows, constants);
        for (int row = 0; row < 3; row += 1) linearFromQuadratic[row][column] = -solution[static_cast<size_t>(row)];
    }
    WideMatrix3 reduced = productOf(mixedScatter, linearFromQuadratic);
    for (int row = 0; row < 3; row += 1) {
        for (int column = 0; column < 3; column += 1) reduced[row][column] += quadraticScatter[row][column];
    }
    // C₁⁻¹·M: C₁⁻¹ = [0 0 ½; 0 −1 0; ½ 0 0] swaps the first and last rows, halving them, and negates the middle.
    WideMatrix3 constrained{};
    for (int column = 0; column < 3; column += 1) {
        constrained[0][column] = reduced[2][column] / 2;
        constrained[1][column] = -reduced[1][column];
        constrained[2][column] = reduced[0][column] / 2;
    }
    std::optional<WideVector3> quadraticPart;
    double smallestEigenvalue = 0;
    for (double eigenvalue : realEigenvaluesOf(constrained)) {
        std::optional<WideVector3> candidate = eigenvectorOf(constrained, eigenvalue);
        if (!candidate) continue;
        double ellipseCondition = 4 * (*candidate)[0] * (*candidate)[2] - (*candidate)[1] * (*candidate)[1];
        if (!(ellipseCondition > 0)) continue;
        if (!quadraticPart || std::fabs(eigenvalue) < smallestEigenvalue) {
            quadraticPart = candidate;
            smallestEigenvalue = std::fabs(eigenvalue);
        }
    }
    if (!quadraticPart) return std::nullopt;
    double a = (*quadraticPart)[0], b = (*quadraticPart)[1], c = (*quadraticPart)[2];
    double d = 0, e = 0, f = 0;
    for (int column = 0; column < 3; column += 1) {
        d += linearFromQuadratic[0][column] * (*quadraticPart)[static_cast<size_t>(column)];
        e += linearFromQuadratic[1][column] * (*quadraticPart)[static_cast<size_t>(column)];
        f += linearFromQuadratic[2][column] * (*quadraticPart)[static_cast<size_t>(column)];
    }
    // The centre is where the conic's gradient (2ax + by + d, bx + 2cy + e) vanishes.
    double ellipseDenominator = 4 * a * c - b * b;
    double centreX = (b * e - 2 * c * d) / ellipseDenominator;
    double centreY = (b * d - 2 * a * e) / ellipseDenominator;
    double valueAtCentre = f + (d * centreX + e * centreY) / 2;
    double rotation = std::atan2(b, a - c) / 2;
    double cosine = std::cos(rotation);
    double sine = std::sin(rotation);
    double formAlong = a * cosine * cosine + b * sine * cosine + c * sine * sine;
    double formAcross = a * sine * sine - b * sine * cosine + c * cosine * cosine;
    double squaredAlong = -valueAtCentre / formAlong;
    double squaredAcross = -valueAtCentre / formAcross;
    if (!(squaredAlong > 0) || !(squaredAcross > 0)) return std::nullopt;
    double radiusAlong = std::sqrt(squaredAlong);
    double radiusAcross = std::sqrt(squaredAcross);
    if (radiusAlong < radiusAcross) {
        std::swap(radiusAlong, radiusAcross);
        rotation += std::numbers::pi / 2;
    }
    if (rotation > std::numbers::pi / 2) rotation -= std::numbers::pi;
    if (rotation <= -std::numbers::pi / 2) rotation += std::numbers::pi;
    Ellipse2 ellipse;
    ellipse.centre = Vector2(static_cast<float>(meanX + centreX * spread), static_cast<float>(meanY + centreY * spread));
    ellipse.majorRadius = static_cast<float>(radiusAlong * spread);
    ellipse.minorRadius = static_cast<float>(radiusAcross * spread);
    ellipse.rotation = Angle::fromRadians(static_cast<float>(rotation));
    double sumSquared = 0;
    for (Vector2 point : points) {
        double gap = ellipse.distance(point);
        sumSquared += gap * gap;
    }
    return FittedEllipse{ellipse, static_cast<float>(std::sqrt(sumSquared / static_cast<double>(points.size())))};
}

} // namespace editor
