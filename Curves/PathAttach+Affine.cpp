// The affine maps that place a path by three or more pairs of points: the exact one through
// three, the nearest in least squares for more (graphics_cpp attach.cpp `exact` and `fitted`).
#include <algorithm>
#include <cmath>

#include "../Linear/MathProblem.h"
#include "../Numerics/LinearSystem.h"
#include "PathAttach.h"

namespace editor {

namespace {

/// How close to a line three points may be, relative to the product of their spans, and still be
/// called collinear (no affine map is decided by them).
constexpr float collinearityShare = 1e-5f;

} // namespace

/// The affine map taking the three `sources` onto the three `destinations`, or nothing when the
/// sources lie on a line (then no one map is meant).
///
/// How: with `u = b − a`, `v = c − a` (sources) and `U = B − A`, `V = C − A` (destinations), a
/// point `q` has coordinates `(x, y)` in the frame `(u, v)` from `a`, found by Cramer's rule
/// `x = (r × v)/(u × v)`, `y = (u × r)/(u × v)` with `r = q − a`, and goes to `A + x·U + y·V`.
/// So the map's x axis is `(U·v.y − V·u.y)/det`, its y axis `(V·u.x − U·v.x)/det` and its
/// translation `A − (x axis · a.x + y axis · a.y)`, where `det = u × v`.
///
///     a ●──────● b          A ●─────● B
///       │                      ╲
///     c ●          ⟹          C ●
///
/// Conventions: y-up plane. Collinear means |det| < 1e-5·max(1, |u|·|v|). Throws a `MathProblem`
/// unless there are exactly three of each.
///
/// Source: Farin, *Curves and Surfaces for CAGD*, barycentric (affine) maps; graphics_cpp
/// curves/attach.cpp `exact`.
///
/// Use when: placing a path by three of its points onto three targets
/// Tags: affine from triangles, three point transform, barycentric map, shear fit
/// Node: planned (placePath)
std::optional<AffineTransform2> PathAttachment::affineThroughTriangle(
    const std::vector<Vector2>& sources, const std::vector<Vector2>& destinations) {
    if (sources.size() != 3 || destinations.size() != 3) throw MathProblem("An affine map through a triangle needs three points and three targets.");
    Vector2 u = sources[1] - sources[0];
    Vector2 v = sources[2] - sources[0];
    float determinant = u.cross(v);
    if (std::fabs(determinant) < collinearityShare * std::max(1.0f, u.length() * v.length())) return std::nullopt;
    Vector2 targetU = destinations[1] - destinations[0];
    Vector2 targetV = destinations[2] - destinations[0];
    AffineTransform2 map;
    map.xAxis = (targetU * v.y - targetV * u.y) / determinant;
    map.yAxis = (targetV * u.x - targetU * v.x) / determinant;
    map.translation = destinations[0] - (map.xAxis * sources[0].x + map.yAxis * sources[0].y);
    return map;
}

/// The affine map that comes nearest to taking each source onto its destination, in least squares,
/// or nothing when the sources lie on a line.
///
/// How: with sources and destinations both centred on their means, the translation drops out and
/// the map's four linear numbers solve two normal-equation systems (`LinearSystem::leastSquares`),
/// one per output coordinate: `X = a·x + c·y` and `Y = b·x + d·y` over all pairs. The translation
/// then puts the source mean onto the destination mean.
///
/// Conventions: y-up plane. Needs at least three pairs. Collinear sources (the centred covariance
/// determinant below 1e-5 of the product of the variances) give nothing. Throws a `MathProblem` for
/// fewer than three pairs or lists of different sizes.
///
/// Source: Golub & Van Loan, *Matrix Computations* (1996), §5.3; graphics_cpp curves/attach.cpp
/// `fitted`.
///
/// Use when: placing a path by four or more of its points, as closely as one affine map can
/// Tags: least squares affine, fit transform, best fit map, Procrustes affine
/// Node: planned (placePath)
std::optional<AffineTransform2> PathAttachment::affineLeastSquares(
    const std::vector<Vector2>& sources, const std::vector<Vector2>& destinations) {
    if (sources.size() < 3 || sources.size() != destinations.size()) throw MathProblem("A least-squares affine map needs at least three points and as many targets.");
    Vector2 sourceCentre = Vector2::mean(sources);
    Vector2 destinationCentre = Vector2::mean(destinations);
    double xx = 0;
    double xy = 0;
    double yy = 0;
    LinearSystem::Matrix rows;
    LinearSystem::Row outputX;
    LinearSystem::Row outputY;
    for (size_t index = 0; index < sources.size(); index += 1) {
        Vector2 offset = sources[index] - sourceCentre;
        Vector2 target = destinations[index] - destinationCentre;
        xx += static_cast<double>(offset.x) * offset.x;
        xy += static_cast<double>(offset.x) * offset.y;
        yy += static_cast<double>(offset.y) * offset.y;
        rows.push_back({offset.x, offset.y});
        outputX.push_back(target.x);
        outputY.push_back(target.y);
    }
    double determinant = xx * yy - xy * xy;
    if (std::fabs(determinant) < collinearityShare * std::max(1.0, xx * yy)) return std::nullopt;
    std::vector<double> alongX = LinearSystem::leastSquares(rows, outputX);
    std::vector<double> alongY = LinearSystem::leastSquares(rows, outputY);
    AffineTransform2 map;
    // Output X = a·x + c·y, Y = b·x + d·y: the x axis is the image of (1, 0), the y axis of (0, 1).
    map.xAxis = Vector2(static_cast<float>(alongX[0]), static_cast<float>(alongY[0]));
    map.yAxis = Vector2(static_cast<float>(alongX[1]), static_cast<float>(alongY[1]));
    map.translation = destinationCentre - (map.xAxis * sourceCentre.x + map.yAxis * sourceCentre.y);
    return map;
}

} // namespace editor
