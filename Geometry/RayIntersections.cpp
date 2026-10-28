// Where a ray meets a shape (Engine/Math/Primitives/RayIntersections.swift). See
// RayIntersections.h for the rules every query follows.
#include <cmath>
#include <limits>

#include "Bounds.h"
#include "Plane.h"
#include "Ray.h"
#include "../Linear/Scalar.h"
#include "Sphere.h"
#include "Triangle.h"

namespace editor {

// How: points of the ray satisfy normal·(origin + direction·t) = distance. Solving for t:
//
//     t = (distance − normal·origin) / (normal·direction)
//
// - Parallel (|normal·direction| ≤ Tolerance::geometric): empty, even if the ray lies in the
//   plane, because every point would be a hit and none is the first.
// - t < 0 (plane is behind the origin): empty.
// - Origin on the plane: t = 0.
std::optional<float> Ray::intersectionDistance(const Plane& plane) const {
    float approachSpeed = plane.normal.dot(direction);
    if (isNearlyZero(approachSpeed)) return std::nullopt;

    float hitDistance = (plane.distance - plane.normal.dot(origin)) / approachSpeed;
    if (hitDistance >= 0) return hitDistance;
    return std::nullopt;
}

// Source: Möller & Trumbore, "Fast, Minimum Storage Ray/Triangle Intersection" (1997). Write the
// hit as both a point on the ray and a blend of the corners, with weights u (second corner) and
// v (third corner): origin + direction·t = first + u·edge1 + v·edge2. Cramer's rule gives:
//
//     pvector = direction × edge2
//     determinant = edge1·pvector
//     tvector = origin − first;   u = (tvector·pvector) / determinant
//     qvector = tvector × edge1;  v = (direction·qvector) / determinant
//     t = (edge2·qvector) / determinant
//
// The ray hits when u ≥ 0, v ≥ 0, u + v ≤ 1 and t ≥ 0. The reported weights are (1 − u − v, u, v).
//
// - Two-sided: back faces are hit too.
// - Parallel to the triangle's plane: empty. |determinant| / |edge1 × edge2| is the cosine of the
//   angle between the ray and the triangle's normal, and that is what is compared with
//   Tolerance::geometric, so a 1 mm triangle and a 1 km triangle are treated alike.
// - Hits on an edge or corner count, with no safety margin.
// - Degenerate triangles have a zero determinant and are never hit.
std::optional<TriangleHit> Ray::intersection(const Triangle& triangle) const {
    Vector3 firstEdge = triangle.second - triangle.first;
    Vector3 secondEdge = triangle.third - triangle.first;

    Vector3 directionCrossSecondEdge = direction.cross(secondEdge);
    float determinant = firstEdge.dot(directionCrossSecondEdge);
    float twiceArea = firstEdge.cross(secondEdge).length();
    // `|determinant| / twiceArea` is the cosine of the incidence angle. Comparing it, not the raw
    // determinant, keeps the test independent of the triangle's size. A zero-area triangle gives
    // 0 ≤ 0 and is never hit.
    if (!(std::fabs(determinant) > Tolerance::geometric * twiceArea)) return std::nullopt;

    Vector3 fromFirstCorner = origin - triangle.first;
    float secondWeight = fromFirstCorner.dot(directionCrossSecondEdge) / determinant;
    if (!(secondWeight >= 0 && secondWeight <= 1)) return std::nullopt;

    Vector3 fromCornerCrossFirstEdge = fromFirstCorner.cross(firstEdge);
    float thirdWeight = direction.dot(fromCornerCrossFirstEdge) / determinant;
    if (!(thirdWeight >= 0 && secondWeight + thirdWeight <= 1)) return std::nullopt;

    float hitDistance = secondEdge.dot(fromCornerCrossFirstEdge) / determinant;
    if (!(hitDistance >= 0)) return std::nullopt;

    BarycentricCoordinates barycentric{1 - secondWeight - thirdWeight, secondWeight, thirdWeight};
    return TriangleHit{hitDistance, barycentric};
}

// How: a ray point is on the sphere when |origin + direction·t − centre|² = radius². With
// offset = origin − centre and |direction| = 1: t² + 2·(offset·direction)·t + (offset·offset −
// radius²) = 0, roots t = −along ± √discriminant, along = offset·direction.
//
// The discriminant is computed geometrically (Haines et al., "Precision Improvements for
// Ray/Sphere Intersection", Ray Tracing Gems, 2019, ch. 7), because along² − (offset·offset −
// radius²) cancels when the sphere is far away compared with its radius. With `closest` the
// vector from the centre to the ray's point nearest it:
//
//     closest = offset − direction·along;   discriminant = radius² − closest·closest
//
// The roots then come from the cancellation-free form of Press et al., Numerical Recipes §5.6:
//
//     nearRoot = −(along + sign(along)·√discriminant),  other root = constant / nearRoot
//
// with constant = offset·offset − radius² (the roots multiply to it, by Vieta). The smaller root
// is where the ray enters, the larger where it leaves.
//
// - Both roots ahead: the entry distance.
// - Origin inside (entry behind, exit ahead): the exit distance.
// - Sphere entirely behind: empty.
// - Grazing (discriminant exactly 0): the single touching distance, if ahead.
std::optional<float> Ray::intersectionDistance(const Sphere& sphere) const {
    Vector3 offset = origin - sphere.centre;
    float along = offset.dot(direction);
    Vector3 closest = offset - direction * along;
    float squaredRadius = sphere.radius * sphere.radius;
    float discriminant = squaredRadius - closest.dot(closest);
    if (!(discriminant >= 0)) return std::nullopt;

    float entry;
    float exit;
    if (discriminant == 0) {
        entry = -along;
        exit = -along;
    } else {
        // Matching the sign of `along` makes this a sum of two same-signed numbers.
        float signedRoot = along >= 0 ? std::sqrt(discriminant) : -std::sqrt(discriminant);
        float nearRoot = -(along + signedRoot);
        float otherRoot = (offset.dot(offset) - squaredRadius) / nearRoot;
        entry = smallerOf(nearRoot, otherRoot);
        exit = largerOf(nearRoot, otherRoot);
    }

    if (entry >= 0) return entry;
    if (exit >= 0) return exit;
    return std::nullopt;
}

// Source: the "slab method", Kay & Kajiya (1986), in the form of Williams, Barrus, Morley &
// Shirley, "An Efficient and Robust Ray–Box Intersection Algorithm" (2005). A box is the overlap
// of three slabs. For each axis the ray is inside the slab during an interval of t:
//
//     t_near = (slabMin − origin) / direction,  t_far = (slabMax − origin) / direction
//
// (swapped if direction is negative). The box interval is [largest t_near, smallest t_far]; if
// it is empty, or ends behind the origin, the ray misses.
//
// - A direction component of 0 means the ray runs parallel to that slab: it is inside forever if
//   the origin is between the planes (faces included), and misses if not. Components so small
//   that their reciprocal overflows to infinity (below about 3e-39) are treated the same way,
//   because 0·∞ = NaN could wrongly empty the interval.
// - Origin inside the box: the exit distance.
// - Box entirely behind the origin: empty.
// - Touching a face, edge or corner counts as a hit.
std::optional<float> Ray::intersectionDistance(const Bounds& bounds) const {
    float entry = -std::numeric_limits<float>::infinity();
    float exit = std::numeric_limits<float>::infinity();

    for (int axis = 0; axis < 3; axis += 1) {
        float reciprocal = 1 / direction[axis];
        if (!std::isfinite(reciprocal)) {
            float originOnAxis = origin[axis];
            if (!(originOnAxis >= bounds.minimum[axis]
                  && originOnAxis <= bounds.maximum[axis])) {
                return std::nullopt;
            }
            continue;
        }

        float toMinimumFace = (bounds.minimum[axis] - origin[axis]) * reciprocal;
        float toMaximumFace = (bounds.maximum[axis] - origin[axis]) * reciprocal;
        entry = largerOf(entry, smallerOf(toMinimumFace, toMaximumFace));
        exit = smallerOf(exit, largerOf(toMinimumFace, toMaximumFace));
    }

    if (!(entry <= exit && exit >= 0)) return std::nullopt;
    return entry >= 0 ? entry : exit;
}

} // namespace editor
