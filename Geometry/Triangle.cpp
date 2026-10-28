// Triangles, the surface of every mesh (Engine/Math/Primitives/Triangle.swift).
#include "Triangle.h"

#include "../Linear/Scalar.h"
#include "Segment.h"

namespace editor {

Vector3 Triangle::scaledNormal() const { return (second - first).cross(third - first); }

bool Triangle::isDegenerate() const {
    float longestSquaredEdge = largerOf(
        largerOf((second - first).dot(second - first), (third - second).dot(third - second)),
        (first - third).dot(first - third));
    return scaledNormal().length() <= Tolerance::geometric * longestSquaredEdge;
}

Vector3 Triangle::normal() const { return isDegenerate() ? Vector3(0, 0, 0) : scaledNormal().normalized(); }

float Triangle::area() const { return scaledNormal().length() * 0.5f; }

Vector3 Triangle::centroid() const { return (first + second + third) / 3.0f; }

std::optional<BarycentricCoordinates> Triangle::barycentricCoordinates(Vector3 point) const {
    if (isDegenerate()) return std::nullopt;

    Vector3 firstEdge = second - first;
    Vector3 secondEdge = third - first;
    Vector3 offset = point - first;

    float firstEdgeSquared = firstEdge.dot(firstEdge);
    float secondEdgeSquared = secondEdge.dot(secondEdge);
    float edgesTogether = firstEdge.dot(secondEdge);
    float offsetAlongFirst = offset.dot(firstEdge);
    float offsetAlongSecond = offset.dot(secondEdge);

    float denominator = firstEdgeSquared * secondEdgeSquared - edgesTogether * edgesTogether;
    float secondWeight = (secondEdgeSquared * offsetAlongFirst - edgesTogether * offsetAlongSecond) / denominator;
    float thirdWeight = (firstEdgeSquared * offsetAlongSecond - edgesTogether * offsetAlongFirst) / denominator;
    return BarycentricCoordinates{1 - secondWeight - thirdWeight, secondWeight, thirdWeight};
}

Vector3 Triangle::closestPoint(Vector3 point) const {
    if (isDegenerate()) {
        Vector3 candidates[3] = {
            Segment{first, second}.closestPoint(point),
            Segment{second, third}.closestPoint(point),
            Segment{third, first}.closestPoint(point),
        };
        // Swift's `min(by:)` keeps the first of equally near candidates.
        Vector3 nearest = candidates[0];
        for (int index = 1; index < 3; index += 1) {
            if (point.distance(candidates[index]) < point.distance(nearest)) {
                nearest = candidates[index];
            }
        }
        return nearest;
    }

    Vector3 firstEdge = second - first;
    Vector3 secondEdge = third - first;

    // Corner region of `first`.
    Vector3 fromFirst = point - first;
    float alongFirstEdgeFromFirst = firstEdge.dot(fromFirst);
    float alongSecondEdgeFromFirst = secondEdge.dot(fromFirst);
    if (alongFirstEdgeFromFirst <= 0 && alongSecondEdgeFromFirst <= 0) return first;

    // Corner region of `second`.
    Vector3 fromSecond = point - second;
    float alongFirstEdgeFromSecond = firstEdge.dot(fromSecond);
    float alongSecondEdgeFromSecond = secondEdge.dot(fromSecond);
    if (alongFirstEdgeFromSecond >= 0 && alongSecondEdgeFromSecond <= alongFirstEdgeFromSecond) return second;

    // Edge region of first→second.
    float firstEdgeRegion = alongFirstEdgeFromFirst * alongSecondEdgeFromSecond
        - alongFirstEdgeFromSecond * alongSecondEdgeFromFirst;
    if (firstEdgeRegion <= 0 && alongFirstEdgeFromFirst >= 0 && alongFirstEdgeFromSecond <= 0) {
        float fraction = alongFirstEdgeFromFirst / (alongFirstEdgeFromFirst - alongFirstEdgeFromSecond);
        return first + firstEdge * fraction;
    }

    // Corner region of `third`.
    Vector3 fromThird = point - third;
    float alongFirstEdgeFromThird = firstEdge.dot(fromThird);
    float alongSecondEdgeFromThird = secondEdge.dot(fromThird);
    if (alongSecondEdgeFromThird >= 0 && alongFirstEdgeFromThird <= alongSecondEdgeFromThird) return third;

    // Edge region of first→third.
    float secondEdgeRegion = alongFirstEdgeFromThird * alongSecondEdgeFromFirst
        - alongFirstEdgeFromFirst * alongSecondEdgeFromThird;
    if (secondEdgeRegion <= 0 && alongSecondEdgeFromFirst >= 0 && alongSecondEdgeFromThird <= 0) {
        float fraction = alongSecondEdgeFromFirst / (alongSecondEdgeFromFirst - alongSecondEdgeFromThird);
        return first + secondEdge * fraction;
    }

    // Edge region of second→third.
    float thirdEdgeRegion = alongFirstEdgeFromSecond * alongSecondEdgeFromThird
        - alongFirstEdgeFromThird * alongSecondEdgeFromSecond;
    float secondToThirdFromSecond = alongSecondEdgeFromSecond - alongFirstEdgeFromSecond;
    float secondToThirdFromThird = alongFirstEdgeFromThird - alongSecondEdgeFromThird;
    if (thirdEdgeRegion <= 0 && secondToThirdFromSecond >= 0 && secondToThirdFromThird >= 0) {
        float fraction = secondToThirdFromSecond / (secondToThirdFromSecond + secondToThirdFromThird);
        return second + (third - second) * fraction;
    }

    // Face region: the point projects straight onto the triangle.
    float regionSum = thirdEdgeRegion + secondEdgeRegion + firstEdgeRegion;
    float secondWeight = secondEdgeRegion / regionSum;
    float thirdWeight = firstEdgeRegion / regionSum;
    return first + firstEdge * secondWeight + secondEdge * thirdWeight;
}

} // namespace editor
