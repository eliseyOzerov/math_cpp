// Rotation groups closed from generators (Engine/Math/Algebra/RotationGroup.swift).
#include "RotationGroup.h"

#include <algorithm>
#include <set>

#include "MathProblem.h"

namespace editor {

namespace {

/// Where `turn` sends each of `points`, as the number of the point it lands nearest.
std::vector<int> permutationOf(const Matrix3& turn, const std::vector<Vector3>& points) {
    std::vector<int> moves;
    for (const Vector3& point : points) {
        Vector3 image = turn.transform(point);
        size_t nearest = 0;
        for (size_t candidate = 1; candidate < points.size(); ++candidate) {
            if ((points[candidate] - image).length() < (points[nearest] - image).length()) nearest = candidate;
        }
        if (!((points[nearest] - image).length() <= 1e-3f * std::max(point.length(), 1.0f))) {
            throw MathProblem("A generating turn does not carry the points onto themselves.");
        }
        moves.push_back(static_cast<int>(nearest));
    }
    return moves;
}

} // namespace

RotationGroup::RotationGroup(const std::vector<Matrix3>& generators, std::vector<Vector3> actingOn)
    : pointList(std::move(actingOn)) {
    turnList = {Matrix3::identity()};
    permutationList = {permutationOf(Matrix3::identity(), pointList)};
    std::set<std::vector<int>> seen = {permutationList[0]};
    size_t next = 0;
    while (next < turnList.size()) {
        for (const Matrix3& generator : generators) {
            Matrix3 product = generator * turnList[next];
            std::vector<int> moves = permutationOf(product, pointList);
            if (!seen.insert(moves).second) continue;
            turnList.push_back(product);
            permutationList.push_back(moves);
        }
        next += 1;
    }
}

RotationGroup RotationGroup::subgroupKeepingAxis(Vector3 axis) const {
    Vector3 unitAxis = axis.normalized();
    std::vector<Matrix3> keptTurns;
    std::vector<std::vector<int>> keptPermutations;
    for (size_t index = 0; index < turnList.size(); ++index) {
        Vector3 image = turnList[index].transform(unitAxis);
        if ((image - unitAxis).length() <= 1e-4f || (image + unitAxis).length() <= 1e-4f) {
            keptTurns.push_back(turnList[index]);
            keptPermutations.push_back(permutationList[index]);
        }
    }
    return RotationGroup(pointList, std::move(keptTurns), std::move(keptPermutations));
}

int RotationGroup::stabilizerOrder(Vector3 site) const {
    int count = 0;
    for (const Matrix3& turn : turnList) {
        if ((turn.transform(site) - site).length() <= 1e-4f) count += 1;
    }
    return count;
}

std::optional<Matrix3> RotationGroup::firstTurnCarrying(Vector3 start, Vector3 end) const {
    for (const Matrix3& turn : turnList) {
        if ((turn.transform(start) - end).length() <= 1e-4f) return turn;
    }
    return std::nullopt;
}

} // namespace editor
