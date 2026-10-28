// Building the half-edges of a mesh, the questions asked about its edges, and Catmull–Clark
// subdivision (Engine/Geometry/HalfEdgeMesh.swift, HalfEdgeMesh+Edges.swift,
// HalfEdgeMesh+Subdivision.swift and Mesh+Extrude.swift). Sources: Mäntylä, "An Introduction
// to Solid Modeling" (1988); Kettner, "Using generic programming for designing a data structure
// for polyhedral surfaces" (1999).

#include "HalfEdgeMesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <unordered_map>

#include "../Linear/GeometryProblem.h"

namespace editor {

namespace {

/// A side walked from `start` to `end`, packed into one number to key a hash map.
std::uint64_t directedSide(int start, int end) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(start)) << 32) | static_cast<std::uint32_t>(end);
}

} // namespace

HalfEdgeMesh::HalfEdgeMesh(const Mesh& mesh) : positions(mesh.positions) {
    std::unordered_map<std::uint64_t, int> sides;
    int pointCount = static_cast<int>(mesh.positions.size());
    for (size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex) {
        const std::vector<int>& corners = mesh.faces[faceIndex].corners;
        std::string face = std::to_string(faceIndex);
        if (corners.size() < 3) throw GeometryProblem("Face " + face + " has fewer than 3 corners.");
        std::set<int> seen;
        for (int corner : corners) {
            if (corner < 0 || corner >= pointCount) {
                throw GeometryProblem("Face " + face + " names point " + std::to_string(corner) + ", which does not exist.");
            }
            if (!seen.insert(corner).second) {
                throw GeometryProblem("Face " + face + " lists point " + std::to_string(corner) + " more than once.");
            }
        }
        int start = static_cast<int>(halfEdges.size());
        int count = static_cast<int>(corners.size());
        faceStart.push_back(start);
        for (int index = 0; index < count; ++index) {
            int sideStart = corners[index];
            int sideEnd = corners[(index + 1) % count];
            if (!sides.emplace(directedSide(sideStart, sideEnd), start + index).second) {
                throw GeometryProblem("Two faces walk the side from point " + std::to_string(sideStart) + " to point "
                                      + std::to_string(sideEnd)
                                      + " the same way: a flipped face, or more than two faces on one edge.");
            }
            halfEdges.push_back(HalfEdge{corners[index], start + (index + 1) % count,
                                         start + (index + count - 1) % count, std::nullopt,
                                         static_cast<int>(faceIndex)});
        }
    }
    outgoing.assign(positions.size(), std::nullopt);
    std::vector<int> outgoingCount(positions.size(), 0);
    for (size_t index = 0; index < halfEdges.size(); ++index) {
        int origin = halfEdges[index].origin;
        int end = halfEdges[halfEdges[index].next].origin;
        auto twin = sides.find(directedSide(end, origin));
        if (twin != sides.end()) halfEdges[index].twin = twin->second;
        outgoingCount[origin] += 1;
        // A border half-edge wins, so the walk around a border point starts at its fan's end.
        if (!outgoing[origin] || !halfEdges[index].twin) outgoing[origin] = static_cast<int>(index);
    }
    for (size_t point = 0; point < positions.size(); ++point) {
        if (outgoingCount[point] == 0) continue;
        if (static_cast<int>(outgoingHalfEdges(static_cast<int>(point)).size()) != outgoingCount[point]) {
            throw GeometryProblem("The faces around point " + std::to_string(point)
                                  + " do not form one fan: the surface pinches there.");
        }
    }
}

std::pair<int, int> HalfEdgeMesh::halfEdgesOfFace(int face) const {
    int end = face + 1 < faceCount() ? faceStart[face + 1] : static_cast<int>(halfEdges.size());
    return {faceStart[face], end};
}

std::vector<int> HalfEdgeMesh::corners(int face) const {
    auto [first, end] = halfEdgesOfFace(face);
    std::vector<int> result;
    for (int halfEdge = first; halfEdge < end; ++halfEdge) result.push_back(halfEdges[halfEdge].origin);
    return result;
}

std::vector<int> HalfEdgeMesh::outgoingHalfEdges(int point) const {
    if (!outgoing[point]) return {};
    int first = *outgoing[point];
    std::vector<int> result;
    int current = first;
    do {
        result.push_back(current);
        std::optional<int> next = halfEdges[halfEdges[current].previous].twin;
        if (!next) break;
        current = *next;
    } while (current != first && result.size() <= halfEdges.size());
    return result;
}

std::vector<int> HalfEdgeMesh::edgeHalfEdges() const {
    std::vector<int> result;
    for (size_t index = 0; index < halfEdges.size(); ++index) {
        std::optional<int> twin = halfEdges[index].twin;
        if (!twin || static_cast<int>(index) < *twin) result.push_back(static_cast<int>(index));
    }
    return result;
}

Vector3 HalfEdgeMesh::faceNormal(int face) const {
    Polygon polygon;
    for (int corner : corners(face)) polygon.corners.push_back(positions[corner]);
    return polygon.normal();
}

std::optional<Angle> HalfEdgeMesh::bendAngle(int halfEdge) const {
    std::optional<int> twin = halfEdges[halfEdge].twin;
    if (!twin) return std::nullopt;
    return Angle::between(faceNormal(halfEdges[halfEdge].face), faceNormal(halfEdges[*twin].face));
}

std::vector<MeshEdge> HalfEdgeMesh::edgesBendingMoreThan(Angle angle) const {
    std::vector<MeshEdge> result;
    for (int halfEdge : edgeHalfEdges()) {
        std::optional<Angle> bend = bendAngle(halfEdge);
        if (bend && bend->radians > angle.radians) result.push_back(edge(halfEdge));
    }
    return result;
}

std::vector<std::vector<int>> HalfEdgeMesh::selectedRuns(int point, const std::vector<bool>& isSelected) const {
    std::vector<int> fanEdges = outgoingHalfEdges(point);
    std::vector<int> fan;
    for (int halfEdge : fanEdges) fan.push_back(halfEdges[halfEdge].face);
    bool isClosed = !fanEdges.empty() && !isBorder(fanEdges.front());
    if (isClosed) {
        auto firstUnselected = std::find_if(fan.begin(), fan.end(), [&](int face) { return !isSelected[face]; });
        if (firstUnselected != fan.end()) std::rotate(fan.begin(), firstUnselected + 1, fan.end());
    }
    std::vector<std::vector<int>> runs;
    std::vector<int> current;
    for (int face : fan) {
        if (isSelected[face]) {
            current.push_back(face);
        } else if (!current.empty()) {
            runs.push_back(current);
            current.clear();
        }
    }
    if (!current.empty()) runs.push_back(current);
    return runs;
}

float HalfEdgeMesh::sharpness(int halfEdge, const EdgeCreases& creases) const {
    // A border edge is sharp for ever; a missing crease is smooth.
    if (isBorder(halfEdge)) return std::numeric_limits<float>::infinity();
    auto crease = creases.find(edge(halfEdge));
    return std::max(crease == creases.end() ? 0.0f : crease->second, 0.0f);
}

std::pair<Mesh, EdgeCreases> HalfEdgeMesh::subdividedOnce(const EdgeCreases& creases) const {
    int pointCount = static_cast<int>(positions.size());
    std::vector<int> edges = edgeHalfEdges();
    std::vector<int> edgeNumber(halfEdges.size(), 0);
    for (size_t number = 0; number < edges.size(); ++number) {
        int halfEdge = edges[number];
        edgeNumber[halfEdge] = static_cast<int>(number);
        if (halfEdges[halfEdge].twin) edgeNumber[*halfEdges[halfEdge].twin] = static_cast<int>(number);
    }
    int edgePointStart = pointCount;
    int facePointStart = pointCount + static_cast<int>(edges.size());

    std::vector<Vector3> facePoints;
    facePoints.reserve(faceCount());
    for (int face = 0; face < faceCount(); ++face) {
        Polygon polygon;
        for (int corner : corners(face)) polygon.corners.push_back(positions[corner]);
        facePoints.push_back(polygon.centroid());
    }

    std::vector<Vector3> edgePoints;
    edgePoints.reserve(edges.size());
    for (int halfEdge : edges) {
        Vector3 start = positions[halfEdges[halfEdge].origin];
        Vector3 end = positions[destination(halfEdge)];
        Vector3 midpoint = Vector3::linearBlend(start, end, 0.5f);
        float edgeSharpness = sharpness(halfEdge, creases);
        std::optional<int> twin = halfEdges[halfEdge].twin;
        if (!(edgeSharpness < 1) || !twin) {
            edgePoints.push_back(midpoint);
            continue;
        }
        Vector3 smooth = (start + end + facePoints[halfEdges[halfEdge].face] + facePoints[halfEdges[*twin].face]) / 4;
        edgePoints.push_back(Vector3::linearBlend(smooth, midpoint, edgeSharpness));
    }

    Mesh result;
    result.positions.reserve(positions.size() + edgePoints.size() + facePoints.size());
    for (int point = 0; point < pointCount; ++point) {
        result.positions.push_back(movedOldPoint(point, facePoints, creases));
    }
    result.positions.insert(result.positions.end(), edgePoints.begin(), edgePoints.end());
    result.positions.insert(result.positions.end(), facePoints.begin(), facePoints.end());

    result.faces.reserve(halfEdges.size());
    for (int face = 0; face < faceCount(); ++face) {
        auto [first, end] = halfEdgesOfFace(face);
        for (int halfEdge = first; halfEdge < end; ++halfEdge) {
            result.faces.push_back(Mesh::Face{{
                halfEdges[halfEdge].origin,
                edgePointStart + edgeNumber[halfEdge],
                facePointStart + face,
                edgePointStart + edgeNumber[halfEdges[halfEdge].previous],
            }});
        }
    }

    EdgeCreases childCreases;
    for (size_t number = 0; number < edges.size(); ++number) {
        int halfEdge = edges[number];
        if (isBorder(halfEdge)) continue;
        float remaining = sharpness(halfEdge, creases) - 1;
        if (!(remaining > 0)) continue;
        int edgePoint = edgePointStart + static_cast<int>(number);
        childCreases[MeshEdge(halfEdges[halfEdge].origin, edgePoint)] = remaining;
        childCreases[MeshEdge(edgePoint, destination(halfEdge))] = remaining;
    }
    return {std::move(result), std::move(childCreases)};
}

Vector3 HalfEdgeMesh::movedOldPoint(int point, const std::vector<Vector3>& facePoints,
                                    const EdgeCreases& creases) const {
    std::vector<int> around = outgoingHalfEdges(point);
    Vector3 old = positions[point];
    if (around.empty()) return old;

    // The edges at the point: every outgoing half-edge, plus, at a border, the incoming border
    // half-edge that closes the fan (its twin does not exist, so no outgoing half-edge names it).
    std::vector<int> farEnds;
    std::vector<float> edgeSharpness;
    for (int halfEdge : around) {
        farEnds.push_back(destination(halfEdge));
        edgeSharpness.push_back(sharpness(halfEdge, creases));
    }
    int closing = halfEdges[around.back()].previous;
    if (isBorder(closing)) {
        farEnds.push_back(halfEdges[closing].origin);
        edgeSharpness.push_back(std::numeric_limits<float>::infinity());
    }
    std::vector<std::pair<int, float>> sharpEnds;
    for (size_t index = 0; index < farEnds.size(); ++index) {
        if (edgeSharpness[index] > 0) sharpEnds.push_back({farEnds[index], edgeSharpness[index]});
    }
    // A one-face corner of an open sheet: its only outgoing half-edge is a border.
    bool isBorderCorner = around.size() == 1 && isBorder(around[0]);
    float valence = static_cast<float>(farEnds.size());

    Vector3 faceSum;
    for (int halfEdge : around) faceSum = faceSum + facePoints[halfEdges[halfEdge].face];
    Vector3 faceMean = faceSum / static_cast<float>(around.size());
    Vector3 midpointSum;
    for (int end : farEnds) midpointSum = midpointSum + Vector3::linearBlend(old, positions[end], 0.5f);
    Vector3 midpointMean = midpointSum / valence;
    Vector3 smooth = (faceMean + 2 * midpointMean + (valence - 3) * old) / valence;

    Vector3 sharp;
    if (isBorderCorner || sharpEnds.size() > 2) {
        sharp = old;
    } else if (sharpEnds.size() == 2) {
        sharp = (positions[sharpEnds[0].first] + 6 * old + positions[sharpEnds[1].first]) / 8;
    } else {
        return smooth;
    }
    float sharpnessSum = 0;
    for (const auto& end : sharpEnds) sharpnessSum += end.second;
    float meanSharpness = sharpnessSum / static_cast<float>(sharpEnds.size());
    return meanSharpness >= 1 ? sharp : Vector3::linearBlend(smooth, sharp, meanSharpness);
}

} // namespace editor
