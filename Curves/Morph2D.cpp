// See Morph2D.h. Ported from graphics_cpp's morph.cpp.
#include "Morph2D.h"

#include <algorithm>
#include <limits>
#include <optional>

#include "../Linear/MathProblem.h"
#include "Shapes2D.h"

namespace editor {

namespace {

using Cubic = CubicBezier<Vector2>;

/// A path as a list of cubic pieces, closed or not, with the questions morphing asks of it.
struct PieceList {
    std::vector<Cubic> pieces;
    bool closed = false;

    /// Where each piece starts, then where the last ends: one more node than pieces.
    std::vector<Vector2> nodes() const {
        std::vector<Vector2> result;
        for (const Cubic& piece : pieces) result.push_back(piece.start);
        result.push_back(pieces.back().end);
        return result;
    }

    /// The length up to each node, in the units of `metric`.
    std::vector<float> distances(Vector2 metric) const {
        std::vector<float> result{0};
        for (const Cubic& piece : pieces) {
            Cubic scaled(piece.start.toMetric(metric), piece.startHandle.toMetric(metric),
                         piece.endHandle.toMetric(metric), piece.end.toMetric(metric));
            result.push_back(result.back() + scaled.totalLength());
        }
        return result;
    }

    /// The same pieces run backwards.
    PieceList reversed() const {
        PieceList result{{}, closed};
        for (auto piece = pieces.rbegin(); piece != pieces.rend(); ++piece) result.pieces.push_back(piece->reversed());
        return result;
    }

    /// A closed list started at piece `first`.
    PieceList startingAt(size_t first) const {
        PieceList result{{}, closed};
        result.pieces.assign(pieces.begin() + first, pieces.end());
        result.pieces.insert(result.pieces.end(), pieces.begin(), pieces.begin() + first);
        return result;
    }

    Path2D path() const { return Shapes2D::pathOfCubics(pieces, closed); }
};

PieceList pieceListOf(const Path2D& path) {
    PieceList list;
    list.pieces = Shapes2D::cubicsOf(path);
    list.closed = path.isClosed();
    return list;
}

/// Which node of the long list each node of the short one lands on, and what that costs.
struct Alignment {
    std::vector<int> target;
    float cost = 0;
};

/// The closest match, in order, of every node of `shortNodes` to a node of `longNodes` (both ends
/// to both ends), with its cost: the sum of the distances, in the units of `metric`. Costs within
/// `tolerance` count as equal, so the earlier wins.
///
/// How: best[s][l] is the cheapest match of the short nodes 0…s with short s on long l; each
/// entry takes the best of the entries one short node back at a smaller long index.
Alignment align(const std::vector<Vector2>& shortNodes, const std::vector<Vector2>& longNodes, Vector2 metric,
                float tolerance) {
    const int shortCount = int(shortNodes.size()), longCount = int(longNodes.size());
    auto distance = [&](int shortIndex, int longIndex) {
        return shortNodes[shortIndex].toMetric(metric).distance(longNodes[longIndex].toMetric(metric));
    };
    const float infinity = std::numeric_limits<float>::infinity();
    std::vector<std::vector<float>> best(shortCount, std::vector<float>(longCount, infinity));
    std::vector<std::vector<int>> back(shortCount, std::vector<int>(longCount, -1));
    best[0][0] = distance(0, 0);
    for (int s = 1; s < shortCount; s++) {
        for (int l = s; l <= longCount - shortCount + s; l++) {
            if (s == shortCount - 1 && l != longCount - 1) continue;
            for (int q = s - 1; q < l; q++) {
                float cost = best[s - 1][q] + distance(s, l);
                if (cost < best[s][l] - tolerance) {
                    best[s][l] = cost;
                    back[s][l] = q;
                }
            }
        }
    }
    Alignment result;
    result.target.assign(shortCount, 0);
    int l = longCount - 1;
    for (int s = shortCount - 1; s >= 0; s--) {
        result.target[s] = l;
        l = l >= 0 ? back[s][l] : -1;
    }
    result.cost = best[shortCount - 1][longCount - 1];
    return result;
}

} // namespace

MorphCorrespondence Morph2D::alike(const Path2D& first, const Path2D& second, float tolerance, Vector2 metric) {
    if (first.isClosed() != second.isClosed()) throw MathProblem("A closed path morphs only into a closed one.");
    PieceList firstPieces = pieceListOf(first), secondPieces = pieceListOf(second);
    if (firstPieces.pieces.empty() || secondPieces.pieces.empty()) throw MathProblem("A path to morph needs a piece.");
    // Every way round the target could run, and every start when closed; the closest fit wins.
    std::vector<PieceList> candidates;
    for (const PieceList& direction : {secondPieces, secondPieces.reversed()}) {
        if (firstPieces.closed)
            for (size_t start = 0; start < direction.pieces.size(); start++) candidates.push_back(direction.startingAt(start));
        else candidates.push_back(direction);
    }
    std::optional<PieceList> chosen;
    Alignment fit;
    std::vector<Vector2> firstNodes = firstPieces.nodes();
    for (const PieceList& candidate : candidates) {
        std::vector<Vector2> candidateNodes = candidate.nodes();
        Alignment alignment = firstNodes.size() <= candidateNodes.size()
                                  ? align(firstNodes, candidateNodes, metric, tolerance)
                                  : align(candidateNodes, firstNodes, metric, tolerance);
        if (!chosen || alignment.cost < fit.cost - tolerance) {
            chosen = candidate;
            fit = alignment;
        }
    }
    const PieceList& target = *chosen;
    std::vector<Vector2> targetNodes = target.nodes();

    if (firstNodes.size() <= targetNodes.size()) {
        // Too few nodes: add on `first` those of the target it has none for, as far along each of
        // its pieces as they are along the target's between the matched nodes, so it keeps its shape.
        std::vector<float> targetDistances = target.distances(metric);
        PieceList grown{{}, firstPieces.closed};
        std::vector<bool> matched{true};
        for (size_t index = 0; index < firstPieces.pieces.size(); index++) {
            int firstMatch = fit.target[index], lastMatch = fit.target[index + 1];
            float span = targetDistances[lastMatch] - targetDistances[firstMatch];
            std::vector<float> shares;
            for (int added = 0; added < lastMatch - firstMatch - 1; added++) {
                float share = span != 0 ? (targetDistances[firstMatch + 1 + added] - targetDistances[firstMatch]) / span
                                        : float(added + 1) / float(lastMatch - firstMatch);
                shares.push_back(std::clamp(share, shares.empty() ? 0.0f : shares.back(), 1.0f));
            }
            for (const Cubic& piece : firstPieces.pieces[index].cutAtLengthShares(shares, metric)) grown.pieces.push_back(piece);
            for (size_t added = 0; added < shares.size(); added++) matched.push_back(false);
            matched.push_back(true);
        }
        return {grown.path(), target.path(), matched};
    }

    // Too many nodes: each of `first`'s extra nodes goes to the closer (along `first`) of the
    // matched nodes either side, so on the target it sits on that node and the piece between shrinks to nothing.
    std::vector<float> firstDistances = firstPieces.distances(metric);
    std::vector<int> goes(firstNodes.size(), 0);
    std::vector<bool> matched(firstNodes.size(), false);
    for (size_t j = 0; j < fit.target.size(); j++) {
        goes[fit.target[j]] = int(j);
        matched[fit.target[j]] = true;
    }
    for (size_t j = 0; j + 1 < fit.target.size(); j++) {
        for (int node = fit.target[j] + 1; node < fit.target[j + 1]; node++) {
            bool nearerBefore = firstDistances[node] - firstDistances[fit.target[j]] <=
                                firstDistances[fit.target[j + 1]] - firstDistances[node];
            goes[node] = nearerBefore ? int(j) : int(j + 1);
        }
    }
    PieceList shrunk{{}, firstPieces.closed};
    for (size_t index = 0; index < firstPieces.pieces.size(); index++) {
        int startNode = goes[index], endNode = goes[index + 1];
        shrunk.pieces.push_back(startNode == endNode ? Cubic(targetNodes[startNode], targetNodes[startNode],
                                                              targetNodes[startNode], targetNodes[startNode])
                                                      : target.pieces[startNode]);
    }
    return {firstPieces.path(), shrunk.path(), matched};
}

Path2D Morph2D::morph(const Path2D& first, const Path2D& second, float amount, float tolerance, Vector2 metric) {
    MorphCorrespondence correspondence = alike(first, second, tolerance, metric);
    return between(correspondence.from, correspondence.to, amount, tolerance, metric);
}

Path2D Morph2D::between(const Path2D& from, const Path2D& to, float amount, float tolerance, Vector2 metric) {
    PieceList start = pieceListOf(from), end = pieceListOf(to);
    if (start.pieces.size() != end.pieces.size() || start.pieces.empty())
        throw MathProblem("Paths to blend need the same number of pieces: make them alike first.");
    std::vector<Cubic> blended;
    for (size_t index = 0; index < start.pieces.size(); index++) {
        const Cubic& a = start.pieces[index];
        const Cubic& b = end.pieces[index];
        blended.push_back(Cubic(
            Vector2::linearBlend(a.start, b.start, amount), Vector2::linearBlend(a.startHandle, b.startHandle, amount),
            Vector2::linearBlend(a.endHandle, b.endHandle, amount), Vector2::linearBlend(a.end, b.end, amount)));
    }
    // A dropped piece's points are within `tolerance`; the next one starts where the last kept one
    // ended, so none come apart.
    std::vector<Cubic> kept;
    for (const Cubic& piece : blended) {
        Vector2 origin = piece.start.toMetric(metric);
        bool moves = false;
        for (Vector2 point : {piece.startHandle, piece.endHandle, piece.end})
            if (point.toMetric(metric).distance(origin) > tolerance) moves = true;
        if (!moves) continue;
        kept.push_back(kept.empty() ? piece : Cubic(kept.back().end, piece.startHandle, piece.endHandle, piece.end));
    }
    if (kept.empty()) kept.push_back(blended.front());
    else if (start.closed) kept.back() = Cubic(kept.back().start, kept.back().startHandle, kept.back().endHandle, kept.front().start);
    return Shapes2D::pathOfCubics(kept, start.closed);
}

} // namespace editor
