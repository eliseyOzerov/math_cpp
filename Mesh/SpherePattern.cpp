// A ball's pattern as one motif copied by the turns of a Platonic solid
// (Engine/Geometry/SpherePattern.swift, `Docs/Sphere-Patterns.md`).
#include "SpherePattern.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>

#include "../Linear/Angle.h"
#include "../Linear/Quaternion.h"
#include "../Linear/RotationGroup.h"
#include "../Geometry/CentralProjection.h"
#include "../Geometry/Plane.h"

namespace editor {

const std::vector<std::string>& sphereSiteNames() {
    static const std::vector<std::string> names = {"corners", "faces"};
    return names;
}

std::optional<SphereSite> sphereSiteNamed(const std::string& name) {
    const std::vector<std::string>& names = sphereSiteNames();
    for (size_t index = 0; index < names.size(); ++index) {
        if (names[index] == name) return static_cast<SphereSite>(index);
    }
    return std::nullopt;
}

const std::vector<std::string>& sphereCopiesNames() {
    static const std::vector<std::string> names = {"fitting", "full", "aboutAxis"};
    return names;
}

std::optional<SphereCopies> sphereCopiesNamed(const std::string& name) {
    const std::vector<std::string>& names = sphereCopiesNames();
    for (size_t index = 0; index < names.size(); ++index) {
        if (names[index] == name) return static_cast<SphereCopies>(index);
    }
    return std::nullopt;
}

int SpherePattern::panelCount() const {
    int largest = -1;
    for (int panel : panelOfPiece) largest = std::max(largest, panel);
    return largest + 1;
}

namespace {


using Seam = SpherePattern::Seam;
using Problem = SpherePattern::Problem;

/// A site's frame on its tangent plane at distance 1.
struct SiteFrame {
    Vector3 origin;
    Vector3 xAxis;
    Vector3 yAxis;

    /// The frame at unit direction `site`, x along the tangent toward `toward`.
    static SiteFrame at(Vector3 site, Vector3 toward) {
        Vector3 xAxis = (toward - site * toward.dot(site)).normalized();
        return SiteFrame{site, xAxis, site.cross(xAxis)};
    }

    /// The same frame turned by `turn`.
    SiteFrame turnedBy(const Matrix3& turn) const {
        return SiteFrame{turn.transform(origin), turn.transform(xAxis), turn.transform(yAxis)};
    }

    /// Where the line from the centre through `point` meets this tangent plane, in its 2D
    /// coordinates: `(p·x, p·y) / (p·site)`.
    Vector2 flatOnTangentPlane(Vector3 point) const {
        return Vector2(point.dot(xAxis), point.dot(yAxis)) / point.dot(origin);
    }
};

/// A face's own plane, with the flat coordinates of points in space on it.
struct FacePlaneFrame {
    Vector3 origin;
    Vector3 xAxis;
    Vector3 yAxis;

    Vector2 flat(Vector3 point) const { return Vector2((point - origin).dot(xAxis), (point - origin).dot(yAxis)); }
};

/// The corners, faces, edges and turns of a unit Platonic solid, as the pattern needs them.
struct SolidSites {
    PlatonicSolid base;
    /// The corners, length 1.
    std::vector<Vector3> corners;
    /// Each face's corners, counterclockwise from outside.
    std::vector<std::vector<int>> faces;
    /// Each face's middle, length 1.
    std::vector<Vector3> faceCentres;
    /// Each corner's neighbours along an edge, sorted.
    std::vector<std::vector<int>> neighbours;
    /// Each edge once, first corner lower.
    std::vector<std::pair<int, int>> edges;
    /// Every turn of the solid.
    RotationGroup group;

    static RotationGroup groupOf(const std::vector<Vector3>& corners, const std::vector<std::vector<int>>& faces,
                                 const std::vector<Vector3>& faceCentres) {
        size_t firstFace = 0;
        int facesAtCorner = 0;
        bool foundFirst = false;
        for (size_t face = 0; face < faces.size(); ++face) {
            if (std::find(faces[face].begin(), faces[face].end(), 0) != faces[face].end()) {
                if (!foundFirst) firstFace = face;
                foundFirst = true;
                facesAtCorner += 1;
            }
        }
        std::vector<Matrix3> generators = {
            Matrix3::upperLeftOf(Quaternion::fromAxis(corners[0], Angle::fromRadians(2 * Angle::pi / static_cast<float>(facesAtCorner))).matrix()),
            Matrix3::upperLeftOf(Quaternion::fromAxis(faceCentres[firstFace],
                                                      Angle::fromRadians(2 * Angle::pi / static_cast<float>(faces[firstFace].size())))
                                     .matrix()),
        };
        return RotationGroup(generators, corners);
    }

    explicit SolidSites(PlatonicSolid base) : SolidSites(base, Mesh::platonicSolid(base, 1)) {}

    SolidSites(PlatonicSolid base, const Mesh& mesh)
        : base(base),
          corners(normalizedAll(mesh.positions)),
          faces(cornerLists(mesh)),
          faceCentres(centresOf(mesh)),
          neighbours(neighboursOf(faces, corners.size())),
          edges(edgesOf(neighbours)),
          group(groupOf(corners, faces, faceCentres)) {}

    static std::vector<Vector3> normalizedAll(const std::vector<Vector3>& positions) {
        std::vector<Vector3> result;
        for (Vector3 position : positions) result.push_back(position.normalized());
        return result;
    }

    static std::vector<std::vector<int>> cornerLists(const Mesh& mesh) {
        std::vector<std::vector<int>> result;
        for (const Mesh::Face& face : mesh.faces) result.push_back(face.corners);
        return result;
    }

    static std::vector<Vector3> centresOf(const Mesh& mesh) {
        std::vector<Vector3> result;
        for (const Mesh::Face& face : mesh.faces) {
            Vector3 sum;
            for (int corner : face.corners) sum = sum + mesh.positions[static_cast<size_t>(corner)];
            result.push_back(sum.normalized());
        }
        return result;
    }

    static std::vector<std::vector<int>> neighboursOf(const std::vector<std::vector<int>>& faces, size_t cornerCount) {
        std::vector<std::set<int>> sets(cornerCount);
        for (const std::vector<int>& face : faces) {
            for (size_t position = 0; position < face.size(); ++position) {
                size_t corner = static_cast<size_t>(face[position]);
                sets[corner].insert(face[(position + 1) % face.size()]);
                sets[corner].insert(face[(position + face.size() - 1) % face.size()]);
            }
        }
        std::vector<std::vector<int>> result;
        for (const std::set<int>& set : sets) result.emplace_back(set.begin(), set.end());
        return result;
    }

    static std::vector<std::pair<int, int>> edgesOf(const std::vector<std::vector<int>>& neighbours) {
        std::vector<std::pair<int, int>> result;
        for (size_t corner = 0; corner < neighbours.size(); ++corner) {
            for (int other : neighbours[corner]) {
                if (other > static_cast<int>(corner)) result.push_back({static_cast<int>(corner), other});
            }
        }
        return result;
    }

    /// How far a corner's motif may reach on its tangent plane: the tangent of half the angle
    /// to a neighbouring corner, so neighbouring motifs never meet.
    float cornerReachLimit() const { return (Angle::between(corners[0], corners[static_cast<size_t>(neighbours[0][0])]) / 2).tangent(); }

    /// How far a face's motif may reach on its tangent plane: the distance to the middle of a
    /// side there, so the motif stays inside the face.
    float faceReachLimit() const {
        SiteFrame frame = SiteFrame::at(faceCentres[0], corners[static_cast<size_t>(faces[0][0])]);
        Vector2 first = frame.flatOnTangentPlane(corners[static_cast<size_t>(faces[0][0])]);
        Vector2 second = frame.flatOnTangentPlane(corners[static_cast<size_t>(faces[0][1])]);
        return ((first + second) / 2).length();
    }

    /// Face `face`'s own plane, through its corners, with x toward its first corner.
    FacePlaneFrame facePlaneFrame(size_t face) const {
        Vector3 centre = faceCentres[face];
        Vector3 firstCorner = corners[static_cast<size_t>(faces[face][0])];
        Vector3 origin = centre * firstCorner.dot(centre);
        Vector3 xAxis = (firstCorner - origin).normalized();
        Vector3 yAxis = centre.cross(xAxis);
        return FacePlaneFrame{origin, xAxis, yAxis};
    }
};

/// The angle from a site to a point `reach` out on its tangent plane, in degrees, one decimal.
std::string degreesOf(float reach) {
    char text[64];
    std::snprintf(text, sizeof text, "%.1f", static_cast<double>(Angle::fromRadians(std::atan(reach)).degrees()));
    return text;
}

/// The angle turned counterclockwise from `first` to `second`, from 0 up to a full turn,
/// in radians.
float counterclockwiseAngle(Vector2 first, Vector2 second) {
    float turn = Angle::signedTurn(first, second).radians;
    return turn < 0 ? turn + 2 * Angle::pi : turn;
}

/// The corners of `outline` (star-shaped about the origin) strictly inside the
/// counterclockwise angle from direction `start` to direction `end`, from the one nearest
/// `end` to the one nearest `start`.
std::vector<Vector2> outlineCorners(const Polygon2D& outline, Vector2 start, Vector2 end) {
    float span = counterclockwiseAngle(start, end);
    const float margin = 1e-4f;
    struct Angled {
        Vector2 corner;
        float angle;
    };
    std::vector<Angled> inside;
    for (Vector2 corner : outline.corners()) {
        float angle = counterclockwiseAngle(start, corner);
        if (angle > margin && angle < span - margin) inside.push_back({corner, angle});
    }
    std::stable_sort(inside.begin(), inside.end(), [](const Angled& first, const Angled& second) { return first.angle > second.angle; });
    std::vector<Vector2> result;
    for (const Angled& entry : inside) result.push_back(entry.corner);
    return result;
}

/// The turns that copy the motif, chosen by `copies` and the fit rule, with the report.
std::pair<RotationGroup, std::string> copyGroup(const SolidSites& solid, const std::vector<Vector3>& sites, const std::string& siteName,
                                                int motifSymmetry, SphereCopies copies) {
    const RotationGroup& full = solid.group;
    RotationGroup aboutAxis = full.subgroupKeepingAxis(Vector3(0, 0, 1));
    auto fits = [&](const RotationGroup& group) {
        for (Vector3 site : sites) {
            if (motifSymmetry % group.stabilizerOrder(site) != 0) return false;
        }
        return true;
    };
    std::string baseName = platonicSolidNames()[static_cast<size_t>(solid.base)];
    int fold = full.stabilizerOrder(sites[0]);
    std::string motifName = "a " + std::to_string(motifSymmetry) + "-fold motif";
    std::string misfit = motifName + " doesn't fit the " + baseName + "'s " + std::to_string(fold) + "-fold " + siteName;
    std::string fullReport = "all " + std::to_string(full.order()) + " turns: " + motifName + " fits the " + std::to_string(fold) + "-fold " + siteName;
    switch (copies) {
    case SphereCopies::full:
        if (!fits(full)) throw Problem(misfit + "; copies \"fitting\" copies it by the turns about Z instead.");
        return {full, fullReport};
    case SphereCopies::aboutAxis:
        if (!fits(aboutAxis)) {
            throw Problem(motifName + " doesn't fit the turns about Z either; try copies \"fitting\" or another motif.");
        }
        return {aboutAxis, "about Z (" + std::to_string(aboutAxis.order()) + " turns)"};
    case SphereCopies::fitting:
        if (fits(full)) return {full, fullReport};
        if (!fits(aboutAxis)) {
            throw Problem(misfit + ", nor the turns about Z; a motif with " + std::to_string(fold) + "-fold symmetry fits.");
        }
        return {aboutAxis, "about Z (" + std::to_string(aboutAxis.order()) + " turns): " + misfit};
    }
    throw Problem("unknown copy rule");
}

/// One frame per site: the first site of each orbit gets its own, every other the one the
/// group carries to it.
std::vector<SiteFrame> siteFrames(const SolidSites& solid, SphereSite site, const std::vector<Vector3>& sites, const RotationGroup& group) {
    std::vector<SiteFrame> frames;
    for (size_t number = 0; number < sites.size(); ++number) {
        Vector3 direction = sites[number];
        std::optional<SiteFrame> carried;
        for (size_t earlier = 0; earlier < frames.size() && !carried; ++earlier) {
            if (std::optional<Matrix3> turn = group.firstTurnCarrying(sites[earlier], direction)) {
                carried = frames[earlier].turnedBy(*turn);
            }
        }
        if (carried) {
            frames.push_back(*carried);
            continue;
        }
        Vector3 toward = site == SphereSite::corners ? solid.corners[static_cast<size_t>(solid.neighbours[number].front())]
                                                      : solid.corners[static_cast<size_t>(solid.faces[number][0])];
        frames.push_back(SiteFrame::at(direction, toward));
    }
    return frames;
}

/// The face panels around motifs on the corners, and each edge's seam between two motifs.
std::pair<std::vector<SphericalPanel>, std::vector<Seam>> cornerCut(const SolidSites& solid, const std::vector<SphericalPanel>& motifs) {
    // Where the edge from corner `from` toward corner `to` leaves the motif on `from`, on the
    // motif's plane.
    auto crossing = [&](int from, int to) {
        const SphericalPanel& motif = motifs[static_cast<size_t>(from)];
        Vector3 other = solid.corners[static_cast<size_t>(to)];
        Vector2 direction(other.dot(motif.xAxis()), other.dot(motif.yAxis()));
        std::optional<Vector2> flat = motif.outline().firstCrossing(Vector2(), direction);
        if (!flat) throw Problem("the motif must surround its site, so every edge from the site crosses its outline.");
        return motif.point(*flat);
    };
    std::vector<SphericalPanel> facePanels;
    for (size_t face = 0; face < solid.faces.size(); ++face) {
        const std::vector<int>& corners = solid.faces[face];
        FacePlaneFrame frame = solid.facePlaneFrame(face);
        Plane plane(frame.origin, solid.faceCentres[face]);
        std::vector<Vector2> outline;
        for (size_t position = 0; position < corners.size(); ++position) {
            int previous = corners[(position + corners.size() - 1) % corners.size()];
            int next = corners[(position + 1) % corners.size()];
            int corner = corners[position];
            const SphericalPanel& motif = motifs[static_cast<size_t>(corner)];
            auto flatDirection = [&](int other) {
                Vector3 point = solid.corners[static_cast<size_t>(other)];
                return Vector2(point.dot(motif.xAxis()), point.dot(motif.yAxis()));
            };
            std::vector<Vector2> between = outlineCorners(motif.outline(), flatDirection(next), flatDirection(previous));
            std::vector<Vector3> run = {crossing(corner, previous)};
            for (Vector2 flat : between) run.push_back(motif.point(flat));
            run.push_back(crossing(corner, next));
            for (Vector3 point : run) {
                // The motif lies inside its corner's faces, which all face away from the
                // centre on the same side as the face's middle, so the projection exists.
                outline.push_back(frame.flat(centralProjection(plane, point).value()));
            }
        }
        facePanels.push_back(SphericalPanel::make(Polygon2D(outline), frame.origin, frame.xAxis, frame.yAxis).value());
    }
    std::vector<Seam> edgeSeams;
    for (const std::pair<int, int>& edge : solid.edges) {
        Vector3 start = crossing(edge.first, edge.second).normalized();
        Vector3 end = crossing(edge.second, edge.first).normalized();
        edgeSeams.push_back(Seam{start, end});
    }
    return {facePanels, edgeSeams};
}

/// A face with a motif in its middle, cut into one piece per side, on the motif's plane.
std::vector<SphericalPanel> faceSitePieces(const SolidSites& solid, size_t face, const SiteFrame& frame, const Polygon2D& motif) {
    std::vector<Vector2> corners;
    for (int corner : solid.faces[face]) corners.push_back(frame.flatOnTangentPlane(solid.corners[static_cast<size_t>(corner)]));
    std::vector<Vector2> crossings;
    for (Vector2 corner : corners) {
        std::optional<Vector2> crossing = motif.firstCrossing(Vector2(), corner);
        if (!crossing) throw Problem("the motif must surround its site, so every ray from the site crosses its outline.");
        crossings.push_back(*crossing);
    }
    std::vector<SphericalPanel> pieces;
    for (size_t side = 0; side < corners.size(); ++side) {
        size_t next = (side + 1) % corners.size();
        std::vector<Vector2> outline = {corners[side], corners[next], crossings[next]};
        for (Vector2 corner : outlineCorners(motif, corners[side], corners[next])) outline.push_back(corner);
        outline.push_back(crossings[side]);
        pieces.push_back(SphericalPanel::make(Polygon2D(outline), frame.origin, frame.xAxis, frame.yAxis).value());
    }
    return pieces;
}

} // namespace

SpherePattern SpherePattern::tiled(PlatonicSolid base, SphereSite site, const Polygon2D& motif, int motifSymmetry, SphereCopies copies) {
    if (!(motifSymmetry >= 1)) throw Problem("A motif's symmetry is at least 1.");
    SolidSites solid(base);
    const std::vector<Vector3>& sites = site == SphereSite::corners ? solid.corners : solid.faceCentres;
    std::string siteName = site == SphereSite::corners ? "corners" : "faces";
    std::pair<RotationGroup, std::string> chosen = copyGroup(solid, sites, siteName, motifSymmetry, copies);
    std::vector<SiteFrame> frames = siteFrames(solid, site, sites, chosen.first);
    float reach = 0;
    bool anyCorner = false;
    for (Vector2 corner : motif.corners()) {
        reach = anyCorner ? std::max(reach, corner.length()) : corner.length();
        anyCorner = true;
    }
    std::vector<SphericalPanel> motifPanels;
    for (const SiteFrame& frame : frames) {
        motifPanels.push_back(SphericalPanel::make(motif, frame.origin, frame.xAxis, frame.yAxis).value());
    }
    std::string baseName = platonicSolidNames()[static_cast<size_t>(base)];
    SpherePattern pattern;
    pattern.copyReport = chosen.second;
    pattern.motifCount = static_cast<int>(motifPanels.size());
    if (site == SphereSite::corners) {
        float limit = solid.cornerReachLimit();
        if (!(reach < limit)) {
            throw Problem("the motif reaches " + degreesOf(reach) + "° from its corner, but the " + baseName
                          + "'s corners leave room for under " + degreesOf(limit) + "°.");
        }
        std::pair<std::vector<SphericalPanel>, std::vector<Seam>> cut = cornerCut(solid, motifPanels);
        pattern.pieces = motifPanels;
        for (const SphericalPanel& panel : cut.first) pattern.pieces.push_back(panel);
        for (size_t piece = 0; piece < pattern.pieces.size(); ++piece) pattern.panelOfPiece.push_back(static_cast<int>(piece));
        pattern.edgeSeams = cut.second;
        return pattern;
    }
    float limit = solid.faceReachLimit();
    if (!(reach < limit)) {
        throw Problem("the motif reaches " + degreesOf(reach) + "° from its face's middle, but the " + baseName
                      + "'s faces leave room for under " + degreesOf(limit) + "°.");
    }
    pattern.pieces = motifPanels;
    for (size_t piece = 0; piece < motifPanels.size(); ++piece) pattern.panelOfPiece.push_back(static_cast<int>(piece));
    for (size_t face = 0; face < frames.size(); ++face) {
        for (const SphericalPanel& piece : faceSitePieces(solid, face, frames[face], motif)) {
            pattern.pieces.push_back(piece);
            pattern.panelOfPiece.push_back(static_cast<int>(motifPanels.size() + face));
        }
    }
    for (const std::pair<int, int>& edge : solid.edges) {
        pattern.edgeSeams.push_back(Seam{solid.corners[static_cast<size_t>(edge.first)], solid.corners[static_cast<size_t>(edge.second)]});
    }
    return pattern;
}

std::vector<Vector3> SpherePattern::greatCircleArc(Vector3 start, Vector3 end, float radius, float spacing) {
    long long pieces = std::max(1LL, static_cast<long long>(std::ceil(radius * Angle::between(start, end).radians / spacing)));
    std::vector<Vector3> points;
    for (long long piece = 0; piece <= pieces; ++piece) {
        points.push_back(Vector3::linearBlend(start, end, static_cast<float>(piece) / static_cast<float>(pieces)).normalized() * radius);
    }
    return points;
}

} // namespace editor
