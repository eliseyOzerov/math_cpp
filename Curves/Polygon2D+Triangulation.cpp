// Cutting a flat outline into triangles, so a shape that is not convex (a star, a panel with
// bites taken out of it) can be filled and meshed
// (Engine/Math/Paths/Polygon2D+Triangulation.swift).
#include <algorithm>
#include <optional>

#include "Polygon2D.h"

namespace editor {

/// The polygon cut into `corners.count − 2` triangles that cover it exactly, by **ear
/// clipping**.
///
/// Source: Eberly, *Triangulation by Ear Clipping* (Geometric Tools, 2002); Meisters, "Polygons
/// have ears", *American Mathematical Monthly* 82 (1975).
///
/// How: an **ear** is a corner whose triangle with its two neighbours lies inside the
/// polygon: the corner is convex (`(corner − previous) × (next − corner) > 0` for a
/// counterclockwise outline) and no other remaining corner lies in or on the triangle.
/// Every simple polygon of four or more corners has at least two ears (Meisters), so
/// cutting one off and repeating ends with one last triangle:
///
///        previous ●───────● next        the ear's triangle is cut off,
///                  ╲     ╱              and previous and next become
///                   ╲   ╱               neighbours
///                    ● ear
///
/// The search goes round the remaining corners in order and cuts the first ear it finds. A
/// corner on the straight line between its neighbours (as the middle corner of a pentagon's
/// side) is never an ear, so no triangle has zero area while another choice exists. If
/// rounding leaves no ear, the most convex corner is cut, so the loop always ends.
///
/// Conventions: the polygon must be simple (no side crossing another). A clockwise outline
/// (negative signed area) is walked backwards, so every triangle comes out counterclockwise.
/// The triangles name the original corner numbers. O(n³) at worst, which is fine for the tens
/// of corners a panel has.
///
/// Use when: filling a star or a cut ball panel with triangles before meshing it; capping a
///   flat outline that is not convex
/// Tags: ear clipping, polygon triangulation, triangulate polygon, concave polygon fill,
///   tessellate outline, earcut
/// Node: inside sphericalPanels, tileSphere
std::vector<Polygon2D::CornerTriangle> Polygon2D::triangulated() const {
    std::vector<int> remaining;
    for (size_t index = 0; index < cornerList.size(); ++index) remaining.push_back(static_cast<int>(index));
    if (signedArea() < 0) std::reverse(remaining.begin(), remaining.end());
    std::vector<CornerTriangle> triangles;
    triangles.reserve(cornerList.size() - 2);
    while (remaining.size() > 3) {
        std::optional<size_t> ear;
        for (size_t position = 0; position < remaining.size() && !ear; ++position) {
            if (isEar(position, remaining)) ear = position;
        }
        size_t earPosition = ear ? *ear : mostConvexPosition(remaining);
        size_t count = remaining.size();
        triangles.push_back(CornerTriangle{remaining[(earPosition + count - 1) % count], remaining[earPosition],
                                           remaining[(earPosition + 1) % count]});
        remaining.erase(remaining.begin() + static_cast<long>(earPosition));
    }
    triangles.push_back(CornerTriangle{remaining[0], remaining[1], remaining[2]});
    return triangles;
}

/// The area enclosed, positive when the corners run counterclockwise.
///
/// Source: the shoelace formula (Meister, 1769; Gauss).
///
/// How: half the sum of `corner_i × corner_{i+1}` over the sides, the last back to the
/// first. Each term is twice the signed area of the triangle from the origin to that side,
/// and the parts outside the polygon cancel.
///
/// Conventions: y up. For a polygon whose sides cross, the parts wound clockwise count
/// negative.
///
/// Use when: telling which way an outline runs; the area of a flat shape
/// Tags: signed area, shoelace formula, polygon area, orientation, winding direction,
///   clockwise test
/// Node: none (2D outline measure; inside sphericalPanels)
float Polygon2D::signedArea() const {
    float total = 0;
    for (const Segment2& side : sides()) total = total + side.start.cross(side.end);
    return total / 2;
}

/// How sharply the outline turns left at the remaining corner at `position`: twice the area
/// of the triangle it makes with its neighbours, positive when convex.
///
/// Use when: choosing which corner of a polygon to cut next
/// Tags: convex corner test, turn direction, cross product of neighbours
/// Node: inside sphericalPanels, tileSphere
float Polygon2D::turn(size_t position, const std::vector<int>& remaining) const {
    size_t count = remaining.size();
    Vector2 previous = cornerList[static_cast<size_t>(remaining[(position + count - 1) % count])];
    Vector2 corner = cornerList[static_cast<size_t>(remaining[position])];
    Vector2 next = cornerList[static_cast<size_t>(remaining[(position + 1) % count])];
    return (corner - previous).cross(next - corner);
}

/// Whether the remaining corner at `position` is an ear: convex, with no other remaining
/// corner in or on its triangle.
///
/// Use when: ear clipping a polygon
/// Tags: ear test, point in triangle, convex corner
/// Node: inside sphericalPanels, tileSphere
bool Polygon2D::isEar(size_t position, const std::vector<int>& remaining) const {
    if (!(turn(position, remaining) > 0)) return false;
    size_t count = remaining.size();
    int triangle[3] = {remaining[(position + count - 1) % count], remaining[position], remaining[(position + 1) % count]};
    Vector2 first = cornerList[static_cast<size_t>(triangle[0])];
    Vector2 second = cornerList[static_cast<size_t>(triangle[1])];
    Vector2 third = cornerList[static_cast<size_t>(triangle[2])];
    for (int index : remaining) {
        if (index == triangle[0] || index == triangle[1] || index == triangle[2]) continue;
        Vector2 point = cornerList[static_cast<size_t>(index)];
        // A corner at the same place as one of the triangle's (a polygon touching itself)
        // does not block it.
        if (point == first || point == second || point == third) continue;
        if ((second - first).cross(point - first) >= 0 && (third - second).cross(point - second) >= 0
            && (first - third).cross(point - third) >= 0) {
            return false;
        }
    }
    return true;
}

/// The position of the remaining corner that turns left the most, cut when rounding leaves no
/// ear. Ties keep the first, as Swift's `max(by:)` does.
///
/// Use when: ear clipping finds no ear
/// Tags: most convex corner, fallback ear
/// Node: inside sphericalPanels, tileSphere
size_t Polygon2D::mostConvexPosition(const std::vector<int>& remaining) const {
    size_t best = 0;
    for (size_t position = 1; position < remaining.size(); ++position) {
        if (turn(best, remaining) < turn(position, remaining)) best = position;
    }
    return best;
}

} // namespace editor
