// One flat-ish face as a loop of points, and what can be measured on it
// (Engine/Geometry/Polygon.swift).
#pragma once

#include <vector>

#include "../Linear/Angle.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A closed loop of corners in 3D, walked in order; the last corner joins back to the first.
///
/// Conventions: corners run counter-clockwise seen from the side the face looks toward
/// (right-handed), so the normal points out of a closed mesh.
///
/// Use when: measuring one face of a mesh: its normal, its centre, its size
/// Tags: polygon, face loop, ngon, polyline closed
/// Node: none (geometry the mesh nodes measure inside)
struct Polygon {
    std::vector<Vector3> corners;

    /// A vector along the face's normal, twice as long as its area.
    ///
    /// How: Newell's method (Newell, 1972; see Sutherland, Sproull and Schumacker, "A
    /// Characterization of Ten Hidden-Surface Algorithms", 1974). For each side from c to n,
    /// both taken relative to the first corner to keep the numbers small:
    ///
    ///     sum.x += (c.y − n.y)(c.z + n.z)
    ///     sum.y += (c.z − n.z)(c.x + n.x)
    ///     sum.z += (c.x − n.x)(c.y + n.y)
    ///
    /// It works for any polygon, even one that is not quite flat. Fewer than 3 corners gives
    /// the zero vector.
    ///
    /// Use when: the normal of a quad that may be slightly bent; summing the normals of
    /// several faces weighted by their areas
    /// Tags: Newell normal, area vector, polygon normal, face normal
    /// Node: inside extrude
    Vector3 newellVector() const {
        if (corners.size() < 3) return Vector3();
        Vector3 origin = corners[0];
        Vector3 sum;
        for (size_t index = 0; index < corners.size(); ++index) {
            Vector3 current = corners[index] - origin;
            Vector3 next = corners[(index + 1) % corners.size()] - origin;
            sum.x += (current.y - next.y) * (current.z + next.z);
            sum.y += (current.z - next.z) * (current.x + next.x);
            sum.z += (current.x - next.x) * (current.y + next.y);
        }
        return sum;
    }

    /// The average of the corners.
    ///
    /// How: first + Σ(corner − first) / count, offsets from the first corner keep the sum small.
    /// No corners gives the origin.
    ///
    /// Use when: the face point of Catmull–Clark subdivision; a label at the middle of a face
    /// Tags: centroid, average, mean point, face centre, barycentre
    /// Node: inside subdivide
    Vector3 centroid() const {
        if (corners.empty()) return Vector3();
        Vector3 origin = corners[0];
        Vector3 offsets;
        for (Vector3 corner : corners) offsets = offsets + (corner - origin);
        return origin + offsets / static_cast<float>(corners.size());
    }

    /// The length of the loop, in metres: the sum of its sides, the closing side included.
    ///
    /// Use when: a size to compare an area against, to tell a thin sliver from a real face
    /// Tags: perimeter, circumference, outline length
    /// Node: none (used inside normal)
    float perimeter() const {
        float total = 0;
        for (size_t index = 0; index < corners.size(); ++index) {
            total += (corners[(index + 1) % corners.size()] - corners[index]).length();
        }
        return total;
    }

    /// The unit normal, or the zero vector if the face has no area to speak of.
    ///
    /// How: the Newell vector, normalised. A face whose Newell vector is no longer than a
    /// millionth of its perimeter squared (a sliver, or corners on one line) has no reliable
    /// direction and gives zero.
    ///
    /// Use when: the direction to inset a face's corners; the bend between two faces
    /// Tags: face normal, unit normal, surface direction, polygon normal
    /// Node: inside inset, subdivide
    Vector3 normal() const {
        Vector3 vector = newellVector();
        float size = perimeter();
        if (!(vector.length() > Tolerance::geometric * size * size)) return Vector3();
        return vector.normalized();
    }

    /// The area: half the length of the Newell vector.
    ///
    /// Use when: measuring a face; weighting faces by size; summing a mesh's surface area
    /// Tags: polygon area, face area, surface area, shoelace, Newell area
    /// Node: measureFaces
    float area() const { return newellVector().length() * 0.5f; }

    /// The angle between the two edges that meet at the corner `index`.
    ///
    /// How: `Angle::between(previous - corner, next - corner)`, from 0 to 180 degrees. For a
    /// convex polygon this is the interior angle; a reflex corner gives the smaller angle between
    /// the edges.
    ///
    /// Conventions: the corners of a triangle give angles that add up to 180 degrees. `index`
    /// must be a corner of the polygon.
    ///
    /// Use when: weighting a face's contribution to a vertex normal; measuring how sharp a corner is
    /// Tags: corner angle, interior angle, vertex angle, angle weight, polygon angle
    /// Node: measureCorners
    Angle cornerAngle(int index) const {
        size_t count = corners.size();
        Vector3 corner = corners[index];
        Vector3 previous = corners[(index + count - 1) % count];
        Vector3 next = corners[(index + 1) % count];
        return Angle::between(previous - corner, next - corner);
    }
};

} // namespace editor
