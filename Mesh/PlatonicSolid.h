// The five Platonic solids, by name (Engine/Geometry/Mesh+PlatonicSolids.swift).
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace editor {

/// One of the five Platonic solids: the convex solids whose faces are all the same regular
/// polygon and whose corners all look alike. They are the coarsest ways to make a shape the
/// same in every direction, and the starting points a sphere is refined from.
///
/// Swift's raw values are the words `platonicSolidNames` lists, in the order of the cases.
///
/// Use when: choosing the seed shape of a volume (a die, a gem, a crystal, a stand-in ball)
/// Tags: platonic solid, regular polyhedron, tetrahedron, cube, octahedron, icosahedron,
///   dodecahedron, d4 d6 d8 d12 d20
enum class PlatonicSolid {
    /// 4 triangles, 3 meeting at each corner.
    tetrahedron,
    /// 6 squares, 3 meeting at each corner.
    cube,
    /// 8 triangles, 4 meeting at each corner.
    octahedron,
    /// 20 triangles, 5 meeting at each corner.
    icosahedron,
    /// 12 pentagons, 3 meeting at each corner.
    dodecahedron,
};

/// The name of every solid, in the order of the cases (Swift's `allCases.map(\.rawValue)`).
inline const std::vector<std::string>& platonicSolidNames() {
    static const std::vector<std::string> names = {"tetrahedron", "cube", "octahedron", "icosahedron", "dodecahedron"};
    return names;
}

/// The solid called `name`, or nothing for any other word (Swift's `PlatonicSolid(rawValue:)`).
inline std::optional<PlatonicSolid> platonicSolidNamed(const std::string& name) {
    const std::vector<std::string>& names = platonicSolidNames();
    for (size_t index = 0; index < names.size(); ++index) {
        if (names[index] == name) return static_cast<PlatonicSolid>(index);
    }
    return std::nullopt;
}

} // namespace editor
