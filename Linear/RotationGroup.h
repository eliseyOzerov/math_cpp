// The turns that carry a solid onto itself, found from a few of them. A ball's pattern is one
// motif copied by these turns, so the copies agree only if the group is exact and complete
// (Engine/Math/Algebra/RotationGroup.swift).
#pragma once

#include <optional>
#include <vector>

#include "Vector3.h"
#include "Matrix3.h"

namespace editor {

/// A finite **group of rotations**: every turn that a few generating turns make when combined
/// in any order, each kept with the permutation it makes of a set of points (the corners of a
/// base solid).
///
/// Use when: copying a motif to every site of a ball pattern; asking which turns keep a corner
///   or a face of a solid in place
/// Tags: rotation group, symmetry group, point group, tetrahedral group, octahedral group,
///   icosahedral group, dihedral group, group closure, orbit, stabilizer
/// Node: inside tileSphere
class RotationGroup {
public:
    /// The group the turns `generators` make, acting on `points`.
    ///
    /// Source: Coxeter, *Regular Polytopes* (1973), ch. 3; Armstrong, *Groups and Symmetry*
    /// (1988), for groups given by generators.
    ///
    /// How: a breadth-first search. Start from the turn that changes nothing; for each turn
    /// found, apply each generator after it, and keep the product if it is new. Two turns are
    /// the same when they move the points the same way, so each is named by its permutation of
    /// the points (whole numbers, compared exactly) rather than by its rounded matrix. A
    /// rotation that fixes three points not in one plane through the origin is the identity, so
    /// for a solid's corners the permutation names the turn:
    ///
    ///     found:  e ──a──▶ a ──a──▶ a² ...      each new turn tried with every generator
    ///              ╲                            until no product is new
    ///               b──▶ b ──a──▶ ab ...
    ///
    /// The search ends because the turns permute a finite set of points: there are at most
    /// `points.count!` of them, and for the Platonic solids 12, 24 or 60.
    ///
    /// Conventions: the generators must carry the set of points onto itself (each point to
    /// within 1e-3 of another, relative to its length); a point's image is matched to the
    /// nearest point (the first of equals). The points must span 3D space so a permutation
    /// names one turn. Swift's precondition is a `MathProblem` here.
    ///
    /// Use when: building the symmetries of a base solid from its corner turn and face turn
    /// Tags: group closure, generate group, breadth-first search, permutation group,
    ///   von Dyck presentation, symmetry from generators
    /// Node: inside tileSphere
    RotationGroup(const std::vector<Matrix3>& generators, std::vector<Vector3> actingOn);

    /// The points the turns permute, as given.
    const std::vector<Vector3>& points() const { return pointList; }

    /// Every turn, the turn that changes nothing first, then in the order the search reached them.
    const std::vector<Matrix3>& turns() const { return turnList; }

    /// For each turn, where it sends each point: `permutations()[turn][point]` is the number of
    /// the point it lands on.
    const std::vector<std::vector<int>>& permutations() const { return permutationList; }

    /// The number of turns: the group's **order**.
    ///
    /// Use when: reporting how many copies a pattern makes
    /// Tags: group order, number of symmetries, size of group
    /// Node: inside tileSphere
    int order() const { return static_cast<int>(turnList.size()); }

    /// The turns that carry the line along `axis` onto itself: those sending `axis` to `axis`
    /// or to `−axis`. They form a group of their own, a **subgroup**.
    ///
    /// How: a turn keeps the line when `turn·axis` is within 1e-4 of `±axis` (both unit). If
    /// two turns keep the line, so does their product, and so does each one's inverse, which is
    /// what makes it a group. For a solid with a corner, face or edge on Z it is the dihedral
    /// group about Z (D4 for the cube about a face, D5 for the icosahedron about a corner), or a
    /// cyclic one where no half turn swaps the ends (C3 for the tetrahedron about a corner).
    ///
    /// Conventions: `axis` must not be zero. The turns keep their order.
    ///
    /// Use when: copying a motif about one axis when it cannot follow every turn of the solid
    /// Tags: subgroup, axis stabilizer, dihedral group, cyclic group, turns about an axis,
    ///   setwise stabilizer
    /// Node: inside tileSphere
    RotationGroup subgroupKeepingAxis(Vector3 axis) const;

    /// How many turns keep the unit direction `site` in place: the order of its
    /// **stabilizer**.
    ///
    /// How: count the turns with `turn·site` within 1e-4 of `site`. By the orbit–stabilizer
    /// theorem (Armstrong 1988), the group's order is this count times the number of sites
    /// `site` can be carried to: for the cube's 24 turns, a corner has 3 (8 corners), a face
    /// centre 4 (6 faces), an edge middle 2 (12 edges).
    ///
    /// Conventions: `site` has length 1.
    ///
    /// Use when: the fit rule for a ball motif: an n-fold site needs a motif with n-fold symmetry
    /// Tags: stabilizer, isotropy group, site symmetry, n-fold axis, orbit-stabilizer theorem
    /// Node: inside tileSphere
    int stabilizerOrder(Vector3 site) const;

    /// The first turn (by number) that carries the unit direction `start` to `end`, or nothing
    /// if none does, because the two lie in different orbits.
    ///
    /// Use when: carrying a motif's frame from its orbit's first site to another site
    /// Tags: turn between sites, transporting rotation, group element mapping a to b, orbit
    /// Node: inside tileSphere
    std::optional<Matrix3> firstTurnCarrying(Vector3 start, Vector3 end) const;

private:
    RotationGroup(std::vector<Vector3> points, std::vector<Matrix3> turns, std::vector<std::vector<int>> permutations)
        : pointList(std::move(points)), turnList(std::move(turns)), permutationList(std::move(permutations)) {}

    std::vector<Vector3> pointList;
    std::vector<Matrix3> turnList;
    std::vector<std::vector<int>> permutationList;
};

} // namespace editor
