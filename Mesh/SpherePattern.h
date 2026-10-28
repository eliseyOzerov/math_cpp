// A ball's pattern as one motif copied by the turns of a Platonic solid, with the face panels
// cut around the motifs, all drawn flat so `sphericalPanels` lays every straight seam on an
// exact great-circle arc (Engine/Geometry/SpherePattern.swift, `Docs/Sphere-Patterns.md`).
#pragma once

#include <exception>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../Curves/Polygon2D.h"
#include "../Linear/Vector3.h"
#include "PlatonicSolid.h"
#include "SphericalPanels.h"

namespace editor {

/// Where on a base solid a ball's motif sits. The words are Swift's raw values, in the order of the cases.
///
/// Use when: choosing whether a ball's stars sit on the corners or the faces of its base solid
/// Tags: pattern site, motif site, corner site, face site
/// Node: none (option of tileSphere)
enum class SphereSite {
    /// On every corner of the solid, centred on the corner's direction.
    corners,
    /// On every face of the solid, centred on the face's middle.
    faces,
};

/// The words of `SphereSite`, in the order of the cases.
const std::vector<std::string>& sphereSiteNames();
/// The site called `name`, or nothing.
std::optional<SphereSite> sphereSiteNamed(const std::string& name);

/// Which turns copy a ball's motif from site to site. The words are Swift's raw values, in the
/// order of the cases.
///
/// Use when: choosing how a ball's motif is copied when it may not fit every turn of the solid
/// Tags: copy group, symmetry choice, full symmetry, dihedral about axis
/// Node: none (option of tileSphere)
enum class SphereCopies {
    /// Every turn of the solid if the motif fits it, otherwise the turns about Z.
    fitting,
    /// Every turn of the solid.
    full,
    /// Only the turns that keep the Z axis: about a face of the cube, a corner of the tetrahedron
    /// or the icosahedron.
    aboutAxis,
};

/// The words of `SphereCopies`, in the order of the cases.
const std::vector<std::string>& sphereCopiesNames();
/// The copy rule called `name`, or nothing.
std::optional<SphereCopies> sphereCopiesNamed(const std::string& name);

/// A ball's pattern drawn flat, ready for `sphericalPanels`: one motif panel on each site,
/// the rest of each face as one or more pieces, and the seams the motifs leave on the solid's
/// edges.
///
/// Use when: reading back what `SpherePattern::tiled` laid out before meshing it
/// Tags: ball pattern, sphere pattern, tiled sphere, panel layout
/// Node: inside tileSphere
struct SpherePattern {
    /// Why a pattern cannot be laid out, in one sentence.
    struct Problem : std::exception {
        explicit Problem(std::string message) : message(std::move(message)) {}
        const char* what() const noexcept override { return message.c_str(); }

        /// The sentence.
        std::string message;
    };

    /// One end to end seam: its two ends on the unit sphere.
    struct Seam {
        Vector3 start;
        Vector3 end;

        bool operator==(const Seam&) const = default;
    };

    /// Every piece to mesh, in panel order: the motifs first, in site order, then the face pieces.
    std::vector<SphericalPanel> pieces;
    /// For each piece, the panel it belongs to: a face with a motif in its middle is cut into
    /// one piece per side, all one panel.
    std::vector<int> panelOfPiece;
    /// How many of the first pieces are motifs, one per site.
    int motifCount = 0;
    /// Each solid edge's seam outside the motifs, as its two ends on the unit sphere.
    std::vector<Seam> edgeSeams;
    /// Which turns copied the motif, and why, in words.
    std::string copyReport;

    bool operator==(const SpherePattern&) const = default;

    /// The number of panels: one per motif and one per face of the solid.
    int panelCount() const;

    /// The pattern of `motif` copied onto every corner or face of `base`, with every face cut
    /// around the motifs. Throws a `Problem` when it cannot.
    ///
    /// Source: construction (`Docs/Sphere-Patterns.md` §1–4); the rotation groups as in
    /// Coxeter, *Regular Polytopes* (1973), ch. 3.
    ///
    /// How:
    /// 1. **Group.** The solid's turns are made from two: about corner 0 by a full turn over the
    ///    faces meeting there, and about the middle of a face at corner 0 by a full turn over
    ///    its sides (`RotationGroup`).
    /// 2. **Fit.** A motif that repeats `motifSymmetry` times about its centre fits a site
    ///    whose turns in place number a divisor of that, or copies made by different turns to
    ///    the same site would land turned differently. `fitting` takes every turn when the motif
    ///    fits, otherwise only those keeping Z.
    /// 3. **Frames.** Each orbit's lowest-numbered site gets a frame on its tangent plane at
    ///    distance 1: x toward its lowest-numbered neighbouring corner (corner sites) or the
    ///    face's first corner (face sites), y = site × x. Every other site gets the frame the
    ///    group's first turn to it carries.
    /// 4. **Cuts, corner sites.** On each face, at each corner K, rays along the two edges from
    ///    K cross K's motif at H_prev and H_next (in K's tangent plane). The outline corners in
    ///    between (counterclockwise angle from the edge to the next corner, strictly inside the
    ///    face's angle) run from H_prev back to H_next. The face's outline is these runs, corner
    ///    by corner, carried onto the face's plane by central projection:
    ///
    ///        prev ●──────H_prev               H_next──────● next
    ///                         ╲   motif at K   ╱
    ///                          ╲_____●K_______╱
    ///
    /// 5. **Cuts, face sites.** The face, on its own tangent plane, is cut into one piece per
    ///    side: the side, the ray back to the motif, the motif's outline between the two rays,
    ///    and the other ray.
    ///
    /// Conventions: Z-up, unit sphere (scale by the radius when meshing). `motif` is drawn on
    /// the tangent plane at distance 1, around its site at (0, 0), and must be star-shaped
    /// about it. A motif reaching too far (into a neighbour's, or past its face's sides)
    /// throws a `Problem` naming the limit.
    ///
    /// Use when: laying out a Telstar, a Champions League star ball or any motif-on-a-solid
    ///   pattern before meshing it
    /// Tags: tile sphere, ball pattern, football panels, sphere tiling, motif on solid,
    ///   symmetric pattern, cut panels, polyhedral pattern
    /// Node: inside tileSphere
    static SpherePattern tiled(PlatonicSolid base, SphereSite site, const Polygon2D& motif, int motifSymmetry, SphereCopies copies);

    /// Points along the shorter great-circle arc from `start` to `end` on a ball of `radius`,
    /// no two neighbours further apart than `spacing`, both ends included.
    ///
    /// Source: central projection (`Docs/Sphere-Patterns.md` §1): a straight chord seen from the
    /// centre is a great-circle arc.
    ///
    /// How: the arc's angle θ = angle between the ends, cut into `n = max(1, ⌈radius·θ / spacing⌉)`
    /// pieces. Point k is the chord blend `start + (end − start)·k/n` pushed onto the ball,
    /// `radius · p / |p|`. Every chord point lies in the plane through the centre and both ends,
    /// so every sample lies exactly on their great circle (the samples are not evenly spaced in
    /// angle, which a seam does not need).
    ///
    /// Conventions: `start` and `end` length 1, not opposite (the chord would pass through the
    /// centre); `radius` and `spacing` positive.
    ///
    /// Use when: drawing a ball's seam along a solid's edge; any curve that must lie on a great circle
    /// Tags: great circle arc, geodesic arc, seam curve, arc on sphere, sample arc
    /// Node: inside tileSphere
    static std::vector<Vector3> greatCircleArc(Vector3 start, Vector3 end, float radius, float spacing);
};

} // namespace editor
