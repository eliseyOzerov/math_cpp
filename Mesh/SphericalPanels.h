// Flat panels laid on a ball by central projection (Engine/Geometry/Mesh+SphericalPanels.swift).
// Swift's `Mesh.sphericalPanels` is the free function `sphericalPanels` here, so `Mesh.h` stays as it is.
#pragma once

#include <optional>
#include <vector>

#include "../Curves/Polygon2D.h"
#include "../Linear/Vector3.h"
#include "Mesh.h"

namespace editor {

/// One panel of a ball, drawn flat: a straight-sided outline on a plane that misses the ball's
/// centre. Laid on the sphere by central projection, each of its straight sides becomes an exact
/// great-circle arc.
///
/// Use when: describing a ball panel (a star, a hexagon, a face with bites cut out) before it is
///   meshed onto the sphere
/// Tags: spherical panel, ball panel, gnomonic panel, flat panel on sphere, panel outline
/// Node: inside sphericalPanels, tileSphere
class SphericalPanel {
public:
    /// The panel with `outline` drawn on the plane through `origin` spanned by `xAxis` and
    /// `upAxis`; nothing when the axes are parallel (or zero) or the plane passes within 1e-4 of
    /// the centre relative to `origin`'s distance (Swift's failable initializer).
    ///
    /// How: `xAxis` is normalized, and `upAxis` loses its part along `xAxis`
    /// (Gram–Schmidt) before it is normalized, so any two directions that span the plane work.
    static std::optional<SphericalPanel> make(Polygon2D outline, Vector3 origin, Vector3 xAxis, Vector3 upAxis);

    /// The most grid divisions a panel's triangles are cut into, so one panel stays within
    /// 64² small triangles per triangle of its outline.
    static constexpr int maximumDivisions = 64;

    /// The outline, in the plane's own 2D coordinates.
    const Polygon2D& outline() const { return outlinePolygon; }
    /// The plane's point at 2D coordinates (0, 0).
    Vector3 origin() const { return planeOrigin; }
    /// The plane's direction of 2D x, length 1.
    Vector3 xAxis() const { return unitX; }
    /// The plane's direction of 2D y, length 1 and at right angles to `xAxis`.
    Vector3 yAxis() const { return unitY; }

    bool operator==(const SphericalPanel&) const = default;

    /// The point of the plane at 2D coordinates `flat`: `origin + x·xAxis + y·yAxis`.
    ///
    /// Use when: lifting a panel's outline or grid point off its 2D drawing into space
    /// Tags: plane point, 2D to 3D, plane coordinates, local to world
    /// Node: inside sphericalPanels, tileSphere
    Vector3 point(Vector2 flat) const;

    /// The signed distance from the ball's centre to the plane, along `xAxis × yAxis`: positive
    /// when that normal points away from the centre.
    ///
    /// Use when: scaling a panel's grid to a spacing; telling whether its drawing faces outward
    /// Tags: plane distance, panel plane height, distance from centre
    /// Node: inside sphericalPanels
    float planeDistance() const;

    /// How many pieces each side of the panel's triangles is cut into so that no small triangle
    /// is longer than `spacing` on a ball of `radius`.
    ///
    /// How: central projection onto the sphere, `p ↦ radius · p / |p|`, stretches lengths near
    /// `p` by at most `radius / |p|`, and every point of the plane is at least the plane's
    /// distance `h` from the centre. So a flat piece of length `ℓ / n` lands no longer than
    /// `radius · ℓ / (h · n)`, and
    ///
    ///     n = ⌈ longest triangle side · radius / (h · spacing) ⌉,  at least 1
    ///
    /// keeps every piece under `spacing`. One count for the whole panel lets neighbouring
    /// triangles share the points along their common side.
    ///
    /// Conventions: `radius` and `spacing` positive. The result is not capped; compare it with
    /// `maximumDivisions`. Swift's `Int` is `long long` here; a count that is not finite is
    /// `LLONG_MAX`.
    ///
    /// Use when: choosing how finely to mesh a ball panel for a target triangle size
    /// Tags: grid resolution, subdivision count, mesh density, spacing to divisions
    /// Node: inside sphericalPanels, tileSphere
    long long gridDivisions(float radius, float spacing) const;

private:
    SphericalPanel(Polygon2D outline, Vector3 origin, Vector3 xAxis, Vector3 yAxis)
        : outlinePolygon(std::move(outline)), planeOrigin(origin), unitX(xAxis), unitY(yAxis) {}

    Polygon2D outlinePolygon;
    Vector3 planeOrigin;
    Vector3 unitX;
    Vector3 unitY;
};

/// A ball made of panels: the mesh, the panel every face belongs to, and each panel's border.
///
/// Use when: reading back the result of meshing a ball's panels
/// Tags: panelled sphere, ball mesh, panel ids, seams
/// Node: inside sphericalPanels, tileSphere
struct PanelledSphere {
    /// The ball.
    Mesh mesh;
    /// For each face of `mesh`, in order, the number of the panel it lies in.
    std::vector<int> panels;
    /// For each panel, its border on the ball, closed (the last point repeats the first), made
    /// of the mesh's own points along it.
    std::vector<std::vector<Vector3>> borders;
};

/// The panels `panels` laid on a ball of `radius`, panel i's triangles each cut into a grid
/// of `divisions[i]²` small triangles.
///
/// Source: construction (`Docs/Sphere-Patterns.md` §3); the grid as in `geodesicSphere`
/// (Kenner, 1976).
///
/// How, for each panel:
/// 1. Cut its outline into triangles by ear clipping (`Polygon2D::triangulated`).
/// 2. Cut each triangle into an even grid of `n²` small triangles: grid point `P(i, j)`
///    has weights `(n − i − j, i, j)` on the triangle's corners, so it sits at the
///    weighted mean of the flat corners.
/// 3. Lift every grid point onto the panel's plane and push it onto the sphere,
///    `radius · p / |p|`.
///
///     flat outline ─ear clip─▶ triangles ─grid─▶ small triangles ─p/|p|─▶ ball
///
/// A grid point is named by the outline corners it weighs and their weights, in corner
/// order, so two triangles sharing a side share its points. Each outline side is a side of
/// one triangle, so its grid points lie on the side's straight line and land exactly on its
/// great-circle arc; the panel's border is made of them.
///
/// Conventions: Z-up, centred on the origin; faces counterclockwise seen from outside (a
/// panel whose plane faces the centre is turned over). Each panel's points are its own: two
/// panels sharing a seam each put their own points along it, so the ball is closed only up
/// to the spacing (`Docs/Sphere-Patterns.md`, open questions). `divisions` holds one count
/// from 1 to `SphericalPanel::maximumDivisions` per panel; `radius` positive and finite
/// (Swift's preconditions are `GeometryProblem`s here).
///
/// Use when: meshing the panels of a football, a volleyball or any ball pattern drawn flat
/// Tags: spherical panels, panel meshing, gnomonic meshing, ball panels, sphere tiling,
///   great-circle seams, project polygon onto sphere
/// Node: sphericalPanels; inside tileSphere
PanelledSphere sphericalPanels(const std::vector<SphericalPanel>& panels, const std::vector<int>& divisions, float radius);

} // namespace editor
