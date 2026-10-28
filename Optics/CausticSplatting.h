// Caustics: the bright, curved lines at the bottom of a pool, where a wavy water surface
// focuses sunlight. We find them by sending many rays of light through the surface and
// counting how much arrives where (Engine/Math/Optics/CausticSplatting.swift).
#pragma once

#include <vector>

#include "../Linear/Vector2.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A surface given as a height above the xy plane: z = h(x, y). Water waves and terrain are
/// height fields. Z is up. (Swift protocol `HeightField`; here an abstract class.)
///
/// Use when: describing a water surface or terrain that can be refracted or lit
/// Tags: height field, heightmap, water surface, z equals h(x y), wave surface
class HeightField {
public:
    virtual ~HeightField() = default;

    /// The height z of the surface above the point (x, y).
    ///
    /// Use when: reading the height of a water surface at a point
    /// Tags: height, elevation, surface height, heightmap sample
    /// Node: none (renderer shading)
    virtual float height(float x, float y) const = 0;

    /// The slope of the surface at (x, y): the partial derivatives (∂h/∂x, ∂h/∂y).
    ///
    /// Use when: finding how steeply a water surface tilts; computing the normal of a
    ///   wave
    /// Tags: slope, gradient, partial derivatives, surface tilt, dh/dx
    /// Node: none (renderer shading)
    virtual Vector2 slope(float x, float y) const = 0;

    /// The unit normal of the surface at (x, y), pointing up (+z).
    ///
    /// Formula: the surface is the zero set of `F(x, y, z) = z − h(x, y)`, whose gradient is
    /// the normal direction: `(−∂h/∂x, −∂h/∂y, 1)`, then normalized. A flat surface gives
    /// (0, 0, 1). A slope of 1 (45°) gives (−1, 0, 1)/√2.
    ///
    /// Use when: getting the surface normal of water waves to refract light
    /// Tags: surface normal, height field normal, wave normal, heightmap normal, normal
    ///   from gradient
    /// Node: none (renderer shading)
    Vector3 normal(float x, float y) const;
};

/// Light power per area on a horizontal rectangle, in a grid of cells.
///
/// Use when: collecting the light that lands on the pool floor to draw a caustic pattern
/// Tags: irradiance grid, light accumulation, caustic map, photon map, light buffer
/// Node: none (value type)
class IrradianceGrid {
public:
    /// The lower-left and upper-right corners (x, y) of the rectangle covered.
    Vector2 lowerCorner;
    Vector2 upperCorner;
    /// The number of cells along x and along y.
    int columns;
    int rows;

    /// A grid of the given number of columns and rows covering the rectangle between the
    /// two corners, with no light in it.
    IrradianceGrid(Vector2 lowerCorner, Vector2 upperCorner, int columns, int rows);

    /// The irradiance of each cell, row by row from `lowerCorner.y` upwards, `columns` per
    /// row. Units: power per area (watts per m² if the lengths are metres and powers watts).
    const std::vector<float>& values() const { return cellValues; }

    /// The size of one cell along x and along y.
    ///
    /// Use when: converting between grid cells and positions on the floor
    /// Tags: cell size, grid spacing, texel size, grid resolution
    /// Node: none (renderer shading)
    Vector2 cellSize() const;

    /// The area of one cell: `cellSize.area`, width times height.
    ///
    /// Use when: turning light power into power per area
    /// Tags: cell area, texel area, grid cell area, area per cell
    /// Node: none (renderer shading)
    float cellArea() const;

    /// The irradiance in the cell `column` (from the left) and `row` (from the bottom)
    /// (Swift `subscript(column:row:)`).
    float operator()(int column, int row) const { return cellValues[static_cast<size_t>(row * columns + column)]; }

    /// The power falling on the whole rectangle: each cell's irradiance times the cell area.
    /// Summed in Double so thousands of cells do not lose digits.
    ///
    /// Use when: checking that no light power is lost or invented when light is gathered
    ///   on the grid
    /// Tags: total power, energy conservation, sum of irradiance, power check, flux total
    /// Node: none (renderer shading)
    float totalPower() const;

    /// Spreads `power` over the four cells nearest to `position`, in proportion to how close
    /// each is (**bilinear splatting**), and records it as irradiance (`power / cellArea`).
    ///
    /// Cell centres are at half-cell offsets from the corner. In cell units, with
    /// `u = (x − left)/cellWidth − ½`, `v = (y − bottom)/cellHeight − ½`, `i = ⌊u⌋`,
    /// `j = ⌊v⌋`, `fu = u − i`, `fv = v − j`:
    ///
    ///     cell (i, j)     gets (1 − fu)(1 − fv)      cell (i+1, j)   gets fu (1 − fv)
    ///     cell (i, j+1)   gets (1 − fu) fv           cell (i+1, j+1) gets fu fv
    ///
    /// The four shares add up to 1, so the power is conserved. A photon exactly at a cell
    /// centre goes entirely into that cell. Near the edge, neighbours outside the grid are
    /// replaced by the nearest edge cell, so no power is lost (the edge cells get slightly
    /// more). `position` must be inside the rectangle.
    ///
    /// Use when: adding a refracted light ray's energy to the pool floor image
    /// Tags: splat, photon splatting, bilinear splat, accumulate light, scatter write
    /// Node: none (renderer shading)
    void splat(float power, Vector2 position);

    /// Whether `position` lies inside the rectangle (edges included).
    ///
    /// Use when: skipping rays that land outside the receiving floor
    /// Tags: point in rectangle, bounds check, inside grid, contains point, in range
    /// Node: none (renderer shading)
    bool contains(Vector2 position) const;

private:
    std::vector<float> cellValues;
};

/// What a caustic simulation produced.
///
/// Use when: reading the picture of bright light patterns produced by a caustic run
/// Tags: caustic result, caustic map, light pattern, simulation output
/// Node: none (value type)
struct CausticResult {
    /// The light that reached the receiver, per area.
    IrradianceGrid irradiance;
    /// The power sent at the surface: light irradiance times the area covered by the rays.
    float emittedPower;
    /// The power that got through the surface (the sum of the Fresnel transmittances times
    /// each ray's share): `emittedPower` minus what the surface reflected.
    float transmittedPower;
    /// The part of `transmittedPower` that did not land in the grid: it missed the
    /// receiver's rectangle or did not travel down to it.
    float powerMissingReceiver;
};

/// **Photon splatting** for caustics under a refracting surface.
///
/// Source: Jensen, *Realistic Image Synthesis Using Photon Mapping* (2001), ch. 7–8, the
/// idea of sending photons from the light and estimating light per area from where they
/// land; here the density estimate is bilinear splatting into a regular grid instead of a
/// nearest-neighbour search. Light concentrates where the surface focuses it, so photon
/// density, not photon colour, makes the pattern: where neighbouring rays are bent closer
/// together than they started, more of them land per area.
///
/// Use when: simulating the bright wobbling light patterns on a pool floor under moving water
///   or beneath a glass ball
/// Tags: caustics, photon splatting, pool light, water caustics, refracted light pattern,
///   forward light tracing
struct CausticSplatting {
    /// Refracts parallel light, travelling straight down (−z), through `surface` and
    /// accumulates what lands on the horizontal plane at `receiverHeight`.
    ///
    /// How.
    /// 1. The rectangle `emissionLowerCorner…emissionUpperCorner` is divided into
    ///    `rayColumns × rayRows` equal cells. One ray per cell goes down through the cell's
    ///    centre `(x, y)`. It carries `power = lightIrradiance × cellArea`, the light that
    ///    falls on its cell.
    /// 2. It hits the surface at `(x, y, h(x, y))`, where the normal is `surface.normal`.
    ///    `RefractingInterface.continuation` bends it from `refractiveIndexAbove` into
    ///    `refractiveIndexBelow` and scales its power by the Fresnel transmittance. (Light
    ///    going down into a denser medium is never totally reflected.)
    /// 3. The refracted ray travels to the plane z = `receiverHeight` and
    ///    `IrradianceGrid.splat` deposits its power there.
    ///
    ///     ↓ ↓ ↓ ↓ ↓ ↓      parallel light, straight down
    ///     ╲╱╲╱╲╱╲╱╲╱       wavy surface z = h(x, y): rays bend toward the thicker part
    ///      ╲  │  ╱         of the water
    ///       ╲ │ ╱
    ///     ────●──────      receiver at z = receiverHeight: more rays per area where
    ///                      the rays cross, a bright line
    ///
    /// Conservation: every ray's power ends up in exactly one of three places, a lost
    /// reflection (`emittedPower − transmittedPower`), the grid, or `powerMissingReceiver`;
    /// so `emittedPower = reflected + irradiance.totalPower + powerMissingReceiver`. The
    /// test suite checks this identity.
    ///
    /// Conventions: Z-up. Rays start above every point of the surface. The receiver must be
    /// below the surface everywhere (a ray reaching the plane from below, or travelling
    /// sideways or up, counts as missing). Lengths in one unit (metres), powers in watts if
    /// `lightIrradiance` is W/m². More rays per cell than receiver cells per ray is smoother;
    /// the picture is only accurate down to the spacing between rays (about a few cells).
    ///
    /// Use when: computing the light pattern on a pool floor or table below a water
    ///   surface
    /// Tags: caustics, receiver irradiance, water light pattern, refraction simulation,
    ///   pool floor light, caustic map
    /// Node: none (renderer shading)
    static CausticResult receiverIrradiance(
        const HeightField& surface,
        float refractiveIndexAbove,
        float refractiveIndexBelow,
        float lightIrradiance,
        Vector2 emissionLowerCorner,
        Vector2 emissionUpperCorner,
        int rayColumns,
        int rayRows,
        float receiverHeight,
        const IrradianceGrid& receiver);
};

} // namespace editor
