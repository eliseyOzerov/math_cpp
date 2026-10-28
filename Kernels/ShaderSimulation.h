// The arithmetic of the shader library's simulations, ported from graphics_metal's Simulation.metal:
// the Gray–Scott reaction-diffusion step, Conway's Game of Life, a one-pass fluid and the
// stateless particle field, plus the three colour views of the first three.
//
// What stays in Metal and what is here. A simulation step reads the previous frame as a texture
// (`imageTap`), which has no meaning on the CPU. Each texture-reading Metal function is split in
// two: the "taps" (the texture reads, the neighbour loops and the early `return` on an unwritten
// texture, whose alpha is 0) stay in the Metal backend, and everything that is arithmetic on the
// values already read is a function here. Each function's comment names the Metal function it
// came from and the taps its caller makes. `particleField` reads no texture and is here whole.
//
// Name map from graphics_metal (each function's Tags line repeats its old name):
//   reactionDiffusionStep → grayScottStartState, grayScottLaplacian, grayScottReaction,
//     grayScottBrushed (taps stay in Metal),
//   reactionDiffusionView → grayScottViewColor,
//   lifeStep → lifeCellStep, lifeCellCentreUv, lifeStartColor, lifeAdvances, lifeNeighbourPoint,
//     lifeCellIsAlive, lifeRule, lifeNextState (taps stay in Metal),
//   lifeView → lifeViewColor,
//   fluidVelocity → fluidVelocityOf, fluidDivergence → fluidDivergenceOf,
//   fluidStep → fluidStartState, fluidAspect, fluidAdvectionOrigin, fluidViscousVelocity,
//     fluidDampedVelocity, fluidFalloff, fluidNextState (taps stay in Metal),
//   fluidView → fluidViewColor, particleField → particleField (with particlePosition).
//   New: grayScottEdgeWeight, grayScottCornerWeight, particleMaximumCount.
//
// Stays in the Metal backend: `imageTap`, `texelOf` and the neighbour loops (all texture reads),
// and each step's early return for an unwritten texture (it tests the alpha of a tap).
//
// Differences from graphics_metal, all on purpose: `mix`, `smoothstep`, `clamp`, `fract`, `floor`
// and the others are the prelude's names (`linearBlend`, `safeSmoothStep`, `clamped`,
// `fractionalPart`, `roundedDown`), `glslMod` is `flooredModulo`, `hsvToRgb` is `rgbFromHSV`,
// the constants are `tau<float>()`, and a smooth step with equal edges is a hard step, not
// undefined (it differs from graphics_metal only when a particle's `size` is under about 0.000001).
#pragma once

#include "Hashes.h"
#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderColor.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

/// The weight of the four neighbours that share an edge with a cell (left, right, up, down) in the
/// Gray–Scott Laplacian: 0.2.
///
/// How: the nine-point stencil with weights 0.2 for edge neighbours, 0.05 for corner neighbours and
/// −1 for the centre, the one graphics_metal uses; the weights sum to 0.2×4 + 0.05×4 − 1 = 0, so a
/// flat field has no Laplacian.
///
/// Use when: weighting the four edge samples of a reaction-diffusion Laplacian
/// Tags: Laplacian weights, diffusion stencil, nine-point stencil, reactionDiffusionStep
/// Node: inside reaction-diffusion
inline float grayScottEdgeWeight() { return 0.2f; }

/// The weight of the four neighbours that touch a cell only at a corner in the Gray–Scott
/// Laplacian: 0.05.
///
/// Use when: weighting the four corner samples of a reaction-diffusion Laplacian
/// Tags: Laplacian weights, diffusion stencil, nine-point stencil, reactionDiffusionStep
/// Node: inside reaction-diffusion
inline float grayScottCornerWeight() { return 0.05f; }

/// The state of a Gray–Scott texture that has never been written: chemical A everywhere, chemical
/// B in sparse specks and in a disc at the middle.
///
/// From: the "start here" branch of `reactionDiffusionStep`, taken when the tap at `uv` has alpha 0.
/// Taps: none (the caller reads the centre only to see its alpha).
///
/// How: the speck cell is ⌊uv / (texel × 12)⌋, a block of 12×12 texels; B is 1 where hash21(cell)
/// > 0.985 (about 1.5% of blocks) or where |uv − (0.5, 0.5)| < 0.04, else 0. The result is
/// (A, B, 0, 1) = (1, B, 0, 1): red is A, green is B, alpha 1 marks the texture as written.
///
/// Conventions: `uv` 0…1 from the bottom left; `texel` is 1 / (width, height) of the state.
///
/// Use when: seeding a reaction-diffusion simulation so patterns grow from specks and a central disc
/// Tags: reactionDiffusionStep, Gray-Scott, seed, initial state, specks
/// Node: inside reaction-diffusion
inline Vector4Of<float> grayScottStartState(Vector2Of<float> uv, Vector2Of<float> texel) {
    bool speck = hash21(roundedDown(uv / (texel * 12.0f))) > 0.985f;
    bool disc = lengthOf(uv - 0.5f) < 0.04f;
    return Vector4Of<float>(1.0f, chosen(speck || disc, 1.0f, 0.0f), 0.0f, 1.0f);
}

/// The Laplacian of the two Gray–Scott chemicals at a cell: how far (A, B) is from the weighted
/// average of its neighbours.
///
/// From: the nine-tap loop of `reactionDiffusionStep`.
/// Taps: the eight neighbours of the cell, at uv + (x, y) × texel for x, y in −1…1 (not both 0);
/// the caller adds the red and green of the four edge ones into `edgeSum` and of the four corner
/// ones into `cornerSum`.
///
/// How: ∇² ≈ −centre + 0.2 × edgeSum + 0.05 × cornerSum, component by component (A in x, B in y).
/// A discrete Laplacian: zero on a flat field, negative at a peak, positive in a pit.
///
/// Conventions: `centre`, `edgeSum` and `cornerSum` are (A, B) pairs, each chemical 0…1.
///
/// Use when: the diffusion term of a reaction-diffusion step
/// Tags: reactionDiffusionStep, Laplacian, diffusion, nine-point stencil, discrete Laplace operator
/// Node: inside reaction-diffusion
inline Vector2Of<float> grayScottLaplacian(Vector2Of<float> centre, Vector2Of<float> edgeSum, Vector2Of<float> cornerSum) {
    return grayScottEdgeWeight() * edgeSum + grayScottCornerWeight() * cornerSum - centre;
}

/// One Gray–Scott step of a cell: the next (A, B) from the cell's (A, B), its Laplacians and the
/// parameters.
///
/// From: the last lines of `reactionDiffusionStep` before the brush.
/// Taps: none; `laplacian` comes from `grayScottLaplacian`.
///
/// How: reaction = A B² (B turns A into more B);
/// A' = A + (diffusionA ∇²A − reaction + feed (1 − A)) × rate;
/// B' = B + (diffusionB ∇²B + reaction − (kill + feed) B) × rate;
/// both clamped to 0…1. (Gray and Scott, 1984; Pearson, 1993.) `feed` refills A, `kill` removes B.
/// The brush (`grayScottBrushed`) may be applied after the clamp: it sets B to 1, which the clamp
/// would keep, so the order does not matter.
///
/// Conventions: returns (A, B); with B = 0 and ∇² = 0 and A = 1 the state does not change (A refills
/// to 1, nothing reacts). Stable for rate × diffusion up to about 1.
///
/// Use when: advancing a reaction-diffusion texture by one frame; spots, stripes and coral patterns
/// Tags: reactionDiffusionStep, Gray-Scott, Turing pattern, feed, kill, reaction diffusion
/// Node: inside reaction-diffusion
inline Vector2Of<float> grayScottReaction(Vector2Of<float> state, Vector2Of<float> laplacian, float feed, float kill,
                                          float diffusionA, float diffusionB, float rate) {
    float a = state.x;
    float b = state.y;
    float reaction = a * b * b;
    a += (diffusionA * laplacian.x - reaction + feed * (1.0f - a)) * rate;
    b += (diffusionB * laplacian.y + reaction - (kill + feed) * b) * rate;
    return Vector2Of<float>(clamped(a, 0.0f, 1.0f), clamped(b, 0.0f, 1.0f));
}

/// The state with chemical B forced to 1 inside the brush disc, so a pointer can paint patterns.
///
/// From: the brush line of `reactionDiffusionStep`.
/// Taps: none.
///
/// How: when `brushing` > 0.5 and |(uv − brush) / (texel × 8)| < 1 (a disc of 8 texels around the
/// brush), B = 1; A is kept. Otherwise the state is returned unchanged.
///
/// Conventions: `brush` is a uv position; `brushing` is a 0…1 flag.
///
/// Use when: painting into a reaction-diffusion texture with the pointer
/// Tags: reactionDiffusionStep, brush, paint, pointer, stir
/// Node: inside reaction-diffusion
inline Vector2Of<float> grayScottBrushed(Vector2Of<float> state, Vector2Of<float> uv, Vector2Of<float> brush, float brushing,
                                         Vector2Of<float> texel) {
    bool inside = brushing > 0.5f && lengthOf((uv - brush) / (texel * 8.0f)) < 1.0f;
    return Vector2Of<float>(state.x, chosen(inside, 1.0f, state.y));
}

/// The colour a Gray–Scott state is drawn with: `low` where there is no B, `high` where B is 0.5
/// or more.
///
/// From: `reactionDiffusionView`.
/// Taps: none (the caller passes the state's colour).
///
/// How: rgb = linearBlend(low, high, smoothstep(0, 0.5, B)) with B the state's green; alpha 1.
///
/// Use when: showing a reaction-diffusion simulation as a two-colour image
/// Tags: reactionDiffusionView, Gray-Scott view, colourise, two-tone
/// Node: inside reaction-diffusion
inline Vector4Of<float> grayScottViewColor(Vector4Of<float> state, Vector3Of<float> low, Vector3Of<float> high) {
    Vector3Of<float> color = linearBlend(low, high, safeSmoothStep(0.0f, 0.5f, state.y));
    return Vector4Of<float>(color.x, color.y, color.z, 1.0f);
}

/// The size of a Life cell in uv: `cell` pixels wide (at least 1) divided by the texture size.
///
/// From: the first lines of `lifeStep`.
/// Taps: none (the caller passes the texture's width and height as `pixels`).
///
/// How: max(cell, 1) / (width, height).
///
/// Use when: laying a Life grid of square cells over a texture
/// Tags: lifeStep, cell size, grid step, cellular automaton
/// Node: inside game-of-life
inline Vector2Of<float> lifeCellStep(float cell, Vector2Of<float> pixels) {
    return Vector2Of<float>(largerOf(cell, 1.0f)) / pixels;
}

/// The uv of the centre of the Life cell that contains `uv`.
///
/// From: `centreUv` in `lifeStep`. Taps: the caller reads the cell's state there.
///
/// How: (⌊uv / step⌋ + 0.5) × step.
///
/// Use when: reading one value per cell instead of per pixel
/// Tags: lifeStep, cell centre, snap to grid, cellular automaton
/// Node: inside game-of-life
inline Vector2Of<float> lifeCellCentreUv(Vector2Of<float> uv, Vector2Of<float> step) {
    return (roundedDown(uv / step) + 0.5f) * step;
}

/// The colour of a cell the first time the texture is read: white if alive, black if dead, chosen
/// by a hash of the cell so a given `density` of cells starts alive.
///
/// From: the "start here" branch of `lifeStep`. Taps: none (the caller read alpha 0 at the centre).
///
/// How: alive = hash21(⌊uv / step⌋) < density; the colour is (alive, alive, alive, 1).
///
/// Conventions: `density` 0…1 is the fraction alive; the same cell always gets the same answer.
///
/// Use when: seeding a Game of Life with random cells
/// Tags: lifeStep, random start, seed, density, cellular automaton
/// Node: inside game-of-life
inline Vector4Of<float> lifeStartColor(Vector2Of<float> uv, Vector2Of<float> step, float density) {
    float alive = chosen(hash21(roundedDown(uv / step)) < density, 1.0f, 0.0f);
    return Vector4Of<float>(alive, alive, alive, 1.0f);
}

/// Whether this frame is one on which the Life generation advances: one frame in every `interval`.
///
/// From: the `glslMod(frame, max(interval, 1.0)) >= 1.0` early return of `lifeStep` (inverted: when
/// this is false the caller returns the centre unchanged). Taps: none.
///
/// How: flooredModulo(frame, max(interval, 1)) < 1.
///
/// Use when: slowing a Life to a few generations per second
/// Tags: lifeStep, interval, generation, frame rate, cellular automaton
/// Node: inside game-of-life
inline bool lifeAdvances(float frame, float interval) {
    return flooredModulo(frame, largerOf(interval, 1.0f)) < 1.0f;
}

/// The uv of a neighbour of a Life cell, wrapping round the edge so the world is a torus.
///
/// From: `point` in `lifeStep`. Taps: the caller reads the state there for each of the eight
/// (x, y) in −1…1, not both 0, and counts those with `lifeCellIsAlive`.
///
/// How: fract(centreUv + (x, y) × step).
///
/// Use when: counting neighbours on a wrapped grid
/// Tags: lifeStep, neighbour, torus, wrap around, cellular automaton
/// Node: inside game-of-life
inline Vector2Of<float> lifeNeighbourPoint(Vector2Of<float> centreUv, int offsetX, int offsetY, Vector2Of<float> step) {
    Vector2Of<float> offset(static_cast<float>(offsetX), static_cast<float>(offsetY));
    return fractionalPart(centreUv + offset * step);
}

/// Whether a state counts as alive: its red is more than 0.5.
///
/// From: the `imageTap(...).r > 0.5` tests of `lifeStep` (neighbours) and its `centre.r > 0.5`.
/// Taps: none; the caller passes the red of a tap.
///
/// Use when: counting live neighbours or reading the cell's own state
/// Tags: lifeStep, alive, threshold, cellular automaton
/// Node: inside game-of-life
inline bool lifeCellIsAlive(float red) { return red > 0.5f; }

/// Conway's rule (B3/S23): whether a cell is alive next generation.
///
/// From: `lives` in `lifeStep`. Taps: none; the neighbours were counted by the caller.
///
/// How: exactly 3 live neighbours gives birth (or keeps a live cell); exactly 2 keeps a live cell
/// alive and leaves a dead one dead; any other count kills (loneliness under 2, crowding over 3).
///
/// Conventions: `neighbours` is 0…8.
///
/// Use when: stepping Conway's Game of Life
/// Tags: lifeStep, Conway, B3/S23, birth, survival, game of life
/// Node: inside game-of-life
inline bool lifeRule(bool wasAlive, int neighbours) {
    return neighbours == 3 || (wasAlive && neighbours == 2);
}

/// The next state of a Life cell: white if it lives, otherwise a blue trail that fades.
///
/// From: the tail of `lifeStep`. Taps: none; `centre` is the cell's current state.
///
/// How: wasAlive = centre.r > 0.5; if `lifeRule` then (1, 1, 1, 1); else trail = 1 if it was alive
/// (just died), or centre.b × fade (the old trail fading); the colour is
/// (0.25 trail, 0.6 trail, trail, 1). So red > 0.5 means alive and blue carries the trail.
///
/// Conventions: `fade` 0…1 is the fraction of the trail kept per generation.
///
/// Use when: finishing a Game of Life generation
/// Tags: lifeStep, next state, trail, fade, cellular automaton
/// Node: inside game-of-life
inline Vector4Of<float> lifeNextState(Vector4Of<float> centre, int neighbours, float fade) {
    bool wasAlive = lifeCellIsAlive(centre.x);
    if (lifeRule(wasAlive, neighbours)) return Vector4Of<float>(1.0f);
    float trail = chosen(wasAlive, 1.0f, centre.z * fade);
    return Vector4Of<float>(trail * 0.25f, trail * 0.6f, trail, 1.0f);
}

/// The colour a Life state is drawn with: `alive` for a live cell, `dead` with a blue glow of the
/// trail for a dead one.
///
/// From: `lifeView`. Taps: none.
///
/// How: trail = 0 if red > 0.5 else blue; rgb = alive if red > 0.5, else
/// linearBlend(dead, dead + (0, 0.15, 0.4), trail); alpha 1.
///
/// Use when: showing a Game of Life
/// Tags: lifeView, Game of Life view, colourise, trail glow
/// Node: inside game-of-life
inline Vector4Of<float> lifeViewColor(Vector4Of<float> state, Vector3Of<float> alive, Vector3Of<float> dead) {
    bool isAlive = lifeCellIsAlive(state.x);
    float trail = chosen(isAlive, 0.0f, state.z);
    Vector3Of<float> color = chosen(isAlive, alive, linearBlend(dead, dead + Vector3Of<float>(0.0f, 0.15f, 0.4f), trail));
    return Vector4Of<float>(color.x, color.y, color.z, 1.0f);
}

/// A velocity from a stored colour: red and green hold the velocity as 0.5 + v / 2, so 0.5 is
/// still.
///
/// From: `fluidVelocity` (its tap `imageTap(state, uv + offset × texel).rg` stays in Metal).
///
/// How: (rg − 0.5) × 2, a velocity in −1…1.
///
/// Use when: decoding the fluid state's velocity
/// Tags: fluidVelocity, decode velocity, 0.5 is still, fluid
/// Node: inside fluid
inline Vector2Of<float> fluidVelocityOf(Vector2Of<float> storedRedGreen) {
    return (storedRedGreen - 0.5f) * 2.0f;
}

/// The divergence of the velocity at a point from its four neighbours' velocities: how much fluid
/// leaves it.
///
/// From: `fluidDivergence`. Taps: the velocities at the point ± one texel in x and in y (four
/// `fluidVelocity` reads; only the x of right and left and the y of up and down are used).
///
/// How: central difference, ∇·v ≈ ½ ((vx(right) − vx(left)) + (vy(up) − vy(down))).
///
/// Conventions: one texel is the unit of length; positive is outflow.
///
/// Use when: the pressure-like damping of a fluid step; checking a velocity field is incompressible
/// Tags: fluidDivergence, divergence, central difference, incompressible
/// Node: inside fluid
inline float fluidDivergenceOf(Vector2Of<float> right, Vector2Of<float> left, Vector2Of<float> up, Vector2Of<float> down) {
    return 0.5f * ((right.x - left.x) + (up.y - down.y));
}

/// The state of a fluid texture that has never been written: no velocity (0.5, 0.5) and no dye.
///
/// From: the early return of `fluidStep` when the centre's alpha is 0. Taps: none.
///
/// Use when: starting a fluid simulation
/// Tags: fluidStep, initial state, still fluid
/// Node: inside fluid
inline Vector4Of<float> fluidStartState() { return Vector4Of<float>(0.5f, 0.5f, 0.0f, 1.0f); }

/// The width-to-height ratio as (ratio, 1), so a distance measured in uv is round, not stretched.
///
/// From: `aspect` in `fluidStep`. Taps: none (the caller passes the texture's width and height).
///
/// Use when: measuring distances on a non-square texture
/// Tags: fluidStep, aspect ratio, round distances
/// Node: inside fluid
inline Vector2Of<float> fluidAspect(float width, float height) { return Vector2Of<float>(width / height, 1.0f); }

/// Where a fluid parcel now at `uv` came from: back along its velocity.
///
/// From: `origin` in `fluidStep`. Taps: the caller reads the state there (`moved`) and the four
/// velocities and four divergences around it.
///
/// How: semi-Lagrangian advection (Stam, "Stable Fluids", 1999): uv − velocity × texel × 20, the
/// velocity (−1…1) times 20 texels.
///
/// Use when: advecting the velocity and dye of a fluid without instability
/// Tags: fluidStep, advection, semi-Lagrangian, backtrace, stable fluids
/// Node: inside fluid
inline Vector2Of<float> fluidAdvectionOrigin(Vector2Of<float> uv, Vector2Of<float> velocity, Vector2Of<float> texel) {
    return uv - velocity * texel * 20.0f;
}

/// The velocity after viscosity: pulled toward the average of its four neighbours.
///
/// From: the `moving = mix(...)` line of `fluidStep`. Taps: the velocities right, left, up, down of
/// the origin (decoded with `fluidVelocityOf`).
///
/// How: linearBlend(moving, ¼ (right + left + up + down), clamp(viscosity, 0, 1)). Viscosity 1 is a
/// pure average (a blur of the velocity), 0 leaves it alone.
///
/// Use when: making a fluid thick and smooth
/// Tags: fluidStep, viscosity, diffusion, velocity blur
/// Node: inside fluid
inline Vector2Of<float> fluidViscousVelocity(Vector2Of<float> moving, Vector2Of<float> right, Vector2Of<float> left,
                                             Vector2Of<float> up, Vector2Of<float> down, float viscosity) {
    return linearBlend(moving, 0.25f * (right + left + up + down), clamped(viscosity, 0.0f, 1.0f));
}

/// The velocity with its divergence damped: one Jacobi iteration of a pressure solve started from
/// zero pressure, which smooths divergence away.
///
/// From: the `moving += damping × 0.5 × ...` line of `fluidStep`. Taps: the divergence (see
/// `fluidDivergenceOf`) at the origin ± one texel in x and in y: four divergences, sixteen reads.
///
/// How: v + damping × ½ × (div(right) − div(left), div(up) − div(down)), i.e. a step down the
/// gradient of the divergence (the pressure gradient projection of Stam 1999, one iteration).
///
/// Use when: keeping a fluid roughly incompressible without a full pressure solve
/// Tags: fluidStep, pressure, Jacobi, divergence damping, projection
/// Node: inside fluid
inline Vector2Of<float> fluidDampedVelocity(Vector2Of<float> moving, float divergenceRight, float divergenceLeft,
                                            float divergenceUp, float divergenceDown, float damping) {
    return moving + damping * 0.5f * Vector2Of<float>(divergenceRight - divergenceLeft, divergenceUp - divergenceDown);
}

/// How strongly the stir at `source` reaches `uv`: 1 at the source, falling off as a Gaussian of
/// the distance.
///
/// From: `falloff` in `fluidStep`. Taps: none.
///
/// How: exp(−|(uv − source) × aspect|² / (radius² + 0.000001)). The distance is measured with
/// `aspect` so the push is round on a non-square texture.
///
/// Conventions: `radius` in uv heights; the 0.000001 keeps a zero radius finite (a point push).
///
/// Use when: pushing a fluid with a pointer; the weight of a soft brush
/// Tags: fluidStep, falloff, gaussian brush, stir force, distance to source
/// Node: inside fluid
inline float fluidFalloff(Vector2Of<float> uv, Vector2Of<float> source, Vector2Of<float> aspect, float radius) {
    Vector2Of<float> away = (uv - source) * aspect;
    return exponential(-dotProduct(away, away) / (radius * radius + 0.000001f));
}

/// The next fluid state: the velocity pushed along `push`, the dye faded and fed, both packed back
/// into a colour.
///
/// From: the last four lines of `fluidStep`. Taps: none; `movedDye` is the blue of the tap at the
/// advection origin.
///
/// How: velocity = moving + push × force × falloff; dye = movedDye × dissipation + falloff ×
/// min(|push| × force × 4, 1); the colour is (clamp(velocity / 2 + 0.5, 0, 1), clamp(dye, 0, 1), 1)
/// (the inverse of `fluidVelocityOf`).
///
/// Conventions: `dissipation` 0…1 is the dye kept per frame (1 never fades).
///
/// Use when: finishing a fluid frame
/// Tags: fluidStep, dissipation, dye, stir, push, pack velocity
/// Node: inside fluid
inline Vector4Of<float> fluidNextState(Vector2Of<float> moving, float movedDye, Vector2Of<float> push, float force,
                                       float falloff, float dissipation) {
    Vector2Of<float> velocity = moving + push * force * falloff;
    float dye = movedDye * dissipation + falloff * smallerOf(lengthOf(push) * force * 4.0f, 1.0f);
    Vector2Of<float> stored = clamped(velocity * 0.5f + 0.5f, 0.0f, 1.0f);
    return Vector4Of<float>(stored.x, stored.y, clamped(dye, 0.0f, 1.0f), 1.0f);
}

/// The colour a fluid state is drawn with: the dye colour over a dark blue, plus a blue-white glow
/// where the fluid moves fast.
///
/// From: `fluidView`. Taps: none.
///
/// How: speed = clamp(|velocity| × 3, 0, 1) with velocity = fluidVelocityOf(state.rg);
/// rgb = linearBlend((0.02, 0.03, 0.06), dyeColor, state.b) + speed × (0.15, 0.25, 0.4); alpha 1.
///
/// Use when: showing a fluid simulation
/// Tags: fluidView, fluid view, dye colour, speed glow
/// Node: inside fluid
inline Vector4Of<float> fluidViewColor(Vector4Of<float> state, Vector3Of<float> dyeColor) {
    float speed = clamped(lengthOf(fluidVelocityOf(Vector2Of<float>(state.x, state.y))) * 3.0f, 0.0f, 1.0f);
    Vector3Of<float> color = linearBlend(Vector3Of<float>(0.02f, 0.03f, 0.06f), dyeColor, state.z)
        + speed * Vector3Of<float>(0.15f, 0.25f, 0.4f);
    return Vector4Of<float>(color.x, color.y, color.z, 1.0f);
}

/// The most particles `particleField` draws: 64. A larger `count` is cut to it.
///
/// Use when: knowing the cost bound of the particle field (it loops this many times at most)
/// Tags: particleField, particle limit, maximum count
/// Node: inside particle-field
inline int particleMaximumCount() { return 64; }

/// Where particle number `random` is at `time`, and how alive it is, for one of the four motions.
///
/// From: the `if (motion == ...)` chain inside `particleField`. Taps: none.
///
/// How: `random` and `random2` are two hashes of the particle's index (hash22 of (i, seed) and of
/// (i + 100, seed)); in uv, with t = speed × time:
///   0 drifting: fract(random + (random2 − 0.5) × t × 0.2), life 1;
///   1 rising: x = random.x + 0.03 sin(time (1 + 2 random2.x) + τ random.y) (a sway),
///     y = fract(random.y + (0.3 + random2.y) × t × 0.1); life = 1 − y (fades as it rises);
///   2 orbiting: radius = 0.05 + 0.4 random.x spread, angle = τ random.y + (random2.x + 0.5) × t × 0.5,
///     position = 0.5 + radius (cos, sin) / aspect (round on a wide screen), life 1;
///   3 outward (any other number): age = fract(random.x + t × 0.1), angle = τ random.y,
///     position = 0.5 + (cos, sin) × age × 0.6 × spread / aspect, life = 1 − age.
/// Returns (x, y, life).
///
/// Conventions: positions are in uv 0…1 (may leave it for sway); `aspect` is (width / height, 1);
/// `motion` outside 0…2 is 3.
///
/// Use when: placing a particle from its index with no stored state
/// Tags: particleField, particle motion, drift, rise, orbit, outward burst, stateless particles
/// Node: inside particle-field
inline Vector3Of<float> particlePosition(Vector2Of<float> random, Vector2Of<float> random2, Vector2Of<float> aspect,
                                         float speed, float time, float spread, int motion) {
    float elapsed = speed * time;
    if (motion == 0) {
        Vector2Of<float> position = fractionalPart(random + (random2 - 0.5f) * elapsed * 0.2f);
        return Vector3Of<float>(position.x, position.y, 1.0f);
    }
    if (motion == 1) {
        float x = random.x + 0.03f * sine(time * (1.0f + random2.x * 2.0f) + random.y * tau<float>());
        float y = fractionalPart(random.y + (0.3f + random2.y) * elapsed * 0.1f);
        return Vector3Of<float>(x, y, 1.0f - y);
    }
    if (motion == 2) {
        float radius = 0.05f + 0.4f * random.x * spread;
        float angle = random.y * tau<float>() + (random2.x + 0.5f) * elapsed * 0.5f;
        Vector2Of<float> position = 0.5f + radius * Vector2Of<float>(cosine(angle), sine(angle)) / aspect;
        return Vector3Of<float>(position.x, position.y, 1.0f);
    }
    float age = fractionalPart(random.x + elapsed * 0.1f);
    float angle = random.y * tau<float>();
    Vector2Of<float> position = 0.5f + Vector2Of<float>(cosine(angle), sine(angle)) * age * 0.6f * spread / aspect;
    return Vector3Of<float>(position.x, position.y, 1.0f - age);
}

/// Soft coloured dots moving as a function of their index and the time, with no stored state:
/// colour in rgb, density in alpha.
///
/// From: `particleField`, whole (it reads no texture). Taps: none.
///
/// How: for each of n = clamp(⌊count⌋, 0, 64) particles i: position and life from `particlePosition`;
/// distance = |uv × aspect − position × aspect| (round on a wide screen); shape = smoothstep(size,
/// 0.2 size, distance) × life (1 at the centre, 0 beyond `size`); the colour adds
/// rgbFromHSV(hue = random2.y, saturation 0.5, value 1) × shape; the density adds shape. The result
/// is (the colour sum, clamp(density, 0, 1)).
///
/// Conventions: `uv` 0…1 from the bottom left; `aspect` is (width / height, 1); `size` in screen
/// heights; `seed` picks a different arrangement; `motion` 0 drifting, 1 rising, 2 orbiting,
/// 3 flying outward from the middle; `spread` scales orbit radius and outward reach. Colour is not
/// clamped, so overlapping particles add up.
///
/// Use when: dust, sparks, bokeh or a starburst over a background; any effect of many soft dots
/// Tags: particleField, particles, dust, bokeh, sparks, stateless, procedural particles
/// Node: inside particle-field
inline Vector4Of<float> particleField(Vector2Of<float> uv, Vector2Of<float> aspect, float count, float size, float speed,
                                      float time, float seed, int motion, float spread) {
    Vector2Of<float> p = uv * aspect;
    Vector3Of<float> total(0.0f);
    float density = 0.0f;
    int particles = static_cast<int>(clamped(count, 0.0f, static_cast<float>(particleMaximumCount())));
    for (int index = 0; index < particles; index += 1) {
        Vector2Of<float> random = hash22(Vector2Of<float>(static_cast<float>(index), seed));
        Vector2Of<float> random2 = hash22(Vector2Of<float>(static_cast<float>(index) + 100.0f, seed));
        Vector3Of<float> placed = particlePosition(random, random2, aspect, speed, time, spread, motion);
        float distance = lengthOf(p - Vector2Of<float>(placed.x, placed.y) * aspect);
        float shape = safeSmoothStep(size, size * 0.2f, distance) * placed.z;
        total += rgbFromHSV(Vector3Of<float>(random2.y, 0.5f, 1.0f)) * shape;
        density += shape;
    }
    return Vector4Of<float>(total.x, total.y, total.z, clamped(density, 0.0f, 1.0f));
}

} // namespace kernels
} // namespace editor
