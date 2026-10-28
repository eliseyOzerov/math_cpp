# math_cpp's foundations: modules, layers, kernels, determinism

The rules the library grows by. math_cpp is one body of math shared by several products
(Marble's 3D editor, Fountain's fonts and vector ink, Kagiro's shaders, an Unreal plugin), so
each field is its own module that a host can take without the others.

## Modules and layers

Each top-level folder is one module, owns one main data type, and may include only from
modules in lower layers. `make` builds each as its own static library
(`build/lib/libmath_<module>.a`); `Package.swift` makes each a SwiftPM product (`MathLinear`,
`MathFields`, …). A host links only the modules it needs.

| Layer | Module (folder) | Uses | Main types |
|---|---|---|---|
| L0 | `Kernels` | — | the shared CPU/GPU kernels and their prelude (header-only) |
| L0 | `Linear` | Kernels | Scalar, Angle, Vector2/3/4, Matrix3/4, Quaternion, Transform; NumberText and the problem types every layer reports with (GeometryProblem, ParameterProblem) |
| L1 | `Numerics` | Linear | roots, quadrature, particle integrators, seeded random numbers |
| L1 | `Color` | Kernels, Linear | sRGB, wavelength colour |
| L2 | `Geometry` | Linear | bounds, lines, planes, rays, segments, triangles, spheres, intersections |
| L2 | `Curves` | Geometry, Numerics, Linear | Bézier, B-spline, Catmull–Rom, 2D paths and shapes, booleans, offsets, fitting, ink strokes, SVG paths |
| L3 | `Mesh` | Curves, Geometry, Linear | Mesh, HalfEdgeMesh, primitives, revolve, sweep, shading, spherical panels |
| L3 | `Fields` | Mesh, Curves, Geometry, Numerics, Linear | signed distance fields, noise, surface nets, narrow-band meshing |
| L3 | `Optics` | Color, Geometry, Linear | refraction, Fresnel, microfacets, dispersion, caustics |
| L4 | `Physics` | Linear | damped springs, mass properties |
| L4 | `Animation` | Linear | bones, skeletons, poses |
| L4 | `Shaders` | — (the kernels as text) | shader graphs, the node catalogue, the compiler and its Metal backend |
| later | `Signal`, `Statistics`, `Optimization`, dense linear algebra, `Decimal` | | FFT and DSP; distributions and fits; minimisers; large matrices; exact money arithmetic |

The "Uses" column is what a module includes directly; the Makefile's `DEPENDS_<Module>` and
`Package.swift`'s target dependencies repeat it. `make libraries-of MODULE=Fields` prints the
libraries a program using Fields links.

**The include rule:** a file may include only from its own module and from modules in lower
layers. `Tests/LayerTests.cpp` maps every module folder to its layer, and fails on a folder
with no layer, and on an include that points up, or sideways within a layer without being
listed. The allowed sideways uses are Linear → Kernels, Curves → Geometry and Fields → Mesh;
there are no other exceptions. Hosts and domain libraries (fonts, audio, painting, a node
graph) depend on math_cpp, never the other way round.

Modules include each other by relative path (`#include "../Linear/Vector3.h"`), so a host
needs only the repository's root folder on its include path and the libraries it links:
`#include "math_cpp/Fields/SignedDistances.h"` with `-I <folder holding math_cpp>`.

## Shared kernels: one source for CPU and GPU

The Metal Shading Language is a dialect of C++, so a formula written once can compile for
both. Those formulas live in `Kernels/`, under stricter rules than the rest of
the core:

- **Header-only, C++14-level features, no standard library.** The Metal check builds with
  `-std=metal3.0`, which is based on C++14; we keep that floor for older devices.
- **No** heap, exceptions, recursion, virtual functions, function pointers, or `double`.
  Values are passed by copy, so no Metal address-space words (`device`, `thread`) appear in
  shared code; this also keeps an HLSL port possible later.
- **Templated on the number type** (`template <typename Real>`), so the same formula runs
  on `float`, and on the deterministic number types below.
- **A prelude** (`Kernels/KernelPrelude.h`) maps the system math functions to `std::` on
  the CPU and `metal::` on the GPU through one namespace alias, and calls them by
  argument-dependent lookup, so a number type of our own brings its own `sqrt`. It holds the
  scalar helpers (`squareRoot`, `absolute`, `smallerOf`, `largerOf`, `clamped`,
  `linearBlend`, `roundedDown`, `fractionalPart`, `sine`, `cosine`, `power`, `chosen`,
  `bitsOf`, …); `Scalar.h` calls them, so each formula exists once.
- **Kernel vectors** (`Kernels/KernelVectors.h`): `Vector2Of`, `Vector3Of`, `Vector4Of<Real>`
  are Metal's own `vec<Real, N>` on the GPU and small structs with the same operators on the
  CPU, with dot and cross products, length, normalising and the prelude's helpers applied
  per component. A kernel written against them compiles unchanged on both sides.
- **Checked on both compilers:** `make kernels` (run by `make test`) syntax-checks every
  kernel header as C++14 with `-pedantic`, and compiles `Tests/KernelChecks/<Header>.metal`
  (one per kernel header, which includes it and calls each function, since a template is only
  checked when used) with `xcrun metal -Werror` (skipped, with a message, where Xcode's
  Metal compiler is missing). A test fails if a kernel header has no check file. `Tests/KernelTests.cpp` checks the kernels on the CPU against hand-worked
  values.
- **Agreement on the GPU:** `make gpu` (also run by `make test` where Metal is installed) runs
  kernels on the Mac's GPU and on the CPU over the same inputs and compares them within a
  stated tolerance; the CPU result is the reference. A case is a compute function in
  `Tests/GPUAgreement/<Header>.metal` and its CPU twin in `<Header>.cpp` beside it, registered
  with `AGREEMENT({...})` (`Agreement.h`); `Runner.mm` is the only file that talks to Metal. The
  GPU builds with Metal's default fast math, as real shaders do, so a case allows a few units
  in the last place. Every kernel gets a case.

The first kernel is `Kernels/SchlickFresnel.h`, Schlick's Fresnel ported from
Marble's Swift `Microfacet.swift`. A kernel cannot throw, so where Swift has a
precondition the kernel's doc says the caller checks.

graphics_metal's whole function library is in the kernels, rewritten to these rules under
textbook names. Each header's top comment lists the old names, and each function's `Tags:` line
repeats its old one, so `grep -n "noiseFractal" Kernels/*.h` finds `fractalNoise`:

| Header | From | Holds |
|---|---|---|
| `ShaderMath.h`, `Hashes.h` | Core.metal, Math.metal | constants (`pi<Real>()`), guarded arithmetic, remapping (`glslMod` is `flooredModulo`, `mapRange` is `remapped`), smooth minimum, easing, logic, vector helpers, integer hashes |
| `ShaderNoise.h` | Noise.metal | value, Perlin, simplex, fractal, domain warp, curl, looping, flow, Gabor, cellular and white noise |
| `ShaderColor.h`, `BlendModes.h` | Color.metal, Blend.metal | sRGB, HSV, HSL, OKLab, OKLCH, OKHSV; grading; the 18 W3C blend modes and alpha over |
| `ShaderCoordinates.h` | Coordinates.metal | mirror, kaleidoscope, twirl, spherize, row offset, hexagonal and triangular grids, pixelate |
| `ShaderDistances.h`, `ShaderShapes.h` | SDF.metal, Shapes.metal | 2D masks, smooth booleans, repetition, 3D distances and point transforms; 18 2D shape distances |
| `ShaderPatterns.h` | Patterns.metal | antialiased masks (checker, stripes, grid, bricks, hexagons, Truchet, halftone, …), Bayer dither, gradient ramp |
| `Raymarching.h` | Raymarch.metal | sphere tracing, normals, lighting, occlusion, soft shadows, fog, sky, cameras, density marching |
| `ShaderSimulation.h` | Simulation.metal | the arithmetic of one step of Gray-Scott, the Game of Life, the fluid and the particles, and their view colours |
| `ShaderEffects.h` | Effects.metal | blur weights, Sobel, emboss, bloom, CRT, mosaic, Kuwahara, vignette, grain, lens distortion |

Three rules came out of the port:

- **Constants are functions** (`pi<Real>()`), because shared code cannot name Metal's
  `constant` address space.
- **The pixel footprint is a parameter.** `fwidth` exists only in a fragment shader, so every
  pattern that antialiased with it takes `pixelSize` instead; the fragment shader measures it
  (graphics_metal's `patternPixel`, which stays in the Metal backend).
- **A scene is a callable.** `Raymarching.h` takes the distance function as a template
  parameter: a lambda on the CPU, a struct with `operator()` in a Metal kernel (`-std=metal3.0`
  rejects lambdas). A template on a callable is resolved at compile time; it is not a function
  pointer.

Texture taps stay in the Metal backend, because only a shader can sample a texture: the
Texture, Effects and Simulation helpers there read the neighbouring texels and pass the values
to the `ShaderEffects.h` and `ShaderSimulation.h` kernels, which do the arithmetic. `triplanar`
and `lookupColor` stay whole in the backend (triplanar's blend weights have no kernel yet).
`loopAccumulate` was not ported: it takes a function by pointer.

Only code that both sides need goes into Kernels. Branchy or allocating code stays ordinary
C++.

## Two tiers of arithmetic

| Tier | Where | Rules | For |
|---|---|---|---|
| **Deterministic** | CPU only | our own `sin`, `cos`, `exp`, `sqrt`; fused multiply-adds off (`-ffp-contract=off`); a fixed order of operations; fixed-point where float is not enough | gameplay simulation, netcode and replays, anti-cheat re-checks, parity tests, anything saved and re-evaluated |
| **Fast** | CPU or GPU | system math, fused multiply-adds allowed | anything that is only drawn: shading, caustics, previews |

**The rule:** anything that must give the same bits on another machine runs on the CPU in
the deterministic tier. The GPU never decides an outcome; it only draws one.

## CPU or GPU: by the shape of the work

- **GPU:** the same formula over thousands of independent items whose result stays in
  rendering: shading, caustics, distance-field grids, large particle or cloth simulations
  that are only drawn.
- **CPU:** branchy, small, exact or sequential work: topology changes (inset, extrude,
  subdivide), a host's graph evaluator, anything feeding parity tests or an export.
- **CPU throughput:** a thread pool on `std::jthread` (Apple's libc++ has no parallel
  `std::execution`), and data laid out one array per field so loops vectorise.

## Shader graphs

A shader graph compiles to Metal source (HLSL later), which the platform compiles. `Shaders`
holds the graph, the node catalogue (one file per family in `Shaders/Nodes`), the compiler and
its Metal backend. The source it writes starts with the kernels' own text: `make
shader-library` copies every header of `Kernels/` into `ShaderLibrarySource.cpp` (committed; a
test fails when it is stale), so a node's template calls the same `checkerPattern` the CPU
tests check. Only what is Metal API stays in the backend: the uniforms, the vertex and fragment
wrappers, textures, and `loopAccumulate`, which takes a function. `make shaders` sends the
preview of every node kind through Metal's runtime compiler and builds its pipeline.

## The 2D curves

The 2D curve work of graphics_cpp (itself ported from fountain-ts) was rewritten into `Curves`
to these rules: floats, namespace `editor`, full names, `MathProblem` for broken
preconditions. The editing layer (selection, node edits), the C API and the wasm build were
left behind, and so was bit-parity with fountain-ts: lengths come from adaptive quadrature and
agree to about 1e-4, and boolean tolerances scale with the shapes. The tests transcribe
graphics_cpp's golden cases with a tolerance. graphics_cpp stays frozen as Fountain's copy.

## Each module names its reference

Most modules are checked against Marble's Swift engine (Marble's parity tests) until the
Swift engine retires. A module may name a different reference; the module's doc says which.

## Self-contained

Only our own code, the C++ standard library and, in platform code, the platform's own API
(Metal in the backend and the GPU tests). Other libraries are allowed as inspiration or as a
source of math, rewritten to these rules and cited in the doc comment. A GPL library may not
be used or copied.
