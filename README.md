# math_cpp

The math behind several products, in C++20 with only the standard library: vectors and
matrices, numerics, colour, geometry, curves, meshes, signed distance fields, optics, physics,
skeletons, shader graphs, and kernels that compile both as C++ and as Metal. Each field is its
own module, so a host takes only what it needs.

| Module | Uses | |
|---|---|---|
| `Kernels` | — | shared CPU/GPU formulas: noise, colour, blend modes, patterns, shape distances, raymarching, effects (header-only, C++14, compiles as Metal) |
| `Linear` | Kernels | Scalar, Angle, Vector2/3/4, Matrix3/4, Quaternion, Transform |
| `Numerics` | Linear | polynomial roots, integration, particle integrators, seeded random numbers |
| `Color` | Kernels, Linear | sRGB, wavelength colour |
| `Geometry` | Linear | bounds, lines, planes, rays, segments, triangles, spheres, intersections |
| `Curves` | Geometry, Numerics, Linear | Bézier, B-spline, Catmull–Rom, paths, polygons, offsets, fitting |
| `Mesh` | Curves, Geometry, Linear | meshes, half-edge meshes, primitives, revolve, sweep, shading |
| `Fields` | Mesh, Curves, Geometry, Numerics, Linear | signed distance fields, gradient noise, surface nets |
| `Optics` | Color, Geometry, Linear | refraction, Fresnel, microfacets, dispersion, caustics |
| `Physics` | Linear | damped springs, mass properties |
| `Animation` | Linear | bones, skeletons, poses |
| `Shaders` | — | shader graphs compiled to Metal source, with every node family |
| `Curves2D` | — | 2D curves in doubles with a C API and wasm build (formerly graphics_cpp) |

[Docs/Foundations.md](Docs/Foundations.md) has the layers, the include rule, the kernel rules
and the deterministic tier.

## Build and test

```bash
make test
```

builds one static library per module (`build/lib/libmath_<module>.a`) and runs the unit tests,
the kernel checks (C++14 and Metal), the GPU agreement cases and the shader previews (the last
three need Xcode's Metal compiler and are skipped without it).

`Curves2D` builds with CMake:

```bash
cmake -S Curves2D -B Curves2D/build && cmake --build Curves2D/build && Curves2D/build/curves_tests
```

## Using it

**make or any build system:** put the folder that holds `math_cpp` on the include path,
include `"math_cpp/<Module>/<Header>.h"`, and link the module's library and those it uses
(`make libraries-of MODULE=Fields` prints them in link order). Compile with
`-ffp-contract=off` so results agree to the last bit on every machine.

**SwiftPM:** each module is a product named `Math<Module>` (`MathLinear`, `MathFields`, …);
`Curves2D` keeps the products `CCurves` and `Curves`.

```swift
.package(path: "../math_cpp"),
// ...
.target(name: "App", dependencies: [.product(name: "MathFields", package: "math_cpp")],
        swiftSettings: [.interoperabilityMode(.Cxx)])
```

Everything is in the C++ namespace `editor` (`editor.Vector3` from Swift).

## License

MIT, see [LICENSE](LICENSE). `Curves2D` builds on Pomax's Bézier work
([Curves2D/THIRD_PARTY_NOTICES.md](Curves2D/THIRD_PARTY_NOTICES.md)).
