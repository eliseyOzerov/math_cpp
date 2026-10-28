# math_cpp: agent instructions

C++20 math shared by several products (Marble, Fountain, Kagiro, an Unreal plugin). Humans must
be able to read, understand and maintain every line. Read
[Docs/Foundations.md](Docs/Foundations.md) before adding a module, a kernel, or anything that
must give the same result on every machine.

## Modules

- Each top-level folder is one module and one static library. A file includes only from its
  own module and lower layers (`Tests/LayerTests.cpp`); a new folder is added there, to the
  Makefile's `MODULES` and `DEPENDS_<Module>`, and to `Package.swift`, in the same change.
- Modules include each other by relative path (`"../Linear/Vector3.h"`).
- `Kernels/` is header-only C++14 that also compiles as Metal: no standard library, heap,
  exceptions, recursion, virtual functions, function pointers or `double`. Each kernel header
  has `Tests/KernelChecks/<Header>.metal` and a GPU agreement case in `Tests/GPUAgreement/`.
  After changing a kernel, run `make shader-library` (a test fails when
  `Shaders/ShaderLibrarySource.cpp` is stale).
- `Curves2D/` keeps its own layout, rules and reference (fountain-ts); see its README.

## Self-contained

Only the C++ standard library, and in platform code (the Metal backend, the GPU tests) the
platform's own API. Other libraries may be used as inspiration or a source of math, rewritten
to these rules and cited in the doc comment ("Möller–Trumbore, 1997"). Never copy code from
GPL projects.

## Readability

- **Functions over expressions:** an expression may appear inline only when it is a one-line,
  one-off use. If the concept has a name in the domain (degrees to radians, the length of a
  vector, a linear blend, the normal of a triangle), it is a named function, written once in
  the module it belongs to. Search for an existing one first.
- Prefer domain types to bare numbers (`Angle`, not a `float` named `radians`).
- Full, textbook names: `Vector3`, `fieldOfView`, `nearPlane`. No one-letter parameters except
  the components `x`, `y`, `z`, `w`.
- Names and messages ported from Marble's Swift engine are kept word for word, so the two can
  be compared side by side.
- Comments explain *why*. Every repeatable function's doc comment says what it returns, how
  (the formula, with a diagram when geometric), and its conventions (units, Z-up right-handed,
  row- or column-major, input ranges, edge cases). It ends with:

```cpp
/// Use when: picking the object under the cursor; snapping a point onto a road's centre line.
/// Tags: nearest point, project onto line, distance to segment, snapping
/// Node: planned (closestPoint)
```

  `Use when:` gives one to three concrete situations; `Tags:` the words someone would search
  for; `Node:` (modelling modules) the node that offers it, uses it, or still needs it, or why
  it is not a modelling step. Initializers, constants and enum cases need only the summary.

## Proven math

Every math function has a cited source, its derivation in the doc comment, tests with
hand-worked values, and tests of properties that must always hold.

## Building and testing

```bash
make test
```

Unit tests are in `Tests/*.cpp` (`TEST(name) { ... }`, `Tests/Testing.h`). Compare floats with
a tolerance unless the result is exact. `swift build` and `swift test` check the SwiftPM
products. `Curves2D` builds and tests with CMake (README.md).

## Git

Never add `Co-Authored-By` lines to commit messages.
