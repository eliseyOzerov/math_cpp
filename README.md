# graphics_cpp

A shared C++20 graphics library. Its first part is `curves` (namespace, targets and modules keep that name): generic
2D curve maths, cubic Béziers measured, cut, fitted and edited. It's shared by
[Fountain](../../../apps/Fountain), the font editor, and a 3D editor. It is ported from
`fountain-ts/src/curves`, which holds no font concepts and stays the reference: every TS file is ported and checked
against it. Much of it
goes back to Pomax's [Bezier.js](https://github.com/Pomax/bezierjs) and the
[Primer on Bézier Curves](https://pomax.github.io/bezierinfo) (see THIRD_PARTY_NOTICES.md).

```
include/curves/*.hpp   The C++ API, namespace curves: one header per TS file (geometry.hpp, bezier.hpp…);
                       curves.hpp includes them all.
include/curves.h       The C API (cv_*) for Swift, wasm and any other FFI.
include/module.modulemap
                       Exposes curves.h to Swift as the module CCurves.
src/                   The implementation, one .cpp per header, plus capi.cpp, js.hpp (JS arithmetic, internal) and
                       wasm.cpp (the embind API, built only by emscripten).
tests/golden/cases/    One .ts per TS file: the golden cases, run against fountain-ts.
tests/golden/gen.ts    Runs a cases file and writes tests/golden/<file>.json, which is committed.
tests/<file>.cpp       The C++ for each case of <file>.json, registered by name.
tests/main.cpp         Runs every golden file against its registered C++.
swift/Curves           The Swift module Curves: the C++ API through Swift's C++ interop.
swift/Smoke            A Swift test over the C API; swift/CurvesSmoke one over the C++ API.
tests/wasm/smoke.ts    Checks the embind API against fountain-ts, call for call.
```

## Build and test

```bash
cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
```

Run `build/curves_tests` directly to see each failure with its arguments, expected and actual values;
`build/curves_tests path.json` runs one file. After a change to the TS, regenerate a file's golden values (node ≥ 23.6,
fountain-ts at `~/Developer/apps/Fountain`):

```bash
node tests/golden/gen.ts path
```

As a CMake dependency: `add_subdirectory(curves)` or `FetchContent`, then `target_link_libraries(app PRIVATE
curves::curves)`. `cmake --install` installs the headers and a `curvesConfig` package.

## C++ API

The C++ API is plain functions over value types, with the same names and arguments as the TS:

| Type | What it is |
|---|---|
| `Pt` | `{x, y}`, with `+ - *` |
| `Cubic` | `std::array<Pt, 4>` |
| `Piece` | A cubic, or a line when `line` is set (the TS's 2- or 4-point pieces) |
| `Contour` | `std::vector<Piece>` |
| `Box` | `{lo, hi}` |
| `Affine` | `{a, b, c, d, e, f}` |
| `Path` | The TS's Path: immutable, its edits return a new one (path.hpp) |
| `Selection` | Weighted nodes of paths, and the moves on them (selection.hpp) |
| `Bool` | A boolean of operands, nested and traced lazily (bool.hpp) |
| `Mod` | A selection method as data: parameters, arguments, its call and its line of code (mods.hpp) |

Optional TS arguments are `std::optional` or default arguments, and option objects are structs (`ThroughOptions`).
Results of unknown size are `std::vector`. Inputs are `std::span`. Unions are `std::variant`, callbacks
`std::function`, and `undefined` results `std::optional`; each header lists its own mappings under "Decisions of the
port". Where a TS name is a C++ keyword it gets a trailing underscore (`Selection::and_`, `Op::union_`).

Path, Selection, analysis and attach import each other in the TS. Here their shared types live in path.hpp, and each
member is defined in the .cpp of the TS file that defines it (`Path::select` in selection.cpp, `Path::attach` in
attach.cpp).

Name `curves::apply` in full when calling it: argument-dependent lookup on the `std::array` arguments would otherwise
find `std::apply`.

## Data

How the library's values are saved as JSON (drawings, nibs, keys, cuts) is in [DATA.md](DATA.md).

## C API (`curves.h`)

The C API is the same functions, renamed in snake case with the `cv_` prefix, over plain structs whose layout matches
the C++ types:

- `cv_pt`
- `cv_cubic`
- `cv_box`
- `cv_affine`
- `cv_cubic_kind`

It follows these rules:

- It never allocates. A result of unknown size goes into the caller's `out` buffer of `cap` items. The function returns
  the full count, which can be more than `cap`. Results with a fixed maximum size take fixed arrays, for example
  `double out[3]`.
- An optional argument is a pointer, and `NULL` takes the default.
- Nothing throws across it.

The first pass covers geometry and bezier, except the helpers that take lists. The C++ API has all of them. Add to the C
API as callers need more.

## From Swift

`Package.swift` builds the sources as a SwiftPM C++ target and has two products. It needs macOS 13.3 or iOS 16.3, for
`std::to_chars` on doubles (JS number formatting).

- **`Curves`**: the whole C++ API (Path, Selection, Bool, mods, svg…) through Swift's C++ interop. A target that
  depends on it needs `swiftSettings: [.interoperabilityMode(.Cxx)]`.
- **`CCurves`**: the C API (`curves.h`) only, for a target that can't turn interop on.

To depend on it from a package, add it in `Package.swift`:

```swift
.package(path: "../../packages/cpp/curves")
```

and to the target:

```swift
.target(name: "App", dependencies: [.product(name: "Curves", package: "curves")],
        swiftSettings: [.interoperabilityMode(.Cxx)])
```

From an Xcode project, add the folder as a local package and set the target's C++ and Objective-C Interoperability
build setting to C++/Objective-C++. Then:

```swift
import Curves
let p = curves.from(curves.Pt(x: 0, y: 0)).line(curves.Pt(x: 100, y: 0)).line(curves.Pt(x: 100, y: 100))
let raised = p.select().shift(10, 90).path()                 // Selection's methods, as in the C++
let d = String(curves.svgPath(curves.parseSvgPath(std.string("M0 0L10 0L10 10Z")), 2))
```

Swift sees the namespace as `curves`, `std::string` as `std.string` (`String(…)` converts), `std::vector` as a
collection (array literals convert to it, `Array(v)` back), `std::optional` with `.init(v)` and `.value`, and a
`const&` return as a pointer (`curves.mod(std.string("rotate")).pointee`). Inside `extension curves`, `Bool` is the
C++ class: Swift's is `Swift.Bool`.

The import alone leaves gaps, which two layers fill:

- **`CurvesCxx`** (`swift/CurvesCxx/include/CurvesCxx.h`, C++ for Swift only): names for the list types Swift can't
  spell (`curves.PtList`, `ContourList`, `ContourGroups`…), for optionals (`OptInt`, `OptDoubles`…) and for unnamed
  callbacks; makers for each variant alternative (`curves.cxx.strength(0.5)`); overloads taking vectors of every
  function that takes `std::span`; overloads taking callbacks with their arguments by value where the C++ takes them
  by reference (`CornerSize`, `WindingRule`, `NodeWarp`), since Swift can't turn a closure into those; and the
  double instances of the templates with defaulted template arguments (`curves.cxx.keysAt`, `spread`, `sorted`).
- **`Curves`** (`swift/Curves/Curves.swift`) re-exports it and adds what Swift's import drops: an overload without
  the trailing arguments for each function whose defaults are taken by `const&` (option structs, `Pivot`,
  `FalloffSpec`, optionals: `curves.oval(0, 0, 100, 50)`, `sel.rotate(30)`, `path.warp { … }`), inits on the variants
  (`curves.Strength(0.5)`, `curves.NodePick([2])`, `curves.NodePick { p, i, at in … }`, `curves.Values(["deg": 30])`)
  and closures where the C++ wants a callback (`sel.moveEach { p, path, i in curves.Warp { … } }`,
  `curves.boolean([a, b]) { w in w[0] != 0 && w[1] != 0 }`, `curves.roundCorners(cs) { corner in 2 }`).

Defaults that are numbers, `Pt`s or taken by value are imported as they are. A named `std::function` typedef takes a
closure (`curves.Warp { q in … }`); an unnamed one or one taking references doesn't, hence the layers above.

With `CCurves` the C API looks like `cv_bez(cv_cubic(p: (…)), 0.5)`: C fixed arrays are tuples (`c.p.0`), output
buffers are passed with `&array`. `swift test` runs the C API smoke test and the interop tests
(`swift/CurvesSmoke`).

## From TypeScript (wasm)

The library is plain C++ with no I/O, so it compiles with emscripten as it is (`brew install emscripten`; emscripten
needs Python 3.10 or newer, so where `python3` is Xcode's 3.9, set `EMSDK_PYTHON=/opt/homebrew/bin/python3`):

```bash
emcmake cmake -B build-wasm -DCMAKE_BUILD_TYPE=Release && cmake --build build-wasm
node build-wasm/curves_tests.js      # the golden tests, in wasm
node tests/wasm/smoke.ts             # the embind API against fountain-ts
```

This writes `build-wasm/curves.js` and `curves.wasm`, an ES module with the factory `createCurves()`. `src/wasm.cpp`
binds the C++ API with [embind](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html) in the
TS's shape: points are `[x, y]`, cubics lists of four points, contours lists of pieces, what the TS reads as getters
(`length`, `nodes`, `segs`, `path`…) are properties, option objects are objects and kinds are strings.

```ts
import createCurves from 'curves/build-wasm/curves.js'
const cv = await createCurves()
const p = cv.poly([0, 0], [[100, 0], [100, 80]]).close()
const moved = p.select([1, 2]).shift(10, 90).path          // a Path, as in the TS
cv.svgPath(moved.outline(6))
cv.applyMod(p.select(), 'inflate', { r: 0.1 })         // modifiers by name, values as in the TS
p.delete(); moved.delete()                                  // wasm objects are freed by hand
```

`src/wasm.d.ts` types the module (self-contained: its own `Pt`, `Cubic`, `Contour`, Path, Selection, options…); the
build copies it to `build-wasm/curves.d.ts`, beside `curves.js`, where TS finds it. `tests/wasm/smoke.ts` type-checks
against it:

```bash
tsc --noEmit --strict --target esnext --module esnext --moduleResolution bundler --allowImportingTsExtensions \
  --skipLibCheck --types node tests/wasm/smoke.ts
```

Where the TS spreads arguments, the binding takes a list (`poly(p, ps)`). A node pick is a list of indices or a function
`(p, i, at) => weight` (or a boolean), as in the TS. Path and Selection live in the wasm heap: call `.delete()` on each
one when done with it.

Every export of the lower files is bound too (but geometry's script sugar `p`, `x`, `y`, `placeXY`), and the class `Bool` (`new cv.Bool('union', [path, contours, bool])`,
`.delete()` it too). Where they differ from the TS:

- A name two TS files export is one binding that takes either: `project`, `extrema`, `inflections`, `bbox`,
  `curvature`, `normal`, `osculating`, `mold`, `offset`, `outline`, `arcs`, `lineCrossings`, `circleCrossings` and
  `selfCrossings` give analysis's answer for a Path and bezier's, offset's or intersect's for a cubic (or a list of
  cubics for `selfCrossings`). The Path versions return Paths, to `.delete()`.
- geometry's `then` is `then_` (a module with a `then` would be taken for a promise). `union` is boolean's union.
- `stitch` returns the stitched contour rather than changing the one passed in.
- `times` and `plus` take up to four strengths. `KAPPA`, `identity` and `falloffs` are constants; `mod(name)` gives
  the mod's `{ name, group, doc, params }`, as `modList()` does.
- `caps.triangle`, `caps.rect` and `caps.arc` are `capsTriangle`, `capsRect` and `capsArc`.
- Nodes are plain `{ p, in?, out?, kind? }`: a node's other fields aren't carried through the generic node functions.
- A warp, wave or function made in C++ (`mesh`, `pinned`, `spine`, `wobble`, `wobbleWave`, `nearness`, the profile
  waves, `falloffs`, `noise2`) comes back as a JS function; one
  passed in can be any JS function. A returned one holds its C++ closure until it is garbage-collected.
- `pathOf` takes nodes or cubics: `pathOf(nodes, closed, metric?)` or `pathOf(cubics, closed?)`.

The C API (`cv_*`, listed in `CMakeLists.txt`) is exported too;
there a struct passed by value goes as a pointer into the heap, and a returned one into memory whose pointer is passed
first.

## Port status

Every file of `fountain-ts/src/curves` (3956 lines) is ported and golden-tested, 7126 cases in all:

| TS files | Cases |
|---|---:|
| geometry, bezier | 660 |
| fit, boolean, profile, offset | 568 |
| bezn, intersect, corners, warp, svg | 1273 |
| path, selection, analysis, attach | 3037 |
| nodes, spline, shape, morph, bool, mods | 1588 |

Not ported: `p`, `x`, `y` and `placeXY` from geometry.ts (script sugar).

No known differences in behaviour. One note on layout:

- `Place` is defined in selection.hpp. Move it to geometry.hpp if geometry ever needs it.

## Porting rules

- **Port the TS faithfully.** Keep the same formulas in the same order, the same names and the same doc comments, so a
  diff of behaviour is a bug in one or the other. Improve the TS first, then port.
- **Round as JS does.** The sources turn off fused multiply-adds (`#pragma STDC FP_CONTRACT OFF`, and
  `-ffp-contract=off` in CMake). With FMA, a double root came out at 0.83 instead of 2/3. The inline functions in the
  headers compile with the caller's flags, so those that multiply and add turn it off themselves (`CURVES_NO_FMA`).
- **`hypot` is V8's.** `Math.hypot` in V8 scales by the larger value and sums with Kahan compensation, which differs
  from `std::hypot` (Apple's, and musl's in wasm) in the last bit; on a symmetric span that changed which of two equal
  distances fitCubics took as the worst. Use `curves::hypot` (geometry.hpp), never `std::hypot`.
- **Test every function against golden values.** Add cases in `tests/golden/cases/<file>.ts`, regenerate, and register
  the C++ for each case name in `tests/<file>.cpp`. The tolerance is 1e-9 relative. Values the TS can't serialise
  (functions, class instances) are named by data in the cases: `tests/golden/cases/paths.ts` and `tests/paths.hpp`
  read and write paths the same way on both sides. Cases where the TS throws can't be recorded; a cases file skips
  them with a wrapper that tries the call first.
- **Keep JS's handling of edge cases.** `x || d` is `js::orElse`, `x ? :` on a number `js::truthy`, `Math.min` `js::min` (src/js.hpp); NaN and infinities pass through as they do in JS (`bounds`
  is NaN where any point is, like `Math.min`). Where they cannot, write the difference in the doc comment.
- **Keep fonts out.** Like `src/curves`, this library knows no glyphs, nibs or ink.
