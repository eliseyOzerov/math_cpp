// swift-tools-version:5.9
// math_cpp for Swift and any SwiftPM host: one library per module, so a product takes only the
// modules it needs (Docs/Foundations.md has the layers). The C++ is used through C++ interop
// (`.interoperabilityMode(.Cxx)` in the Swift target that imports it). Curves2D, the double-precision
// 2D curves ported from fountain-ts, keeps its own products CCurves and Curves.
import PackageDescription

/// -ffp-contract=off: no fused multiply-adds, so results agree to the last bit on every machine.
/// Unsafe flags are allowed in local (path) dependencies and in the root package.
let deterministic: [CXXSetting] = [.unsafeFlags(["-ffp-contract=off"])]

/// A module folder as a C++ target, its headers public, depending on the modules it includes.
func module(_ name: String, _ dependencies: [String] = []) -> Target {
  .target(
    name: "Math" + name,
    dependencies: dependencies.map { Target.Dependency(stringLiteral: "Math" + $0) },
    path: name,
    publicHeadersPath: ".",
    cxxSettings: deterministic
  )
}

let modules: [(String, [String])] = [
  ("Linear", ["Kernels"]),
  ("Numerics", ["Linear"]),
  ("Color", ["Kernels", "Linear"]),
  ("Geometry", ["Linear"]),
  ("Curves", ["Geometry", "Numerics", "Linear"]),
  ("Mesh", ["Curves", "Geometry", "Linear"]),
  ("Fields", ["Mesh", "Curves", "Geometry", "Numerics", "Linear"]),
  ("Optics", ["Color", "Geometry", "Linear"]),
  ("Physics", ["Linear"]),
  ("Animation", ["Linear"]),
  ("Shaders", []),
]

let package = Package(
  name: "math_cpp",
  // std::to_chars for doubles (Curves2D formats numbers as JS does) needs macOS 13.3.
  platforms: [.macOS("13.3"), .iOS("16.3")],
  products: [.library(name: "MathKernels", targets: ["MathKernels"])]
    + modules.map { .library(name: "Math" + $0.0, targets: ["Math" + $0.0]) }
    + [
      .library(name: "CCurves", targets: ["CCurves"]),
      .library(name: "Curves", targets: ["Curves"]),
    ],
  targets: [
    // Header-only: the one source in Swift/MathKernels includes the kernels so the target builds.
    .target(
      name: "MathKernels",
      path: ".",
      exclude: ["Tests", "Tools", "Docs", "Curves2D", "Swift/LinearSmoke", "Swift/FieldsSmoke", "README.md", "AGENTS.md", "LICENSE", "Makefile", ".gitignore"]
        + modules.map { $0.0 },
      sources: ["Swift/MathKernels"],
      publicHeadersPath: "Kernels",
      cxxSettings: deterministic
    ),
  ]
    + modules.map { module($0.0, $0.1) }
    + [
      .target(
        name: "CCurves",
        path: "Curves2D",
        exclude: ["tests", "swift", "src/wasm.d.ts", "README.md", "DATA.md", "CMakeLists.txt", "LICENSE", "THIRD_PARTY_NOTICES.md"],
        sources: ["src"],
        publicHeadersPath: "include",
        // Optimized in debug builds too: callers' hot loops (a font's ink) run ~10× slower at -O0.
        cxxSettings: [.unsafeFlags(["-O3"], .when(configuration: .debug))]
      ),
      .target(name: "CurvesCxx", dependencies: ["CCurves"], path: "Curves2D/swift/CurvesCxx"),
      .target(
        name: "Curves", dependencies: ["CurvesCxx"], path: "Curves2D/swift/Curves",
        swiftSettings: [.interoperabilityMode(.Cxx)]
      ),
      .testTarget(
        name: "LinearSmoke", dependencies: ["MathLinear"], path: "Swift/LinearSmoke",
        swiftSettings: [.interoperabilityMode(.Cxx)]
      ),
      .testTarget(
        name: "FieldsSmoke", dependencies: ["MathFields"], path: "Swift/FieldsSmoke",
        swiftSettings: [.interoperabilityMode(.Cxx)]
      ),
      .testTarget(name: "CCurvesSmoke", dependencies: ["CCurves"], path: "Curves2D/swift/Smoke"),
      .testTarget(
        name: "CurvesSmoke", dependencies: ["Curves"], path: "Curves2D/swift/CurvesSmoke",
        swiftSettings: [.interoperabilityMode(.Cxx)]
      ),
    ],
  cxxLanguageStandard: .cxx20
)
