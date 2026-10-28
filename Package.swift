// swift-tools-version:5.9
// math_cpp for Swift and any SwiftPM host: one library per module, so a product takes only the
// modules it needs (Docs/Foundations.md has the layers). The C++ is used through C++ interop
// (`.interoperabilityMode(.Cxx)` in the Swift target that imports it).
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
  // The oldest systems it is built and tested for.
  platforms: [.macOS("13.3"), .iOS("16.3")],
  products: [.library(name: "MathKernels", targets: ["MathKernels"])]
    + modules.map { .library(name: "Math" + $0.0, targets: ["Math" + $0.0]) },
  targets: [
    // Header-only: the one source in Swift/MathKernels includes the kernels so the target builds.
    .target(
      name: "MathKernels",
      path: ".",
      exclude: ["Tests", "Tools", "Docs", "Swift/LinearSmoke", "Swift/FieldsSmoke", "README.md", "AGENTS.md", "LICENSE", "Makefile", ".gitignore"]
        + modules.map { $0.0 },
      sources: ["Swift/MathKernels"],
      publicHeadersPath: "Kernels",
      cxxSettings: deterministic
    ),
  ]
    + modules.map { module($0.0, $0.1) }
    + [
      .testTarget(
        name: "LinearSmoke", dependencies: ["MathLinear"], path: "Swift/LinearSmoke",
        swiftSettings: [.interoperabilityMode(.Cxx)]
      ),
      .testTarget(
        name: "FieldsSmoke", dependencies: ["MathFields"], path: "Swift/FieldsSmoke",
        swiftSettings: [.interoperabilityMode(.Cxx)]
      ),
    ],
  cxxLanguageStandard: .cxx20
)
