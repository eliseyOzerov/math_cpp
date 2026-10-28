// swift-tools-version:5.9
// curves for Swift: the C API (include/curves.h) as the module CCurves; the C++ API as the module CurvesCxx (the
// headers through C++ interop, plus swift/CurvesCxx/include/CurvesCxx.h), re-exported by the Swift module Curves with
// the default arguments and variant inits Swift's import drops. See README.md, "From Swift".
import PackageDescription

let package = Package(
  name: "graphics_cpp",
  // std::to_chars for doubles, which formats numbers as JS does, needs macOS 13.3.
  platforms: [.macOS("13.3"), .iOS("16.3")],
  products: [
    .library(name: "CCurves", targets: ["CCurves"]),
    .library(name: "Curves", targets: ["Curves"]),
  ],
  targets: [
    .target(
      name: "CCurves",
      path: ".",
      exclude: ["tests", "swift", "build", "README.md", "CMakeLists.txt", "LICENSE", "THIRD_PARTY_NOTICES.md"],
      sources: ["src"],
      publicHeadersPath: "include",
      // Optimized in debug builds too: callers' hot loops (a font's ink) run ~10× slower at -O0.
      cxxSettings: [.unsafeFlags(["-O3"], .when(configuration: .debug))]
    ),
    .target(name: "CurvesCxx", dependencies: ["CCurves"], path: "swift/CurvesCxx"),
    .target(
      name: "Curves", dependencies: ["CurvesCxx"], path: "swift/Curves",
      swiftSettings: [.interoperabilityMode(.Cxx)]
    ),
    .testTarget(name: "CCurvesSmoke", dependencies: ["CCurves"], path: "swift/Smoke"),
    .testTarget(
      name: "CurvesSmoke", dependencies: ["Curves"], path: "swift/CurvesSmoke",
      swiftSettings: [.interoperabilityMode(.Cxx)]
    ),
  ],
  cxxLanguageStandard: .cxx20
)
