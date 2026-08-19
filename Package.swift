// swift-tools-version: 6.1
import PackageDescription
import Foundation

// Prefer a local sibling checkout (../<name>) when present, else the published URL — so the whole
// OCCT ecosystem SHARES the single OCCTSwift/Libraries/OCCT.xcframework instead of each repo
// extracting its own 1.3 GB copy. CI / fresh clones (no sibling) use the URL pin. `#filePath`-relative
// so it's independent of build CWD.
func occtDep(_ name: String, from version: String) -> Package.Dependency {
    let manifestDir = URL(fileURLWithPath: #filePath).deletingLastPathComponent().path
    // Only trust a sibling checkout for a REAL local dev clone — never when this manifest is itself a
    // transitively-resolved checkout under a consumer's `.build/checkouts/` (SwiftPM lays every dep out
    // flat there, so `../\(name)` spuriously exists and flips this to a path dep → a SwiftPM identity
    // conflict with the URL-based dep. See SecondMouseAU/ecosystem#14.
    if !manifestDir.contains("/.build/"),
       FileManager.default.fileExists(atPath: manifestDir + "/../\(name)/Package.swift") {
        return .package(path: "../\(name)")
    }
    return .package(url: "https://github.com/SecondMouseAU/\(name).git", from: Version(version)!)
}

let package = Package(
    name: "OCCTSwiftMesh",
    platforms: [
        .iOS(.v15),
        .macOS(.v12)
    ],
    products: [
        .library(
            name: "OCCTSwiftMesh",
            targets: ["OCCTSwiftMesh"]
        ),
    ],
    dependencies: [
        // SemVer-stable from OCCTSwift v1.0.0 (OCCT 8.0.0 GA, 2026-05-07).
        // v0.156.2 was the original pin — first release exposing the public
        // Mesh(vertices:normals:indices:) initializer (OCCTSwift#94) that
        // Mesh.simplified(_:) needs to wrap its raw output. v1.0.x preserves it.
        // Floored at 1.7.1 for OCCT 8.0.0p1 (redesigned BRepGraph/TopologyGraph).
        occtDep("OCCTSwift", from: "3.0.0"),   // ≥3.0.0: Rule 2 major (SEMVER.md#v300, issue #44); OCCT itself does not move (still 8.0.1, kernel rebuilt as v3.0.0-kernel.1 to carry two patches the v2.0.0 asset was missing, OCCTSwift#905/#913). Three breaks, all compile errors, none reachable from this package: (a) Selector.SubShapeType.compsolid renamed .compSolid and (b) Shape.ShapeFilterType.RawValue moving Int32 to Int are both zero-hit here (grepped Sources, Tests, README and docs); (c) Shape.bounds/.size/.center, Wire.bounds, Edge.bounds and Face.bounds/.exactBounds becoming Optional (OCCTSwift#943, the break that actually bites downstream) touches nothing here, because this package never reads a bounding box off an OCCT type: its whole contact surface with OCCT topology is one test fixture calling Shape.sphere(radius:) then .mesh(linearDeflection:angularDeflection:). The 40 grep candidates counted at filing time are MeshContour.bounds (this package's own 2D contour type, non-Optional and unaffected) plus .center/.size struct fields inside the vendored meshoptimizer C++. Re-audited from scratch for v3.0.0 rather than inherited from the v2.0.0 note below, which covered sub-shape enumeration and mass properties and says nothing about the bounds accessors; zero source changes, green swift build and swift test against the real v3.0.0 sibling checkout; ≥2.0.0: correctness release (issue #39) — OCCT re-pinned to 8.0.1, 17 breaking Swift API changes (SEMVER.md#v200), none reachable from this package: it never calls Shape.faces()/.edges()/.vertices(), Mesh.Triangle.faceIndex/trianglesWithFaces(), meshTriangleAdjacency/meshNodeTriangle*, AAG, or any mass-property/centerOfMass accessor — audited by grep across Sources/Tests plus a green swift build/test against the real v2.0.0 sibling checkout, zero source changes needed; ≥1.17.0: Pass 1a duplication/bug-fix audit (OCCTSwift#377/#380) — continuity enum consolidation (source-compatible via deprecated aliases), Surface.drawMesh/evaluateGrid now return SurfaceGrid (not used here); ≥1.15.10: accumulated kernel/bridge crash + reentrancy fixes through #349 (patches 0003-0018); no API changes vs 1.12.9
    ],
    targets: [
        // Public Swift API: Mesh.simplified(_:) and friends.
        .target(
            name: "OCCTSwiftMesh",
            dependencies: [
                .product(name: "OCCTSwift", package: "OCCTSwift"),
                "OCCTMeshOptimizer",
            ],
            path: "Sources/OCCTSwiftMesh",
            swiftSettings: [
                .swiftLanguageMode(.v6)
            ]
        ),

        // C++ bridge target that vendors meshoptimizer (MIT).
        // All of meshoptimizer's .cpp files compile here; the wrapper
        // exposes a small C ABI to the Swift layer.
        .target(
            name: "OCCTMeshOptimizer",
            path: "Sources/OCCTMeshOptimizer",
            exclude: [
                "src/README.md",
                "src/meshoptimizer/LICENSE.md",
            ],
            sources: ["src"],
            publicHeadersPath: "include",
            cxxSettings: [
                .define("MESHOPTIMIZER_NO_EXPERIMENTAL", to: "0")
            ],
            linkerSettings: [
                .linkedLibrary("c++")
            ]
        ),

        // Tests
        .testTarget(
            name: "OCCTSwiftMeshTests",
            dependencies: ["OCCTSwiftMesh", "OCCTMeshOptimizer"],
            path: "Tests/OCCTSwiftMeshTests"
        ),
    ],
    cxxLanguageStandard: .cxx17
)
