// swift-tools-version: 6.1
import PackageDescription

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
        .package(url: "https://github.com/gsdali/OCCTSwift.git", from: "1.7.1"),
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
