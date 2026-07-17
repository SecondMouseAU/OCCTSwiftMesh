# OCCTSwiftMesh

[![Swift](https://img.shields.io/endpoint?url=https%3A%2F%2Fswiftpackageindex.com%2Fapi%2Fpackages%2FSecondMouseAU%2FOCCTSwiftMesh%2Fbadge%3Ftype%3Dswift-versions)](https://swiftpackageindex.com/SecondMouseAU/OCCTSwiftMesh)
[![Platforms](https://img.shields.io/endpoint?url=https%3A%2F%2Fswiftpackageindex.com%2Fapi%2Fpackages%2FSecondMouseAU%2FOCCTSwiftMesh%2Fbadge%3Ftype%3Dplatforms)](https://swiftpackageindex.com/SecondMouseAU/OCCTSwiftMesh)

Mesh-domain algorithms for the [OCCTSwift](https://github.com/SecondMouseAU/OCCTSwift) ecosystem. Operates on `OCCTSwift.Mesh` instances; complements the OCCT-side topology kernel rather than extending it.

Part of the [OCCTSwift ecosystem](https://github.com/SecondMouseAU/OCCTSwift/blob/main/docs/ecosystem.md) — see the ecosystem map for how this package fits with the kernel, viewport, and sibling layers.

```
OCCTSwift           — B-Rep solid modelling kernel (wraps OpenCASCADE)
OCCTSwiftMesh       — mesh-domain algorithms (decimation, smoothing, repair, ...)
OCCTSwiftViewport   — Metal viewport for rendering
OCCTSwiftScripts    — script harness + occtkit CLI verbs
```

## Why a separate package

OCCT's open-source distribution provides `BRepMesh_*` for mesh **generation** but no **decimation, simplification, smoothing, hole-filling, remeshing, or other mesh-side post-processing**. OCCT-Components ships a paywalled "Mesh Decimation" module — this package fills the same role with permissive, vendored implementations.

OCCTSwift itself stays focused on its mission as an OCCT wrapper. Mesh algorithms that happen to consume OCCT-produced meshes live here.

## Status

✅ **v1.1.0** — SemVer-stable. Ships `Mesh.simplified(_:)` (decimation, vendored [meshoptimizer](https://github.com/zeux/meshoptimizer) v1.1) and `Mesh.crossSection(plane:)` (planar slicing into closed contours). Requires OCCTSwift v1.0.1 or later. See [docs/CHANGELOG.md](docs/CHANGELOG.md).

## API

### Decimation — `Mesh.simplified(_:)`

```swift
import OCCTSwift
import OCCTSwiftMesh

let mesh: Mesh = shape.mesh()  // from OCCTSwift
let simplified = mesh.simplified(.init(
    targetTriangleCount: 5_000,
    preserveBoundary: true,
    preserveTopology: true
))

if let result = simplified {
    print("\(result.beforeTriangleCount) → \(result.afterTriangleCount)")
    print("Hausdorff: \(result.hausdorffDistance)")
    let reducedMesh = result.mesh
}
```

### Slicing — `Mesh.crossSection(plane:)`

Intersect a mesh with a plane and recover the closed contours where it cuts the
surface — the perimeter step a 3D-printer slicer performs. Works directly on
**open / unwelded** scan meshes (no B-Rep sewing first). A thin-walled tube
slices into separate outer and inner loops, so wall thickness is just their
offset; inner-vs-outer comes from contour nesting, not triangle winding.

```swift
let section = mesh.crossSection(plane: CutPlane(point: p, normal: n))
for c in section!.contours {
    // c.depth == 0 → outer solid boundary; c.isHole → inner wall / pocket
    print(c.points.count, "pts, area", c.area, c.isHole ? "(hole)" : "")
}

// Or a whole slicer layer stack along an axis:
let stack = mesh.crossSections(axis: axis, through: p, spacing: 2.0)
```

## Installation

```swift
// Package.swift
dependencies: [
    .package(url: "https://github.com/SecondMouseAU/OCCTSwiftMesh.git", from: "1.1.2"),
],
targets: [
    .target(
        name: "YourApp",
        dependencies: [
            .product(name: "OCCTSwiftMesh", package: "OCCTSwiftMesh"),
        ]
    )
]
```

## License

LGPL-2.1, matching OCCTSwift. Vendored components retain their own permissive licenses (notably meshoptimizer under MIT). See [NOTICE.md](NOTICE.md).

## Roadmap

Beyond initial decimation:

- Subdivision (Catmull-Clark, Loop)
- Laplacian / Taubin smoothing
- Mesh repair (non-manifold cleanup, hole filling)
- Remeshing (uniform / adaptive)
- glTF mesh-export niceties (LOD chains, meshopt-encoded streams)
- GPU-accelerated mesh ops where worthwhile

Community needs drive priority — file an issue if you want one of these (or something else) sooner.

## Related projects

- [OCCTSwift](https://github.com/SecondMouseAU/OCCTSwift) — OCCT wrapper, source of `Mesh`
- [OCCTSwiftScripts](https://github.com/SecondMouseAU/OCCTSwiftScripts) — script harness; `simplify-mesh` verb consumes this package ([#22](https://github.com/SecondMouseAU/OCCTSwiftScripts/issues/22))
- [OCCTMCP](https://github.com/SecondMouseAU/OCCTMCP) — MCP server; `simplify_mesh` tool consumes this package via OCCTSwiftScripts ([#6](https://github.com/SecondMouseAU/OCCTMCP/issues/6))
