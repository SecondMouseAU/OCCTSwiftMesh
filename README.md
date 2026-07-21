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

✅ **v1.2.0** — SemVer-stable. Ships `Mesh.simplified(_:)` (decimation, vendored [meshoptimizer](https://github.com/zeux/meshoptimizer) v1.1), `Mesh.crossSection(plane:)` (planar slicing into closed contours), mesh foundations (weld / normals / adjacency / components / boundary loops / integrity report), and `Mesh.segmented(_:)` (dihedral region-growing + primitive-fit merge). Requires OCCTSwift v1.12.9 or later. See [docs/CHANGELOG.md](docs/CHANGELOG.md).

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

### Mesh foundations — weld, adjacency, components, integrity

```swift
let welded = mesh.welded()                          // grid-hash vertex merge; everything below needs this first
let normals = welded.faceNormals()
let adjacency = welded.triangleAdjacency()
let pieces = welded.connectedComponents()            // largest-first, deterministic
let loops = welded.boundaryLoops()                   // closed rings of open (1-triangle) edges
let sub = welded.subMesh(triangleIndices: [0, 1, 2])

let report = welded.integrityReport()
print(report.isWatertight, report.boundaryLoopCount, report.components.count)
```

### Region segmentation — `Mesh.segmented(_:)`

Dihedral region-growing (breaks at sharp face-normal changes) followed by a primitive-fit merge
pass — adjacent regions merge back together when their union still fits ONE analytic surface
(plane / cylinder / sphere / cone). Without the merge pass, a coarsely tessellated curved surface
(e.g. a 12-facet cylinder) shatters into one region per facet; with it, every region arrives with
a fitted primitive for free.

```swift
let result = welded.segmented(.init(maxDihedralDegrees: 20, maxRegions: 64))
for region in result.regions {
    print(region.triangleIndices.count, "tris,", region.area, "area,", region.boundaryLoopCount, "loops")
}
for fit in result.fits {
    print(fit.kind, fit.residualRMS)   // e.g. .cylinder, 0.03
}
if result.truncated { print("capped at maxRegions") }
```

Deterministic: two calls on identical (welded) input return byte-identical output.

## Installation

```swift
// Package.swift
dependencies: [
    .package(url: "https://github.com/SecondMouseAU/OCCTSwiftMesh.git", from: "1.2.0"),
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
