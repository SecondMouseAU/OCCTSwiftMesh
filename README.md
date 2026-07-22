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

✅ **v1.6.0** — SemVer-stable. Ships `Mesh.simplified(_:)` (decimation, vendored [meshoptimizer](https://github.com/zeux/meshoptimizer) v1.1), `Mesh.crossSection(plane:)` (planar slicing into closed contours), the mesh connectivity toolkit (`welded`, `faceNormals`, `vertexNormals`, `triangleAdjacency`, `connectedComponents`, `subMesh`, `boundaryLoops`, `integrityReport`), `Mesh.segmented(_:)` (dihedral region-growing + primitive-fit merge), `Mesh.vertexCurvatures()` (Rusinkiewicz per-face curvature tensor), `Mesh.aligned(to:options:)` (point-to-plane ICP registration), and `Mesh.slippage(forTriangles:maxSamples:)` (Gelfand-Guibas slippage analysis). Requires OCCTSwift v1.12.9 or later. See [docs/CHANGELOG.md](docs/CHANGELOG.md).

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

### Mesh foundations — weld, connectivity, integrity

Raw OCCT tessellation and STL import both produce (near-)unshared vertices — three unique
positions per triangle even where triangles are geometrically edge-adjacent. `welded(tolerance:)`
merges coincident vertices; every adjacency-based operation below needs that welded substrate to
see real connectivity.

```swift
let welded = mesh.welded()                 // 0 auto-derives 1e-6 × the bbox diagonal

welded.faceNormals()                       // [SIMD3<Float>], one per triangle
welded.vertexNormals()                     // area-weighted, per vertex
welded.triangleAdjacency()                 // [[Int]] — edge-adjacent triangles
welded.connectedComponents()               // [MeshRegion], largest-first
welded.subMesh(triangleIndices: [0, 1])    // extract a compact standalone Mesh
welded.boundaryLoops()                     // [[UInt32]] — open-edge rings

let report = mesh.integrityReport()        // welds internally — safe to call on raw input
print(report.isWatertight, report.nonManifoldEdgeCount, report.boundaryLoopCount)
print(report.eulerCharacteristic, report.genus as Any)
```

### Segmentation — `Mesh.segmented(_:)`

Dihedral region-growing splits a mesh into smoothly-connected surface patches, then a
primitive-fit merge pass undoes coarse-tessellation "confetti" (a low-poly cylinder's facets,
each past the dihedral threshold, growing back into one cylinder + end caps). Welds internally,
so unwelded input doesn't silently degrade to one region per triangle.

```swift
let segmented = mesh.segmented()           // Mesh.SegmentOptions() defaults
for (region, fit) in zip(segmented.regions, segmented.fits) {
    print(region.triangleIndices.count, "triangles →", fit.kind, fit.residualRMS)
}
// segmented.truncatedTriangleCount reports anything dropped by maxRegions / minRegionTriangles —
// never silent. segmented.fitMergeSkipped reports when the fit-gated merge pass itself couldn't
// run (raw region count still over an internal cap after the cheap coplanar pre-merge) — also
// never silent.
```

### Alignment — `Mesh.aligned(to:options:)`

Point-to-plane ICP registration: recovers the rigid transform that best aligns this (source) mesh
onto a reference mesh. PCA/bbox pre-align (tries all 4 sign-valid orientations, keeps the best)
runs before the iterative refinement; normal-space sampling (on by default) keeps a small
feature's rare normal direction from being drowned out by a flat majority; trimmed correspondence
handles partial overlap between the two meshes.

```swift
if let result = scan.aligned(to: cad) {
    print(result.transform)       // simd_double4x4 — maps scan's original vertices into cad's frame
    print(result.residualRMS, result.iterations, result.converged)
}
### Curvature — `Mesh.vertexCurvatures()`

Per-vertex principal curvatures and directions (Rusinkiewicz per-face tensor method, chosen over
Meyer's cotan-Laplacian for robustness on noisy/irregular tessellation — no obtuse-triangle
clamp anywhere in the pipeline). Requires welded input, same precondition as `triangleAdjacency()`
/`connectedComponents()`.

```swift
let welded = mesh.welded()
for c in welded.vertexCurvatures() {
    print(c.k1, c.k2, c.mean, c.gaussian)  // c.d1/c.d2 — unit principal directions
}
// k1 is signed positive for a convex bulge (e.g. an outward-facing sphere), matching
// vertexNormals()'s own outward-normal convention. A face too degenerate/sliver-thin for a
// stable fit is excluded from it entirely; a vertex touched only by such faces reports
// k1 == k2 == 0, never NaN.
```

### Slippage analysis — `Mesh.slippage(forTriangles:maxSamples:)`

Classifies a segmented region's surface kind (plane / sphere / cylinder / extrusion / revolution /
helix / freeform) and recovers its characteristic axis, by local slippage analysis (Gelfand &
Guibas, SGP 2004). Like `triangleAdjacency()`, operates on this mesh's own vertex/normal arrays —
pass an already-welded mesh.

```swift
let result = mesh.slippage(forTriangles: region.triangleIndices)
print(result.kind, result.axisPoint as Any, result.axisDirection as Any)
// .helix additionally sets result.pitch (translation per radian of rotation)
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
