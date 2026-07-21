---
title: Home
nav_order: 1
---

# OCCTSwiftMesh documentation

Mesh-domain algorithms for the [OCCTSwift](https://github.com/SecondMouseAU/OCCTSwift) ecosystem.
OCCTSwiftMesh operates on the `Mesh` triangle-soup value type (declared in `OCCTSwift` and
re-exported here) and adds the mesh-side post-processing OCCT's open-source distribution
leaves out: **quadric-error-metric (QEM) decimation** that reports its achieved Hausdorff
error, **planar cross-sectioning** that recovers a slice's closed contours the way a
3D-printer slicer does, **mesh foundations** (weld / normals / adjacency / connected components /
boundary loops / an integrity check-list), and **region segmentation** (dihedral region-growing +
primitive-fit merge, so every region arrives with a fitted plane/cylinder/sphere/cone for free).
Pure-Swift algorithms over `Mesh.vertices` / `Mesh.indices` — no extra OCCT kernel calls — so they
stay robust on the open, unwelded meshes that raw STL / scan bodies actually are.

```swift
import OCCTSwift
import OCCTSwiftMesh

let mesh: Mesh = shape.mesh()               // a tessellated Shape from OCCTSwift
guard let result = mesh.simplified(.init(targetTriangleCount: 5_000)) else { return }
print("\(result.beforeTriangleCount) → \(result.afterTriangleCount) triangles")
print("Hausdorff error: \(result.hausdorffDistance)")   // absolute, in input mesh units
let reduced: Mesh = result.mesh
```

## Cookbook

Task-oriented, example-rich guides — each a short bit of prose plus runnable Swift. The
**[Cookbook index](guides/cookbook/)** lists all recipes:

[Decimating a Mesh](guides/cookbook/decimating-a-mesh.md) ·
[Target Count vs. Reduction Ratio](guides/cookbook/target-count-vs-ratio.md) ·
[Cross-Sections & Contours](guides/cookbook/cross-sections.md)

## Reference

- **[API Reference](reference/)** — the detailed, per-type reference: signatures, parameters,
  return values, and runnable examples for every public type (decimation and cross-section types
  only as of v1.2.0 — the foundations/segmentation types below are documented in the algorithm
  note, README, and in-source doc comments; per-type reference pages are a follow-up).
- [Decimation algorithm notes](algorithms/decimation.md) — the QEM backend, vendored
  meshoptimizer, Hausdorff units, and edge-case semantics.
- [Mesh foundations + segmentation notes](algorithms/segmentation.md) — weld/adjacency/components/
  integrity, the dihedral region-growing + primitive-fit merge pass, and the determinism
  guarantees `Mesh.segmented(_:)` makes.
- [Changelog](CHANGELOG.md) — release-by-release history.
- [Vendoring](VENDORING.md) — the re-vendoring procedure for the bundled meshoptimizer.

## Project

OCCTSwiftMesh is part of the OCCTSwift family. It depends on `OCCTSwift` for the `Mesh` type
and vendors [meshoptimizer](https://github.com/zeux/meshoptimizer) (MIT) for the QEM backend.
Install via Swift Package Manager:

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

- Source & issues: [github.com/SecondMouseAU/OCCTSwiftMesh](https://github.com/SecondMouseAU/OCCTSwiftMesh)
- Platforms: macOS 12+, iOS 15+. LGPL-2.1, matching OCCTSwift.
