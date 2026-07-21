---
type: component
title: Components index
resource: https://github.com/SecondMouseAU/OCCTSwiftMesh
tags: [index]
description: Public modules / API surfaces exposed by OCCTSwiftMesh.
timestamp: 2026-06-22
---

# Components

Public products and targets from `Package.swift`, plus the key API surfaces from the README.

- **`OCCTSwiftMesh`** (library product / target) — the public Swift 6 API. Extends OCCTSwift's
  `Mesh` with mesh-domain operations.
  - `Mesh.simplified(_:)` — decimation/simplification to a target triangle count, with
    boundary/topology preservation options; returns before/after triangle counts and Hausdorff
    distance alongside the reduced `Mesh`.
  - `Mesh.crossSection(plane:)` / `Mesh.crossSections(axis:through:spacing:)` — planar slicing of a
    (possibly open/unwelded) mesh into closed contours; the perimeter step a 3D-printer slicer
    performs. Inner-vs-outer is recovered from contour nesting via `contour.depth` / `contour.isHole`.
  - Mesh foundations — `Mesh.welded(tolerance:)`, `faceNormals()`, `vertexNormals()`,
    `triangleAdjacency()`, `connectedComponents()`, `subMesh(triangleIndices:)`,
    `boundaryLoops()`, `integrityReport()`. Adjacency-based operations need a welded mesh;
    `integrityReport()` welds internally. Pure Swift + simd, no OCCT kernel calls.
  - `Mesh.segmented(_:)` — dihedral region-growing + primitive-fit merge, splitting a mesh into
    plane/cylinder/sphere/cone surface regions (`MeshRegion` + `FittedPrimitive` pairs via
    `SegmentedMesh`). Welds internally; depends on the mesh foundations layer.
- **`OCCTMeshOptimizer`** (internal C++ bridge target) — vendors meshoptimizer (MIT) and exposes a
  small C ABI consumed by the Swift layer. Not a public product.
