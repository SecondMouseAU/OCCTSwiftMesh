# Changelog

All notable changes to OCCTSwiftMesh.

## v1.2.0 — mesh foundations + region segmentation

Adds the mesh connectivity toolkit ([#16](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/16))
and dihedral region-growing + primitive-fit segmentation
([#17](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/17)) — both pure Swift + simd, no
OCCT kernel calls, ported from OCCTReconstruct's `ReconstructCompute` reference implementation.

**Mesh foundations** (`Mesh+Welding.swift`, `Mesh+Topology.swift`, `MeshIntegrityReport.swift`):

```swift
let welded = mesh.welded(tolerance: 0)        // 0 auto-derives 1e-6 × bbox diagonal
welded.faceNormals()                          // [SIMD3<Float>], one per triangle
welded.vertexNormals()                        // area-weighted, per vertex
welded.triangleAdjacency()                    // [[Int]] — edge-adjacent triangles
welded.connectedComponents()                  // [MeshRegion], largest-first
welded.subMesh(triangleIndices:)               // extract a compact standalone Mesh, or nil
welded.boundaryLoops()                        // [[UInt32]] — open-edge rings
mesh.integrityReport()                        // MeshIntegrityReport — welds internally
```

- `triangleAdjacency()`, `connectedComponents()`, and `boundaryLoops()` key connectivity off
  shared vertex INDICES — they need welded input. Raw OCCT tessellation and STL import produce
  per-triangle-unique vertices (no two triangles share an index at all), so call `.welded()`
  first or every triangle comes back isolated. This is a deliberate, documented precondition, not
  an oversight — see each method's doc comment.
- `integrityReport()` welds internally (so it's safe to call on raw input directly) but counts
  `duplicateTriangleCount` / `degenerateTriangleCount` from the RAW welded topology before
  cleanup, while every other metric (manifoldness, Euler characteristic, genus, components,
  sliver signals) is computed on the deduplicated, non-degenerate topology — so a handful of
  exact duplicate faces doesn't masquerade as a real non-manifold defect.
- Semantics follow the Open3D `TriangleMesh` conventions (`is_edge_manifold` /
  `is_vertex_manifold` / `is_watertight` / `cluster_connected_triangles`).
- `genus` is `nil` unless the mesh is watertight and orientable; computed additively across all
  closed components (`genus = components.count - eulerCharacteristic / 2`).

**Segmentation** (`Mesh+Segmentation.swift`, `PrimitiveFitter.swift`, `FittedPrimitive.swift`):

```swift
let segmented = mesh.segmented(.init(maxDihedralDegrees: 20))
for (region, fit) in zip(segmented.regions, segmented.fits) {
    print(region.triangleIndices.count, fit.kind, fit.residualRMS)
}
```

- Region-growing breaks at sharp edges (`maxDihedralDegrees`, default 20°); the merge pass then
  greedily unions adjacent regions whose union still fits a single plane/cylinder/sphere/cone
  within `mergeRelativeTolerance × bboxDiagonal`, undoing coarse-tessellation "confetti" (a
  12-facet cylinder's facets, each past the dihedral threshold, merge back into one cylinder).
  A cube's 90° corners are correctly left unmerged.
- Welds internally (`SegmentOptions.weldTolerance`) — unwelded input does not silently degrade to
  one region per triangle; every `MeshRegion.triangleIndices` still refers to the input mesh's
  own (possibly unwelded) triangle order.
- A cheap coplanar pre-merge (fit-free, ~2° threshold) runs before the fit-gated pass whenever
  the raw region count exceeds an internal cap (1500, matching the OCCTReconstruct reference),
  keeping segmentation usable at 400k+ triangle scan meshes.
- `SegmentOptions.maxRegions` / `minRegionTriangles` never truncate silently:
  `SegmentedMesh.truncatedTriangleCount` reports exactly how many triangles were dropped and why.
- Determinism is load-bearing, not incidental: region composition, merge order, and boundary-loop
  chaining all have explicit tie-breaks (documented inline) so two runs on identical input
  produce byte-identical output — Dictionary/Set iteration order is NOT a substitute.
- Out of scope for this release (tracked as follow-ups per #17): curvature-based seeding, the
  Schnabel-style RANSAC alternative strategy + auto-selection bake-off, and slippage analysis.

New public types: `MeshRegion`, `MeshIntegrityReport`, `Mesh.SegmentOptions`, `SegmentedMesh`,
`FittedPrimitive` (+ nested `FittedPrimitive.Kind`).

## v1.1.6 — repin OCCTSwift 1.12.9 (#318 and #323 crash/hang fixes)

Repin the OCCTSwift floor to **1.12.9**, which carries kernel patch 0006 (a `BRepGProp_EdgeTool` null-curve-on-surface guard, [OCCTSwift#318](https://github.com/SecondMouseAU/OCCTSwift/issues/318)) and patches 0007 through 0009 (free-bounds `lwire` reset, boolean-path BSpline O(1) periodic normalization, STEP-writer oversized-string split; [OCCTSwift#323](https://github.com/SecondMouseAU/OCCTSwift/issues/323)), on top of the earlier patches. No API or behaviour change.

## v1.1.5 — repin OCCTSwift 1.12.7 (ShapeFix_Face null-context crash fix)

Repin the OCCTSwift floor to **1.12.7**, which carries OCCT kernel patch 0005: `ShapeFix_Face::FixPeriodicDegenerated` guards a null `Context()`, fixing the SIGSEGV in [OCCTSwift#317](https://github.com/SecondMouseAU/OCCTSwift/issues/317) (upstream [OCCT#1380](https://github.com/Open-Cascade-SAS/OCCT/pull/1380)), on top of the free-bounds (#310) and fillet (#298) patches. No API or behaviour change.

## v1.1.4 — repin OCCTSwift 1.12.6 (free-bounds crash fix)

Repin the OCCTSwift floor to **1.12.6**, which carries OCCT kernel patch 0004 — `ShapeAnalysis_FreeBounds` no longer returns a null `owires` on empty input, fixing the uncatchable free-bounds SIGSEGV ([OCCTSwift#310](https://github.com/SecondMouseAU/OCCTSwift/issues/310), upstream [OCCT#1377](https://github.com/Open-Cascade-SAS/OCCT/pull/1377)) — on top of the thread-safe-fillet patch (#298). No API or behaviour change.

## v1.1.3 — repin OCCTSwift 1.12.3 (thread-safe fillet)

Repin the OCCTSwift floor to **1.12.3**, which carries OCCT kernel patch 0003 making 3D fillet/chamfer reentrant across threads ([OCCTSwift#298](https://github.com/SecondMouseAU/OCCTSwift/issues/298) / upstream [OCCT#1374](https://github.com/Open-Cascade-SAS/OCCT/pull/1374)). Ecosystem-wide floor bump; no API or behaviour change.

## v1.1.0 — `Mesh.crossSection(plane:)` planar slicing

Adds a mesh **slicer**: intersect a mesh with a plane and recover the closed
contours where it cuts the surface — the perimeter step a 3D-printer slicer
performs. Pure geometry (no OCCT kernel calls), so it works directly on the
**open and unwelded** meshes that raw STL/scan bodies actually are, where sewing
to a B-Rep first would fail.

```swift
let section = mesh.crossSection(plane: CutPlane(point: p, normal: n))
// section.contours: closed loops, each classified by nesting:
//   depth 0 = outer solid boundary, depth 1 = a hole (inner wall / pocket), …
// A thin-walled tube → two separate loops; wall thickness = their offset.
let stack = mesh.crossSections(axis: axis, through: p, spacing: 2.0)  // slicer layer stack
```

- Intersection points welded by quantized world position (`weld:` tolerance,
  auto-derived from bbox), so coincident crossings chain even on unwelded STL.
- Inner-vs-outer comes from **contour nesting** (containment + signed area),
  not triangle winding — reliable on meshes with inconsistent orientation.
- Orientation normalized: even nesting depth CCW, odd CW.
- Open polylines (plane exits through a boundary edge) returned separately in
  `openPaths`.

New public types: `CutPlane`, `MeshContour`, `MeshCrossSection`.

## v1.0.0 — SemVer-stable

Promoted `Mesh.simplified(_:)` to a stable 1.0 line; pinned to OCCTSwift v1.0.1
(OCCT 8.0.0 GA). No API change from v0.1.0.

## v0.1.0 — `Mesh.simplified(_:)` via vendored meshoptimizer

Initial release. QEM mesh decimation backed by [meshoptimizer](https://github.com/zeux/meshoptimizer) v1.1 (MIT, vendored under `Sources/OCCTMeshOptimizer/src/meshoptimizer/`).

Requires OCCTSwift v0.156.2 or later (public `Mesh(vertices:normals:indices:)` initializer from [OCCTSwift#94](https://github.com/gsdali/OCCTSwift/issues/94)).

```swift
let result = mesh.simplified(.init(targetTriangleCount: 5_000))
// → SimplifiedMesh(mesh:, beforeTriangleCount:, afterTriangleCount:, hausdorffDistance:)
```

Validation:

- `targetTriangleCount` and `targetReduction` are mutually exclusive; one must be set.
- `targetTriangleCount` must be in `[1, input.triangleCount]`.
- `targetReduction` must be in `[0.0, 1.0]`.
- `maxHausdorffDistance`, when set, must be `>= 0`.
- Empty input meshes are rejected.

Bridge ABI (`Sources/OCCTMeshOptimizer/include/OCCTMeshOptimizer.h`):

- `OCCTMeshSimplify(...)` — runs the QEM pass, compacts orphan vertices via meshoptimizer's fetch remap, reports absolute Hausdorff distance.
- `OCCTMeshSimplifyFreeResult(...)` — releases caller-owned output buffers.
- `OCCTMeshSimplifyScale(...)` — exposes meshoptimizer's bbox-diagonal scale factor for callers that work in relative error units.

Tracking issue: [#1](https://github.com/gsdali/OCCTSwiftMesh/issues/1).

---

## Pre-release scaffold (2026-04-29)

Repository created with package skeleton, build scaffolding, and implementation plan in `docs/INITIAL_IMPLEMENTATION.md`. No public API yet — see [issue #1](https://github.com/gsdali/OCCTSwiftMesh/issues/1).
