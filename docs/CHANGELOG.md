# Changelog

All notable changes to OCCTSwiftMesh.

## v1.2.0 — mesh foundations + region segmentation (#16, #17)

Adds the mesh-domain primitives needed for raw-mesh region analysis (OCCTMCP #101
`segment_mesh_zones` / #102 `zone_continuity_sweep`), ported from OCCTReconstruct's
`ReconstructCompute` reference implementation (pure Swift + simd, same lineage, no third-party
entanglement).

**Foundations (#16):**

- `Mesh.welded(tolerance:)` — grid-hash vertex weld (tolerance `0` auto-derives `1e-6 ×`
  bbox diagonal, matching `crossSection`'s existing weld default). Everything below assumes
  welded input — OCCT tessellation and STL loading produce (near-)unshared vertices in practice,
  so an unwelded soup has no shared edges at all.
- `Mesh.faceNormals()` / `Mesh.vertexNormals()` — unit per-triangle and area-weighted per-vertex
  normals.
- `Mesh.triangleAdjacency()` — per-triangle edge adjacency, deterministically ordered.
- `Mesh.subMesh(triangleIndices:)` — extract a triangle subset as a standalone, compactly
  reindexed mesh.
- `Mesh.boundaryLoops()` — closed rings of open (1-triangle) boundary edges.
- `Mesh.connectedComponents() -> [MeshRegion]` — union-find split into physically disjoint
  pieces, largest-first, deterministic tie-break.
- `Mesh.integrityReport() -> MeshIntegrityReport` — watertight / edge-manifold / vertex-manifold /
  orientable, non-manifold edge/vertex counts, boundary-loop count, duplicate/degenerate triangle
  counts, Euler characteristic + genus (single watertight component only), per-component
  breakdown, and sliver signals (worst + p05 minimum angle, worst + p95 aspect ratio). Semantics
  follow the Open3D `TriangleMesh` conventions.
- New shared type `MeshRegion`: triangle-index subset plus area / bbox / mean-normal /
  boundary-loop-count, computed once at construction — the common return unit for both
  `connectedComponents()` and `segmented(_:)` below.

**Segmentation (#17):** `Mesh.segmented(_:) -> SegmentedMesh`

- Dihedral region-growing (deterministic DFS flood over edge-adjacent triangles, gated on
  face-normal agreement) followed by the **mandatory** primitive-fit merge pass: adjacent regions
  merge when their union still fits ONE analytic primitive (plane / cylinder / sphere / cone)
  within tolerance. Without the merge pass a coarsely tessellated curved surface (e.g. a 12-facet
  cylinder) shatters into one region per facet — pinned by
  `coarsePrismMergesBarrel` in `MeshSegmentationTests`.
- Every region arrives with a fitted `FittedPrimitive` (kind, params, residual RMS/max, inlier
  ratio) for free.
- `SegmentOptions.maxRegions` caps the result; truncation is reported via
  `SegmentedMesh.truncated`, never silent.
- **Determinism**: two calls on identical input produce byte-identical output (regions AND their
  internal triangle-index order). This required sorting `triangleAdjacency()`'s per-triangle
  neighbour lists and canonicalizing every region's triangle-index array ascending — Dictionary
  iteration order is not a reproducible tie-break on its own, even within a single process run.
- First-order only (face normals); no curvature term. Curvature-seeded growing is a deliberate
  follow-up (needs a curvature estimator first).

No OCCTSwift API or vendored-dependency change.

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
