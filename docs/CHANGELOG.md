# Changelog

All notable changes to OCCTSwiftMesh.

## v1.6.0 — slippage analysis (Gelfand-Guibas)

Adds `Mesh.slippage(forTriangles:maxSamples:)` ([#26](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/26)) — classifies a segmented region's surface kind (plane / sphere / cylinder / extrusion / revolution / helix / freeform) and recovers its characteristic axis, by local slippage analysis (Gelfand & Guibas, SGP 2004). Pure Swift + simd — no vendored library. See [docs/algorithms/slippage.md](algorithms/slippage.md).

```swift
let result = mesh.slippage(forTriangles: region.triangleIndices)
print(result.kind, result.axisPoint as Any, result.axisDirection as Any, result.pitch as Any)
```

- Builds the 6×6 "slippage covariance" of per-sample constraint rows `[p×n, n]`; near-zero
  eigenvalue RATIOS (relative to the largest, never absolute — patch-extent-independent) mark
  rigid motions the surface tolerates. Their count and character (pure translation / pure rotation
  / coupled screw) determine the kind and, for the curved kinds, the axis directly.
- `Linalg.eigenSymmetric(_:)` — a new N×N generalization of the existing `eigenSymmetric3`'s
  classical Jacobi eigensolver, reused here for the 6×6 case ("Jacobi generalizes directly").
- Per-vertex sample weighting (barycentric area lumping, as in a mass matrix) so densely
  tessellated sub-patches (e.g. a UV sphere's pole rings) don't bias the covariance away from the
  continuous surface integral it approximates.
- Points are normalized to a unit box before eigen-analysis; axis points and pitch are converted
  back to the mesh's real coordinate frame afterward.
- Like `triangleAdjacency()`/`connectedComponents()`, operates on THIS mesh's own vertex/normal
  arrays with no internal welding — callers pass an already-welded mesh.
- Deterministic: the same even-stride `maxSamples` subsample as ICP, and `eigenSymmetric`'s
  classical (largest-off-diagonal-element) Jacobi sweep, are both free of unordered-collection
  iteration or randomness.

## v1.5.0 — point-to-plane ICP alignment

Adds `Mesh.aligned(to:options:)` ([#22](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/22)) — point-to-plane ICP registration (PCA pre-align, normal-space sampling, trimmed correspondence), pure Swift + simd, no vendored library. See [docs/algorithms/alignment.md](algorithms/alignment.md).

```swift
let result = scan.aligned(to: cad)   // Mesh.AlignOptions() defaults
if let result {
    print(result.transform, result.residualRMS, result.iterations, result.converged)
}
```

- PCA/bbox pre-align tries all 4 orientation-preserving sign combinations of the two dominant
  principal axes (PCA eigenvectors have no inherent sign) and keeps whichever gives the lowest
  quick correspondence residual — avoiding a silently-wrong 180°-flipped starting pose.
- Normal-space sampling (Rusinkiewicz & Levoy, 2001) is on by default: source points are bucketed
  by their own normal direction and picked round-robin across buckets, so a small feature's rare
  normal direction gets comparable representation to the flat majority regardless of population
  size — the flat majority can no longer make the feature "slide" underneath noise.
- Trimmed ICP (distance cap + additional worst-`trimFraction` residual trim) handles partial
  overlap between the two meshes without converging to a wrong-but-locally-plausible alignment of
  just the non-overlapping parts.
- `AlignResult.residualRMS` is measured at the RETURNED pose (a final correspondence pass after
  the loop), not the pose one iteration prior.
- Deterministic: the internal k-d tree's median split, normal-space sampling's bucket order, and
  trimmed-correspondence ties all have explicit tie-breaks; PCA pre-align's 4 candidates are
  tried in a fixed order.
- Returns `nil` for degenerate input (either mesh has fewer than 3 points after welding) rather
  than a meaningless transform.
## v1.4.0 — discrete curvature estimation

Adds `Mesh.vertexCurvatures()` ([#23](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/23)) — per-vertex principal curvatures and directions via the Rusinkiewicz per-face tensor method, pure Swift + simd, ported from the algorithm trimesh2's `TriMesh_curvature.cc` (MIT) and the PMP library's curvature module implement. See [docs/algorithms/curvature.md](algorithms/curvature.md).

```swift
let welded = mesh.welded()                 // requires welded input, same precondition as
                                            // triangleAdjacency()/connectedComponents()
for c in welded.vertexCurvatures() {
    print(c.k1, c.k2, c.mean, c.gaussian)  // c.d1/c.d2 — unit principal directions
}
```

- Chosen over Meyer et al.'s cotan-Laplacian approach specifically for robustness on noisy/
  irregular real-scan tessellation: no obtuse-triangle clamp anywhere in the pipeline (the cotan
  approach needs one, and still degrades on slivers).
- Sliver-robust by construction: a face degenerate or an extreme sliver (near-zero area relative
  to its longest edge squared) is excluded from the fit entirely — never fed into an
  ill-conditioned solve that could poison its corners with a garbage tensor. A vertex touched
  only by excluded faces (or none at all) reports `k1 == k2 == 0`, never `NaN`.
  `Linalg.solve` returning `nil` and any non-finite intermediate result are both guarded directly
  too, as defense in depth beyond the area/aspect pre-filter.
- `k1` is always the eigenvalue of larger magnitude, signed so a convex bulge (e.g. an
  outward-facing sphere) is positive — matching `vertexNormals()`'s own outward-normal
  convention. Analytic test fixtures (a sphere zone, an open multi-ring cylinder, a flat grid)
  confirm this against the closed-form curvature of each shape.
- Deterministic: no Dictionary/Set iteration in the hot path, and each vertex's arbitrary initial
  tangent-frame pick doesn't affect the final (eigenvalue-invariant) result.

## v1.3.0 — #19 review follow-ups (fitMergeSkipped, isWatertight, region-local fit floor)

Follow-ups from the v1.2.0 (#19) review, tracked in [#20](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/20). None affect the correctness of what shipped in v1.2.0; two are behavior fixes, the rest are diagnostics/docs/tests.

- **`SegmentedMesh.fitMergeSkipped: Bool`** (new field) — `true` when even the coplanar
  pre-merge couldn't get the raw region count under the internal 1500-region cap, so the
  fit-gated merge pass was skipped entirely; `regions`/`fits` are then the unmerged seed regions.
  Previously silent — the caller saw thousands of "confetti" regions with no indication why.
- **`MeshIntegrityReport.isWatertight` now folds in vertex-manifoldness** (behavior fix):
  requires `nonManifoldVertexCount == 0` in addition to zero boundary/non-manifold edges, matching
  Open3D's actual `is_watertight` definition. Previously, two closed shells pinched at one shared
  vertex (a bowtie of closed shells) reported watertight, since neither shell has a boundary or
  non-manifold edge.
- **`integrityReport()`'s duplicate-face key is now exact at any welded-vertex count** — replaced
  a bit-packed `UInt64` key (three 21-bit fields, exact only below ~2M welded vertices) with a
  3-field `UInt32` struct key.
- **`PrimitiveFitter.bestFit`'s fit-kind tie-break floor now scales with the REGION's own
  bounding-box diagonal, not the whole mesh's** (behavior fix, `bodyDiag` dropped from `bestFit`'s
  signature) — a body-wide floor could swamp a small, genuinely-curved region's tiny residual on
  a much larger body (an R5000-class, few-mm-sagitta roof panel on a multi-metre body), reporting
  `fit.kind == .plane` where the surface was really a shallow arc. Region membership was never
  affected; only the reported `fit.kind` on the affected region.
- `docs/algorithms/segmentation.md` now documents that `segmented(_:)`, unlike `welded(_:)`, does
  not drop triangles that degenerate as a result of the internal weld (mitigated by
  `SegmentOptions.minRegionTriangles`).
- New test fixtures/coverage: an inconsistent-winding (`isOrientable == false`) fixture, a torus
  (genus 1) fixture, and an unwelded curved-body segmentation case (previously only the box was
  tested unwelded).

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
