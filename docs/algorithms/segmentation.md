# Mesh foundations + region segmentation

`Mesh.welded` / `faceNormals` / `vertexNormals` / `triangleAdjacency` / `subMesh` /
`boundaryLoops` / `connectedComponents` / `integrityReport`, and `Mesh.segmented(_:)`.

## Provenance

Ported from OCCTReconstruct's `ReconstructCompute` layer (`IndexedMesh`, `SubMesh.swift`,
`RegionSegmentation.swift`, `RegionMerging.swift`, `PrimitiveFitting.swift`, `Linalg.swift`) —
pure Swift + simd, same organizational lineage, no third-party entanglement. Adapted from the
`IndexedMesh` value type onto `OCCTSwift.Mesh`, and from the reference's `MeshRegion` (a bare
triangle-index list) onto this package's richer `MeshRegion` (area / bbox / mean-normal /
boundary-loop-count computed once at construction, since the primary consumer — a per-zone
metadata table — needs all four immediately).

## Why welding comes first

OCCT tessellation and STL loading produce (near-)unshared vertices in practice: three vertex
copies per triangle, no index sharing at all. Every adjacency-based algorithm here — adjacency
itself, region growing, connected components, boundary-loop tracing — needs a welded substrate
or it finds zero neighbours anywhere. `welded(tolerance:)` is a grid-hash merge (round each
vertex's position to a `tolerance`-sized grid cell, dedupe by cell), with `tolerance = 0`
auto-deriving `1e-6 × bboxDiagonal` — the same default `crossSection`'s intersection-point weld
already uses.

## Region growing: dihedral flood fill

`segmentSmoothRegions` (internal to `segmented(_:)`) is a deterministic DFS flood over
edge-adjacent triangles: a neighbour joins the current region iff its face normal is within
`maxDihedralDegrees` of the seed triangle's running region membership (`dot(n_t, n_neighbour) >=
cos(threshold)`). First-order only — no curvature term. Regions are seeded by ascending triangle
index and returned largest-first with a deterministic tie-break (lowest first-triangle index),
carried over from the reference's `IndexedMesh.regionOrder`.

## The merge pass is mandatory, not optional

A coarsely tessellated curved surface has real facet-to-facet dihedral angles — a 12-sided prism
approximating a cylinder has ~30° between adjacent side facets, comfortably over the default 20°
growth threshold. Region growing alone therefore shatters the barrel into 12 separate per-facet
planes ("confetti"). `RegionMerging.merge` repairs this: it walks adjacent region pairs
smoothest-shared-boundary-first (bounded by `maxMergeAngleDegrees`, default 50°, which is what
distinguishes a coarse cylinder's ~30° facet seams from a box's 90° corner) and accepts a merge
only when the union still fits ONE analytic primitive (plane / cylinder / sphere / cone) within
`max(mergeRelativeTolerance × bboxDiagonal, 1e-6)`, and doesn't degrade either parent's own fit
beyond a small slack. `MeshSegmentationTests.coarsePrismMergesBarrel` pins this: a 12-facet prism
segments to exactly 3 regions (barrel + 2 caps), not 14.

Primitive fitting (`PrimitiveFitting.swift`) is plane (PCA), sphere (algebraic/Kåsa), cylinder
(axis = smallest eigenvector of the normal covariance, then a 2D circle fit in the perpendicular
plane), and cone (axis from the normals' covariance, then a perpendicular-distance apex solve) —
all built on a small 3×3 symmetric Jacobi eigensolver (`Linalg.swift`) since this only ever needs
per-region point clouds, not a general dense linear-algebra dependency.

## Determinism

Two `segmented(_:)` calls on identical (welded) input return byte-identical output — same
regions, same triangle-index order within each region, same fits. Getting this right required
more than just processing pairs in a sorted order:

- `pairMinDot`'s merge-candidate list is sorted by `(minDot desc, packed-pair-key asc)` — ties on
  dihedral angle are real (a symmetric prism has many identical facet angles) and need a fixed
  tie-break, or the union-find would consolidate differently run to run.
- `triangleAdjacency()`'s per-triangle neighbour lists are sorted ascending before being handed
  out. Building the underlying edge→triangles map is itself a `Dictionary`, and iterating a
  `Dictionary` visits a triangle's (up to 3) edges in hash-bucket order — not a reproducible
  order on its own, even across two calls in the *same process* with *identical* input (verified
  empirically while writing `MeshSegmentationTests.segmentationDeterministic`: the first version
  of this code passed the regression-pin tests but failed determinism on the DFS-grown regions'
  internal member order).
- Every region's `triangleIndices` array is additionally canonicalized ascending at every
  construction point (raw DFS growth, and the merge pass's `union`) as a second line of defense —
  a region's triangle list is a stable value independent of how it was traversed or which side of
  a union-find pair happened to be root.

## Out of scope (deliberate)

- **Curvature-seeded growing.** Needs a curvature estimator first (Rusinkiewicz per-face tensor
  is the planned approach); tracked as a follow-up once that lands.
- **RANSAC primitive fitting.** The current fitter is deterministic least-squares per candidate
  region, not a robust/outlier-tolerant search; a Schnabel-style RANSAC alternative is a
  follow-up, not a replacement.
- **Slippage analysis** (Gelfand–Guibas): classifying a region's *motion* (extrude direction /
  revolve axis / helix pitch), not just its static shape. A separate follow-up.
