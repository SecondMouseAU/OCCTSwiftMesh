# Mesh foundations (`Mesh.welded(_:)` and the connectivity toolkit)

Weld, per-triangle-adjacency, connected components, sub-mesh extraction, boundary loops, and a
manifoldness/validity/quality report (`MeshIntegrityReport`). Pure Swift + simd — no OCCT kernel
calls — ported from OCCTReconstruct's `ReconstructCompute.IndexedMesh` / `MeshRepair` /
`MeshClosing`, adapted to operate directly on `OCCTSwift.Mesh`'s own `vertices`/`indices` arrays.

## Why weld first

OCCT tessellation and STL loading both produce (near-)unshared vertices in practice: three
unique positions per triangle, even where two triangles are geometrically edge-adjacent. Every
adjacency-based primitive here (`triangleAdjacency`, `connectedComponents`, `boundaryLoops`)
keys its connectivity off shared VERTEX INDICES, not positions — so on unwelded input, no two
triangles share an index and each comes back looking isolated (every triangle its own component,
every edge a "boundary"). `welded(tolerance:)` is the deliberate, documented prerequisite.

`integrityReport()` and `segmented(_:)` (see [segmentation.md](segmentation.md)) weld internally
instead, precisely because unwelded input silently degrading to a meaningless result (a
non-manifold report with zero non-manifold edges; one segmentation region per triangle) is a
worse failure mode than the extra weld pass's cost.

## Weld algorithm: grid-hash, not radius search

`welded(tolerance:)` snaps each vertex to a spatial grid cell (`tolerance`, or `1e-6 ×` the
bounding-box diagonal when `0`) and merges every vertex landing in the same cell, keeping the
first-encountered position as the cell's representative. This is the same technique
`crossSection`'s intersection-point welding already uses in this package (`CrossSection.swift`),
kept for consistency.

**Known limitation** (inherited from that same precedent): two points within `tolerance` of each
other but straddling a grid-cell boundary can round into different cells and fail to merge. The
cell size is tiny relative to the model by default (`1e-6 ×` bbox diagonal), far below any real
wall thickness, so this only bites when a caller passes an explicit `tolerance` and places points
adversarially close to a cell boundary — not a concern in practice, and not worth the complexity
of a proper neighbor-cell radius search for a foundational, high-call-volume primitive.

`welded(tolerance:)` also drops any triangle that degenerates (repeats a vertex) as a result of
welding, since a zero-area triangle would otherwise corrupt every downstream adjacency
computation. It does NOT drop duplicate (non-degenerate, repeated-vertex-set) triangles — that
diagnostic is reserved for `integrityReport()`, which surfaces it explicitly rather than silently
cleaning it up.

## `MeshIntegrityReport` — counted before cleanup vs. computed after

`duplicateTriangleCount` and `degenerateTriangleCount` are counted from the RAW welded topology,
before any cleanup — that's the whole point of reporting them. Every other metric
(`isWatertight`, `isOrientable`, `nonManifoldEdgeCount`, `nonManifoldVertexCount`,
`boundaryLoopCount`, `eulerCharacteristic`, `genus`, `components`, the sliver signals) is computed
on the deduplicated, non-degenerate topology instead, so a handful of exact duplicate faces
doesn't masquerade as a real non-manifold defect (a duplicated triangle trivially makes its three
edges non-manifold — that's noise, not signal).

The duplicate-face check keys each triangle on its sorted, welded vertex-index triple (three
`UInt32`s) rather than bit-packing them into one integer — exact at any welded-vertex count,
with no field-width ceiling to document or hit.

Semantics follow the [Open3D `TriangleMesh`](https://www.open3d.org/docs/release/tutorial/geometry/mesh.html)
conventions (`is_edge_manifold` / `is_vertex_manifold` / `is_watertight` /
`cluster_connected_triangles`) — the cleanest in the field per the tracking issue.

### Non-manifold vertex detection

A vertex `v` is non-manifold if the triangles around it don't form a single fan. For every
triangle containing `v`, the edge OPPOSITE `v` is one link of `v`'s "umbrella"; in a manifold
those opposite edges chain into a single simple path (open, at a boundary) or cycle (closed). A
branch point (an opposite-edge endpoint touched by 3+ opposite edges) or a second disconnected
chain (a pinch point / bowtie, two cones of triangles touching at exactly one vertex) makes `v`
non-manifold. Implemented by building the per-vertex "opposite edge" graph and checking max
degree ≤ 2 and exactly one connected component.

### `isWatertight` folds in vertex-manifoldness

`isWatertight` requires `nonManifoldVertexCount == 0` in addition to zero boundary and zero
non-manifold edges — matching Open3D's actual `is_watertight` definition (edge-manifold AND
vertex-manifold AND no boundary), not just the edge-manifold half of it. Edge-manifoldness alone
misses a real defect: two otherwise-closed shells pinched together at a single shared vertex (a
"bowtie" of closed shells) have zero boundary edges and zero non-manifold edges — every edge is
still shared by exactly two triangles — but the pinch vertex is not a single triangle fan, so the
result is not a single watertight solid.

### Genus

`genus` is `nil` unless `isWatertight && isOrientable` (and the Euler characteristic is
consistent with a valid closed 2-manifold — an odd numerator defensively yields `nil` rather than
a nonsensical fractional genus). For `C` disjoint closed orientable components,
`χ_total = Σ(2 − 2gᵢ) = 2C − 2·G_total`, so `genus = (2·components.count − eulerCharacteristic) / 2`.

### Sliver signals

- `minAngleDegrees`: per-triangle smallest interior angle, reduced to `(min, p05)` — the absolute
  worst case and the 5th percentile (more robust to a single degenerate outlier).
- `aspectRatio`: `longestEdge / (2·√3·inradius)`, the VTK/FEM convention where an equilateral
  triangle scores exactly `1.0` and every other triangle scores higher — reduced to `(max, p95)`.

## Determinism

`connectedComponents()` and `boundaryLoops()` both build their groupings from Dictionary/Set
iteration internally, which is NOT stable across process runs (Swift randomizes hash seeds).
Both are made deterministic by an explicit tie-break at the point results are materialized:

- `connectedComponents()`: regions sorted largest-first, ties broken by lowest triangle index
  (`MeshRegion.order`). Bucket MEMBERSHIP is independent of dictionary iteration order (union-find
  correctness doesn't depend on processing order); only the final SORT needs the tie-break.
- `boundaryLoops()`: loops are seeded in sorted edge-key order, and each vertex's neighbor list is
  sorted before the walk. Without this, a different seed chains edges into different loops at
  non-manifold junctions, changing the result between runs on identical input.

## What's NOT here

Mesh repair (non-manifold membrane removal via generalized winding number, hole-filling/capping)
stays in OCCTReconstruct's `MeshRepair.swift` / `MeshClosing.swift` — out of scope for this
release. `integrityReport()` diagnoses; it does not repair.
