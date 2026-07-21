# Segmentation (`Mesh.segmented(_:)`)

Dihedral region-growing followed by a mandatory primitive-fit merge pass, splitting a mesh into
plane/cylinder/sphere/cone surface regions. Pure Swift + simd — no OCCT kernel calls — ported
from OCCTReconstruct's `ReconstructCompute` (`RegionSegmentation.swift`, `RegionMerging.swift`,
`PrimitiveFitting.swift`, `Linalg.swift`), adapted to operate on `OCCTSwift.Mesh`'s own arrays.
Depends on the mesh foundations layer — see [mesh-foundations.md](mesh-foundations.md).

## Why merging is mandatory, not optional

Region-growing alone shatters a coarsely-tessellated curved surface: a low-poly cylinder's
adjacent facets sit further apart than `maxDihedralDegrees`, so each becomes its own planar
region ("confetti"). OCCTReconstruct's own bake-off tests pin this failure mode — substantial-
clean coverage drops to ~0% on smooth bodies without the merge pass. The merge pass greedily
unions adjacent regions whose union still fits ONE primitive within tolerance, collapsing those
facets back into one cylinder, while leaving real edges (a box's 90° corners, a chamfer's
distinct cone) intact because their union fits nothing well.

## Two normal-vector roles, deliberately kept separate

`segmented(_:)` welds internally (`SegmentOptions.weldTolerance`) to build the ADJACENCY graph —
region-growing and the merge pass's boundary-dihedral tracking both use face normals computed on
the WELDED positions. Primitive fitting (`PrimitiveFitter.bestFit`), by contrast, reads points
from the mesh's ORIGINAL (unwelded) `vertices`/`indices` — fitting wants the truest available
geometry, unperturbed by weld-snapping, and welding only ever moves a point by less than
`tolerance`. Region membership (`MeshRegion.triangleIndices`) always refers to the caller's own
original triangle order and count — welding remaps which triangles are considered neighbors, it
never drops or reorders triangles the caller sees.

This is why unwelded input does NOT silently degrade to one region per triangle (the failure
mode the tracking issue explicitly calls out): adjacency is always computed on welded topology
regardless of whether the input `Mesh` itself was pre-welded.

**Unlike `welded(tolerance:)`, `segmented(_:)` does not drop triangles that degenerate (repeat a
vertex) as a result of the internal weld.** A degenerate triangle still gets a face normal (the
+Z fallback — see `faceNormals()`) and still participates in region-growing / merging like any
other triangle. On dirty scans this can surface as junk single-triangle regions; the existing
`SegmentOptions.minRegionTriangles` filter is the mitigation (raise it above 1 to drop them).

## Performance: `bodyDiag` is hoisted, not recomputed

The OCCTReconstruct reference recomputes the mesh's bounding-box diagonal on every relevant
call site (used to scale the merge-acceptance tolerance). Doing that on every candidate merge
during the fit-gated pass would be an O(vertices) bounds scan per call — quadratic-ish at scale.
This port hoists `bodyDiag` to a single computation in `segmented(_:)`, threaded through
`RegionMerging.merge` as a parameter for `mergeTol` — the same math, computed once. Given the
tracking issue's explicit "usable at 400k+ triangles" requirement, this isn't optional polish.

`PrimitiveFitter.bestFit`'s own absolute residual floor (the fit-KIND tie-break, not the merge
gate above) does NOT use `bodyDiag` — it scales off the REGION's own bounding-box diagonal,
computed from the region's points directly inside `bestFit`. A body-wide floor was too coarse: a
shallow, genuinely-curved region on a much larger flat body (issue #20 item 4 — an R5000-class,
few-mm-sagitta roof panel on a multi-metre body) had its tiny residual swamped by a floor scaled
to the whole body, so `fit.kind` read `plane` where the surface was really a shallow arc. Region
MEMBERSHIP was never affected by this (the merge gate above uses residual RMS against `mergeTol`
only) — only the reported `fit.kind` on the affected region.

## Scale: coplanar pre-merge before the fit-gated pass

The fit-gated merge pass is O(regions × primitive-fit) per iteration and capped internally at
1500 regions (matching the OCCTReconstruct reference) to stay tractable. A 400k+ triangle scan
can easily produce more raw seed regions than that. Before the fit-gated pass runs, a cheap,
FIT-FREE coplanar pre-merge (`RegionMerging.coplanarPreMerge`, ~2° threshold) collapses adjacent
regions whose face normals are nearly identical — this only ever fuses genuinely-flat confetti
(coarse facets of what should be one plane), never a real curved-surface seam, because a ~2°
threshold is far tighter than any real edge. A single union-find pass suffices here (unlike the
fit-gated pass, its merges are unconditional, so there's no need to re-evaluate pending pairs
after each merge).

## Explicit, never-silent truncation

`SegmentOptions.maxRegions` (cap the OUTPUT to the largest N regions) and `minRegionTriangles`
(drop undersized regions) both report exactly what they dropped via
`SegmentedMesh.truncatedTriangleCount`, rather than silently shrinking the result. This mirrors
the tracking issue's explicit requirement — a cap that silently drops coverage reads as "fully
segmented" when it wasn't.

The same ethos applies to the fit-gated merge pass itself: if the coplanar pre-merge can't get
the raw region count under the internal 1500-region cap, the fit-gated pass is skipped entirely
rather than silently returning thousands of unmerged confetti regions with no explanation.
`SegmentedMesh.fitMergeSkipped` reports exactly that — `true` means `regions`/`fits` are the
pre-merge seed regions, not the fully-merged result.

## Determinism

Region composition and merge order both depend on decisions that would otherwise be
Dictionary-iteration-order-dependent (a fresh, randomized hash seed per process run in Swift):

- **Region growing** (`segmentSmoothRegions`) is a pure graph-reachability computation over a
  fixed adjacency SET — membership doesn't depend on `adjacency[t]`'s internal element order,
  only on seed order (`0..<triangleCount`, already deterministic).
- **Region merging**'s pair-processing order is explicitly sorted (softest boundary first,
  i.e. highest dihedral dot product, tie-broken on the packed region-pair key) rather than left
  to dictionary iteration — equal-dihedral pairs would otherwise merge in a different order each
  run, producing different region compositions and non-reproducible fits.

A dedicated determinism test (segment the same mesh twice, compare regions/fits/params exactly)
guards this — see `MeshSegmentationTests.swift`.

## Out of scope (tracked as follow-ups per issue #17)

- Curvature-based seeding (needs a curvature estimator first — Rusinkiewicz per-face tensor,
  planned separately).
- The Schnabel-style RANSAC alternative strategy (`PrimitiveRANSAC` in the OCCTReconstruct
  reference) and its auto-selection bake-off against dihedral region-growing.
- Slippage analysis (Gelfand-Guibas: per-region extrude/revolve/helix classification with axes).

These are phased behind the downstream OCCTMCP tool landing, per the tracking issue.
