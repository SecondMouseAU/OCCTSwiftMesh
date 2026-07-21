# Alignment (`Mesh.aligned(to:options:)`)

Point-to-plane ICP registration recovering the rigid transform (rotation + translation) that best
aligns this mesh (the SOURCE) onto a reference mesh. Pure Swift + simd — no OCCT kernel calls, no
vendored library. Ported from the classic ICP literature: Chen & Medioni's point-to-plane
objective, Rusinkiewicz & Levoy's normal-space sampling ("Efficient Variants of the ICP
Algorithm", 2001), and Low's linearized point-to-plane solve ("Linear Least-Squares Optimization
for Point-to-Plane ICP", 2004). Welds both meshes internally (for per-point normals and a
deduplicated point cloud) — a composite, mesh-level operation, not a low-level connectivity
primitive the caller pre-welds for, unlike `triangleAdjacency()`/`vertexCurvatures()`.

## Why point-to-plane, and why it needs a good starting pose

Point-to-plane ICP minimizes each correspondence's distance to the reference surface's TANGENT
PLANE rather than to the reference point itself — it converges far faster on engineering surfaces
than point-to-point ICP (Chen & Medioni), but only from a reasonable starting pose: like any
gradient-descent-style local optimization, it can converge to the wrong local minimum from a bad
start. `AlignOptions.preAlign` (on by default) fixes this with a PCA/bbox pre-alignment pass
before the iterative refinement — see below.

## Algorithm sketch

1. **Weld + normals.** Both meshes are welded internally; reference vertex normals become the
   plane normals correspondences are measured against.
2. **PCA pre-align** (`AlignOptions.preAlign`). Align centroids and principal axes — but PCA
   eigenvectors have no inherent sign, so a naive single choice can pick a 180°-flipped starting
   pose. This package tries all 4 orientation-preserving sign combinations of the two dominant
   axes (the third axis is always re-derived via cross product to keep an orthonormal,
   right-handed frame — never an improper/reflected one) and keeps whichever gives the lowest
   quick correspondence residual over a coarse subsample. Deterministic: candidates are tried in
   a fixed order, ties broken by that order.
3. **Normal-space sampling** (`AlignOptions.normalSpaceSampling`, on by default — mandatory in
   practice per the tracking issue). Source points are bucketed by their own normal direction (a
   coarse lat/long grid) and picked round-robin across non-empty buckets in a fixed order, rather
   than uniformly. On a mostly-flat surface with a small feature, uniform sampling lets the flat
   majority dominate the correspondence set and the feature "slide" underneath noise — normal-
   space sampling gives every distinct normal direction, including the feature's rare one,
   comparable representation regardless of how many points share it. The sample is a FIXED
   subset, computed once and reused every iteration (not resampled per iteration) — deterministic,
   and resampling isn't necessary for convergence at a fixed sample size.
4. **Per iteration:** find each sampled point's nearest reference point (a k-d tree over the
   welded reference positions — `KDTree3.swift`, an internal implementation detail), reject
   correspondences farther apart than `correspondenceDistanceCap` (auto: `0.15 ×` the reference
   mesh's bounding-box diagonal), then additionally trim the worst `trimFraction` of SURVIVING
   correspondences by point-to-plane residual (trimmed ICP — robust to partial overlap). Solve
   Low's linearized 6-DOF system for the incremental rigid transform via `Linalg.solve` (the same
   generic Gaussian-elimination solver `PrimitiveFitter` and `vertexCurvatures()` use, just at
   6×6 instead of 3×3 or 4×4), and compose it onto the running pose. Stop when the residual RMS
   stops improving meaningfully, when `maxIterations` is reached, or when too few correspondences
   survive to solve the next increment (fewer than 6).
5. **Report.** `AlignResult.residualRMS` is measured with one final correspondence pass at the
   RETURNED pose (not the pose one iteration prior), for an accurate reported metric.

## Sign/rotation conventions

`AlignResult.transform` maps the SOURCE mesh's ORIGINAL vertex positions into the reference
mesh's frame. The incremental-rotation solve uses Rodrigues' rotation formula directly (exact for
any angle, not a small-angle linearization of the rotation ITSELF — only the linear SYSTEM being
solved for the incremental rotation vector is a small-angle approximation, standard for ICP).

## Determinism

No Dictionary/Set iteration affects results order: the k-d tree's per-level median split sorts
with an index tie-break, normal-space sampling's bucket order is a sorted key list, and the
trimmed-correspondence sort ties-break on original order. PCA pre-align's 4 candidates are tried
in the same fixed order every run.

## What's NOT here

The GOM-style alignment-MODE enum (pre-align / best-fit / local-best-fit / 3-2-1 / RPS-datum) and
`OCCTMCP`'s `align_bodies` tool are layered on top of this primitive on the OCCTMCP side (tracked
upstream, not in this package — see
[issue #22](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/22)'s context). This is just the
registration primitive itself.
