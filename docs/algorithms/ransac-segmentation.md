# RANSAC segmentation (`Mesh.segmentedRANSAC(_:)`, `Mesh.segmentedAutoSelect(dihedral:ransac:)`)

Schnabel, Wahl, Klein, "Efficient RANSAC for Point-Cloud Shape Detection" (2007). An alternative
segmentation strategy to `segmented(_:)`'s dihedral region-growing, for multi-primitive scenes
where growing shatters or under-segments. Pure Swift + simd, no OCCT kernel calls. Depends on the
mesh foundations layer and reuses `PrimitiveFitter` (see [segmentation.md](segmentation.md)).

## Dihedral growing vs. RANSAC

`segmented(_:)` only ever absorbs edge-ADJACENT neighbours — a single continuous surface graph to
walk, which is exactly right for one integrated part. RANSAC instead fits a candidate primitive
and claims its inliers across the WHOLE remaining point set, wherever they are — the right model
for a scene where the same primitive kind (plane, cylinder, …) shows up in several physically
disconnected patches. `segmentedAutoSelect(dihedral:ransac:)` runs both and keeps whichever scores
higher on a coverage metric (see below), rather than hard-coding which strategy wins.

## Algorithm sketch

Each round:

1. **Draw a small deterministic sample** (`RANSACSegmentOptions.sampleSize`, default 6 triangles)
   from the remaining (unclaimed) pool.
2. **Fit every primitive kind the sample supports** (`PrimitiveFitter.allFits`, not `bestFit` —
   see "Why `allFits`, not `bestFit`" below) and score EACH one by its GLOBAL inlier count: every
   remaining triangle within `inlierEpsilon` of the candidate surface AND within
   `maxNormalDeviationDegrees` of the candidate's own expected normal there (see "Orientation-
   agnostic normal gate" below) counts as an inlier, regardless of whether it's contiguous with
   the sample.
3. **Refine the winner.** A 6-triangle sample's own fit is noisy; refit the SAME primitive kind
   using all of ITS OWN inliers (a much larger, cleaner point set) and re-score globally once
   more. Skipping this step routinely fragments one true surface across several rounds, each only
   ever as accurate as its own tiny sample — see "Why refinement is not optional" below.
4. **Cluster the (refined) global inliers by spatial proximity** (`clusterEpsilon`) — filters out
   a triangle that only satisfies the tolerance/normal gates by coincidence, far from the rest,
   rather than genuinely belonging to the same physical feature. Each cluster at or above
   `minSupportCount` becomes its own `MeshRegion` (re-fit one final time from just its own
   triangles) + `FittedPrimitive` pair.
5. **Stop when a round finds nothing** meeting `minSupportCount` after clustering — "coverage
   stalled." Unclaimed triangles are reported via `SegmentedMesh.truncatedTriangleCount`, never
   silently dropped.

## Candidate generation: a documented simplification vs. Schnabel's minimal sets

Schnabel's original method solves each primitive kind from its OWN minimal oriented-point set in
closed form (1-2 points for plane/sphere/cylinder, 3 for a cone). This port instead draws a small
sample and reuses the already-tested `PrimitiveFitter.bestFit`/`allFits` least-squares machinery —
slightly more expensive per candidate (solving a small linear system instead of a closed form), but
far more numerically robust (no new per-primitive-type closed-form derivation to get subtly
wrong) and reuses fitting code this package already trusts rather than adding a second, parallel
fitting path. `sampleSize`'s default (6) is a general-purpose middle ground; a scene with many
small, thin primitives may want it smaller (see below).

## Why `allFits`, not `bestFit`

`PrimitiveFitter.bestFit`'s simplicity bias (prefer plane, then cylinder, then cone, then sphere,
among fits within 1.25× + a floor of the best residual) exists to break genuinely ambiguous ties
on a REGION-SIZED point set. It actively hurts RANSAC's SAMPLE-SIZED candidates: a small local
patch of ANY smooth curved surface looks nearly flat up close, so its plane residual and its true
(sphere/cylinder/cone) residual are almost always close enough for the simplicity bias to pick
"plane" — silently starving every other primitive kind of a chance to ever be proposed at all,
regardless of how well it would actually fit once scored globally. `allFits` returns every kind
that fit at all, unfiltered, so the GLOBAL inlier count (the true arbiter — a plane candidate from
a locally-flat patch of a sphere will have far fewer global inliers than the true sphere candidate
from the very same sample) decides instead of a residual tie-break tuned for a different sample
scale.

## Why refinement is not optional

A 6-triangle sample's fit (however it scores among `allFits`' candidates) still only reflects 6
triangles' worth of noise. Its own raw global-inlier count under a real tolerance can genuinely
miss triangles that belong to the same surface, simply because the noisy sample-based fit's
parameters are a little off. Skipping the refit step (score the sample's own fit once, cluster,
move on) empirically fragments a single true sphere/cylinder across MULTIPLE rounds, each
capturing a different partial patch with a slightly different noisy center/radius/axis — several
smaller regions where the surface should have been one. Refitting the SAME primitive kind
directly (not via `bestFit`, which could flip kind on a still-partial patch for the exact reason
above) against the sample's own (much larger) inlier set converges close enough to the true
surface to capture its full extent in one round.

## Orientation-agnostic normal gate

`maxNormalDeviationDegrees` compares a candidate triangle's face normal against the fitted
primitive's own expected normal there using `|cos deviation|`, not the signed dot product — a
triangle with flipped/unknown winding still lies tangent to the fitted surface, and real scan
meshes routinely have inconsistent or unknown global winding (the exact motivation for
`windingNumber`/`orientationReport`, issue #30 — see [winding-number.md](winding-number.md)). A
signed check would silently reject half the triangles of any consistently-but-oppositely-wound
mesh; only the tangent-plane match should matter here.

## `clusterEpsilon`: connectivity is spatial proximity, not mesh topology

Unlike `inlierEpsilon` (auto-derives from the mesh's bounding-box diagonal — a FIT tolerance,
which should scale with the model's overall size), `clusterEpsilon` auto-derives from the mesh's
own mean edge length — a CONNECTIVITY tolerance, which should scale with tessellation density
instead. Clustering is a spatial grid-hash union-find over inlier CENTROIDS (the same grid-hash
style as `weldPositions`), not mesh adjacency — a real scan surface's connectivity graph can have
seams/gaps that don't reflect true spatial adjacency, and Schnabel's own design clusters over the
point cloud independent of any pre-existing mesh topology for exactly this reason.

## Adaptive trial budget

`successProbability` drives `Mesh.requiredRANSACTrials`: as a round's best-found inlier support
grows, fewer further candidate trials are needed to be `successProbability`-confident nothing
larger remains undiscovered — Schnabel's own adaptive stopping formula
(`trials ≈ log(1 - successProbability) / log(1 - (support/remaining)^sampleSize)`), always capped
by `maxCandidatesPerRound` regardless.

## `segmentedAutoSelect`: the bake-off metric

"Substantial-clean coverage" (the OCCTReconstruct bake-off convention this package's segmentation
docs already reference): the fraction of total mesh area covered by regions that are both
SUBSTANTIAL (`>= minAreaFraction` of the total, default 1%) and CLEAN (`inlierRatio >=
minInlierRatio`, default 0.8). Whichever strategy scores higher wins; a tie (including both
scoring zero) favors `dihedral`, the cheaper strategy — matching the tracking issue's framing that
dihedral growing should win decisively on a single integrated part, with RANSAC as the specialist
for multi-primitive scenes.

## Determinism

Classic RANSAC draws random minimal sets; this needs repeat calls to be byte-identical instead.
`Mesh.deterministicSample(trial:poolSize:sampleSize:)` (a splitmix64 index hash keyed by trial
number, not a system RNG) gives each candidate trial an independent, well-mixed, fully
reproducible sample — unlike a sliding window over one fixed shuffle, which only ever offers as
many distinct windows as the remaining pool size regardless of how many trials are requested, an
early design this package moved away from specifically because it starved candidate diversity on
small multi-primitive pools (a 12-triangle box's 6 separate faces). `spatialClusters`' grid-hash
union-find is likewise deterministic regardless of `Dictionary` iteration order — the final
partition depends only on which pairs are within `epsilon`, not the order buckets are visited in.

## Test fixture notes

A COARSE UV sphere's own tessellation has a real, unavoidable discretization "sagitta" — a face
centroid sits measurably below the true analytic sphere surface, worse at coarser tessellation.
If that sagitta exceeds `inlierEpsilon`, even a PERFECT sphere fit legitimately fails the distance
gate on much of the mesh — fragmenting across rounds for a reason that's about tessellation
coarseness relative to tolerance, not a segmentation defect. `RANSACSegmentationTests`' sphere
fixture uses a fine-enough tessellation that this doesn't dominate; a real caller segmenting a
coarse mesh should widen `inlierEpsilon` accordingly.
