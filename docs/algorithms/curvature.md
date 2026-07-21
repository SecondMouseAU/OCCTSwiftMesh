# Discrete curvature (`Mesh.vertexCurvatures()`)

Per-vertex principal curvatures and directions via the Rusinkiewicz per-face tensor method
(Rusinkiewicz, "Estimating Curvatures and Their Derivatives on Triangle Meshes", 3DPVT 2004) —
the same algorithm trimesh2's `TriMesh_curvature.cc` (MIT) and the PMP library's curvature module
implement. Pure Swift + simd — no OCCT kernel calls. Requires WELDED input, the same precondition
as `triangleAdjacency()`/`connectedComponents()` — see [mesh-foundations.md](mesh-foundations.md).

## Why Rusinkiewicz over Meyer's cotan-Laplacian

Meyer et al.'s mixed-area cotan-Laplacian approach is the other well-known discrete-curvature
method, but it needs an obtuse-triangle clamp: a naive Voronoi-area vertex weight goes negative
(or the cotan weight blows up) on an obtuse triangle, forcing a fallback rule for that case. Real
scan meshes have plenty of those — including outright slivers (see `MeshIntegrityReport`'s own
`minAngleDegrees`/`aspectRatio` sliver signals). Rusinkiewicz's method fits a curvature tensor per
FACE directly from that face's own edge vectors and vertex-normal differences, then averages face
tensors onto each vertex — no clamp anywhere in the pipeline, and (this package's choice) a
simple area-weighted average onto vertices rather than Meyer's mixed Voronoi area, trading a
little accuracy on the most skewed triangles for one less failure mode to reason about.

## Algorithm sketch

For each face, in an orthonormal `(t, b)` basis tangent to the face (`t` along one edge, `b`
completing the frame with the face normal):

1. Build a small 3×3 least-squares system from the face's 3 edges: each edge's `(u, v) = (e·t,
   e·b)` projection, and the corresponding difference in the two OTHER corners' vertex normals
   (not the face normal — the whole point is to sense how the surface's true normal varies).
2. Solve for the face's own second-fundamental-form coefficients `(e, f, g)` (the tensor
   `[[e, f], [f, g]]` in the face's `(t, b)` basis) via `Linalg.solve` (reusing the same Gaussian
   elimination `PrimitiveFitter` uses — this is genuinely just a 3-unknown linear solve, no need
   for the dedicated LDLT trimesh2 uses).
3. Re-express that tensor in each of the face's 3 vertices' own (arbitrary, per-vertex) tangent
   bases, weight by the face's area, and accumulate.
4. Per vertex: normalize by the total accumulated weight, then diagonalize the averaged tensor to
   recover `k1` (larger-magnitude eigenvalue), `k2`, and their tangent directions.

The basis-rotation step (`rotCoordSys`/`projCurv`/`diagonalizeCurv` in `Mesh+Curvature.swift`) is
ported directly from trimesh2's implementation of the same algorithm — it's the exact minimal
rotation taking one unit normal to another, not a first-order approximation, so it's valid even
though a face's own normal and its corners' vertex normals never exactly agree.

Each vertex's initial `(pdir1, pdir2)` tangent basis is arbitrary (the last-encountered incident
edge, projected orthogonal to the vertex normal, in triangle-index order — deterministic, not
random) but harmless: since `k1`/`k2` are eigenvalues of the accumulated tensor, they're
independent of which orthonormal basis the tensor happens to be accumulated in.

## Sign convention

`k1`/`k2` are signed so a convex bulge (e.g. an outward-facing sphere) is **positive** — matching
`vertexNormals()`'s own outward-normal convention for a consistently-wound mesh. `k1` is always
the eigenvalue of larger magnitude (matching common practice in the reference implementations);
for a plane or a sphere, where the two are equal (or in the sphere's case, the eigenspace is
degenerate), the split between `k1` and `k2` is not geometrically meaningful, just whichever the
tie-break landed on. `mean = (k1 + k2) / 2`, `gaussian = k1 * k2`.

## Sliver-robust: excluded from the fit, not blown up

A face is excluded from the fit entirely — contributing nothing to its corners' accumulated
tensors — when it's degenerate (near-zero area) or an extreme sliver (area tiny relative to its
longest edge squared, i.e. a near-zero minimum angle): feeding such a face's ill-conditioned
edge/normal-difference system into the solve risks a huge (if still technically finite) garbage
tensor that would then poison that face's own corners. This is a documented degradation — those
corners simply don't get this one face's contribution, same as `PrimitiveFitter`'s per-face
guards — never a silently propagated `NaN`. `Linalg.solve` returning `nil` (a genuinely singular
system) and any non-finite result from the solve or the final diagonalization are both guarded
directly, as defense in depth beyond the area/aspect pre-filter.

A vertex touched only by excluded faces (or no faces at all, e.g. an orphan vertex) reports
`k1 == k2 == 0` rather than `NaN` — the same "degrade to a harmless default" pattern
`PrimitiveFitter.bestFit` and `welded(tolerance:)` already use elsewhere in this package.

## What's NOT here

Curvature-ordered segmentation seeding (growing low-curvature regions first, leaving high-
curvature strips as fillet/blend candidates) and a single-body curvature render/heatmap mode are
both OCCTMCP-side follow-ups layered on top of this primitive (tracked upstream, not in this
package — see [issue #23](https://github.com/SecondMouseAU/OCCTSwiftMesh/issues/23)'s context).
Curvature DERIVATIVES (the extension of Rusinkiewicz's paper this package doesn't implement) are
also out of scope — only the curvature tensor itself, not its rate of change.
