# Slippage analysis (`Mesh.slippage(forTriangles:maxSamples:)`)

Classifies a mesh region's surface kind (plane / sphere / cylinder / extrusion / revolution /
helix / freeform) and recovers its characteristic axis, by local slippage analysis (Gelfand &
Guibas, "Shape Segmentation Using Local Slippage Analysis", SGP 2004). Pure Swift + simd — no
OCCT kernel calls, no vendored library.

Like `triangleAdjacency()`/`connectedComponents()` (the "weld precondition family" — see
`Mesh+Topology.swift`'s header, and unlike `segmented(_:)`/`aligned(to:)`, which weld internally),
`slippage(forTriangles:maxSamples:)` reads this mesh's OWN `vertexNormals()` with no internal
weld: callers pass an already-welded mesh so those normals reflect real surface curvature.

## Algorithm sketch

A rigid motion — angular velocity `ω`, linear velocity `v` — SLIPS a surface over itself
(preserves it, to first order) exactly where its velocity field has zero component along every
surface normal: `ω·(pᵢ×nᵢ) + v·nᵢ = 0` for every sample `(pᵢ, nᵢ)`. That is one linear constraint
per sample on the 6-vector `(ω, v)`. Stack the constraint rows `cᵢ = [pᵢ×nᵢ, nᵢ]` and the
slippable motions are the near-null space of the 6×6 "slippage covariance" `M = Σᵢ cᵢcᵢᵀ` — small
eigenvalues (RATIOS relative to the largest, never absolute magnitudes, since those scale with
patch extent) mark motions the surface tolerates.

1. **Gather + weight samples.** Deduplicate the region's vertices (from `triangleIndices`); each
   vertex's weight is 1/3 the area of every region triangle touching it (barycentric lumping, as
   in a mass matrix). An unweighted point sum badly overrepresents densely tessellated patches
   (e.g. a lat/long UV sphere's pole rings: many vertices, tiny actual area each) relative to
   coarser ones, biasing the covariance away from the continuous surface integral it approximates.
   Capped to `maxSamples` via the same deterministic even-stride subsample as ICP
   (`Mesh.uniformSample`).
2. **Normalize to a unit box.** Points are centered and scaled so the bounding box's largest
   extent is `1` before the covariance is built — eigenvalue thresholds are then scale-free
   ratios, not absolute magnitudes tied to the region's physical size. Normals need no
   normalization (already unit, direction is scale-invariant).
3. **Build and eigen-decompose the 6×6 covariance** (`Linalg.eigenSymmetric`, the same classical
   Jacobi method as `eigenSymmetric3`, generalized — "Jacobi generalizes directly" per the
   algorithm's source).
4. **Threshold + classify each near-zero eigenvector.** An eigenvector `(ω, v)` (itself unit-norm
   over its 6 components) is:
   - **pure translation** if `|ω|` is ~zero — direction is `v`, normalized.
   - **pure rotation** if `v`'s component along `ω` is ~zero — the surface is a rotation about an
     axis through `q = (ω × v) / |ω|²` (the canonical, axis-perpendicular choice of `q`), direction
     `ω / |ω|`.
   - **helix (screw)** otherwise — same axis-point formula using only `v`'s component
     perpendicular to `ω`; pitch (translation per radian) is `v`'s parallel component divided by
     `|ω|`.
5. **Count + combine.** The counts and sub-kinds of the slippable eigenvectors determine the
   region's kind directly, per Gelfand & Guibas:
   - 3 slippable, 2 translations + 1 rotation → **plane** (rotation axis = plane normal)
   - 3 slippable, 3 rotations (about the same center) → **sphere**
   - 2 slippable, 1 rotation + 1 translation along the SAME axis → **cylinder**
   - 1 slippable, pure translation → **extrusion**
   - 1 slippable, pure rotation → **revolution**
   - 1 slippable, helix → **helix**
   - anything else → **freeform**
6. **Convert back to real space.** Axis points and pitch were derived in the normalized frame;
   points transform back via the inverse of step 2's affine map, and pitch (a length) scales by
   `1/scale`. Direction vectors are untouched (normalization has no rotational component).

## Confidence

`SlippageResult.confidence` is the eigenvalue-ratio gap straddling the slippable/non-slippable
boundary, scaled by the slip threshold and clamped to `[0, 1]`: a wide gap (slippable modes
near-exactly zero, the rest clearly not) means a confident classification; a gap barely past the
threshold means the boundary itself is close to arbitrary. It is a diagnostic, not a probability.

## Determinism

Fully deterministic: `uniformSample`'s even-stride subsample, the barycentric area weighting, and
`Linalg.eigenSymmetric`'s classical (largest-off-diagonal-element) Jacobi sweep are all free of
unordered-collection iteration or randomness.

## Test fixture notes

A coarse lat/long UV sphere (or a cone built as a single fan from one apex vertex) is a poor test
fixture: pole clustering / too few distinct sample rings can leave the discrete covariance's
near-zero eigenvalues one to three orders of magnitude away from where a genuinely continuous
surface would put them, right at the slip threshold's edge. `Tests/OCCTSwiftMeshTests`'s fixtures
use a fine UV sphere and a multi-ring cone for exactly this reason — see `MeshFixtures.swift`.
