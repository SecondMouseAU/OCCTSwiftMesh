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
2. **Normalize to a unit box.** Points are centered and scaled — by a single ISOTROPIC factor, so
   the result stays rotation-invariant — so the bounding box's largest extent is `1` before the
   covariance is built. Eigenvalue thresholds are then scale-free ratios, not absolute magnitudes
   tied to the region's physical size. Normals need no normalization (already unit, direction is
   scale-invariant). See "Elongated regions" below for a consequence of this choice.
3. **Build and eigen-decompose the 6×6 covariance** (`Linalg.eigenSymmetric`, the same classical
   Jacobi method as `eigenSymmetric3`, generalized — "Jacobi generalizes directly" per the
   algorithm's source).
4. **Pick the slippable count `d`** by the largest spectral gap among candidates: the ratio just
   below the cut must stay under a ceiling (`0.02`), and the jump to the next ratio must clear
   both an absolute floor and a *relative* one (the next ratio must be several times the included
   one). A fixed threshold alone doesn't work — how far from exact zero a genuinely slippable
   eigenvalue sits depends on tessellation quality, and a freeform patch's smallest eigenvalue can
   land under any fixed cutoff purely because *something* has to be smallest in a six-number
   spread, not because it means anything. The relative-jump test is what tells those apart: a real
   near-zero cluster sits orders of magnitude below its neighbours; two merely-smallish freeform
   values don't.
5. **Classify the slippable SUBSPACE, not each eigenvector.** This is the crux of the design and
   the part most likely to look "obviously fine" and be wrong: see "Basis invariance" below.
   - `d == 1`: the null space is 1-D, unique up to sign, so the lone eigenvector's own
     rotational/translational split is classified directly — pure translation → **extrusion**,
     pure rotation → **revolution**, coupled rotation+translation → **helix** (pitch = the
     translation-per-radian along the shared axis).
   - `d >= 2`: build the 3×3 Gram matrix `G = Σ ωₖωₖᵀ` over the `d` slippable eigenvectors'
     rotational parts and eigen-decompose it (`eigenSymmetric3`). Its rank `r` (eigenvalues above
     a floor relative to `G`'s own largest) is the number of independent rotation axes the
     subspace contains, and — critically — is INVARIANT to which particular orthonormal basis of
     the subspace Jacobi happened to return. Classify by `(d, r)`:
     - `(3, 1)` → **plane** (the one rotation axis is the plane's normal; the other two slippable
       directions are in-plane translations)
     - `(3, 3)` → **sphere** (rotation about any axis, through one common center)
     - `(2, 1)` → **cylinder** (rotation about an axis + translation along that same axis)
     - anything else → **freeform**
6. **Recover the axis**, also via formulas invariant to the eigenvector basis (see "Basis
   invariance"):
   - Axis DIRECTION for `(3,1)`/`(2,1)`: `G`'s dominant eigenvector.
   - Axis POINT for `(2,1)` (cylinder): pick the subspace's largest-`|ω|` mode and compute
     `q = ω × v⊥ / |ω|²` (`v⊥` = that mode's `v` with the axis-parallel component removed). Any
     mode with nonzero `ω` gives the identical `q` — see the doc comment on
     `Mesh.slippageAxisPoint`.
   - Center for `(3,3)` (sphere): every slippable mode satisfies `v = -ω × q` for the SAME `q`
     (true for any combination of the true generators, since the relation is linear), so `q` is
     the least-squares solution of that system stacked over all `d` modes — NOT an average of each
     mode's own axis-foot independently, which is not the sphere's center for a non-canonical
     basis (it systematically pulls the estimate toward the origin). See
     `Mesh.slippageSphereCenter`.
7. **Convert back to real space.** Axis points and pitch were derived in the normalized frame;
   points transform back via the inverse of step 2's affine map, and pitch (a length) scales by
   `1/scale`. Direction vectors are untouched (normalization has no rotational component).

## Basis invariance

When the slippable null space is more than 1-dimensional (plane: 3-D, cylinder: 2-D, sphere:
3-D), the slippage constraint being LINEAR means any orthonormal basis of that subspace is an
equally valid set of eigenvectors — which particular basis Jacobi returns is decided by
tessellation noise and floating-point rounding, not by the surface. An axis-aligned test fixture's
null space happens to line up with R⁶'s coordinate axes, so each eigenvector comes out "pure" (a
lone translation or a lone rotation) purely by construction-time coincidence; under a generic pose
the basis mixes rotational and translational content within each eigenvector, and classifying each
eigenvector independently silently misreads a plane as a sphere, a cylinder as freeform, and so
on — while looking entirely correct against any axis-aligned fixture. The fix classifies the
subspace as a whole (step 5 above) using only quantities — the Gram matrix's rank/dominant
eigenvector, the axis-point and sphere-center formulas — that are provably unchanged by an
orthogonal change of basis within the subspace. `Tests/OCCTSwiftMeshTests/SlippageTests.swift`'s
`SlippagePoseInvarianceTests` puts every kind through a generic (non-axis-aligned, translated)
pose specifically to guard against this regressing.

## Elongated regions

Because step 2 normalizes by a single ISOTROPIC scale factor (never per-axis — that would break
rotation invariance), a region elongated far beyond its cross-section (e.g. a long thin extruded
beam, height ≫ cross-section size) has its cross-sectional extent shrink toward zero in normalized
coordinates. At extreme aspect ratios the cross-section's shape becomes a vanishingly small
perturbation and the region genuinely starts to APPROXIMATE a rotationally-symmetric shape in the
normalized frame — this is a real consequence of an isotropic-normalization, ratio-based design,
not a bug to route around per-axis. `MeshFixtures.triangularPrismLateralMesh`'s and
`helicoidStripMesh`'s default parameters are deliberately chosen to sit well clear of this regime;
their doc comments explain the specific tuning.

## Confidence

`SlippageResult.confidence` is the spectral gap found in step 4, scaled by the slip ceiling and
clamped to `[0, 1]`: a wide gap (slippable modes near-exactly zero, the rest clearly not) means a
confident classification; a gap barely past the floor means the boundary itself is close to
arbitrary. It is a diagnostic, not a probability.

## Determinism

Fully deterministic: `uniformSample`'s even-stride subsample, the barycentric area weighting, and
`Linalg.eigenSymmetric`'s classical (largest-off-diagonal-element) Jacobi sweep are all free of
unordered-collection iteration or randomness.

## Test fixture notes

A coarse lat/long UV sphere (or a cone built as a single fan from one apex vertex, or a prism
profile with only corner vertices and no interior-to-a-face samples) is a poor test fixture: pole
clustering / too few distinct sample rings / every sample sitting exactly on a crease between two
faces can all leave the discrete covariance's near-zero eigenvalues far from where a genuinely
continuous surface would put them. `Tests/OCCTSwiftMeshTests`'s fixtures use a fine UV sphere, a
multi-ring cone, and an edge-subdivided prism profile for exactly this reason — see
`MeshFixtures.swift`.
