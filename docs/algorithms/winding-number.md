# Generalized winding number (`Mesh.windingNumber(at:)`, `Mesh.orientationReport(samples:)`)

Jacobson, Kavan, Sorkine-Hornung, "Robust Inside-Out Segmentation Using Generalized Winding
Numbers" (SIGGRAPH 2013). Pure Swift + simd, no OCCT kernel calls, no welding precondition — every
triangle contributes independently regardless of adjacency.

## Why not parity/ray tests

The classical point-in-polygon tests (even-odd ray crossing counts) need a closed,
non-self-intersecting surface to mean anything at all, and degrade unpredictably — wrong answers,
not just imprecise ones — on the open shells and self-intersecting soup that real scan meshes
actually are. The generalized winding number is the direct solid-angle sum instead:

```
w(p) = (1 / 4π) · Σ over triangles of signedSolidAngle(p, triangle)
```

For a closed, coherently-oriented mesh this recovers the usual indicator function exactly
(`w ≈ 1` inside, `w ≈ 0` outside) via the divergence theorem — but stays a well-defined, smooth
real number (not an undefined or arbitrary result) on open shells and self-intersecting input too.

## `windingNumber(at:)` — the per-triangle formula

Each triangle's contribution uses the van Oosterom–Strackee (1983) solid-angle formula: for a
triangle with corners `a, b, c` and vectors `A = a - p`, `B = b - p`, `C = c - p`,

```
numerator   = A · (B × C)
denominator = |A||B||C| + (A·B)|C| + (B·C)|A| + (C·A)|B|
contribution = 2 · atan2(numerator, denominator)
```

summed and divided by `4π`. This is numerically robust (no coordinate-frame branch, unlike naive
spherical-excess formulas) and exact up to floating-point rounding. `windingNumber` is
O(triangleCount) per call, no spatial acceleration — fine at diagnostic sample counts (a few dozen
points × a few hundred thousand triangles); a hierarchical (Barnes-Hut-style) evaluation is the
scale-up path for high-frequency sampling, not implemented here (per the tracking issue).

## Linearity in orientation

Swapping two of a triangle's three vertices negates its contribution — a direct algebraic
consequence of the cross product's antisymmetry (`A · (C × B) = -A · (B × C)`). Reversing EVERY
triangle's winding therefore negates `w(p)` at every point `p`, everywhere, unconditionally:

```
w_reversed(p) = -w_original(p),  for all p
```

This is a pure consequence of linearity, true regardless of the mesh's shape, openness, or
self-intersection — `WindingNumberTests.reversalNegatesEverywhere` checks it directly.

## `orientationReport(samples:)` — and its closed-mesh limitation

Samples `windingNumber` at points offset outward from the mesh's own bounding box (a deterministic
Fibonacci-sphere spiral around the bbox center, at a radius comfortably beyond the bbox's own
circumscribing sphere — no randomness, so repeat calls agree exactly), and reports the mean plus a
`looksInverted` heuristic (mean below a floor calibrated well clear of float noise).

**The important, easy-to-miss caveat, direct from the linearity fact above:** for a genuinely
closed, watertight mesh, the winding number at any point strictly outside every enclosed volume is
EXACTLY `0` (the same divergence-theorem fact that makes the method work at all) — regardless of
orientation, since `0` negated is still `0`. Global inversion instead flips the INTERIOR reading
(`≈ 1` → `≈ -1`), which this exterior-only diagnostic never samples. **This means
`orientationReport` is provably powerless to detect inversion on a closed, watertight solid** — a
caller who needs to check one should sample a point known to be INSIDE it instead (e.g. its
centroid, for a reasonably convex/blob-shaped body).

Where this diagnostic IS useful — its primary intended purpose, matching the OCCTMCP deviation
suite's original motivation (upgrading the `ambiguousFraction`-near-1.0 heuristic) — is an OPEN
shell: a raw scan surface patch with no enclosed volume at all has no such cancellation guarantee,
so the winding number away from the shell is generically nonzero and fractional, and its sign
genuinely tracks which way the shell's visible normals face relative to the sample points. Even
there, a single generic point (or even a full spherical sweep of exterior points) can land
anywhere from a strong signal to near-total front/back cancellation depending on the shell's own
shape and the sample points' placement relative to it — `meanExteriorWinding` reports a robust
aggregate for exactly this reason, but is not guaranteed to be far from zero for every open shell
under generic sampling. `WindingNumberTests.openShellHollowCenterReadsNonzero` demonstrates a
clearly-informative case directly (a point at an open tube's own hollow center, which "sees" the
whole wall from inside — a large, one-signed solid angle), rather than relying on
`orientationReport`'s generic bbox-exterior sampling to happen to find one.

## Relationship to `integrityReport().isOrientable`

Complementary, not overlapping: `isOrientable` checks CONSISTENCY (does every 2-triangle edge get
traversed in opposite directions by its two triangles?) — it catches a mix of correctly- and
incorrectly-wound triangles within the same mesh, but says nothing about whether a GLOBALLY
consistent winding happens to be inside-out. `windingNumber`/`orientationReport` is the reverse:
it says nothing about local consistency, but (on a mesh sampled at a point where it IS
informative — see above) can catch a globally-consistent-but-inverted winding that `isOrientable`
would report as perfectly fine (every edge still traversed in opposite directions by its two
triangles — it's just that "opposite directions" is uniformly backwards).

## Determinism

The Fibonacci-sphere sample directions are a fixed, index-derived sequence (no randomness), and
`windingNumber`'s own per-triangle sum has no unordered-collection iteration — repeat calls on
identical input agree exactly.
