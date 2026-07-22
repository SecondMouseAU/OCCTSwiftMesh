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

## `orientationReport(samples:)` — two sampling regimes

`orientationReport` branches on `boundaryLoops().isEmpty` (closed vs. open) into two genuinely
different sampling strategies, because a single strategy cannot serve both cases — see "Why a
single far-exterior sweep doesn't work" below for the (initially shipped, then reverted) approach
that looked reasonable but structurally couldn't detect anything on an open shell.

**Closed mesh** (`exteriorSampleMean`): samples `windingNumber` at points offset outward from the
mesh's own bounding box (a deterministic Fibonacci-sphere spiral around the bbox center, at a
radius comfortably beyond the bbox's own circumscribing sphere). **The important, easy-to-miss
caveat, direct from the linearity fact above:** for a genuinely closed, watertight mesh, the
winding number at any point strictly outside every enclosed volume is EXACTLY `0` (the same
divergence-theorem fact that makes the method work at all) — regardless of orientation, since `0`
negated is still `0`. Global inversion instead flips the INTERIOR reading (`≈ 1` → `≈ -1`), which
this exterior-only sampling never sees. **This means `orientationReport` is provably powerless to
detect inversion on a closed, watertight solid** — a caller who needs to check one should sample a
point known to be INSIDE it instead (e.g. its centroid, for a reasonably convex/blob-shaped body).
This branch exists specifically so that documented limitation stays literally true, rather than
becoming accidentally (and inconsistently) informative through whatever the open-shell strategy
happens to do to a closed mesh.

**Open shell** (`hollowSampleMean`): samples clustered around the AREA-WEIGHTED CENTROID of the
shell's own triangles, jittered by a small deterministic Fibonacci-sphere pattern (a modest
fraction of the bbox half-diagonal, not the far-exterior sweep's large multiple). The centroid is
derived ONLY from vertex positions — never from face normals — which matters for a subtle but
important reason: the probe LOCATION must be identical for a mesh and its reversal, or the
reversed mesh's own flipped normals could bias where its probes land, breaking the clean
`w_reversed(p) = -w_original(p)` argument (see "orientation-derived probes are tautological"
below for what goes wrong if a probe location depends on the very orientation being tested). For a
shell shaped like a bowl, dome, or tube, that centroid sits inside the concavity — where the
winding number is large in magnitude and reliably signed, exactly the mechanism
`WindingNumberTests.openShellHollowCenterReadsNonzero` demonstrates directly (a point at an open
tube's own hollow center "sees" the whole wall from inside — a large, one-signed solid angle).
`WindingNumberTests.open{Dome,Tube}InversionIsDetected` are the flagship positive cases: an
inverted open dome and an inverted open tube both flag `looksInverted == true`; their
correctly-oriented twins don't.

## Why a single far-exterior sweep doesn't work

An earlier version of this method used ONE strategy for both regimes: sample far outside the
bounding box, on a full surrounding Fibonacci sphere, for every mesh. That's correct — necessary,
even — for the closed-mesh case, but is a structural dead end for open shells, not merely an
undertuned threshold: averaged over a FULL surrounding sphere, directions roughly facing the
shell's front read positive and directions facing its back read negative by (almost) the same
amount, so the mean collapses toward zero REGARDLESS of which way the shell is actually wound — a
70°-cap dome measured this way read `±3.7e-5` for both orientations, five orders of magnitude
short of any reasonable threshold. No retuning the threshold fixes an average that cancels by
construction; the sampling itself had to move to where the signal actually lives (the shell's own
hollow, not its far field).

## Orientation-derived probes are tautological

A tempting alternative for the open-shell regime: offset each sample a small distance from a
mesh triangle's own centroid, backward along THAT triangle's own face normal (an "into the local
surface" probe). This looks plausible — near a flat patch, the winding number does read a large,
reliably-signed local value — but it's a dead end: the probe LOCATION itself depends on the
triangle's own (potentially flipped) normal, so under a full mesh reversal the probe silently
moves to the OPPOSITE physical point too. Working through the algebra, `windingNumber` at the
reversed mesh's own (relocated) probe converges back to very nearly the SAME value the original
mesh's probe read — the measurement ends up telling you "which side my own normal happens to
point away from," which is always true by construction and says nothing about whether that normal
is globally correct. Any viable probe-placement rule must depend only on the mesh's SHAPE (vertex
positions), never on face normals or vertex winding order, or it risks this same self-referential
trap. `hollowSampleMean`'s area-weighted centroid is shape-only for exactly this reason.

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
