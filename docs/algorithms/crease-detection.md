# Crease-edge detection (`Mesh.creaseEdges(minAngleDegrees:)`)

Finds edges whose dihedral fold angle (the angle between the two triangles sharing that edge)
exceeds a threshold, then chains them into rings (closed loops, e.g. a door outline) and paths
(open chains, e.g. a crease that runs off an open mesh boundary) — outlining recessed/raised
features (doors, panels, window returns) on raw scan meshes where BREP feature recognition does
not exist. Pure Swift + simd, no OCCT kernel calls.

Like `triangleAdjacency()`/`boundaryLoops()` (the "weld precondition family" — see
`Mesh+Topology.swift`'s header), this requires WELDED input: on unwelded (per-triangle-unique)
input every edge is used by exactly one triangle, so the dihedral angle is undefined and every
edge comes back "boundary," not "crease."

## Algorithm sketch

1. **Find crease edges.** Only edges shared by EXACTLY two triangles have a well-defined dihedral
   fold angle — a boundary edge (one triangle) or a non-manifold edge (3+) does not, the same
   restriction `MeshIntegrityReport.isOrientable` applies. An edge's fold angle is
   `acos(dot(n1, n2))` between its two triangles' face normals; it's a crease iff that angle
   `>= minAngleDegrees`.
2. **Chain into rings/paths.** Build the crease-edge graph (nodes = vertices touching at least
   one crease edge) and classify each vertex's degree in that graph:
   - degree 1: an open end (a crease running off an open mesh boundary).
   - degree 2: an ordinary chain vertex — passed through.
   - degree 3+: a junction (3+ creases meeting, e.g. a T- or Y-shaped intersection).

   **Pass 1** walks from every non-degree-2 vertex (sorted) along each of its own incident crease
   edges (sorted), advancing through degree-2 vertices and stopping the INSTANT the walk reaches
   another non-degree-2 vertex — including possibly the SAME vertex it started from, which closes
   a ring whose one junction point is itself (a "lasso"). This is the crux of the design: a walk
   never wanders through a junction picking an arbitrary continuation, because the stopping
   condition is checked before ever considering which edge to take next past a junction. **Pass 2**
   handles whatever's left — by the end of pass 1, every crease edge touching at least one
   non-degree-2 vertex has been consumed (attempted from that vertex, whether or not it was
   already claimed by a different vertex's walk first), so the remainder can only be pure closed
   loops made entirely of degree-2 vertices; these are seeded and walked exactly like
   `boundaryLoops()`.
3. **Aggregate per-ring stats.** `length` (sum of edge lengths), `bbox`, `meanFoldAngleDegrees`/
   `maxFoldAngleDegrees` (over the ring's own constituent edges) — computed once the full
   vertex chain is known.

## Never-silent: `unchainedCreaseEdgeCount`

Both passes carry a defensive walk-length cap (`totalCreaseEdges + 2`, mirroring
`boundaryLoops()`'s own `boundary.count + 2` cap) as a backstop against a runaway walk. Given the
pass-1/pass-2 split above, every crease edge is provably claimed by one pass or the other on
well-formed input — the cap is not expected to fire in practice. If it ever does (or if a
walk fails to close cleanly), those edges are counted in `unchainedCreaseEdgeCount` rather than
silently dropped or emitted as a nonsensical ring, the same `SegmentedMesh.truncatedTriangleCount`
discipline.

## Result shape

`Mesh.creaseEdges` returns a `CreaseDetectionResult` (not a bare `[CreaseRing]`) — the
`SegmentedMesh` convention of pairing the primary array with a never-silent diagnostic count,
rather than a bare array with nowhere to put `unchainedCreaseEdgeCount`.

`CreaseRing.length`/`bbox` are reported in the mesh's own coordinate units, not literally
millimetres, matching every other length-valued field in this package (`FittedPrimitive.radius`,
`AlignResult.residualRMS`, `SlippageResult.pitch`, …) — none of which carry a unit suffix, since
the package itself is unit-agnostic. (A unit-suffixed name was in the tracking issue's initial
sketch; consistency with the rest of the package's naming took priority.)

## Determinism

Seed order (sorted vertex/edge keys) and neighbour-pick order (sorted per vertex) are both
explicit, the exact discipline `boundaryLoops()` already established — Dictionary/Set iteration
order is never a substitute. `CreaseRing.order` (longest first, lowest-vertex-index tie-break)
mirrors `MeshRegion.order`'s convention for the same reason: unordered-collection-derived buckets
must not leak into the returned order.

## Test fixture notes

A generic XY-grid "raised mesa" (lift a rectangular block of grid vertices to test a stepped
feature) turns out to be a POOR fixture for exercising closed rings cleanly: the grid's fixed
diagonal-triangulation choice interacts with the height step at the mesa's four corners, creating
several additional, differently-angled crease edges right at the corners instead of one clean
90°-ish transition all the way around — useful for exercising junction/fragmentation handling, but
not the "two clean nested rings" case. `coarseCappedCylinderMesh` (a fan-triangulated cap sharing
an exact boundary ring with the barrel — no corner ambiguity at all) is the fixture that actually
demonstrates two clean 90° closed rings; see `CreaseDetectionTests.swift`.
