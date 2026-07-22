// Mesh+Crease.swift — dihedral-fold edge detection and ring/path chaining.
//
// Finds edges whose fold angle (the dihedral angle between the two triangles sharing that edge)
// exceeds a threshold, then chains them into rings (closed loops, e.g. a door outline) and paths
// (open chains, e.g. a crease that runs off an open mesh boundary) — outlining recessed/raised
// features (doors, panels, window returns) on raw scan meshes where BREP feature recognition
// does not exist.
//
// Like `triangleAdjacency()`/`boundaryLoops()` (the "weld precondition family" — see
// Mesh+Topology.swift's header), this requires WELDED input: on unwelded (per-triangle-unique)
// input every edge is used by exactly one triangle, so the dihedral angle is undefined and every
// edge would come back "boundary," not "crease."

import simd
import OCCTSwift

extension Mesh {

    /// Find dihedral-fold edges (fold angle >= `minAngleDegrees`) and chain them into rings
    /// (closed loops) and paths (open chains), longest first.
    ///
    /// Requires a WELDED mesh — see the file header. Only edges shared by EXACTLY two triangles
    /// have a well-defined fold angle (a boundary edge or a non-manifold edge does not), the same
    /// restriction `MeshIntegrityReport.isOrientable` applies.
    ///
    /// Chaining follows `boundaryLoops()`'s exact determinism discipline (sorted seed order,
    /// sorted-neighbour walk) with one addition: junctions where 3+ crease edges meet a single
    /// vertex are never wandered through. A walk always stops the instant it reaches a junction
    /// (or closes there, if the junction is itself the ring's own start), so a Y- or T-shaped
    /// crease intersection splits deterministically into distinct rings/paths instead of picking
    /// an arbitrary continuation through the junction.
    ///
    /// - Parameter minAngleDegrees: dihedral fold-angle threshold, in degrees.
    public func creaseEdges(minAngleDegrees: Float = 30) -> CreaseDetectionResult {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        guard tc > 0, !verts.isEmpty else {
            return CreaseDetectionResult(rings: [], unchainedCreaseEdgeCount: 0)
        }

        let normals = Mesh.faceNormals(vertices: verts, indices: idx, triangleCount: tc)

        func ekey(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        var edgeTriangles: [UInt64: [Int]] = [:]
        for t in 0..<tc {
            let base = t * 3
            let a = idx[base], b = idx[base + 1], c = idx[base + 2]
            for e in [ekey(a, b), ekey(b, c), ekey(c, a)] { edgeTriangles[e, default: []].append(t) }
        }

        // Only 2-triangle (manifold) edges have a well-defined dihedral fold angle.
        var creaseAngle: [UInt64: Double] = [:]
        for (e, tris) in edgeTriangles where tris.count == 2 {
            let d = Double(max(-1, min(1, simd_dot(normals[tris[0]], normals[tris[1]]))))
            let angle = acos(d) * 180 / .pi
            if angle >= Double(minAngleDegrees) { creaseAngle[e] = angle }
        }
        guard !creaseAngle.isEmpty else {
            return CreaseDetectionResult(rings: [], unchainedCreaseEdgeCount: 0)
        }
        let totalCreaseEdges = creaseAngle.count

        var nbr: [UInt32: [UInt32]] = [:]
        for e in creaseAngle.keys {
            let a = UInt32(e >> 32), b = UInt32(e & 0xffff_ffff)
            nbr[a, default: []].append(b)
            nbr[b, default: []].append(a)
        }
        // DETERMINISM: same discipline as boundaryLoops() — sort neighbour lists and iterate
        // vertices/edges in sorted order, since Dictionary/Set iteration order is not.
        for k in Array(nbr.keys) { nbr[k]?.sort() }
        func degree(_ v: UInt32) -> Int { nbr[v]?.count ?? 0 }

        var remaining = Set(creaseAngle.keys)
        var rings: [CreaseRing] = []
        var unchainedCount = 0

        func makeRing(_ chain: [UInt32], closed: Bool) -> CreaseRing {
            let edgeCount = closed ? chain.count : chain.count - 1
            var length = 0.0
            var angleSum = 0.0
            var angleMax = 0.0
            for i in 0..<max(0, edgeCount) {
                let a = chain[i], b = chain[(i + 1) % chain.count]
                length += Double(simd_distance(verts[Int(a)], verts[Int(b)]))
                let ang = creaseAngle[ekey(a, b)] ?? 0
                angleSum += ang
                angleMax = max(angleMax, ang)
            }
            var lo = verts[Int(chain[0])], hi = verts[Int(chain[0])]
            for v in chain { lo = simd_min(lo, verts[Int(v)]); hi = simd_max(hi, verts[Int(v)]) }
            let mean = edgeCount > 0 ? angleSum / Double(edgeCount) : 0
            return CreaseRing(vertexIndices: chain, closed: closed, length: length,
                              bbox: (lo, hi), meanFoldAngleDegrees: mean, maxFoldAngleDegrees: angleMax)
        }
        // Defensive walk-length backstop shared by both passes below — mirrors boundaryLoops()'s
        // `loop.count > boundary.count + 2` cap, sized off the FIXED total crease-edge count
        // (never the shrinking `remaining`), so a genuine long chain across most of the mesh's
        // creases is never mistaken for a runaway walk.
        let walkCap = totalCreaseEdges + 2

        // Pass 1: chain from every "special" vertex (degree != 2 — an open end or a 3+-way
        // junction) along each of its own incident crease edges, stopping the INSTANT the walk
        // reaches another special vertex (never wandering through a junction — see the doc
        // comment above).
        let specialVertices = nbr.keys.filter { degree($0) != 2 }.sorted()
        for s in specialVertices {
            for firstStep in nbr[s] ?? [] where remaining.contains(ekey(s, firstStep)) {
                var chain = [s, firstStep]
                var consumed = [ekey(s, firstStep)]
                remaining.remove(ekey(s, firstStep))
                var prev = s, cur = firstStep
                var capped = false
                while degree(cur) == 2 {
                    guard let nxt = (nbr[cur] ?? []).first(where: { $0 != prev }),
                          remaining.contains(ekey(cur, nxt)) else { break }
                    remaining.remove(ekey(cur, nxt))
                    consumed.append(ekey(cur, nxt))
                    chain.append(nxt)
                    prev = cur
                    cur = nxt
                    if consumed.count > walkCap { capped = true; break }
                }
                if capped {
                    unchainedCount += consumed.count
                    continue
                }
                let closed = (cur == s)
                if closed && chain.count < 3 {
                    unchainedCount += consumed.count   // pathological duplicate-edge case
                } else {
                    rings.append(makeRing(chain, closed: closed))
                }
            }
        }

        // Pass 2: whatever's left touches only degree-2 vertices — pure closed loops with no
        // junction anywhere along them. Same seed/walk discipline as `boundaryLoops()`.
        for seed in remaining.sorted() where remaining.contains(seed) {
            let sa = UInt32(seed >> 32), sb = UInt32(seed & 0xffff_ffff)
            remaining.remove(seed)
            var consumed = [seed]
            var chain = [sa, sb]
            var prev = sa, cur = sb
            var didClose = false
            var capped = false
            while true {
                guard let nxt = (nbr[cur] ?? []).first(where: { $0 != prev && remaining.contains(ekey(cur, $0)) })
                else { break }
                let ek = ekey(cur, nxt)
                remaining.remove(ek)
                consumed.append(ek)
                if nxt == sa { didClose = true; break }
                chain.append(nxt)
                prev = cur
                cur = nxt
                if consumed.count > walkCap { capped = true; break }
            }
            if didClose, !capped, chain.count >= 3 {
                rings.append(makeRing(chain, closed: true))
            } else {
                unchainedCount += consumed.count
            }
        }

        // Defensive: everything should have been claimed by pass 1 or 2 (every crease edge
        // touches at least one degree-2-or-not vertex, and both passes together enumerate every
        // vertex kind) — but report, never silently drop, anything a subtle edge case still left
        // behind.
        unchainedCount += remaining.count

        return CreaseDetectionResult(rings: rings.sorted(by: CreaseRing.order),
                                     unchainedCreaseEdgeCount: unchainedCount)
    }
}
