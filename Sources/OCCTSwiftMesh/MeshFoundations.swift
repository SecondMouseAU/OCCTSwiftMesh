// MeshFoundations — weld, adjacency, normals, components, sub-mesh, boundary loops (#16).
//
// Everything adjacency-based (region growing, curvature, boundary tracing) needs a WELDED
// substrate first: OCCT tessellation and STL loading produce (near-)unshared vertices in
// practice (3 verts per triangle, no sharing), so a raw import has no shared edges at all.
// `welded(tolerance:)` is the entry point every other function here assumes has already run.
//
// Ported from OCCTReconstruct's `ReconstructCompute` layer (`IndexedMesh`, `SubMesh.swift`,
// `MeshSegmentation.swift`'s `connectedComponents()`) — pure Swift + simd, same lineage, no
// third-party entanglement. Adapted from the `IndexedMesh` value type onto `OCCTSwift.Mesh`.

import simd
import OCCTSwift

public extension Mesh {

    /// Merge coincident vertices within `tolerance` (grid-hash weld) and drop triangles that
    /// become degenerate as a result. `tolerance` of `0` auto-derives `1e-6 × bboxDiagonal`,
    /// matching `crossSection`'s weld default.
    ///
    /// - Returns: the welded mesh, or `self` unchanged if welding would leave no valid triangles
    ///   (e.g. a degenerate 1- or 2-triangle input).
    func welded(tolerance: Double = 0) -> Mesh {
        let verts = vertices
        guard !verts.isEmpty else { return self }
        var lo = verts[0], hi = verts[0]
        for p in verts { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let diag = Double(simd_length(hi - lo))
        let tol = tolerance > 0 ? tolerance : max(1e-9, 1e-6 * diag)

        struct GridKey: Hashable { var x: Int64; var y: Int64; var z: Int64 }
        var indexForKey: [GridKey: UInt32] = [:]
        var newPositions: [SIMD3<Float>] = []
        var remap = [UInt32](repeating: 0, count: verts.count)
        for (i, p) in verts.enumerated() {
            let key = GridKey(x: Int64((Double(p.x) / tol).rounded()),
                               y: Int64((Double(p.y) / tol).rounded()),
                               z: Int64((Double(p.z) / tol).rounded()))
            if let existing = indexForKey[key] {
                remap[i] = existing
            } else {
                let newIndex = UInt32(newPositions.count)
                indexForKey[key] = newIndex
                newPositions.append(p)
                remap[i] = newIndex
            }
        }

        let oldIndices = indices
        var newIndices: [UInt32] = []
        newIndices.reserveCapacity(oldIndices.count)
        var i = 0
        while i + 2 < oldIndices.count {
            let a = remap[Int(oldIndices[i])], b = remap[Int(oldIndices[i + 1])], c = remap[Int(oldIndices[i + 2])]
            i += 3
            if a == b || b == c || a == c { continue }   // degenerate after weld
            newIndices.append(a); newIndices.append(b); newIndices.append(c)
        }
        guard let out = Mesh(vertices: newPositions, indices: newIndices) else { return self }
        return out
    }

    /// Unit face normals, one per triangle (a zero-area triangle gets a +Z fallback).
    func faceNormals() -> [SIMD3<Float>] {
        let verts = vertices
        let idx = indices
        var normals: [SIMD3<Float>] = []
        normals.reserveCapacity(idx.count / 3)
        var i = 0
        while i + 2 < idx.count {
            let a = verts[Int(idx[i])], b = verts[Int(idx[i + 1])], c = verts[Int(idx[i + 2])]
            let n = simd_cross(b - a, c - a)
            let len = simd_length(n)
            normals.append(len > 1e-12 ? n / len : SIMD3<Float>(0, 0, 1))
            i += 3
        }
        return normals
    }

    /// Area-weighted per-vertex normals (unit length). Requires welded input to be meaningful —
    /// on unwelded input every vertex is used by exactly one triangle, so this degenerates to
    /// `faceNormals()` repeated per corner.
    func vertexNormals() -> [SIMD3<Float>] {
        let verts = vertices
        let idx = indices
        var accum = [SIMD3<Float>](repeating: .zero, count: verts.count)
        var i = 0
        while i + 2 < idx.count {
            let ia = Int(idx[i]), ib = Int(idx[i + 1]), ic = Int(idx[i + 2])
            let a = verts[ia], b = verts[ib], c = verts[ic]
            let cross = simd_cross(b - a, c - a)   // magnitude = 2x triangle area — area weighting for free
            accum[ia] += cross; accum[ib] += cross; accum[ic] += cross
            i += 3
        }
        return accum.map { v in
            let len = simd_length(v)
            return len > 1e-12 ? v / len : SIMD3<Float>(0, 0, 1)
        }
    }

    /// Per-triangle adjacency: the triangles sharing a welded edge with each triangle. Built from
    /// an undirected edge → triangles map, so it needs `welded` input to find any neighbours at
    /// all (an unwelded soup has zero shared edges).
    func triangleAdjacency() -> [[Int]] {
        let idx = indices
        let tc = triangleCount
        var edgeMap: [UInt64: [Int]] = [:]
        func key(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        for t in 0..<tc {
            let a = idx[t * 3], b = idx[t * 3 + 1], c = idx[t * 3 + 2]
            for e in [key(a, b), key(b, c), key(c, a)] { edgeMap[e, default: []].append(t) }
        }
        var adjacency = [[Int]](repeating: [], count: tc)
        for (_, tris) in edgeMap where tris.count > 1 {
            for i in 0..<tris.count {
                for j in (i + 1)..<tris.count {
                    adjacency[tris[i]].append(tris[j])
                    adjacency[tris[j]].append(tris[i])
                }
            }
        }
        // `edgeMap` is a Dictionary: iterating it visits a triangle's (up to 3) edges in
        // hash-bucket order, not a reproducible one. Left unsorted, that leaks into every
        // consumer that walks `adjacency[t]` in order (region-growing DFS, in particular) —
        // determinism requires a FIXED tie-break, not "whatever order this ran in", so every
        // per-triangle neighbour list is sorted ascending before it's handed out.
        for t in 0..<tc { adjacency[t].sort() }
        return adjacency
    }

    /// Extract a subset of triangles as a standalone mesh, with vertices remapped to a compact
    /// range starting at 0. Used to isolate a region/zone/component for downstream measurement.
    ///
    /// - Returns: `nil` if `triangleIndices` is empty or resolves to zero valid triangles.
    func subMesh(triangleIndices: [Int]) -> Mesh? {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        var remap: [UInt32: UInt32] = [:]
        var newPositions: [SIMD3<Float>] = []
        var newIndices: [UInt32] = []
        newIndices.reserveCapacity(triangleIndices.count * 3)
        for t in triangleIndices {
            guard t >= 0, t < tc else { continue }
            for k in 0..<3 {
                let g = idx[t * 3 + k]
                let local: UInt32
                if let m = remap[g] {
                    local = m
                } else {
                    local = UInt32(newPositions.count)
                    remap[g] = local
                    newPositions.append(verts[Int(g)])
                }
                newIndices.append(local)
            }
        }
        return Mesh(vertices: newPositions, indices: newIndices)
    }

    /// Split into connected components (physically disjoint pieces) by union-find over shared
    /// (welded) vertices. Components are returned largest-first, deterministically.
    func connectedComponents() -> [MeshRegion] {
        let tc = triangleCount
        guard tc > 0 else { return [] }
        let idx = indices
        var parent = Array(0..<vertexCount)
        func find(_ x: Int) -> Int {
            var r = x
            while parent[r] != r { r = parent[r] }
            var c = x
            while parent[c] != c { let next = parent[c]; parent[c] = r; c = next }
            return r
        }
        func union(_ a: Int, _ b: Int) {
            let ra = find(a), rb = find(b)
            if ra != rb { parent[ra] = rb }
        }
        for t in 0..<tc {
            let a = Int(idx[t * 3]), b = Int(idx[t * 3 + 1]), c = Int(idx[t * 3 + 2])
            union(a, b); union(b, c)
        }

        var buckets: [Int: [Int]] = [:]
        for t in 0..<tc { buckets[find(Int(idx[t * 3])), default: []].append(t) }
        let keys = Array(buckets.keys)   // fixed order for this call; final result is re-sorted deterministically below

        let normals = faceNormals()
        let adjacency = triangleAdjacency()
        var regionOf = [Int](repeating: -1, count: tc)
        for (i, k) in keys.enumerated() { for t in buckets[k]! { regionOf[t] = i } }
        let loopCounts = MeshRegion.boundaryLoopCounts(mesh: self, regionOf: regionOf, regionCount: keys.count,
                                                        adjacency: adjacency)

        var regions: [MeshRegion] = []
        regions.reserveCapacity(keys.count)
        for (i, k) in keys.enumerated() {
            regions.append(MeshRegion.build(mesh: self, triangleIndices: buckets[k]!, faceNormals: normals,
                                             boundaryLoopCount: loopCounts[i]))
        }
        return regions.sorted(by: MeshRegion.order)
    }

    /// Open boundary loops: closed rings of edges shared by exactly one triangle (a mesh boundary
    /// edge — either the mesh is genuinely open there, or non-manifold input pinches at that
    /// edge). Returned largest-first, deterministically.
    func boundaryLoops() -> [[UInt32]] {
        let idx = indices
        let tc = triangleCount
        func key(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        var edgeCount: [UInt64: Int] = [:]
        var endpoints: [UInt64: (UInt32, UInt32)] = [:]
        for t in 0..<tc {
            let a = idx[t * 3], b = idx[t * 3 + 1], c = idx[t * 3 + 2]
            for (x, y) in [(a, b), (b, c), (c, a)] {
                let k = key(x, y)
                edgeCount[k, default: 0] += 1
                if endpoints[k] == nil { endpoints[k] = (x, y) }
            }
        }
        let boundaryEdges = edgeCount.compactMap { k, cnt -> (UInt32, UInt32)? in cnt == 1 ? endpoints[k] : nil }
        return Loops.trace(edges: boundaryEdges).sorted { a, b in
            a.count != b.count ? a.count > b.count : (a.first ?? 0) < (b.first ?? 0)
        }
    }
}
