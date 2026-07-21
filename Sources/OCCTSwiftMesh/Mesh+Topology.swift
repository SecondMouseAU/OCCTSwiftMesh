// Mesh+Topology.swift — the connectivity toolkit: face/vertex normals, triangle adjacency,
// connected components, sub-mesh extraction, and boundary loops.
//
// `triangleAdjacency()`, `connectedComponents()`, and `boundaryLoops()` all key their
// connectivity off shared VERTEX INDICES — they need a welded mesh (see `welded(tolerance:)`)
// to see real adjacency. On unwelded input (per-triangle-unique vertices, the common case for
// raw OCCT tessellation / STL import) no two triangles share an index, so each comes back
// isolated: every triangle its own component, every edge a "boundary." Weld first.

import simd
import OCCTSwift

extension Mesh {

    /// Unit face normals, one per triangle (a degenerate triangle gets a +Z fallback).
    /// Independent of welding — computed directly from each triangle's own three corners.
    public func faceNormals() -> [SIMD3<Float>] {
        Mesh.faceNormals(vertices: vertices, indices: indices, triangleCount: triangleCount)
    }

    static func faceNormals(vertices: [SIMD3<Float>], indices: [UInt32], triangleCount: Int) -> [SIMD3<Float>] {
        var out: [SIMD3<Float>] = []
        out.reserveCapacity(triangleCount)
        for t in 0..<triangleCount {
            let base = t * 3
            let a = vertices[Int(indices[base])]
            let b = vertices[Int(indices[base + 1])]
            let c = vertices[Int(indices[base + 2])]
            let n = simd_cross(b - a, c - a)
            let len = simd_length(n)
            out.append(len > 1e-12 ? n / len : SIMD3<Float>(0, 0, 1))
        }
        return out
    }

    /// Area-weighted per-vertex normals, summed from adjacent triangles' face normals.
    ///
    /// Meaningful smoothing requires a WELDED mesh (call `.welded()` first) — on unwelded
    /// input each vertex touches exactly one triangle, so the result is just that triangle's
    /// flat face normal repeated per corner. Independent of `Mesh.normals`, which may come
    /// from the source B-Rep face tessellation rather than this mesh's own topology.
    public func vertexNormals() -> [SIMD3<Float>] {
        let verts = vertices
        let idx = indices
        var normals = [SIMD3<Float>](repeating: .zero, count: verts.count)
        var tri = 0
        while tri + 2 < idx.count {
            let ia = idx[tri], ib = idx[tri + 1], ic = idx[tri + 2]
            tri += 3
            let a = verts[Int(ia)], b = verts[Int(ib)], c = verts[Int(ic)]
            let faceNormal = simd_cross(b - a, c - a)   // magnitude = 2·area (area weighting)
            normals[Int(ia)] += faceNormal
            normals[Int(ib)] += faceNormal
            normals[Int(ic)] += faceNormal
        }
        for i in normals.indices {
            let len = simd_length(normals[i])
            normals[i] = len > 1e-12 ? normals[i] / len : SIMD3<Float>(0, 0, 1)
        }
        return normals
    }

    /// Per-triangle adjacency: the indices of triangles sharing a WELDED edge with each
    /// triangle, each sorted ascending. Requires `.welded()` first — see the file header.
    public func triangleAdjacency() -> [[Int]] {
        Mesh.triangleAdjacency(indices: indices, triangleCount: triangleCount)
    }

    static func triangleAdjacency(indices: [UInt32], triangleCount: Int) -> [[Int]] {
        var edgeMap: [UInt64: [Int]] = [:]
        func key(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        for t in 0..<triangleCount {
            let base = t * 3
            let a = indices[base], b = indices[base + 1], c = indices[base + 2]
            for e in [key(a, b), key(b, c), key(c, a)] { edgeMap[e, default: []].append(t) }
        }
        var adjacency = [[Int]](repeating: [], count: triangleCount)
        for (_, tris) in edgeMap where tris.count > 1 {
            for i in 0..<tris.count {
                for j in (i + 1)..<tris.count {
                    adjacency[tris[i]].append(tris[j])
                    adjacency[tris[j]].append(tris[i])
                }
            }
        }
        // DETERMINISM: edgeMap is a Dictionary, whose iteration order Swift randomizes per
        // process (a fresh hash seed each run) — without this sort, a triangle's neighbour
        // list comes back in a different ORDER each run (same set, different order), which
        // leaks into segmentSmoothRegions' DFS visit order and makes MeshRegion.triangleIndices
        // non-reproducible between runs on identical input.
        for i in adjacency.indices { adjacency[i].sort() }
        return adjacency
    }

    /// Split into connected components (physically disjoint pieces) by union-find over shared
    /// WELDED vertex indices. Requires `.welded()` first — see the file header. Components are
    /// returned largest-first (`MeshRegion`'s deterministic ordering).
    public func connectedComponents() -> [MeshRegion] {
        let tc = triangleCount
        guard tc > 0 else { return [] }
        let idx = indices
        let verts = vertices

        var parent = Array(0..<verts.count)
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
            let base = t * 3
            union(Int(idx[base]), Int(idx[base + 1]))
            union(Int(idx[base + 1]), Int(idx[base + 2]))
        }

        var buckets: [Int: [Int]] = [:]
        for t in 0..<tc {
            buckets[find(Int(idx[t * 3])), default: []].append(t)
        }
        return buckets.values
            .map { tris in MeshRegion(triangleIndices: tris, area: Mesh.area(ofTriangles: tris, vertices: verts, indices: idx)) }
            .sorted(by: MeshRegion.order)
    }

    /// Extract a subset of triangles as a standalone mesh, with vertices remapped to a
    /// compact range — used to isolate a region or component for separate processing.
    ///
    /// - Returns: the extracted mesh, or `nil` if `triangleIndices` is empty or every index
    ///   is out of range.
    public func subMesh(triangleIndices: [Int]) -> Mesh? {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        var remap: [UInt32: UInt32] = [:]
        var newPositions: [SIMD3<Float>] = []
        var newIndices: [UInt32] = []
        newIndices.reserveCapacity(triangleIndices.count * 3)

        for t in triangleIndices {
            guard t >= 0, t < tc else { continue }
            let base = t * 3
            for g in [idx[base], idx[base + 1], idx[base + 2]] {
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

    /// Ordered boundary loops (vertex-index rings) of the open edges — edges used by exactly
    /// one triangle. Requires `.welded()` first — see the file header; on unwelded input
    /// every edge is used by exactly one triangle, so every triangle's three edges come back
    /// as their own 3-edge "loop" instead of the mesh's real open boundary.
    public func boundaryLoops() -> [[UInt32]] {
        let idx = indices
        let tc = triangleCount
        func ekey(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        var edgeCount: [UInt64: Int] = [:]
        for t in 0..<tc {
            let base = t * 3
            let a = idx[base], b = idx[base + 1], c = idx[base + 2]
            for e in [ekey(a, b), ekey(b, c), ekey(c, a)] { edgeCount[e, default: 0] += 1 }
        }
        let boundary = edgeCount.filter { $0.value == 1 }.keys
        guard !boundary.isEmpty else { return [] }

        var remaining = Set<UInt64>(boundary)
        var nbr: [UInt32: [UInt32]] = [:]
        for e in boundary {
            let a = UInt32(e >> 32), b = UInt32(e & 0xffff_ffff)
            nbr[a, default: []].append(b)
            nbr[b, default: []].append(a)
        }
        // DETERMINISM: seed loops in sorted edge order and pick neighbours in sorted order.
        // Set/dict iteration order varies run-to-run, so without this a different seed chains
        // edges into different loops at non-manifold junctions — making the result
        // non-reproducible between runs.
        for k in Array(nbr.keys) { nbr[k]?.sort() }
        let seeds = boundary.sorted()

        var loops: [[UInt32]] = []
        for seed in seeds where remaining.contains(seed) {
            let sa = UInt32(seed >> 32), sb = UInt32(seed & 0xffff_ffff)
            remaining.remove(seed)
            var loop: [UInt32] = [sa, sb]
            var prev = sa, cur = sb
            while cur != sa {
                let opts = (nbr[cur] ?? []).filter { $0 != prev && remaining.contains(ekey(cur, $0)) }
                guard let nx = opts.first ?? (nbr[cur] ?? []).first(where: { remaining.contains(ekey(cur, $0)) }) else { break }
                remaining.remove(ekey(cur, nx))
                if nx == sa { break }
                loop.append(nx)
                prev = cur
                cur = nx
                if loop.count > boundary.count + 2 { break }
            }
            if loop.count >= 3 { loops.append(loop) }
        }
        return loops
    }
}
