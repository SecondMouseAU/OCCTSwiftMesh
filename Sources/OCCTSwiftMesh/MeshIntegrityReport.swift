// MeshIntegrityReport.swift — a holistic manifoldness / validity / quality snapshot.
//
// Semantics follow the Open3D `TriangleMesh` conventions (is_edge_manifold /
// is_vertex_manifold / is_watertight / cluster_connected_triangles), which are the cleanest
// in the field. Welds internally so the report is meaningful on raw, per-triangle-unique OCCT
// tessellation or STL import without the caller needing to weld first — but duplicate and
// degenerate triangles are counted BEFORE they're cleaned up (that's the point of the report),
// while every other metric (manifoldness, Euler characteristic, genus, components, sliver
// signals) is computed on the deduplicated, non-degenerate topology so a handful of exact
// duplicate faces don't masquerade as a real non-manifold defect.

import Foundation
import simd
import OCCTSwift

/// A triangle's welded, sorted vertex-index triple — an exact duplicate-face key at any mesh
/// size, unlike a bit-packed integer key (which needs per-field bit budgets that cap the exact
/// vertex-count range).
private struct FaceKey: Hashable {
    let a, b, c: UInt32
}

/// A mesh's manifoldness, validity, and quality snapshot.
public struct MeshIntegrityReport: Sendable {
    /// Every welded edge is shared by exactly two triangles (no boundary, no non-manifold
    /// edges), AND every vertex is manifold (no pinch points) — matching Open3D's
    /// `is_watertight`, which folds vertex-manifoldness in too (`nonManifoldVertexCount == 0`
    /// is necessary, not just edge-manifoldness).
    public let isWatertight: Bool
    /// Every 2-triangle welded edge is traversed in opposite directions by its two triangles —
    /// a consistent winding exists. Only evaluated over 2-triangle edges; not meaningful when
    /// `nonManifoldEdgeCount > 0`.
    public let isOrientable: Bool
    /// Welded edges shared by three or more triangles.
    public let nonManifoldEdgeCount: Int
    /// Vertices whose surrounding triangles don't form a single fan (a "pinch point" / bowtie,
    /// or a branching link).
    public let nonManifoldVertexCount: Int
    /// Count of open boundary loops (see `boundaryLoops()`).
    public let boundaryLoopCount: Int
    /// Triangles that repeat an earlier triangle's vertex set (any winding), counted from the
    /// raw welded topology before cleanup.
    public let duplicateTriangleCount: Int
    /// Triangles that repeat a vertex post-weld (a welded edge collapsed to a point), counted
    /// from the raw welded topology before cleanup.
    public let degenerateTriangleCount: Int
    /// `V - E + F` of the deduplicated, non-degenerate, welded topology.
    public let eulerCharacteristic: Int
    /// Total genus across all closed components, or `nil` unless `isWatertight && isOrientable`
    /// (and the Euler characteristic is consistent with a valid closed 2-manifold).
    public let genus: Int?
    /// Connected components (see `connectedComponents()`), largest-first.
    public let components: [(triangleCount: Int, area: Double)]
    /// Smallest interior angle across all triangles, in degrees: the absolute worst case and
    /// the 5th percentile (a sliver signal more robust to one degenerate outlier).
    public let minAngleDegrees: (min: Double, p05: Double)
    /// Equilateral-normalized aspect ratio (`longestEdge / (2·√3·inradius)`, the VTK/FEM
    /// convention where an equilateral triangle scores `1.0`): worst case and 95th percentile.
    public let aspectRatio: (max: Double, p95: Double)
}

extension Mesh {
    /// Compute a manifoldness / validity / quality snapshot of this mesh.
    ///
    /// - Parameter weldTolerance: forwarded to the internal weld pass. `0` (default)
    ///   auto-derives `1e-6 ×` the mesh's bounding-box diagonal.
    public func integrityReport(weldTolerance: Double = 0) -> MeshIntegrityReport {
        let empty = MeshIntegrityReport(
            isWatertight: false, isOrientable: true, nonManifoldEdgeCount: 0,
            nonManifoldVertexCount: 0, boundaryLoopCount: 0, duplicateTriangleCount: 0,
            degenerateTriangleCount: 0, eulerCharacteristic: 0, genus: nil, components: [],
            minAngleDegrees: (0, 0), aspectRatio: (0, 0))

        let verts0 = vertices
        let idx0 = indices
        guard !verts0.isEmpty, idx0.count >= 3 else { return empty }

        let (remap, weldedPositions) = Mesh.weldPositions(verts0, tolerance: weldTolerance)
        let tc0 = idx0.count / 3

        var degenerateCount = 0
        var duplicateCount = 0
        var seenFaces = Set<FaceKey>()
        var cleanIndices: [UInt32] = []
        cleanIndices.reserveCapacity(idx0.count)
        for t in 0..<tc0 {
            let base = t * 3
            let a = remap[Int(idx0[base])], b = remap[Int(idx0[base + 1])], c = remap[Int(idx0[base + 2])]
            if a == b || b == c || a == c { degenerateCount += 1; continue }
            let s = [a, b, c].sorted()
            let key = FaceKey(a: s[0], b: s[1], c: s[2])
            if !seenFaces.insert(key).inserted { duplicateCount += 1; continue }
            cleanIndices.append(a); cleanIndices.append(b); cleanIndices.append(c)
        }

        guard !cleanIndices.isEmpty, let clean = Mesh(vertices: weldedPositions, indices: cleanIndices) else {
            return MeshIntegrityReport(
                isWatertight: false, isOrientable: true, nonManifoldEdgeCount: 0,
                nonManifoldVertexCount: 0, boundaryLoopCount: 0, duplicateTriangleCount: duplicateCount,
                degenerateTriangleCount: degenerateCount, eulerCharacteristic: 0, genus: nil,
                components: [], minAngleDegrees: (0, 0), aspectRatio: (0, 0))
        }

        let cIdx = clean.indices
        let cVerts = clean.vertices
        let tc = clean.triangleCount

        func ekey(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        var edgeCount: [UInt64: Int] = [:]
        var edgeDir: [UInt64: (fwd: Int, bwd: Int)] = [:]
        for t in 0..<tc {
            let base = t * 3
            let tri = [cIdx[base], cIdx[base + 1], cIdx[base + 2]]
            for k in 0..<3 {
                let a = tri[k], b = tri[(k + 1) % 3]
                let key = ekey(a, b)
                edgeCount[key, default: 0] += 1
                var d = edgeDir[key] ?? (0, 0)
                if a < b { d.fwd += 1 } else { d.bwd += 1 }
                edgeDir[key] = d
            }
        }
        let nonManifoldEdgeCount = edgeCount.values.filter { $0 >= 3 }.count
        let boundaryEdgeCount = edgeCount.values.filter { $0 == 1 }.count

        var isOrientable = true
        for (key, cnt) in edgeCount where cnt == 2 {
            let d = edgeDir[key] ?? (0, 0)
            if d.fwd != 1 || d.bwd != 1 { isOrientable = false; break }
        }

        let nonManifoldVertexCount = Mesh.countNonManifoldVertices(indices: cIdx)
        // Edge-manifold alone isn't enough: two closed shells pinched at one shared vertex have
        // zero boundary/non-manifold EDGES but are not a single watertight fan at that vertex.
        let isWatertight = boundaryEdgeCount == 0 && nonManifoldEdgeCount == 0 && nonManifoldVertexCount == 0
        let boundaryLoopCount = clean.boundaryLoops().count
        let components = clean.connectedComponents().map { (triangleCount: $0.triangleIndices.count, area: $0.area) }

        // V is the count of vertices actually REFERENCED by a clean triangle, not
        // `cVerts.count` — `weldedPositions` holds every welded vertex from the whole input,
        // computed before degenerate/duplicate triangles are dropped, so a vertex used only by
        // a dropped triangle would otherwise survive as an uncounted orphan and inflate V (and
        // so corrupt the Euler characteristic / genus, which only OCCTSwift.Mesh's initializer
        // — never trimming unreferenced vertices — lets through).
        let V = Set(cIdx).count
        let E = edgeCount.count
        let F = tc
        let euler = V - E + F
        var genus: Int? = nil
        if isWatertight, isOrientable {
            let c = max(components.count, 1)
            let numerator = 2 * c - euler
            if numerator >= 0, numerator % 2 == 0 { genus = numerator / 2 }
        }

        let (minAngle, aspect) = Mesh.angleAndAspectStats(vertices: cVerts, indices: cIdx, triangleCount: tc)

        return MeshIntegrityReport(
            isWatertight: isWatertight, isOrientable: isOrientable,
            nonManifoldEdgeCount: nonManifoldEdgeCount, nonManifoldVertexCount: nonManifoldVertexCount,
            boundaryLoopCount: boundaryLoopCount, duplicateTriangleCount: duplicateCount,
            degenerateTriangleCount: degenerateCount, eulerCharacteristic: euler, genus: genus,
            components: components, minAngleDegrees: minAngle, aspectRatio: aspect)
    }

    /// A vertex is non-manifold if the triangles around it don't form a single fan. For each
    /// triangle containing `v`, the edge OPPOSITE `v` is one link of `v`'s "umbrella"; in a
    /// manifold those opposite edges chain into a single simple path (open) or cycle (closed).
    /// A branch point (an opposite-edge endpoint touched by 3+ opposite edges) or a second
    /// disconnected chain (a pinch point / bowtie) makes `v` non-manifold.
    static func countNonManifoldVertices(indices: [UInt32]) -> Int {
        var oppositeEdges: [UInt32: [(UInt32, UInt32)]] = [:]
        let tc = indices.count / 3
        for t in 0..<tc {
            let base = t * 3
            let tri = [indices[base], indices[base + 1], indices[base + 2]]
            for k in 0..<3 {
                let v = tri[k]
                let b = tri[(k + 1) % 3], c = tri[(k + 2) % 3]
                oppositeEdges[v, default: []].append((b, c))
            }
        }
        var count = 0
        for (_, edges) in oppositeEdges {
            guard edges.count > 1 else { continue }
            var adj: [UInt32: Set<UInt32>] = [:]
            for (b, c) in edges {
                adj[b, default: []].insert(c)
                adj[c, default: []].insert(b)
            }
            if adj.values.contains(where: { $0.count > 2 }) { count += 1; continue }
            var visited = Set<UInt32>()
            var components = 0
            for start in adj.keys where !visited.contains(start) {
                components += 1
                var stack = [start]
                visited.insert(start)
                while let n = stack.popLast() {
                    for nb in adj[n] ?? [] where !visited.contains(nb) {
                        visited.insert(nb)
                        stack.append(nb)
                    }
                }
            }
            if components > 1 { count += 1 }
        }
        return count
    }

    /// Per-triangle minimum interior angle (degrees) and equilateral-normalized aspect ratio
    /// distributions, reduced to (min, p05) and (max, p95) respectively.
    static func angleAndAspectStats(vertices: [SIMD3<Float>], indices: [UInt32], triangleCount: Int)
        -> (minAngle: (min: Double, p05: Double), aspect: (max: Double, p95: Double)) {
        guard triangleCount > 0 else { return ((0, 0), (0, 0)) }

        func angleAt(_ p: SIMD3<Double>, _ q: SIMD3<Double>, _ r: SIMD3<Double>) -> Double {
            let u = simd_normalize(q - p), v = simd_normalize(r - p)
            let d = max(-1.0, min(1.0, simd_dot(u, v)))
            return acos(d) * 180 / .pi
        }
        func percentile(_ xs: [Double], _ p: Double) -> Double {
            let sorted = xs.sorted()
            let i = min(sorted.count - 1, max(0, Int((p * Double(sorted.count - 1)).rounded())))
            return sorted[i]
        }

        var perTriMinAngle: [Double] = []
        var perTriAspect: [Double] = []
        perTriMinAngle.reserveCapacity(triangleCount)
        perTriAspect.reserveCapacity(triangleCount)

        for t in 0..<triangleCount {
            let base = t * 3
            let a = SIMD3<Double>(vertices[Int(indices[base])])
            let b = SIMD3<Double>(vertices[Int(indices[base + 1])])
            let c = SIMD3<Double>(vertices[Int(indices[base + 2])])
            let ab = simd_length(b - a), bc = simd_length(c - b), ca = simd_length(a - c)
            guard ab > 1e-12, bc > 1e-12, ca > 1e-12 else { continue }

            perTriMinAngle.append(min(angleAt(a, b, c), angleAt(b, c, a), angleAt(c, a, b)))

            let longest = max(ab, bc, ca)
            let area = simd_length(simd_cross(b - a, c - a)) * 0.5
            let semiperimeter = (ab + bc + ca) / 2
            guard semiperimeter > 1e-12 else { continue }
            let inradius = area / semiperimeter
            guard inradius > 1e-12 else { continue }
            perTriAspect.append(longest / (2 * 3.0.squareRoot() * inradius))
        }

        guard !perTriMinAngle.isEmpty, !perTriAspect.isEmpty else { return ((0, 0), (0, 0)) }
        return (
            (perTriMinAngle.min() ?? 0, percentile(perTriMinAngle, 0.05)),
            (perTriAspect.max() ?? 0, percentile(perTriAspect, 0.95))
        )
    }
}
