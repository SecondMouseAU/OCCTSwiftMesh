// MeshIntegrity — the cheapest, most decision-relevant signals for an LLM inspecting a raw mesh
// (#16). Semantics follow the Open3D `TriangleMesh` conventions (`is_edge_manifold` /
// `is_vertex_manifold` / `is_watertight` / `cluster_connected_triangles`), the cleanest in the
// field for this exact question set.

import simd
import OCCTSwift

/// One connected component's size, for `MeshIntegrityReport.components`.
public struct ComponentSummary: Sendable, Codable, Equatable {
    public var triangleCount: Int
    public var area: Double
    public init(triangleCount: Int, area: Double) {
        self.triangleCount = triangleCount
        self.area = area
    }
}

/// Integrity check-list for a raw mesh: watertightness, manifoldness, orientability, boundary
/// structure, component breakdown, and sliver/degenerate triangle signals — the printability-
/// check-list shape (QIF / common 3D-print-diagnostic tools).
public struct MeshIntegrityReport: Sendable, Codable, Equatable {
    /// Every edge shared by exactly two triangles (no boundary, no non-manifold junction).
    public var isWatertight: Bool
    /// No edge is shared by more than two triangles.
    public var isEdgeManifold: Bool
    /// No vertex is a "pinch point" joining triangle fans that don't share edges.
    public var isVertexManifold: Bool
    /// Every interior (2-triangle) edge is traversed in opposite directions by its two
    /// triangles — consistent winding. `ambiguousFraction ~ 1.0` in the deviation engine is this
    /// repo's existing heuristic for the failure this formalizes.
    public var isOrientable: Bool
    /// Edges shared by 3+ triangles.
    public var nonManifoldEdgeCount: Int
    public var nonManifoldVertexCount: Int
    /// Closed rings of boundary (1-triangle) edges.
    public var boundaryLoopCount: Int
    public var duplicateTriangleCount: Int
    public var degenerateTriangleCount: Int
    /// V − E + F over valid (non-degenerate) triangles.
    public var eulerCharacteristic: Int
    /// (2 − eulerCharacteristic) / 2, meaningful only for a single watertight closed component;
    /// `nil` otherwise (open mesh, or more than one component — per-component genus isn't summed
    /// here since the two cases need different arbitration than a single scalar can carry).
    public var genus: Int?
    public var components: [ComponentSummary]
    /// Sliver signals over all valid triangles: the single worst (smallest) angle, and the 5th
    /// percentile of per-triangle minimum angles (most triangles are fine; the tail is where
    /// slivers live). Degrees.
    public var minAngleDegrees: Double
    public var minAngleP05Degrees: Double
    /// Aspect ratio = longestEdge / (2√3 × inradius); 1.0 = equilateral, higher = more sliver-like.
    /// Worst (max) and 95th-percentile.
    public var maxAspectRatio: Double
    public var aspectRatioP95: Double

    public init(isWatertight: Bool, isEdgeManifold: Bool, isVertexManifold: Bool, isOrientable: Bool,
                nonManifoldEdgeCount: Int, nonManifoldVertexCount: Int, boundaryLoopCount: Int,
                duplicateTriangleCount: Int, degenerateTriangleCount: Int, eulerCharacteristic: Int,
                genus: Int?, components: [ComponentSummary], minAngleDegrees: Double,
                minAngleP05Degrees: Double, maxAspectRatio: Double, aspectRatioP95: Double) {
        self.isWatertight = isWatertight
        self.isEdgeManifold = isEdgeManifold
        self.isVertexManifold = isVertexManifold
        self.isOrientable = isOrientable
        self.nonManifoldEdgeCount = nonManifoldEdgeCount
        self.nonManifoldVertexCount = nonManifoldVertexCount
        self.boundaryLoopCount = boundaryLoopCount
        self.duplicateTriangleCount = duplicateTriangleCount
        self.degenerateTriangleCount = degenerateTriangleCount
        self.eulerCharacteristic = eulerCharacteristic
        self.genus = genus
        self.components = components
        self.minAngleDegrees = minAngleDegrees
        self.minAngleP05Degrees = minAngleP05Degrees
        self.maxAspectRatio = maxAspectRatio
        self.aspectRatioP95 = aspectRatioP95
    }
}

public extension Mesh {

    func integrityReport() -> MeshIntegrityReport {
        let idx = indices
        let verts = vertices
        let tc = triangleCount
        guard tc > 0 else {
            return MeshIntegrityReport(isWatertight: false, isEdgeManifold: true, isVertexManifold: true,
                                        isOrientable: true, nonManifoldEdgeCount: 0, nonManifoldVertexCount: 0,
                                        boundaryLoopCount: 0, duplicateTriangleCount: 0, degenerateTriangleCount: 0,
                                        eulerCharacteristic: 0, genus: nil, components: [],
                                        minAngleDegrees: 0, minAngleP05Degrees: 0, maxAspectRatio: 0,
                                        aspectRatioP95: 0)
        }
        func ekey(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }

        var edgeCount: [UInt64: Int] = [:]
        var endpoints: [UInt64: (UInt32, UInt32)] = [:]
        var dirFwd: [UInt64: Int] = [:]
        var dirRev: [UInt64: Int] = [:]
        var faceKeys = Set<UInt64>()
        var degenerate = 0
        var duplicate = 0

        for t in 0..<tc {
            let a = idx[t * 3], b = idx[t * 3 + 1], c = idx[t * 3 + 2]
            if a == b || b == c || a == c { degenerate += 1; continue }
            let sorted3 = [a, b, c].sorted()
            let fkey = (UInt64(sorted3[0]) << 42) ^ (UInt64(sorted3[1]) << 21) ^ UInt64(sorted3[2])
            if !faceKeys.insert(fkey).inserted { duplicate += 1 }
            for (x, y) in [(a, b), (b, c), (c, a)] {
                let k = ekey(x, y)
                if endpoints[k] == nil { endpoints[k] = (x, y) }
                edgeCount[k, default: 0] += 1
                if endpoints[k]!.0 == x { dirFwd[k, default: 0] += 1 } else { dirRev[k, default: 0] += 1 }
            }
        }

        let nonManifoldEdges = edgeCount.values.filter { $0 > 2 }.count
        let boundaryEdgePairs = edgeCount.compactMap { k, cnt -> (UInt32, UInt32)? in cnt == 1 ? endpoints[k] : nil }
        let boundaryLoopRings = Loops.trace(edges: boundaryEdgePairs)
        let isWatertight = !edgeCount.values.contains { $0 != 2 }
        let isOrientable = edgeCount.allSatisfy { k, cnt in
            guard cnt == 2 else { return true }
            return (dirFwd[k] ?? 0) == 1 && (dirRev[k] ?? 0) == 1
        }

        // Non-manifold vertices: a vertex is a "pinch point" if its incident triangles don't form
        // ONE connected fan (edge-adjacency restricted to triangles touching this vertex).
        var incident = [[Int]](repeating: [], count: vertexCount)
        for t in 0..<tc {
            let a = Int(idx[t * 3]), b = Int(idx[t * 3 + 1]), c = Int(idx[t * 3 + 2])
            incident[a].append(t); incident[b].append(t); incident[c].append(t)
        }
        let adjacency = triangleAdjacency()
        var nonManifoldVertices = 0
        for v in 0..<vertexCount {
            let tris = incident[v]
            guard tris.count > 1 else { continue }
            var indexOf: [Int: Int] = [:]
            for (i, t) in tris.enumerated() { indexOf[t] = i }
            var parent = Array(0..<tris.count)
            func find(_ x: Int) -> Int { var r = x; while parent[r] != r { r = parent[r] }; return r }
            for (i, t) in tris.enumerated() {
                for n in adjacency[t] {
                    guard let j = indexOf[n] else { continue }
                    let ri = find(i), rj = find(j)
                    if ri != rj { parent[ri] = rj }
                }
            }
            let roots = Set(tris.indices.map { find($0) })
            if roots.count > 1 { nonManifoldVertices += 1 }
        }

        let validTriangles = tc - degenerate
        let eulerCharacteristic = vertexCount - edgeCount.count + validTriangles
        let components = connectedComponents().map { ComponentSummary(triangleCount: $0.triangleIndices.count, area: $0.area) }
        let genus: Int? = (isWatertight && components.count == 1) ? max(0, (2 - eulerCharacteristic) / 2) : nil

        // Sliver signals.
        var minAngles: [Double] = []
        var aspects: [Double] = []
        minAngles.reserveCapacity(tc)
        aspects.reserveCapacity(tc)
        for t in 0..<tc {
            let a = verts[Int(idx[t * 3])], b = verts[Int(idx[t * 3 + 1])], c = verts[Int(idx[t * 3 + 2])]
            let lab = Double(simd_distance(a, b)), lbc = Double(simd_distance(b, c)), lca = Double(simd_distance(c, a))
            guard lab > 1e-12, lbc > 1e-12, lca > 1e-12 else { continue }
            func angle(opposite: Double, s1: Double, s2: Double) -> Double {
                let cosA = (s1 * s1 + s2 * s2 - opposite * opposite) / (2 * s1 * s2)
                return acos(min(1, max(-1, cosA))) * 180 / .pi
            }
            let angA = angle(opposite: lbc, s1: lab, s2: lca)
            let angB = angle(opposite: lca, s1: lab, s2: lbc)
            let angC = 180 - angA - angB
            minAngles.append(min(angA, angB, angC))

            let s = (lab + lbc + lca) / 2
            let area = Double(simd_length(simd_cross(b - a, c - a))) * 0.5
            let inradius = s > 1e-12 ? area / s : 0
            let longest = max(lab, lbc, lca)
            aspects.append(inradius > 1e-12 ? longest / (2 * 1.7320508075688772 * inradius) : .infinity)
        }
        minAngles.sort()
        aspects.sort()
        func percentile(_ arr: [Double], _ p: Double) -> Double {
            guard !arr.isEmpty else { return 0 }
            let i = min(arr.count - 1, max(0, Int((p * Double(arr.count)).rounded(.down))))
            return arr[i]
        }

        return MeshIntegrityReport(
            isWatertight: isWatertight,
            isEdgeManifold: nonManifoldEdges == 0,
            isVertexManifold: nonManifoldVertices == 0,
            isOrientable: isOrientable,
            nonManifoldEdgeCount: nonManifoldEdges,
            nonManifoldVertexCount: nonManifoldVertices,
            boundaryLoopCount: boundaryLoopRings.count,
            duplicateTriangleCount: duplicate,
            degenerateTriangleCount: degenerate,
            eulerCharacteristic: eulerCharacteristic,
            genus: genus,
            components: components,
            minAngleDegrees: minAngles.first ?? 0,
            minAngleP05Degrees: percentile(minAngles, 0.05),
            maxAspectRatio: aspects.last ?? 0,
            aspectRatioP95: percentile(aspects, 0.95))
    }
}
