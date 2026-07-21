// MeshRegion — a subset of a mesh's triangles plus region-level summary metrics.
//
// The common return unit for both `Mesh.connectedComponents()` (OCCTSwiftMesh#16) and
// `Mesh.segmented(_:)` (OCCTSwiftMesh#17): every summary value (area, bbox, mean normal,
// boundary-loop count) is computed once when the region is built, so a downstream per-zone
// metadata table never needs to re-touch the mesh.

import simd
import OCCTSwift

/// A subset of a mesh's triangles, with summary metrics computed once at construction time.
public struct MeshRegion: Sendable, Codable, Equatable {
    /// Indices into the mesh's triangle list (0-based: triangle `t`'s corners are
    /// `mesh.indices[3*t]`, `[3*t+1]`, `[3*t+2]`).
    public var triangleIndices: [Int]

    /// Geometric surface area (sum of triangle areas), in the mesh's linear units squared.
    public var area: Double

    /// Axis-aligned bounding box of the region's triangle corners.
    public var bboxMin: SIMD3<Float>
    public var bboxMax: SIMD3<Float>

    /// Area-weighted mean unit face normal.
    public var meanNormal: SIMD3<Float>

    /// Number of distinct closed boundary loops bounding this region: its own open rims
    /// (mesh boundary edges) PLUS every edge where it borders a different region/component.
    /// A disc-shaped region has 1; an annulus (or a region with one hole) has 2.
    public var boundaryLoopCount: Int

    public init(triangleIndices: [Int], area: Double, bboxMin: SIMD3<Float>, bboxMax: SIMD3<Float>,
                meanNormal: SIMD3<Float>, boundaryLoopCount: Int) {
        self.triangleIndices = triangleIndices
        self.area = area
        self.bboxMin = bboxMin
        self.bboxMax = bboxMax
        self.meanNormal = meanNormal
        self.boundaryLoopCount = boundaryLoopCount
    }
}

extension MeshRegion {

    /// Stable ordering: more triangles first, then lowest first-triangle index. Union-find /
    /// dictionary bucketing has no natural order of its own — without this tie-break, equal-sized
    /// regions/components would land in a different order on every run (dictionary iteration is
    /// unordered), so a region index would never refer to the same region twice. Ported convention
    /// from OCCTReconstruct's `IndexedMesh.regionOrder`.
    static func order(_ a: MeshRegion, _ b: MeshRegion) -> Bool {
        if a.triangleIndices.count != b.triangleIndices.count {
            return a.triangleIndices.count > b.triangleIndices.count
        }
        return (a.triangleIndices.min() ?? 0) < (b.triangleIndices.min() ?? 0)
    }

    /// Same ordering, for raw triangle-index arrays (pre-merge segmentation regions haven't been
    /// promoted to `MeshRegion` yet — that would waste bbox/normal/loop-count work on the
    /// pre-merge shattered regions the merge pass immediately discards).
    static func orderTriangleSets(_ a: [Int], _ b: [Int]) -> Bool {
        if a.count != b.count { return a.count > b.count }
        return (a.min() ?? 0) < (b.min() ?? 0)
    }

    /// Build a `MeshRegion` from a triangle set, computing area/bbox/mean-normal directly.
    /// `boundaryLoopCount` is supplied by the caller (see `boundaryLoopCounts(mesh:regionOf:...)`
    /// below) since computing it per-region independently would repeat the whole-mesh edge pass.
    static func build(mesh: Mesh, triangleIndices: [Int], faceNormals: [SIMD3<Float>],
                       boundaryLoopCount: Int) -> MeshRegion {
        let verts = mesh.vertices
        let idx = mesh.indices
        guard let first = triangleIndices.first else {
            return MeshRegion(triangleIndices: [], area: 0, bboxMin: .zero, bboxMax: .zero,
                               meanNormal: SIMD3<Float>(0, 0, 1), boundaryLoopCount: boundaryLoopCount)
        }
        var lo = verts[Int(idx[first * 3])]
        var hi = lo
        var area = 0.0
        var normalSum = SIMD3<Float>.zero
        for t in triangleIndices {
            let a = verts[Int(idx[t * 3])], b = verts[Int(idx[t * 3 + 1])], c = verts[Int(idx[t * 3 + 2])]
            lo = simd_min(lo, simd_min(a, simd_min(b, c)))
            hi = simd_max(hi, simd_max(a, simd_max(b, c)))
            let cross = simd_cross(b - a, c - a)
            area += Double(simd_length(cross)) * 0.5
            normalSum += faceNormals[t]
        }
        let nlen = simd_length(normalSum)
        let meanNormal = nlen > 1e-9 ? normalSum / nlen : SIMD3<Float>(0, 0, 1)
        return MeshRegion(triangleIndices: triangleIndices, area: area, bboxMin: lo, bboxMax: hi,
                           meanNormal: meanNormal, boundaryLoopCount: boundaryLoopCount)
    }

    /// Boundary-loop count for every region in a partition, computed in ONE pass over the mesh's
    /// edges — calling this for N regions costs the same as calling it once, instead of N separate
    /// whole-mesh edge passes.
    ///
    /// - Parameters:
    ///   - regionOf: `regionOf[t]` is the region index of triangle `t` (0..<regionCount), or -1 if
    ///     the triangle isn't assigned to any region in this partition.
    static func boundaryLoopCounts(mesh: Mesh, regionOf: [Int], regionCount: Int,
                                    adjacency: [[Int]]) -> [Int] {
        guard regionCount > 0 else { return [] }
        let idx = mesh.indices
        let tc = mesh.triangleCount
        func key(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        // edge -> triangles sharing it, over the WHOLE mesh (not just one region) — needed so a
        // region's internal boundary (bordering a different region) is distinguished from a
        // genuine mesh boundary edge, without a second full pass per region.
        var edgeTriangles: [UInt64: [Int]] = [:]
        edgeTriangles.reserveCapacity(tc * 3)
        for t in 0..<tc {
            let a = idx[t * 3], b = idx[t * 3 + 1], c = idx[t * 3 + 2]
            for e in [key(a, b), key(b, c), key(c, a)] { edgeTriangles[e, default: []].append(t) }
        }
        var edgesByRegion = [[(UInt32, UInt32)]](repeating: [], count: regionCount)
        for t in 0..<tc {
            let rid = regionOf[t]
            guard rid >= 0, rid < regionCount else { continue }
            let a = idx[t * 3], b = idx[t * 3 + 1], c = idx[t * 3 + 2]
            for (x, y) in [(a, b), (b, c), (c, a)] {
                let neighbors = edgeTriangles[key(x, y)] ?? []
                let sameRegionSharers = neighbors.filter { regionOf[$0] == rid }.count
                // sameRegionSharers counts THIS triangle too: an interior edge shared by two
                // same-region triangles reads 2 (not a boundary edge of the region); an edge with
                // only this triangle in the region reads 1 (region boundary, whether the other
                // side is a different region or nothing at all).
                if sameRegionSharers <= 1 { edgesByRegion[rid].append((x, y)) }
            }
        }
        return edgesByRegion.map { Loops.trace(edges: $0).count }
    }
}

/// Chains undirected mesh edges into closed vertex-index loops. Shared by `boundaryLoops()`,
/// `MeshRegion.boundaryLoopCounts`, and `MeshIntegrityReport`'s boundary-loop count.
enum Loops {

    /// - Returns: each closed loop as an ordered vertex-index ring (not repeating the start
    ///   vertex). A dangling (non-closing) chain — possible on non-manifold input where more than
    ///   two triangles share an edge — is still returned as a "loop" so the count reflects what's
    ///   actually there rather than silently dropping it.
    static func trace(edges: [(UInt32, UInt32)]) -> [[UInt32]] {
        guard !edges.isEmpty else { return [] }
        var adjacency: [UInt32: [UInt32]] = [:]
        for (a, b) in edges {
            adjacency[a, default: []].append(b)
            adjacency[b, default: []].append(a)
        }
        for k in adjacency.keys { adjacency[k]?.sort() }
        func key(_ a: UInt32, _ b: UInt32) -> UInt64 {
            let lo = UInt64(min(a, b)), hi = UInt64(max(a, b))
            return (hi << 32) | lo
        }
        var visited = Set<UInt64>()
        var loops: [[UInt32]] = []
        for start in adjacency.keys.sorted() {
            while let firstNext = adjacency[start]?.first(where: { !visited.contains(key(start, $0)) }) {
                var loop = [start]
                var current = start
                let next = firstNext
                visited.insert(key(current, next))
                loop.append(next)
                current = next
                while current != start {
                    guard let nxt = adjacency[current]?.first(where: { !visited.contains(key(current, $0)) }) else {
                        break
                    }
                    visited.insert(key(current, nxt))
                    loop.append(nxt)
                    current = nxt
                }
                if loop.count >= 3 {
                    loops.append(loop.first == loop.last ? Array(loop.dropLast()) : loop)
                }
            }
        }
        return loops
    }
}
