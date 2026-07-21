// MeshRegion — a named subset of a mesh's triangles.
//
// The common currency between connected-component splitting (Mesh.connectedComponents())
// and surface segmentation (Mesh.segmented(_:)): both hand back regions as triangle-index
// groups, largest-first, with a precomputed geometric area.

import simd
import OCCTSwift

/// A subset of a mesh's triangles, e.g. one connected component or one segmented surface patch.
public struct MeshRegion: Sendable, Equatable {
    /// Indices into the owning mesh's triangle list (`indices[i*3 ..< i*3+3]` per `i` here).
    public let triangleIndices: [Int]
    /// Total geometric area of the region's triangles, in the owning mesh's units.
    public let area: Double

    public init(triangleIndices: [Int], area: Double) {
        self.triangleIndices = triangleIndices
        self.area = area
    }

    /// Deterministic ordering used everywhere regions are returned: more triangles first,
    /// ties broken by lowest triangle index. Without the tie-break, equal-sized regions
    /// built from unordered Dictionary/Set buckets would shuffle order between runs.
    static func order(_ a: MeshRegion, _ b: MeshRegion) -> Bool {
        if a.triangleIndices.count != b.triangleIndices.count {
            return a.triangleIndices.count > b.triangleIndices.count
        }
        return (a.triangleIndices.min() ?? 0) < (b.triangleIndices.min() ?? 0)
    }
}

extension Mesh {
    /// Sum of triangle areas for a set of triangle indices into `indices`/`vertices`.
    static func area(ofTriangles triangleIndices: [Int], vertices: [SIMD3<Float>], indices: [UInt32]) -> Double {
        var sum = 0.0
        for t in triangleIndices {
            let base = t * 3
            let a = vertices[Int(indices[base])]
            let b = vertices[Int(indices[base + 1])]
            let c = vertices[Int(indices[base + 2])]
            sum += Double(simd_length(simd_cross(b - a, c - a)) * 0.5)
        }
        return sum
    }
}
