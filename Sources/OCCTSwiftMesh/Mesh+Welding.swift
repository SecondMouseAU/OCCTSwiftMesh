// Mesh+Welding.swift — merge coincident vertices so adjacency-based algorithms have a
// shared substrate to work on.
//
// OCCT tessellation and STL loading both produce (near-)unshared vertices in practice: three
// unique positions per triangle, even where triangles are geometrically edge-adjacent. Every
// adjacency-based primitive in this package (triangleAdjacency, connectedComponents,
// boundaryLoops, segmented) needs a WELDED mesh to see that adjacency at all — on unwelded
// input, no two triangles share a vertex index, so each looks isolated. Weld first.

import simd
import OCCTSwift

extension Mesh {
    /// Snap each vertex to a spatial grid cell and merge every vertex landing in the same
    /// cell, keeping the first-encountered position as that cell's representative. Returns
    /// the per-original-vertex remap (into the deduplicated `positions`) alongside the
    /// deduplicated positions themselves. Internal building block shared by `welded(tolerance:)`
    /// and every algorithm that needs welded topology without discarding the caller's own
    /// triangle indexing (weld positions, keep indices/order intact).
    static func weldPositions(_ vertices: [SIMD3<Float>], tolerance: Double) -> (remap: [UInt32], positions: [SIMD3<Float>]) {
        guard !vertices.isEmpty else { return ([], []) }
        var lo = vertices[0], hi = vertices[0]
        for p in vertices { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let diag = Double(simd_length(hi - lo))
        // Same auto-derivation as crossSection's weld default: tiny relative to the model,
        // far below any real wall thickness, so distinct points stay distinct.
        let cell = tolerance > 0 ? tolerance : max(1e-9, 1e-6 * diag)

        struct GridKey: Hashable { var x: Int64; var y: Int64; var z: Int64 }
        // Clamped so a huge coordinate over a tiny cell (e.g. every vertex coincident, so
        // `diag == 0` and `cell` floors to `1e-9`, combined with coordinates of order 1e10+)
        // can't overflow the Double → Int64 conversion and trap. Values only ever meet at this
        // boundary for coordinate magnitudes many orders beyond any plausible model — safe.
        func gridCoord(_ x: Float, _ cell: Double) -> Int64 {
            let d = (Double(x) / cell).rounded()
            guard d.isFinite else { return 0 }
            return Int64(max(-9.0e18, min(9.0e18, d)))
        }
        var cellToIndex: [GridKey: UInt32] = [:]
        var remap = [UInt32](repeating: 0, count: vertices.count)
        var positions: [SIMD3<Float>] = []
        positions.reserveCapacity(vertices.count)
        for i in vertices.indices {
            let p = vertices[i]
            let key = GridKey(x: gridCoord(p.x, cell), y: gridCoord(p.y, cell), z: gridCoord(p.z, cell))
            if let existing = cellToIndex[key] {
                remap[i] = existing
            } else {
                let newIndex = UInt32(positions.count)
                cellToIndex[key] = newIndex
                positions.append(p)
                remap[i] = newIndex
            }
        }
        return (remap, positions)
    }

    /// Merge coincident vertices within `tolerance`, remapping every triangle to its shared
    /// vertex and dropping any triangle that degenerates (repeats a vertex) as a result.
    ///
    /// - Parameter tolerance: spatial merge radius. `0` (default) auto-derives
    ///   `1e-6 ×` the mesh's bounding-box diagonal, matching `crossSection`'s weld default.
    /// - Returns: the welded mesh, or `self` unchanged if it has no vertices/triangles, or if
    ///   welding collapsed every triangle to degenerate (a pathological all-coincident input).
    public func welded(tolerance: Double = 0) -> Mesh {
        let verts = vertices
        let idx = indices
        guard !verts.isEmpty, idx.count >= 3 else { return self }

        let (remap, positions) = Mesh.weldPositions(verts, tolerance: tolerance)

        var newIndices: [UInt32] = []
        newIndices.reserveCapacity(idx.count)
        var tri = 0
        while tri + 2 < idx.count {
            let a = remap[Int(idx[tri])], b = remap[Int(idx[tri + 1])], c = remap[Int(idx[tri + 2])]
            tri += 3
            guard a != b, b != c, a != c else { continue }
            newIndices.append(a); newIndices.append(b); newIndices.append(c)
        }
        guard let out = Mesh(vertices: positions, indices: newIndices) else { return self }
        return out
    }
}
