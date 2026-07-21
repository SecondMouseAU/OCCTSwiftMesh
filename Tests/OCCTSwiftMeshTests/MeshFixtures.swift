import simd
import OCCTSwift

// Shared synthetic-mesh builders for the mesh-foundations and segmentation test suites.
// All pure geometry — no OCCT tessellation needed, mirroring CrossSectionTests' approach.

/// A unit cube, corner at the origin, vertices SHARED across faces (8 vertices, 12 triangles) —
/// already welded by construction.
func weldedUnitCube() -> Mesh {
    let p: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(1, 1, 0), SIMD3(0, 1, 0),
        SIMD3(0, 0, 1), SIMD3(1, 0, 1), SIMD3(1, 1, 1), SIMD3(0, 1, 1),
    ]
    let faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
    var indices: [UInt32] = []
    for f in faces {
        indices.append(contentsOf: [UInt32(f[0]), UInt32(f[1]), UInt32(f[2])])
        indices.append(contentsOf: [UInt32(f[0]), UInt32(f[2]), UInt32(f[3])])
    }
    return Mesh(vertices: p, indices: indices)!
}

/// The same unit cube, but EVERY triangle gets its own 3 fresh vertices (36 vertices total, 12
/// triangles, no two triangles sharing an index anywhere) — the fully-unshared "soup" that raw
/// OCCT tessellation / STL import actually produces. Geometrically identical to
/// `weldedUnitCube()`; welding should collapse it back to 8 vertices.
func unweldedUnitCube() -> Mesh {
    let corners: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(1, 1, 0), SIMD3(0, 1, 0),
        SIMD3(0, 0, 1), SIMD3(1, 0, 1), SIMD3(1, 1, 1), SIMD3(0, 1, 1),
    ]
    let faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
    var positions: [SIMD3<Float>] = []
    var indices: [UInt32] = []
    for f in faces {
        let quad = f.map { corners[$0] }
        for tri in [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]] {
            let base = UInt32(positions.count)
            positions.append(contentsOf: tri)
            indices.append(contentsOf: [base, base + 1, base + 2])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// Two unit cubes far apart, each welded internally but sharing no vertices with the other —
/// two connected components.
func disjointCubesMesh() -> Mesh {
    let a = weldedUnitCube()
    let b = weldedUnitCube()
    let offset = SIMD3<Float>(100, 100, 100)
    let positions = a.vertices + b.vertices.map { $0 + offset }
    let bBase = UInt32(a.vertices.count)
    let indices = a.indices + b.indices.map { $0 + bBase }
    return Mesh(vertices: positions, indices: indices)!
}

/// An open cylindrical tube (barrel only, no end caps), welded by construction: shared ring
/// vertices at every angular step. One connected component; boundary is exactly the top and
/// bottom rim — 2 boundary loops.
func openCylinderShellMesh(radius: Float = 3, height: Float = 5, segments: Int = 16) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for k in 0..<2 {
        let z = Float(k) * height
        for i in 0..<segments {
            let a = Float(i) / Float(segments) * 2 * .pi
            positions.append(SIMD3(radius * cos(a), radius * sin(a), z))
        }
    }
    var indices: [UInt32] = []
    let lo: UInt32 = 0, hi = UInt32(segments)
    for i in 0..<segments {
        let a = lo + UInt32(i), b = lo + UInt32((i + 1) % segments)
        let c = hi + UInt32(i), d = hi + UInt32((i + 1) % segments)
        indices.append(contentsOf: [a, b, c, b, d, c])
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A coarse capped cylinder: `sides`-sided barrel (adjacent facets 360/sides° apart — coarse
/// enough to shatter under a 20° dihedral threshold) plus flat top/bottom disk caps. Welded by
/// construction. Segmentation should merge the shattered barrel facets back into one cylinder
/// region, while the caps (already-coplanar fans) stay separate — 3 regions total.
func coarseCappedCylinderMesh(radius: Float = 4, sides: Int = 12, rings: Int = 5, height: Float = 4) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for k in 0..<rings {
        let z = Float(k) / Float(rings - 1) * height
        for i in 0..<sides {
            let a = Float(i) / Float(sides) * 2 * .pi
            positions.append(SIMD3(radius * cos(a), radius * sin(a), z))
        }
    }
    let bottomCenter = UInt32(positions.count); positions.append(SIMD3(0, 0, 0))
    let topCenter = UInt32(positions.count); positions.append(SIMD3(0, 0, height))

    var indices: [UInt32] = []
    for k in 0..<(rings - 1) {
        let lo = UInt32(k * sides), hi = UInt32((k + 1) * sides)
        for i in 0..<sides {
            let a = lo + UInt32(i), b = lo + UInt32((i + 1) % sides)
            let c = hi + UInt32(i), d = hi + UInt32((i + 1) % sides)
            indices.append(contentsOf: [a, b, c, b, d, c])
        }
    }
    for i in 0..<sides {
        let a = UInt32(i), b = UInt32((i + 1) % sides)
        indices.append(contentsOf: [bottomCenter, b, a])
    }
    let topRingBase = UInt32((rings - 1) * sides)
    for i in 0..<sides {
        let a = topRingBase + UInt32(i), b = topRingBase + UInt32((i + 1) % sides)
        indices.append(contentsOf: [topCenter, a, b])
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A closed, orientable tetrahedron plus one extra degenerate triangle that references a fresh,
/// otherwise-unused vertex — that vertex becomes an orphan once the degenerate triangle is
/// dropped during `integrityReport()`'s cleanup pass.
func tetrahedronWithOrphanDegenerateTriangle() -> Mesh {
    let base = orientableTetrahedronMesh()
    var positions = base.vertices
    positions.append(SIMD3(5, 5, 5))
    var indices = base.indices
    indices.append(contentsOf: [0, 0, UInt32(positions.count - 1)])   // degenerate: repeats vertex 0
    return Mesh(vertices: positions, indices: indices)!
}

/// Three triangles sharing one edge (a "book") — a non-manifold EDGE (valence 3), no
/// non-manifold vertex.
func nonManifoldEdgeFixture() -> Mesh {
    let p: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0),   // shared edge (0, 1)
        SIMD3(0, 1, 0), SIMD3(0, -1, 0), SIMD3(0, 0, 1),
    ]
    let indices: [UInt32] = [0, 1, 2, 1, 0, 3, 0, 1, 4]
    return Mesh(vertices: p, indices: indices)!
}

/// Two triangles sharing exactly one vertex (a "bowtie" pinch point) — a non-manifold VERTEX,
/// no non-manifold edge.
func bowtieVertexFixture() -> Mesh {
    let p: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0),   // triangle A, shares vertex index 2
        SIMD3(0, 1, 1), SIMD3(1, 1, 1),                   // triangle B
    ]
    let indices: [UInt32] = [0, 1, 2, 2, 3, 4]
    return Mesh(vertices: p, indices: indices)!
}

/// A closed, watertight tetrahedron with hand-verified CONSISTENT outward winding (every shared
/// edge is traversed in opposite directions by its two triangles) — for asserting
/// `isOrientable`/`genus`, which need a known-consistent fixture rather than an
/// orientation-agnostic one.
func orientableTetrahedronMesh() -> Mesh {
    let p: [SIMD3<Float>] = [SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 1)]
    let indices: [UInt32] = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3]
    return Mesh(vertices: p, indices: indices)!
}

/// One valid triangle, an exact duplicate of it, and a triangle that degenerates to a repeated
/// vertex once vertex 3 (coincident with vertex 0) is welded.
func duplicateAndDegenerateFixture() -> Mesh {
    let p: [SIMD3<Float>] = [SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 0)]
    let indices: [UInt32] = [0, 1, 2, 0, 1, 2, 0, 3, 1]
    return Mesh(vertices: p, indices: indices)!
}
