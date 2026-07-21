import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

// Foundations (#16): weld, adjacency, normals, components, sub-mesh, boundary loops, integrity.

/// A closed box as a triangle mesh. `unwelded` repeats vertices per-triangle (as raw STL /
/// tessellation output does); otherwise vertices are shared per corner (8 unique positions).
private func boxMesh(_ sx: Float, _ sy: Float, _ sz: Float, unwelded: Bool = false) -> Mesh {
    let hx = sx / 2, hy = sy / 2, hz = sz / 2
    var v: [SIMD3<Float>] = []
    var idx: [UInt32] = []
    func quad(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, _ d: SIMD3<Float>) {
        let base = UInt32(v.count)
        v.append(contentsOf: [a, b, c, d])
        idx.append(contentsOf: [base, base + 1, base + 2, base, base + 2, base + 3])
    }
    quad([-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz])   // bottom
    quad([-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz])       // top
    quad([-hx, -hy, -hz], [hx, -hy, -hz], [hx, -hy, hz], [-hx, -hy, hz])  // front
    quad([hx, hy, -hz], [-hx, hy, -hz], [-hx, hy, hz], [hx, hy, hz])      // back
    quad([-hx, hy, -hz], [-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, hz])  // left
    quad([hx, -hy, -hz], [hx, hy, -hz], [hx, hy, hz], [hx, -hy, hz])      // right
    if unwelded {
        // Give every triangle its own vertex copies (no index sharing at all) — the shape of a
        // raw STL / unshared tessellation.
        var v2: [SIMD3<Float>] = []
        var idx2: [UInt32] = []
        var i = 0
        while i + 2 < idx.count {
            let base = UInt32(v2.count)
            v2.append(v[Int(idx[i])]); v2.append(v[Int(idx[i + 1])]); v2.append(v[Int(idx[i + 2])])
            idx2.append(contentsOf: [base, base + 1, base + 2])
            i += 3
        }
        return Mesh(vertices: v2, indices: idx2)!
    }
    // Each `quad()` call mints fresh vertices, so adjacent faces don't share indices yet even
    // though their corners coincide in position — weld to get the real 8-corner, edge-connected
    // box the adjacency/component/integrity tests below assume.
    return Mesh(vertices: v, indices: idx)!.welded()
}

/// An open tube (full circle, no end caps): 1 component, exactly 2 boundary loops.
private func openTubeMesh(radius: Float, length: Float, segments: Int) -> Mesh {
    var v: [SIMD3<Float>] = []
    var idx: [UInt32] = []
    let hz = length / 2
    for i in 0..<segments {
        let a0 = Float(i) / Float(segments) * 2 * .pi
        let a1 = Float(i + 1) / Float(segments) * 2 * .pi
        let p0b = SIMD3<Float>(radius * cos(a0), radius * sin(a0), -hz)
        let p1b = SIMD3<Float>(radius * cos(a1), radius * sin(a1), -hz)
        let p0t = SIMD3<Float>(radius * cos(a0), radius * sin(a0), hz)
        let p1t = SIMD3<Float>(radius * cos(a1), radius * sin(a1), hz)
        let base = UInt32(v.count)
        v.append(contentsOf: [p0b, p1b, p1t, p0t])
        idx.append(contentsOf: [base, base + 1, base + 2, base, base + 2, base + 3])
    }
    return Mesh(vertices: v, indices: idx)!.welded()
}

@Suite("Mesh foundations — weld, adjacency, components, boundary loops, integrity")
struct MeshFoundationsTests {

    @Test("welded() reduces an unshared-vertex box down to 8 unique corners")
    func weldReducesToUniqueCorners() {
        let mesh = boxMesh(10, 6, 4, unwelded: true)
        #expect(mesh.vertexCount == 36)   // 12 triangles × 3, no sharing
        let welded = mesh.welded()
        #expect(welded.vertexCount == 8)
        #expect(welded.triangleCount == 12)
    }

    @Test("welded() is idempotent")
    func weldIdempotent() {
        let once = boxMesh(10, 6, 4, unwelded: true).welded()
        let twice = once.welded()
        #expect(once.vertexCount == twice.vertexCount)
        #expect(once.indices == twice.indices)
    }

    @Test("Box: 12 triangles, each with exactly 3 edge-adjacent neighbours")
    func boxAdjacency() {
        let mesh = boxMesh(10, 6, 4)
        #expect(mesh.triangleCount == 12)
        let adjacency = mesh.triangleAdjacency()
        #expect(adjacency.count == 12)
        for neighbours in adjacency { #expect(neighbours.count == 3) }
    }

    @Test("Box: one connected component covering all 12 triangles")
    func boxSingleComponent() {
        let components = boxMesh(10, 6, 4).connectedComponents()
        #expect(components.count == 1)
        #expect(components[0].triangleIndices.count == 12)
        #expect(abs(components[0].area - (2 * (10 * 6 + 10 * 4 + 6 * 4))) < 1e-2)
    }

    @Test("Two disjoint boxes: two components, largest (or tied, lowest-index) first")
    func twoDisjointComponents() {
        let a = boxMesh(10, 10, 10)
        let b = boxMesh(4, 4, 4)
        let bTranslated = SIMD3<Float>(100, 0, 0)
        let verts = a.vertices + b.vertices.map { $0 + bTranslated }
        let idx = a.indices + b.indices.map { $0 + UInt32(a.vertexCount) }
        let combined = Mesh(vertices: verts, indices: idx)!
        let components = combined.connectedComponents()
        #expect(components.count == 2)
        #expect(components[0].triangleIndices.count == 12)   // both boxes have 12 tris; tie -> lowest index first
        #expect(components[1].triangleIndices.count == 12)
        #expect(components[0].triangleIndices.min()! < components[1].triangleIndices.min()!)
    }

    @Test("connectedComponents() is deterministic across repeated calls")
    func componentsDeterministic() {
        let mesh = boxMesh(10, 6, 4, unwelded: true).welded()
        let r1 = mesh.connectedComponents()
        let r2 = mesh.connectedComponents()
        #expect(r1 == r2)
    }

    @Test("Open tube: one component, two boundary loops (top and bottom rims)")
    func openTubeBoundaryLoops() {
        let mesh = openTubeMesh(radius: 5, length: 20, segments: 24).welded()
        let components = mesh.connectedComponents()
        #expect(components.count == 1)
        let loops = mesh.boundaryLoops()
        #expect(loops.count == 2)
        for loop in loops { #expect(loop.count == 24) }
    }

    @Test("subMesh(triangleIndices:) isolates the requested triangles with a compact vertex range")
    func subMeshIsolation() {
        let mesh = boxMesh(10, 6, 4)
        let sub = mesh.subMesh(triangleIndices: [0, 1])
        #expect(sub != nil)
        #expect(sub!.triangleCount == 2)
        #expect(sub!.vertexCount <= 6)
    }

    @Test("subMesh(triangleIndices:) returns nil for an empty selection")
    func subMeshEmptyIsNil() {
        let mesh = boxMesh(10, 6, 4)
        #expect(mesh.subMesh(triangleIndices: []) == nil)
    }

    @Test("Non-manifold fixture: an edge shared by three triangles is flagged")
    func nonManifoldEdgeDetected() {
        // Three triangles fanned around one common edge (a, b) — a "book" with 3 pages.
        let a = SIMD3<Float>(0, 0, 0), b = SIMD3<Float>(0, 0, 1)
        let c1 = SIMD3<Float>(1, 0, 0), c2 = SIMD3<Float>(0, 1, 0), c3 = SIMD3<Float>(-1, -1, 0)
        let v = [a, b, c1, c2, c3]
        let idx: [UInt32] = [0, 1, 2, 0, 1, 3, 0, 1, 4]
        let mesh = Mesh(vertices: v, indices: idx)!
        let report = mesh.integrityReport()
        #expect(report.nonManifoldEdgeCount == 1)
        #expect(!report.isEdgeManifold)
        #expect(!report.isWatertight)
    }

    @Test("Box integrity: watertight, manifold, orientable, genus 0, no boundary")
    func boxIntegrity() {
        let report = boxMesh(10, 6, 4).integrityReport()
        #expect(report.isWatertight)
        #expect(report.isEdgeManifold)
        #expect(report.isVertexManifold)
        #expect(report.nonManifoldEdgeCount == 0)
        #expect(report.boundaryLoopCount == 0)
        #expect(report.duplicateTriangleCount == 0)
        #expect(report.degenerateTriangleCount == 0)
        #expect(report.components.count == 1)
        if let g = report.genus { #expect(g == 0) } else { Issue.record("expected a genus value for a single watertight component") }
    }

    @Test("Open tube integrity: not watertight, boundaryLoopCount 2, genus nil")
    func openTubeIntegrity() {
        let report = openTubeMesh(radius: 5, length: 20, segments: 24).integrityReport()
        #expect(!report.isWatertight)
        #expect(report.boundaryLoopCount == 2)
        #expect(report.genus == nil)
    }

    @Test("Duplicate triangle is detected")
    func duplicateTriangleDetected() {
        let mesh = boxMesh(10, 6, 4)
        let idx = mesh.indices + Array(mesh.indices[0..<3])   // repeat the first triangle
        let dup = Mesh(vertices: mesh.vertices, indices: idx)!
        let report = dup.integrityReport()
        #expect(report.duplicateTriangleCount == 1)
    }
}
