import OCCTSwift
import Testing
import simd

@testable import OCCTSwiftMesh

// Slicing is pure geometry, so these fixtures build meshes directly from
// vertex/index arrays; no OCCT tessellation needed.

/// An axis-aligned box centred at the origin, as a closed triangle mesh.
private func boxMesh(_ sx: Float, _ sy: Float, _ sz: Float) -> Mesh {
    let hx = sx / 2
    let hy = sy / 2
    let hz = sz / 2
    let v: [SIMD3<Float>] = [
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
    ]
    // 12 triangles (outward winding not required by the slicer).
    let f: [(Int, Int, Int)] = [
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
    ]
    let idx = f.flatMap { [UInt32($0.0), UInt32($0.1), UInt32($0.2)] }
    return Mesh(vertices: v, indices: idx)!
}

/// A square tube: outer box minus a concentric inner box, open at both Z ends,
/// a thin-walled prism, exactly the body-0 shape in miniature.
private func squareTubeMesh(outer: Float, inner: Float, length: Float) -> Mesh {
    let ho = outer / 2
    let hi = inner / 2
    let hz = length / 2
    var v: [SIMD3<Float>] = []
    var idx: [UInt32] = []
    func quad(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, _ d: SIMD3<Float>) {
        let base = UInt32(v.count)
        v.append(contentsOf: [a, b, c, d])
        idx.append(contentsOf: [base, base + 1, base + 2, base, base + 2, base + 3])
    }
    // Outer side walls (4), inner side walls (4); no end caps (open tube).
    let oc: [SIMD2<Float>] = [[-ho, -ho], [ho, -ho], [ho, ho], [-ho, ho]]
    let ic: [SIMD2<Float>] = [[-hi, -hi], [hi, -hi], [hi, hi], [-hi, hi]]
    for i in 0..<4 {
        let j = (i + 1) % 4
        quad(
            SIMD3(oc[i].x, oc[i].y, -hz), SIMD3(oc[j].x, oc[j].y, -hz),
            SIMD3(oc[j].x, oc[j].y, hz), SIMD3(oc[i].x, oc[i].y, hz))
        quad(
            SIMD3(ic[i].x, ic[i].y, hz), SIMD3(ic[j].x, ic[j].y, hz),
            SIMD3(ic[j].x, ic[j].y, -hz), SIMD3(ic[i].x, ic[i].y, -hz))
    }
    return Mesh(vertices: v, indices: idx)!
}

@Suite("Mesh.crossSection, slicing into closed contours")
struct CrossSectionTests {

    @Test("A box sliced mid-height yields one rectangular loop of the right size")
    func boxSingleLoop() throws {
        let mesh = boxMesh(10, 6, 20)
        let s = try #require(
            mesh.crossSection(
                plane: CutPlane(point: .zero, normal: SIMD3(0, 0, 1))))
        #expect(s.contours.count == 1)
        let c = s.contours[0]
        #expect(c.depth == 0)
        #expect(!c.isHole)
        // The plane's in-plane u/v basis is arbitrary, so compare the section's
        // two extents as a set, not by axis: a 10×6×20 box cut ⊥Z is 10×6.
        let b = c.bounds
        let extents = [b.max.x - b.min.x, b.max.y - b.min.y].sorted()
        #expect(abs(extents[0] - 6) < 1e-3)
        #expect(abs(extents[1] - 10) < 1e-3)
        // CCW solid boundary.
        #expect(c.signedArea > 0)
        #expect(abs(c.area - 60) < 1e-2)
    }

    @Test("A square tube slices into separate outer + inner loops; thickness recoverable")
    func tubeTwoLoops() throws {
        let mesh = squareTubeMesh(outer: 12, inner: 8, length: 40)
        let s = try #require(
            mesh.crossSection(
                plane: CutPlane(point: .zero, normal: SIMD3(0, 0, 1))))
        #expect(s.contours.count == 2)
        let outer = try #require(s.contours.first { $0.depth == 0 })
        let inner = try #require(s.contours.first { $0.depth == 1 })
        // Outer is the solid boundary, inner is a hole.
        #expect(!outer.isHole)
        #expect(inner.isHole)
        #expect(inner.parent == s.contours.firstIndex { $0.depth == 0 })
        // Sizes.
        let ob = outer.bounds
        let ib = inner.bounds
        #expect(abs((ob.max.x - ob.min.x) - 12) < 1e-3)
        #expect(abs((ib.max.x - ib.min.x) - 8) < 1e-3)
        // Wall thickness = (outer extent − inner extent) / 2 = (12 − 8)/2 = 2.
        let thickness = ((ob.max.x - ob.min.x) - (ib.max.x - ib.min.x)) / 2
        #expect(abs(thickness - 2) < 1e-3)
    }

    @Test("Plane that misses the mesh returns nil")
    func planeMisses() {
        let mesh = boxMesh(10, 6, 20)
        #expect(
            mesh.crossSection(
                plane: CutPlane(point: SIMD3(0, 0, 100), normal: SIMD3(0, 0, 1))) == nil)
    }

    @Test("crossSections stack produces multiple slices along the long axis")
    func sliceStack() {
        let mesh = squareTubeMesh(outer: 12, inner: 8, length: 40)
        let stack = mesh.crossSections(axis: SIMD3(0, 0, 1), through: .zero, spacing: 5)
        #expect(stack.count >= 6)
        // Every interior slice has the two-loop tube signature.
        for sec in stack { #expect(sec.contours.count == 2) }
    }
}
