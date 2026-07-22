import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.segmentedRANSAC — Schnabel-style RANSAC primitive extraction")
struct RANSACSegmentationTests {

    @Test("A flat box face set finds plane regions covering the whole box")
    func boxFindsPlanes() {
        let mesh = weldedUnitCube()
        var options = Mesh.RANSACSegmentOptions()
        options.minSupportCount = 2   // each box face is only 2 triangles
        options.sampleSize = 2        // matches a single face's triangle count exactly
        let result = mesh.segmentedRANSAC(options)
        #expect(!result.regions.isEmpty)
        for fit in result.fits { #expect(fit.kind == .plane) }
        let covered = result.regions.reduce(0) { $0 + $1.triangleIndices.count }
        #expect(covered + result.truncatedTriangleCount == mesh.triangleCount)
    }

    @Test("A sphere is recovered as a single sphere region with the correct radius")
    func sphereIsRecovered() {
        // A fine-enough tessellation that the discretization sagitta (a coarse UV sphere's
        // face-centroid distance below its own true analytic surface) stays comfortably under
        // the default auto-derived inlierEpsilon (~0.005 × bbox diagonal) — otherwise even a
        // perfect sphere fit legitimately fails the distance gate on much of a coarse mesh's own
        // triangles, fragmenting across rounds for a reason that's about tessellation coarseness
        // vs. tolerance, not a segmentation defect.
        let mesh = sphereMesh(radius: 5, latSegments: 24, lonSegments: 32)
        let result = mesh.segmentedRANSAC()
        let spheres = result.fits.filter { $0.kind == .sphere }
        #expect(!spheres.isEmpty)
        if let radius = spheres.first?.radius {
            #expect(abs(radius - 5) < 0.5)
        }
        // The dominant sphere region should cover the large majority of the mesh.
        let bestArea = result.regions.map(\.area).max() ?? 0
        let totalArea = Mesh.area(ofTriangles: Array(0..<mesh.triangleCount), vertices: mesh.vertices, indices: mesh.indices)
        #expect(bestArea / totalArea > 0.7)
    }

    @Test("An open cylindrical shell is recovered as a cylinder with the correct radius")
    func cylinderIsRecovered() {
        let mesh = openCylinderMultiRingMesh(radius: 6, height: 20, segments: 24, rings: 8)
        let result = mesh.segmentedRANSAC()
        let cylinders = result.fits.filter { $0.kind == .cylinder }
        #expect(!cylinders.isEmpty)
        if let radius = cylinders.first?.radius {
            #expect(abs(radius - 6) < 0.5)
        }
    }

    @Test("minSupportCount rejects small/noisy candidates and reports leftovers, never silently")
    func minSupportCountReportsLeftovers() {
        let mesh = weldedUnitCube()
        var options = Mesh.RANSACSegmentOptions()
        options.minSupportCount = 1000   // impossibly high for a 12-triangle box
        let result = mesh.segmentedRANSAC(options)
        #expect(result.regions.isEmpty)
        #expect(result.truncatedTriangleCount == mesh.triangleCount)
    }

    @Test("maxRegions caps the result and reports the truncated triangle count")
    func maxRegionsCapsAndReports() {
        let mesh = weldedUnitCube()
        var options = Mesh.RANSACSegmentOptions()
        options.minSupportCount = 2
        options.sampleSize = 2
        options.maxRegions = 1
        let result = mesh.segmentedRANSAC(options)
        #expect(result.regions.count == 1)
        #expect(result.truncatedTriangleCount > 0)
    }

    @Test("Repeated calls on the same mesh are byte-identical (determinism)")
    func determinism() {
        let mesh = sphereMesh(radius: 5)
        let a = mesh.segmentedRANSAC()
        let b = mesh.segmentedRANSAC()
        #expect(a.regions.map(\.triangleIndices) == b.regions.map(\.triangleIndices))
        #expect(a.fits.map(\.kind) == b.fits.map(\.kind))
        #expect(a.fits.map(\.params) == b.fits.map(\.params))
        #expect(a.truncatedTriangleCount == b.truncatedTriangleCount)
    }

    @Test("An empty mesh returns an empty result rather than crashing")
    func emptyMeshIsHandled() {
        if let empty = Mesh(vertices: [], indices: []) {
            let result = empty.segmentedRANSAC()
            #expect(result.regions.isEmpty)
            #expect(result.truncatedTriangleCount == 0)
        }
    }
}

@Suite("Mesh.segmentedAutoSelect — dihedral vs. RANSAC bake-off")
struct SegmentationAutoSelectTests {

    @Test("A single integrated part (a box) picks the dihedral strategy")
    func boxPicksDihedral() {
        let mesh = weldedUnitCube()
        let auto = mesh.segmentedAutoSelect()
        #expect(auto.strategy == .dihedral)
        #expect(auto.dihedralScore >= auto.ransacScore)
        #expect(auto.result.regions.map(\.triangleIndices) == mesh.segmented().regions.map(\.triangleIndices))
    }

    @Test("The winning result matches calling that strategy directly")
    func winningResultMatchesDirectCall() {
        let mesh = sphereMesh(radius: 5)
        let auto = mesh.segmentedAutoSelect()
        switch auto.strategy {
        case .dihedral:
            #expect(auto.result.regions.map(\.triangleIndices) == mesh.segmented().regions.map(\.triangleIndices))
        case .ransac:
            #expect(auto.result.regions.map(\.triangleIndices) == mesh.segmentedRANSAC().regions.map(\.triangleIndices))
        }
    }

    @Test("Determinism across repeated calls")
    func determinism() {
        let mesh = coarseCappedCylinderMesh()
        let a = mesh.segmentedAutoSelect()
        let b = mesh.segmentedAutoSelect()
        #expect(a.strategy == b.strategy)
        #expect(a.dihedralScore == b.dihedralScore)
        #expect(a.ransacScore == b.ransacScore)
    }
}
