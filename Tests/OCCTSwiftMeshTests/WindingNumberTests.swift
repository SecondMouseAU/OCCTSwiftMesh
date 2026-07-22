import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.windingNumber — generalized winding number")
struct WindingNumberTests {

    // NOTE: `orientableTetrahedronMesh()` was the pre-existing hand-verified fixture for exactly
    // this need (`IntegrityReportTests.orientableClosedSurfaceReport` already used it for the
    // same reason) — used below alongside `weldedUnitCube()`, which turned out to have its OWN
    // orientation bug (two of its six faces wound inward, not the one originally suspected — see
    // that fixture's doc comment) and has since been fixed and is exercised directly too.

    @Test("A point enclosed by a closed, coherently-oriented mesh reads ~1")
    func enclosedPointReadsOne() {
        let tet = orientableTetrahedronMesh()
        let centroid = SIMD3<Double>(0.25, 0.25, 0.25)   // (0,0,0),(1,0,0),(0,1,0),(0,0,1)'s centroid
        #expect(abs(tet.windingNumber(at: centroid) - 1) < 1e-3)
    }

    @Test("A point far outside a closed mesh reads ~0")
    func exteriorPointReadsZero() {
        let tet = orientableTetrahedronMesh()
        let far = SIMD3<Double>(100, 100, 100)
        #expect(abs(tet.windingNumber(at: far)) < 1e-5)
    }

    @Test("weldedUnitCube(), now fixed, reads ~1 at its center — regression guard for the fixture fix")
    func fixedCubeEnclosedPointReadsOne() {
        let cube = weldedUnitCube()
        #expect(abs(cube.windingNumber(at: SIMD3(0.5, 0.5, 0.5)) - 1) < 1e-3)
        #expect(abs(cube.windingNumber(at: SIMD3(100, 100, 100))) < 1e-5)
    }

    @Test("sphereMesh(), now fixed, reads ~1 at its center — regression guard for the fixture fix")
    func fixedSphereEnclosedPointReadsOne() {
        let sphere = sphereMesh(radius: 5)
        #expect(abs(sphere.windingNumber(at: .zero) - 1) < 1e-2)
    }

    @Test("Reversing every triangle's winding negates the winding number everywhere")
    func reversalNegatesEverywhere() {
        // Unlike the two tests above, this holds regardless of whether the ORIGINAL mesh's own
        // winding happens to be consistent — reversing every triangle negates each one's own
        // contribution unconditionally (see the file header), so `weldedUnitCube()` is fine here.
        let cube = weldedUnitCube()
        let flipped = reversedWinding(cube)
        let points: [SIMD3<Double>] = [SIMD3(0.5, 0.5, 0.5), SIMD3(0.1, 0.9, 0.3), SIMD3(100, 100, 100)]
        for p in points {
            let a = cube.windingNumber(at: p)
            let b = flipped.windingNumber(at: p)
            #expect(abs(a + b) < 1e-3)
        }
    }

    @Test("windingNumber on an empty mesh returns 0 rather than crashing")
    func emptyMeshReturnsZero() {
        if let empty = Mesh(vertices: [], indices: []) {
            #expect(empty.windingNumber(at: .zero) == 0)
        }
    }

    @Test("An open shell's own hollow center reads a clearly nonzero winding — no enclosed-volume cancellation")
    func openShellHollowCenterReadsNonzero() {
        // Unlike a closed solid (where exterior points always read ~0 regardless of orientation
        // — see OrientationReportTests below), an open shell has no enclosed-volume cancellation
        // guarantee: a point at the hollow center of an open tube "sees" the whole wall from
        // inside, a large, clearly one-signed solid angle.
        let tube = openCylinderShellMesh(radius: 4, height: 4, segments: 32)
        let center = SIMD3<Double>(0, 0, 2)   // mid-height, on the tube's own axis
        let w = tube.windingNumber(at: center)
        #expect(abs(w) > 0.1)
        let flippedW = reversedWinding(tube).windingNumber(at: center)
        #expect(abs(w + flippedW) < 1e-6)
    }
}

@Suite("Mesh.orientationReport — global inside-out diagnostic")
struct OrientationReportTests {

    @Test("A correctly-oriented closed tetrahedron reads a near-zero mean exterior winding, not inverted")
    func closedTetrahedronCorrectOrientation() {
        // orientableTetrahedronMesh(), not weldedUnitCube() — see WindingNumberTests' note on
        // why the box fixture isn't suitable here.
        let report = orientableTetrahedronMesh().orientationReport()
        #expect(abs(report.meanExteriorWinding) < 1e-3)
        #expect(!report.looksInverted)
    }

    @Test("A globally-reversed closed tetrahedron ALSO reads near-zero exterior winding — the documented closed-mesh limitation")
    func closedTetrahedronReversedOrientationStillReadsZeroOutside() {
        // Exterior points of a closed mesh read 0 regardless of orientation (flipping negates
        // 0 to 0) — this exterior-only diagnostic is provably powerless on a watertight solid.
        // See the caveats on Mesh.orientationReport(samples:).
        let reversed = reversedWinding(orientableTetrahedronMesh())
        let report = reversed.orientationReport()
        #expect(abs(report.meanExteriorWinding) < 1e-3)
        #expect(!report.looksInverted)
    }

    @Test("Reversing an open shell's winding negates its mean winding exactly")
    func openShellReversalNegatesMean() {
        // meanExteriorWinding is a fixed average over a FIXED sample-point set (the hollow-probe
        // points are derived from vertex POSITIONS only, never face normals, so the sample set
        // itself is identical for a mesh and its reversal), and windingNumber is exactly linear
        // in orientation (see the file header) — so this holds by construction for ANY mesh,
        // closed or open.
        let dome = domeMesh()
        let a = dome.orientationReport()
        let b = reversedWinding(dome).orientationReport()
        #expect(abs(a.meanExteriorWinding + b.meanExteriorWinding) < 1e-6)
    }

    @Test("An inverted open dome is flagged; its correctly-oriented twin is not")
    func openDomeInversionIsDetected() {
        // domeMesh()'s own default winding happens to be inverted (see its doc comment in
        // MeshFixtures.swift) — so the DEFAULT reads inverted here and reversedWinding(_:) is
        // the correctly-oriented one.
        let invertedReport = domeMesh().orientationReport()
        #expect(invertedReport.looksInverted)
        #expect(invertedReport.meanExteriorWinding < -0.25)

        let correctReport = reversedWinding(domeMesh()).orientationReport()
        #expect(!correctReport.looksInverted)
        #expect(correctReport.meanExteriorWinding > 0)
    }

    @Test("An inverted open tube is flagged; its correctly-oriented twin is not")
    func openTubeInversionIsDetected() {
        let tube = openCylinderShellMesh(radius: 4, height: 4, segments: 32)
        let correctReport = tube.orientationReport()
        #expect(!correctReport.looksInverted)
        #expect(correctReport.meanExteriorWinding > 0)

        let invertedReport = reversedWinding(tube).orientationReport()
        #expect(invertedReport.looksInverted)
        #expect(invertedReport.meanExteriorWinding < -0.25)
    }

    @Test("orientationReport is deterministic across repeated calls")
    func determinism() {
        let dome = domeMesh()
        let a = dome.orientationReport()
        let b = dome.orientationReport()
        #expect(a.meanExteriorWinding == b.meanExteriorWinding)
        #expect(a.looksInverted == b.looksInverted)
    }

    @Test("An empty mesh returns a non-inverted, zero report rather than crashing")
    func emptyMeshIsHandled() {
        if let empty = Mesh(vertices: [], indices: []) {
            let report = empty.orientationReport()
            #expect(report.meanExteriorWinding == 0)
            #expect(!report.looksInverted)
        }
    }
}
