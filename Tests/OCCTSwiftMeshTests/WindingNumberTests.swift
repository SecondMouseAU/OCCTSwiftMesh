import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.windingNumber — generalized winding number")
struct WindingNumberTests {

    // NOTE: these use `orientableTetrahedronMesh()`, not `weldedUnitCube()` — the same fixture
    // choice `IntegrityReportTests.orientableClosedSurfaceReport` makes, and for the same reason
    // (see that fixture's doc comment): `weldedUnitCube()`'s face list isn't hand-verified for
    // CONSISTENT outward winding (nothing before this needed it — topology/dihedral-angle
    // algorithms don't care, only this one does), and turns out to have exactly one of its 6
    // faces flipped relative to the other 5 — enclosed-point winding reads a very deliberate
    // 2/3 there instead of 1, a good demonstration of why this matters but not what these two
    // tests are checking.

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

    @Test("Reversing an open shell's winding negates its mean exterior winding exactly")
    func openShellReversalNegatesMean() {
        // meanExteriorWinding is a fixed average over a FIXED sample-point set, and
        // windingNumber is exactly linear in orientation (see the file header) — so this holds
        // by construction for ANY mesh, closed or open. What differs for an open shell (vs. the
        // closed-mesh case above) is that the mean itself need not be near zero to begin with —
        // see `openShellHollowCenterReadsNonzero` in WindingNumberTests for a fixture/point where
        // it clearly isn't, since a generic exterior-only sample here can land anywhere from a
        // strong signal to near-total front/back cancellation depending on the shell's shape and
        // the sample points' placement relative to it.
        let dome = domeMesh()
        let a = dome.orientationReport()
        let b = reversedWinding(dome).orientationReport()
        #expect(abs(a.meanExteriorWinding + b.meanExteriorWinding) < 1e-6)
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
