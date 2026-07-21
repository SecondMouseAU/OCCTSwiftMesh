import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.segmented — dihedral region-growing + primitive-fit merge")
struct SegmentationTests {

    @Test("A box segments into 6 unmerged planar regions")
    func boxSegmentsIntoSixRegions() {
        let result = weldedUnitCube().segmented()
        #expect(result.regions.count == 6)
        #expect(result.fits.count == 6)
        #expect(result.truncatedTriangleCount == 0)
        for fit in result.fits {
            #expect(fit.kind == .plane)
            #expect(fit.residualRMS < 1e-5)
        }
        // Box faces meet at 90°, well outside maxMergeAngleDegrees (50°) — must NOT fuse.
        for region in result.regions { #expect(region.triangleIndices.count == 2) }
    }

    @Test("Unwelded box still segments into 6 regions — weld precondition is handled internally")
    func unweldedBoxStillSegments() {
        let result = unweldedUnitCube().segmented()
        #expect(result.regions.count == 6)
        for region in result.regions { #expect(region.triangleIndices.count == 2) }
    }

    @Test("A coarse capped cylinder merges its shattered barrel facets back into one cylinder")
    func coarseCylinderMergesToBarrelPlusCaps() {
        let mesh = coarseCappedCylinderMesh(radius: 4, sides: 12, rings: 5, height: 4)

        // Sanity: smooth region-growing alone shatters the coarse barrel — adjacent facets are
        // 30° apart, past the 20° default dihedral threshold, so each becomes its own region
        // before the merge pass runs.
        let normals = mesh.faceNormals()
        let adjacency = mesh.triangleAdjacency()
        let seeded = Mesh.segmentSmoothRegions(triangleCount: mesh.triangleCount, normals: normals,
                                               adjacency: adjacency, maxDihedralDegrees: 20)
        // 12 vertical barrel strips (each internally coplanar top-to-bottom) + 2 flat cap fans.
        #expect(seeded.count == 14)

        let result = mesh.segmented()
        #expect(result.regions.count == 3)
        let cylinders = result.fits.filter { $0.kind == .cylinder }
        let planes = result.fits.filter { $0.kind == .plane }
        #expect(cylinders.count == 1)
        #expect(planes.count == 2)
        if let radius = cylinders.first?.radius {
            #expect(abs(radius - 4) < 0.15)
        }
    }

    @Test("An open cylindrical shell (no caps) merges into a single cylinder region")
    func openShellSegmentsToOneCylinder() {
        // The "open half-cylinder shell" case from the tracking issue: an uncapped barrel, only
        // the round wall — coarse enough (12 sides, 30° apart) to shatter under the default 20°
        // dihedral threshold, with nothing else present for the merge to confuse it with.
        let mesh = openCylinderShellMesh(radius: 4, height: 4, segments: 12)
        let result = mesh.segmented()
        #expect(result.regions.count == 1)
        #expect(result.fits.first?.kind == .cylinder)
        if let radius = result.fits.first?.radius {
            #expect(abs(radius - 4) < 0.15)
        }
    }

    @Test("Segmentation is deterministic across repeated calls")
    func determinism() {
        let mesh = coarseCappedCylinderMesh()
        let a = mesh.segmented()
        let b = mesh.segmented()
        #expect(a.regions.map(\.triangleIndices) == b.regions.map(\.triangleIndices))
        #expect(a.regions.map(\.area) == b.regions.map(\.area))
        #expect(a.fits.map(\.kind) == b.fits.map(\.kind))
        #expect(a.fits.map(\.params) == b.fits.map(\.params))
        #expect(a.truncatedTriangleCount == b.truncatedTriangleCount)
    }

    @Test("maxRegions caps the result and reports the truncated triangle count, never silently")
    func maxRegionsCapsAndReports() {
        var options = Mesh.SegmentOptions()
        options.maxRegions = 3
        let result = weldedUnitCube().segmented(options)
        #expect(result.regions.count == 3)
        #expect(result.truncatedTriangleCount == 6)   // 3 dropped regions × 2 triangles each
    }

    @Test("A negative maxRegions is treated as zero rather than crashing")
    func negativeMaxRegionsDoesNotCrash() {
        var options = Mesh.SegmentOptions()
        options.maxRegions = -1
        let result = weldedUnitCube().segmented(options)
        #expect(result.regions.isEmpty)
        #expect(result.fits.isEmpty)
        #expect(result.truncatedTriangleCount == 12)
    }

    @Test("minRegionTriangles drops undersized regions and reports them as truncated")
    func minRegionTrianglesFiltersAndReports() {
        var options = Mesh.SegmentOptions()
        options.minRegionTriangles = 3   // every box region has only 2 triangles
        let result = weldedUnitCube().segmented(options)
        #expect(result.regions.isEmpty)
        #expect(result.fits.isEmpty)
        #expect(result.truncatedTriangleCount == 12)
    }
}
