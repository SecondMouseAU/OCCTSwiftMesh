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

    @Test("Unwelded curved body still merges its shattered barrel — internal weld path holds beyond the box case")
    func unweldedCurvedBodyStillSegments() {
        let mesh = unwelded(coarseCappedCylinderMesh(radius: 4, sides: 12, rings: 5, height: 4))
        let result = mesh.segmented()
        #expect(result.regions.count == 3)
        let cylinders = result.fits.filter { $0.kind == .cylinder }
        let planes = result.fits.filter { $0.kind == .plane }
        #expect(cylinders.count == 1)
        #expect(planes.count == 2)
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

    @Test("The normal (under-cap) path never reports the fit-gated merge pass as skipped")
    func fitMergeNotSkippedInNormalPath() {
        #expect(!weldedUnitCube().segmented().fitMergeSkipped)
    }
}

@Suite("RegionMerging.merge — fit-merge-skipped diagnostic (issue #20 item 1)")
struct FitMergeSkippedTests {
    @Test("When even the coplanar pre-merge can't get under the cap, the fit-gated pass is skipped and reported")
    func skipIsReportedWhenCapExceeded() {
        let mesh = weldedUnitCube()
        let normals = mesh.faceNormals()
        let adjacency = mesh.triangleAdjacency()
        let seeds = Mesh.segmentSmoothRegions(triangleCount: mesh.triangleCount, normals: normals,
                                              adjacency: adjacency, maxDihedralDegrees: 20)
            .map { MeshRegion(triangleIndices: $0,
                              area: Mesh.area(ofTriangles: $0, vertices: mesh.vertices, indices: mesh.indices)) }
        // 6 box faces, 90° apart — the ~2° coplanar pre-merge can't touch them, so the region
        // count stays at 6, above the artificially tiny cap below.
        #expect(seeds.count == 6)

        var lo = mesh.vertices[0], hi = mesh.vertices[0]
        for p in mesh.vertices { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let bodyDiag = Double(simd_length(hi - lo))

        let (regions, fits, skipped) = RegionMerging.merge(
            vertices: mesh.vertices, indices: mesh.indices, regions: seeds, faceNormals: normals,
            adjacency: adjacency, bodyDiag: bodyDiag, relativeTolerance: 0.004, maxMergeAngleDegrees: 50,
            maxRegionsToMerge: 3)
        #expect(skipped)
        #expect(regions.count == 6)   // unmerged seed regions — the fit-gated pass never ran
        #expect(fits.count == 6)
    }

    @Test("A single region (nothing to merge) is not reported as skipped")
    func singleRegionIsNotSkipped() {
        let mesh = weldedUnitCube()
        let region = MeshRegion(triangleIndices: Array(0..<mesh.triangleCount),
                                area: Mesh.area(ofTriangles: Array(0..<mesh.triangleCount),
                                                vertices: mesh.vertices, indices: mesh.indices))
        let normals = mesh.faceNormals()
        let adjacency = mesh.triangleAdjacency()
        let (_, _, skipped) = RegionMerging.merge(
            vertices: mesh.vertices, indices: mesh.indices, regions: [region], faceNormals: normals,
            adjacency: adjacency, bodyDiag: 1, relativeTolerance: 0.004, maxMergeAngleDegrees: 50)
        #expect(!skipped)
    }
}

@Suite("PrimitiveFitter.bestFit — region-local floor (issue #20 item 4)")
struct PrimitiveFitterFloorTests {
    @Test("A shallow cylindrical arc classifies as a cylinder via a region-local tie-break floor")
    func shallowArcClassifiesAsCylinder() {
        let mesh = shallowCylindricalArcMesh(radius: 500, widthUnits: 200, axialUnits: 200)
        let allTriangles = Array(0..<mesh.triangleCount)
        let region = MeshRegion(triangleIndices: allTriangles,
                                area: Mesh.area(ofTriangles: allTriangles, vertices: mesh.vertices, indices: mesh.indices))
        let fit = PrimitiveFitter.bestFit(vertices: mesh.vertices, indices: mesh.indices, region: region,
                                          faceNormals: mesh.faceNormals())
        #expect(fit.kind == .cylinder)
        if let radius = fit.radius { #expect(abs(radius - 500) < 5) }
    }
}
