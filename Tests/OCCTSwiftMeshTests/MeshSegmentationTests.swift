import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

// Segmentation (#17): dihedral region-growing + primitive-fit merge.

private func quad(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, _ d: SIMD3<Float>,
                   into v: inout [SIMD3<Float>], _ idx: inout [UInt32]) {
    let base = UInt32(v.count)
    v.append(contentsOf: [a, b, c, d])
    idx.append(contentsOf: [base, base + 1, base + 2, base, base + 2, base + 3])
}

private func boxMesh(_ sx: Float, _ sy: Float, _ sz: Float) -> Mesh {
    let hx = sx / 2, hy = sy / 2, hz = sz / 2
    var v: [SIMD3<Float>] = []
    var idx: [UInt32] = []
    quad([-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz], into: &v, &idx)
    quad([-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz], into: &v, &idx)
    quad([-hx, -hy, -hz], [hx, -hy, -hz], [hx, -hy, hz], [-hx, -hy, hz], into: &v, &idx)
    quad([hx, hy, -hz], [-hx, hy, -hz], [-hx, hy, hz], [hx, hy, hz], into: &v, &idx)
    quad([-hx, hy, -hz], [-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, hz], into: &v, &idx)
    quad([hx, -hy, -hz], [hx, hy, -hz], [hx, hy, hz], [hx, -hy, hz], into: &v, &idx)
    // Each `quad()` call mints fresh vertices, so weld to get the real edge-connected box.
    return Mesh(vertices: v, indices: idx)!.welded()
}

/// A regular `segments`-sided prism approximating a cylinder — capped at both ends. With few
/// segments the barrel's facet-to-facet dihedral exceeds the default 20° growth threshold, so the
/// barrel shatters into per-facet planes UNLESS the merge pass re-fuses it (the regression this
/// pins). With many segments the dihedral is small enough that region-growing alone yields one
/// barrel region.
private func prismMesh(radius: Float, height: Float, segments: Int) -> Mesh {
    var v: [SIMD3<Float>] = []
    var idx: [UInt32] = []
    let hz = height / 2
    var rim = [(SIMD3<Float>, SIMD3<Float>)]()   // (bottom, top) per angle step
    for i in 0...segments {
        let a = Float(i) / Float(segments) * 2 * .pi
        let x = radius * cos(a), y = radius * sin(a)
        rim.append((SIMD3(x, y, -hz), SIMD3(x, y, hz)))
    }
    for i in 0..<segments {
        let (b0, t0) = rim[i], (b1, t1) = rim[i + 1]
        quad(b0, b1, t1, t0, into: &v, &idx)   // side facet
    }
    // End caps: triangle fans from the centre.
    let bottomCentreIdx = UInt32(v.count)
    v.append(SIMD3(0, 0, -hz))
    for i in 0..<segments {
        let base = UInt32(v.count)
        v.append(rim[i].0); v.append(rim[i + 1].0)
        idx.append(contentsOf: [bottomCentreIdx, base + 1, base])   // wound opposite to the top fan
    }
    let topCentreIdx = UInt32(v.count)
    v.append(SIMD3(0, 0, hz))
    for i in 0..<segments {
        let base = UInt32(v.count)
        v.append(rim[i].1); v.append(rim[i + 1].1)
        idx.append(contentsOf: [topCentreIdx, base, base + 1])
    }
    return Mesh(vertices: v, indices: idx)!.welded()
}

/// A box whose top face has a rectangular BLIND recess (pocket) sunk `depth` into it, centred,
/// with half-extents (rx, ry) strictly inside the top face. Segmenting this should isolate the
/// recess's flat floor as its own region, distinct from the surrounding top rim — the "recess
/// isolated" acceptance case.
private func boxWithRecessMesh(_ sx: Float, _ sy: Float, _ sz: Float,
                                rx: Float, ry: Float, depth: Float) -> Mesh {
    let hx = sx / 2, hy = sy / 2, hz = sz / 2
    var v: [SIMD3<Float>] = []
    var idx: [UInt32] = []
    // 5 untouched faces.
    quad([-hx, -hy, -hz], [-hx, hy, -hz], [hx, hy, -hz], [hx, -hy, -hz], into: &v, &idx)     // bottom (outward -Z)
    quad([-hx, -hy, -hz], [hx, -hy, -hz], [hx, -hy, hz], [-hx, -hy, hz], into: &v, &idx)     // front
    quad([hx, hy, -hz], [-hx, hy, -hz], [-hx, hy, hz], [hx, hy, hz], into: &v, &idx)         // back
    quad([-hx, hy, -hz], [-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, hz], into: &v, &idx)     // left
    quad([hx, -hy, -hz], [hx, hy, -hz], [hx, hy, hz], [hx, -hy, hz], into: &v, &idx)         // right

    let fz = hz - depth
    let o0 = SIMD3<Float>(-hx, -hy, hz), o1 = SIMD3<Float>(hx, -hy, hz), o2 = SIMD3<Float>(hx, hy, hz), o3 = SIMD3<Float>(-hx, hy, hz)
    let i0 = SIMD3<Float>(-rx, -ry, hz), i1 = SIMD3<Float>(rx, -ry, hz), i2 = SIMD3<Float>(rx, ry, hz), i3 = SIMD3<Float>(-rx, ry, hz)
    let f0 = SIMD3<Float>(-rx, -ry, fz), f1 = SIMD3<Float>(rx, -ry, fz), f2 = SIMD3<Float>(rx, ry, fz), f3 = SIMD3<Float>(-rx, ry, fz)
    // Top rim (frame): 4 trapezoids, all coplanar at z = hz.
    quad(o0, o1, i1, i0, into: &v, &idx)
    quad(o1, o2, i2, i1, into: &v, &idx)
    quad(o2, o3, i3, i2, into: &v, &idx)
    quad(o3, o0, i0, i3, into: &v, &idx)
    // Pocket walls (vertical, connecting the inner rim down to the floor).
    quad(i0, i1, f1, f0, into: &v, &idx)
    quad(i1, i2, f2, f1, into: &v, &idx)
    quad(i2, i3, f3, f2, into: &v, &idx)
    quad(i3, i0, f0, f3, into: &v, &idx)
    // Pocket floor.
    quad(f0, f1, f2, f3, into: &v, &idx)
    return Mesh(vertices: v, indices: idx)!.welded()
}

@Suite("Mesh.segmented(_:) — region growing + primitive-fit merge")
struct MeshSegmentationTests {

    @Test("Box: exactly 6 planar regions (one per face), each a perfect plane fit")
    func boxSixRegions() {
        let mesh = boxMesh(10, 6, 4)
        let result = mesh.segmented()
        #expect(result.regions.count == 6)
        #expect(result.fits.count == 6)
        #expect(!result.truncated)
        for fit in result.fits {
            #expect(fit.kind == .plane)
            #expect(fit.residualRMS < 1e-4)
        }
        for region in result.regions {
            #expect(region.triangleIndices.count == 2)
            #expect(region.boundaryLoopCount == 1)
        }
        // Total area recovers the box's surface area.
        let totalArea = result.regions.reduce(0.0) { $0 + $1.area }
        #expect(abs(totalArea - (2 * (10.0 * 6 + 10.0 * 4 + 6.0 * 4))) < 1e-1)
    }

    @Test("12-facet prism: barrel facets re-fuse into ONE cylinder region (the shatter regression)")
    func coarsePrismMergesBarrel() {
        let mesh = prismMesh(radius: 5, height: 20, segments: 12)
        let result = mesh.segmented()
        // Without the merge pass this would be 14 regions (12 side facets + 2 caps); the merge
        // pass must re-fuse the 12 side facets (30° apart, under the default 50° merge-angle
        // cap) back into a single cylinder.
        #expect(result.regions.count == 3)
        let kinds = result.fits.map(\.kind).sorted { $0.rawValue < $1.rawValue }
        #expect(kinds == [.cylinder, .plane, .plane])
        guard let cylFit = result.fits.first(where: { $0.kind == .cylinder }) else {
            Issue.record("expected a cylinder fit among the merged regions")
            return
        }
        #expect(cylFit.residualRMS < 0.5)
        #expect(abs((cylFit.radius ?? 0) - 5) < 0.5)
    }

    @Test("Smooth 36-facet tube: barrel is already ONE region from growing alone (dihedral < threshold)")
    func smoothTubeSingleRegion() {
        let mesh = prismMesh(radius: 5, height: 20, segments: 36)
        let result = mesh.segmented()
        #expect(result.regions.count == 3)   // barrel + 2 caps
        guard let cylFit = result.fits.first(where: { $0.kind == .cylinder }) else {
            Issue.record("expected a cylinder fit")
            return
        }
        #expect(cylFit.residualRMS < 0.1)
    }

    @Test("Box with a blind recess: the recess floor is isolated as its own region")
    func recessIsolated() {
        let mesh = boxWithRecessMesh(20, 20, 10, rx: 4, ry: 3, depth: 2)
        let result = mesh.segmented()
        // 5 untouched faces + top rim + 4 pocket walls + pocket floor = 11.
        #expect(result.regions.count == 11)

        // Two regions face +Z (the top rim and the recess floor); they must be distinguishable by
        // area and by z-position — that's what "isolated" means here, not merged away or missed.
        let upwardFacing = result.regions.filter { simd_dot($0.meanNormal, SIMD3<Float>(0, 0, 1)) > 0.99 }
        #expect(upwardFacing.count == 2)

        let floorArea = Double(4 * 2) * Double(3 * 2)   // rx*2 x ry*2
        guard let floor = upwardFacing.first(where: { abs($0.area - floorArea) < 1e-2 }) else {
            Issue.record("no region matched the recess floor's expected area (\(floorArea))")
            return
        }
        #expect(abs(floor.bboxMin.z - 3) < 1e-4)   // hz(5) - depth(2) = 3
        #expect(abs(floor.bboxMax.z - 3) < 1e-4)
        #expect(floor.triangleIndices.count == 2)

        guard let rim = upwardFacing.first(where: { $0.area != floor.area }) else {
            Issue.record("no distinct top-rim region found")
            return
        }
        #expect(abs(rim.bboxMax.z - 5) < 1e-4)   // hz
        #expect(rim.area > floor.area)   // the frame has far more area than the small pocket floor
    }

    @Test("segmented() is deterministic across repeated calls")
    func segmentationDeterministic() {
        let mesh = prismMesh(radius: 5, height: 20, segments: 12)
        let r1 = mesh.segmented()
        let r2 = mesh.segmented()
        #expect(r1 == r2)
    }

    @Test("maxRegions caps the result and reports truncation explicitly")
    func maxRegionsTruncates() {
        let mesh = boxMesh(10, 6, 4)
        let result = mesh.segmented(SegmentOptions(maxRegions: 3))
        #expect(result.regions.count == 3)
        #expect(result.truncated)
    }
}
