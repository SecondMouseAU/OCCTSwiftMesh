import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.vertexCurvatures — discrete curvature estimation")
struct MeshCurvatureTests {

    @Test("Flat grid: zero curvature everywhere, including boundary vertices")
    func flatPlaneIsZeroCurvature() {
        let mesh = flatGridMesh()
        let curvatures = mesh.vertexCurvatures()
        #expect(curvatures.count == mesh.vertexCount)
        for c in curvatures {
            #expect(abs(c.k1) < 1e-4)
            #expect(abs(c.k2) < 1e-4)
            #expect(abs(c.mean) < 1e-4)
            #expect(abs(c.gaussian) < 1e-6)
        }
    }

    @Test("Sphere zone: k1 == k2 == 1/radius at interior vertices")
    func sphereInteriorCurvatureMatchesAnalytic() {
        let radius: Float = 10
        let rings = 10
        let segments = 24
        let mesh = sphereZoneMesh(radius: radius, latitudeSpanDegrees: 60, segments: segments, rings: rings)
        let curvatures = mesh.vertexCurvatures()
        let expected = 1.0 / Double(radius)

        // Interior rings only, excluding a 2-ring buffer from the top/bottom open boundary: the
        // boundary rings themselves have an incomplete triangle fan (less accurate vertex
        // normals), and that inaccuracy propagates one ring further inward through the shared
        // faces used to fit THOSE vertices' neighbors — verified empirically (rings 0/1 and
        // 8/9 of this 10-ring mesh show a several-% bias; rings 2-7 converge to <0.1%).
        for ring in 2..<(rings - 2) {
            for i in 0..<segments {
                let c = curvatures[ring * segments + i]
                #expect(abs(c.k1 - expected) < expected * 0.05)
                #expect(abs(c.k2 - expected) < expected * 0.05)
                #expect(abs(c.gaussian - expected * expected) < expected * expected * 0.1)
            }
        }
    }

    @Test("Cylinder: k1 == 1/radius, k2 == 0, d2 along the axis, at interior vertices")
    func cylinderInteriorCurvatureMatchesAnalytic() {
        let radius: Float = 6
        let rings = 8
        let segments = 24
        let mesh = openCylinderMultiRingMesh(radius: radius, height: 20, segments: segments, rings: rings)
        let curvatures = mesh.vertexCurvatures()
        let expected = 1.0 / Double(radius)
        let axis = SIMD3<Double>(0, 0, 1)

        for ring in 2..<(rings - 2) {
            for i in 0..<segments {
                let c = curvatures[ring * segments + i]
                #expect(abs(c.k1 - expected) < expected * 0.05)
                #expect(abs(c.k2) < expected * 0.05)
                // d2 (the k2 == 0 direction) should be parallel to the cylinder's axis; d1
                // (circumferential) should be perpendicular to it. Sign/labeling of d1 vs d2 is
                // unambiguous here since |k1| >> |k2|.
                let d2Alignment = abs(simd_dot(SIMD3<Double>(c.d2), axis))
                #expect(d2Alignment > 0.98)
                #expect(abs(simd_dot(SIMD3<Double>(c.d1), axis)) < 0.2)
            }
        }
    }

    @Test("Unwelded input reports zero curvature everywhere — documents the weld precondition")
    func unweldedInputIsZeroCurvature() {
        let mesh = unwelded(sphereZoneMesh())
        let curvatures = mesh.vertexCurvatures()
        #expect(curvatures.count == mesh.vertexCount)
        for c in curvatures {
            #expect(c.k1 == 0)
            #expect(c.k2 == 0)
        }
    }

    @Test("A sliver triangle glued onto an edge doesn't propagate NaN, and leaves every vertex's curvature unchanged")
    func sliverTriangleIsExcludedNotPropagated() {
        let base = sphereZoneMesh()
        let contaminated = sphereZoneMeshWithSliver()
        let baseCurvatures = base.vertexCurvatures()
        let contaminatedCurvatures = contaminated.vertexCurvatures()

        #expect(contaminatedCurvatures.count == base.vertexCount + 1)
        for c in contaminatedCurvatures {
            #expect(c.k1.isFinite)
            #expect(c.k2.isFinite)
            #expect(c.d1.x.isFinite && c.d1.y.isFinite && c.d1.z.isFinite)
            #expect(c.d2.x.isFinite && c.d2.y.isFinite && c.d2.z.isFinite)
        }
        // The sliver's own two shared corners (0, 1) and every untouched vertex are numerically
        // unaffected — the sliver's contribution is excluded from the curvature FIT entirely
        // (guard on area/aspect), not merely down-weighted. (Not bit-identical: appending the
        // sliver triangle also changes which face is "last" to set corners 0/1's arbitrary
        // initial tangent-frame pick — see the file header — a different but equally valid
        // orthonormal basis, so the accumulation happens in a different order and picks up
        // ~1e-6-level floating-point noise, even though the underlying tensor contributions
        // summed are identical.)
        for i in 0..<base.vertexCount {
            #expect(abs(contaminatedCurvatures[i].k1 - baseCurvatures[i].k1) < 1e-5)
            #expect(abs(contaminatedCurvatures[i].k2 - baseCurvatures[i].k2) < 1e-5)
        }
        // The new orphan apex vertex has no OTHER face contributing to it (its only triangle was
        // excluded), so it degrades cleanly to zero rather than NaN.
        let apex = contaminatedCurvatures[base.vertexCount]
        #expect(apex.k1 == 0)
        #expect(apex.k2 == 0)
    }

    @Test("Determinism: repeated calls on identical input are bit-identical")
    func deterministic() {
        let mesh = sphereZoneMesh()
        let a = mesh.vertexCurvatures()
        let b = mesh.vertexCurvatures()
        #expect(a.count == b.count)
        for (x, y) in zip(a, b) {
            #expect(x.k1 == y.k1)
            #expect(x.k2 == y.k2)
            #expect(x.d1 == y.d1)
            #expect(x.d2 == y.d2)
        }
    }

    @Test("d1 and d2 are unit length and mutually perpendicular")
    func principalDirectionsAreOrthonormal() {
        let mesh = sphereZoneMesh()
        for c in mesh.vertexCurvatures() {
            #expect(abs(Double(simd_length(c.d1)) - 1) < 1e-4)
            #expect(abs(Double(simd_length(c.d2)) - 1) < 1e-4)
            #expect(abs(Double(simd_dot(c.d1, c.d2))) < 1e-4)
        }
    }
}
