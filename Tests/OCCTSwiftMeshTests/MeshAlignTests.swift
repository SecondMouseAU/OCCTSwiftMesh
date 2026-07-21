import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.aligned(to:options:) — point-to-plane ICP registration")
struct MeshAlignTests {

    @Test("Recovers a known applied rigid transform")
    func recoversKnownTransform() {
        let reference = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 2)
        let rotation = Mesh.rodrigues(axis: simd_normalize(SIMD3<Double>(0.3, 1, 0.2)), angle: 0.35)
        let translation = SIMD3<Double>(6, -4, 3)
        let applied = Mesh.rigidTransform(rotation: rotation, translation: translation)
        let source = transformedMesh(reference, by: applied)

        guard let result = source.aligned(to: reference) else {
            Issue.record("alignment returned nil")
            return
        }
        #expect(result.residualRMS < 0.05)

        // result.transform should undo `applied`: composing them maps every reference point back
        // to itself.
        let composed = simd_mul(result.transform, applied)
        for v in reference.vertices {
            let p = SIMD3<Double>(v)
            let mapped = Mesh.apply(composed, p)
            #expect(simd_length(mapped - p) < 0.1)
        }
    }

    @Test("Partial overlap: converges to the correct overlap region's alignment, not a wrong plausible one")
    func partialOverlapConverges() {
        // Two same-size, same-aspect-ratio (20×20) patches of the SAME underlying bumpy surface
        // (shared world coordinates), offset in X — a genuine partial-overlap case (60% overlap,
        // not two identical meshes), with matching overall footprints so PCA pre-align's
        // principal-axis correspondence isn't itself ambiguous (a separate concern from what
        // this test targets: the trim/cap mechanism).
        // A low frequency (period ~105, far larger than the ~28-unit combined patch extent) so
        // the surface has no repeating pattern within the test's range — a periodic surface here
        // would let ICP alias onto a wrong-but-locally-plausible phase shift, which is a fixture
        // problem (a naturally non-repeating real scan wouldn't have this ambiguity), not what
        // this test targets (the trim/cap mechanism itself).
        let patchA = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 2, frequency: 0.06)
        let patchBReference = bumpyPatchMesh(xRange: 8...28, yRange: 0...20, segmentsPerUnit: 2, frequency: 0.06)

        let rotation = Mesh.rodrigues(axis: SIMD3<Double>(0, 0, 1), angle: 0.05)
        let translation = SIMD3<Double>(1.5, -0.8, 0.4)
        let applied = Mesh.rigidTransform(rotation: rotation, translation: translation)
        let patchBSource = transformedMesh(patchBReference, by: applied)

        var options = Mesh.AlignOptions()
        options.trimFraction = 0.45   // ~40% of patchB (x in [20, 28]) falls outside patchA
        guard let result = patchBSource.aligned(to: patchA, options: options) else {
            Issue.record("alignment returned nil")
            return
        }
        let composed = simd_mul(result.transform, applied)
        // Check only points that actually lie in the overlap region (patchBReference's own
        // vertices there coincide with patchA's surface).
        var checked = 0
        for v in patchBReference.vertices where v.x >= 8.5 && v.x <= 19.5 {
            let p = SIMD3<Double>(v)
            let mapped = Mesh.apply(composed, p)
            #expect(simd_length(mapped - p) < 0.3)
            checked += 1
        }
        #expect(checked > 10)
    }

    @Test("Normal-space sampling guarantees a minority feature-normal direction gets representation; uniform sampling can miss it entirely")
    func normalSpaceSamplingRepresentsMinorityFeature() {
        // Off-center, off-diagonal bump placement: a centered bump sits exactly on the raster
        // path uniform even-stride sampling walks on a regular row-major grid (row·cols + col
        // aliases with the stride near the diagonal), which would let uniform sampling hit the
        // feature by grid-aliasing coincidence rather than genuine representation — not what this
        // test is checking.
        let mesh = flatWithBumpMesh(bumpCenter: SIMD2(7, 29), bumpSigma: 1.5)
        let normals = mesh.vertexNormals().map { SIMD3<Double>($0) }

        // "Feature" vertices: those whose normal deviates meaningfully (> 10°) from the flat
        // majority's (0, 0, 1) — a small minority by construction (a small, localized bump).
        let featureIndices = Set(normals.indices.filter { i in
            let z = max(-1, min(1, normals[i].z))
            return acos(z) * 180 / .pi > 10
        })
        #expect(!featureIndices.isEmpty)
        #expect(Double(featureIndices.count) / Double(normals.count) < 0.05)

        let budget = 40
        let normalSpacePicks = Set(Mesh.normalSpaceSample(normals: normals, count: budget))
        let uniformPicks = Set(Mesh.uniformSample(total: normals.count, take: budget))

        #expect(!normalSpacePicks.isDisjoint(with: featureIndices))
        #expect(uniformPicks.isDisjoint(with: featureIndices))
    }

    @Test("End to end: normal-space sampling on the flat+feature mesh recovers an in-plane translation a flat plane alone couldn't disambiguate")
    func normalSpaceSamplingEndToEndRecoversInPlaneShift() {
        let reference = flatWithBumpMesh()
        let translation = SIMD3<Double>(3, 2, 0)
        let applied = Mesh.rigidTransform(rotation: matrix_identity_double3x3, translation: translation)
        let source = transformedMesh(reference, by: applied)

        var options = Mesh.AlignOptions()
        options.maxSamples = 40
        options.normalSpaceSampling = true
        guard let result = source.aligned(to: reference, options: options) else {
            Issue.record("alignment returned nil")
            return
        }
        let composed = simd_mul(result.transform, applied)
        let origin = SIMD3<Double>(0, 0, 0)
        #expect(simd_length(Mesh.apply(composed, origin) - origin) < 0.5)
    }

    @Test("Determinism: repeated calls on identical input are bit-identical")
    func deterministic() {
        let reference = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 2)
        let applied = Mesh.rigidTransform(
            rotation: Mesh.rodrigues(axis: simd_normalize(SIMD3<Double>(0.1, 0.9, 0.3)), angle: 0.2),
            translation: SIMD3<Double>(2, 1, 1))
        let source = transformedMesh(reference, by: applied)

        let a = source.aligned(to: reference)
        let b = source.aligned(to: reference)
        guard let a, let b else {
            Issue.record("alignment returned nil")
            return
        }
        #expect(a.transform == b.transform)
        #expect(a.residualRMS == b.residualRMS)
        #expect(a.iterations == b.iterations)
        #expect(a.converged == b.converged)
    }

    @Test("Too few points returns nil rather than crashing")
    func tooFewPointsReturnsNil() {
        let tiny = Mesh(vertices: [SIMD3(0, 0, 0), SIMD3(1, 0, 0)], indices: [0, 1, 0])!
        let reference = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 2)
        #expect(tiny.aligned(to: reference) == nil)
        #expect(reference.aligned(to: tiny) == nil)
    }
}
