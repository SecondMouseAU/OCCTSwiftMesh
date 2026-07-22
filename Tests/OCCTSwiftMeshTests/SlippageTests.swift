import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.slippage — Gelfand-Guibas slippage analysis")
struct SlippageTests {

    @Test("A flat grid patch classifies as a plane, normal along the patch's own normal")
    func flatPatchClassifiesAsPlane() {
        // amplitude: 0 flattens bumpyPatchMesh's terrain grid into a genuine flat patch — a
        // single box face (4 vertices) is too sparse a point set for the 6-unknown constraint
        // system to resolve reliably.
        let mesh = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 1, amplitude: 0)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .plane)
        #expect(result.pitch == nil)
        if let axis = result.axisDirection { #expect(abs(abs(axis.z) - 1) < 1e-6) }
    }

    @Test("An open cylindrical shell classifies as a cylinder, axis along the barrel's own axis")
    func cylinderShellClassifiesAsCylinder() {
        let mesh = openCylinderShellMesh(radius: 3, height: 5, segments: 24)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .cylinder)
        #expect(result.pitch == nil)
        if let axis = result.axisDirection { #expect(abs(abs(axis.z) - 1) < 1e-3) }
        if let point = result.axisPoint {
            #expect(abs(point.x) < 0.05)
            #expect(abs(point.y) < 0.05)
        }
    }

    @Test("A UV sphere classifies as a sphere, center near the sphere's own center")
    func sphereClassifiesAsSphere() {
        let mesh = sphereMesh(radius: 5, latSegments: 40, lonSegments: 60)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .sphere)
        #expect(result.axisDirection == nil)
        #expect(result.pitch == nil)
        if let center = result.axisPoint { #expect(simd_length(center) < 0.1) }
    }

    @Test("A cone's lateral surface classifies as a surface of revolution, not a cylinder")
    func coneClassifiesAsRevolution() {
        let mesh = coneLateralMesh(baseRadius: 4, height: 10, segments: 24)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .revolution)
        #expect(result.pitch == nil)
        if let axis = result.axisDirection { #expect(abs(abs(axis.z) - 1) < 1e-2) }
        if let point = result.axisPoint {
            #expect(abs(point.x) < 0.1)
            #expect(abs(point.y) < 0.1)
        }
    }

    @Test("A triangular prism's lateral surface classifies as an extrusion along its own axis")
    func prismClassifiesAsExtrusion() {
        let mesh = triangularPrismLateralMesh()   // default height (8) — see the fixture's doc comment
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .extrusion)
        #expect(result.pitch == nil)
        if let axis = result.axisDirection { #expect(abs(abs(axis.z) - 1) < 1e-3) }
    }

    @Test("A helicoid strip classifies as a helix, with the axis and pitch it was built from")
    func helicoidClassifiesAsHelix() {
        let mesh = helicoidStripMesh()   // default pitch/turns — see the fixture's doc comment
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .helix)
        if let axis = result.axisDirection { #expect(abs(abs(axis.z) - 1) < 1e-2) }
        // `pitch` is translation per RADIAN of rotation; the fixture's `pitch` parameter is
        // translation per full turn (2π radians).
        if let pitch = result.pitch { #expect(abs(abs(pitch) - 20.0 / (2 * .pi)) < 1.0) }
    }

    @Test("A bumpy freeform patch has no continuous slippable motion")
    func bumpyPatchClassifiesAsFreeform() {
        let mesh = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 1)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .freeform)
        #expect(result.axisPoint == nil)
        #expect(result.axisDirection == nil)
        #expect(result.pitch == nil)
    }

    @Test("An empty triangle list is freeform, not a crash")
    func emptyTriangleListIsFreeform() {
        let result = weldedUnitCube().slippage(forTriangles: [])
        #expect(result.kind == .freeform)
        #expect(result.eigenRatios.count == 6)
    }

    @Test("eigenRatios always has 6 entries, ascending, last entry exactly 1")
    func eigenRatiosShapeIsStable() {
        let mesh = openCylinderShellMesh()
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.eigenRatios.count == 6)
        #expect(result.eigenRatios == result.eigenRatios.sorted())
        #expect(abs((result.eigenRatios.last ?? 0) - 1) < 1e-12)
    }

    @Test("maxSamples subsampling still classifies correctly on a dense region")
    func maxSamplesSubsamplingStillClassifies() {
        let mesh = sphereMesh(radius: 5, latSegments: 40, lonSegments: 60)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount), maxSamples: 400)
        #expect(result.kind == .sphere)
    }

    @Test("Slippage classification is deterministic across repeated calls")
    func determinism() {
        let mesh = coneLateralMesh()
        let a = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        let b = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(a == b)
    }
}

/// Every kind, put through a GENERIC (non-axis-aligned) pose. This is the regression coverage
/// for the basis-invariance fix: in an axis-aligned fixture the slippable null space happens to
/// line up with R^6's coordinate axes and Jacobi's eigenvectors come out "pure" (a lone
/// translation or lone rotation) by construction-time coincidence. Rotating the input mixes the
/// basis within any multi-dimensional null space (plane's 3-D, cylinder's 2-D) without changing
/// the surface's actual continuous symmetry at all — a per-eigenvector classification that only
/// worked by that coincidence fails here; the subspace-rank classification (`Mesh+Slippage.swift`)
/// must not.
@Suite("Mesh.slippage — generic (rotated + translated) pose invariance")
struct SlippagePoseInvarianceTests {

    /// A fixed, non-axis-aligned rigid transform (a skew rotation axis, plus a modest
    /// translation) — chosen so the null space's basis genuinely mixes, not just rotates rigidly
    /// end-to-end with the object (which alone wouldn't perturb eigenVALUES: cross products
    /// commute with rotation, so a pure rotation of the whole mesh is an orthogonal similarity
    /// transform of the slippage covariance and leaves eigenvalues exactly unchanged — only the
    /// translation component actually perturbs them at all).
    static func generic(_ mesh: Mesh, seed: Double) -> Mesh {
        let axis = simd_normalize(SIMD3<Double>(0.3 + seed, 0.5, 0.8 - seed / 2))
        let r = Mesh.rodrigues(axis: axis, angle: 0.6 + seed)
        let m = Mesh.rigidTransform(rotation: r, translation: SIMD3<Double>(7, -3, 2))
        return transformedMesh(mesh, by: m)
    }

    static let seeds: [Double] = [0.0, 0.1, 0.25, 0.4, 0.7]

    @Test("rotated+translated plane", arguments: seeds)
    func rotatedPlane(seed: Double) {
        let base = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 1, amplitude: 0)
        let mesh = Self.generic(base, seed: seed)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .plane, "seed \(seed): got \(result.kind), ratios \(result.eigenRatios)")
    }

    @Test("rotated+translated cylinder", arguments: seeds)
    func rotatedCylinder(seed: Double) {
        let mesh = Self.generic(openCylinderShellMesh(radius: 3, height: 5, segments: 24), seed: seed)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .cylinder, "seed \(seed): got \(result.kind), ratios \(result.eigenRatios)")
    }

    @Test("rotated+translated sphere", arguments: seeds)
    func rotatedSphere(seed: Double) {
        let mesh = Self.generic(sphereMesh(radius: 5, latSegments: 40, lonSegments: 60), seed: seed)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .sphere, "seed \(seed): got \(result.kind), ratios \(result.eigenRatios)")
    }

    @Test("rotated+translated cone (revolution)", arguments: seeds)
    func rotatedRevolution(seed: Double) {
        let mesh = Self.generic(coneLateralMesh(), seed: seed)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .revolution, "seed \(seed): got \(result.kind), ratios \(result.eigenRatios)")
    }

    @Test("rotated+translated prism (extrusion)", arguments: seeds)
    func rotatedExtrusion(seed: Double) {
        let mesh = Self.generic(triangularPrismLateralMesh(), seed: seed)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .extrusion, "seed \(seed): got \(result.kind), ratios \(result.eigenRatios)")
    }

    @Test("rotated+translated helicoid (helix)", arguments: seeds)
    func rotatedHelix(seed: Double) {
        let mesh = Self.generic(helicoidStripMesh(), seed: seed)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .helix, "seed \(seed): got \(result.kind), ratios \(result.eigenRatios)")
    }

    @Test("rotated+translated freeform patch stays freeform", arguments: seeds)
    func rotatedFreeform(seed: Double) {
        let base = bumpyPatchMesh(xRange: 0...20, yRange: 0...20, segmentsPerUnit: 1)
        let mesh = Self.generic(base, seed: seed)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .freeform, "seed \(seed): got \(result.kind), ratios \(result.eigenRatios)")
    }
}

/// Sphere-center recovery specifically off-origin — the case a real consumer actually has (a
/// scanned dome/boss zone from `segmented()`, whose bounding-box center is nowhere near the
/// sphere's true center). Exercises `Mesh.slippageSphereCenter`'s least-squares solve rather than
/// (incorrectly) averaging each slippable eigenvector's own axis-foot independently.
@Suite("Mesh.slippage — sphere center recovery off-origin")
struct SlippageSphereCenterTests {
    @Test("A translated full sphere reports its true (translated) center")
    func translatedSphere() {
        let base = sphereMesh(radius: 5, latSegments: 40, lonSegments: 60)
        let t = SIMD3<Double>(30, 0, 0)
        let moved = transformedMesh(base, by: Mesh.rigidTransform(rotation: matrix_identity_double3x3, translation: t))
        let result = moved.slippage(forTriangles: Array(0..<moved.triangleCount))
        #expect(result.kind == .sphere)
        if let c = result.axisPoint { #expect(simd_length(c - t) < 0.5, "center off by \(simd_length(c - t))") }
    }

    @Test("An off-center partial-sphere dome reports the true sphere center, not its own centroid")
    func domeReportsTrueCenterNotCentroid() {
        let center = SIMD3<Float>(30, -10, 5)
        let mesh = domeMesh(center: center, radius: 5, capAngleDegrees: 50)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .sphere)
        if let c = result.axisPoint {
            let expected = SIMD3<Double>(center)
            #expect(simd_length(c - expected) < 0.5, "center off by \(simd_length(c - expected))")
        }
    }
}

/// Pins that a coarser tessellation than the other cylinder tests still classifies correctly —
/// the eigenvalue-ratio "how near zero is near enough" boundary is exactly where sampling noise
/// bites hardest.
@Suite("Mesh.slippage — threshold robustness on coarse tessellation")
struct SlippageCoarseTessellationTests {
    @Test("A coarse (few-segment) open cylinder still classifies as a cylinder")
    func coarseCylinderStillClassifies() {
        let mesh = openCylinderShellMesh(radius: 3, height: 5, segments: 8)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .cylinder)
    }
}

