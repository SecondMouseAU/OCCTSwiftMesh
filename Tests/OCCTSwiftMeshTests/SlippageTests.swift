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
        let mesh = triangularPrismLateralMesh(height: 20)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .extrusion)
        #expect(result.pitch == nil)
        if let axis = result.axisDirection { #expect(abs(abs(axis.z) - 1) < 1e-6) }
    }

    @Test("A helicoid strip classifies as a helix, with the axis and pitch it was built from")
    func helicoidClassifiesAsHelix() {
        let meshPitch: Float = 6
        let mesh = helicoidStripMesh(innerRadius: 2, outerRadius: 5, pitch: meshPitch, turns: 3)
        let result = mesh.slippage(forTriangles: Array(0..<mesh.triangleCount))
        #expect(result.kind == .helix)
        if let axis = result.axisDirection { #expect(abs(abs(axis.z) - 1) < 1e-2) }
        // `pitch` is translation per RADIAN of rotation; the fixture's `pitch` parameter is
        // translation per full turn (2π radians).
        if let pitch = result.pitch { #expect(abs(abs(pitch) - Double(meshPitch) / (2 * .pi)) < 0.05) }
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
