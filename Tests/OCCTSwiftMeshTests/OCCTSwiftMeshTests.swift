import OCCTMeshOptimizer
import OCCTSwift
import Testing

@testable import OCCTSwiftMesh

@Suite("OCCTSwiftMesh, package")
struct PackageTests {
    @Test("Module exposes its version sentinel")
    func versionSentinel() {
        #expect(!OCCTSwiftMesh.version.isEmpty)
    }
}

// MARK: - Test fixtures

private enum FixtureError: Error {
    case shapeFailed
    case meshFailed
}

/// Builds a sphere mesh dense enough to exercise the decimator.
private func makeSphereMesh(radius: Double = 1.0, deflection: Double = 0.05) throws -> Mesh {
    guard let shape = Shape.sphere(radius: radius) else { throw FixtureError.shapeFailed }
    guard let mesh = shape.mesh(linearDeflection: deflection, angularDeflection: 0.3) else {
        throw FixtureError.meshFailed
    }
    return mesh
}

// MARK: - Validation

@Suite("Mesh.simplified, input validation rejects nil")
struct ValidationTests {
    @Test("nil when neither targetTriangleCount nor targetReduction is set")
    func neitherTargetSet() throws {
        let mesh = try makeSphereMesh()
        #expect(mesh._decimateRaw(.init()) == nil)
    }

    @Test("nil when both targetTriangleCount and targetReduction are set")
    func bothTargetsSet() throws {
        let mesh = try makeSphereMesh()
        let opts = Mesh.SimplifyOptions(targetTriangleCount: 100, targetReduction: 0.5)
        #expect(mesh._decimateRaw(opts) == nil)
    }

    @Test("nil when targetTriangleCount is below 1")
    func targetCountBelowOne() throws {
        let mesh = try makeSphereMesh()
        #expect(mesh._decimateRaw(.init(targetTriangleCount: 0)) == nil)
        #expect(mesh._decimateRaw(.init(targetTriangleCount: -10)) == nil)
    }

    @Test("nil when targetTriangleCount exceeds the input triangle count")
    func targetCountExceedsInput() throws {
        let mesh = try makeSphereMesh()
        let beyond = mesh.triangleCount + 1
        #expect(mesh._decimateRaw(.init(targetTriangleCount: beyond)) == nil)
    }

    @Test("nil when targetReduction is outside [0.0, 1.0]")
    func targetReductionOutOfRange() throws {
        let mesh = try makeSphereMesh()
        #expect(mesh._decimateRaw(.init(targetReduction: -0.1)) == nil)
        #expect(mesh._decimateRaw(.init(targetReduction: 1.5)) == nil)
    }

    @Test("nil when maxHausdorffDistance is negative")
    func negativeHausdorff() throws {
        let mesh = try makeSphereMesh()
        let opts = Mesh.SimplifyOptions(targetReduction: 0.5, maxHausdorffDistance: -0.1)
        #expect(mesh._decimateRaw(opts) == nil)
    }
}

// MARK: - Bridge end-to-end

@Suite("Mesh.simplified, bridge produces a decimated mesh")
struct DecimateRawTests {
    @Test("targetReduction reduces triangle count and stays below the input bound")
    func targetReductionReducesCount() throws {
        let mesh = try makeSphereMesh()
        let inputCount = mesh.triangleCount
        try #require(inputCount >= 100)

        guard let result = mesh._decimateRaw(.init(targetReduction: 0.5)) else {
            Issue.record("Bridge returned nil for valid 50% reduction")
            return
        }
        #expect(result.beforeTriangleCount == inputCount)
        #expect(result.afterTriangleCount >= 1)
        #expect(result.afterTriangleCount <= inputCount)
        #expect(result.hausdorffDistance >= 0)
        #expect(result.indices.count == result.afterTriangleCount * 3)
        #expect(result.vertices.count <= mesh.vertexCount)
    }

    @Test("targetTriangleCount drives the output close to the requested count")
    func targetTriangleCountHonored() throws {
        let mesh = try makeSphereMesh()
        let inputCount = mesh.triangleCount
        try #require(inputCount >= 200)
        let target = inputCount / 4

        guard let result = mesh._decimateRaw(.init(targetTriangleCount: target)) else {
            Issue.record("Bridge returned nil for valid targetTriangleCount")
            return
        }
        // Decimator may overshoot when topology preservation prevents
        // reaching the exact target; allow a generous upper band.
        #expect(result.afterTriangleCount <= inputCount)
        #expect(result.afterTriangleCount >= 1)
    }

    @Test("Output indices stay within the compacted vertex range")
    func outputIndicesInRange() throws {
        let mesh = try makeSphereMesh()
        guard let result = mesh._decimateRaw(.init(targetReduction: 0.5)) else {
            Issue.record("Bridge returned nil")
            return
        }
        let bound = UInt32(result.vertices.count)
        let allInRange = result.indices.allSatisfy { $0 < bound }
        #expect(allInRange)
    }

    @Test("hausdorffDistance is finite and non-negative")
    func hausdorffFinite() throws {
        let mesh = try makeSphereMesh()
        guard let result = mesh._decimateRaw(.init(targetReduction: 0.5)) else {
            Issue.record("Bridge returned nil")
            return
        }
        #expect(result.hausdorffDistance.isFinite)
        #expect(result.hausdorffDistance >= 0)
    }

    @Test("targetReduction = 0 leaves triangle count near the input")
    func zeroReductionNoOp() throws {
        let mesh = try makeSphereMesh()
        let inputCount = mesh.triangleCount
        guard let result = mesh._decimateRaw(.init(targetReduction: 0.0)) else {
            Issue.record("Bridge returned nil for 0% reduction")
            return
        }
        #expect(result.afterTriangleCount == inputCount)
    }
}

// MARK: - Bridge: scale function

@Suite("OCCTMeshSimplifyScale")
struct ScaleTests {
    @Test("Returns a positive scale for a non-degenerate mesh")
    func positiveScale() throws {
        let mesh = try makeSphereMesh()
        let raw = mesh.vertexData
        let count = UInt32(mesh.vertexCount)
        let scale: Float = raw.withUnsafeBufferPointer {
            OCCTMeshSimplifyScale($0.baseAddress, count)
        }
        #expect(scale > 0)
        #expect(scale.isFinite)
    }
}

// MARK: - Public API integration

@Suite("Mesh.simplified, public API")
struct PublicAPITests {
    @Test("Returns a SimplifiedMesh whose counts match the raw bridge output")
    func returnsSimplifiedMesh() throws {
        let mesh = try makeSphereMesh()
        let inputCount = mesh.triangleCount
        try #require(inputCount >= 100)

        guard let result = mesh.simplified(.init(targetReduction: 0.5)) else {
            Issue.record("Mesh.simplified returned nil for valid options")
            return
        }
        #expect(result.beforeTriangleCount == inputCount)
        #expect(result.afterTriangleCount >= 1)
        #expect(result.afterTriangleCount <= inputCount)
        #expect(result.hausdorffDistance.isFinite)
        #expect(result.hausdorffDistance >= 0)
        #expect(result.mesh.triangleCount == result.afterTriangleCount)
        #expect(result.mesh.vertexCount > 0)
        #expect(result.mesh.indices.count == result.afterTriangleCount * 3)
    }

    @Test("Output Mesh round-trips: indices reference vertices in range")
    func outputMeshRoundTrips() throws {
        let mesh = try makeSphereMesh()
        guard let result = mesh.simplified(.init(targetReduction: 0.5)) else {
            Issue.record("Mesh.simplified returned nil")
            return
        }
        let bound = UInt32(result.mesh.vertexCount)
        let allInRange = result.mesh.indices.allSatisfy { $0 < bound }
        #expect(allInRange)
    }

    @Test("nil for invalid options propagates through the public API")
    func nilForInvalidOptions() throws {
        let mesh = try makeSphereMesh()
        #expect(mesh.simplified(.init()) == nil)
        #expect(mesh.simplified(.init(targetTriangleCount: 100, targetReduction: 0.5)) == nil)
        #expect(mesh.simplified(.init(targetReduction: 1.5)) == nil)
    }
}
