// Mesh+Simplify; QEM decimation via vendored meshoptimizer.

import Foundation
import OCCTMeshOptimizer
import OCCTSwift

extension Mesh {
    /// Decimate this mesh using a quadric-error-metric edge-collapse algorithm.
    ///
    /// - Parameter options: target count or reduction ratio, plus optional
    ///   boundary/topology/Hausdorff constraints.
    /// - Returns: a `SimplifiedMesh` carrying the decimated mesh and the
    ///   achieved Hausdorff distance, or `nil` if the options are invalid
    ///   (neither/both targets set, out-of-range values) or the input mesh
    ///   is empty.
    public func simplified(_ options: Mesh.SimplifyOptions) -> SimplifiedMesh? {
        guard let raw = _decimateRaw(options) else { return nil }
        guard let outMesh = Mesh(vertices: raw.vertices, indices: raw.indices) else { return nil }
        return SimplifiedMesh(
            mesh: outMesh,
            beforeTriangleCount: raw.beforeTriangleCount,
            afterTriangleCount: raw.afterTriangleCount,
            hausdorffDistance: raw.hausdorffDistance)
    }

    /// Internal helper that runs validation, extracts input arrays, calls
    /// the bridge, and returns the raw decimation output.
    ///
    /// Exposed at
    /// `internal` visibility so tests can verify the full bridge path
    /// without depending on the OCCTSwift Mesh public initializer.
    internal func _decimateRaw(_ options: Mesh.SimplifyOptions) -> RawDecimationResult? {
        // Validate exactly one target is set.
        let hasCount = options.targetTriangleCount != nil
        let hasRatio = options.targetReduction != nil
        guard hasCount != hasRatio else { return nil }

        let inputTriangleCount = self.triangleCount
        let inputVertexCount = self.vertexCount
        guard inputTriangleCount > 0, inputVertexCount > 0 else { return nil }

        // Resolve target triangle count.
        let targetTriangleCount: Int
        if let count = options.targetTriangleCount {
            guard count >= 1, count <= inputTriangleCount else { return nil }
            targetTriangleCount = count
        } else if let ratio = options.targetReduction {
            guard ratio >= 0.0, ratio <= 1.0 else { return nil }
            targetTriangleCount = max(
                1, Int((Double(inputTriangleCount) * (1.0 - ratio)).rounded()))
        } else {
            return nil  // unreachable; covered by the hasCount != hasRatio guard
        }

        if let cap = options.maxHausdorffDistance, cap < 0 { return nil }

        let vertexFloats = self.vertexData
        let indices = self.indices
        guard !vertexFloats.isEmpty, !indices.isEmpty else { return nil }

        let targetIndexCount = UInt32(targetTriangleCount * 3)
        let indexCount = UInt32(indices.count)
        let vertexCount = UInt32(inputVertexCount)
        let targetError =
            options.maxHausdorffDistance.map { Float($0) } ?? Float.greatestFiniteMagnitude

        var bridgeResult = OCCTMeshSimplifyResult()
        let ok: Bool = vertexFloats.withUnsafeBufferPointer { vbuf in
            indices.withUnsafeBufferPointer { ibuf in
                OCCTMeshSimplify(
                    vbuf.baseAddress,
                    vertexCount,
                    ibuf.baseAddress,
                    indexCount,
                    targetIndexCount,
                    targetError,
                    options.preserveBoundary,
                    options.preserveTopology,
                    &bridgeResult)
            }
        }
        guard ok else { return nil }
        defer { OCCTMeshSimplifyFreeResult(&bridgeResult) }

        let outVertexCount = Int(bridgeResult.vertexCount)
        let outTriangleCount = Int(bridgeResult.triangleCount)
        let outIndexCount = outTriangleCount * 3

        var outVertices: [SIMD3<Float>] = []
        outVertices.reserveCapacity(outVertexCount)
        if let vp = bridgeResult.vertices {
            for i in 0..<outVertexCount {
                outVertices.append(SIMD3<Float>(vp[i * 3], vp[i * 3 + 1], vp[i * 3 + 2]))
            }
        }

        var outIndices: [UInt32] = []
        outIndices.reserveCapacity(outIndexCount)
        if let ip = bridgeResult.indices {
            for i in 0..<outIndexCount {
                outIndices.append(ip[i])
            }
        }

        return RawDecimationResult(
            vertices: outVertices,
            indices: outIndices,
            beforeTriangleCount: Int(bridgeResult.beforeTriangleCount),
            afterTriangleCount: outTriangleCount,
            hausdorffDistance: bridgeResult.hausdorffDistance)
    }
}

/// Internal value type carrying the raw output of QEM decimation before it
/// is wrapped into an `OCCTSwift.Mesh`.
///
/// Pending OCCTSwift#94, this is the
/// surface used by tests to verify the bridge end-to-end.
internal struct RawDecimationResult: Sendable {
    let vertices: [SIMD3<Float>]
    let indices: [UInt32]
    let beforeTriangleCount: Int
    let afterTriangleCount: Int
    let hausdorffDistance: Double
}
