// AlignResult — successful result of Mesh.aligned(to:options:).

import simd

/// The rigid transform that best aligns one mesh onto another (point-to-plane ICP).
public struct AlignResult: Sendable {
    /// The rigid transform mapping the SOURCE mesh's original vertex positions onto the
    /// reference mesh's frame: `transform * SIMD4(sourceVertex, 1)`.
    public let transform: simd_double4x4

    /// Point-to-plane residual RMS at the returned transform, over the final surviving
    /// (distance-capped, trimmed) correspondence set.
    public let residualRMS: Double

    /// Number of ICP refinement iterations actually run (not counting the PCA pre-align step).
    public let iterations: Int

    /// `true` when the residual RMS stopped improving meaningfully before `maxIterations` was
    /// reached. `false` means either `maxIterations` was exhausted, or correspondence search
    /// broke down (too few surviving correspondences to solve the next increment) before that.
    public let converged: Bool

    public init(transform: simd_double4x4, residualRMS: Double, iterations: Int, converged: Bool) {
        self.transform = transform
        self.residualRMS = residualRMS
        self.iterations = iterations
        self.converged = converged
    }
}
