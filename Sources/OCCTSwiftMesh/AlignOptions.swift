// AlignOptions — input parameters for Mesh.aligned(to:options:).

import OCCTSwift

/// Parameters controlling point-to-plane ICP registration.
///
/// ## Example
///
/// ```swift
/// var options = Mesh.AlignOptions()
/// options.trimFraction = 0.2   // more outlier tolerance for a partial-overlap scan
/// let result = scan.aligned(to: cad, options: options)
/// ```
extension Mesh {
    public struct AlignOptions: Sendable {
        /// Maximum number of ICP refinement iterations after the PCA pre-align step.
        public var maxIterations: Int

        /// Correspondences farther apart than this are rejected outright. `nil` (default)
        /// auto-derives `0.15 ×` the reference mesh's bounding-box diagonal.
        public var correspondenceDistanceCap: Double?

        /// After the distance cap, additionally drop the worst `trimFraction` of surviving
        /// correspondences by point-to-plane residual each iteration (trimmed ICP) — robust to
        /// partial overlap between the two meshes.
        public var trimFraction: Double

        /// Run a PCA/bbox pre-alignment pass (centroids + principal axes, trying all 4
        /// orientation-preserving sign combinations and keeping the one with the lowest quick
        /// correspondence residual) before the iterative refinement. Point-to-plane ICP only
        /// converges reliably from a reasonable starting pose.
        public var preAlign: Bool

        /// Sample source points proportional to normal-direction diversity (Rusinkiewicz &
        /// Levoy, "Efficient Variants of the ICP Algorithm", 2001) rather than uniformly. On a
        /// mostly-flat surface with a small feature, uniform sampling lets the flat majority
        /// dominate the correspondence set and the feature "slide" underneath noise; normal-space
        /// sampling gives every distinct normal direction — including the feature's rare one —
        /// comparable representation regardless of how many points share it.
        public var normalSpaceSampling: Bool

        /// Cap on how many source points are used for correspondence search each iteration (the
        /// same fixed sample is reused every iteration, for determinism and speed — resampling
        /// per iteration isn't necessary for convergence and would cost determinism for no
        /// accuracy benefit at a fixed sample size).
        public var maxSamples: Int

        public init(
            maxIterations: Int = 50,
            correspondenceDistanceCap: Double? = nil,
            trimFraction: Double = 0.1,
            preAlign: Bool = true,
            normalSpaceSampling: Bool = true,
            maxSamples: Int = 2000
        ) {
            self.maxIterations = maxIterations
            self.correspondenceDistanceCap = correspondenceDistanceCap
            self.trimFraction = trimFraction
            self.preAlign = preAlign
            self.normalSpaceSampling = normalSpaceSampling
            self.maxSamples = maxSamples
        }
    }
}
