// RANSACSegmentOptions — input parameters for Mesh.segmentedRANSAC(_:).

import OCCTSwift

/// Parameters controlling Schnabel-style RANSAC primitive extraction (see
/// `Mesh.segmentedRANSAC(_:)`), following the Schnabel et al. (2007) conventions.
///
/// ## Example
///
/// ```swift
/// var options = Mesh.RANSACSegmentOptions()
/// options.inlierEpsilon = 0.1   // mesh's own units, e.g. mm
/// let result = mesh.segmentedRANSAC(options)
/// ```
extension Mesh {
    public struct RANSACSegmentOptions: Sendable {
        /// Maximum point-to-primitive distance for a triangle to count as an inlier of a
        /// candidate primitive, in the mesh's own units (an ABSOLUTE distance — Schnabel's own
        /// convention prefers this over a bounding-box-relative fraction at the API surface,
        /// unlike `SegmentOptions.mergeRelativeTolerance`). `0` (default) auto-derives
        /// `0.005 ×` the mesh's bounding-box diagonal.
        public var inlierEpsilon: Double

        /// Spatial proximity radius used to split one candidate's GLOBAL inlier set (which can
        /// span disconnected patches — see the file header) into separate connected clusters,
        /// filtering out inliers that only satisfy the tolerance/normal gates by coincidence
        /// rather than belonging to the same physical feature. `0` (default) auto-derives
        /// `2 ×` the mesh's own mean edge length.
        public var clusterEpsilon: Double

        /// A candidate triangle's face normal must be within this many degrees of the fitted
        /// primitive's own surface normal at that triangle's centroid — OR of that normal's
        /// exact opposite — to count as an inlier. The check is orientation-agnostic
        /// (`|cos deviation|`, not signed) on purpose: a triangle with inconsistent or unknown
        /// winding still lies tangent to the fitted surface, and real scan meshes routinely have
        /// inconsistent/unknown global winding — the same reason `windingNumber`/
        /// `orientationReport` (issue #30) exist at all. Only the tangent-plane alignment is
        /// gated here, never which way the normal happens to point.
        public var maxNormalDeviationDegrees: Float

        /// Minimum inlier-cluster triangle count for a candidate primitive to be accepted as a
        /// found region; smaller clusters are treated as noise and left unclaimed (reported via
        /// `SegmentedMesh.truncatedTriangleCount`).
        public var minSupportCount: Int

        /// Target probability (Schnabel's convention) of not having missed the best candidate
        /// primitive each round — drives the adaptive per-round candidate-trial budget: as a
        /// round's best-found support grows, fewer further trials are needed to be this confident
        /// nothing larger remains undiscovered. Higher costs more candidate evaluations.
        public var successProbability: Double

        /// Hard cap on candidate primitives evaluated per round, regardless of
        /// `successProbability`'s adaptive estimate — bounds worst-case cost on a large mesh.
        public var maxCandidatesPerRound: Int

        /// Triangle count of each deterministic candidate sample fed to `PrimitiveFitter.bestFit`
        /// (see the file header's "candidate generation" note — a small least-squares sample
        /// rather than Schnabel's exact closed-form minimal set per primitive type).
        public var sampleSize: Int

        /// Cap on the number of regions returned. When set and exceeded, the largest `maxRegions`
        /// regions (by area) are kept and the rest counted in `SegmentedMesh.truncatedTriangleCount`.
        public var maxRegions: Int?

        /// Forwarded to the internal weld pass that establishes adjacency for clustering. `0`
        /// (default) auto-derives `1e-6 ×` the mesh's bounding-box diagonal.
        public var weldTolerance: Double

        public init(
            inlierEpsilon: Double = 0,
            clusterEpsilon: Double = 0,
            maxNormalDeviationDegrees: Float = 25,
            minSupportCount: Int = 30,
            successProbability: Double = 0.99,
            maxCandidatesPerRound: Int = 300,
            sampleSize: Int = 6,
            maxRegions: Int? = nil,
            weldTolerance: Double = 0
        ) {
            self.inlierEpsilon = inlierEpsilon
            self.clusterEpsilon = clusterEpsilon
            self.maxNormalDeviationDegrees = maxNormalDeviationDegrees
            self.minSupportCount = minSupportCount
            self.successProbability = successProbability
            self.maxCandidatesPerRound = maxCandidatesPerRound
            self.sampleSize = sampleSize
            self.maxRegions = maxRegions
            self.weldTolerance = weldTolerance
        }
    }
}
