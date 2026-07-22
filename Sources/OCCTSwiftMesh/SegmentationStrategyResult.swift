// SegmentationStrategyResult — result of Mesh.segmentedAutoSelect(dihedral:ransac:).

/// Which segmentation strategy `Mesh.segmentedAutoSelect(dihedral:ransac:)` picked, and why.
public struct SegmentationStrategyResult: Sendable {
    public enum Strategy: String, Sendable, Equatable {
        /// Dihedral region-growing + primitive-fit merge (`Mesh.segmented(_:)`) — wins decisively
        /// on a single integrated part.
        case dihedral
        /// Schnabel-style RANSAC primitive extraction (`Mesh.segmentedRANSAC(_:)`) — wins on
        /// multi-primitive scenes where region-growing shatters or under-segments.
        case ransac
    }

    /// The winning strategy's result.
    public let result: SegmentedMesh

    /// Which strategy was picked.
    public let strategy: Strategy

    /// `dihedral(_:)`'s substantial-clean-coverage score.
    public let dihedralScore: Double

    /// `ransac(_:)`'s substantial-clean-coverage score.
    public let ransacScore: Double

    public init(result: SegmentedMesh, strategy: Strategy, dihedralScore: Double, ransacScore: Double) {
        self.result = result
        self.strategy = strategy
        self.dihedralScore = dihedralScore
        self.ransacScore = ransacScore
    }
}
