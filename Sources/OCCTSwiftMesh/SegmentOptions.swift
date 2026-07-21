// SegmentOptions — input parameters for Mesh.segmented(_:).

import OCCTSwift

/// Parameters controlling dihedral region-growing + primitive-fit merge.
///
/// ## Example
///
/// ```swift
/// var options = Mesh.SegmentOptions()
/// options.maxDihedralDegrees = 15
/// let segmented = mesh.segmented(options)
/// ```
extension Mesh {
    public struct SegmentOptions: Sendable {
        /// Region-growing breaks where the dihedral angle between adjacent face normals
        /// exceeds this threshold.
        public var maxDihedralDegrees: Float

        /// Merge tolerance as a fraction of the mesh's bounding-box diagonal: a merge is
        /// accepted only when the union still fits a single primitive within
        /// `max(mergeRelativeTolerance × bboxDiagonal, 1e-6)`.
        public var mergeRelativeTolerance: Double

        /// Two adjacent regions are only merge-eligible when even their sharpest shared
        /// boundary edge is below this dihedral angle — what distinguishes a coarse
        /// cylinder's shallow facet seams from a box's 90° corner.
        public var maxMergeAngleDegrees: Float

        /// Regions smaller than this (after growing + merging) are dropped from the result
        /// and their triangles counted in `SegmentedMesh.truncatedTriangleCount`.
        public var minRegionTriangles: Int

        /// Cap on the number of regions returned. When set and exceeded, the largest
        /// `maxRegions` regions are kept and the rest counted in
        /// `SegmentedMesh.truncatedTriangleCount` — truncation is always reported, never silent.
        public var maxRegions: Int?

        /// Forwarded to the internal weld pass that establishes adjacency. `0` (default)
        /// auto-derives `1e-6 ×` the mesh's bounding-box diagonal.
        public var weldTolerance: Double

        public init(
            maxDihedralDegrees: Float = 20,
            mergeRelativeTolerance: Double = 0.004,
            maxMergeAngleDegrees: Float = 50,
            minRegionTriangles: Int = 1,
            maxRegions: Int? = nil,
            weldTolerance: Double = 0
        ) {
            self.maxDihedralDegrees = maxDihedralDegrees
            self.mergeRelativeTolerance = mergeRelativeTolerance
            self.maxMergeAngleDegrees = maxMergeAngleDegrees
            self.minRegionTriangles = minRegionTriangles
            self.maxRegions = maxRegions
            self.weldTolerance = weldTolerance
        }
    }
}
