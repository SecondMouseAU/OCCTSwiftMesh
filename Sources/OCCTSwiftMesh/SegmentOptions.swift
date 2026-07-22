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

        /// Order region-growing seeds by ASCENDING per-face curvature (mean of the corners'
        /// `max(|k1|, |k2|)`) instead of raw triangle-index order, AND switch the growing rule's
        /// absorption test from pairwise-adjacent (the default: gate each step against the
        /// CURRENT frontier triangle's own normal) to seed-relative (gate every candidate against
        /// the growing region's fixed SEED normal instead). The pairwise rule's region partition
        /// is a graph-connectivity invariant — reordering seeds alone could never change it, since
        /// two regions can never contend for the same triangle under a rule that doesn't
        /// distinguish who's asking. Seed-relative growing does distinguish that, so flat/
        /// low-curvature regions (processed first, ascending) claim their full extent — their own
        /// seed-relative reach is identical to pairwise for a flat surface — before a
        /// high-curvature blend strip (a fillet, a transition), whose total angular span from its
        /// SEED is now capped at `maxDihedralDegrees` rather than pairwise's unbounded gradual-
        /// drift tolerance, surfaces as its own smaller region once its later, higher-curvature
        /// seed is finally tried. See `segmentSmoothRegions`'s doc comment for the full mechanism.
        ///
        /// `false` (default) preserves the original triangle-index, pairwise-adjacent behavior
        /// exactly — opt-in, since this is a genuine change to the growing rule itself (not just
        /// an ordering tweak), and existing consumers relying on today's segmentation should not
        /// see it shift underfoot. Requires an extra `vertexCurvatures()` pass on the same welded
        /// intermediate `segmented(_:)` already builds — a modest cost, skipped entirely when
        /// this is `false`.
        public var curvatureSeeding: Bool

        public init(
            maxDihedralDegrees: Float = 20,
            mergeRelativeTolerance: Double = 0.004,
            maxMergeAngleDegrees: Float = 50,
            minRegionTriangles: Int = 1,
            maxRegions: Int? = nil,
            weldTolerance: Double = 0,
            curvatureSeeding: Bool = false
        ) {
            self.maxDihedralDegrees = maxDihedralDegrees
            self.mergeRelativeTolerance = mergeRelativeTolerance
            self.maxMergeAngleDegrees = maxMergeAngleDegrees
            self.minRegionTriangles = minRegionTriangles
            self.maxRegions = maxRegions
            self.weldTolerance = weldTolerance
            self.curvatureSeeding = curvatureSeeding
        }
    }
}
