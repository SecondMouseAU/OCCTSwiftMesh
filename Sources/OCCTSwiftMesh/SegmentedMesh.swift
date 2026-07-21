// SegmentedMesh — successful result of Mesh.segmented(_:).

/// A mesh split into surface regions, each with its best-fit primitive.
public struct SegmentedMesh: Sendable {
    /// Segmented regions, largest-first. `fits[i]` is the primitive fitted to `regions[i]`.
    public let regions: [MeshRegion]
    public let fits: [FittedPrimitive]

    /// Triangles excluded from `regions` because the raw result exceeded
    /// `SegmentOptions.maxRegions` (the smallest regions were dropped) or fell under
    /// `SegmentOptions.minRegionTriangles`. `0` when nothing was truncated.
    public let truncatedTriangleCount: Int

    /// `true` when the coplanar pre-merge alone couldn't get the raw region count under the
    /// internal fit-gated-merge cap, so the primitive-fit merge pass was skipped entirely —
    /// `regions`/`fits` are the UNMERGED seed regions from dihedral growing (plus coplanar
    /// pre-merge), not the fully-merged result. Coarse-tessellation "confetti" will not have
    /// been collapsed. `false` in the normal case where the fit-gated pass ran.
    public let fitMergeSkipped: Bool

    public init(regions: [MeshRegion], fits: [FittedPrimitive], truncatedTriangleCount: Int,
                fitMergeSkipped: Bool = false) {
        self.regions = regions
        self.fits = fits
        self.truncatedTriangleCount = truncatedTriangleCount
        self.fitMergeSkipped = fitMergeSkipped
    }
}
