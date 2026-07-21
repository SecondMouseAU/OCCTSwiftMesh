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

    public init(regions: [MeshRegion], fits: [FittedPrimitive], truncatedTriangleCount: Int) {
        self.regions = regions
        self.fits = fits
        self.truncatedTriangleCount = truncatedTriangleCount
    }
}
