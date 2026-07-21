// MeshSegmentation — dihedral region-growing + primitive-fit merge (#17): `Mesh.segmented(_:)`,
// the public entry point. Segment triangles into smoothly-connected regions (region-grow across
// shared edges, breaking at sharp dihedral angles), then run the mandatory `RegionMerging` pass
// so coarse tessellation doesn't shatter curved surfaces into one region per facet.
//
// Ported from OCCTReconstruct's `RegionSegmentation.swift` (`segmentSmoothRegions`). First-order
// only (face normals); no curvature term — a curvature-seeded variant is deliberately out of
// scope here (needs a curvature estimator first, tracked as a follow-up).
//
// Precondition: expects WELDED input (`welded(tolerance:)`). Unwelded input (raw STL/scan
// triangle soup) has no shared edges at all, so every triangle becomes its own region.

import simd
import OCCTSwift

/// Options for `Mesh.segmented(_:)`.
public struct SegmentOptions: Sendable, Equatable {
    /// Region-growing breaks when the dihedral angle between adjacent face normals exceeds this.
    public var maxDihedralDegrees: Float
    /// Merge pass acceptance tolerance, as a fraction of the mesh's bbox diagonal.
    public var mergeRelativeTolerance: Double
    /// A region pair can only be considered for merging if even its sharpest shared boundary edge
    /// is below this angle (distinguishes a coarse cylinder's facet seams from a real corner).
    public var maxMergeAngleDegrees: Float
    /// Regions with fewer triangles than this are dropped before merging.
    public var minRegionTriangles: Int
    /// Cap on the number of regions returned. Truncation is reported via `SegmentedMesh.truncated`
    /// — never silent (issue #17's explicit requirement).
    public var maxRegions: Int?
    /// If pre-merge region growing produces more regions than this, the merge pass is skipped
    /// entirely (it's O(regions²) per pass) and regions are returned as-grown. Configurable so
    /// callers on scan-scale input can raise it once pre-merge coplanar-confetti reduction lands.
    public var maxRegionsToMerge: Int

    public init(maxDihedralDegrees: Float = 20, mergeRelativeTolerance: Double = 0.004,
                maxMergeAngleDegrees: Float = 50, minRegionTriangles: Int = 1,
                maxRegions: Int? = nil, maxRegionsToMerge: Int = 1500) {
        self.maxDihedralDegrees = maxDihedralDegrees
        self.mergeRelativeTolerance = mergeRelativeTolerance
        self.maxMergeAngleDegrees = maxMergeAngleDegrees
        self.minRegionTriangles = minRegionTriangles
        self.maxRegions = maxRegions
        self.maxRegionsToMerge = maxRegionsToMerge
    }
}

/// Result of `Mesh.segmented(_:)`: regions largest-first, paired 1:1 with each region's best-fit
/// analytic primitive.
public struct SegmentedMesh: Sendable, Equatable {
    public var regions: [MeshRegion]
    public var fits: [FittedPrimitive]
    /// `true` when `SegmentOptions.maxRegions` truncated the result.
    public var truncated: Bool

    public init(regions: [MeshRegion], fits: [FittedPrimitive], truncated: Bool) {
        self.regions = regions
        self.fits = fits
        self.truncated = truncated
    }
}

public extension Mesh {

    /// Segment the mesh into smoothly-connected surface regions, then merge adjacent regions that
    /// still fit ONE analytic primitive (plane / cylinder / sphere / cone) — the pass that keeps a
    /// coarsely-tessellated curved surface (e.g. a 12-facet cylinder) from shattering into one
    /// region per facet. Every merged region arrives with a fitted primitive (kind, params,
    /// residual RMS) for free.
    ///
    /// Expects WELDED input (`welded(tolerance:)`) — see the file header.
    func segmented(_ options: SegmentOptions = .init()) -> SegmentedMesh {
        let tc = triangleCount
        guard tc > 0 else { return SegmentedMesh(regions: [], fits: [], truncated: false) }

        let normals = faceNormals()
        let adjacency = triangleAdjacency()
        let cosThreshold = cos(options.maxDihedralDegrees * .pi / 180)

        // DFS flood over edge-adjacent triangles, gated on face-normal agreement. Seeded by
        // ascending triangle index, so the region composition (and hence the merge pass fed by
        // it) is deterministic run-to-run.
        var regionOf = [Int](repeating: -1, count: tc)
        var rawRegions: [[Int]] = []
        for seed in 0..<tc where regionOf[seed] == -1 {
            let id = rawRegions.count
            var members: [Int] = []
            var stack = [seed]
            regionOf[seed] = id
            while let t = stack.popLast() {
                members.append(t)
                let nt = normals[t]
                for n in adjacency[t] where regionOf[n] == -1 {
                    if simd_dot(nt, normals[n]) >= cosThreshold {
                        regionOf[n] = id
                        stack.append(n)
                    }
                }
            }
            // `members` accumulates in DFS pop order — canonicalize ascending so a region's
            // triangle list is a stable value independent of traversal order (determinism).
            rawRegions.append(members.sorted())
        }
        rawRegions.sort(by: MeshRegion.orderTriangleSets)
        rawRegions = rawRegions.filter { $0.count >= options.minRegionTriangles }

        let (mergedSets, fits) = RegionMerging.merge(
            mesh: self, regions: rawRegions, faceNormals: normals, adjacency: adjacency,
            relativeTolerance: options.mergeRelativeTolerance,
            maxMergeAngleDegrees: options.maxMergeAngleDegrees,
            maxRegionsToMerge: options.maxRegionsToMerge)

        var finalSets = mergedSets
        var finalFits = fits
        var truncated = false
        if let cap = options.maxRegions, finalSets.count > cap {
            finalSets = Array(finalSets.prefix(cap))     // already largest-first
            finalFits = Array(finalFits.prefix(cap))
            truncated = true
        }

        var finalRegionOf = [Int](repeating: -1, count: tc)
        for (i, tris) in finalSets.enumerated() { for t in tris { finalRegionOf[t] = i } }
        let loopCounts = MeshRegion.boundaryLoopCounts(mesh: self, regionOf: finalRegionOf,
                                                        regionCount: finalSets.count, adjacency: adjacency)
        let regions = finalSets.enumerated().map { i, tris in
            MeshRegion.build(mesh: self, triangleIndices: tris, faceNormals: normals, boundaryLoopCount: loopCounts[i])
        }
        return SegmentedMesh(regions: regions, fits: finalFits, truncated: truncated)
    }
}
