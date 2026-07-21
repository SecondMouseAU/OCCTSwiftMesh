// RegionMerging — agglomerative merge that repairs coarse-tessellation shattering (#17). A
// low-poly cylinder or cone has adjacent facets >20° apart, so smooth-region growing breaks it
// into per-facet planes. This pass greedily merges adjacent regions whose union fits a single
// primitive within tolerance — collapsing those facets back into one cylinder/cone — while
// leaving real edges (where the union fits nothing well) intact. Without it, curved bodies
// "shatter into confetti" (the regression `SegmentationBakeoffTests` pins in the reference).
//
// Ported from OCCTReconstruct's `RegionMerging.swift`, adapted to operate on raw triangle-index
// arrays (`[[Int]]`) rather than a `MeshRegion` list — this package's `MeshRegion` is the richer,
// final-result type built only once per FINAL (post-merge) region; building it for every
// pre-merge shattered region would be pure waste on a fine mesh.

import simd
import OCCTSwift

/// Internal implementation detail of `Mesh.segmented(_:)` — not part of the public API surface.
enum RegionMerging {

    /// Merge regions by primitive fit. Returns the merged regions' triangle sets (largest-first)
    /// and each merged region's best fit. `relativeTolerance` is a fraction of the mesh's bbox
    /// diagonal.
    static func merge(mesh: Mesh, regions: [[Int]], faceNormals: [SIMD3<Float>],
                       adjacency: [[Int]], relativeTolerance: Double = 0.004,
                       maxMergeAngleDegrees: Float = 50,
                       maxRegionsToMerge: Int = 1500) -> (regions: [[Int]], fits: [FittedPrimitive]) {
        let n = regions.count
        guard n > 1, n <= maxRegionsToMerge else {
            let fits = regions.map { PrimitiveFitter.bestFit(mesh: mesh, triangleIndices: $0, faceNormals: faceNormals) }
            return (regions, fits)
        }

        let diag = PrimitiveFitter.boundsDiagonal(mesh)
        let mergeTol = max(relativeTolerance * diag, 1e-6)

        var regionOf = [Int](repeating: -1, count: mesh.triangleCount)
        for (rid, region) in regions.enumerated() {
            for t in region { regionOf[t] = rid }
        }

        // Region adjacency, tracking the SOFTEST→HARDEST boundary: per pair keep the sharpest
        // dihedral (min dot) across shared facet edges. A pair is mergeable only if even its
        // sharpest shared edge is "soft" (below maxMergeAngle) — this is what distinguishes a
        // coarse cylinder's ~36° facet seams from a box's 90° corner.
        let cosMergeAngle = cos(maxMergeAngleDegrees * .pi / 180)
        func packPair(_ a: Int, _ b: Int) -> UInt64 { (UInt64(min(a, b)) << 32) | UInt64(max(a, b)) }
        var pairMinDot = [UInt64: Float]()
        for t in 0..<mesh.triangleCount {
            let ra = regionOf[t]
            guard ra >= 0 else { continue }
            for nb in adjacency[t] where regionOf[nb] != ra {
                guard regionOf[nb] >= 0 else { continue }
                let key = packPair(ra, regionOf[nb])
                let d = simd_dot(faceNormals[t], faceNormals[nb])
                pairMinDot[key] = min(pairMinDot[key] ?? 1, d)
            }
        }
        // Soft-boundary pairs only, processed SMOOTHEST first (highest minDot) so each surface
        // consolidates before sharper cross-feature boundaries are evaluated against the whole.
        // DETERMINISM: sort by minDot desc, tie-broken on the packed pair key — equal-dihedral
        // pairs would otherwise land in dictionary/unstable-sort order, giving a DIFFERENT region
        // composition (and non-reproducible fits) on every run.
        let pairList = pairMinDot
            .filter { $0.value >= cosMergeAngle }
            .sorted { $0.value != $1.value ? $0.value > $1.value : $0.key < $1.key }
            .map { (Int($0.key >> 32), Int($0.key & 0xFFFF_FFFF)) }

        // The degrade guard: a merge is allowed only if the union still fits a single primitive
        // about as well as the better of the two parts. This is what keeps a chamfer (a cone)
        // from being absorbed into the shaft (which would lose the chamfer AND pull the cylinder
        // radius off) while still letting a coarse cylinder's coplanar facets fuse.
        let slack = 0.5 * mergeTol

        var parent = Array(0..<n)
        var members = regions
        var rootFit = regions.map { PrimitiveFitter.bestFit(mesh: mesh, triangleIndices: $0, faceNormals: faceNormals) }
        func find(_ x: Int) -> Int { var r = x; while parent[r] != r { parent[r] = parent[parent[r]]; r = parent[r] }; return r }

        var changed = true
        var passes = 0
        while changed, passes < 60 {
            changed = false; passes += 1
            for (ra, rb) in pairList {
                let a = find(ra), b = find(rb)
                if a == b { continue }
                // Sorted ascending — canonical order, independent of which side was "a" vs "b".
                let union = (members[a] + members[b]).sorted()
                let fit = PrimitiveFitter.bestFit(mesh: mesh, triangleIndices: union, faceNormals: faceNormals)
                let separateBest = min(rootFit[a].residualRMS, rootFit[b].residualRMS)
                guard fit.residualRMS <= mergeTol, fit.residualRMS <= separateBest + slack else { continue }
                parent[b] = a
                members[a] = union
                members[b] = []
                rootFit[a] = fit
                changed = true
            }
        }

        var pairs: [([Int], FittedPrimitive)] = []
        for r in 0..<n where find(r) == r && !members[r].isEmpty {
            pairs.append((members[r], rootFit[r]))
        }
        pairs.sort { MeshRegion.orderTriangleSets($0.0, $1.0) }
        return (pairs.map { $0.0 }, pairs.map { $0.1 })
    }
}
