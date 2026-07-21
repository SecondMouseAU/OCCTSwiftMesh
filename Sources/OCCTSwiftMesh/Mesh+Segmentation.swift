// Mesh+Segmentation.swift — dihedral region-growing + primitive-fit merge.
//
// Region-growing alone shatters a coarsely-tessellated curved surface: a low-poly cylinder's
// facets sit >maxDihedralDegrees apart, so each becomes its own planar region ("confetti").
// The merge pass is the mandatory companion — it greedily unions adjacent regions whose union
// still fits ONE primitive within tolerance, collapsing those facets back into one cylinder,
// while leaving real edges (where the union fits nothing well) intact.
//
// Ported from OCCTReconstruct's ReconstructCompute (RegionSegmentation.swift +
// RegionMerging.swift), adapted to operate on OCCTSwift.Mesh's own arrays instead of a
// separate IndexedMesh type, and to weld internally (see below) rather than assume a
// pre-welded input.

import Foundation
import simd
import OCCTSwift

extension Mesh {

    /// Segment into smoothly-connected surface regions (dihedral region-growing), then merge
    /// adjacent regions whose union still fits a single primitive — undoing coarse-tessellation
    /// "confetti."
    ///
    /// Welds internally (`options.weldTolerance`) so unwelded input (raw OCCT tessellation,
    /// STL import) does not silently degrade to one region per triangle: adjacency always
    /// reflects the welded topology, but every `MeshRegion.triangleIndices` refers to THIS
    /// mesh's own (possibly unwelded) triangle order — welding only affects which triangles
    /// are considered neighbours, never the triangle indexing callers see.
    public func segmented(_ options: SegmentOptions = .init()) -> SegmentedMesh {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        guard tc > 0, !verts.isEmpty else { return SegmentedMesh(regions: [], fits: [], truncatedTriangleCount: 0) }

        var lo = verts[0], hi = verts[0]
        for p in verts { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let bodyDiag = Double(simd_length(hi - lo))

        // Weld for topology only — geometric fitting below uses the ORIGINAL vertices/indices.
        let (remap, weldedPositions) = Mesh.weldPositions(verts, tolerance: options.weldTolerance)
        var weldedIndices = [UInt32](repeating: 0, count: idx.count)
        for i in idx.indices { weldedIndices[i] = remap[Int(idx[i])] }

        let normals = Mesh.faceNormals(vertices: weldedPositions, indices: weldedIndices, triangleCount: tc)
        let adjacency = Mesh.triangleAdjacency(indices: weldedIndices, triangleCount: tc)

        let seeds = Mesh.segmentSmoothRegions(triangleCount: tc, normals: normals, adjacency: adjacency,
                                              maxDihedralDegrees: options.maxDihedralDegrees)
            .map { MeshRegion(triangleIndices: $0, area: Mesh.area(ofTriangles: $0, vertices: verts, indices: idx)) }
            .sorted(by: MeshRegion.order)

        let (mergedRegions, fits) = RegionMerging.merge(
            vertices: verts, indices: idx, regions: seeds, faceNormals: normals, adjacency: adjacency,
            bodyDiag: bodyDiag, relativeTolerance: options.mergeRelativeTolerance,
            maxMergeAngleDegrees: options.maxMergeAngleDegrees)

        var filteredRegions: [MeshRegion] = []
        var filteredFits: [FittedPrimitive] = []
        var dropped = 0
        for (region, fit) in zip(mergedRegions, fits) {
            if region.triangleIndices.count < options.minRegionTriangles {
                dropped += region.triangleIndices.count
            } else {
                filteredRegions.append(region)
                filteredFits.append(fit)
            }
        }
        if let rawCap = options.maxRegions {
            let cap = max(rawCap, 0)   // a non-positive cap safely means "keep none", never a crash
            if filteredRegions.count > cap {
                for region in filteredRegions[cap...] { dropped += region.triangleIndices.count }
                filteredRegions = Array(filteredRegions[..<cap])
                filteredFits = Array(filteredFits[..<cap])
            }
        }

        return SegmentedMesh(regions: filteredRegions, fits: filteredFits, truncatedTriangleCount: dropped)
    }

    /// Deterministic DFS flood over edge-adjacent triangles, absorbing an unassigned neighbour
    /// iff its face normal is within `maxDihedralDegrees` of the seed triangle's. Region
    /// membership is a pure reachability computation over a fixed (order-independent) adjacency
    /// SET, so it doesn't depend on `adjacency[t]`'s internal element order — only on seed order
    /// (`0..<triangleCount`, deterministic).
    static func segmentSmoothRegions(triangleCount tc: Int, normals: [SIMD3<Float>], adjacency: [[Int]],
                                     maxDihedralDegrees: Float) -> [[Int]] {
        guard tc > 0 else { return [] }
        let cosThreshold = cos(maxDihedralDegrees * .pi / 180)

        var region = [Int](repeating: -1, count: tc)
        var regions: [[Int]] = []
        for seed in 0..<tc where region[seed] == -1 {
            let id = regions.count
            var members: [Int] = []
            var stack = [seed]
            region[seed] = id
            while let t = stack.popLast() {
                members.append(t)
                let nt = normals[t]
                for n in adjacency[t] where region[n] == -1 {
                    if simd_dot(nt, normals[n]) >= cosThreshold {
                        region[n] = id
                        stack.append(n)
                    }
                }
            }
            regions.append(members)
        }
        return regions
    }
}

/// Agglomerative region merging that repairs coarse-tessellation shattering.
enum RegionMerging {

    /// Merge regions by primitive fit. Returns the merged regions (largest-first) and each
    /// merged region's best fit.
    static func merge(vertices: [SIMD3<Float>], indices: [UInt32], regions inputRegions: [MeshRegion],
                      faceNormals: [SIMD3<Float>], adjacency: [[Int]], bodyDiag: Double,
                      relativeTolerance: Double, maxMergeAngleDegrees: Float,
                      maxRegionsToMerge: Int = 1500) -> (regions: [MeshRegion], fits: [FittedPrimitive]) {
        var regions = inputRegions
        // Cheap coplanar pre-merge keeps the fit-gated pass below usable at 400k+ triangle
        // scans, where raw region counts can run into the thousands: a tight (~2°) coplanar
        // threshold only ever collapses genuinely-flat confetti, never a real curved seam.
        if regions.count > maxRegionsToMerge {
            regions = coplanarPreMerge(vertices: vertices, indices: indices, regions: regions,
                                       faceNormals: faceNormals, adjacency: adjacency)
        }

        let n = regions.count
        guard n > 1, n <= maxRegionsToMerge else {
            let fits = regions.map {
                PrimitiveFitter.bestFit(vertices: vertices, indices: indices, region: $0,
                                       faceNormals: faceNormals, bodyDiag: bodyDiag)
            }
            return (regions, fits)
        }

        let mergeTol = max(relativeTolerance * bodyDiag, 1e-6)

        var regionOf = [Int](repeating: -1, count: indices.count / 3)
        for (rid, region) in regions.enumerated() { for t in region.triangleIndices { regionOf[t] = rid } }

        // Region adjacency, tracking the SOFTEST→HARDEST boundary: per pair keep the sharpest
        // dihedral (min dot) across shared facet edges. A pair is mergeable only if even its
        // sharpest shared edge is "soft" (below maxMergeAngle).
        let cosMergeAngle = cos(maxMergeAngleDegrees * .pi / 180)
        func packPair(_ a: Int, _ b: Int) -> UInt64 { (UInt64(min(a, b)) << 32) | UInt64(max(a, b)) }
        var pairMinDot = [UInt64: Float]()
        for t in 0..<(indices.count / 3) {
            let ra = regionOf[t]
            for nb in adjacency[t] where regionOf[nb] != ra {
                let key = packPair(ra, regionOf[nb])
                let d = simd_dot(faceNormals[t], faceNormals[nb])
                pairMinDot[key] = min(pairMinDot[key] ?? 1, d)
            }
        }
        // Soft-boundary pairs only, processed SMOOTHEST first (highest minDot) so each surface
        // consolidates before sharper cross-feature boundaries are evaluated. DETERMINISM:
        // sort by minDot desc, tie-break on the packed pair key — otherwise equal-dihedral
        // pairs would order arbitrarily (dictionary iteration), making the union-find process
        // them in a different order each run → different region compositions.
        let pairList = pairMinDot
            .filter { $0.value >= cosMergeAngle }
            .sorted { $0.value != $1.value ? $0.value > $1.value : $0.key < $1.key }
            .map { (Int($0.key >> 32), Int($0.key & 0xFFFF_FFFF)) }

        // The degrade guard: a merge is allowed only if the union still fits a single primitive
        // about as well as the better of the two parts.
        let slack = 0.5 * mergeTol

        var parent = Array(0..<n)
        var members = regions.map { $0.triangleIndices }
        var rootFit = regions.map {
            PrimitiveFitter.bestFit(vertices: vertices, indices: indices, region: $0,
                                   faceNormals: faceNormals, bodyDiag: bodyDiag)
        }
        func find(_ x: Int) -> Int { var r = x; while parent[r] != r { parent[r] = parent[parent[r]]; r = parent[r] }; return r }

        var changed = true
        var passes = 0
        while changed, passes < 60 {
            changed = false; passes += 1
            for (ra, rb) in pairList {
                let a = find(ra), b = find(rb)
                if a == b { continue }
                let union = members[a] + members[b]
                let unionArea = Mesh.area(ofTriangles: union, vertices: vertices, indices: indices)
                let fit = PrimitiveFitter.bestFit(vertices: vertices, indices: indices,
                                                 region: MeshRegion(triangleIndices: union, area: unionArea),
                                                 faceNormals: faceNormals, bodyDiag: bodyDiag)
                let separateBest = min(rootFit[a].residualRMS, rootFit[b].residualRMS)
                guard fit.residualRMS <= mergeTol, fit.residualRMS <= separateBest + slack else { continue }
                parent[b] = a
                members[a] = union
                members[b] = []
                rootFit[a] = fit
                changed = true
            }
        }

        var pairs: [(MeshRegion, FittedPrimitive)] = []
        for r in 0..<n where find(r) == r && !members[r].isEmpty {
            let area = Mesh.area(ofTriangles: members[r], vertices: vertices, indices: indices)
            pairs.append((MeshRegion(triangleIndices: members[r], area: area), rootFit[r]))
        }
        pairs.sort { MeshRegion.order($0.0, $1.0) }
        return (pairs.map { $0.0 }, pairs.map { $0.1 })
    }

    /// Cheap, fit-free pre-pass: union-find adjacent regions whose shared boundary is nearly
    /// coplanar (within ~2°), unconditionally. A single pass suffices — union-find correctly
    /// merges transitive chains regardless of the order pairs are processed in, since every
    /// pair above threshold merges unconditionally (unlike the fit-gated pass, whose merges are
    /// conditional on a per-step primitive-fit test and so need iterative re-evaluation).
    static func coplanarPreMerge(vertices: [SIMD3<Float>], indices: [UInt32], regions: [MeshRegion],
                                 faceNormals: [SIMD3<Float>], adjacency: [[Int]]) -> [MeshRegion] {
        let n = regions.count
        var regionOf = [Int](repeating: -1, count: indices.count / 3)
        for (rid, r) in regions.enumerated() { for t in r.triangleIndices { regionOf[t] = rid } }

        let cosCoplanar = cos(2.0 * .pi / 180)
        func packPair(_ a: Int, _ b: Int) -> UInt64 { (UInt64(min(a, b)) << 32) | UInt64(max(a, b)) }
        var pairMinDot = [UInt64: Float]()
        for t in 0..<(indices.count / 3) {
            let ra = regionOf[t]
            for nb in adjacency[t] where regionOf[nb] != ra {
                let key = packPair(ra, regionOf[nb])
                let d = simd_dot(faceNormals[t], faceNormals[nb])
                pairMinDot[key] = min(pairMinDot[key] ?? 1, d)
            }
        }
        let pairList = pairMinDot
            .filter { Double($0.value) >= cosCoplanar }
            .sorted { $0.value != $1.value ? $0.value > $1.value : $0.key < $1.key }
            .map { (Int($0.key >> 32), Int($0.key & 0xFFFF_FFFF)) }

        var parent = Array(0..<n)
        func find(_ x: Int) -> Int { var r = x; while parent[r] != r { parent[r] = parent[parent[r]]; r = parent[r] }; return r }
        for (a, b) in pairList {
            let ra = find(a), rb = find(b)
            if ra != rb { parent[rb] = ra }
        }

        var buckets: [Int: [Int]] = [:]
        for (i, r) in regions.enumerated() { buckets[find(i), default: []].append(contentsOf: r.triangleIndices) }
        return buckets.values
            .map { tris in MeshRegion(triangleIndices: tris.sorted(), area: Mesh.area(ofTriangles: tris, vertices: vertices, indices: indices)) }
            .sorted(by: MeshRegion.order)
    }
}
