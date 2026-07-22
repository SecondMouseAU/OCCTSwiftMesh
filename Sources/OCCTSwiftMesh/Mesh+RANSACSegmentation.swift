// Mesh+RANSACSegmentation.swift — Schnabel-style RANSAC primitive extraction, an alternative
// segmentation strategy to `segmented(_:)`'s dihedral region-growing, plus a measured
// auto-selection between the two.
//
// Schnabel, Wahl, Klein, "Efficient RANSAC for Point-Cloud Shape Detection" (2007): repeatedly
// fit a candidate primitive to a small sample, count its inliers across the WHOLE remaining point
// set (not just triangles contiguous with the sample — the key difference from dihedral region-
// growing, which only ever absorbs edge-adjacent neighbours), commit the best candidate found
// within a probability-driven trial budget, and recurse on the residual until nothing more can be
// found. Complements `segmented(_:)`: dihedral growing wins on a single integrated part (one
// continuous surface graph to walk); RANSAC wins on multi-primitive scenes where the same
// primitive appears in several disconnected patches, or growing shatters/under-segments.
//
// CANDIDATE GENERATION — a documented simplification vs. Schnabel's exact closed-form minimal-set
// solvers (1-2 oriented points per primitive type, solved in closed form). Instead, each candidate
// is a small (`RANSACSegmentOptions.sampleSize`, default 6) deterministic sample of triangles fed
// straight into the EXISTING, already-tested `PrimitiveFitter.bestFit` least-squares fit — slightly
// more expensive per candidate than a true minimal set, but far more numerically robust (no
// per-primitive-type closed-form derivation to get subtly wrong) and reuses fitting code this
// package already trusts. See docs/algorithms/ransac-segmentation.md.
//
// DETERMINISM — classic RANSAC draws random minimal sets; this needs repeat calls to be
// byte-identical instead. `Mesh.deterministicSample(trial:poolSize:sampleSize:)` (a splitmix64
// index hash keyed by trial number) gives each candidate trial an INDEPENDENT, well-mixed, fully
// reproducible sample with no system RNG, and every other choice (the per-round trial budget,
// cluster grouping) is similarly free of unordered-collection iteration.

import Foundation
import simd
import OCCTSwift

extension Mesh {

    /// Schnabel-style RANSAC primitive extraction — an alternative to `segmented(_:)`'s dihedral
    /// region-growing. See the file header for the algorithm sketch and its documented
    /// candidate-generation simplification.
    ///
    /// Welds internally (`RANSACSegmentOptions.weldTolerance`), matching `segmented(_:)`'s
    /// ergonomics — unwelded input does not need a separate `.welded()` call first.
    public func segmentedRANSAC(_ options: RANSACSegmentOptions = .init()) -> SegmentedMesh {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        guard tc > 0, !verts.isEmpty else { return SegmentedMesh(regions: [], fits: [], truncatedTriangleCount: 0) }

        var lo = verts[0], hi = verts[0]
        for p in verts { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let bodyDiag = Double(simd_length(hi - lo))

        // Weld for topology only (clustering) — fitting below uses the ORIGINAL vertices/indices,
        // the same split `segmented(_:)` makes.
        let (remap, weldedPositions) = Mesh.weldPositions(verts, tolerance: options.weldTolerance)
        var weldedIndices = [UInt32](repeating: 0, count: idx.count)
        for i in idx.indices { weldedIndices[i] = remap[Int(idx[i])] }
        let normals = Mesh.faceNormals(vertices: weldedPositions, indices: weldedIndices, triangleCount: tc)

        let inlierEps = options.inlierEpsilon > 0 ? options.inlierEpsilon : max(1e-9, 0.005 * bodyDiag)
        let meanEdge = Mesh.meanEdgeLength(vertices: weldedPositions, indices: weldedIndices, triangleCount: tc)
        let clusterEps = options.clusterEpsilon > 0 ? options.clusterEpsilon : max(1e-9, 2.0 * meanEdge)
        let cosNormalDeviation = Double(cos(options.maxNormalDeviationDegrees * .pi / 180))
        let sampleSize = max(1, options.sampleSize)

        var centroids = [SIMD3<Float>](repeating: .zero, count: tc)
        for t in 0..<tc {
            let base = t * 3
            let a = verts[Int(idx[base])], b = verts[Int(idx[base + 1])], c = verts[Int(idx[base + 2])]
            centroids[t] = (a + b + c) / 3
        }

        var remaining = Array(0..<tc)
        var regions: [MeshRegion] = []
        var fits: [FittedPrimitive] = []
        var claimed = [Bool](repeating: false, count: tc)

        let minPoolSize = max(options.minSupportCount, sampleSize)
        var roundsRun = 0
        let maxRounds = tc + 1   // a hard backstop; every successful round claims >= minSupportCount

        while remaining.count >= minPoolSize, roundsRun < maxRounds {
            roundsRun += 1

            var bestFitCandidate: FittedPrimitive? = nil
            var bestInliers: [Int] = []
            var trialsRun = 0
            var trialIndex = 0

            while true {
                let budget = Mesh.requiredRANSACTrials(
                    bestSupport: bestInliers.count, remaining: remaining.count, sampleSize: sampleSize,
                    successProbability: options.successProbability, cap: options.maxCandidatesPerRound)
                guard trialsRun < budget else { break }
                trialsRun += 1

                let positions = Mesh.deterministicSample(trial: trialIndex, poolSize: remaining.count,
                                                         sampleSize: min(sampleSize, remaining.count))
                trialIndex += 1
                guard !positions.isEmpty else { continue }
                let sampleTris = positions.map { remaining[$0] }

                let sampleArea = Mesh.area(ofTriangles: sampleTris, vertices: verts, indices: idx)
                let sampleRegion = MeshRegion(triangleIndices: sampleTris, area: sampleArea)
                // Score EVERY fit kind this sample supports, not just `bestFit`'s own
                // simplicity-biased winner — see `PrimitiveFitter.allFits`'s doc comment: a small
                // local sample of a smooth curved surface fits "plane" almost as well as the true
                // surface, so relying on bestFit's tie-break alone would starve sphere/cylinder/
                // cone candidates entirely. The GLOBAL inlier count below is the true arbiter.
                let sampleCandidates = PrimitiveFitter.allFits(vertices: verts, indices: idx,
                                                               region: sampleRegion, faceNormals: normals)
                for candidate in sampleCandidates {
                    guard candidate.residualRMS.isFinite else { continue }
                    var inliers: [Int] = []
                    for t in remaining {
                        guard Mesh.distanceToPrimitive(centroids[t], candidate) <= inlierEps else { continue }
                        let expected = Mesh.primitiveNormal(at: centroids[t], candidate)
                        // ABSOLUTE deviation — a locally-flipped triangle (inconsistent/unknown
                        // global winding, common on real scan meshes, is literally why #30's
                        // orientation diagnostics exist) still lies tangent to the fitted
                        // surface; only the TANGENT PLANE match matters here, not which way the
                        // normal happens to point.
                        guard abs(Double(simd_dot(normals[t], expected))) >= cosNormalDeviation else { continue }
                        inliers.append(t)
                    }
                    if inliers.count > bestInliers.count {
                        bestInliers = inliers
                        bestFitCandidate = candidate
                    }
                }
            }

            guard bestInliers.count >= options.minSupportCount, let winner = bestFitCandidate else { break }

            // Refine: a small sample's own fit is noisy, so its raw global-inlier count can miss
            // triangles that genuinely belong to the same surface — refit using ALL of that
            // sample's own inliers (a much larger, cleaner point set) and re-score once more.
            // Without this, one true surface routinely fragments across several rounds instead
            // of being captured in one, each round only ever as accurate as its tiny sample.
            //
            // Refit with the SAME primitive kind directly (not `bestFit`, which could switch
            // kind — e.g. a still-somewhat-localized inlier patch of a large sphere can look
            // planar enough for `bestFit`'s simplicity bias to prefer "plane" again, exactly the
            // problem this refinement step exists to correct for in the first place).
            let region = MeshRegion(triangleIndices: bestInliers,
                                    area: Mesh.area(ofTriangles: bestInliers, vertices: verts, indices: idx))
            let refined: FittedPrimitive?
            switch winner.kind {
            case .plane: refined = PrimitiveFitter.fitPlane(points: PrimitiveFitter.regionPoints(
                vertices: verts, indices: idx, region: region))
            case .sphere: refined = PrimitiveFitter.fitSphere(points: PrimitiveFitter.regionPoints(
                vertices: verts, indices: idx, region: region))
            case .cylinder: refined = PrimitiveFitter.fitCylinder(
                points: PrimitiveFitter.regionPoints(vertices: verts, indices: idx, region: region),
                normals: bestInliers.map { SIMD3<Double>(normals[$0]) })
            case .cone: refined = PrimitiveFitter.fitCone(
                points: PrimitiveFitter.regionPoints(vertices: verts, indices: idx, region: region),
                normals: bestInliers.map { SIMD3<Double>(normals[$0]) })
            }
            if let refined, refined.residualRMS.isFinite {
                var refinedInliers: [Int] = []
                for t in remaining {
                    guard Mesh.distanceToPrimitive(centroids[t], refined) <= inlierEps else { continue }
                    let expected = Mesh.primitiveNormal(at: centroids[t], refined)
                    guard abs(Double(simd_dot(normals[t], expected))) >= cosNormalDeviation else { continue }
                    refinedInliers.append(t)
                }
                if refinedInliers.count > bestInliers.count { bestInliers = refinedInliers }
            }

            // Cluster the global inliers by spatial proximity (clusterEpsilon) — a candidate
            // that coincidentally satisfies the tolerance/normal gates far from the rest doesn't
            // get glued into the same region as the main cluster.
            let clusterPoints = bestInliers.map { centroids[$0] }
            let clusterGroups = Mesh.spatialClusters(points: clusterPoints, epsilon: clusterEps)
            var claimedAnyThisRound = false
            for group in clusterGroups where group.count >= options.minSupportCount {
                let tris = group.map { bestInliers[$0] }.sorted()
                let area = Mesh.area(ofTriangles: tris, vertices: verts, indices: idx)
                let refit = PrimitiveFitter.bestFit(vertices: verts, indices: idx,
                                                    region: MeshRegion(triangleIndices: tris, area: area),
                                                    faceNormals: normals)
                regions.append(MeshRegion(triangleIndices: tris, area: area))
                fits.append(refit)
                for t in tris { claimed[t] = true }
                claimedAnyThisRound = true
            }
            // Coverage stalled: nothing this round actually met minSupportCount after clustering,
            // so retrying would just rediscover the identical candidate forever.
            guard claimedAnyThisRound else { break }
            remaining.removeAll { claimed[$0] }
        }

        var sortedPairs = Array(zip(regions, fits))
        sortedPairs.sort { MeshRegion.order($0.0, $1.0) }
        var finalRegions = sortedPairs.map { $0.0 }
        var finalFits = sortedPairs.map { $0.1 }
        var truncated = remaining.count   // never claimed by any primitive — reported, not dropped

        if let rawCap = options.maxRegions {
            let cap = max(rawCap, 0)
            if finalRegions.count > cap {
                for region in finalRegions[cap...] { truncated += region.triangleIndices.count }
                finalRegions = Array(finalRegions[..<cap])
                finalFits = Array(finalFits[..<cap])
            }
        }

        return SegmentedMesh(regions: finalRegions, fits: finalFits, truncatedTriangleCount: truncated)
    }

    /// Run both `segmented(_:)` and `segmentedRANSAC(_:)` on this mesh and keep whichever scores
    /// higher on a "substantial-clean coverage" metric (the OCCTReconstruct bake-off convention —
    /// see `segmentation.md`): the fraction of total mesh area covered by regions that are both
    /// SUBSTANTIAL (at least `minAreaFraction` of the total) and CLEAN (`inlierRatio >=
    /// minInlierRatio`). Ties (including both scoring zero) favor `dihedral`, the cheaper strategy.
    public func segmentedAutoSelect(dihedral: SegmentOptions = .init(), ransac: RANSACSegmentOptions = .init(),
                                    minInlierRatio: Double = 0.8, minAreaFraction: Double = 0.01)
        -> SegmentationStrategyResult {
        let totalArea = Mesh.area(ofTriangles: Array(0..<triangleCount), vertices: vertices, indices: indices)
        let dihedralResult = segmented(dihedral)
        let ransacResult = segmentedRANSAC(ransac)
        let dihedralScore = Mesh.substantialCleanCoverage(dihedralResult, totalArea: totalArea,
                                                          minInlierRatio: minInlierRatio, minAreaFraction: minAreaFraction)
        let ransacScore = Mesh.substantialCleanCoverage(ransacResult, totalArea: totalArea,
                                                        minInlierRatio: minInlierRatio, minAreaFraction: minAreaFraction)
        if ransacScore > dihedralScore {
            return SegmentationStrategyResult(result: ransacResult, strategy: .ransac,
                                              dihedralScore: dihedralScore, ransacScore: ransacScore)
        }
        return SegmentationStrategyResult(result: dihedralResult, strategy: .dihedral,
                                          dihedralScore: dihedralScore, ransacScore: ransacScore)
    }

    /// Fraction of `totalArea` covered by regions that are both substantial (>= `minAreaFraction`
    /// of `totalArea`) and clean (`inlierRatio >= minInlierRatio`) — the scoring metric behind
    /// `segmentedAutoSelect`.
    static func substantialCleanCoverage(_ result: SegmentedMesh, totalArea: Double,
                                         minInlierRatio: Double, minAreaFraction: Double) -> Double {
        guard totalArea > 1e-15 else { return 0 }
        var covered = 0.0
        for (region, fit) in zip(result.regions, result.fits) {
            guard fit.inlierRatio >= minInlierRatio, region.area / totalArea >= minAreaFraction else { continue }
            covered += region.area
        }
        return covered / totalArea
    }

    // MARK: - RANSAC helpers

    /// A splitmix64 index hash — the deterministic (no system RNG) substitute for random draws
    /// used throughout candidate generation below.
    static func splitmix64(_ x0: UInt64) -> UInt64 {
        var z = x0 &+ 0x9E37_79B9_7F4A_7C15
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }

    /// `sampleSize` DISTINCT indices into `0..<poolSize`, deterministically hashed from `trial` —
    /// each trial number gives an INDEPENDENT well-mixed sample (unlike a sliding window over one
    /// fixed shuffle, which only ever offers `poolSize` distinct windows regardless of how many
    /// trials are requested): with `maxCandidatesPerRound` independent draws, a small pool with
    /// several distinct primitives (e.g. a box's 6 faces) still has good odds of eventually
    /// sampling triangles that all belong to the same one. No system RNG — same `trial` always
    /// hashes to the same sample, so repeat calls agree exactly.
    static func deterministicSample(trial: Int, poolSize: Int, sampleSize: Int) -> [Int] {
        guard poolSize > 0, sampleSize > 0 else { return [] }
        let n = min(sampleSize, poolSize)
        var result: [Int] = []
        result.reserveCapacity(n)
        var seen = Set<Int>()
        var salt: UInt64 = 0
        let maxAttempts = UInt64(poolSize) * 4 + 64   // generous — avoids an unbounded loop if n == poolSize
        while result.count < n, salt < maxAttempts {
            let h = Mesh.splitmix64(UInt64(bitPattern: Int64(trial)) &* 0x9E37_79B9_7F4A_7C15 &+ salt)
            let idx = Int(h % UInt64(poolSize))
            if seen.insert(idx).inserted { result.append(idx) }
            salt += 1
        }
        return result
    }

    /// Schnabel's adaptive per-round trial budget: as a round's best-found inlier support grows,
    /// fewer further candidate trials are needed to be `successProbability`-confident nothing
    /// larger remains undiscovered. Falls back to `cap` whenever the estimate isn't well-defined
    /// (no candidate found yet, or a degenerate probability) — always capped by `cap` regardless.
    static func requiredRANSACTrials(bestSupport: Int, remaining: Int, sampleSize: Int,
                                     successProbability: Double, cap: Int) -> Int {
        guard cap > 0 else { return 0 }
        guard bestSupport > 0, remaining > 0, sampleSize > 0 else { return cap }
        let ratio = Double(bestSupport) / Double(remaining)
        guard ratio > 0, ratio < 1 else { return cap }
        let p = pow(ratio, Double(sampleSize))
        guard p > 0, p < 1 else { return cap }
        let denom = log(1 - p)
        guard denom < 0 else { return cap }
        let needed = log(1 - successProbability) / denom
        guard needed.isFinite, needed > 0 else { return cap }
        return min(cap, Int(needed.rounded(.up)))
    }

    /// Mean triangle edge length — the proxy `RANSACSegmentOptions.clusterEpsilon` auto-derives
    /// from when unset, distinct from `inlierEpsilon`'s bbox-diagonal-relative fit tolerance:
    /// connectivity should scale with the mesh's own tessellation density, not its overall size.
    static func meanEdgeLength(vertices: [SIMD3<Float>], indices: [UInt32], triangleCount tc: Int) -> Double {
        guard tc > 0 else { return 0 }
        var sum = 0.0
        for t in 0..<tc {
            let base = t * 3
            let a = vertices[Int(indices[base])], b = vertices[Int(indices[base + 1])], c = vertices[Int(indices[base + 2])]
            sum += Double(simd_distance(a, b)) + Double(simd_distance(b, c)) + Double(simd_distance(c, a))
        }
        return sum / Double(tc * 3)
    }

    /// Point-to-primitive-surface distance, for any `FittedPrimitive.Kind` — the global-inlier
    /// distance gate `segmentedRANSAC` scores candidates against.
    static func distanceToPrimitive(_ point: SIMD3<Float>, _ fit: FittedPrimitive) -> Double {
        let p = SIMD3<Double>(point)
        switch fit.kind {
        case .plane:
            guard fit.params.count >= 4 else { return .infinity }
            let n = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
            return abs(simd_dot(n, p) - fit.params[3])
        case .sphere:
            guard fit.params.count >= 4 else { return .infinity }
            let c = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
            return abs(simd_length(p - c) - fit.params[3])
        case .cylinder:
            guard fit.params.count >= 7 else { return .infinity }
            let q = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
            let axis = SIMD3<Double>(fit.params[3], fit.params[4], fit.params[5])
            let rel = p - q
            let radial = rel - simd_dot(rel, axis) * axis
            return abs(simd_length(radial) - fit.params[6])
        case .cone:
            guard fit.params.count >= 7 else { return .infinity }
            let apex = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
            let axis = SIMD3<Double>(fit.params[3], fit.params[4], fit.params[5])
            let halfAngle = fit.params[6]
            let rel = p - apex
            let axial = simd_dot(rel, axis)
            let radial = simd_length(rel - axial * axis)
            return abs(axial * sin(halfAngle) - radial * cos(halfAngle))
        }
    }

    /// Unit outward surface normal of the fitted primitive nearest `point` — the candidate's
    /// expected normal `segmentedRANSAC` compares each triangle's own face normal against.
    static func primitiveNormal(at point: SIMD3<Float>, _ fit: FittedPrimitive) -> SIMD3<Float> {
        let p = SIMD3<Double>(point)
        let fallback = SIMD3<Double>(0, 0, 1)
        let n: SIMD3<Double>
        switch fit.kind {
        case .plane:
            guard fit.params.count >= 3 else { return SIMD3<Float>(fallback) }
            n = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
        case .sphere:
            guard fit.params.count >= 3 else { return SIMD3<Float>(fallback) }
            let c = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
            let d = p - c
            n = simd_length(d) > 1e-12 ? simd_normalize(d) : fallback
        case .cylinder:
            guard fit.params.count >= 6 else { return SIMD3<Float>(fallback) }
            let q = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
            let axis = SIMD3<Double>(fit.params[3], fit.params[4], fit.params[5])
            let rel = p - q
            let radial = rel - simd_dot(rel, axis) * axis
            n = simd_length(radial) > 1e-12 ? simd_normalize(radial) : fallback
        case .cone:
            guard fit.params.count >= 7 else { return SIMD3<Float>(fallback) }
            let apex = SIMD3<Double>(fit.params[0], fit.params[1], fit.params[2])
            let axis = SIMD3<Double>(fit.params[3], fit.params[4], fit.params[5])
            let halfAngle = fit.params[6]
            let rel = p - apex
            let axial = simd_dot(rel, axis)
            let radialVec = rel - axial * axis
            let radialDir = simd_length(radialVec) > 1e-12 ? simd_normalize(radialVec) : fallback
            // Outward cone normal: perpendicular to the slant direction (cosθ·axis + sinθ·radialDir),
            // pointing away from the axis — verified against the apex-at-origin worked example in
            // docs/algorithms/ransac-segmentation.md.
            n = simd_normalize(radialDir * cos(halfAngle) - axis * sin(halfAngle))
        }
        return SIMD3<Float>(n)
    }

    /// Union-find over a spatial grid (the same grid-hash style as `weldPositions`): groups
    /// `points` into connected clusters where two points are linked whenever they're within
    /// `epsilon` of each other (transitively — clusters can span many points, not just direct
    /// pairs). Order-independent: the final partition depends only on which pairs are within
    /// `epsilon`, never on `Dictionary` iteration order, so this is deterministic despite the
    /// grid being built as a `Dictionary`.
    static func spatialClusters(points: [SIMD3<Float>], epsilon: Double) -> [[Int]] {
        guard !points.isEmpty else { return [] }
        guard epsilon > 0 else { return points.indices.map { [$0] } }

        struct GridKey: Hashable { var x: Int64; var y: Int64; var z: Int64 }
        func cell(_ p: SIMD3<Float>) -> GridKey {
            GridKey(x: Int64((Double(p.x) / epsilon).rounded(.down)),
                    y: Int64((Double(p.y) / epsilon).rounded(.down)),
                    z: Int64((Double(p.z) / epsilon).rounded(.down)))
        }
        var buckets: [GridKey: [Int]] = [:]
        for (i, p) in points.enumerated() { buckets[cell(p), default: []].append(i) }

        var parent = Array(points.indices)
        func find(_ x: Int) -> Int { var r = x; while parent[r] != r { parent[r] = parent[parent[r]]; r = parent[r] }; return r }
        func union(_ a: Int, _ b: Int) { let ra = find(a), rb = find(b); if ra != rb { parent[ra] = rb } }

        let offsets = [-1, 0, 1]
        let epsF = Float(epsilon)
        for (key, idxs) in buckets {
            for dx in offsets { for dy in offsets { for dz in offsets {
                let nKey = GridKey(x: key.x + Int64(dx), y: key.y + Int64(dy), z: key.z + Int64(dz))
                guard let nIdxs = buckets[nKey] else { continue }
                for a in idxs {
                    for b in nIdxs where a < b && simd_distance(points[a], points[b]) <= epsF { union(a, b) }
                }
            }}}
        }
        var groups: [Int: [Int]] = [:]
        for i in points.indices { groups[find(i), default: []].append(i) }
        return groups.values.map { $0.sorted() }.sorted { ($0.first ?? 0) < ($1.first ?? 0) }
    }
}
