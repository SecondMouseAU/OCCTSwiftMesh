// Mesh+Align.swift — point-to-plane ICP registration with PCA pre-align and normal-space
// sampling.
//
// Ported from the classic ICP literature: Chen & Medioni's point-to-plane objective (converges
// far faster than point-to-point on engineering surfaces), Rusinkiewicz & Levoy's normal-space
// sampling ("Efficient Variants of the ICP Algorithm", 2001), and Low's linearized point-to-plane
// solve ("Linear Least-Squares Optimization for Point-to-Plane ICP", 2004). Pure Swift + simd —
// no OCCT kernel calls, no vendored library.
//
// Welds both meshes internally (for per-point normals and a deduplicated point cloud) — like
// `segmented(_:)`, not like `triangleAdjacency()` — since alignment is a composite, mesh-level
// operation, not a low-level connectivity primitive the caller is expected to pre-weld for.

import Foundation
import simd
import OCCTSwift

extension Mesh {

    /// Rigid-transform registration of this (SOURCE) mesh onto `reference`, via point-to-plane
    /// ICP. Returns `nil` if either mesh has too few points to register (fewer than 3 after
    /// welding).
    ///
    /// `result.transform` maps THIS mesh's original vertex positions into `reference`'s frame.
    public func aligned(to reference: Mesh, options: AlignOptions = .init()) -> AlignResult? {
        let sourceWelded = welded()
        let refWelded = reference.welded()
        let sourcePoints = sourceWelded.vertices.map { SIMD3<Double>($0) }
        let refPoints = refWelded.vertices.map { SIMD3<Double>($0) }
        guard sourcePoints.count >= 3, refPoints.count >= 3 else { return nil }

        let sourceNormals = sourceWelded.vertexNormals().map { SIMD3<Double>($0) }
        let refNormals = refWelded.vertexNormals().map { SIMD3<Double>($0) }

        var lo = refPoints[0], hi = refPoints[0]
        for p in refPoints { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let bboxDiag = simd_length(hi - lo)
        let distanceCap = options.correspondenceDistanceCap ?? max(1e-9, 0.15 * bboxDiag)

        let tree = KDTree3(points: refPoints)

        let sampleCount = max(0, min(options.maxSamples, sourcePoints.count))
        let sampleIndices = options.normalSpaceSampling
            ? Mesh.normalSpaceSample(normals: sourceNormals, count: sampleCount)
            : Mesh.uniformSample(total: sourcePoints.count, take: sampleCount)
        let samplePoints = sampleIndices.map { sourcePoints[$0] }

        var pose = options.preAlign
            ? Mesh.pcaPrealign(source: sourcePoints, reference: refPoints, tree: tree)
            : matrix_identity_double4x4

        func correspondences(at pose: simd_double4x4) -> [(src: SIMD3<Double>, ref: SIMD3<Double>, normal: SIMD3<Double>)] {
            samplePoints.compactMap { p in
                let tp = Mesh.apply(pose, p)
                guard let (idx, dist) = tree.nearest(to: tp), dist <= distanceCap else { return nil }
                return (tp, refPoints[idx], refNormals[idx])
            }
        }

        let convergenceEps = max(1e-12, 1e-7 * bboxDiag)
        var lastRMS = Double.infinity
        var iterations = 0
        var converged = false

        let maxIterations = max(0, options.maxIterations)
        for iter in 0..<maxIterations {
            let corr = correspondences(at: pose)
            guard corr.count >= 6 else { break }

            let residuals = corr.map { abs(simd_dot($0.normal, $0.src - $0.ref)) }
            let rms = (residuals.reduce(0) { $0 + $1 * $1 } / Double(residuals.count)).squareRoot()

            if iter > 0, abs(lastRMS - rms) < convergenceEps {
                converged = true
                lastRMS = rms
                break
            }
            lastRMS = rms
            iterations = iter + 1

            // Trimmed ICP: keep the best (1 - trimFraction) of correspondences by residual
            // magnitude. DETERMINISM: tie-break by original correspondence order.
            let keepCount = max(6, Int((Double(corr.count) * (1 - max(0, min(0.9, options.trimFraction)))).rounded()))
            let order = residuals.indices.sorted { residuals[$0] != residuals[$1] ? residuals[$0] < residuals[$1] : $0 < $1 }
            let kept = order.prefix(min(keepCount, order.count)).map { corr[$0] }
            guard kept.count >= 6 else { break }

            // Low (2004) linearized point-to-plane solve for a small incremental rigid transform
            // (r = small-angle rotation vector, t = translation): for each correspondence,
            // n·((p + r×p + t) - q) = 0  =>  (p×n)·r + n·t = n·(q - p).
            var ata = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
            var atb = [Double](repeating: 0, count: 6)
            for c in kept {
                let pxn = simd_cross(c.src, c.normal)
                let row = [pxn.x, pxn.y, pxn.z, c.normal.x, c.normal.y, c.normal.z]
                let rhs = simd_dot(c.normal, c.ref - c.src)
                for i in 0..<6 {
                    atb[i] += row[i] * rhs
                    for j in 0..<6 { ata[i][j] += row[i] * row[j] }
                }
            }
            guard let sol = Linalg.solve(ata, atb) else { break }
            guard sol.allSatisfy({ $0.isFinite }) else { break }

            let r = SIMD3<Double>(sol[0], sol[1], sol[2])
            let t = SIMD3<Double>(sol[3], sol[4], sol[5])
            let angle = simd_length(r)
            let rotation = angle > 1e-14 ? Mesh.rodrigues(axis: r / angle, angle: angle) : matrix_identity_double3x3
            let incremental = Mesh.rigidTransform(rotation: rotation, translation: t)
            pose = simd_mul(incremental, pose)
        }

        // Report the residual at the FINAL pose (post-last-increment), not the pose one step
        // prior — a fresh correspondence pass, cheap relative to the iterations already run.
        let finalCorr = correspondences(at: pose)
        let finalRMS = finalCorr.isEmpty
            ? lastRMS
            : (finalCorr.map { let d = simd_dot($0.normal, $0.src - $0.ref); return d * d }.reduce(0, +) / Double(finalCorr.count)).squareRoot()

        return AlignResult(transform: pose, residualRMS: finalRMS, iterations: iterations, converged: converged)
    }

    // MARK: - PCA pre-align

    /// Centroid + principal-axis pre-alignment, trying all 4 orientation-preserving sign
    /// combinations of the two dominant axes (PCA eigenvectors have no inherent sign — the
    /// third axis is always re-derived via cross product to keep an orthonormal, right-handed
    /// frame) and keeping whichever gives the lowest quick correspondence residual. Deterministic:
    /// candidates are tried in a fixed order and ties broken by that same order.
    static func pcaPrealign(source: [SIMD3<Double>], reference: [SIMD3<Double>], tree: KDTree3) -> simd_double4x4 {
        let (srcCov, srcCentroid) = Linalg.covariance(source)
        let (refCov, refCentroid) = Linalg.covariance(reference)
        let (_, srcVecs) = Linalg.eigenSymmetric3(srcCov)
        let (_, refVecs) = Linalg.eigenSymmetric3(refCov)

        // eigenSymmetric3 returns ascending eigenvalues; index 2 is the dominant axis.
        let srcA0 = SIMD3<Double>(srcVecs[2][0], srcVecs[2][1], srcVecs[2][2])
        let srcA1 = SIMD3<Double>(srcVecs[1][0], srcVecs[1][1], srcVecs[1][2])
        let refA0 = SIMD3<Double>(refVecs[2][0], refVecs[2][1], refVecs[2][2])
        let refA1 = SIMD3<Double>(refVecs[1][0], refVecs[1][1], refVecs[1][2])
        let refBasis = simd_double3x3(columns: (refA0, refA1, simd_cross(refA0, refA1)))

        let signCombos: [(Double, Double)] = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        let sampleStride = max(1, source.count / 200)

        var best: (transform: simd_double4x4, score: Double)?
        for (s0, s1) in signCombos {
            let a0 = s0 * srcA0, a1 = s1 * srcA1
            let srcBasis = simd_double3x3(columns: (a0, a1, simd_cross(a0, a1)))
            let rotation = simd_mul(refBasis, srcBasis.transpose)
            let translation = refCentroid - simd_mul(rotation, srcCentroid)
            let candidate = Mesh.rigidTransform(rotation: rotation, translation: translation)

            var sumDist = 0.0, n = 0
            var i = 0
            while i < source.count {
                let tp = Mesh.apply(candidate, source[i])
                if let (_, dist) = tree.nearest(to: tp) { sumDist += dist; n += 1 }
                i += sampleStride
            }
            let score = n > 0 ? sumDist / Double(n) : .infinity
            if best == nil || score < best!.score { best = (candidate, score) }
        }
        return best?.transform ?? matrix_identity_double4x4
    }

    // MARK: - Sampling

    /// Sample `count` indices proportional to normal-direction diversity: bucket by normal
    /// direction (a coarse lat/long grid), then round-robin across NON-EMPTY buckets in a fixed
    /// order so every direction gets comparable representation regardless of population size —
    /// the flat majority of a mostly-planar surface doesn't crowd out a small feature's rare
    /// normal direction. Deterministic (fixed bucket order, in-bucket order by point index).
    static func normalSpaceSample(normals: [SIMD3<Double>], count: Int) -> [Int] {
        guard count > 0, !normals.isEmpty else { return [] }
        guard count < normals.count else { return Array(0..<normals.count) }

        let lonBuckets = 12, latBuckets = 6
        func bucket(_ n: SIMD3<Double>) -> Int {
            let len = simd_length(n)
            let u = len > 1e-12 ? n / len : SIMD3<Double>(0, 0, 1)
            let lon = atan2(u.y, u.x)
            let lat = asin(max(-1, min(1, u.z)))
            let lonIdx = min(lonBuckets - 1, Int((lon + .pi) / (2 * .pi) * Double(lonBuckets)))
            let latIdx = min(latBuckets - 1, Int((lat + .pi / 2) / .pi * Double(latBuckets)))
            return latIdx * lonBuckets + lonIdx
        }

        var buckets: [Int: [Int]] = [:]
        for i in normals.indices { buckets[bucket(normals[i]), default: []].append(i) }
        let bucketKeys = buckets.keys.sorted()

        var cursors = [Int: Int](minimumCapacity: bucketKeys.count)
        for k in bucketKeys { cursors[k] = 0 }

        var picked: [Int] = []
        picked.reserveCapacity(count)
        var anyProgress = true
        while picked.count < count, anyProgress {
            anyProgress = false
            for k in bucketKeys {
                guard picked.count < count else { break }
                let list = buckets[k]!
                let cur = cursors[k]!
                guard cur < list.count else { continue }
                picked.append(list[cur])
                cursors[k] = cur + 1
                anyProgress = true
            }
        }
        return picked
    }

    /// Deterministic even-stride subsample of `0..<total`, `take` indices (or all of them, if
    /// `take >= total`).
    static func uniformSample(total: Int, take: Int) -> [Int] {
        guard take > 0, total > 0 else { return [] }
        guard take < total else { return Array(0..<total) }
        let stride = Double(total) / Double(take)
        var result: [Int] = []
        result.reserveCapacity(take)
        var seen = Set<Int>()
        var i = 0
        while result.count < take, i < total {
            let idx = min(total - 1, Int(Double(i) * stride))
            if seen.insert(idx).inserted { result.append(idx) }
            i += 1
        }
        return result
    }

    // MARK: - Rigid-transform helpers

    static func apply(_ m: simd_double4x4, _ p: SIMD3<Double>) -> SIMD3<Double> {
        let v = simd_mul(m, SIMD4<Double>(p.x, p.y, p.z, 1))
        return SIMD3<Double>(v.x, v.y, v.z)
    }

    static func rigidTransform(rotation: simd_double3x3, translation: SIMD3<Double>) -> simd_double4x4 {
        let c0 = SIMD4<Double>(rotation.columns.0, 0)
        let c1 = SIMD4<Double>(rotation.columns.1, 0)
        let c2 = SIMD4<Double>(rotation.columns.2, 0)
        let c3 = SIMD4<Double>(translation, 1)
        return simd_double4x4(columns: (c0, c1, c2, c3))
    }

    /// Rotation matrix for a right-handed rotation of `angle` radians about unit `axis`
    /// (Rodrigues' rotation formula).
    static func rodrigues(axis: SIMD3<Double>, angle: Double) -> simd_double3x3 {
        let c = cos(angle), s = sin(angle), t = 1 - c
        let x = axis.x, y = axis.y, z = axis.z
        let col0 = SIMD3<Double>(c + t * x * x, s * z + t * x * y, -s * y + t * x * z)
        let col1 = SIMD3<Double>(-s * z + t * y * x, c + t * y * y, s * x + t * y * z)
        let col2 = SIMD3<Double>(s * y + t * z * x, -s * x + t * z * y, c + t * z * z)
        return simd_double3x3(columns: (col0, col1, col2))
    }
}
