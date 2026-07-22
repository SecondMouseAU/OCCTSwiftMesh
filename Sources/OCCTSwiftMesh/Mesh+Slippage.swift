// Mesh+Slippage.swift — per-region surface classification by local slippage analysis.
//
// Gelfand & Guibas, "Shape Segmentation Using Local Slippage Analysis" (SGP 2004): a rigid
// motion (angular velocity ω, linear velocity v) SLIPS a surface over itself iff its velocity
// field has zero normal component everywhere: ω·(pᵢ×nᵢ) + v·nᵢ = 0 for every sample (pᵢ, nᵢ).
// That's one linear constraint per sample on the 6-vector (ω, v); the slippable motions are the
// near-null space of Σᵢ cᵢcᵢᵀ, cᵢ = [pᵢ×nᵢ, nᵢ] — a 6×6 symmetric "slippage covariance" whose
// small eigenvalues (relative to the largest — never absolute, since raw magnitudes scale with
// patch extent) mark the surface's own symmetries. Their count and character (pure translation /
// pure rotation / coupled screw) identify the surface kind and, for the curved kinds, recover its
// axis directly from the eigenvector's rotational/translational parts.
//
// Like `triangleAdjacency()`/`connectedComponents()` (the "weld precondition family" — see
// Mesh+Topology.swift's header), this operates on THIS mesh's own vertex/normal arrays with no
// internal welding: callers pass a pre-welded mesh so `vertexNormals()` reflects real topology.

import Foundation
import simd
import OCCTSwift

extension Mesh {

    /// Classify a region's surface kind (plane / sphere / cylinder / extrusion / revolution /
    /// helix / freeform) and recover its characteristic axis, by local slippage analysis.
    ///
    /// Requires a welded mesh (`triangleAdjacency()`'s precondition family) — `vertexNormals()`
    /// only reflects real surface curvature once shared vertices are merged.
    ///
    /// - Parameters:
    ///   - triangleIndices: indices into THIS mesh's triangle list (as in `MeshRegion`).
    ///   - maxSamples: caps how many of the region's (deduplicated) vertices feed the analysis,
    ///     for speed on large regions. The same deterministic even-stride subsample as ICP's
    ///     `maxSamples` (`Mesh.uniformSample`).
    public func slippage(forTriangles triangleIndices: [Int], maxSamples: Int = 2000) -> SlippageResult {
        let degenerate = SlippageResult(kind: .freeform, axisPoint: nil, axisDirection: nil, pitch: nil,
                                        eigenRatios: [0, 0, 0, 0, 0, 0], confidence: 0)

        let verts = vertices
        let idx = indices
        guard !triangleIndices.isEmpty, !verts.isEmpty else { return degenerate }

        let normals = vertexNormals()

        // Per-vertex weight = 1/3 of the area of every region triangle touching it (barycentric
        // lumping, as in a mass matrix) — an UNweighted point sum badly overrepresents densely
        // tessellated patches (e.g. a lat/long UV sphere's pole rings: many vertices, each with
        // tiny actual surface area) relative to coarser ones, biasing the covariance away from
        // the continuous surface integral it's meant to approximate.
        var seen = Set<UInt32>()
        var order: [UInt32] = []
        var weight: [UInt32: Double] = [:]
        for t in triangleIndices {
            let base = t * 3
            guard base + 2 < idx.count else { continue }
            let tri = [idx[base], idx[base + 1], idx[base + 2]]
            let a = verts[Int(tri[0])], b = verts[Int(tri[1])], c = verts[Int(tri[2])]
            let area = Double(simd_length(simd_cross(b - a, c - a)) * 0.5)
            for g in tri {
                if seen.insert(g).inserted { order.append(g) }
                weight[g, default: 0] += area / 3
            }
        }
        // Need at least as many samples as unknowns (6) for the constraint covariance to be
        // meaningful at all.
        guard order.count >= 6 else { return degenerate }

        let take = max(0, min(maxSamples, order.count))
        let sampleVertexIdx = Mesh.uniformSample(total: order.count, take: take).map { order[$0] }
        let points = sampleVertexIdx.map { SIMD3<Double>(verts[Int($0)]) }
        let sampleWeights = sampleVertexIdx.map { weight[$0] ?? 0 }
        let sampleNormals = sampleVertexIdx.map { g -> SIMD3<Double> in
            let n = SIMD3<Double>(normals[Int(g)])
            let len = simd_length(n)
            return len > 1e-12 ? n / len : SIMD3<Double>(0, 0, 1)
        }

        // Normalize points to a unit box BEFORE eigen-analysis (points only — normals are
        // already unit and direction is scale-invariant) so eigenvalue RATIOS, not absolute
        // magnitudes, drive thresholding regardless of the region's physical size.
        var lo = points[0], hi = points[0]
        for p in points { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let center = (lo + hi) * 0.5
        let extent = hi - lo
        let maxExtent = max(extent.x, max(extent.y, extent.z))
        let scale = maxExtent > 1e-12 ? 1.0 / maxExtent : 1.0
        let normPoints = points.map { ($0 - center) * scale }

        var m = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
        for i in points.indices {
            let p = normPoints[i], n = sampleNormals[i], w = sampleWeights[i]
            let pxn = simd_cross(p, n)
            let row = [pxn.x, pxn.y, pxn.z, n.x, n.y, n.z]
            for a in 0..<6 {
                for b in a..<6 { m[a][b] += w * row[a] * row[b] }
            }
        }
        for a in 0..<6 { for b in 0..<a { m[a][b] = m[b][a] } }

        let (values, vectors) = Linalg.eigenSymmetric(m)   // ascending
        let lambdaMax = values.last ?? 0
        guard lambdaMax > 1e-15 else { return degenerate }
        let ratios = values.map { $0 / lambdaMax }

        // Eigenvalue-ratio threshold for "slippable" (a rigid motion the surface tolerates):
        // real surfaces never hit exactly zero, so this is a relative-noise-floor cutoff, not a
        // mathematical zero test.
        let slipThreshold = 1e-3
        // Within a slippable eigenvector (itself unit-norm over its 6 components), the threshold
        // below which a rotational (ω) or axial-translation (v·axis) component counts as zero.
        let componentThreshold = 1e-3

        struct Mode { let omega: SIMD3<Double>; let v: SIMD3<Double> }
        enum SubKind { case translation(SIMD3<Double>), rotation(SIMD3<Double>, SIMD3<Double>),
                       helix(SIMD3<Double>, SIMD3<Double>, Double) }

        func classify(_ mode: Mode) -> SubKind {
            let omegaLen = simd_length(mode.omega)
            guard omegaLen >= componentThreshold else {
                let dir = simd_length(mode.v) > 1e-12 ? simd_normalize(mode.v) : SIMD3<Double>(0, 0, 1)
                return .translation(dir)
            }
            let axis = mode.omega / omegaLen
            let vParallelLen = simd_dot(mode.v, axis)
            let vPerp = mode.v - vParallelLen * axis
            if abs(vParallelLen) < componentThreshold {
                let q = simd_cross(mode.omega, mode.v) / (omegaLen * omegaLen)
                return .rotation(axis, q)
            }
            let q = simd_cross(mode.omega, vPerp) / (omegaLen * omegaLen)
            return .helix(axis, q, vParallelLen / omegaLen)
        }

        // Real-space conversion: axis points and pitch were derived in the NORMALIZED frame
        // (p' = (p - center) · scale); points transform back via the inverse affine map, and
        // pitch (a length) scales by 1/scale — direction vectors are untouched since
        // normalization has no rotational component.
        func toRealPoint(_ q: SIMD3<Double>) -> SIMD3<Double> { q / scale + center }
        func toRealPitch(_ p: Double) -> Double { p / scale }

        let modes: [Mode] = (0..<6).filter { ratios[$0] < slipThreshold }.map {
            let e = vectors[$0]
            return Mode(omega: SIMD3(e[0], e[1], e[2]), v: SIMD3(e[3], e[4], e[5]))
        }
        let subKinds = modes.map(classify)

        var translations: [SIMD3<Double>] = []
        var rotations: [(axis: SIMD3<Double>, point: SIMD3<Double>)] = []
        var helices: [(axis: SIMD3<Double>, point: SIMD3<Double>, pitch: Double)] = []
        for sk in subKinds {
            switch sk {
            case .translation(let d): translations.append(d)
            case .rotation(let a, let q): rotations.append((a, q))
            case .helix(let a, let q, let p): helices.append((a, q, p))
            }
        }

        var centroid = SIMD3<Double>.zero
        for p in points { centroid += p }
        centroid /= Double(points.count)

        let confidence = Mesh.slippageConfidence(ratios: ratios, slippableCount: modes.count,
                                                  threshold: slipThreshold)

        func result(_ kind: SlippageResult.Kind, axisPoint: SIMD3<Double>?, axisDirection: SIMD3<Double>?,
                   pitch: Double?) -> SlippageResult {
            SlippageResult(kind: kind, axisPoint: axisPoint, axisDirection: axisDirection, pitch: pitch,
                          eigenRatios: ratios, confidence: confidence)
        }

        switch (translations.count, rotations.count, helices.count) {
        case (2, 1, 0):   // plane: 2 in-plane translations + 1 rotation about the normal
            return result(.plane, axisPoint: centroid, axisDirection: rotations[0].axis, pitch: nil)

        case (0, 3, 0):   // sphere: 3 independent rotations, all about the same center
            var q = SIMD3<Double>.zero
            for r in rotations { q += r.point }
            q /= 3
            return result(.sphere, axisPoint: toRealPoint(q), axisDirection: nil, pitch: nil)

        case (1, 1, 0) where abs(simd_dot(simd_normalize(translations[0]), rotations[0].axis)) > 0.9:
            // cylinder: rotation about an axis + translation along that same axis
            return result(.cylinder, axisPoint: toRealPoint(rotations[0].point), axisDirection: rotations[0].axis,
                         pitch: nil)

        case (1, 0, 0):   // extrusion: pure translation, no rotational symmetry
            return result(.extrusion, axisPoint: centroid, axisDirection: translations[0], pitch: nil)

        case (0, 1, 0):   // surface of revolution: pure rotation, axis through a fixed point
            return result(.revolution, axisPoint: toRealPoint(rotations[0].point), axisDirection: rotations[0].axis,
                         pitch: nil)

        case (0, 0, 1):   // helix: coupled rotation + translation along the same axis (a screw)
            let h = helices[0]
            return result(.helix, axisPoint: toRealPoint(h.point), axisDirection: h.axis, pitch: toRealPitch(h.pitch))

        default:
            return result(.freeform, axisPoint: nil, axisDirection: nil, pitch: nil)
        }
    }

    /// How cleanly the slippable eigenvalues separate from the non-slippable ones: the ratio
    /// gap straddling the slippable/non-slippable boundary, scaled by `threshold` and clamped to
    /// `[0, 1]`. A wide gap (slippable modes near-exactly zero, the rest clearly not) means a
    /// confident classification; a gap barely past the threshold means the boundary itself is
    /// close to arbitrary.
    static func slippageConfidence(ratios: [Double], slippableCount k: Int, threshold: Double) -> Double {
        guard threshold > 0 else { return 0 }
        let below = k > 0 ? ratios[k - 1] : 0
        let above = k < ratios.count ? ratios[k] : 1
        return max(0, min(1, (above - below) / threshold))
    }
}
