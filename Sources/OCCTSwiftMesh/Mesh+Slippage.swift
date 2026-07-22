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
// BASIS INVARIANCE. When the slippable null space is more than 1-dimensional (plane: 3-D,
// cylinder: 2-D, sphere: 3-D), the constraint being linear means ANY orthonormal basis of that
// subspace is an equally valid set of eigenvectors — Jacobi's particular choice among them is an
// artifact of tessellation noise and floating-point rounding, not a property of the surface. In
// an axis-aligned test fixture the null space happens to line up with R^6's coordinate axes and
// each eigenvector comes out "pure" (a lone translation or a lone rotation) by coincidence; under
// a generic pose the basis mixes and per-eigenvector classification silently misreads a plane as
// a sphere, a cylinder as freeform, and so on. The fix (below) classifies the SUBSPACE, not each
// eigenvector: the rank of the Gram matrix Σ ωₖωₖᵀ over the slippable eigenvectors' rotational
// parts is invariant to how that subspace's basis was chosen (an orthogonal change of basis
// within the subspace leaves Σ ωₖωₖᵀ exactly unchanged), and axis recovery uses only formulas
// that are similarly invariant to which combination of the true generators Jacobi happened to
// return. Only a 1-dimensional null space (extrusion/revolution/helix) has a basis unique enough
// (up to sign) for direct per-eigenvector classification.
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

        // How many of the 6 eigenvalue ratios are "slippable" (a rigid motion the surface
        // tolerates). Real surfaces never hit exactly zero, and — critically — how far from zero
        // varies with tessellation quality (a coarse/noisy sample can leave a genuinely slippable
        // mode an order of magnitude above a fixed cutoff while still being separated from the
        // non-slippable modes by a wide gap). So `d` is chosen by the SPECTRAL GAP: the split
        // point (among candidates under `slipCeiling`, which bounds how far "near zero" is ever
        // allowed to reach) with the largest jump to the next ratio, not a fixed threshold
        // comparison. `slipCeiling` only rules out treating an obviously-not-small ratio as
        // slippable regardless of gap size; `minGap` rejects a "best" gap too small to mean
        // anything in absolute terms. `minRelativeJump` additionally requires the excluded ratio
        // to be at least this many TIMES the included one — freeform data's smallest ratio can
        // still land under `slipCeiling` by chance (it's just wherever a generic 6-eigenvalue
        // spectrum happens to bottom out, not a real cluster), but the very next ratio then sits
        // close beside it rather than jumping away, and the relative test catches that a plain
        // absolute `minGap` (calibrated for genuine near-zero clusters, which sit orders of
        // magnitude below their neighbours) is too easily cleared by two merely-smallish values.
        let slipCeiling = 0.02
        let minGap = 2e-4
        let minRelativeJump = 3.0
        let (d, gap) = Mesh.selectSlippableCount(ratios: ratios, ceiling: slipCeiling, minGap: minGap,
                                                 minRelativeJump: minRelativeJump)
        let confidence = max(0, min(1, gap / slipCeiling))

        struct Mode { let omega: SIMD3<Double>; let v: SIMD3<Double> }
        let modes: [Mode] = (0..<d).map {
            let e = vectors[$0]
            return Mode(omega: SIMD3(e[0], e[1], e[2]), v: SIMD3(e[3], e[4], e[5]))
        }

        var centroid = SIMD3<Double>.zero
        for p in points { centroid += p }
        centroid /= Double(points.count)

        // Real-space conversion: axis points and pitch were derived in the NORMALIZED frame
        // (p' = (p - center) · scale); points transform back via the inverse affine map, and
        // pitch (a length) scales by 1/scale — direction vectors are untouched since
        // normalization has no rotational component.
        func toRealPoint(_ q: SIMD3<Double>) -> SIMD3<Double> { q / scale + center }
        func toRealPitch(_ p: Double) -> Double { p / scale }

        func result(_ kind: SlippageResult.Kind, axisPoint: SIMD3<Double>?, axisDirection: SIMD3<Double>?,
                   pitch: Double?) -> SlippageResult {
            SlippageResult(kind: kind, axisPoint: axisPoint, axisDirection: axisDirection, pitch: pitch,
                          eigenRatios: ratios, confidence: confidence)
        }

        guard d > 0 else { return result(.freeform, axisPoint: nil, axisDirection: nil, pitch: nil) }

        if d == 1 {
            // A 1-D null space has a basis unique up to sign — no basis-mixing ambiguity, so the
            // single eigenvector's own rotational/translational split is classified directly.
            let mode = modes[0]
            let omegaLen = simd_length(mode.omega)
            let componentThreshold = 1e-3   // fraction of the (unit-norm) eigenvector's own scale
            guard omegaLen >= componentThreshold else {
                let dir = simd_length(mode.v) > 1e-12 ? simd_normalize(mode.v) : SIMD3<Double>(0, 0, 1)
                return result(.extrusion, axisPoint: centroid, axisDirection: dir, pitch: nil)
            }
            let axis = mode.omega / omegaLen
            let vParallelLen = simd_dot(mode.v, axis)
            let vPerp = mode.v - vParallelLen * axis
            let q = simd_cross(mode.omega, vPerp) / (omegaLen * omegaLen)
            if abs(vParallelLen) < componentThreshold {
                return result(.revolution, axisPoint: toRealPoint(q), axisDirection: axis, pitch: nil)
            }
            return result(.helix, axisPoint: toRealPoint(q), axisDirection: axis,
                         pitch: toRealPitch(vParallelLen / omegaLen))
        }

        // d >= 2: classify the SUBSPACE via the rank of the Gram matrix of its eigenvectors'
        // rotational (ω) parts — invariant to which particular basis Jacobi returned for it (see
        // the file header). `r` is the number of independent rotation axes the slippable subspace
        // contains: 0 (pure translations only), 1 (all rotation content shares one axis — plane's
        // normal-axis rotation, or cylinder's axis), or 3 (sphere: rotation about any axis).
        var g = [[Double]](repeating: [0, 0, 0], count: 3)
        for mode in modes {
            let w = mode.omega
            g[0][0] += w.x * w.x; g[0][1] += w.x * w.y; g[0][2] += w.x * w.z
            g[1][1] += w.y * w.y; g[1][2] += w.y * w.z; g[2][2] += w.z * w.z
        }
        g[1][0] = g[0][1]; g[2][0] = g[0][2]; g[2][1] = g[1][2]
        let (gValues, gVectors) = Linalg.eigenSymmetric3(g)   // ascending
        let gMax = gValues.last ?? 0
        let rankFloor = 1e-2   // relative to gMax; the Gram matrix's own spread, not eigenRatios'
        let r = gMax > 1e-18 ? gValues.filter { $0 / gMax > rankFloor }.count : 0
        let axisDirection = gMax > 1e-18 ? SIMD3<Double>(gVectors[2][0], gVectors[2][1], gVectors[2][2]) : nil

        switch (d, r) {
        case (3, 1):   // plane: rotation content is 1-D (about the normal); the rest is translation
            return result(.plane, axisPoint: centroid, axisDirection: axisDirection, pitch: nil)

        case (3, 3):   // sphere: rotation content spans all of R³ — any axis through one center
            guard let q = Mesh.slippageSphereCenter(modes.map { ($0.omega, $0.v) }) else {
                return result(.sphere, axisPoint: nil, axisDirection: nil, pitch: nil)
            }
            return result(.sphere, axisPoint: toRealPoint(q), axisDirection: nil, pitch: nil)

        case (2, 1):   // cylinder: rotation about an axis + translation along that same axis
            guard let axis = axisDirection,
                  let q = Mesh.slippageAxisPoint(modes.map { ($0.omega, $0.v) }, axis: axis) else {
                return result(.freeform, axisPoint: nil, axisDirection: nil, pitch: nil)
            }
            return result(.cylinder, axisPoint: toRealPoint(q), axisDirection: axis, pitch: nil)

        default:
            return result(.freeform, axisPoint: nil, axisDirection: nil, pitch: nil)
        }
    }

    /// Picks the slippable-mode count `d` by the largest spectral gap among candidates whose
    /// upper (included) ratio stays under `ceiling` AND whose jump to the next ratio clears both
    /// `minGap` (absolute) and `minRelativeJump` (relative to the included ratio) — rather than a
    /// fixed threshold comparison. See the call site's comment. Returns `(0, 0)` if no candidate
    /// qualifies.
    static func selectSlippableCount(ratios: [Double], ceiling: Double, minGap: Double,
                                     minRelativeJump: Double) -> (d: Int, gap: Double) {
        var bestD = 0
        var bestGap = 0.0
        for d in 1..<ratios.count where ratios[d - 1] < ceiling {
            let gap = ratios[d] - ratios[d - 1]
            guard gap >= minGap, ratios[d] >= minRelativeJump * ratios[d - 1] else { continue }
            if gap > bestGap { bestGap = gap; bestD = d }
        }
        return bestGap > 0 ? (bestD, bestGap) : (0, 0)
    }

    /// The axis point for a rank-1 rotational subspace (cylinder — also correct for a 1-D
    /// null space's single rotation/helix mode, though that path is handled directly since it
    /// needs no subspace machinery). Any mode in the subspace with nonzero ω gives the SAME
    /// point: for two generators sharing an axis (pure rotation about it, pure translation along
    /// it), a combination `c = α·rotation + β·translation` has `ω_c = α·a` and
    /// `v_c,⊥ = α·v_rotation,⊥` — the `β` (translation) and `α` (overall scale) drop out of
    /// `q = ω_c × v_c,⊥ / |ω_c|²` entirely. Picks the largest-|ω| mode for conditioning.
    static func slippageAxisPoint(_ modes: [(omega: SIMD3<Double>, v: SIMD3<Double>)],
                                  axis: SIMD3<Double>) -> SIMD3<Double>? {
        guard let best = modes.max(by: { simd_length_squared($0.omega) < simd_length_squared($1.omega) }) else {
            return nil
        }
        let omegaLen2 = simd_length_squared(best.omega)
        guard omegaLen2 > 1e-18 else { return nil }
        let vPerp = best.v - simd_dot(best.v, axis) * axis
        return simd_cross(best.omega, vPerp) / omegaLen2
    }

    /// The common center of a rank-3 rotational subspace (sphere): every mode satisfies
    /// `v = -ω × q` for the SAME `q` (true regardless of which combination of the 3 true
    /// generators Jacobi returned, since the relation is linear in `(ω, v)`), so `q` is the
    /// least-squares solution of `ω × q = -v` stacked over every mode —
    /// `[Σ(|ω|²I - ωωᵀ)] q = Σ(ω × v)` — rather than averaging each mode's own axis-foot
    /// independently (which, for a non-orthogonal/non-canonical basis, is NOT the sphere's
    /// center: it systematically pulls the estimate toward the origin).
    static func slippageSphereCenter(_ modes: [(omega: SIMD3<Double>, v: SIMD3<Double>)]) -> SIMD3<Double>? {
        var ata = [[Double]](repeating: [0, 0, 0], count: 3)
        var atb = [Double](repeating: 0, count: 3)
        for mode in modes {
            let omegaLen = simd_length(mode.omega)
            guard omegaLen > 1e-9 else { continue }
            let axis = mode.omega / omegaLen
            let vPerp = mode.v - simd_dot(mode.v, axis) * axis
            let w = mode.omega
            ata[0][0] += omegaLen * omegaLen - w.x * w.x
            ata[1][1] += omegaLen * omegaLen - w.y * w.y
            ata[2][2] += omegaLen * omegaLen - w.z * w.z
            ata[0][1] -= w.x * w.y; ata[0][2] -= w.x * w.z; ata[1][2] -= w.y * w.z
            let cross = simd_cross(mode.omega, vPerp)
            atb[0] += cross.x; atb[1] += cross.y; atb[2] += cross.z
        }
        ata[1][0] = ata[0][1]; ata[2][0] = ata[0][2]; ata[2][1] = ata[1][2]
        guard let sol = Linalg.solve(ata, atb), sol.allSatisfy({ $0.isFinite }) else { return nil }
        return SIMD3<Double>(sol[0], sol[1], sol[2])
    }
}
