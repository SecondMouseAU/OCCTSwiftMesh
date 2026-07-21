// Mesh+Curvature.swift — Rusinkiewicz per-face curvature tensor, averaged onto vertices.
//
// Ported from the algorithm in Rusinkiewicz, "Estimating Curvatures and Their Derivatives on
// Triangle Meshes" (3DPVT 2004) — the same method trimesh2's TriMesh_curvature.cc (MIT) and the
// PMP library's curvature module implement. Chosen over Meyer et al.'s cotan-Laplacian approach
// specifically for robustness on noisy/irregular real-scan tessellation: this method fits a
// curvature tensor directly from each face's edge vectors and vertex-normal differences, with no
// obtuse-triangle clamp anywhere in the pipeline (the cotan approach needs one, and still
// degrades on slivers).
//
// Requires WELDED input, same precondition as `triangleAdjacency()`/`connectedComponents()` (see
// Mesh+Topology.swift's file header) — per-face tensors are averaged onto each vertex over every
// triangle sharing that vertex's WELDED index; on unwelded input every vertex touches exactly one
// triangle, so the result is just that triangle's own (unaveraged, and for a single flat triangle,
// zero) curvature repeated per corner.

import simd
import OCCTSwift

extension Mesh {

    /// Per-vertex principal curvatures and directions (see `VertexCurvature`).
    ///
    /// Requires a WELDED mesh — see the file header. Degenerate (near-zero-area) triangles are
    /// excluded from the fit entirely rather than propagating garbage: a face whose fit is
    /// singular, or whose contribution weight (area) is negligible, simply contributes nothing
    /// to its corners' averages. A vertex touched only by such faces (or by none at all) reports
    /// `k1 == k2 == 0`, never `NaN`.
    public func vertexCurvatures() -> [VertexCurvature] {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        let n = verts.count
        guard n > 0, tc > 0 else { return [] }

        let normalsF = vertexNormals()
        let normals = normalsF.map { SIMD3<Double>($0) }

        // Initial per-vertex tangent basis: an arbitrary but deterministic incident edge
        // (last-writer-wins over faces in triangle-index order, which is itself deterministic),
        // projected orthogonal to the vertex normal. Only used as bookkeeping for accumulating a
        // symmetric tensor per vertex — the final principal directions/curvatures (after
        // diagonalization) are independent of this initial choice.
        var pdir1 = [SIMD3<Double>](repeating: .zero, count: n)
        for t in 0..<tc {
            let base = t * 3
            let ia = Int(idx[base]), ib = Int(idx[base + 1]), ic = Int(idx[base + 2])
            let a = SIMD3<Double>(verts[ia]), b = SIMD3<Double>(verts[ib]), c = SIMD3<Double>(verts[ic])
            pdir1[ia] = b - a
            pdir1[ib] = c - b
            pdir1[ic] = a - c
        }
        for i in 0..<n {
            let nrm = normals[i]
            var p = simd_cross(pdir1[i], nrm)
            let len = simd_length(p)
            if len > 1e-12 {
                p /= len
            } else {
                let helper = abs(nrm.x) < 0.9 ? SIMD3<Double>(1, 0, 0) : SIMD3<Double>(0, 1, 0)
                p = simd_normalize(simd_cross(nrm, helper))
            }
            pdir1[i] = p
        }
        var pdir2 = [SIMD3<Double>](repeating: .zero, count: n)
        for i in 0..<n { pdir2[i] = simd_cross(normals[i], pdir1[i]) }

        var curv1 = [Double](repeating: 0, count: n)
        var curv12 = [Double](repeating: 0, count: n)
        var curv2 = [Double](repeating: 0, count: n)
        var weightSum = [Double](repeating: 0, count: n)

        for t in 0..<tc {
            let base = t * 3
            let vi = (Int(idx[base]), Int(idx[base + 1]), Int(idx[base + 2]))
            let p0 = SIMD3<Double>(verts[vi.0]), p1 = SIMD3<Double>(verts[vi.1]), p2 = SIMD3<Double>(verts[vi.2])
            // e[j] is the edge OPPOSITE vertex j.
            let e = (p2 - p1, p0 - p2, p1 - p0)
            let faceNormalUnnormalized = simd_cross(e.0, e.1)
            let twiceArea = simd_length(faceNormalUnnormalized)
            let longestEdge = max(simd_length(e.0), max(simd_length(e.1), simd_length(e.2)))
            // Sliver-robust: exclude degenerate/zero-area faces AND extreme slivers (area tiny
            // relative to the longest edge squared, i.e. a near-zero minimum angle) from the fit
            // entirely, rather than feeding an ill-conditioned edge/normal-difference system that
            // could solve to a huge (if still technically finite) garbage tensor. A documented
            // degradation — the sliver's own corners simply don't get this face's contribution —
            // rather than poisoning them with an unstable fit.
            guard longestEdge > 1e-15, twiceArea > 2e-4 * longestEdge * longestEdge else { continue }
            let tTang = e.0 / simd_length(e.0)
            let bTang = simd_normalize(simd_cross(faceNormalUnnormalized, tTang))
            guard tTang.x.isFinite, bTang.x.isFinite else { continue }

            let vn = (normals[vi.0], normals[vi.1], normals[vi.2])
            let edges = [e.0, e.1, e.2]

            var w00 = 0.0, w01 = 0.0, w22 = 0.0
            var m0 = 0.0, m1 = 0.0, m2 = 0.0
            for j in 0..<3 {
                let u = simd_dot(edges[j], tTang)
                let v = simd_dot(edges[j], bTang)
                w00 += u * u; w01 += u * v; w22 += v * v
                let vnPrev = j == 0 ? vn.2 : (j == 1 ? vn.0 : vn.1)   // (j+2)%3
                let vnNext = j == 0 ? vn.1 : (j == 1 ? vn.2 : vn.0)   // (j+1)%3
                let dn = vnPrev - vnNext
                let dnu = simd_dot(dn, tTang), dnv = simd_dot(dn, bTang)
                m0 += dnu * u; m1 += dnu * v + dnv * u; m2 += dnv * v
            }
            let w11 = w00 + w22
            let system: [[Double]] = [[w00, w01, 0], [w01, w11, w01], [0, w01, w22]]
            guard let sol = Linalg.solve(system, [m0, m1, m2]) else { continue }   // singular: skip
            let (eVal, fVal, gVal) = (sol[0], sol[1], sol[2])
            guard eVal.isFinite, fVal.isFinite, gVal.isFinite else { continue }

            let area = Double(twiceArea) * 0.5
            let corners = [vi.0, vi.1, vi.2]
            for vIdx in corners {
                let (ku, kuv, kv) = Mesh.projCurv(
                    oldU: tTang, oldV: bTang, oldKu: eVal, oldKuv: fVal, oldKv: gVal,
                    newU: pdir1[vIdx], newV: pdir2[vIdx])
                curv1[vIdx] += area * ku
                curv12[vIdx] += area * kuv
                curv2[vIdx] += area * kv
                weightSum[vIdx] += area
            }
        }

        var result = [VertexCurvature]()
        result.reserveCapacity(n)
        for i in 0..<n {
            let w = weightSum[i]
            guard w > 1e-15 else {
                result.append(VertexCurvature(k1: 0, k2: 0, d1: SIMD3<Float>(pdir1[i]), d2: SIMD3<Float>(pdir2[i])))
                continue
            }
            let ku = curv1[i] / w, kuv = curv12[i] / w, kv = curv2[i] / w
            let (pd1, pd2, k1, k2) = Mesh.diagonalizeCurv(
                oldU: pdir1[i], oldV: pdir2[i], ku: ku, kuv: kuv, kv: kv, newNorm: normals[i])
            if k1.isFinite, k2.isFinite, pd1.x.isFinite, pd2.x.isFinite {
                result.append(VertexCurvature(k1: k1, k2: k2, d1: SIMD3<Float>(pd1), d2: SIMD3<Float>(pd2)))
            } else {
                result.append(VertexCurvature(k1: 0, k2: 0, d1: SIMD3<Float>(pdir1[i]), d2: SIMD3<Float>(pdir2[i])))
            }
        }
        return result
    }

    // MARK: - Curvature-tensor rotation helpers (Rusinkiewicz 2004 / trimesh2)

    /// Rotate the tangent basis `(oldU, oldV)` so it's perpendicular to `newNorm` instead of its
    /// own implied normal `cross(oldU, oldV)`, via the minimal rotation that takes one normal to
    /// the other (exact for any pair of unit normals, not just nearby ones).
    static func rotCoordSys(oldU: SIMD3<Double>, oldV: SIMD3<Double>, newNorm: SIMD3<Double>)
        -> (SIMD3<Double>, SIMD3<Double>) {
        var newU = oldU
        var newV = oldV
        let oldNorm = simd_cross(oldU, oldV)
        let ndot = simd_dot(oldNorm, newNorm)
        // Antipodal (or numerically indistinguishable from it): rotating would divide by ~0 in
        // `dperp` below, so just flip instead of computing a blown-up correction.
        guard ndot > -1 + 1e-6 else { return (-newU, -newV) }
        let perpOld = newNorm - ndot * oldNorm
        let dperp = (1.0 / (1.0 + ndot)) * (oldNorm + newNorm)
        newU -= dperp * simd_dot(perpOld, newU)
        newV -= dperp * simd_dot(perpOld, newV)
        return (newU, newV)
    }

    /// Re-express a curvature tensor given in the `(oldU, oldV)` basis (as `[[oldKu, oldKuv],
    /// [oldKuv, oldKv]]`) in the `(newU, newV)` basis instead — `newU`/`newV` need not be exactly
    /// tangent to the same surface point; `rotCoordSys` reconciles the small normal mismatch.
    static func projCurv(oldU: SIMD3<Double>, oldV: SIMD3<Double>, oldKu: Double, oldKuv: Double, oldKv: Double,
                         newU: SIMD3<Double>, newV: SIMD3<Double>) -> (Double, Double, Double) {
        let oldNorm = simd_cross(oldU, oldV)
        let (rNewU, rNewV) = rotCoordSys(oldU: newU, oldV: newV, newNorm: oldNorm)
        let u1 = simd_dot(rNewU, oldU), v1 = simd_dot(rNewU, oldV)
        let u2 = simd_dot(rNewV, oldU), v2 = simd_dot(rNewV, oldV)
        let newKu = oldKu * u1 * u1 + oldKuv * (2 * u1 * v1) + oldKv * v1 * v1
        let newKuv = oldKu * u1 * u2 + oldKuv * (u1 * v2 + u2 * v1) + oldKv * v1 * v2
        let newKv = oldKu * u2 * u2 + oldKuv * (2 * u2 * v2) + oldKv * v2 * v2
        return (newKu, newKuv, newKv)
    }

    /// Diagonalize a vertex's accumulated curvature tensor (given in its own `(oldU, oldV)`
    /// basis) into principal curvatures/directions. `k1` is always the eigenvalue of larger
    /// magnitude; `pdir2` is always `cross(newNorm, pdir1)`.
    static func diagonalizeCurv(oldU: SIMD3<Double>, oldV: SIMD3<Double>, ku: Double, kuv: Double, kv: Double,
                                newNorm: SIMD3<Double>) -> (pdir1: SIMD3<Double>, pdir2: SIMD3<Double>, k1: Double, k2: Double) {
        let (rOldU, rOldV) = rotCoordSys(oldU: oldU, oldV: oldV, newNorm: newNorm)

        var c = 1.0, s = 0.0, tt = 0.0
        if kuv != 0 {
            let h = 0.5 * (kv - ku) / kuv
            tt = 1.0 / (abs(h) + (h * h + 1).squareRoot())
            if h < 0 { tt = -tt }
            c = 1.0 / (tt * tt + 1).squareRoot()
            s = tt * c
        }
        var k1 = ku - tt * kuv
        var k2 = kv + tt * kuv

        let pdir1: SIMD3<Double>
        if abs(k1) >= abs(k2) {
            pdir1 = c * rOldU - s * rOldV
        } else {
            swap(&k1, &k2)
            pdir1 = s * rOldU + c * rOldV
        }
        let pdir2 = simd_cross(newNorm, pdir1)
        return (pdir1, pdir2, k1, k2)
    }
}
