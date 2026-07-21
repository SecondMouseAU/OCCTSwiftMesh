// PrimitiveFitter.swift — fits plane / sphere / cylinder / cone candidates to a region's
// points and picks the best, with a simplicity bias. Internal engine behind
// `Mesh.segmented(_:)`; ported from OCCTReconstruct's ReconstructCompute.PrimitiveFitting.

import Foundation
import simd
import OCCTSwift

enum PrimitiveFitter {

    /// Fit the best primitive (lowest residual, with a simplicity bias toward planes) to a
    /// region. `vertices`/`indices` should be the mesh's ORIGINAL (unwelded) arrays — fitting
    /// wants the truest available geometry, unperturbed by weld snapping.
    static func bestFit(vertices: [SIMD3<Float>], indices: [UInt32], region: MeshRegion,
                        faceNormals: [SIMD3<Float>]) -> FittedPrimitive {
        let pts = regionPoints(vertices: vertices, indices: indices, region: region)
        let normals = region.triangleIndices.map { SIMD3<Double>(faceNormals[$0]) }

        var candidates: [FittedPrimitive] = []
        if let plane = fitPlane(points: pts) { candidates.append(plane) }
        if let sphere = fitSphere(points: pts) { candidates.append(sphere) }
        if let cyl = fitCylinder(points: pts, normals: normals) { candidates.append(cyl) }
        if let cone = fitCone(points: pts, normals: normals) { candidates.append(cone) }

        // A cone with a tiny half-angle IS a cylinder the cone fit stole: its extra DOF absorbs
        // scan noise so it always scores >= the cylinder, and the preference tie-break only
        // fires within 1.25x. Drop near-cylindrical cones whenever a cylinder candidate exists.
        if let coneIdx = candidates.firstIndex(where: { $0.kind == .cone }),
           let half = candidates[coneIdx].coneHalfAngleDegrees, abs(half) < 2.5,
           candidates.contains(where: { $0.kind == .cylinder }) {
            candidates.remove(at: coneIdx)
        }

        guard let bestRMS = candidates.map(\.residualRMS).min() else {
            return FittedPrimitive(kind: .plane, params: [0, 0, 1, 0],
                                   residualRMS: .infinity, residualMax: .infinity, inlierRatio: 0)
        }
        // Among fits about as good as the best residual, prefer the simpler / more-common
        // primitive (plane > cylinder > cone > sphere) — this breaks ambiguous ties (e.g. a
        // short band a sphere also fits perfectly is really a cylinder). The absolute floor
        // matters: when the best fit is near-zero, a near-perfect cone must still count as
        // "acceptable" against an exact sphere. Scaled by the REGION's own bounding-box
        // diagonal, not the whole mesh's — a floor scaled by a large body would swamp a small,
        // genuinely-curved region's tiny residual and misclassify a shallow arc as a plane
        // (issue #20 item 4: a big flat body with an R5000-class, few-mm-sagitta roof).
        var lo = pts.first ?? .zero, hi = pts.first ?? .zero
        for p in pts { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let regionDiag = simd_length(hi - lo)
        let absFloor = 1e-3 * regionDiag
        let preference: [FittedPrimitive.Kind: Int] = [.plane: 0, .cylinder: 1, .cone: 2, .sphere: 3]
        let acceptable = candidates.filter { $0.residualRMS <= bestRMS * 1.25 + absFloor }
        return acceptable.min { (preference[$0.kind] ?? 9) < (preference[$1.kind] ?? 9) }!
    }

    // MARK: Plane

    static func fitPlane(points pts: [SIMD3<Double>]) -> FittedPrimitive? {
        guard pts.count >= 3 else { return nil }
        let (cov, centroid) = Linalg.covariance(pts)
        let (_, vectors) = Linalg.eigenSymmetric3(cov)
        let n = SIMD3<Double>(vectors[0][0], vectors[0][1], vectors[0][2])   // smallest-variance dir
        let d = simd_dot(n, centroid)
        let residuals = pts.map { abs(simd_dot(n, $0) - d) }
        let (rms, mx, inl) = stats(residuals)
        return FittedPrimitive(kind: .plane, params: [n.x, n.y, n.z, d],
                               residualRMS: rms, residualMax: mx, inlierRatio: inl)
    }

    // MARK: Sphere (algebraic / Kåsa)

    static func fitSphere(points pts: [SIMD3<Double>]) -> FittedPrimitive? {
        guard pts.count >= 4 else { return nil }
        // Solve [2x,2y,2z,1]·[cx,cy,cz,k] = x²+y²+z², then r² = k + |c|².
        var ata = [[Double]](repeating: [0, 0, 0, 0], count: 4)
        var atb = [Double](repeating: 0, count: 4)
        for p in pts {
            let row = [2 * p.x, 2 * p.y, 2 * p.z, 1.0]
            let rhs = p.x * p.x + p.y * p.y + p.z * p.z
            for i in 0..<4 {
                atb[i] += row[i] * rhs
                for j in 0..<4 { ata[i][j] += row[i] * row[j] }
            }
        }
        guard let sol = Linalg.solve(ata, atb) else { return nil }
        let c = SIMD3<Double>(sol[0], sol[1], sol[2])
        let r2 = sol[3] + simd_dot(c, c)
        guard r2 > 0 else { return nil }
        let r = r2.squareRoot()
        let residuals = pts.map { abs(simd_length($0 - c) - r) }
        let (rms, mx, inl) = stats(residuals)
        return FittedPrimitive(kind: .sphere, params: [c.x, c.y, c.z, r],
                               residualRMS: rms, residualMax: mx, inlierRatio: inl)
    }

    // MARK: Cylinder

    static func fitCylinder(points pts: [SIMD3<Double>], normals: [SIMD3<Double>]) -> FittedPrimitive? {
        guard pts.count >= 6, !normals.isEmpty else { return nil }
        // Axis = direction the surface normals avoid = smallest eigenvector of Σ nnᵀ.
        var s = [[0.0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for n in normals {
            s[0][0] += n.x * n.x; s[0][1] += n.x * n.y; s[0][2] += n.x * n.z
            s[1][1] += n.y * n.y; s[1][2] += n.y * n.z; s[2][2] += n.z * n.z
        }
        s[1][0] = s[0][1]; s[2][0] = s[0][2]; s[2][1] = s[1][2]
        let (_, evecs) = Linalg.eigenSymmetric3(s)
        let axis = simd_normalize(SIMD3<Double>(evecs[0][0], evecs[0][1], evecs[0][2]))

        let helper = abs(axis.x) < 0.9 ? SIMD3<Double>(1, 0, 0) : SIMD3<Double>(0, 1, 0)
        let u = simd_normalize(simd_cross(axis, helper))
        let v = simd_cross(axis, u)

        var origin = SIMD3<Double>.zero
        for p in pts { origin += p }
        origin /= Double(pts.count)

        // Fit a circle in (u, v): [2pu, 2pv, 1]·[a, b, k] = pu²+pv², r² = k + a²+b².
        var ata = [[Double]](repeating: [0, 0, 0], count: 3)
        var atb = [Double](repeating: 0, count: 3)
        var proj: [(Double, Double)] = []
        proj.reserveCapacity(pts.count)
        for p in pts {
            let rel = p - origin
            let pu = simd_dot(rel, u), pv = simd_dot(rel, v)
            proj.append((pu, pv))
            let row = [2 * pu, 2 * pv, 1.0]
            let rhs = pu * pu + pv * pv
            for i in 0..<3 {
                atb[i] += row[i] * rhs
                for j in 0..<3 { ata[i][j] += row[i] * row[j] }
            }
        }
        guard let sol = Linalg.solve(ata, atb) else { return nil }
        let (a, b) = (sol[0], sol[1])
        let r2 = sol[2] + a * a + b * b
        guard r2 > 0 else { return nil }
        let r = r2.squareRoot()
        let center3d = origin + a * u + b * v

        let residuals = proj.map { abs((($0.0 - a) * ($0.0 - a) + ($0.1 - b) * ($0.1 - b)).squareRoot() - r) }
        let (rms, mx, inl) = stats(residuals)
        return FittedPrimitive(kind: .cylinder,
                               params: [center3d.x, center3d.y, center3d.z, axis.x, axis.y, axis.z, r],
                               residualRMS: rms, residualMax: mx, inlierRatio: inl)
    }

    // MARK: Cone

    static func fitCone(points pts: [SIMD3<Double>], normals: [SIMD3<Double>]) -> FittedPrimitive? {
        guard pts.count >= 6, normals.count >= 3 else { return nil }

        // On a cone, every surface normal makes a constant angle with the axis: n·a = -sin(α).
        // So the unit normals lie in a plane whose normal is the axis a. Fit that plane: a is
        // the least-variance direction of the (centered) normals; sin(α) = |mean(n·a)|.
        let (cov, meanNormal) = Linalg.covariance(normals)
        let (_, evecs) = Linalg.eigenSymmetric3(cov)
        var axis = simd_normalize(SIMD3<Double>(evecs[0][0], evecs[0][1], evecs[0][2]))
        let sinAlpha = abs(simd_dot(meanNormal, axis))
        guard sinAlpha > 0.0871, sinAlpha < 0.9962 else { return nil }   // α ∈ (~5°, ~85°)
        let alpha = asin(min(max(sinAlpha, 0), 1))
        let cosAlpha = cos(alpha)

        var c0 = SIMD3<Double>.zero
        for p in pts { c0 += p }
        c0 /= Double(pts.count)

        func axialRadial(_ axis: SIMD3<Double>) -> (A: [Double], R: [Double]) {
            var A = [Double](), R = [Double]()
            A.reserveCapacity(pts.count); R.reserveCapacity(pts.count)
            for p in pts {
                let rel = p - c0
                let a = simd_dot(rel, axis)
                let radial = simd_length(rel - a * axis)
                A.append(a); R.append(radial)
            }
            return (A, R)
        }

        var (axialA, radialR) = axialRadial(axis)
        // Orient the axis so radius grows away from the apex (positive covariance of A and R).
        let meanA = axialA.reduce(0, +) / Double(pts.count)
        let meanR = radialR.reduce(0, +) / Double(pts.count)
        var covAR = 0.0
        for i in 0..<pts.count { covAR += (axialA[i] - meanA) * (radialR[i] - meanR) }
        if covAR < 0 { axis = -axis; (axialA, radialR) = axialRadial(axis) }

        // Apex q = c0 + s·a. Perpendicular deviation d_i = R_i cosα − A_i sinα + s sinα; solve s.
        var meanE = 0.0
        for i in 0..<pts.count { meanE += radialR[i] * cosAlpha - axialA[i] * sinAlpha }
        meanE /= Double(pts.count)
        let s = -meanE / sinAlpha
        let apex = c0 + s * axis

        let residuals = (0..<pts.count).map {
            abs(radialR[$0] * cosAlpha - axialA[$0] * sinAlpha + s * sinAlpha)
        }
        let (rms, mx, inl) = stats(residuals)
        return FittedPrimitive(kind: .cone,
                               params: [apex.x, apex.y, apex.z, axis.x, axis.y, axis.z, alpha],
                               residualRMS: rms, residualMax: mx, inlierRatio: inl)
    }

    // MARK: Helpers

    static func regionPoints(vertices: [SIMD3<Float>], indices: [UInt32], region: MeshRegion) -> [SIMD3<Double>] {
        var seen = Set<UInt32>()
        var pts: [SIMD3<Double>] = []
        for t in region.triangleIndices {
            let base = t * 3
            for g in [indices[base], indices[base + 1], indices[base + 2]] where seen.insert(g).inserted {
                pts.append(SIMD3<Double>(vertices[Int(g)]))
            }
        }
        return pts
    }

    /// (rms, max, inlierRatio) of residuals. Inlier = within max(2·rms, 1e-4).
    static func stats(_ residuals: [Double]) -> (Double, Double, Double) {
        guard !residuals.isEmpty else { return (.infinity, .infinity, 0) }
        var sumSq = 0.0, mx = 0.0
        for r in residuals { sumSq += r * r; mx = max(mx, r) }
        let rms = (sumSq / Double(residuals.count)).squareRoot()
        let tol = max(2 * rms, 1e-4)
        let inliers = residuals.reduce(0) { $0 + ($1 <= tol ? 1 : 0) }
        return (rms, mx, Double(inliers) / Double(residuals.count))
    }
}
