// Linalg.swift — small dense linear algebra used by primitive fitting.
//
// Fitting runs in Double for stability even though mesh data is Float. Internal to the
// package — PrimitiveFitter is the only consumer.

import Foundation
import simd

enum Linalg {

    /// Eigen-decomposition of a symmetric 3×3 matrix via cyclic Jacobi rotations.
    /// Returns eigenvalues ascending, with `vectors[k]` the unit eigenvector for `values[k]`.
    static func eigenSymmetric3(_ input: [[Double]]) -> (values: [Double], vectors: [[Double]]) {
        var a = input
        var v: [[Double]] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        let pairs = [(0, 1), (0, 2), (1, 2)]

        for _ in 0..<60 {
            var (p, q) = (0, 1)
            var off = abs(a[0][1])
            for (i, j) in pairs where abs(a[i][j]) > off { off = abs(a[i][j]); p = i; q = j }
            if off < 1e-15 { break }

            let phi = 0.5 * atan2(2 * a[p][q], a[q][q] - a[p][p])
            let c = cos(phi), s = sin(phi)

            for k in 0..<3 {
                let akp = a[k][p], akq = a[k][q]
                a[k][p] = c * akp - s * akq
                a[k][q] = s * akp + c * akq
            }
            for k in 0..<3 {
                let apk = a[p][k], aqk = a[q][k]
                a[p][k] = c * apk - s * aqk
                a[q][k] = s * apk + c * aqk
            }
            for k in 0..<3 {
                let vkp = v[k][p], vkq = v[k][q]
                v[k][p] = c * vkp - s * vkq
                v[k][q] = s * vkp + c * vkq
            }
        }

        var triples = (0..<3).map { (a[$0][$0], [v[0][$0], v[1][$0], v[2][$0]]) }
        triples.sort { $0.0 < $1.0 }
        return (triples.map { $0.0 }, triples.map { normalize3($0.1) })
    }

    /// Covariance (scatter) matrix of points about their centroid, as a 3×3 symmetric matrix.
    static func covariance(_ points: [SIMD3<Double>]) -> (matrix: [[Double]], centroid: SIMD3<Double>) {
        guard !points.isEmpty else { return ([[0, 0, 0], [0, 0, 0], [0, 0, 0]], .zero) }
        var c = SIMD3<Double>.zero
        for p in points { c += p }
        c /= Double(points.count)
        var m = [[0.0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for p in points {
            let d = p - c
            m[0][0] += d.x * d.x; m[0][1] += d.x * d.y; m[0][2] += d.x * d.z
            m[1][1] += d.y * d.y; m[1][2] += d.y * d.z; m[2][2] += d.z * d.z
        }
        m[1][0] = m[0][1]; m[2][0] = m[0][2]; m[2][1] = m[1][2]
        return (m, c)
    }

    /// Solve a small linear system `Ax = b` by Gaussian elimination with partial pivoting.
    /// Returns nil if singular.
    static func solve(_ A: [[Double]], _ b: [Double]) -> [Double]? {
        let n = b.count
        var m = A.map { $0 }
        var x = b
        for col in 0..<n {
            var pivot = col
            for r in (col + 1)..<n where abs(m[r][col]) > abs(m[pivot][col]) { pivot = r }
            if abs(m[pivot][col]) < 1e-15 { return nil }
            m.swapAt(col, pivot); x.swapAt(col, pivot)
            let inv = 1.0 / m[col][col]
            for r in 0..<n where r != col {
                let f = m[r][col] * inv
                if f == 0 { continue }
                for k in col..<n { m[r][k] -= f * m[col][k] }
                x[r] -= f * x[col]
            }
        }
        for i in 0..<n { x[i] /= m[i][i] }
        return x
    }

    static func normalize3(_ v: [Double]) -> [Double] {
        let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).squareRoot()
        return len > 1e-300 ? [v[0] / len, v[1] / len, v[2] / len] : [0, 0, 1]
    }

    /// Eigen-decomposition of a symmetric N×N matrix via cyclic (classical) Jacobi rotations —
    /// the same method as `eigenSymmetric3`, generalized to arbitrary size. Used by slippage
    /// analysis's 6×6 constraint covariance ("Jacobi generalizes directly" per the algorithm's
    /// source). Returns eigenvalues ascending, with `vectors[k]` the unit eigenvector for
    /// `values[k]`.
    static func eigenSymmetric(_ input: [[Double]]) -> (values: [Double], vectors: [[Double]]) {
        let n = input.count
        var a = input
        var v = [[Double]](repeating: [Double](repeating: 0, count: n), count: n)
        for i in 0..<n { v[i][i] = 1 }

        // More sweeps than eigenSymmetric3's 60: N(N-1)/2 off-diagonal pairs (15 at N=6, vs 3
        // at N=3) means classical (largest-element) Jacobi needs proportionally more single-pair
        // rotations to reduce every pair below the tolerance.
        for _ in 0..<(20 * n * n) {
            var off = 0.0
            var p = 0, q = 1
            for i in 0..<n {
                for j in (i + 1)..<n where abs(a[i][j]) > off { off = abs(a[i][j]); p = i; q = j }
            }
            if off < 1e-15 { break }

            let phi = 0.5 * atan2(2 * a[p][q], a[q][q] - a[p][p])
            let c = cos(phi), s = sin(phi)

            for k in 0..<n {
                let akp = a[k][p], akq = a[k][q]
                a[k][p] = c * akp - s * akq
                a[k][q] = s * akp + c * akq
            }
            for k in 0..<n {
                let apk = a[p][k], aqk = a[q][k]
                a[p][k] = c * apk - s * aqk
                a[q][k] = s * apk + c * aqk
            }
            for k in 0..<n {
                let vkp = v[k][p], vkq = v[k][q]
                v[k][p] = c * vkp - s * vkq
                v[k][q] = s * vkp + c * vkq
            }
        }

        var triples = (0..<n).map { i in (a[i][i], (0..<n).map { v[$0][i] }) }
        triples.sort { $0.0 < $1.0 }
        return (triples.map { $0.0 }, triples.map { normalizeN($0.1) })
    }

    static func normalizeN(_ v: [Double]) -> [Double] {
        let len = v.reduce(0) { $0 + $1 * $1 }.squareRoot()
        guard len > 1e-300 else {
            var fallback = [Double](repeating: 0, count: v.count)
            fallback[v.count - 1] = 1
            return fallback
        }
        return v.map { $0 / len }
    }
}
