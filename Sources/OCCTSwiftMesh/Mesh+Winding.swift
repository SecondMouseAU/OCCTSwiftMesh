// Mesh+Winding.swift — generalized winding number (Jacobson, Kavan, Sorkine-Hornung, "Robust
// Inside-Out Segmentation Using Generalized Winding Numbers", SIGGRAPH 2013).
//
// The classical point-in-polygon tests (parity / ray-casting) need a CLOSED, non-self-
// intersecting surface to mean anything, and degrade unpredictably on the open shells and
// self-intersecting soup that real scan meshes actually are. The generalized winding number is
// the direct solid-angle sum instead: `w(p) = (1 / 4π) · Σ_triangles signedSolidAngle(p, triangle)`.
// For a closed, coherently-oriented mesh this recovers the usual indicator function exactly
// (`w ≈ 1` inside, `w ≈ 0` outside) via the divergence theorem, but — unlike parity/ray tests —
// it stays well-behaved (a smooth, well-defined real number) on open shells and self-intersecting
// input too, which is why it generalizes rather than merely computes the same thing differently.
//
// Per-triangle contribution uses the van Oosterom–Strackee (1983) formula for the solid angle
// subtended by a triangle from a point — numerically robust (no coordinate-frame branch, unlike
// naive spherical-excess formulas) and exact up to floating-point rounding.
//
// Pure geometry: no welding precondition (unlike the `triangleAdjacency()` family) — every
// triangle contributes independently regardless of whether the mesh is welded.

import simd
import OCCTSwift

extension Mesh {

    /// The generalized winding number of `point` with respect to this mesh — the direct
    /// solid-angle sum over every triangle (van Oosterom–Strackee), no spatial acceleration.
    ///
    /// For a closed, coherently-oriented mesh this is `≈ 1` for a point enclosed by the surface
    /// and `≈ 0` for a point outside it. On an open shell or self-intersecting input it stays a
    /// well-defined real number (fractional, not a crisp 0/1) rather than an undefined or
    /// arbitrary result the way a parity/ray test would give.
    ///
    /// O(triangleCount) per call — acceptable at diagnostic sample counts (a few dozen points ×
    /// a few hundred thousand triangles); a hierarchical (Barnes-Hut-style) evaluation would be
    /// the scale-up path for high-frequency sampling, not implemented here.
    public func windingNumber(at point: SIMD3<Double>) -> Double {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        guard tc > 0 else { return 0 }

        var sum = 0.0
        for t in 0..<tc {
            let base = t * 3
            let a = SIMD3<Double>(verts[Int(idx[base])]) - point
            let b = SIMD3<Double>(verts[Int(idx[base + 1])]) - point
            let c = SIMD3<Double>(verts[Int(idx[base + 2])]) - point
            let la = simd_length(a), lb = simd_length(b), lc = simd_length(c)
            // A query point exactly on a vertex makes the solid angle undefined — skip that
            // triangle's contribution rather than divide by (near-)zero.
            guard la > 1e-12, lb > 1e-12, lc > 1e-12 else { continue }

            let numerator = simd_dot(a, simd_cross(b, c))
            let denominator = la * lb * lc + simd_dot(a, b) * lc + simd_dot(b, c) * la + simd_dot(c, a) * lb
            sum += 2 * atan2(numerator, denominator)
        }
        return sum / (4 * .pi)
    }

    /// Sample the generalized winding number to flag a globally inverted winding — at points
    /// offset outward from the bounding box for a CLOSED mesh, or near the shell's own "hollow"
    /// for an OPEN one (see "Two sampling regimes" below; `boundaryLoops()` tells them apart).
    ///
    /// - Parameter samples: number of sample points (capped/adjusted internally per regime — see
    ///   each helper below), placed on a fixed (deterministic, no randomness) Fibonacci-sphere
    ///   pattern so two runs on identical input agree exactly.
    ///
    /// ## Interpretation caveats
    ///
    /// `windingNumber(at:)` is exactly LINEAR in triangle orientation: reversing every triangle's
    /// winding negates `w(p)` at every point `p`, everywhere, unconditionally (a direct
    /// consequence of the solid-angle formula's antisymmetry under swapping two of a triangle's
    /// three vertices — see the file header). One consequence matters enough to shape this
    /// method's whole design:
    ///
    /// **On a genuinely closed, watertight mesh, EXTERIOR sampling cannot detect a global
    /// inversion, structurally, not just as a tuning issue.** A closed mesh's winding number at
    /// any point strictly outside every enclosed volume is exactly `0` (the divergence-theorem
    /// fact underlying the whole method) — REGARDLESS of orientation, since `0` negated is still
    /// `0`. Global inversion instead flips the INTERIOR reading (`≈ 1` → `≈ -1`), which exterior
    /// sampling never sees. `isOrientable` on `integrityReport()` is the complementary check
    /// here: it catches inconsistent winding (some triangles disagreeing with their neighbours),
    /// which this method does not, while this method catches a globally-consistent-but-inside-out
    /// winding, which `isOrientable` does not — see that property's doc comment.
    ///
    /// ## Two sampling regimes
    ///
    /// **Closed mesh** (`boundaryLoops().isEmpty`): samples the bounding-box-EXTERIOR pattern —
    /// provably reads `≈ 0` regardless of orientation, per the caveat above. This branch exists
    /// so the documented limitation stays literally true rather than becoming accidentally
    /// informative through some other mechanism — a caller who needs to check a closed solid's
    /// orientation should sample a point known to be INSIDE it instead (e.g. its centroid, for a
    /// reasonably convex/blob-shaped body).
    ///
    /// **Open shell**: bounding-box-exterior sampling here structurally cancels for ANY open
    /// shell regardless of orientation — averaged over a full surrounding sphere, directions
    /// facing the shell's front read positive and directions facing its back read negative by
    /// (almost) the same amount, so the mean collapses toward zero no matter which way the shell
    /// is actually wound. This is not a threshold problem; no amount of retuning `-0.25` fixes an
    /// average that cancels by construction. Instead, this branch samples near the shell's own
    /// "hollow" — points clustered around the AREA-WEIGHTED CENTROID of the shell's own triangles
    /// (a position derived ONLY from vertex positions, deliberately never from face normals: the
    /// probe location must be IDENTICAL for a mesh and its reversal, or the reversed mesh's own
    /// flipped normals could bias where its own probes land, defeating the "windingNumber at an
    /// orientation-independent point negates exactly under reversal" argument this whole
    /// diagnostic leans on). For a shell shaped like a bowl, dome, or tube, that centroid sits
    /// inside the concavity, where the winding number is large in magnitude and reliably signed —
    /// see `WindingNumberTests.openShellHollowCenterReadsNonzero` for the mechanism directly.
    public func orientationReport(samples: Int = 64) -> OrientationReport {
        let verts = vertices
        let idx = indices
        let tc = triangleCount
        guard !verts.isEmpty, tc > 0, samples > 0 else {
            return OrientationReport(looksInverted: false, meanExteriorWinding: 0)
        }

        var lo = verts[0], hi = verts[0]
        for p in verts { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let halfDiagonal = Double(simd_length(hi - lo)) * 0.5

        let mean: Double
        if boundaryLoops().isEmpty {
            mean = exteriorSampleMean(samples: samples, halfDiagonal: halfDiagonal)
        } else {
            mean = hollowSampleMean(samples: samples, halfDiagonal: halfDiagonal,
                                    vertices: verts, indices: idx, triangleCount: tc)
        }
        // A correctly-oriented mesh reads ~0 (closed, by the caveat above) or a non-negative
        // value (open shell, near its own hollow); a clearly negative mean is the inversion
        // signal. -0.25 sits well clear of float noise around 0 while comfortably under the
        // ~0.3+ magnitudes a genuinely inverted open shell's hollow-probe mean shows in practice.
        let looksInverted = mean < -0.25
        return OrientationReport(looksInverted: looksInverted, meanExteriorWinding: mean)
    }

    /// The CLOSED-mesh sampling regime: a fixed Fibonacci-sphere spiral around the bounding box's
    /// center, at a radius comfortably beyond the bbox's own circumscribing sphere — every sample
    /// point is therefore strictly outside the mesh, convex or not, and (per the caveat on
    /// `orientationReport`) provably reads `≈ 0` regardless of orientation.
    private func exteriorSampleMean(samples: Int, halfDiagonal: Double) -> Double {
        var lo = vertices[0], hi = vertices[0]
        for p in vertices { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let center = SIMD3<Double>((lo + hi) * 0.5)
        // The absolute floor covers the degenerate zero-size-bbox case (a coincident cluster).
        let radius = max(2.0 * halfDiagonal, 1e-6)
        var sum = 0.0
        for direction in Mesh.fibonacciSphereDirections(samples) {
            sum += windingNumber(at: center + direction * radius)
        }
        return sum / Double(samples)
    }

    /// The OPEN-shell sampling regime: points clustered around the shell's own area-weighted
    /// triangle centroid — a position-only quantity (never derived from face normals; see
    /// `orientationReport`'s doc comment for why that matters) that lands inside the concavity
    /// for a bowl/dome/tube-shaped shell, where the winding number is large and reliably signed.
    /// A small jitter (a fixed fraction of the bbox half-diagonal) around that single point gives
    /// `samples` genuinely different-but-nearby probes to average, rather than repeating one
    /// point — still anchored close enough to the hollow to stay clear of the far-field
    /// cancellation `exteriorSampleMean` is subject to.
    private func hollowSampleMean(samples: Int, halfDiagonal: Double, vertices: [SIMD3<Float>],
                                  indices: [UInt32], triangleCount tc: Int) -> Double {
        var weightedSum = SIMD3<Double>.zero
        var totalArea = 0.0
        for t in 0..<tc {
            let base = t * 3
            let a = vertices[Int(indices[base])], b = vertices[Int(indices[base + 1])], c = vertices[Int(indices[base + 2])]
            let area = Double(simd_length(simd_cross(b - a, c - a))) * 0.5
            weightedSum += SIMD3<Double>(a + b + c) * (Double(1) / 3) * area
            totalArea += area
        }
        guard totalArea > 1e-15 else { return 0 }
        let centroid = weightedSum / totalArea

        let jitterRadius = max(0.15 * halfDiagonal, 1e-6)
        var sum = 0.0
        for direction in Mesh.fibonacciSphereDirections(samples) {
            sum += windingNumber(at: centroid + direction * jitterRadius)
        }
        return sum / Double(samples)
    }

    /// `samples` unit directions on a fixed Fibonacci-sphere spiral — deterministic (no
    /// randomness), evenly distributed, shared by both `orientationReport` sampling regimes.
    static func fibonacciSphereDirections(_ samples: Int) -> [SIMD3<Double>] {
        guard samples > 0 else { return [] }
        var directions: [SIMD3<Double>] = []
        directions.reserveCapacity(samples)
        let angleStep = Double.pi * (3.0 - (5.0).squareRoot())
        let denom = Double(max(1, samples - 1))
        for i in 0..<samples {
            let y = 1.0 - (Double(i) / denom) * 2.0   // 1 down to -1
            let radiusAtY = (max(0, 1 - y * y)).squareRoot()
            let theta = angleStep * Double(i)
            directions.append(SIMD3(cos(theta) * radiusAtY, y, sin(theta) * radiusAtY))
        }
        return directions
    }
}
