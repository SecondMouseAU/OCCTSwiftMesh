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

    /// Sample the generalized winding number at points offset outward from the mesh's own
    /// bounding box, to flag a globally inverted winding.
    ///
    /// - Parameter samples: number of exterior sample points, placed on a fixed (deterministic,
    ///   no randomness) Fibonacci-sphere spiral around the bounding box's center, at a radius
    ///   comfortably beyond the bounding box's own circumscribing sphere — every sample point is
    ///   therefore strictly outside the mesh, convex or not.
    ///
    /// ## Interpretation caveats
    ///
    /// `windingNumber(at:)` is exactly LINEAR in triangle orientation: reversing every triangle's
    /// winding negates `w(p)` at every point `p`, everywhere, unconditionally (a direct
    /// consequence of the solid-angle formula's antisymmetry under swapping two of a triangle's
    /// three vertices — see the file header). Two consequences follow that matter for reading
    /// this report correctly:
    ///
    /// 1. **On a genuinely closed, watertight mesh, this exterior-only check cannot detect a
    ///    global inversion.** A closed mesh's winding number at any point strictly outside every
    ///    enclosed volume is exactly `0` (the divergence-theorem fact underlying the whole
    ///    method) — REGARDLESS of orientation, since `w = 0` negated is still `0`. Global
    ///    inversion instead flips the INTERIOR reading (`≈ 1` → `≈ -1`); a caller who needs to
    ///    check a closed solid's orientation should sample a point known to be inside it (e.g.
    ///    its centroid, for a reasonably convex/blob-shaped body), not outside. `isOrientable` on
    ///    `integrityReport()` is the complementary check here: it catches inconsistent winding
    ///    (some triangles disagreeing with their neighbours), which this method does not, while
    ///    this method catches a globally-consistent-but-inside-out winding, which `isOrientable`
    ///    does not — see that property's doc comment.
    /// 2. **On an open shell (no enclosed volume at all) — the common case for a raw scan
    ///    surface patch — exterior sampling IS informative**, because there is no enclosed-volume
    ///    cancellation guaranteeing `0`: the winding number is fractional and its sign genuinely
    ///    tracks which way the shell's visible normals face relative to the sample points. This
    ///    is the primary intended use of this diagnostic. Values also grow fractional (neither
    ///    cleanly `0` nor `±1`) near an opening even on an otherwise mostly-closed shape, which is
    ///    why `meanExteriorWinding` reports a robust aggregate (the mean over many samples) rather
    ///    than any single sample.
    public func orientationReport(samples: Int = 64) -> OrientationReport {
        let verts = vertices
        guard !verts.isEmpty, triangleCount > 0, samples > 0 else {
            return OrientationReport(looksInverted: false, meanExteriorWinding: 0)
        }

        var lo = verts[0], hi = verts[0]
        for p in verts { lo = simd_min(lo, p); hi = simd_max(hi, p) }
        let center = SIMD3<Double>((lo + hi) * 0.5)
        let halfDiagonal = Double(simd_length(hi - lo)) * 0.5
        // Comfortably beyond the bounding SPHERE (radius == half the bbox diagonal), so every
        // sample point is strictly outside the mesh regardless of its shape; the absolute floor
        // covers the degenerate zero-size-bbox case (a single point / a coincident cluster).
        let sampleRadius = max(2.0 * halfDiagonal, 1e-6)

        // Deterministic Fibonacci-sphere spiral — evenly distributed sample directions with no
        // randomness, so two runs on identical input agree exactly.
        var windings: [Double] = []
        windings.reserveCapacity(samples)
        let angleStep = Double.pi * (3.0 - (5.0).squareRoot())
        for i in 0..<samples {
            let denom = Double(max(1, samples - 1))
            let y = 1.0 - (Double(i) / denom) * 2.0   // 1 down to -1
            let radiusAtY = (max(0, 1 - y * y)).squareRoot()
            let theta = angleStep * Double(i)
            let direction = SIMD3<Double>(cos(theta) * radiusAtY, y, sin(theta) * radiusAtY)
            let sample = center + direction * sampleRadius
            windings.append(windingNumber(at: sample))
        }

        let mean = windings.reduce(0, +) / Double(windings.count)
        // A correctly-oriented mesh reads ~0 (closed) or a small/non-negative fractional value
        // (open shell) at true exterior points; a clearly negative mean is the inversion signal
        // the file-header caveats describe. -0.25 sits well clear of float noise around 0 while
        // still catching a shell whose reversed winding pulls the mean substantially negative.
        let looksInverted = mean < -0.25
        return OrientationReport(looksInverted: looksInverted, meanExteriorWinding: mean)
    }
}
