// VertexCurvature.swift — the per-vertex differential-geometry summary produced by
// `Mesh.vertexCurvatures()`.

import simd

/// Principal curvatures and directions estimated at one mesh vertex.
///
/// `k1` is the principal curvature of larger magnitude, `k2` the other — both signed such that a
/// convex bulge (e.g. an outward-facing sphere) is positive, matching the mesh's own vertex-normal
/// orientation. `d1`/`d2` are the corresponding unit tangent directions, mutually perpendicular
/// and perpendicular to the vertex normal (`d2` always `cross(normal, d1)`).
public struct VertexCurvature: Sendable, Equatable {
    /// Principal curvature of larger magnitude.
    public let k1: Double
    /// Principal curvature of smaller magnitude.
    public let k2: Double
    /// Unit tangent direction of `k1`.
    public let d1: SIMD3<Float>
    /// Unit tangent direction of `k2` (`= cross(normal, d1)`).
    public let d2: SIMD3<Float>

    public init(k1: Double, k2: Double, d1: SIMD3<Float>, d2: SIMD3<Float>) {
        self.k1 = k1
        self.k2 = k2
        self.d1 = d1
        self.d2 = d2
    }

    /// Mean curvature `(k1 + k2) / 2`.
    public var mean: Double { (k1 + k2) / 2 }
    /// Gaussian curvature `k1 * k2`.
    public var gaussian: Double { k1 * k2 }
}
