// CreaseRing.swift — one ring/path found by Mesh.creaseEdges(minAngleDegrees:).

import simd

/// One crease — a chain of dihedral-fold edges exceeding a fold-angle threshold, outlining a
/// recessed/raised feature (e.g. a door, panel, or window return) on a welded mesh.
public struct CreaseRing: Sendable {
    /// Ordered welded-mesh vertex indices along the ring/path. For a closed ring the last vertex
    /// connects back to the first (not repeated) — the same implicit-closure convention as
    /// `boundaryLoops()` / `MeshContour.points`.
    public let vertexIndices: [UInt32]

    /// `true` for a closed loop (e.g. a door outline); `false` for an open path (a crease that
    /// runs off an open mesh boundary, or terminates at a junction from just one side).
    public let closed: Bool

    /// Total edge length along the ring/path, in the mesh's own units.
    public let length: Double

    /// Axis-aligned bounds of the ring/path's vertices.
    public let bbox: (min: SIMD3<Float>, max: SIMD3<Float>)

    /// Mean dihedral fold angle across the ring/path's edges, in degrees.
    public let meanFoldAngleDegrees: Double

    /// Largest dihedral fold angle across the ring/path's edges, in degrees.
    public let maxFoldAngleDegrees: Double

    public init(vertexIndices: [UInt32], closed: Bool, length: Double,
                bbox: (min: SIMD3<Float>, max: SIMD3<Float>),
                meanFoldAngleDegrees: Double, maxFoldAngleDegrees: Double) {
        self.vertexIndices = vertexIndices
        self.closed = closed
        self.length = length
        self.bbox = bbox
        self.meanFoldAngleDegrees = meanFoldAngleDegrees
        self.maxFoldAngleDegrees = maxFoldAngleDegrees
    }

    /// Deterministic ordering used by `Mesh.creaseEdges`: longest first, ties broken by the
    /// lowest vertex index — the `MeshRegion.order` convention.
    static func order(_ a: CreaseRing, _ b: CreaseRing) -> Bool {
        if a.length != b.length { return a.length > b.length }
        return (a.vertexIndices.min() ?? 0) < (b.vertexIndices.min() ?? 0)
    }
}
