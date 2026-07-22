// CreaseDetectionResult.swift — result of Mesh.creaseEdges(minAngleDegrees:).

/// Dihedral-fold-edge rings/paths found by `Mesh.creaseEdges(minAngleDegrees:)`.
public struct CreaseDetectionResult: Sendable {
    /// Crease rings/paths, longest first (`CreaseRing.order`).
    public let rings: [CreaseRing]

    /// Crease edges that could not be chained into a ring/path — a defensive walk-length-cap
    /// backstop, not expected to fire on well-formed input. Counted, never silently dropped —
    /// the `SegmentedMesh.truncatedTriangleCount` convention.
    public let unchainedCreaseEdgeCount: Int

    public init(rings: [CreaseRing], unchainedCreaseEdgeCount: Int) {
        self.rings = rings
        self.unchainedCreaseEdgeCount = unchainedCreaseEdgeCount
    }
}
