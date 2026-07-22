// OrientationReport.swift — result of Mesh.orientationReport(samples:).

/// A global inside-out / winding-orientation diagnostic (see `Mesh.orientationReport(samples:)`).
public struct OrientationReport: Sendable, Equatable {
    /// `true` when the sampled exterior winding pattern suggests the mesh's triangle winding is
    /// globally inverted (normals pointing inward). See the caveats on
    /// `Mesh.orientationReport(samples:)` — this is a documented heuristic, not a certainty, and
    /// is powerless on a genuinely closed, watertight mesh (see that method's doc comment).
    public let looksInverted: Bool

    /// Mean generalized winding number across the sampled exterior points. Near `0` for a
    /// correctly-oriented mesh (or any closed, watertight mesh regardless of orientation — see
    /// caveats); markedly negative suggests inversion.
    public let meanExteriorWinding: Double

    public init(looksInverted: Bool, meanExteriorWinding: Double) {
        self.looksInverted = looksInverted
        self.meanExteriorWinding = meanExteriorWinding
    }
}
