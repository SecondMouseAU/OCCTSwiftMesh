// SlippageResult.swift — output of Mesh.slippage(forTriangles:maxSamples:).

import simd

/// Classification of a region's surface kind by slippage analysis (Gelfand & Guibas, "Shape
/// Segmentation Using Local Slippage Analysis", SGP 2004), with the surface's characteristic
/// axis where one exists.
public struct SlippageResult: Sendable, Equatable {
    public enum Kind: String, Sendable, Equatable {
        case plane, sphere, cylinder, extrusion, revolution, helix, freeform
    }

    public let kind: Kind

    /// A point on the surface's characteristic axis (rotation/screw axis for `cylinder`,
    /// `revolution`, `helix`; the sphere's center; a representative point for `plane` and
    /// `extrusion`). `nil` for `freeform`.
    public let axisPoint: SIMD3<Double>?

    /// Unit direction of the surface's characteristic axis (rotation/screw axis for `cylinder`,
    /// `revolution`, `helix`; the extrude direction for `extrusion`; the normal for `plane`).
    /// `nil` for `sphere` (no preferred axis) and `freeform`.
    public let axisDirection: SIMD3<Double>?

    /// Translation distance per radian of rotation about `axisDirection`, `helix` only.
    public let pitch: Double?

    /// The 6 eigenvalue ratios of the slippage constraint covariance, ascending, each divided
    /// by the largest (so the last entry is always `1`). Near-zero entries are the slippable
    /// (rigid, surface-preserving) motions that drove the classification.
    public let eigenRatios: [Double]

    /// How cleanly the slippable eigenvalues separate from the non-slippable ones, in `[0, 1]`.
    public let confidence: Double

    public init(kind: Kind, axisPoint: SIMD3<Double>?, axisDirection: SIMD3<Double>?, pitch: Double?,
                eigenRatios: [Double], confidence: Double) {
        self.kind = kind
        self.axisPoint = axisPoint
        self.axisDirection = axisDirection
        self.pitch = pitch
        self.eigenRatios = eigenRatios
        self.confidence = confidence
    }
}
