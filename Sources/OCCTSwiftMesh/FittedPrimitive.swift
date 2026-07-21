// FittedPrimitive.swift — the surface a segmented region best fits, with its measured
// deviation from the source triangles.

/// A primitive surface fitted to a mesh region.
public struct FittedPrimitive: Sendable, Equatable {
    public enum Kind: String, Sendable, Equatable {
        case plane, cylinder, sphere, cone
    }

    public let kind: Kind

    /// - `plane`:    `[nx, ny, nz, d]` (`n·x = d`, `|n| = 1`)
    /// - `sphere`:   `[cx, cy, cz, r]`
    /// - `cylinder`: `[px, py, pz, ax, ay, az, r]` (point on axis, unit axis direction, radius)
    /// - `cone`:     `[apexX, apexY, apexZ, ax, ay, az, halfAngle]` (apex, unit axis direction,
    ///   half-angle in radians)
    public let params: [Double]

    public let residualRMS: Double
    public let residualMax: Double
    /// Fraction of the region's vertices within `max(2 · residualRMS, 1e-4)` of the surface.
    public let inlierRatio: Double

    public init(kind: Kind, params: [Double], residualRMS: Double, residualMax: Double, inlierRatio: Double) {
        self.kind = kind
        self.params = params
        self.residualRMS = residualRMS
        self.residualMax = residualMax
        self.inlierRatio = inlierRatio
    }

    public var radius: Double? {
        switch kind {
        case .sphere: return params.count >= 4 ? params[3] : nil
        case .cylinder: return params.count >= 7 ? params[6] : nil
        case .plane, .cone: return nil
        }
    }

    /// Half-angle in degrees, for cones.
    public var coneHalfAngleDegrees: Double? {
        kind == .cone && params.count >= 7 ? params[6] * 180 / .pi : nil
    }
}
