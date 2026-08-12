// SimplifyOptions; input parameters for Mesh.simplified(_:).

import Foundation
import OCCTSwift

/// Parameters controlling QEM mesh decimation.
///
/// Exactly one of `targetTriangleCount` or `targetReduction` must be set;
/// passing both or neither causes `Mesh.simplified(_:)` to return `nil`.
///
/// ## Example
///
/// ```swift
/// var options = Mesh.SimplifyOptions(targetReduction: 0.5)
/// options.preserveBoundary = true
/// options.maxHausdorffDistance = 0.05
/// let result = mesh.simplified(options)
/// ```
extension Mesh {
    public struct SimplifyOptions: Sendable {
        /// Exact target number of triangles in the output mesh.
        ///
        /// Mutually exclusive with `targetReduction`.
        public var targetTriangleCount: Int?

        /// Fraction of input triangles to remove, in `[0.0, 1.0]`.
        ///
        /// `0.0` = no reduction; `1.0` = decimate as far as possible. Mutually exclusive with
        /// `targetTriangleCount`.
        public var targetReduction: Double?

        /// When `true`, edges on the mesh boundary (free edges) are not collapsed.
        public var preserveBoundary: Bool

        /// Forward-compatibility flag.
        ///
        /// The current backend always preserves topology; collapses that would change the
        /// surface's genus (creating holes, splitting components, merging components) are
        /// rejected regardless of this setting.
        public var preserveTopology: Bool

        /// Optional Hausdorff distance cap.
        ///
        /// When set, decimation halts as soon as the measured deviation from the input mesh
        /// would exceed this value, even if the target count has not been reached. Units match
        /// the input mesh's vertex coordinates. Must be `>= 0`.
        public var maxHausdorffDistance: Double?

        public init(
            targetTriangleCount: Int? = nil,
            targetReduction: Double? = nil,
            preserveBoundary: Bool = true,
            preserveTopology: Bool = true,
            maxHausdorffDistance: Double? = nil
        ) {
            self.targetTriangleCount = targetTriangleCount
            self.targetReduction = targetReduction
            self.preserveBoundary = preserveBoundary
            self.preserveTopology = preserveTopology
            self.maxHausdorffDistance = maxHausdorffDistance
        }
    }
}
