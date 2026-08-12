// SimplifiedMesh; successful result of Mesh.simplified(_:).

import OCCTSwift

/// A decimated mesh together with the metadata describing the operation.
public struct SimplifiedMesh: Sendable {
    /// The decimated mesh.
    public let mesh: Mesh

    /// Triangle count of the input mesh.
    public let beforeTriangleCount: Int

    /// Triangle count of the output mesh.
    ///
    /// May exceed the requested target
    /// if the algorithm could not reduce further while respecting the
    /// `maxHausdorffDistance` cap or topology preservation.
    public let afterTriangleCount: Int

    /// Achieved Hausdorff distance from input to output mesh, in input units.
    ///
    /// Reported regardless of whether `maxHausdorffDistance` was set.
    public let hausdorffDistance: Double
}
