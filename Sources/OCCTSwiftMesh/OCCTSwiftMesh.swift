// OCCTSwiftMesh — mesh-domain algorithms for the OCCTSwift ecosystem.
//
// Mesh.simplified(_:) — QEM decimation via vendored meshoptimizer.
// Mesh.crossSection(plane:) — planar slicing into closed contours (a 3D-printer
//   slicer's perimeter step); robust on open / unwelded scan meshes.
// Mesh.welded(tolerance:), .faceNormals(), .vertexNormals(), .triangleAdjacency(),
//   .connectedComponents(), .subMesh(triangleIndices:), .boundaryLoops(),
//   .integrityReport() — the mesh connectivity toolkit and quality/validity snapshot.
// Mesh.segmented(_:) — dihedral region-growing + primitive-fit merge, splitting a mesh
//   into plane/cylinder/sphere/cone surface regions.
// See docs/CHANGELOG.md and docs/algorithms/.

/// Namespace marker for the OCCTSwiftMesh module. The public surface lives
/// on extensions of `OCCTSwift.Mesh` and the value types declared alongside
/// each algorithm — this enum exists only to give Xcode something concrete
/// to attach the module's documentation to.
public enum OCCTSwiftMesh {
    /// Package version. Bump on each tagged release.
    public static let version = "1.3.0"
}
