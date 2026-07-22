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
// Mesh.aligned(to:options:) — point-to-plane ICP registration (PCA pre-align +
//   normal-space sampling + trimmed correspondence).
// Mesh.vertexCurvatures() — per-vertex principal curvatures/directions (Rusinkiewicz
//   per-face tensor method).
// Mesh.slippage(forTriangles:maxSamples:) — Gelfand-Guibas slippage analysis: classifies a
//   region's surface kind (plane/sphere/cylinder/extrusion/revolution/helix/freeform) and
//   recovers its characteristic axis.
// Mesh.creaseEdges(minAngleDegrees:) — dihedral-fold edge detection, chained into rings
//   (closed loops) and paths (open chains) outlining recessed/raised features.
// Mesh.windingNumber(at:), .orientationReport(samples:) — generalized winding number
//   (Jacobson/Kavan/Sorkine-Hornung): robust inside-out / orientation diagnostics on open,
//   soup, or self-intersecting meshes, where parity/ray tests break down.
// SegmentOptions.curvatureSeeding — opt-in curvature-ordered, seed-relative region growing
//   for segmented(_:), so flat regions claim their extent before a fillet/blend strip does.
// Mesh.segmentedRANSAC(_:), .segmentedAutoSelect(dihedral:ransac:) — Schnabel-style RANSAC
//   primitive extraction (an alternative to segmented(_:)'s dihedral growing for multi-
//   primitive scenes) and a measured bake-off between the two strategies.
// See docs/CHANGELOG.md and docs/algorithms/.

/// Namespace marker for the OCCTSwiftMesh module. The public surface lives
/// on extensions of `OCCTSwift.Mesh` and the value types declared alongside
/// each algorithm — this enum exists only to give Xcode something concrete
/// to attach the module's documentation to.
public enum OCCTSwiftMesh {
    /// Package version. Bump on each tagged release.
    public static let version = "1.7.0"
}
