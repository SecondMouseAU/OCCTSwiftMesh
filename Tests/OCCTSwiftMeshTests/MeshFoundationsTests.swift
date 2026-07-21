import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.welded — vertex welding")
struct WeldingTests {
    @Test("Per-triangle-unique-vertex box welds down to the shared-vertex form")
    func unweldedCubeWelds() {
        let cube = unweldedUnitCube()
        #expect(cube.vertexCount == 36)
        let welded = cube.welded()
        #expect(welded.vertexCount == 8)
        #expect(welded.triangleCount == 12)
    }

    @Test("Welding an already-welded mesh is idempotent")
    func weldIdempotence() {
        let once = unweldedUnitCube().welded()
        let twice = once.welded()
        #expect(once.vertexCount == twice.vertexCount)
        #expect(once.triangleCount == twice.triangleCount)

        let alreadyWelded = weldedUnitCube()
        let stillWelded = alreadyWelded.welded()
        #expect(stillWelded.vertexCount == alreadyWelded.vertexCount)
        #expect(stillWelded.triangleCount == alreadyWelded.triangleCount)
    }

    @Test("Tolerance controls whether near-coincident vertices merge")
    func weldToleranceBoundary() {
        // P0/P1 are 0.0003 apart — well inside a single grid cell at tolerance 0.001 (ratio
        // 0.3, rounds to the same cell as P0) and well outside one at tolerance 0.0001 (ratio
        // 3.0, three cells away): comfortably clear of the grid-hash's cell-boundary rounding
        // ambiguity in either direction.
        let p: [SIMD3<Float>] = [
            SIMD3(0, 0, 0), SIMD3(0, 0, 0.0003), SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(1, 1, 1),
        ]
        let mesh = Mesh(vertices: p, indices: [0, 2, 3, 1, 2, 4])!

        let tight = mesh.welded(tolerance: 0.0001)
        #expect(tight.vertexCount == 5)
        #expect(tight.triangleCount == 2)

        let loose = mesh.welded(tolerance: 0.001)
        #expect(loose.vertexCount == 4)
        #expect(loose.triangleCount == 2)
    }

    @Test("Extreme coincident coordinates don't overflow the grid-hash key")
    func hugeCoincidentCoordinatesDoNotCrash() {
        // diag == 0 (every vertex identical) floors the cell size to 1e-9; combined with
        // coordinates of this magnitude, the naive (unclamped) grid coordinate overflows
        // Int64 on conversion. Reaching the assertion at all is the regression check.
        let huge: Float = 1e13
        let p: [SIMD3<Float>] = [SIMD3(huge, huge, huge), SIMD3(huge, huge, huge), SIMD3(huge, huge, huge)]
        let mesh = Mesh(vertices: p, indices: [0, 1, 2])!
        let welded = mesh.welded()
        #expect(welded.vertexCount >= 1)
    }
}

@Suite("Mesh — connectivity toolkit")
struct TopologyTests {
    @Test("faceNormals are unit length")
    func faceNormalsUnitLength() {
        let cube = weldedUnitCube()
        for n in cube.faceNormals() {
            #expect(abs(Double(simd_length(n)) - 1) < 1e-5)
        }
    }

    @Test("vertexNormals are unit length")
    func vertexNormalsUnitLength() {
        let cube = weldedUnitCube()
        for n in cube.vertexNormals() {
            #expect(abs(Double(simd_length(n)) - 1) < 1e-5)
        }
    }

    @Test("Each triangle of a closed welded box has exactly 3 adjacent triangles")
    func adjacencyOnWeldedBox() {
        let cube = weldedUnitCube()
        let adjacency = cube.triangleAdjacency()
        #expect(adjacency.count == 12)
        for neighbors in adjacency { #expect(neighbors.count == 3) }
    }

    @Test("Unwelded input has no adjacency — documents the weld precondition")
    func adjacencyRequiresWeld() {
        let cube = unweldedUnitCube()
        let adjacency = cube.triangleAdjacency()
        for neighbors in adjacency { #expect(neighbors.isEmpty) }
    }

    @Test("A closed welded box is one connected component")
    func connectedComponentsOfBox() {
        let cube = weldedUnitCube()
        let components = cube.connectedComponents()
        #expect(components.count == 1)
        #expect(components[0].triangleIndices.count == 12)
        #expect(components[0].area > 0)
    }

    @Test("Two disjoint boxes are two connected components, deterministically ordered")
    func connectedComponentsOfDisjointBoxes() {
        let mesh = disjointCubesMesh()
        let a = mesh.connectedComponents()
        let b = mesh.connectedComponents()
        #expect(a.count == 2)
        #expect(a[0].triangleIndices.count == 12)
        #expect(a[1].triangleIndices.count == 12)
        #expect(a.map(\.triangleIndices) == b.map(\.triangleIndices))
        // Tie-break: equal-size components ordered by lowest triangle index.
        #expect(a[0].triangleIndices.min() == 0)
        #expect(a[1].triangleIndices.min() == 12)
    }

    @Test("Unwelded input is every triangle its own component — documents the weld precondition")
    func componentsRequireWeld() {
        let cube = unweldedUnitCube()
        #expect(cube.connectedComponents().count == 12)
    }

    @Test("subMesh extracts a compact, self-contained mesh")
    func subMeshExtraction() throws {
        let cube = weldedUnitCube()
        let face = try #require(cube.subMesh(triangleIndices: [0, 1]))
        #expect(face.triangleCount == 2)
        #expect(face.vertexCount == 4)
        let bound = UInt32(face.vertexCount)
        #expect(face.indices.allSatisfy { $0 < bound })
    }

    @Test("subMesh returns nil for an empty triangle list")
    func subMeshEmpty() {
        let cube = weldedUnitCube()
        #expect(cube.subMesh(triangleIndices: []) == nil)
    }

    @Test("subMesh silently skips out-of-range triangle indices instead of crashing")
    func subMeshOutOfRangeIndices() throws {
        let cube = weldedUnitCube()
        let face = try #require(cube.subMesh(triangleIndices: [0, 100, -5]))
        #expect(face.triangleCount == 1)

        #expect(cube.subMesh(triangleIndices: [100, -5]) == nil)
    }

    @Test("An open cylindrical shell has one component and two boundary loops")
    func boundaryLoopsOfOpenShell() {
        let shell = openCylinderShellMesh(segments: 16)
        #expect(shell.connectedComponents().count == 1)
        let loops = shell.boundaryLoops()
        #expect(loops.count == 2)
        for loop in loops { #expect(loop.count == 16) }
    }

    @Test("A closed welded box has no boundary loops")
    func boundaryLoopsOfClosedBox() {
        #expect(weldedUnitCube().boundaryLoops().isEmpty)
    }
}

@Suite("Mesh.integrityReport — validity & quality snapshot")
struct IntegrityReportTests {
    @Test("A closed welded box is watertight, with the expected Euler characteristic")
    func closedBoxReport() {
        let report = weldedUnitCube().integrityReport()
        #expect(report.isWatertight)
        #expect(report.nonManifoldEdgeCount == 0)
        #expect(report.nonManifoldVertexCount == 0)
        #expect(report.boundaryLoopCount == 0)
        #expect(report.duplicateTriangleCount == 0)
        #expect(report.degenerateTriangleCount == 0)
        #expect(report.eulerCharacteristic == 2)
        #expect(report.components.count == 1)
        #expect(report.components[0].triangleCount == 12)
    }

    @Test("A closed, consistently-wound tetrahedron is watertight, orientable, genus 0")
    func orientableClosedSurfaceReport() {
        let report = orientableTetrahedronMesh().integrityReport()
        #expect(report.isWatertight)
        #expect(report.isOrientable)
        #expect(report.eulerCharacteristic == 2)
        #expect(report.genus == 0)
    }

    @Test("A dropped degenerate triangle's orphaned vertex doesn't corrupt Euler characteristic / genus")
    func orphanedVertexFromDegenerateTriangleDoesNotSkewEuler() {
        // Same closed tetrahedron as above (Euler 2, genus 0) plus an extra degenerate triangle
        // that references a fresh, otherwise-unused vertex. That vertex must NOT count toward V
        // once its only triangle is dropped as degenerate.
        let report = tetrahedronWithOrphanDegenerateTriangle().integrityReport()
        #expect(report.degenerateTriangleCount == 1)
        #expect(report.isWatertight)
        #expect(report.isOrientable)
        #expect(report.eulerCharacteristic == 2)
        #expect(report.genus == 0)
    }

    @Test("An open shell reports its two boundary loops and is not watertight")
    func openShellReport() {
        let report = openCylinderShellMesh(segments: 16).integrityReport()
        #expect(!report.isWatertight)
        #expect(report.boundaryLoopCount == 2)
        #expect(report.nonManifoldEdgeCount == 0)
    }

    @Test("An edge shared by three triangles is reported as non-manifold")
    func nonManifoldEdgeReport() {
        let report = nonManifoldEdgeFixture().integrityReport()
        #expect(report.nonManifoldEdgeCount == 1)
        #expect(!report.isWatertight)
    }

    @Test("A pinch-point vertex is reported as non-manifold")
    func nonManifoldVertexReport() {
        let report = bowtieVertexFixture().integrityReport()
        #expect(report.nonManifoldVertexCount == 1)
    }

    @Test("Duplicate and degenerate triangles are counted from the raw topology")
    func duplicateAndDegenerateReport() {
        let report = duplicateAndDegenerateFixture().integrityReport()
        #expect(report.duplicateTriangleCount == 1)
        #expect(report.degenerateTriangleCount == 1)
    }

    @Test("Sliver signals are finite and in a sane range for a right-angled box")
    func sliverSignalsSane() {
        let report = weldedUnitCube().integrityReport()
        #expect(report.minAngleDegrees.min > 0)
        #expect(report.minAngleDegrees.min <= 45 + 1e-6)
        #expect(report.minAngleDegrees.p05 >= report.minAngleDegrees.min)
        #expect(report.aspectRatio.max >= 1.0 - 1e-6)
        #expect(report.aspectRatio.max.isFinite)
        #expect(report.aspectRatio.p95 <= report.aspectRatio.max)
    }

    @Test("Repeated calls on the same mesh are byte-identical (determinism)")
    func determinism() {
        let mesh = coarseCappedCylinderMesh()
        let a = mesh.integrityReport()
        let b = mesh.integrityReport()
        #expect(a.isWatertight == b.isWatertight)
        #expect(a.isOrientable == b.isOrientable)
        #expect(a.nonManifoldEdgeCount == b.nonManifoldEdgeCount)
        #expect(a.nonManifoldVertexCount == b.nonManifoldVertexCount)
        #expect(a.boundaryLoopCount == b.boundaryLoopCount)
        #expect(a.eulerCharacteristic == b.eulerCharacteristic)
        #expect(a.genus == b.genus)
        #expect(a.components.map(\.triangleCount) == b.components.map(\.triangleCount))
    }
}
