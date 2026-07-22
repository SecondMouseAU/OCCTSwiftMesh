import Testing
import simd
import OCCTSwift
@testable import OCCTSwiftMesh

@Suite("Mesh.creaseEdges — dihedral-fold edge detection and ring/path chaining")
struct CreaseDetectionTests {

    @Test("A capped cylinder's smooth (below-threshold) barrel forms exactly two 90° closed crease rings — the cap/barrel seams")
    func cappedCylinderFormsTwoClosedRings() {
        // 48 sides → 7.5° barrel-facet dihedral, safely under the default 30° threshold (the
        // barrel itself never creases); cap-to-barrel is a clean 90° step all the way around,
        // regardless of subdivision, with no diagonal-triangulation ambiguity (fan-triangulated
        // caps sharing an exact boundary ring with the barrel, unlike a grid's corner diagonals).
        let mesh = coarseCappedCylinderMesh(radius: 4, sides: 48, rings: 2, height: 4)
        let result = mesh.creaseEdges()
        #expect(result.rings.count == 2)
        #expect(result.unchainedCreaseEdgeCount == 0)
        for ring in result.rings {
            #expect(ring.closed)
            #expect(ring.vertexIndices.count == 48)
            #expect(ring.length > 0)
            #expect(abs(ring.meanFoldAngleDegrees - 90) < 1e-2)
            #expect(abs(ring.maxFoldAngleDegrees - 90) < 1e-2)
        }
        // Largest-first by length: the outer base/collar ring encloses more area (and is longer)
        // than the inner collar/top ring.
        #expect(result.rings[0].length >= result.rings[1].length)
    }

    @Test("A high enough threshold finds no creases at all")
    func highThresholdFindsNothing() {
        let mesh = plateauMesh()
        let result = mesh.creaseEdges(minAngleDegrees: 89)
        #expect(result.rings.isEmpty)
        #expect(result.unchainedCreaseEdgeCount == 0)
    }

    @Test("A mesa touching the plate's own edge opens both rings into paths instead of loops")
    func plateauTouchingEdgeFormsOpenPaths() {
        let mesh = plateauTouchingEdgeMesh()
        let result = mesh.creaseEdges()
        #expect(!result.rings.isEmpty)
        #expect(result.unchainedCreaseEdgeCount == 0)
        for ring in result.rings { #expect(!ring.closed) }
    }

    @Test("A welded box's 8 corners are junctions — 12 single-edge open paths, no closed rings")
    func boxCornersAreJunctionsNotWandered() {
        // Every real cube edge is a 90° fold between two adjacent faces, and every cube corner
        // is where 3 such creases meet (a junction) — since a box edge isn't subdivided, each of
        // the 12 edges is itself a direct junction-to-junction path with no degree-2 vertex
        // in between, exercising the "never wander through a junction" discipline directly.
        let result = weldedUnitCube().creaseEdges()
        #expect(result.rings.count == 12)
        #expect(result.unchainedCreaseEdgeCount == 0)
        for ring in result.rings {
            #expect(!ring.closed)
            #expect(ring.vertexIndices.count == 2)
            #expect(abs(ring.meanFoldAngleDegrees - 90) < 1e-3)
        }
    }

    @Test("A flat mesh (no dihedral folds anywhere) reports no crease rings")
    func flatMeshHasNoCreases() {
        let result = flatGridMesh().creaseEdges()
        #expect(result.rings.isEmpty)
        #expect(result.unchainedCreaseEdgeCount == 0)
    }

    @Test("Unwelded input finds no creases — documents the weld precondition")
    func unweldedInputFindsNoCreases() {
        let result = unwelded(plateauMesh()).creaseEdges()
        #expect(result.rings.isEmpty)
    }

    @Test("An empty mesh returns an empty result rather than crashing")
    func emptyMeshIsHandled() {
        let empty = Mesh(vertices: [], indices: [])
        if let empty {
            let result = empty.creaseEdges()
            #expect(result.rings.isEmpty)
            #expect(result.unchainedCreaseEdgeCount == 0)
        }
    }

    @Test("Repeated calls on the same mesh are byte-identical (determinism)")
    func determinism() {
        let mesh = plateauMesh()
        let a = mesh.creaseEdges()
        let b = mesh.creaseEdges()
        #expect(a.rings.map(\.vertexIndices) == b.rings.map(\.vertexIndices))
        #expect(a.rings.map(\.closed) == b.rings.map(\.closed))
        #expect(a.rings.map(\.length) == b.rings.map(\.length))
        #expect(a.unchainedCreaseEdgeCount == b.unchainedCreaseEdgeCount)
    }
}
