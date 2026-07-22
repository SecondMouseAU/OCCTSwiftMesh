import simd
import OCCTSwift
@testable import OCCTSwiftMesh

// Shared synthetic-mesh builders for the mesh-foundations and segmentation test suites.
// All pure geometry — no OCCT tessellation needed, mirroring CrossSectionTests' approach.

/// A unit cube, corner at the origin, vertices SHARED across faces (8 vertices, 12 triangles) —
/// already welded by construction.
func weldedUnitCube() -> Mesh {
    let p: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(1, 1, 0), SIMD3(0, 1, 0),
        SIMD3(0, 0, 1), SIMD3(1, 0, 1), SIMD3(1, 1, 1), SIMD3(0, 1, 1),
    ]
    let faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
    var indices: [UInt32] = []
    for f in faces {
        indices.append(contentsOf: [UInt32(f[0]), UInt32(f[1]), UInt32(f[2])])
        indices.append(contentsOf: [UInt32(f[0]), UInt32(f[2]), UInt32(f[3])])
    }
    return Mesh(vertices: p, indices: indices)!
}

/// The same unit cube, but EVERY triangle gets its own 3 fresh vertices (36 vertices total, 12
/// triangles, no two triangles sharing an index anywhere) — the fully-unshared "soup" that raw
/// OCCT tessellation / STL import actually produces. Geometrically identical to
/// `weldedUnitCube()`; welding should collapse it back to 8 vertices.
func unweldedUnitCube() -> Mesh {
    let corners: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(1, 1, 0), SIMD3(0, 1, 0),
        SIMD3(0, 0, 1), SIMD3(1, 0, 1), SIMD3(1, 1, 1), SIMD3(0, 1, 1),
    ]
    let faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
    var positions: [SIMD3<Float>] = []
    var indices: [UInt32] = []
    for f in faces {
        let quad = f.map { corners[$0] }
        for tri in [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]] {
            let base = UInt32(positions.count)
            positions.append(contentsOf: tri)
            indices.append(contentsOf: [base, base + 1, base + 2])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// Two unit cubes far apart, each welded internally but sharing no vertices with the other —
/// two connected components.
func disjointCubesMesh() -> Mesh {
    let a = weldedUnitCube()
    let b = weldedUnitCube()
    let offset = SIMD3<Float>(100, 100, 100)
    let positions = a.vertices + b.vertices.map { $0 + offset }
    let bBase = UInt32(a.vertices.count)
    let indices = a.indices + b.indices.map { $0 + bBase }
    return Mesh(vertices: positions, indices: indices)!
}

/// An open cylindrical tube (barrel only, no end caps), welded by construction: shared ring
/// vertices at every angular step. One connected component; boundary is exactly the top and
/// bottom rim — 2 boundary loops.
func openCylinderShellMesh(radius: Float = 3, height: Float = 5, segments: Int = 16) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for k in 0..<2 {
        let z = Float(k) * height
        for i in 0..<segments {
            let a = Float(i) / Float(segments) * 2 * .pi
            positions.append(SIMD3(radius * cos(a), radius * sin(a), z))
        }
    }
    var indices: [UInt32] = []
    let lo: UInt32 = 0, hi = UInt32(segments)
    for i in 0..<segments {
        let a = lo + UInt32(i), b = lo + UInt32((i + 1) % segments)
        let c = hi + UInt32(i), d = hi + UInt32((i + 1) % segments)
        indices.append(contentsOf: [a, b, c, b, d, c])
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A coarse capped cylinder: `sides`-sided barrel (adjacent facets 360/sides° apart — coarse
/// enough to shatter under a 20° dihedral threshold) plus flat top/bottom disk caps. Welded by
/// construction. Segmentation should merge the shattered barrel facets back into one cylinder
/// region, while the caps (already-coplanar fans) stay separate — 3 regions total.
func coarseCappedCylinderMesh(radius: Float = 4, sides: Int = 12, rings: Int = 5, height: Float = 4) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for k in 0..<rings {
        let z = Float(k) / Float(rings - 1) * height
        for i in 0..<sides {
            let a = Float(i) / Float(sides) * 2 * .pi
            positions.append(SIMD3(radius * cos(a), radius * sin(a), z))
        }
    }
    let bottomCenter = UInt32(positions.count); positions.append(SIMD3(0, 0, 0))
    let topCenter = UInt32(positions.count); positions.append(SIMD3(0, 0, height))

    var indices: [UInt32] = []
    for k in 0..<(rings - 1) {
        let lo = UInt32(k * sides), hi = UInt32((k + 1) * sides)
        for i in 0..<sides {
            let a = lo + UInt32(i), b = lo + UInt32((i + 1) % sides)
            let c = hi + UInt32(i), d = hi + UInt32((i + 1) % sides)
            indices.append(contentsOf: [a, b, c, b, d, c])
        }
    }
    for i in 0..<sides {
        let a = UInt32(i), b = UInt32((i + 1) % sides)
        indices.append(contentsOf: [bottomCenter, b, a])
    }
    let topRingBase = UInt32((rings - 1) * sides)
    for i in 0..<sides {
        let a = topRingBase + UInt32(i), b = topRingBase + UInt32((i + 1) % sides)
        indices.append(contentsOf: [topCenter, a, b])
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A closed, orientable tetrahedron plus one extra degenerate triangle that references a fresh,
/// otherwise-unused vertex — that vertex becomes an orphan once the degenerate triangle is
/// dropped during `integrityReport()`'s cleanup pass.
func tetrahedronWithOrphanDegenerateTriangle() -> Mesh {
    let base = orientableTetrahedronMesh()
    var positions = base.vertices
    positions.append(SIMD3(5, 5, 5))
    var indices = base.indices
    indices.append(contentsOf: [0, 0, UInt32(positions.count - 1)])   // degenerate: repeats vertex 0
    return Mesh(vertices: positions, indices: indices)!
}

/// Three triangles sharing one edge (a "book") — a non-manifold EDGE (valence 3), no
/// non-manifold vertex.
func nonManifoldEdgeFixture() -> Mesh {
    let p: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0),   // shared edge (0, 1)
        SIMD3(0, 1, 0), SIMD3(0, -1, 0), SIMD3(0, 0, 1),
    ]
    let indices: [UInt32] = [0, 1, 2, 1, 0, 3, 0, 1, 4]
    return Mesh(vertices: p, indices: indices)!
}

/// Two triangles sharing exactly one vertex (a "bowtie" pinch point) — a non-manifold VERTEX,
/// no non-manifold edge.
func bowtieVertexFixture() -> Mesh {
    let p: [SIMD3<Float>] = [
        SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0),   // triangle A, shares vertex index 2
        SIMD3(0, 1, 1), SIMD3(1, 1, 1),                   // triangle B
    ]
    let indices: [UInt32] = [0, 1, 2, 2, 3, 4]
    return Mesh(vertices: p, indices: indices)!
}

/// A closed, watertight tetrahedron with hand-verified CONSISTENT outward winding (every shared
/// edge is traversed in opposite directions by its two triangles) — for asserting
/// `isOrientable`/`genus`, which need a known-consistent fixture rather than an
/// orientation-agnostic one.
func orientableTetrahedronMesh() -> Mesh {
    let p: [SIMD3<Float>] = [SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 1)]
    let indices: [UInt32] = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3]
    return Mesh(vertices: p, indices: indices)!
}

/// One valid triangle, an exact duplicate of it, and a triangle that degenerates to a repeated
/// vertex once vertex 3 (coincident with vertex 0) is welded.
func duplicateAndDegenerateFixture() -> Mesh {
    let p: [SIMD3<Float>] = [SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 0)]
    let indices: [UInt32] = [0, 1, 2, 0, 1, 2, 0, 3, 1]
    return Mesh(vertices: p, indices: indices)!
}

/// Two closed, watertight, orientable tetrahedra pinched together at exactly one shared vertex
/// (a bowtie of closed shells, not just two open triangles) — every welded edge is still
/// shared by exactly two triangles (no boundary, no non-manifold edge), so an edge-manifold-only
/// watertight check reports this watertight; the shared apex is a non-manifold VERTEX (two
/// disjoint triangle fans meeting at one point), which is the case `isWatertight` must catch.
func bowtiePinchedClosedShellsFixture() -> Mesh {
    let p: [SIMD3<Float>] = [
        SIMD3(0, 0, 0),                                            // 0: shared apex
        SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 1),             // 1-3: shell A's other verts
        SIMD3(10, 0, 0), SIMD3(0, 10, 0), SIMD3(0, 0, 10),          // 4-6: shell B's other verts
    ]
    let a: [UInt32] = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3]
    let b: [UInt32] = [0, 5, 4, 0, 4, 6, 0, 6, 5, 4, 5, 6]
    return Mesh(vertices: p, indices: a + b)!
}

/// The same tetrahedron as `orientableTetrahedronMesh()`, but one face's winding is flipped —
/// a shared edge is now traversed in the SAME direction by both its triangles instead of
/// opposite directions, breaking the consistent orientation `isOrientable` checks for.
func nonOrientableTetrahedronMesh() -> Mesh {
    let p: [SIMD3<Float>] = [SIMD3(0, 0, 0), SIMD3(1, 0, 0), SIMD3(0, 1, 0), SIMD3(0, 0, 1)]
    let indices: [UInt32] = [0, 2, 1, 0, 1, 3, 0, 2, 3, 1, 2, 3]   // 3rd face flipped: 0,3,2 -> 0,2,3
    return Mesh(vertices: p, indices: indices)!
}

/// Duplicates every triangle's vertices into fresh, per-triangle-unique slots — the fully-
/// unshared "soup" form raw OCCT tessellation / STL import produces. Geometrically identical to
/// `mesh`; welding should collapse it back. A generic counterpart to `unweldedUnitCube()` for
/// meshes built elsewhere (e.g. `coarseCappedCylinderMesh()`), for exercising the internal-weld
/// path on a curved body, not just a box.
func unwelded(_ mesh: Mesh) -> Mesh {
    let verts = mesh.vertices
    let idx = mesh.indices
    var positions: [SIMD3<Float>] = []
    var indices: [UInt32] = []
    positions.reserveCapacity(idx.count)
    indices.reserveCapacity(idx.count)
    for i in idx {
        indices.append(UInt32(positions.count))
        positions.append(verts[Int(i)])
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A closed, watertight, orientable torus (genus 1) — a structured (major × minor) grid with
/// both directions wrapped, triangulated with a uniform diagonal split per quad so winding stays
/// consistent everywhere. Welded by construction (shared ring vertices).
func torusMesh(majorRadius R: Float = 5, minorRadius r: Float = 1.5,
               majorSegments: Int = 12, minorSegments: Int = 8) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for i in 0..<majorSegments {
        let u = Float(i) / Float(majorSegments) * 2 * .pi
        for j in 0..<minorSegments {
            let v = Float(j) / Float(minorSegments) * 2 * .pi
            let x = (R + r * cos(v)) * cos(u)
            let y = (R + r * cos(v)) * sin(u)
            let z = r * sin(v)
            positions.append(SIMD3(x, y, z))
        }
    }
    func vertexIndex(_ i: Int, _ j: Int) -> UInt32 {
        UInt32(((i + majorSegments) % majorSegments) * minorSegments + ((j + minorSegments) % minorSegments))
    }
    var indices: [UInt32] = []
    for i in 0..<majorSegments {
        for j in 0..<minorSegments {
            let a = vertexIndex(i, j), b = vertexIndex(i + 1, j)
            let c = vertexIndex(i, j + 1), d = vertexIndex(i + 1, j + 1)
            indices.append(contentsOf: [a, b, d, a, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A shallow cylindrical arc strip — the kiha40-roof scenario from issue #20 item 4: a big
/// radius (default 500) swept over a small angular span, so the surface deviates from its
/// best-fit plane by only a modest sagitta relative to the strip's own footprint. Welded by
/// construction (shared ring vertices), one connected patch.
func shallowCylindricalArcMesh(radius: Float = 500, widthUnits: Float = 200, axialUnits: Float = 200,
                               sides: Int = 10, rings: Int = 6) -> Mesh {
    let angularSpan = widthUnits / radius
    var positions: [SIMD3<Float>] = []
    for k in 0..<rings {
        let z = Float(k) / Float(rings - 1) * axialUnits
        for i in 0..<sides {
            let a = Float(i) / Float(sides - 1) * angularSpan
            positions.append(SIMD3(radius * cos(a), radius * sin(a), z))
        }
    }
    var indices: [UInt32] = []
    for k in 0..<(rings - 1) {
        let lo = UInt32(k * sides), hi = UInt32((k + 1) * sides)
        for i in 0..<(sides - 1) {
            let a = lo + UInt32(i), b = lo + UInt32(i + 1)
            let c = hi + UInt32(i), d = hi + UInt32(i + 1)
            indices.append(contentsOf: [a, b, c, b, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// An open cylindrical shell (no end caps) with multiple axial rings — unlike
/// `openCylinderShellMesh`'s fixed 2 rings (every vertex a boundary vertex), interior rings here
/// are away from the top/bottom open edges and get a complete triangle fan. Welded by
/// construction (shared ring vertices); axis is +Z.
func openCylinderMultiRingMesh(radius: Float = 6, height: Float = 20, segments: Int = 24, rings: Int = 8) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for k in 0..<rings {
        let z = Float(k) / Float(rings - 1) * height
        for i in 0..<segments {
            let a = Float(i) / Float(segments) * 2 * .pi
            positions.append(SIMD3(radius * cos(a), radius * sin(a), z))
        }
    }
    var indices: [UInt32] = []
    for k in 0..<(rings - 1) {
        let lo = UInt32(k * segments), hi = UInt32((k + 1) * segments)
        for i in 0..<segments {
            let a = lo + UInt32(i), b = lo + UInt32((i + 1) % segments)
            let c = hi + UInt32(i), d = hi + UInt32((i + 1) % segments)
            indices.append(contentsOf: [a, b, c, b, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// An open "zone" cut from a sphere (a latitude band, no pole caps) — wrapped in longitude, open
/// top/bottom in latitude. Avoids the pole singularity a full UV-sphere would introduce, so every
/// interior vertex has the same well-defined analytic curvature (`k1 == k2 == 1/radius`). Welded
/// by construction (shared ring vertices in longitude).
func sphereZoneMesh(radius: Float = 10, latitudeSpanDegrees: Float = 60, segments: Int = 24, rings: Int = 10) -> Mesh {
    var positions: [SIMD3<Float>] = []
    let halfSpan = latitudeSpanDegrees * .pi / 180 / 2
    for k in 0..<rings {
        let lat = -halfSpan + Float(k) / Float(rings - 1) * (2 * halfSpan)
        for i in 0..<segments {
            let lon = Float(i) / Float(segments) * 2 * .pi
            let x = radius * cos(lat) * cos(lon)
            let y = radius * cos(lat) * sin(lon)
            let z = radius * sin(lat)
            positions.append(SIMD3(x, y, z))
        }
    }
    var indices: [UInt32] = []
    for k in 0..<(rings - 1) {
        let lo = UInt32(k * segments), hi = UInt32((k + 1) * segments)
        for i in 0..<segments {
            let a = lo + UInt32(i), b = lo + UInt32((i + 1) % segments)
            let c = hi + UInt32(i), d = hi + UInt32((i + 1) % segments)
            indices.append(contentsOf: [a, b, c, b, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// `sphereZoneMesh()` with one extra degenerate SLIVER triangle glued onto the edge between
/// vertices 0 and 1 (a needle: two shared existing vertices plus one new vertex placed almost
/// exactly on the line between them) — for exercising `vertexCurvatures()`'s sliver-exclusion
/// guard without perturbing the rest of the mesh's analytic curvature.
func sphereZoneMeshWithSliver(radius: Float = 10, latitudeSpanDegrees: Float = 60, segments: Int = 24, rings: Int = 10) -> Mesh {
    let base = sphereZoneMesh(radius: radius, latitudeSpanDegrees: latitudeSpanDegrees, segments: segments, rings: rings)
    var positions = base.vertices
    var indices = base.indices
    let a = positions[0], b = positions[1]
    let ab = b - a
    let helper: SIMD3<Float> = abs(ab.x) < abs(ab.y) ? SIMD3(1, 0, 0) : SIMD3(0, 1, 0)
    let perpDir = simd_normalize(simd_cross(ab, helper))
    let sliverApex = (a + b) * 0.5 + perpDir * (simd_length(ab) * 1e-5)
    let newIndex = UInt32(positions.count)
    positions.append(sliverApex)
    indices.append(contentsOf: [0, 1, newIndex])
    return Mesh(vertices: positions, indices: indices)!
}

/// A flat rectangular grid in the XY plane (`z == 0`) — the trivial curvature case:
/// `k1 == k2 == 0` everywhere, including at boundary vertices (flat is flat regardless of an
/// incomplete triangle fan). Welded by construction (shared grid vertices).
func flatGridMesh(width: Float = 20, depth: Float = 20, segmentsX: Int = 10, segmentsY: Int = 10) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for j in 0...segmentsY {
        let y = Float(j) / Float(segmentsY) * depth
        for i in 0...segmentsX {
            let x = Float(i) / Float(segmentsX) * width
            positions.append(SIMD3(x, y, 0))
        }
    }
    let cols = segmentsX + 1
    var indices: [UInt32] = []
    for j in 0..<segmentsY {
        for i in 0..<segmentsX {
            let a = UInt32(j * cols + i), b = UInt32(j * cols + i + 1)
            let c = UInt32((j + 1) * cols + i), d = UInt32((j + 1) * cols + i + 1)
            indices.append(contentsOf: [a, b, d, a, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A genuinely 3D-shaped ("bumpy terrain") rectangular patch: `z = amplitude · sin(x·freq) ·
/// cos(y·freq·0.7)` over `xRange`/`yRange`, sampled on a regular grid — real curvature/asymmetry
/// everywhere (no flat or rotationally-symmetric regions to confuse ICP correspondence), and
/// GLOBALLY CONSISTENT world coordinates: two patches built from overlapping ranges genuinely
/// overlap in world space (same `z = f(x, y)` everywhere), for partial-overlap alignment tests.
/// Welded by construction (shared grid vertices).
func bumpyPatchMesh(xRange: ClosedRange<Float>, yRange: ClosedRange<Float> = 0...20,
                    segmentsPerUnit: Float = 1, amplitude: Float = 3, frequency: Float = 0.3) -> Mesh {
    let segmentsX = max(2, Int((xRange.upperBound - xRange.lowerBound) * segmentsPerUnit))
    let segmentsY = max(2, Int((yRange.upperBound - yRange.lowerBound) * segmentsPerUnit))
    var positions: [SIMD3<Float>] = []
    for j in 0...segmentsY {
        let y = yRange.lowerBound + Float(j) / Float(segmentsY) * (yRange.upperBound - yRange.lowerBound)
        for i in 0...segmentsX {
            let x = xRange.lowerBound + Float(i) / Float(segmentsX) * (xRange.upperBound - xRange.lowerBound)
            let z = amplitude * sin(x * frequency) * cos(y * frequency * 0.7)
            positions.append(SIMD3(x, y, z))
        }
    }
    let cols = segmentsX + 1
    var indices: [UInt32] = []
    for j in 0..<segmentsY {
        for i in 0..<segmentsX {
            let a = UInt32(j * cols + i), b = UInt32(j * cols + i + 1)
            let c = UInt32((j + 1) * cols + i), d = UInt32((j + 1) * cols + i + 1)
            indices.append(contentsOf: [a, b, d, a, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A mostly-flat rectangular grid (`z ≈ 0` almost everywhere) with a single localized Gaussian
/// bump — the normal-space-sampling motivating case: the bump's tilted-normal vertices are a
/// small minority of the mesh, everywhere else the normal is uniformly `(0, 0, 1)`. One regular
/// grid (no stitching), welded by construction (shared grid vertices).
func flatWithBumpMesh(size: Float = 40, segments: Int = 40, bumpCenter: SIMD2<Float> = SIMD2(20, 20),
                      bumpSigma: Float = 1.5, bumpHeight: Float = 4) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for j in 0...segments {
        let y = Float(j) / Float(segments) * size
        for i in 0...segments {
            let x = Float(i) / Float(segments) * size
            let dx = x - bumpCenter.x, dy = y - bumpCenter.y
            let z = bumpHeight * exp(-(dx * dx + dy * dy) / (2 * bumpSigma * bumpSigma))
            positions.append(SIMD3(x, y, z))
        }
    }
    let cols = segments + 1
    var indices: [UInt32] = []
    for j in 0..<segments {
        for i in 0..<segments {
            let a = UInt32(j * cols + i), b = UInt32(j * cols + i + 1)
            let c = UInt32((j + 1) * cols + i), d = UInt32((j + 1) * cols + i + 1)
            indices.append(contentsOf: [a, b, d, a, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A UV sphere (lat/long grid, poles collapsed to single vertices) — welded by construction.
func sphereMesh(radius: Float = 5, latSegments: Int = 12, lonSegments: Int = 16) -> Mesh {
    var positions: [SIMD3<Float>] = []
    let northPole = UInt32(0); positions.append(SIMD3(0, 0, radius))
    for i in 1..<latSegments {
        let theta = Float(i) / Float(latSegments) * .pi   // 0 (north) ... pi (south)
        let z = radius * cos(theta), r = radius * sin(theta)
        for j in 0..<lonSegments {
            let phi = Float(j) / Float(lonSegments) * 2 * .pi
            positions.append(SIMD3(r * cos(phi), r * sin(phi), z))
        }
    }
    let southPole = UInt32(positions.count); positions.append(SIMD3(0, 0, -radius))

    func ring(_ i: Int, _ j: Int) -> UInt32 { UInt32(1 + (i - 1) * lonSegments + (j % lonSegments)) }

    var indices: [UInt32] = []
    for j in 0..<lonSegments {
        indices.append(contentsOf: [northPole, ring(1, j), ring(1, j + 1)])
    }
    for i in 1..<(latSegments - 1) {
        for j in 0..<lonSegments {
            let a = ring(i, j), b = ring(i, j + 1), c = ring(i + 1, j), d = ring(i + 1, j + 1)
            indices.append(contentsOf: [a, b, d, a, d, c])
        }
    }
    for j in 0..<lonSegments {
        indices.append(contentsOf: [southPole, ring(latSegments - 1, j + 1), ring(latSegments - 1, j)])
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// The lateral (side) surface only of a cone, as a multi-ring stack tapering to the apex (NOT a
/// single fan from one apex vertex — a fan only samples 2 distinct heights (apex + base rim),
/// too degenerate a point set to expose the surface's true 1-parameter rotational symmetry
/// without spurious extra ones). A genuine surface of revolution, distinct from a cylinder
/// because its radius varies along the axis. Welded by construction (shared ring vertices).
func coneLateralMesh(baseRadius: Float = 4, height: Float = 10, segments: Int = 16, rings: Int = 8) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for k in 0..<rings {
        let t = Float(k) / Float(rings - 1)   // 0 at base, 1 near the apex
        let z = t * height
        let r = baseRadius * (1 - t)
        for i in 0..<segments {
            let a = Float(i) / Float(segments) * 2 * .pi
            positions.append(SIMD3(r * cos(a), r * sin(a), z))
        }
    }
    var indices: [UInt32] = []
    for k in 0..<(rings - 1) {
        let lo = UInt32(k * segments), hi = UInt32((k + 1) * segments)
        for i in 0..<segments {
            let a = lo + UInt32(i), b = lo + UInt32((i + 1) % segments)
            let c = hi + UInt32(i), d = hi + UInt32((i + 1) % segments)
            indices.append(contentsOf: [a, b, c, b, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// The lateral surface of a triangular prism — flat quad faces only (no end caps), extruded
/// along +Z. A non-circular cross-section rules out any rotational symmetry, leaving pure
/// translation as the only slippable motion. Welded by construction.
func triangularPrismLateralMesh(height: Float = 20) -> Mesh {
    let base: [SIMD2<Float>] = [SIMD2(0, 0), SIMD2(4, 0), SIMD2(1, 3)]
    var positions: [SIMD3<Float>] = []
    for z in [Float(0), height] {
        for p in base { positions.append(SIMD3(p.x, p.y, z)) }
    }
    var indices: [UInt32] = []
    for i in 0..<3 {
        let a = UInt32(i), b = UInt32((i + 1) % 3)
        let c = a + 3, d = b + 3
        indices.append(contentsOf: [a, b, d, a, d, c])
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// A helicoid strip: `(v·cos(u), v·sin(u), pitch·u / 2π)` for `v ∈ [innerRadius, outerRadius]`,
/// `u` over several full turns — the textbook screw-symmetric ruled surface (rotating by du about
/// Z while translating by `pitch·du/2π` along Z maps the surface exactly onto itself). Welded by
/// construction (shared grid vertices).
func helicoidStripMesh(innerRadius: Float = 2, outerRadius: Float = 5, pitch: Float = 6,
                       turns: Float = 3, uSegments: Int = 96, vSegments: Int = 6) -> Mesh {
    var positions: [SIMD3<Float>] = []
    for i in 0...uSegments {
        let u = Float(i) / Float(uSegments) * turns * 2 * .pi
        let z = pitch * u / (2 * .pi)
        for j in 0...vSegments {
            let v = innerRadius + Float(j) / Float(vSegments) * (outerRadius - innerRadius)
            positions.append(SIMD3(v * cos(u), v * sin(u), z))
        }
    }
    let cols = vSegments + 1
    var indices: [UInt32] = []
    for i in 0..<uSegments {
        for j in 0..<vSegments {
            let a = UInt32(i * cols + j), b = UInt32(i * cols + j + 1)
            let c = UInt32((i + 1) * cols + j), d = UInt32((i + 1) * cols + j + 1)
            indices.append(contentsOf: [a, b, d, a, d, c])
        }
    }
    return Mesh(vertices: positions, indices: indices)!
}

/// Applies a rigid transform to every vertex of `mesh`, keeping its indices unchanged.
func transformedMesh(_ mesh: Mesh, by transform: simd_double4x4) -> Mesh {
    let newPositions = mesh.vertices.map { v -> SIMD3<Float> in
        SIMD3<Float>(Mesh.apply(transform, SIMD3<Double>(v)))
    }
    return Mesh(vertices: newPositions, indices: mesh.indices)!
}
