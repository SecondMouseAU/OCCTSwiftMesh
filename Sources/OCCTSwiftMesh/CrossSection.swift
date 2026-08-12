// CrossSection.swift: planar slicing of a mesh into closed contours.
//
// This is the operation a 3D-printer slicer performs to find each layer's
// perimeters: intersect every triangle with a cut plane, chain the resulting
// segments end-to-end into closed loops, then classify them by nesting. For a
// hollow / thin-walled part the outer wall and inner wall come back as two
// SEPARATE, oppositely-wound, nested loops; and any pocket or window cut on
// the section is its own loop; so "which edge is inside vs outside" falls out
// of containment rather than guesswork.
//
// Pure geometry: works on `Mesh.vertices` / `Mesh.indices` with no OCCT kernel
// calls, so it is robust on the open and mildly non-manifold meshes that raw
// scan/STL bodies actually are (where sewing to a B-Rep first would fail).

import OCCTSwift
import simd

// MARK: - Plane

/// An oriented cut plane: a point on the plane and its normal.
///
/// The normal need not be unit length; `Mesh.crossSection` normalizes it.
public struct CutPlane: Sendable {
    /// A point that lies on the plane.
    public var point: SIMD3<Double>
    /// The plane normal (any non-zero length; normalized on use).
    public var normal: SIMD3<Double>

    public init(point: SIMD3<Double>, normal: SIMD3<Double>) {
        self.point = point
        self.normal = normal
    }

    /// A plane perpendicular to `axis`, positioned `offset` along it from the origin.
    public static func perpendicular(to axis: SIMD3<Double>, offset: Double) -> CutPlane {
        let n = simd_normalize(axis)
        return CutPlane(point: n * offset, normal: n)
    }
}

// MARK: - Contour

/// One closed loop of a cross-section, expressed in the plane's 2D `(u, v)` basis.
///
/// Loops are implicitly closed (the last point connects back to the first; the
/// first point is not repeated). `depth` / `isHole` come from nesting, so they
/// are reliable even on meshes whose triangle winding is inconsistent.
public struct MeshContour: Sendable {
    /// Ordered loop points in the section plane's `(u, v)` coordinates, millimetres.
    public var points: [SIMD2<Double>]

    /// Shoelace signed area in plane coordinates (CCW positive).
    ///
    /// After `crossSection` returns, orientation is normalized so an even `depth`
    /// (solid boundary) is CCW (> 0) and an odd `depth` (hole) is CW (< 0).
    public var signedArea: Double

    /// Nesting depth: 0 = outermost solid boundary, 1 = a hole inside it
    /// (inner wall / window), 2 = solid island inside that hole, etc.
    public var depth: Int

    /// Index (into the section's `contours`) of the immediately enclosing loop,
    /// or `nil` for an outermost loop.
    public var parent: Int?

    /// True when this loop bounds empty space inside a solid (odd nesting depth).
    public var isHole: Bool { depth % 2 == 1 }

    /// Unsigned enclosed area.
    public var area: Double { abs(signedArea) }

    /// Total perimeter length of the loop.
    public var perimeter: Double {
        guard points.count > 1 else { return 0 }
        var sum = 0.0
        for i in points.indices {
            let a = points[i]
            let b = points[(i + 1) % points.count]
            sum += simd_distance(a, b)
        }
        return sum
    }

    /// Axis-aligned bounds in plane coordinates: `(min, max)`.
    public var bounds: (min: SIMD2<Double>, max: SIMD2<Double>) {
        var lo = points.first ?? .zero
        var hi = points.first ?? .zero
        for p in points {
            lo = simd_min(lo, p)
            hi = simd_max(hi, p)
        }
        return (lo, hi)
    }
}

// MARK: - Cross-section

/// A planar cross-section of a mesh: the plane basis plus every closed loop the
/// plane cuts.
///
/// Map 2D loop points back to 3D with `worldPoint(_:)`.
public struct MeshCrossSection: Sendable {
    /// Plane origin (the `(u, v) = (0, 0)` point in world space).
    public var origin: SIMD3<Double>
    /// Unit plane normal.
    public var normal: SIMD3<Double>
    /// In-plane `u` basis (unit).
    public var uAxis: SIMD3<Double>
    /// In-plane `v` basis (unit, = normal × u).
    public var vAxis: SIMD3<Double>
    /// Closed loops, classified by nesting depth.
    public var contours: [MeshContour]
    /// Open polylines; produced where the plane exits the mesh through a
    /// boundary edge (e.g. cutting across an open end or through a window rim).
    ///
    /// Empty for a clean section between features.
    public var openPaths: [[SIMD2<Double>]]

    /// The outermost loops (nesting depth 0): the solid's outer boundary.
    public var outerContours: [MeshContour] { contours.filter { $0.depth == 0 } }

    /// Map a plane `(u, v)` coordinate back to a world-space point.
    public func worldPoint(_ uv: SIMD2<Double>) -> SIMD3<Double> {
        origin + uAxis * uv.x + vAxis * uv.y
    }
}

// MARK: - Slicing

extension Mesh {

    /// Slice the mesh with a plane and return the closed contours where the
    /// plane cuts the surface: a 3D-printer slicer's perimeter step.
    ///
    /// Intersection points are keyed by the mesh edge they lie on, so the two
    /// triangles sharing an edge weld exactly and loops chain without
    /// tolerance fuzz. Inner walls and pockets come back as separate loops,
    /// classified by nesting (`MeshContour.depth` / `isHole`).
    ///
    /// - Parameters:
    ///   - plane: the cut plane (normal need not be unit length).
    ///   - minLoopArea: loops with unsigned area below this are discarded as
    ///     slivers (default `0`, keep everything).
    ///   - weld: spatial tolerance for welding coincident intersection points
    ///     (handles unwelded STL). `0` (default) auto-derives `1e-6 ×` the
    ///     mesh's bounding-box diagonal.
    /// - Returns: the section, or `nil` if the plane misses the mesh entirely.
    public func crossSection(plane: CutPlane, minLoopArea: Double = 0, weld: Double = 0)
        -> MeshCrossSection?
    {
        let verts = vertices
        let idx = indices
        guard verts.count >= 3, idx.count >= 3 else { return nil }

        let n = simd_normalize(plane.normal)
        guard n.x.isFinite, simd_length(n) > 0.5 else { return nil }
        // Build an in-plane basis. Pick the world axis least aligned with n.
        let a = abs(n)
        let seed: SIMD3<Double> =
            (a.x <= a.y && a.x <= a.z)
            ? SIMD3(1, 0, 0)
            : (a.y <= a.z)
                ? SIMD3(0, 1, 0)
                : SIMD3(0, 0, 1)
        let u = simd_normalize(simd_cross(n, seed))
        let v = simd_cross(n, u)
        let origin = plane.point

        // Signed distance of every vertex to the plane.
        let nf = SIMD3<Float>(Float(n.x), Float(n.y), Float(n.z))
        let of = SIMD3<Float>(Float(origin.x), Float(origin.y), Float(origin.z))
        var dist = [Float](repeating: 0, count: verts.count)
        var lo = verts[0]
        var hi = verts[0]
        for i in verts.indices {
            dist[i] = simd_dot(verts[i] - of, nf)
            lo = simd_min(lo, verts[i])
            hi = simd_max(hi, verts[i])
        }
        let diag = Double(simd_length(hi - lo))

        // Treat exactly-on-plane vertices as just-positive so an edge with an
        // endpoint on the plane is handled consistently (no zero-length cuts).
        @inline(__always) func positive(_ d: Float) -> Bool { d >= 0 }

        // Crossing point on edge (va, vb), projected to (u, v). Keyed by its
        // QUANTIZED world position, not the vertex-pair: this welds coincident
        // crossings from adjacent triangles even when the mesh is unwelded (raw
        // STL repeats vertices) or when shared edges are traversed in opposite
        // order (1-ULP interpolation drift). The grid cell is tiny relative to
        // the model, far below any real wall thickness, so distinct points stay
        // distinct.
        let cell = weld > 0 ? weld : max(1e-9, 1e-6 * diag)
        struct GridKey: Hashable {
            var x: Int64
            var y: Int64
            var z: Int64
        }
        var pointForCell: [GridKey: Int] = [:]
        var pts: [SIMD2<Double>] = []
        func crossing(_ va: UInt32, _ vb: UInt32) -> Int {
            let da = dist[Int(va)]
            let db = dist[Int(vb)]
            let t = Double(da / (da - db))  // da, db straddle 0 here
            let pa = verts[Int(va)]
            let pb = verts[Int(vb)]
            let pad = SIMD3<Double>(Double(pa.x), Double(pa.y), Double(pa.z))
            let pbd = SIMD3<Double>(Double(pb.x), Double(pb.y), Double(pb.z))
            let w = pad + (pbd - pad) * t
            let key = GridKey(
                x: Int64((w.x / cell).rounded()),
                y: Int64((w.y / cell).rounded()),
                z: Int64((w.z / cell).rounded()))
            if let p = pointForCell[key] { return p }
            let rel = w - origin
            let uv = SIMD2(simd_dot(rel, u), simd_dot(rel, v))
            let pidx = pts.count
            pts.append(uv)
            pointForCell[key] = pidx
            return pidx
        }

        // Each triangle straddling the plane contributes one segment joining the
        // two crossed edges. Adjacency keyed by point index → exact chaining.
        var adjacency: [Int: [Int]] = [:]
        func addSeg(_ p: Int, _ q: Int) {
            guard p != q else { return }
            adjacency[p, default: []].append(q)
            adjacency[q, default: []].append(p)
        }

        var touched = false
        var tri = 0
        while tri + 2 < idx.count {
            let i0 = idx[tri]
            let i1 = idx[tri + 1]
            let i2 = idx[tri + 2]
            tri += 3
            let s0 = positive(dist[Int(i0)])
            let s1 = positive(dist[Int(i1)])
            let s2 = positive(dist[Int(i2)])
            if s0 == s1 && s1 == s2 { continue }  // entirely on one side
            touched = true
            // The two edges whose endpoints differ in sign are the crossed ones.
            var crossPts: [Int] = []
            if s0 != s1 { crossPts.append(crossing(i0, i1)) }
            if s1 != s2 { crossPts.append(crossing(i1, i2)) }
            if s2 != s0 { crossPts.append(crossing(i2, i0)) }
            if crossPts.count == 2 { addSeg(crossPts[0], crossPts[1]) }
        }
        guard touched else { return nil }

        // Walk the segment graph into loops (closed) and paths (open ends).
        var visitedSeg = Set<UInt64>()
        @inline(__always) func segKey(_ p: Int, _ q: Int) -> UInt64 {
            let lo = UInt64(min(p, q))
            let hi = UInt64(max(p, q))
            return (hi << 32) | lo
        }
        var loops: [[Int]] = []
        var openPaths: [[Int]] = []

        // First, trace open chains starting from degree-1 nodes (boundary cuts).
        func neighbors(_ p: Int) -> [Int] { adjacency[p] ?? [] }
        func walk(from start: Int, preferOpen: Bool) {
            // Greedy walk consuming unused segments.
            var path = [start]
            var current = start
            while true {
                let nbrs = neighbors(current)
                var next: Int? = nil
                for cand in nbrs where !visitedSeg.contains(segKey(current, cand)) {
                    next = cand
                    break
                }
                guard let nxt = next else { break }
                visitedSeg.insert(segKey(current, nxt))
                path.append(nxt)
                current = nxt
                if nxt == start { break }  // closed
            }
            if path.count >= 2 {
                if path.first == path.last && path.count >= 4 {
                    loops.append(Array(path.dropLast()))
                } else {
                    openPaths.append(path)
                }
            }
        }

        let degree1 = adjacency.keys.filter { (adjacency[$0]?.count ?? 0) == 1 }.sorted()
        for s in degree1 {
            if neighbors(s).contains(where: { !visitedSeg.contains(segKey(s, $0)) }) {
                walk(from: s, preferOpen: true)
            }
        }
        // Remaining: closed loops. Seed deterministically by lowest point index.
        for s in adjacency.keys.sorted() {
            while neighbors(s).contains(where: { !visitedSeg.contains(segKey(s, $0)) }) {
                walk(from: s, preferOpen: false)
            }
        }

        // Build contours with signed area.
        func shoelace(_ ring: [SIMD2<Double>]) -> Double {
            var s = 0.0
            for i in ring.indices {
                let a = ring[i]
                let b = ring[(i + 1) % ring.count]
                s += a.x * b.y - b.x * a.y
            }
            return s * 0.5
        }
        var contours: [MeshContour] = []
        for loop in loops {
            let ring = loop.map { pts[$0] }
            let area = shoelace(ring)
            guard abs(area) >= minLoopArea else { continue }
            contours.append(MeshContour(points: ring, signedArea: area, depth: 0, parent: nil))
        }

        // Nesting: depth = how many other loops contain a loop's sample point;
        // parent = the smallest-area loop that contains it.
        func contains(_ ring: [SIMD2<Double>], _ p: SIMD2<Double>) -> Bool {
            var inside = false
            var j = ring.count - 1
            for i in ring.indices {
                let a = ring[i]
                let b = ring[j]
                if (a.y > p.y) != (b.y > p.y) {
                    let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x)
                    if p.x < x { inside.toggle() }
                }
                j = i
            }
            return inside
        }
        for i in contours.indices {
            // Test a BOUNDARY VERTEX of loop i, not its centroid: the outer
            // wall's centroid lies inside the inner wall, but a point ON the
            // outer wall does not. Since section loops never cross, every vertex
            // of loop i is consistently inside-or-outside any other loop k.
            let rep = contours[i].points[0]
            var depth = 0
            var parent: Int? = nil
            var bestParentArea = Double.infinity
            for k in contours.indices where k != i {
                if contains(contours[k].points, rep) {
                    depth += 1
                    if contours[k].area < bestParentArea {
                        bestParentArea = contours[k].area
                        parent = k
                    }
                }
            }
            contours[i].depth = depth
            contours[i].parent = parent
        }

        // Normalize orientation: even depth → CCW (area > 0), odd → CW (area < 0).
        for i in contours.indices {
            let wantCCW = contours[i].depth % 2 == 0
            let isCCW = contours[i].signedArea > 0
            if wantCCW != isCCW {
                contours[i].points.reverse()
                contours[i].signedArea = -contours[i].signedArea
            }
        }

        let openUV = openPaths.map { $0.map { pts[$0] } }
        return MeshCrossSection(
            origin: origin, normal: n, uAxis: u, vAxis: v,
            contours: contours, openPaths: openUV)
    }

    /// A stack of evenly-spaced cross-sections along an axis: a slicer's layer stack.
    ///
    /// Sections that miss the mesh are skipped.
    ///
    /// - Parameters:
    ///   - axis: slicing direction (normalized internally).
    ///   - point: a point the axis passes through (the stack is measured from here).
    ///   - spacing: distance between successive section planes.
    ///   - margin: inset from each end of the mesh's extent along the axis, so the
    ///     first/last slice doesn't sit exactly on an end cap (default `spacing/2`).
    /// - Returns: one `MeshCrossSection` per slice plane that intersects the mesh, ordered
    ///   along the axis.
    public func crossSections(
        axis: SIMD3<Double>, through point: SIMD3<Double>,
        spacing: Double, margin: Double? = nil
    ) -> [MeshCrossSection] {
        guard spacing > 0 else { return [] }
        let n = simd_normalize(axis)
        let nf = SIMD3<Float>(Float(n.x), Float(n.y), Float(n.z))
        let pf = SIMD3<Float>(Float(point.x), Float(point.y), Float(point.z))
        let verts = vertices
        guard !verts.isEmpty else { return [] }
        var lo = Float.greatestFiniteMagnitude
        var hi = -Float.greatestFiniteMagnitude
        for vtx in verts {
            let d = simd_dot(vtx - pf, nf)
            lo = min(lo, d)
            hi = max(hi, d)
        }
        let m = margin ?? (spacing / 2)
        var t = Double(lo) + m
        let end = Double(hi) - m
        var out: [MeshCrossSection] = []
        while t <= end {
            if let s = crossSection(plane: CutPlane(point: point + n * t, normal: n)) {
                out.append(s)
            }
            t += spacing
        }
        return out
    }
}
