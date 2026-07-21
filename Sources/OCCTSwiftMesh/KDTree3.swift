// KDTree3.swift — a minimal static 3D k-d tree for nearest-neighbor correspondence search.
// Internal implementation detail behind Mesh.aligned(to:options:); not part of the public API.

import simd

struct KDTree3 {
    private struct Node {
        var index: Int
        var axis: Int
        var left: Int
        var right: Int
    }

    private let points: [SIMD3<Double>]
    private var nodes: [Node] = []
    private var root: Int = -1

    /// Builds a balanced tree by recursive median split, alternating axis by depth. DETERMINISM:
    /// the per-level sort ties-break on point index, so identical input always builds the same
    /// tree — a plain coordinate sort alone would leave coincident-coordinate ties to whatever
    /// order the input happened to arrive in (harmless for query correctness, but would leak
    /// into any tie-broken downstream consumer).
    init(points: [SIMD3<Double>]) {
        self.points = points
        guard !points.isEmpty else { return }
        var indices = Array(0..<points.count)
        nodes.reserveCapacity(points.count)
        root = Self.build(indices: &indices, lo: 0, hi: indices.count, depth: 0, points: points, nodes: &nodes)
    }

    private static func build(indices: inout [Int], lo: Int, hi: Int, depth: Int,
                              points: [SIMD3<Double>], nodes: inout [Node]) -> Int {
        guard lo < hi else { return -1 }
        let axis = depth % 3
        let slice = indices[lo..<hi].sorted { a, b in
            let pa = points[a][axis], pb = points[b][axis]
            return pa != pb ? pa < pb : a < b
        }
        for i in 0..<slice.count { indices[lo + i] = slice[i] }
        let mid = lo + (hi - lo) / 2
        let nodeIndex = nodes.count
        nodes.append(Node(index: indices[mid], axis: axis, left: -1, right: -1))
        let leftChild = build(indices: &indices, lo: lo, hi: mid, depth: depth + 1, points: points, nodes: &nodes)
        let rightChild = build(indices: &indices, lo: mid + 1, hi: hi, depth: depth + 1, points: points, nodes: &nodes)
        nodes[nodeIndex].left = leftChild
        nodes[nodeIndex].right = rightChild
        return nodeIndex
    }

    /// Nearest neighbor to `query`, as `(index into the original points array, distance)`, or
    /// `nil` if the tree is empty.
    func nearest(to query: SIMD3<Double>) -> (index: Int, distance: Double)? {
        guard root >= 0 else { return nil }
        var bestIndex = -1
        var bestDistSq = Double.infinity
        search(node: root, query: query, bestIndex: &bestIndex, bestDistSq: &bestDistSq)
        guard bestIndex >= 0 else { return nil }
        return (bestIndex, bestDistSq.squareRoot())
    }

    private func search(node: Int, query: SIMD3<Double>, bestIndex: inout Int, bestDistSq: inout Double) {
        guard node >= 0 else { return }
        let n = nodes[node]
        let p = points[n.index]
        let d = query - p
        let distSq = simd_dot(d, d)
        if distSq < bestDistSq {
            bestDistSq = distSq
            bestIndex = n.index
        }
        let diff = query[n.axis] - p[n.axis]
        let (nearChild, farChild) = diff < 0 ? (n.left, n.right) : (n.right, n.left)
        search(node: nearChild, query: query, bestIndex: &bestIndex, bestDistSq: &bestDistSq)
        if diff * diff < bestDistSq {
            search(node: farChild, query: query, bestIndex: &bestIndex, bestDistSq: &bestDistSq)
        }
    }
}
