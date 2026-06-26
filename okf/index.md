---
type: repo
title: OCCTSwiftMesh
resource: https://github.com/SecondMouseAU/OCCTSwiftMesh
tags: [cad, occt, mesh, decimation, slicing, swift, kernel]
description: Mesh-domain algorithms (decimation, planar slicing) for the OCCTSwift ecosystem, operating on OCCTSwift.Mesh instances.
timestamp: 2026-06-22
---

# OCCTSwiftMesh

> Mesh-domain post-processing for the OCCTSwift ecosystem. OCCT ships `BRepMesh_*` for mesh
> **generation** but no decimation, simplification, smoothing, hole-filling, or remeshing — this
> package fills that gap with permissively-licensed, vendored implementations that consume
> OCCT-produced `Mesh` instances. It complements the kernel rather than extending it.

## Role in the ecosystem

- **Cluster:** kernel
- **Depends on:** [OCCTSwift](https://github.com/SecondMouseAU/OCCTSwift) — source of the `Mesh`
  type these algorithms operate on (floored at OCCTSwift v1.7.1).
- **Feeds:** downstream consumers of mesh post-processing — e.g. OCCTSwiftScripts' `simplify-mesh`
  verb and OCCTMCP's `simplify_mesh` tool. Leaf with respect to other intra-org kernel libraries.

## Components

See [`components/`](components/index.md) for the public surface (`Mesh.simplified(_:)`,
`Mesh.crossSection(plane:)`, and the vendored meshoptimizer bridge).

## References

See [`references/`](references/index.md) for the decimation algorithm notes, vendored
meshoptimizer upstream, and OpenCASCADE.

## Notes

- Vendors [meshoptimizer](https://github.com/zeux/meshoptimizer) v1.1 (MIT) via the
  `OCCTMeshOptimizer` C++ bridge target.
- LGPL-2.1, matching OCCTSwift; vendored components retain their own permissive licenses.

## Policies

- [Query `context` first for OCCT / OCCTSwift docs](policies/context-first.md)
- [Documentation updates are mandatory](policies/docs-current.md)
