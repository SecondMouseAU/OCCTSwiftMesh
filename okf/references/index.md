---
type: reference
title: References index
resource: https://github.com/SecondMouseAU/OCCTSwiftMesh
tags: [index]
description: External standards, upstreams, and docs OCCTSwiftMesh depends on.
timestamp: 2026-06-22
---

# References

- [docs/algorithms/decimation.md](https://github.com/SecondMouseAU/OCCTSwiftMesh/blob/main/docs/algorithms/decimation.md)
  — decimation algorithm notes.
- [docs/algorithms/mesh-foundations.md](https://github.com/SecondMouseAU/OCCTSwiftMesh/blob/main/docs/algorithms/mesh-foundations.md)
  — weld, connectivity toolkit, and integrity report notes.
- [docs/algorithms/segmentation.md](https://github.com/SecondMouseAU/OCCTSwiftMesh/blob/main/docs/algorithms/segmentation.md)
  — dihedral region-growing + primitive-fit merge notes.
- [docs/VENDORING.md](https://github.com/SecondMouseAU/OCCTSwiftMesh/blob/main/docs/VENDORING.md)
  — how meshoptimizer is vendored and updated.
- [docs/CHANGELOG.md](https://github.com/SecondMouseAU/OCCTSwiftMesh/blob/main/docs/CHANGELOG.md)
  — release history.
- [meshoptimizer](https://github.com/zeux/meshoptimizer) — vendored upstream (MIT) powering the
  decimation path.
- [OCCTReconstruct](https://github.com/SecondMouseAU/OCCTReconstruct) — source of the
  `ReconstructCompute` reference implementation the mesh foundations and segmentation algorithms
  were ported from.
- [OpenCASCADE Technology (OCCT)](https://dev.opencascade.org/) — upstream B-Rep/mesh kernel the
  ecosystem wraps; OCCT meshes are the input to these algorithms.
