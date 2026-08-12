// OCCTMeshOptimizer; C ABI bridge between Swift and vendored meshoptimizer.
//
// Vendored meshoptimizer version is recorded in NOTICE.md.
// Re-vendoring procedure: docs/VENDORING.md.

#ifndef OCCT_MESH_OPTIMIZER_H
#define OCCT_MESH_OPTIMIZER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /// Result of a successful OCCTMeshSimplify call.
  ///
  /// `vertices` and `indices` are caller-owned buffers allocated by the bridge.
  /// Release them with OCCTMeshSimplifyFreeResult before discarding the struct.
  ///
  /// Vertex layout: 3 floats per vertex, packed as [x0,y0,z0, x1,y1,z1, ...].
  /// Index layout: 3 indices per triangle, packed as [a0,b0,c0, a1,b1,c1, ...].
  /// Vertex array is compacted; orphan vertices left over from decimation are
  /// removed and indices are remapped accordingly.
  typedef struct
  {
    float*    vertices;
    uint32_t  vertexCount;
    uint32_t* indices;
    uint32_t  triangleCount;

    /// Achieved Hausdorff distance from input to simplified mesh, in the
    /// same units as the input vertex coordinates.
    double hausdorffDistance;

    uint32_t beforeTriangleCount;
    uint32_t afterTriangleCount;
  } OCCTMeshSimplifyResult;

  /// Decimate a triangle mesh using meshoptimizer's quadric-error-metric
  /// edge-collapse algorithm.
  ///
  /// `vertices`, `indices`: read-only inputs. Tightly packed (no stride padding).
  /// `vertexCount` >= 3 and `indexCount` >= 3 with `indexCount % 3 == 0`.
  ///
  /// `targetIndexCount`: 3 × desired triangle count. Must be a multiple of 3
  ///   and <= `indexCount`. The achieved count may be larger if the algorithm
  ///   cannot reduce further while respecting the error cap.
  ///
  /// `targetError`: maximum acceptable absolute deviation, in input mesh
  ///   coordinate units. Pass FLT_MAX (or any sufficiently large finite value)
  ///   to disable the error cap and decimate purely to the target count.
  ///
  /// `preserveBoundary`: when true, edges on the mesh boundary (free edges)
  ///   are not collapsed.
  ///
  /// `preserveTopology`: forward-compatibility flag; in the current bridge
  ///   topology is always preserved (meshoptimizer's default behavior).
  ///
  /// On success, `outResult` is populated with caller-owned buffers and the
  /// function returns true. On invalid input or allocation failure, returns
  /// false and `outResult` is left untouched.
  bool OCCTMeshSimplify(const float*            vertices,
                        uint32_t                vertexCount,
                        const uint32_t*         indices,
                        uint32_t                indexCount,
                        uint32_t                targetIndexCount,
                        float                   targetError,
                        bool                    preserveBoundary,
                        bool                    preserveTopology,
                        OCCTMeshSimplifyResult* outResult);

  /// Release the buffers held by an OCCTMeshSimplifyResult populated by
  /// OCCTMeshSimplify. Safe to call on a zero-initialized struct or one
  /// already freed; idempotent.
  void OCCTMeshSimplifyFreeResult(OCCTMeshSimplifyResult* result);

  /// Returns the bounding-box-diagonal scale factor used by meshoptimizer
  /// internally for relative ↔ absolute error conversion.
  ///
  /// The OCCTMeshSimplify ABI itself takes and reports absolute error, so
  /// this function is exposed for callers who want to convert their own
  /// relative tolerances or compare against meshoptimizer's relative-error
  /// API directly.
  float OCCTMeshSimplifyScale(const float* vertices, uint32_t vertexCount);

#ifdef __cplusplus
}
#endif

#endif
