// OCCTMeshOptimizer bridge; wires the C ABI declared in OCCTMeshOptimizer.h
// into the vendored meshoptimizer library under src/meshoptimizer/.

#include "OCCTMeshOptimizer.h"
#include "meshoptimizer/meshoptimizer.h"

#include <cstdlib>
#include <vector>

extern "C"
{

  bool OCCTMeshSimplify(const float*    vertices,
                        uint32_t        vertexCount,
                        const uint32_t* indices,
                        uint32_t        indexCount,
                        uint32_t        targetIndexCount,
                        float           targetError,
                        bool            preserveBoundary,
                        bool /*preserveTopology*/,
                        OCCTMeshSimplifyResult* outResult)
  {
    try
    {
      if (!vertices || !indices || !outResult)
        return false;
      if (vertexCount < 3 || indexCount < 3)
        return false;
      if (indexCount % 3 != 0)
        return false;
      if (targetIndexCount % 3 != 0)
        return false;
      if (targetIndexCount > indexCount)
        return false;

      const size_t stride  = sizeof(float) * 3;
      unsigned int options = meshopt_SimplifyErrorAbsolute;
      if (preserveBoundary)
        options |= meshopt_SimplifyLockBorder;

      std::vector<unsigned int> simplifiedIndices(indexCount);
      float                     resultError = 0.f;

      size_t newIndexCount = meshopt_simplify(simplifiedIndices.data(),
                                              indices,
                                              indexCount,
                                              vertices,
                                              vertexCount,
                                              stride,
                                              targetIndexCount,
                                              targetError,
                                              options,
                                              &resultError);

      // meshopt_simplify reuses the input vertex array layout, leaving
      // orphan vertices once edges collapse. Compact via a fetch remap.
      std::vector<unsigned int> remap(vertexCount);
      size_t newVertexCount = meshopt_optimizeVertexFetchRemap(remap.data(),
                                                               simplifiedIndices.data(),
                                                               newIndexCount,
                                                               vertexCount);

      if (newVertexCount == 0 || newIndexCount == 0)
        return false;

      float* outVertices = static_cast<float*>(std::malloc(sizeof(float) * 3 * newVertexCount));
      unsigned int* outIndices =
        static_cast<unsigned int*>(std::malloc(sizeof(unsigned int) * newIndexCount));
      if (!outVertices || !outIndices)
      {
        std::free(outVertices);
        std::free(outIndices);
        return false;
      }

      meshopt_remapVertexBuffer(outVertices, vertices, vertexCount, stride, remap.data());
      meshopt_remapIndexBuffer(outIndices, simplifiedIndices.data(), newIndexCount, remap.data());

      outResult->vertices            = outVertices;
      outResult->vertexCount         = static_cast<uint32_t>(newVertexCount);
      outResult->indices             = outIndices;
      outResult->triangleCount       = static_cast<uint32_t>(newIndexCount / 3);
      outResult->hausdorffDistance   = static_cast<double>(resultError);
      outResult->beforeTriangleCount = indexCount / 3;
      outResult->afterTriangleCount  = static_cast<uint32_t>(newIndexCount / 3);
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  void OCCTMeshSimplifyFreeResult(OCCTMeshSimplifyResult* result)
  {
    if (!result)
      return;
    std::free(result->vertices);
    std::free(result->indices);
    result->vertices      = nullptr;
    result->indices       = nullptr;
    result->vertexCount   = 0;
    result->triangleCount = 0;
  }

  float OCCTMeshSimplifyScale(const float* vertices, uint32_t vertexCount)
  {
    if (!vertices || vertexCount == 0)
      return 0.f;
    return meshopt_simplifyScale(vertices, vertexCount, sizeof(float) * 3);
  }

} // extern "C"
