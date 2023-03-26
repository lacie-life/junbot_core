#include <helper_cuda.h>
#include <cuda/Cuda.hpp>

namespace semantic_slam
{
  namespace cuda
  {
    void deviceSynchronize()
    {
      checkCudaErrors(cudaDeviceSynchronize());
    }
  }
}
