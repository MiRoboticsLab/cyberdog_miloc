// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "runtime/miloc_model_pre_process.hpp"

namespace cyberdog
{
namespace miloc
{

static int GetBlocks(const int N)
{
  return (N + CUDA_NUM_THREADS - 1) / CUDA_NUM_THREADS;
}

__global__ void local_pre_norm_kernel(const uint8_t *__restrict src, float *dst, int size)
{
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index >= size)
        return;
    if (index >= 240 * 640){
        dst[index] = 1.0;
    }else{
        dst[index] = (float) src[index] / 255.f;
    }
}

__global__ void global_pre_norm_kernel(const uint8_t *__restrict src, float *dst, int size)
{
    int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index >= size)
        return;
    dst[index] = (float) src[index] / 255.f;
}

int LocalModelAPreProcess(MilocMat &src, MilocMat &dst)
{
    int size = src.shape_[1] * src.shape_[2];
    
    local_pre_norm_kernel<<<GetBlocks(size), CUDA_NUM_THREADS>>>(
        (uint8_t *)src.data_, (float *)dst.data_, size);
    
    return cudaGetLastError();
}

int GlobalModelAPreProcess(MilocMat &src, MilocMat &dst)
{
    int size = src.shape_[1] * src.shape_[2];
    
    global_pre_norm_kernel<<<GetBlocks(size), CUDA_NUM_THREADS>>>(
        (uint8_t *)src.data_, (float *)dst.data_, size);
    
    return cudaGetLastError();;
}

}  // namespace miloc
}  // namespace cyberdog