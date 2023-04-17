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

#include <string.h>
#include <cuda_runtime.h>

#include <vector>
#include <functional>
#include <string>
#include <numeric>

#include "runtime/miloc_mat.hpp"
#include "utils/miloc_basic.hpp"

namespace cyberdog
{
namespace miloc
{

inline int64_t Volume(std::vector<int> & shape_)
{
  return std::accumulate(shape_.begin(), shape_.end(), 1, std::multiplies<int64_t>());
}

inline int GetElementSize(MilocMatType type)
{
  switch (type) {
    case SLAM_MAT_U8: return 1;
    case SLAM_MAT_S8: return 1;
    case SLAM_MAT_U16: return 2;
    case SLAM_MAT_S16: return 2;
    case SLAM_MAT_U32: return 4;
    case SLAM_MAT_S32: return 4;
    case SLAM_MAT_U64: return 8;
    case SLAM_MAT_S64: return 8;
    case SLAM_MAT_F32: return 4;
    case SLAM_MAT_F64: return 8;
  }

  return 0;
}

MilocMat::MilocMat()
: flag_malloc_(false), data_(NULL)
{
}

MilocMat::~MilocMat()
{
  if (data_ && flag_malloc_) {
    if (SLAM_MAT_GPU & format_) {
      cudaFree(data_);
    } else {
      free(data_);
    }
    data_ = NULL;
    flag_malloc_ = false;
  }
}

int MilocMat::Init(std::vector<int> & shape_in, MilocMatType type_in, int format_in)
{
  if (data_) {
    return SLAM_ERROR;
  }
  shape_ = shape_in;
  type_ = type_in;
  format_ = format_in;
  dims_ = shape_.size();
  data_size_ = Volume(shape_) * GetElementSize(type_);

  if (SLAM_MAT_GPU & format_) {
    cudaMalloc(&data_, data_size_);
  } else {
    data_ = malloc(data_size_);
  }
  if (data_ == NULL) {
    return SLAM_NULL_PTR;
  }
  flag_malloc_ = true;
  return SLAM_OK;
}

int MilocMat::Clone(MilocMat & src, bool deep_clone)
{
  shape_ = src.shape_;
  type_ = src.type_;
  format_ = src.format_;
  dims_ = src.dims_;
  data_size_ = src.data_size_;
  if (SLAM_MAT_GPU & format_) {
    cudaMalloc(&data_, data_size_);
  } else {
    data_ = malloc(data_size_);
  }
  if (data_ == NULL) {
    return SLAM_NULL_PTR;
  }
  flag_malloc_ = true;
  if (deep_clone) {
    if (SLAM_MAT_GPU & format_ & src.format_) {
      cudaMemcpy(data_, src.data_, src.data_size_, cudaMemcpyDeviceToDevice);
    } else {
      memcpy(data_, src.data_, src.data_size_);
    }
  }
  return SLAM_OK;
}

int MilocMat::Save(const std::string & filename)
{
  FILE * p_file = fopen(filename.c_str(), "wb");
  if (p_file) {
    size_t size = Volume(shape_) * GetElementSize(type_);
    if (format_ & SLAM_MAT_GPU) {
      void * data_cpu = malloc(size);
      cudaMemcpy(data_cpu, data_, size, cudaMemcpyDeviceToHost);
      fwrite(data_cpu, 1, size, p_file);
      free(data_cpu);
    } else {
      fwrite(data_, 1, size, p_file);
    }
    fclose(p_file);
  } else {
    return SLAM_ERROR;
  }
  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
