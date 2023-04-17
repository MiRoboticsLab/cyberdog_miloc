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

#ifndef CYBERDOG_MILOC__RUNTIME__MILOC_TRT_NET_HPP_
#define CYBERDOG_MILOC__RUNTIME__MILOC_TRT_NET_HPP_

#include <unistd.h>
#include <time.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <memory>
#include <utility>

#include "NvInfer.h"
#include "cuda_runtime.h"
#include "cudnn.h"

#include "utils/miloc_basic.hpp"
#include "runtime/miloc_mat.hpp"

namespace cyberdog
{
namespace miloc
{

class Profiler : public nvinfer1::IProfiler
{
public:
  void PrintLayerTimes(int iterations_times)
  {
    float total_time = 0;
    for (size_t i = 0; i < m_profile.size(); i++) {
      printf(
        "%-40.40s %4.3fms\n",
        m_profile[i].first.c_str(), m_profile[i].second / iterations_times);
      total_time += m_profile[i].second;
    }
    printf("Time over all layers: %4.3f\n", total_time / iterations_times);
  }

private:
  typedef std::pair<std::string, float> Record;
  std::vector<Record> m_profile;

  virtual void reportLayerTime(const char * layer_name, float ms) noexcept
  {
    auto record = std::find_if(
      m_profile.begin(), m_profile.end(), [&](
        const Record & r) {return r.first == layer_name;});
    if (record == m_profile.end()) {
      m_profile.push_back(std::make_pair(layer_name, ms));
    } else {
      record->second += ms;
    }
  }
};

class TrtNet
{
public:
  TrtNet();

  ~TrtNet();

  int LoadModel(const std::string & engine_file);

  int SetDimensions(const std::string & input_name, nvinfer1::Dims dim);

  int Inference(std::vector<MilocMat *> & input_data, std::vector<MilocMat *> & output_data);

private:
  std::shared_ptr<nvinfer1::IRuntime> runtime_;
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
  std::vector<void *> buffer_;

  int input_count_;
  int iteration_time_;
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__RUNTIME__MILOC_TRT_NET_HPP_
