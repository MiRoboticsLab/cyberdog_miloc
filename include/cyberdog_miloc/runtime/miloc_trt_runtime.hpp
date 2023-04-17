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

#ifndef CYBERDOG_MILOC__RUNTIME__MILOC_TRT_RUNTIME_HPP_
#define CYBERDOG_MILOC__RUNTIME__MILOC_TRT_RUNTIME_HPP_

#include <vector>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <glog/logging.h>

#include "runtime/miloc_trt_net.hpp"
#include "base/miloc_types.hpp"

namespace cyberdog
{
namespace miloc
{

#define GLOBAL_FEATURE_SIZE 1024
#define LOCAL_FEATURE_SIZE 96

typedef Eigen::Matrix<float, GLOBAL_FEATURE_SIZE, 1> GlobalFeature;
typedef std::array<float, GLOBAL_FEATURE_SIZE> GlobalFeatureHalf;

typedef std::vector<float> LocalFeature;    // should be 256 * n
typedef std::vector<uint16_t> LocalFeatureHalf;    // should be 256 * n
typedef std::array<float, 2> Keypoint;

struct GlobalFeatureData
{
  std::string image_name;
  GlobalFeature feature;
};
struct GlobalFeatureDataHalf
{
  std::string image_name;
  GlobalFeatureHalf feature;
};
typedef std::unordered_map<std::string, GlobalFeature> GlobalFeatureMap;

struct LocalFeatureData
{
  std::string image_name;
  LocalFeature features;
  std::vector<Keypoint> keypoints;
  std::vector<float> scores;
};

struct LocalFeatureDataHalf
{
  std::string image_name;
  LocalFeatureHalf features;
  std::vector<Keypoint> keypoints;
  std::vector<u_int16_t> scores;
};

struct MatchData
{
  std::string match_pair_name;
  std::vector<int> match_idxs;
  std::vector<float> match_scores;
};

class LocalModel
{
public:
  int Load(const std::string & model_path);

  void SetConfig(const ModelParams & model_params)
  {
    m_params = model_params;
  }

  int FilterKps(LocalFeatureData & ouput);
  int Process(const RawImage & input, LocalFeatureData & ouput);

private:
  int status_;
  ModelParams m_params;
  TrtNet net_;
};

class GlobalModel
{
public:
  int Load(const std::string & model_path);
  int Process(const RawImage & input, GlobalFeatureData & ouput);

private:
  int status_;
  TrtNet net_;
};

class MatchModel
{
public:
  int Load(const std::string & model_path);
  int Process(LocalFeatureData & input1, LocalFeatureData & input2, MatchData & output);

private:
  int status_;
  TrtNet net_;
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__RUNTIME__MILOC_TRT_RUNTIME_HPP_
