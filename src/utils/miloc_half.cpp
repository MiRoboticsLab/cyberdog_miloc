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

#include "utils/miloc_basic.hpp"
#include "utils/miloc_half.hpp"

namespace cyberdog
{
namespace miloc
{

namespace miloc_half_impl
{
int LocalFeaturesHalfToFloat(
  LocalFeatureData & local_feature_data,
  LocalFeatureDataHalf & local_feature_data_half)
{
  local_feature_data.image_name = local_feature_data_half.image_name;
  local_feature_data.features.clear();
  local_feature_data.keypoints.clear();
  local_feature_data.scores.clear();

  local_feature_data.keypoints.swap(local_feature_data_half.keypoints);

  int score_num = local_feature_data_half.scores.size();
  for (int i = 0; i < score_num; i++) {
    local_feature_data.scores.emplace_back(HalfToFloat(local_feature_data_half.scores[i]));
  }

  int descriptors_dimension = local_feature_data_half.features.size();

  for (int i = 0; i < descriptors_dimension; i++) {
    local_feature_data.features.emplace_back(HalfToFloat(local_feature_data_half.features[i]));
  }


  return SLAM_OK;
}

int LocalFeaturesFloatToHalf(
  LocalFeatureData & local_feature_data,
  LocalFeatureDataHalf & local_feature_data_half)
{
  local_feature_data_half.image_name = local_feature_data.image_name;
  local_feature_data_half.features.clear();
  local_feature_data_half.keypoints.clear();
  local_feature_data_half.scores.clear();

  local_feature_data_half.keypoints.swap(local_feature_data.keypoints);

  int score_num = local_feature_data.scores.size();
  for (int i = 0; i < score_num; i++) {
    local_feature_data_half.scores.emplace_back(FloatToHalf(local_feature_data.scores[i]));
  }

  int descriptors_dimension = local_feature_data.features.size();

  for (int i = 0; i < descriptors_dimension; i++) {
    local_feature_data_half.features.emplace_back(FloatToHalf(local_feature_data.features[i]));
  }

  return SLAM_OK;
}

int GlobalFeaturesHalfToFloat(
  GlobalFeature & global_feature,
  GlobalFeatureHalf & global_feature_half)
{
  int size = global_feature_half.size();

  for (auto i = 0; i < size; i++) {
    global_feature(i) = HalfToFloat(global_feature_half[i]);
  }

  return SLAM_OK;
}
int GlobalFeaturesFloatToHalf(
  GlobalFeature & global_feature,
  GlobalFeatureHalf & global_feature_half)
{
  int size = global_feature.size();

  for (auto i = 0; i < size; i++) {
    global_feature_half[i] = FloatToHalf(global_feature(i));
  }

  return SLAM_OK;
}

}  // namespace miloc_half_impl
}  // namespace miloc
}  // namespace cyberdog
