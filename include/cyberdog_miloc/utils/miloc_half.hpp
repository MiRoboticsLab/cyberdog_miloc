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

#ifndef CYBERDOG_MILOC__UTILS__MILOC_HALF_HPP_
#define CYBERDOG_MILOC__UTILS__MILOC_HALF_HPP_

#include "Eigen/Core"
#include "runtime/miloc_trt_runtime.hpp"

namespace cyberdog
{
namespace miloc
{
namespace miloc_half_impl
{

using float_16_t = uint16_t;
inline float_16_t FloatToHalf(float f)
{
  return Eigen::half_impl::float_to_half_rtne(f).x;
}

inline float HalfToFloat(float_16_t half)
{
  return Eigen::half_impl::half_to_float(Eigen::half_impl::raw_uint16_to_half(half));
}

int LocalFeaturesHalfToFloat(
  LocalFeatureData & local_feature_data,
  LocalFeatureDataHalf & local_feature_data_half);
int LocalFeaturesFloatToHalf(
  LocalFeatureData & local_feature_data,
  LocalFeatureDataHalf & local_feature_data_half);

int GlobalFeaturesHalfToFloat(
  GlobalFeature & global_feature,
  GlobalFeatureHalf & global_feature_half);
int GlobalFeaturesFloatToHalf(
  GlobalFeature & global_feature_data,
  GlobalFeatureHalf & global_feature_half);

}  // namespace miloc_half_impl
}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__UTILS__MILOC_HALF_HPP_
