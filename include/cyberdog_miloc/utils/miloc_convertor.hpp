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

#ifndef CYBERDOG_MILOC__UTILS__MILOC_CONVERTOR_HPP_
#define CYBERDOG_MILOC__UTILS__MILOC_CONVERTOR_HPP_

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "base/miloc_types.hpp"
#include "runtime/miloc_trt_runtime.hpp"

namespace cyberdog
{
namespace miloc
{

#define CAMERA_FRONT "front"
#define CAMERA_LEFT "left"
#define CAMERA_RIGHT "right"
#define CAMERA_UNKNOWN "unknown"

typedef enum
{
  UNKNOWN = 0,
  FRONT,
  LEFT,
  RIGHT,
} CameraId;

int MatToRawImage(const cv::Mat & mat, RawImage & img);

void ReleaseRawImage(std::vector<RawImage> & decode_data_list);

std::string GetCameraName(int camera_id);

int GetCameraId(const std::string & camera_name);

int KeypointRecover(LocalFeatureData & local_feature, int height, int width);

int KeypointRecover(
  LocalFeatureData & local_feature_raw, LocalFeatureData & local_feature,
  int height, int width);

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__UTILS__MILOC_CONVERTOR_HPP_
