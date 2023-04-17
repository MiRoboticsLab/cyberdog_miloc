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

#include <vector>
#include <string>

#include <glog/logging.h>
#include <boost/algorithm/string.hpp>

#include "utils/miloc_basic.hpp"
#include "utils/miloc_convertor.hpp"

namespace cyberdog
{
namespace miloc
{

int MatToRawImage(const cv::Mat & mat, RawImage & img)
{
  if (mat.empty()) {
    LOG(ERROR) << "Image mat is empty";
    return SLAM_NULL_PTR;
  }

  img.channel = mat.channels();
  img.height = mat.rows;
  img.width = mat.cols;
  img.pitch = mat.step;
  img.elem_type = mat.elemSize1();
  img.data = malloc(img.pitch * img.height);

  memcpy(img.data, mat.data, img.pitch * img.height);
  return SLAM_OK;
}

void ReleaseRawImage(std::vector<RawImage> & decode_data_list)
{
  for (uint32_t i = 0; i < decode_data_list.size(); ++i) {
    if (decode_data_list[i].data != NULL) {
      free(decode_data_list[i].data);
      decode_data_list[i].data = NULL;
    }
  }
}

std::string GetCameraName(int camera_id)
{
  std::string camera_name;
  switch (CameraId(camera_id)) {
    case CameraId::FRONT:
      camera_name = std::string(CAMERA_FRONT);
      break;
    case CameraId::LEFT:
      camera_name = std::string(CAMERA_LEFT);
      break;
    case CameraId::RIGHT:
      camera_name = std::string(CAMERA_RIGHT);
      break;
    default:
      camera_name = std::string(CAMERA_UNKNOWN);
      break;
  }
  return camera_name;
}

int GetCameraId(const std::string & camera_name)
{
  int camera_id;

  if (CAMERA_FRONT == camera_name) {
    camera_id = static_cast<int>(CameraId::FRONT);
  } else if (CAMERA_LEFT == camera_name) {
    camera_id = static_cast<int>(CameraId::LEFT);
  } else if (CAMERA_RIGHT == camera_name) {
    camera_id = static_cast<int>(CameraId::RIGHT);
  } else {
    camera_id = static_cast<int>(CameraId::UNKNOWN);
  }

  return camera_id;
}

int KeypointRecover(
  LocalFeatureData & local_feature_raw, LocalFeatureData & local_feature,
  int height, int width)
{
  local_feature.features.clear();
  local_feature.keypoints.clear();
  local_feature.scores.clear();

  float height_scale = static_cast<float>(height) / 480.;
  float width_scale = static_cast<float>(width) / 640.;

  local_feature.features = local_feature_raw.features;
  local_feature.image_name = local_feature_raw.image_name;
  local_feature.scores = local_feature_raw.scores;
  for (auto & kp_raw : local_feature_raw.keypoints) {
    Keypoint kp;
    kp[0] = (kp_raw[0]) * width_scale + 0.5;
    kp[1] = (kp_raw[1]) * height_scale + 0.5;
    local_feature.keypoints.emplace_back(kp);
  }

  return SLAM_OK;
}

int KeypointRecover(LocalFeatureData & local_feature, int height, int width)
{
  if (height == 480 && width == 640) {
    return SLAM_OK;
  }

  std::vector<Keypoint> kps_resize;
  float height_scale = static_cast<float>(height) / 480.;
  float width_scale = static_cast<float>(width) / 640.;

  for (auto & kp_raw : local_feature.keypoints) {
    Keypoint kp;
    kp[0] = (kp_raw[0]) * width_scale + 0.5;
    kp[1] = (kp_raw[1]) * height_scale + 0.5;
    kps_resize.emplace_back(kp);
  }

  local_feature.keypoints.clear();
  local_feature.keypoints.swap(kps_resize);

  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
