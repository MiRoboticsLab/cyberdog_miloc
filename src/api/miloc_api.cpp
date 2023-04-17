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

#include <memory>
#include <vector>
#include <string>
#include <glog/logging.h>

#include "api/miloc_api.hpp"
#include "utils/miloc_convertor.hpp"
#include "misc/miloc_misc.hpp"

namespace cyberdog
{
namespace miloc
{

std::shared_ptr<MiLocServicer> miloc_servicer_;

std::vector<RawImage> decode_data_list;


int MilocApi::StartUp(const std::string & config_file)
{
  miloc_servicer_ = std::make_shared<MiLocServicer>();
  int ret = miloc_servicer_->InitServer(config_file);
  return ret;
}

int MilocApi::SetSLAM(bool visual_slam)
{
  int ret = miloc_servicer_->SetSLAM(visual_slam);
  return ret;
}

bool MilocApi::GetSLAM()
{
  return miloc_servicer_->GetSLAM();
}

int MilocApi::EstimatePose(
  const std::vector<cv::Mat> & images, Eigen::Quaterniond & orientation, Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation_init, const Eigen::Vector3d & position_init,
  int & reply_status, float & confidence)
{
  int pre_status = miloc_servicer_->GetStatus();
  miloc_servicer_->SetStatus(MilocStatus::kRELOCATING);

  int ret = SLAM_OK;
  decode_data_list.clear();

  cv::Mat image_new;
  std::vector<int> camera_idxs;

  for (u_int32_t i = 0; i < images.size(); ++i) {
    RawImage mat_data;
    camera_idxs.push_back(i + 1);

    mat_data.raw_height = images[i].rows;
    mat_data.raw_width = images[i].cols;

    if (images[i].cols != 640) {
      cv::resize(images[i], image_new, cv::Size(640, 480));
    } else {
      image_new = images[i];
    }

    if (MatToRawImage(image_new, mat_data) != SLAM_OK) {
      LOG(ERROR) << "MatToRawImage transfer fail";
      decode_data_list.push_back(mat_data);
      ReleaseRawImage(decode_data_list);
      miloc_servicer_->SetStatus(pre_status);
      return SLAM_ERROR;
    }
    decode_data_list.push_back(mat_data);
  }

  ret = miloc_servicer_->EstimatePose(decode_data_list, camera_idxs, orientation, position);

  ReleaseRawImage(decode_data_list);

  if (ret != SLAM_OK) {
    if (ret == SLAM_BAD_ARG) {
      reply_status = 5;
    } else {
      reply_status = 3;
    }

  } else {
    int check_ret = miloc_servicer_->CheckResult(
      position, position_init, orientation_init,
      confidence);
    if (SLAM_ERROR == check_ret) {
      reply_status = 2;
    } else {
      reply_status = 1;
    }
  }

  miloc_servicer_->SetStatus(pre_status);

  return ret;
}

int MilocApi::DeleteRelocMap(int map_id)
{
  return miloc_servicer_->DeleteRelocMap(map_id);
}

int MilocApi::GetRelocMapStatus(int map_id)
{
  return miloc_servicer_->GetRelocMapStatus(map_id);
}

int MilocApi::CreateMap(int & map_id)
{
  int ret = miloc_servicer_->CreateMap(map_id);
  return ret;
}

int MilocApi::FinishMap()
{
  int ret = miloc_servicer_->FinishMap();
  return ret;
}

int MilocApi::StartNavigation()
{
  int ret = miloc_servicer_->StartNavigation();
  return ret;
}

int MilocApi::StopNavigation()
{
  int ret = miloc_servicer_->StopNavigation();
  return ret;
}

int MilocApi::CollectImage(
  uint64_t timestamp,
  const std::vector<cv::Mat> & images,
  const std::vector<int> & camera_idxs)
{
  int ret = miloc_servicer_->CollectImage(timestamp, images, camera_idxs);
  return ret;
}

int MilocApi::GetMilocStatus()
{
  return miloc_servicer_->GetStatus();
}

int MilocApi::GetCurrentMapId()
{
  return miloc_servicer_->GetMapId();
}

int MilocApi::ReconstructMap(int map_id, const std::string & map_name)
{
  int ret = miloc_servicer_->ReconstructMap(map_id, map_name);
  return ret;
}

int MilocApi::ShutDown()
{
  miloc_servicer_.reset();
  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
