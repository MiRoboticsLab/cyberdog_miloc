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

#include <fstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "base/miloc_online_handler.hpp"
#include "utils/miloc_basic.hpp"

namespace cyberdog
{
namespace miloc
{

OnlineHandler::~OnlineHandler()
{}

int OnlineHandler::Init(
  std::string & table_name, CyberDB * cyberdb, std::string & map_path,
  std::vector<RealCamera> * cameras)
{
  table_name_ = table_name;
  cyberdb_ = cyberdb;
  map_path_ = map_path;
  cameras_ = cameras;
  map_db_ = new SparseMapDB();

  return SLAM_OK;
}

int OnlineHandler::CreateMap(bool slam_flag, int & current_map_id)
{
  int max_map_id = -1;
  int ret = cyberdb_->GetMaxMapId(max_map_id, table_name_);
  int new_map_id = max_map_id + 1;

  std::string map_name = "unfinished_map_" + std::to_string(new_map_id);
  std::string map_url = map_path_ + "/default/" + map_name;
  ret |= cyberdb_->WriteNewMap(new_map_id, map_name, map_url, table_name_, slam_flag);

  image_path_ = map_url + "/images";
  visual_path_ = map_url + "/visual";
  std::string m_lidar_path = map_url + "/lidar";

  current_map_id = new_map_id;

  if (!boost::filesystem::create_directory(map_url) ||
    !boost::filesystem::create_directory(image_path_) ||
    !boost::filesystem::create_directory(
      image_path_ + "/" +
      GetCameraName(static_cast<int>(CameraId::FRONT))) ||
    !boost::filesystem::create_directory(
      image_path_ + "/" + GetCameraName((static_cast<int>(CameraId::LEFT)))) ||
      !boost::filesystem::create_directory(
        image_path_ + "/" +
        GetCameraName(static_cast<int>(CameraId::RIGHT))) ||
      !boost::filesystem::create_directory(visual_path_) ||
      !boost::filesystem::create_directory(m_lidar_path))
  {
    LOG(ERROR) << "Create new map directory failed";
    return SLAM_ERROR;
  }

  ret |= map_db_->Open(visual_path_ + "/database.db");
  ret |= map_db_->CreateDbTables();
  LOG(INFO) << "Create new map succeed";

  is_creating_map_ = true;

  return ret;
}

int OnlineHandler::FinishMap(bool slam_flag)
{
  is_creating_map_ = false;
  map_db_->Close();

  std::string traj_path = visual_path_ + "/trajectory.txt";
  std::string visual_traj_path = map_path_ + "/default/visual/trajectory.txt";
  std::string lidar_traj_path = map_path_ + "/default/lidar/trajectory.txt";

  if (slam_flag && boost::filesystem::exists(visual_traj_path)) {
    boost::filesystem::copy_file(visual_traj_path, traj_path);
    boost::filesystem::remove(visual_traj_path);
  } else if (!slam_flag && boost::filesystem::exists(lidar_traj_path)) {
    boost::filesystem::copy_file(lidar_traj_path, traj_path);
    boost::filesystem::remove(lidar_traj_path);
  } else {
    return SLAM_ERROR;
  }

  return SLAM_OK;
}

int OnlineHandler::ProcessImage(
  uint64_t timestamp, const std::vector<cv::Mat> & images,
  const std::vector<int> & camera_idxs)
{
  ImageBundle image_bundle;
  CHECK_EQ(images.size(), camera_idxs.size());

  if (is_creating_map_) {
    for (size_t i = 0; i < images.size(); ++i) {
      image_bundle.image = images[i];
      image_bundle.camera_id = camera_idxs[i];
      image_bundle.timestamp = timestamp;
      SaveImage(image_bundle);
    }
  }

  return SLAM_OK;
}

int OnlineHandler::SaveImage(ImageBundle & image_bundle)
{
  cv::imwrite(
    image_path_ + "/" + GetCameraName(image_bundle.camera_id) + "/" +
    TimeStampFormat(
      std::to_string(
        image_bundle.timestamp), TimeStamp::NANOSECOND) + ".jpg", image_bundle.image);
  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
