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
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <glog/logging.h>

#include "base/miloc_restruct.hpp"
#include "base/miloc_io.hpp"
#include "base/miloc_extract.hpp"
#include "utils/miloc_database.hpp"
#include "utils/miloc_interpolation.hpp"

namespace cyberdog
{
namespace miloc
{

int RestructMapper::Init(
  MapperParams * map_params, MilocConfig * config,
  std::vector<RealCamera> * cameras)
{
  if (nullptr == map_params || nullptr == cameras) {
    LOG(ERROR) << "nullptr error ";
    return SLAM_NULL_PTR;
  }

  map_params_ = map_params;
  config_ = config;
  cameras_ = cameras;
  status_ = 1;

  return SLAM_OK;
}

int RestructMapper::ConstructMap(const std::string & map_url)
{
  if (status_ != 1) {
    LOG(ERROR) << "Bad mapper status";
    return SLAM_ERROR;
  }

  int ret = SLAM_OK;
  std::string image_folder = map_url + "/images";
  std::vector<std::string> image_list;
  std::unordered_set<std::string> time_stamps;

  ret = GetImageList(image_folder, image_list, time_stamps);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Get image list error";
    return ret;
  }
  std::vector<std::string> time_stamps_vec;
  std::vector<std::string> fields;
  for (auto iter = time_stamps.begin(); iter != time_stamps.end(); ++iter) {
    fields.clear();
    std::string time_stamp = *iter;
    boost::split(fields, time_stamp, boost::is_any_of("."));
    time_stamp = fields[0] + "." + fields[1];
    time_stamps_vec.push_back(time_stamp);
  }
  std::sort(
    time_stamps_vec.begin(), time_stamps_vec.end(), [](std::string time1, std::string time2) {
      return time1 < time2;
    });

  const int image_num = image_list.size();
  LOG(INFO) << "find total image num " << image_num << " for 3 cameras!";

  // get the colmap cameras and images
  std::vector<colmap::Camera> colmap_cameras;
  ret |= GetColmapCameras(*cameras_, colmap_cameras);
  std::string tra_file = map_url + "/visual/trajectory.txt";
  std::string database_dir = map_url + "/visual";
  std::vector<colmap::Image> colmap_images;
  std::unordered_map<std::string, int> name_map;

  std::map<std::string, std::pair<Trans, Quat>> traj;
  std::map<std::string, std::pair<Trans, Quat>> traj_interpolated;
  ret |= GetTrajectory(tra_file, traj);
  InterpolateTraj(time_stamps_vec, traj, traj_interpolated);
  FilterImages(
    time_stamps_vec, traj_interpolated, config_->mapper_params.translation_interval,
    config_->mapper_params.orientation_interval);
  ret |= ReadColmapImages(traj_interpolated, time_stamps, *cameras_, colmap_images, name_map);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Read colmap images error";
    return ret;
  }
  LOG(INFO) << "get colmap image num " << colmap_images.size();
  if (colmap_images.size() <= 6) {
    LOG(ERROR) << "colmap image num too small";
    return SLAM_ERROR;
  }

  // get colmap database handle
  SparseMapDB * map_db = new SparseMapDB();
  LOG(INFO) << "Open database: " << database_dir + "/database.db";
  map_db->Open(database_dir + "/database.db");
  map_db->CreateDbTables();

  // save cameras to colmap database
  ret = map_db->WriteCameras(colmap_cameras);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Save cameras failed.";
    return ret;
  }

  // save images to colmap database
  map_db->WriteImages(colmap_images, true);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Save images failed.";
    return ret;
  }

  name_map.clear();
  map_db->GetImageIds(name_map);

  std::vector<std::string> name_pair_list;
  const std::string golbal_data_dir = database_dir + "/";

  LOG(INFO) << "Start extracting local and global features.";
  ret = ExtractFeatures(
    map_url, image_list, map_db, name_map, golbal_data_dir, global_model_,
    local_model_);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Extract features failed.";
    return ret;
  }

  ret = ExtractMatches(
    image_list, map_params_->num_matched, name_pair_list,
    map_db, name_map, match_model_);
  LOG(INFO) << "Start matching global features with " << name_pair_list.size() << " pairs.";
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Extract matches failed.";
    return ret;
  }

  map_db->Close();
  delete map_db;
  map_db = nullptr;

  ret = SaveNamePairs(database_dir, name_pair_list);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Save name pairs error";
    return ret;
  }

  LOG(INFO) << "Start verifying matches.";
  VerifyMatch(database_dir);

  LOG(INFO) << "Start point triangulation.";
  ret = RunTriangulation(database_dir, colmap_cameras, colmap_images, database_dir, *map_params_);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Run triangulation error";
    return ret;
  }

  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
