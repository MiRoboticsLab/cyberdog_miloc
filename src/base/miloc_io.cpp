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

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>
#include <unordered_set>
#include <utility>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "base/miloc_io.hpp"
#include "utils/miloc_basic.hpp"

namespace cyberdog
{
namespace miloc
{

int GetImageList(
  const std::string & dir_name, std::vector<std::string> & image_list,
  std::unordered_set<std::string> & time_stamps)
{
  std::string image_folder = dir_name + "/front";
  std::string left_folder = dir_name + "/left";
  std::string right_folder = dir_name + "/right";
  if (!IsValidDir(image_folder) && !IsValidDir(left_folder) && !IsValidDir(right_folder)) {
    LOG(ERROR) << "Invalid dir";
    return SLAM_ERROR;
  }

  if (IsValidDir(image_folder)) {
    ReadImageFolder(image_folder, image_list, time_stamps);
  }

  if (IsValidDir(left_folder)) {
    ReadImageFolder(left_folder, image_list, time_stamps);
  }

  if (IsValidDir(right_folder)) {
    ReadImageFolder(right_folder, image_list, time_stamps);
  }

  std::sort(image_list.begin(), image_list.end());

  return SLAM_OK;
}

int ReadImageFolder(
  const std::string & folder_name, std::vector<std::string> & image_list,
  std::unordered_set<std::string> & time_stamps)
{
  DIR * dir = opendir(folder_name.c_str());
  if (nullptr == dir) {
    LOG(ERROR) << "Open dir error";
    return SLAM_ERROR;
  }

  std::vector<std::string> fields;
  boost::split(fields, folder_name, boost::is_any_of("/"));
  dirent * cur_node = readdir(dir);
  while (cur_node != nullptr) {
    if (strcmp(".", cur_node->d_name) && strcmp("..", cur_node->d_name)) {
      std::string d_name = std::string(cur_node->d_name);
      time_stamps.insert(d_name);
      image_list.push_back(fields.back() + "/" + d_name);
    }
    cur_node = readdir(dir);
  }

  return SLAM_OK;
}

int LoadJpg(const std::string & file_name, RawImage & image)
{
  cv::Mat mat_raw = cv::imread(file_name, 0);
  cv::Mat mat;

  if (mat_raw.empty()) {
    LOG(ERROR) << "Image read error";
    return SLAM_ERROR;
  }

  image.raw_height = mat_raw.rows;
  image.raw_width = mat_raw.cols;

  if (mat.cols != 640 || mat.rows != 480) {
    cv::resize(mat_raw, mat, cv::Size(640, 480));
  } else {
    mat = mat_raw;
  }

  if (image.data != nullptr && image.width == mat.cols && image.height == mat.rows &&
    image.channel == mat.channels())
  {
    memcpy(image.data, mat.data, image.pitch * image.height);
  } else {
    if (image.data != nullptr) {
      free(image.data);
    }

    image.channel = mat.channels();
    image.width = mat.cols;
    image.height = mat.rows;
    image.elem_type = 1;
    image.pitch = image.width * image.channel;
    image.data = malloc(image.pitch * image.height);

    if (nullptr == image.data) {
      LOG(ERROR) << "Malloc error";
      return SLAM_NULL_PTR;
    }
    memcpy(image.data, mat.data, image.pitch * image.height);
  }

  return SLAM_OK;
}

void ReleaseImg(RawImage & image)
{
  if (image.data != nullptr) {
    free(image.data);
  }
}

bool IsValidDir(const std::string & dir_name)
{
  return !access(dir_name.c_str(), 0);
}

bool IsValidFile(const std::string & file_name)
{
  return !access(file_name.c_str(), 0);
}

std::string TimeStampFormat(const std::string & str, TimeStamp type)
{
  double time_val = atof(str.c_str());
  switch (type) {
    case TimeStamp::SECOND:
      break;
    case TimeStamp::MILLISECOND:
      time_val /= 1e3;
      break;
    case TimeStamp::MICROSECOND:
      time_val /= 1e6;
      break;
    case TimeStamp::NANOSECOND:
      time_val /= 1e9;
      break;
    default:
      ;
  }

  char str_val[64];
  snprintf(str_val, sizeof(str_val), "%.3f", time_val);

  return std::string(str_val);
}


int RawSave(GlobalFeatureMap & DATA, const std::string & file_name)
{
  std::string txt_name = file_name + "db_names.txt";
  std::string raw_name = file_name + "db_desc.raw";
  std::ofstream outname(txt_name.c_str(), std::ios::out);
  FILE * outdata = fopen(raw_name.c_str(), "wb");

  for (auto it = DATA.begin(); it != DATA.end(); it++) {
    // GlobalFeatureData data = DATA[i];
    outname << it->first << std::endl;
    // GlobalFeatureHalf feature_half;
    // miloc_half_impl::GlobalFeaturesHalfToFloat(it->second, feature_half);
    fwrite(it->second.data(), sizeof(float), it->second.size(), outdata);
  }
  outname.close();
  fclose(outdata);
  return SLAM_OK;
}

int RawLoad(const std::string & path_name, GlobalFeatureMap & DATA)
{
  std::string txt_name = path_name + "db_names.txt";
  std::string raw_name = path_name + "db_desc.raw";

  if (!boost::filesystem::exists(txt_name) || !boost::filesystem::exists(raw_name)) {
    return SLAM_ERROR;
  }

  std::ifstream inname(txt_name.c_str(), std::ios::in);
  FILE * indata = fopen(raw_name.c_str(), "rb");

  std::string temp;
  while (getline(inname, temp)) {
    // GlobalFeatureData data;
    // data.image_name = temp;
    // fread(data.feature.data(), sizeof(float), GLOBAL_FEATURE_SIZE, indata);
    // DATA.push_back(data);
    // GlobalFeatureHalf feature_half;
    GlobalFeature feature;
    fread(feature.data(), sizeof(float), GLOBAL_FEATURE_SIZE, indata);
    // miloc_half_impl::GlobalFeaturesHalfToFloat(feature, feature_half);
    DATA.insert(std::make_pair(temp, feature));
  }
  inname.close();
  fclose(indata);
  return SLAM_OK;
}

int GetTrajectory(std::string traj_path, std::map<std::string, std::pair<Trans, Quat>> & trajectory)
{
  LOG(INFO) << "Read trajectory: " << traj_path;
  if (access(traj_path.c_str(), F_OK) != 0) {
    LOG(ERROR) << "Trajectory path is not found";
    return SLAM_ERROR;
  }

  std::ifstream traj_file(traj_path);
  std::string line;
  std::vector<std::string> fields;
  while (getline(traj_file, line)) {
    boost::split(fields, line, boost::is_any_of(" "));
    if (fields.size() > 0) {
      // std::string time_stamp = TimeStampFormat(fields[0], TimeStamp::SECOND);
      trajectory.insert(
        std::make_pair(
          fields[0],
          std::make_pair(
            Trans(
              std::atof(fields[1].c_str()),
              std::atof(fields[2].c_str()),
              std::atof(fields[3].c_str())),
            Quat(
              std::atof(fields[7].c_str()),
              std::atof(fields[4].c_str()),
              std::atof(fields[5].c_str()),
              std::atof(fields[6].c_str())))));
    }
    fields.clear();
  }

  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
