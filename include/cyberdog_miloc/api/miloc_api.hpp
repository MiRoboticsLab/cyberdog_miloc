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

#ifndef CYBERDOG_MILOC__API__MILOC_API_HPP_
#define CYBERDOG_MILOC__API__MILOC_API_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <array>

namespace cyberdog
{
namespace miloc
{

// Errorno
#define SLAM_OK                   (0)         // everithing is ok
#define SLAM_ERROR                (-1)        // func call error
#define SLAM_IO_ERROR             (-2)        // io error
#define SLAM_NO_MEM               (-3)        // insufficient memory
#define SLAM_NULL_PTR             (-4)        // null pointer
#define SLAM_BAD_ARG              (-5)        // arg/param is bad
#define SLAM_BAD_OPT              (-6)        // bad operation

class MilocApi
{
public:
  // Start up Miloc service
  int StartUp(const std::string & config_file);

  // Choose visual or laser SLAM system
  int SetSLAM(bool visual_slam);

  // get current SLAM system
  bool GetSLAM();

  int DeleteRelocMap(int map_id);

  int GetRelocMapStatus(int map_id);

  // Estimate current pose with images and camera info
  //      camera_idxs stands for which camera capture the image
  //      orientation, position is the pose output, inline_num is the inline number
  int EstimatePose(
    const std::vector<cv::Mat> & images, Eigen::Quaterniond & orientation,
    Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation_init, const Eigen::Vector3d & position_init,
    int & reply_status, float & confidence);

  // Create a new map
  int CreateMap(int & map_id);

  // Finish a new map
  int FinishMap();

  // Start to navigate
  int StartNavigation();

  // Stop navigating
  int StopNavigation();

  // Collect images
  //        timastamp is for current images, camera_idxs stands for which camera capture the images
  int CollectImage(
    uint64_t timestamp, const std::vector<cv::Mat> & images,
    const std::vector<int> & camera_idxs);

  // Get current server status
  int GetMilocStatus();

  // Get current map id
  int GetCurrentMapId();

  // Shut down Miloc service
  int ShutDown();

  int ReconstructMap(const int map_id, const std::string & map_name);
};  // class MilocApi
}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__API__MILOC_API_HPP_
