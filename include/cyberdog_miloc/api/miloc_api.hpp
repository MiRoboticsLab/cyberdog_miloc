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

/**
 * @brief miloc core api
 * 
 */
class MilocApi
{
public:
  /**
   * @brief Start up Miloc service
   * 
   * @param config_file config file path
   * @return SLAM_OK for startup success and others for startup fail
   */
  int StartUp(const std::string & config_file);
  /**
   * @brief Choose visual or laser SLAM system
   * 
   * @param visual_slam 1 for visual_slam 0 for laser slam
   * @return SLAM_OK for startup success and others for startup fail 
   */
  int SetSLAM(bool visual_slam);

  // get current SLAM system
  bool GetSLAM();
  
  int DeleteRelocMap(int map_id);

  int GetRelocMapStatus(int map_id);

  /**
   * @brief Estimate current pose with images and camera info
   * 
   * @param images input triple camrea images
   * @param orientation one output of the current pose
   * @param position one output of the current pose
   * @param orientation_init input orientation from slam system
   * @param position_init input position from slam system
   * @param reply_status output status defined in base/miloc_types.hpp
   * @param confidence output confidence of current pose
   * @return SLAM_OK for startup success and others for startup fail 
   */
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

  /**
   * @brief Collect images
   * 
   * @param timestamp  current images timastamp
   * @param images current images data
   * @param camera_idxs camera_idxs
   * @return SLAM_OK for startup success and others for startup fail
   */
  int CollectImage(
    uint64_t timestamp, const std::vector<cv::Mat> & images,
    const std::vector<int> & camera_idxs);

  // Get current miloc server status
  int GetMilocStatus();

  // Get current map id
  int GetCurrentMapId();

  // Shut down Miloc service
  int ShutDown();
  /**
   * @brief reconstruct map
   * 
   * @param map_id 
   * @param map_name 
   * @return SLAM_OK for startup success and others for startup fail
   */
  int ReconstructMap(const int map_id, const std::string & map_name);
};  // class MilocApi
}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__API__MILOC_API_HPP_
