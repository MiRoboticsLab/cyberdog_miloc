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

#ifndef CYBERDOG_MILOC__MISC__MILOC_MISC_HPP_
#define CYBERDOG_MILOC__MISC__MILOC_MISC_HPP_

#include <pthread.h>

#include <vector>
#include <string>
#include <array>
#include <thread>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <memory>

#include <Eigen/Core>
#include <colmap/lib/SQLite/sqlite3.h>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>

#include "base/miloc_restruct.hpp"
#include "base/miloc_estimate.hpp"
#include "runtime/miloc_trt_runtime.hpp"
#include "base/miloc_io.hpp"
#include "base/miloc_online_handler.hpp"
#include "base/miloc_types.hpp"
#include "utils/miloc_database.hpp"

namespace cyberdog
{
namespace miloc
{

int LoadCameras(std::vector<RealCamera> & cameras, const std::string & file_name);

int LoadConfig(MilocConfig & config, const std::string & file_name);

class MiLocServicer
{
public:
  MiLocServicer()
  : status_(MilocStatus::kUNINITIALIZED), map_id_(-1), load_map_id_(-1), create_map_id_(-1),
    is_creating_map_(false), is_navigating_(false), cyberdb_(nullptr)
  {
  }
  ~MiLocServicer();
  // init MiLocServicer with yaml file, include:
  //      1 open cyber database
  //      2 init three model class, load model file, set model params
  //      3 set camera and other config params
  //      4 init mapper and estimator class param
  int InitServer(const std::string & config_file);

  // choose visual or laser SLAM system
  int SetSLAM(bool visual_slam);

  // get current SLAM system
  bool GetSLAM();

  // query map info with map_id
  //      map_url is the map data path, map_status stands for query map status, 2 means done
  int QueryMapInfo(int map_id, std::string & map_url, int & map_status);

  int ReconstructMap(int map_id, const std::string & out_map_name);

  // construct map with map_id, out_map_name
  //      map_id specify which map to construct, out_map_name is the map name for done map
  int ReconstructMapCore(int map_id, const std::string & out_map_name);

  // set current load map
  //      map_id specify which map to load, mode should be 1 now
  int LoadMap(int map_id);

  int LoadModel();

  int UnLoadMap();

  // estimate current pose with images and camera info
  int EstimatePose(
    const std::vector<RawImage> & images, const std::vector<int> & camera_idxs,
    Quat & quat, Trans & trans);

  // Create a new map
  int CreateMap(int & current_map_id);

  // Finish a new map
  int FinishMap();

  int StartNavigation();

  int StopNavigation();

  // Collect images
  //        timastamp is for current images, camera_idxs stands for which camera capture the images
  int CollectImage(
    uint64_t timestamp, const std::vector<cv::Mat> & images,
    const std::vector<int> & camera_idxs);

  // get current server status
  int GetStatus() {return status_;}

  void SetStatus(int status) {status_ = status;}

  // get current map id
  int GetMapId() {return map_id_;}

  int CheckResult(
    Eigen::Vector3d & position_reloc, const Eigen::Vector3d & position_vio,
    const Eigen::Quaterniond & quat_vio, float & confidence);

  int DeleteRelocMap(int map_id);

  int GetRelocMapStatus(int map_id);

  OnlineHandler & GetOnlineHandler() {return online_handler_;}

  std::unordered_map<int, std::string> & GetCameraId2Name()
  {
    return camera_id2names_;
  }

private:
  int QueryMapStatus(int map_id);
  int UpdateMapDb(int map_id, const std::string & map_name);
  // int CreateLogDb(int map_id);
  void ReleaseModel();

  __volatile__ int status_;

  int map_id_;
  int load_map_id_;
  int create_map_id_;
  std::string map_name_;
  std::string map_url_;
  int map_status_;
  int map_scene_;

  // Whether to create a new map or navigate
  bool is_creating_map_;
  bool is_navigating_;

  MilocConfig config_;
  CyberDB * cyberdb_ = nullptr;
  std::vector<RealCamera> cameras_;
  std::unordered_map<int, std::string> camera_id2names_;

  GlobalModel * gmodel_;
  LocalModel * lmodel_;
  MatchModel * mmodel_;

  RestructMapper mapper_;
  Estimator estimator_;
  OnlineHandler online_handler_;

  std::unique_ptr<std::thread> m_contruct_thread_;
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__MISC__MILOC_MISC_HPP_
