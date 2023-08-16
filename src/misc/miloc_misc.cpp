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

#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <chrono>
#include <malloc.h>

#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "misc/miloc_misc.hpp"
#include "utils/miloc_basic.hpp"
#include "base/miloc_colmap_helper.hpp"
#include "utils/miloc_convertor.hpp"
#include "cuda_runtime.h"

namespace cyberdog
{
namespace miloc
{

static int GetCameraType(const char * model_str)
{
  if (!strcmp(model_str, "OPENCV")) {
    return 4;
  } else if (!strcmp(model_str, "PINHOLE")) {
    return 1;
  } else if (!strcmp(model_str, "MEI_FISHEYE")) {
    return 9;
  } else {
    return -1;
  }
}

int LoadCameras(std::vector<RealCamera> & cameras, const std::string & file_name)
{
  std::ifstream json_buffer(file_name);
  std::string json_str((std::istreambuf_iterator<char>(json_buffer)),
    std::istreambuf_iterator<char>());
  rapidjson::Document document;
  document.Parse(json_str.c_str());

  RealCamera camera;
  std::vector<std::string> camera_names = {"front", "left", "right"};

  for (auto item : camera_names) {
    if (document.HasMember(item.c_str())) {
      camera.prefix = item;
      const rapidjson::Value & child_value = document[item.c_str()];
      rapidjson::Value::ConstMemberIterator child_iter = child_value.FindMember("camera_id");
      if (child_iter != child_value.MemberEnd()) {
        camera.camera_id = child_iter->value.GetInt();
      }
      child_iter = child_value.FindMember("camera_model");
      if (child_iter != child_value.MemberEnd()) {
        camera.camera_type = GetCameraType(child_iter->value.GetString());
        camera.model_name = child_iter->value.GetString();
      }
      child_iter = child_value.FindMember("width");
      if (child_iter != child_value.MemberEnd()) {
        camera.width = child_iter->value.GetInt();
      }
      child_iter = child_value.FindMember("height");
      if (child_iter != child_value.MemberEnd()) {
        camera.height = child_iter->value.GetInt();
      }
      if (child_value.HasMember("camera_params")) {
        const rapidjson::Value & child2_value = child_value["camera_params"];
        camera.camera_intrinsic.clear();
        for (rapidjson::SizeType i = 0; i < child2_value.Size(); ++i) {
          camera.camera_intrinsic.push_back(child2_value[i].GetDouble());
        }
      }
      if (child_value.HasMember("transform")) {
        const rapidjson::Value & child2_value = child_value["transform"];
        for (rapidjson::SizeType i = 0; i < child2_value.Size(); ++i) {
          const rapidjson::GenericArray<true,
            rapidjson::Value> & array_value = child2_value[i].GetArray();
          int j = 0;
          for (auto iter = array_value.Begin(); iter != array_value.End(); ++iter, ++j) {
            camera.camera_extrinsic[i * 4 + j] = iter->GetDouble();
          }
        }
      }
      cameras.push_back(camera);
    }
  }

  return SLAM_OK;
}

int LoadConfig(MilocConfig & config, const std::string & file_name)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(file_name);
  } catch (...) {
    LOG(ERROR) << "Load yaml error";
    return SLAM_ERROR;
  }

  config.position_threshold = root["position_threshold"].as<double>(1.0);
  config.mapper_params.translation_interval =
    root["mapper_params"]["translation_interval"].as<double>(0.2);
  config.mapper_params.orientation_interval =
    root["mapper_params"]["orientation_interval"].as<double>(M_PI / 6);
  config.mapper_params.max_reproj_error = root["mapper_params"]["max_reproj_error"].as<double>(4.);
  config.mapper_params.min_tri_angle = root["mapper_params"]["min_tri_angle"].as<double>(1.5);
  config.mapper_params.min_track_len = root["mapper_params"]["min_track_len"].as<double>(2.);
  config.mapper_params.num_matched = root["mapper_params"]["num_matched"].as<int>(5);

  config.camera_model_path = boost::filesystem::path(file_name).parent_path().string() +
    "/camera_model.json";

  config.map_path = root["map_path"].as<std::string>("./maps");
  config.map_table_name = root["map_table_name"].as<std::string>("reloc_map");
  config.local_model_path = root["model_path"]["local_model_path"].as<std::string>(
    "./models/local_model.trt");
  config.global_model_path = root["model_path"]["global_model_path"].as<std::string>(
    "./models/global_model.trt");
  config.match_model_path = root["model_path"]["match_model_path"].as<std::string>(
    "./models/match_model.trt");

  config.local_model_params.mask_th = root["local_model_params"]["mask_th"].as<float>(0.2);

  config.reloc_param.retrieval_num = root["reloc_params"]["retrieval_num"].as<int>(10);
  config.reloc_param.inlier_num = root["reloc_params"]["inlier_num"].as<int>(10);
  config.reloc_param.ransac_thresh = root["reloc_params"]["ransac_thresh"].as<int>(12);

  config.traj_from_visual = root["traj_from_visual"].as<bool>(true);
  config.confidence = root["confidence"].as<std::vector<float>>();

  return SLAM_OK;
}

int MiLocServicer::InitServer(const std::string & config_file)
{
  if (status_ == MilocStatus::kINACTIVE) {
    LOG(INFO) << "MiLocServicer init already!";
    return SLAM_OK;
  }
  status_ = MilocStatus::kINITIALIZING;

  int ret = SLAM_OK;
  LOG(INFO) << "load config " << config_file;
  ret = LoadConfig(config_, config_file);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "LoadConfig fail";
    return ret;
  }
  LOG(INFO) << "load config " << config_file << "success!";

  if (!IsValidDir(config_.map_path)) {
    boost::filesystem::create_directories(config_.map_path);
  }

  std::string lidar_odom_path{config_.map_path + "/default/lidar"};
  std::string visual_odom_path{config_.map_path + "/default/visual"};

  if (!IsValidDir(lidar_odom_path)) {
    boost::filesystem::create_directories(lidar_odom_path);
  }
  if (!IsValidDir(visual_odom_path)) {
    boost::filesystem::create_directories(visual_odom_path);
  }

  std::string db_path = config_.map_path + "/cyberdog.db";
  LOG(INFO) << "load database " << db_path;

  cyberdb_ = new CyberDB();
  ret = cyberdb_->Open(db_path);
  ret |= cyberdb_->CreateMilocTable();

  LOG(INFO) << "load database " << db_path << "success!";

  LOG(INFO) << "load camera file " << config_.camera_model_path;
  if (!boost::filesystem::exists(config_.camera_model_path)) {
    return SLAM_ERROR;
  }

  ret = LoadCameras(cameras_, config_.camera_model_path);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "LoadCameras fail";
    return ret;
  }
  for (auto camera : cameras_) {
    camera_id2names_.insert(std::make_pair(camera.camera_id, camera.prefix));
  }
  LOG(INFO) << "load camera file " << config_.camera_model_path << " success!";

  LOG(INFO) << "Init estimator";
  estimator_.Init(&cameras_, &config_);

  LOG(INFO) << "Init mapper";
  mapper_.Init(&config_.mapper_params, &config_, &cameras_);

  LOG(INFO) << "Init online handler";
  online_handler_.Init(config_.map_table_name, cyberdb_, config_.map_path, &cameras_);

  LOG(INFO) << "Init server Done, miloc status INACTIVE";

  status_ = MilocStatus::kINACTIVE;
  return ret;
}

int MiLocServicer::LoadModel()
{
  if (is_model_loaded_){
    LOG(INFO) << "model is already loaded";
    return SLAM_OK;
  }

  int ret = SLAM_OK;
  LOG(INFO) << "load  model file " << config_.global_model_path;
  gmodel_ = new GlobalModel();
  ret |= gmodel_->Load(config_.global_model_path);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Load global model fail";
    return ret;
  }
  LOG(INFO) << "load  gobal model success!";

  LOG(INFO) << "load  model file " << config_.local_model_path;
  lmodel_ = new LocalModel();
  ret |= lmodel_->Load(config_.local_model_path);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Load local model fail";
    return ret;
  }
  lmodel_->SetConfig(config_.local_model_params);
  LOG(INFO) << "load  local model success!";

  LOG(INFO) << "load  model file " << config_.match_model_path;
  mmodel_ = new MatchModel();
  ret |= mmodel_->Load(config_.match_model_path);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Load match model fail";
    return ret;
  }
  LOG(INFO) << "load  model match success!";
  is_model_loaded_ = true;

  return ret;
}

int MiLocServicer::SetSLAM(bool visual_slam)
{
  config_.SetTrajFromVisual(visual_slam);
  return SLAM_OK;
}

bool MiLocServicer::GetSLAM()
{
  return config_.GetTrajFromVisual();
}

int MiLocServicer::QueryMapStatus(int map_id)
{
  MapInfo map_info;
  int ret = cyberdb_->ReadMapInfo(map_id, map_info);
  if (ret != SLAM_OK) {
    return ret;
  }

  map_name_ = map_info.map_name;
  map_url_ = map_info.map_url;
  map_status_ = map_info.status;
  map_scene_ = map_info.scene;
  map_id_ = map_id;

  return SLAM_OK;
}

int MiLocServicer::QueryMapInfo(int map_id, std::string & map_url, int & map_status)
{
  int ret = SLAM_OK;
  ret = QueryMapStatus(map_id);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "QueryMapStatus fail";
    return ret;
  }

  map_url = map_url_;
  map_status = map_status_;

  return SLAM_OK;
}

int MiLocServicer::ReconstructMap(int map_id, const std::string & out_map_name)
{
  int ret = SLAM_OK;

  if (status_ == MilocStatus::kRECONSTRUCTING) {
    ret |= pthread_cancel(m_contruct_thread_->native_handle());
  }

  status_ = MilocStatus::kRECONSTRUCTING;

  if (-1 != load_map_id_) {
    ret |= UnLoadMap();
  }
  ret = QueryMapStatus(map_id);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "QueryMapStatus fail";
    status_ = MilocStatus::kINACTIVE;
    return ret;
  }

  if (map_status_ == 2) {
    LOG(INFO) << "RestructMap for map " << map_id_ << " already done!";
    status_ = MilocStatus::kINACTIVE;
    return SLAM_BAD_ARG;
  }

  if (!boost::filesystem::exists(map_url_) ||
    !boost::filesystem::exists(map_url_ + "/images") ||
    !boost::filesystem::exists(map_url_ + "/visual") ||
    !boost::filesystem::exists(map_url_ + "/visual/trajectory.txt") ||
    boost::filesystem::is_empty(map_url_ + "/images/left/"))
  {
    LOG(ERROR) << "Map data is not found";
    status_ = MilocStatus::kINACTIVE;
    return SLAM_IO_ERROR;
  }

  create_map_id_ = map_id;
  m_contruct_thread_ = std::unique_ptr<std::thread>(
    new std::thread(&MiLocServicer::ReconstructMapCore, this, map_id, out_map_name));

  return ret;
}

int MiLocServicer::ReconstructMapCore(int map_id, const std::string & out_map_name)
{
  int ret = SLAM_OK;
  LOG(INFO) << "Load Model";
  ret |= LoadModel();
  ret |= mapper_.InitModel(lmodel_, gmodel_, mmodel_);
  ret |= mapper_.ConstructMap(map_url_);
  LOG(INFO) << "Reconstruct map done";

  if (ret != SLAM_OK) {
    LOG(ERROR) << "ConstructMap fail";
    create_map_id_ = -1;
    status_ = MilocStatus::kINACTIVE;
    cyberdb_->SetMapStatus(map_id, 0);
    LOG(INFO) << "release model";
    ReleaseModel();
    mapper_.ReleaseModel();
    create_map_id_ = -1;
    m_contruct_thread_->detach();
    LOG(INFO) << "Miloc status INACTIVE";
    return ret;
  }

  std::string map_name = out_map_name.size() > 0 ? out_map_name : "map_" + std::to_string(map_id);
  ret = UpdateMapDb(map_id, map_name);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "UpdateMapDb fail";
  }

  LOG(INFO) << "release Model";
  ReleaseModel();
  mapper_.ReleaseModel();

  LOG(INFO) << "Miloc status INACTIVE";
  status_ = MilocStatus::kINACTIVE;
  create_map_id_ = -1;
  m_contruct_thread_->detach();
  return ret;
}

int MiLocServicer::UpdateMapDb(int map_id, const std::string & map_name)
{
  std::string safe_check_file = map_url_ + "/visual/database.db";
  if (!IsValidFile(safe_check_file)) {
    LOG(ERROR) << "Database file not exists";
    return SLAM_ERROR;
  }

  std::string old_dir = map_url_ + "/visual";
  std::string old_dir_lidar = map_url_ + "/lidar";
  std::string new_dir = config_.map_path + "/map_" + std::to_string(map_id);
  char cmd_buffer[256];

  if (!IsValidDir(new_dir)) {
    snprintf(cmd_buffer, sizeof(cmd_buffer), "mkdir -p %s", new_dir.c_str());
    if (system(cmd_buffer) != 0) {
      LOG(ERROR) << "mkdir fail";
      return SLAM_ERROR;
    }
  }

  snprintf(cmd_buffer, sizeof(cmd_buffer), "mv %s %s", old_dir.c_str(), new_dir.c_str());
  if (system(cmd_buffer) != 0) {
    LOG(ERROR) << "mv fail";
    return SLAM_ERROR;
  }

  snprintf(cmd_buffer, sizeof(cmd_buffer), "mv %s %s", old_dir_lidar.c_str(), new_dir.c_str());
  if (system(cmd_buffer) != 0) {
    LOG(ERROR) << "mv fail";
    return SLAM_ERROR;
  }

  if (!boost::filesystem::remove_all(map_url_)) {
    return SLAM_ERROR;
  }

  map_id_ = map_id;
  map_name_ = map_name;
  map_url_ = new_dir;
  map_status_ = 2;

  std::string name_in_db;
  int ret = cyberdb_->GetMapName(map_id, name_in_db);
  std::string default_name = "unfinished_map_" + std::to_string(map_id);
  if (name_in_db != default_name) {
    ret |= cyberdb_->UpdateMapInfo(map_id_, name_in_db, map_url_, config_.map_table_name);
  } else {
    ret |= cyberdb_->UpdateMapInfo(map_id_, map_name_, map_url_, config_.map_table_name);
  }

  return ret;
}

int MiLocServicer::LoadMap(int map_id)
{
  if (map_id == load_map_id_) {
    LOG(INFO) << "map " << map_id << " already load, skip!";
    return SLAM_OK;
  } else {
    if (-1 != load_map_id_) {
      UnLoadMap();
    }
  }

  int ret = SLAM_OK;
  ret = QueryMapStatus(map_id);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "QueryMapStatus fail";
    return ret;
  }

  if (map_status_ != 2) {
    LOG(ERROR) << "map_status_ error, map is not available";
    return SLAM_ERROR;
  }

  load_map_id_ = map_id;
  estimator_.SetConfig(config_.reloc_param);

  ret = estimator_.LoadMap(map_url_ + "/visual/");
  if (ret != SLAM_OK) {
    LOG(ERROR) << "ReadDb fail";
    return ret;
  }

  return SLAM_OK;
}

int MiLocServicer::UnLoadMap()
{
  int ret = estimator_.ReleaseMap();
  if (ret == SLAM_OK) {
    load_map_id_ = -1;
  }
  return ret;
}

int MiLocServicer::EstimatePose(
  const std::vector<RawImage> & images, const std::vector<int> & camera_idxs, Quat & quat,
  Trans & trans)
{
  if (!is_navigating_) {
    return SLAM_BAD_ARG;
  }

  int ret = SLAM_OK;
  ret = estimator_.EstimatePose(images, camera_idxs, quat, trans);

  return ret;
}

int MiLocServicer::CreateMap(int & current_map_id)
{
  if (!is_creating_map_) {
    int map_num = 0;
    int ret = cyberdb_->GetMapNums(map_num);
    if (ret != SLAM_OK) {
      return SLAM_ERROR;
    }

    if (map_num >= 1) {
      std::vector<int> map_ids;
      ret |= cyberdb_->GetAllMapIds(map_ids);
      for (auto map_id : map_ids) {
        std::cout << map_id << std::endl;
        ret |= DeleteRelocMap(map_id);
      }
    }

    if (ret != SLAM_OK) {
      return SLAM_ERROR;
    }

    is_creating_map_ = true;
    bool slam_flag = config_.GetTrajFromVisual();
    online_handler_.CreateMap(slam_flag, current_map_id);
    LOG(INFO) << "create map done, miloc status MAPPABLE, map_id: " << current_map_id;
    create_map_id_ = current_map_id;
    status_ = MilocStatus::kMAPPABLE;
  } else {
    LOG(ERROR) << "Something goes wrong, last map-creation is not finished properly";
  }
  return SLAM_OK;
}

int MiLocServicer::FinishMap()
{
  if (is_creating_map_) {
    is_creating_map_ = false;
    bool slam_flag = config_.GetTrajFromVisual();
    online_handler_.FinishMap(slam_flag);
    LOG(INFO) << "finish map done, miloc status INACTIVE";
    create_map_id_ = -1;
    status_ = MilocStatus::kINACTIVE;
  } else {
    LOG(ERROR) << "Something goes wrong, map-creation is not started properly";
  }
  return SLAM_OK;
}

int CheckValidUnfinishedMap(std::string & map_url)
{
  std::string traj_path = map_url + "/visual/trajectory.txt";
  boost::filesystem::path left_path{map_url + "/images/left"};
  boost::filesystem::path front_path{map_url + "/images/front"};
  boost::filesystem::path right_path{map_url + "/images/right"};

  if (IsValidFile(traj_path) &&
    !boost::filesystem::is_empty(left_path) &&
    !boost::filesystem::is_empty(front_path) &&
    !boost::filesystem::is_empty(right_path))
  {
    std::string new_traj_path = map_url + "/trajectory.txt";
    std::string viusal_path = map_url + "/visual";
    boost::filesystem::copy_file(traj_path, new_traj_path);
    boost::filesystem::remove_all(viusal_path);
    boost::filesystem::create_directories(viusal_path);
    boost::filesystem::copy_file(new_traj_path, traj_path);
    boost::filesystem::remove(new_traj_path);

    return true;
  } else {
    return false;
  }
}

int MiLocServicer::GetRelocMapStatus(int map_id)
{
  MapInfo map_info;
  cyberdb_->ReadMapInfo(map_id, map_info);
  if (2 == map_info.status) {
    if (IsValidFile(map_info.map_url + "/visual/cameras.bin") &&
      IsValidFile(map_info.map_url + "/visual/images.bin") &&
      IsValidFile(map_info.map_url + "/visual/points3D.bin") &&
      IsValidFile(map_info.map_url + "/visual/database.db") &&
      IsValidFile(map_info.map_url + "/visual/db_desc.raw") &&
      IsValidFile(map_info.map_url + "/visual/db_names.txt") &&
      IsValidFile(map_info.map_url + "/visual/name_pair.txt") &&
      IsValidFile(map_info.map_url + "/visual/trajectory.txt"))
    {
      return SLAM_OK;
    } else {
      DeleteRelocMap(map_id);
      return 302;
    }
  } else if (1 == map_info.status) {
    if (status_ == MilocStatus::kRECONSTRUCTING) {
      return 300;
    }
    if (CheckValidUnfinishedMap(map_info.map_url)) {
      return 301;
    } else {
      DeleteRelocMap(map_id);
      return 302;
    }
  } else {
    DeleteRelocMap(map_id);
    return 302;
  }
}

int MiLocServicer::DeleteRelocMap(int map_id)
{
  int ret = SLAM_OK;

  if (status_ == MilocStatus::kRECONSTRUCTING) {
    if (m_contruct_thread_ != nullptr) {
      ret |= pthread_cancel(m_contruct_thread_->native_handle());
      LOG(INFO) << "Cancel reconstructing" << ret;
      LOG(INFO) << "release Model";
      if (mmodel_ != nullptr) {
        delete mmodel_;
        mmodel_ = nullptr;
      }
      if (lmodel_ != nullptr) {
        delete lmodel_;
        lmodel_ = nullptr;
      }
      if (gmodel_ != nullptr) {
        delete gmodel_;
        gmodel_ = nullptr;
      }
      mapper_.ReleaseModel();
      status_ = MilocStatus::kINACTIVE;
      LOG(INFO) << "Miloc status INACTIVE";
    }
  }

  boost::filesystem::path map_path{config_.map_path + "/map_" + std::to_string(map_id)};
  boost::filesystem::path unfinished_map_path{config_.map_path + "/default/unfinished_map_" +
    std::to_string(map_id)};

  LOG(INFO) << "Delete map file";
  if (boost::filesystem::exists(map_path)) {
    boost::filesystem::remove_all(map_path);
  }
  if (boost::filesystem::exists(unfinished_map_path)) {
    boost::filesystem::remove_all(unfinished_map_path);
  }

  LOG(INFO) << "Delete map database";
  ret |= cyberdb_->DeleteMap(map_id);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Delete map failed";
    return SLAM_ERROR;
  }
  status_ = MilocStatus::kINACTIVE;

  return ret;
}

int MiLocServicer::CollectImage(
  uint64_t timestamp, const std::vector<cv::Mat> & images,
  const std::vector<int> & camera_idxs)
{
  online_handler_.ProcessImage(timestamp, images, camera_idxs);
  return SLAM_OK;
}

int MiLocServicer::StartNavigation()
{
  if (is_navigating_ == true) {
    LOG(ERROR) << "navigation is already started";
    return SLAM_ERROR;
  }

  if (-1 == load_map_id_) {
    LOG(INFO) << "start load map";
    if (LoadMap(1) != SLAM_OK) {
      LOG(INFO) << "load map error, map is not available..";
      return SLAM_ERROR;
    } else {
      LOG(INFO) << "load map success";
    }
  }

  is_navigating_ = true;
  status_ = MilocStatus::kLOADING;
  // load deep learning model
  int ret = SLAM_OK;
  LOG(INFO) << "start load model";
  ret |= LoadModel();
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Load model error";
    is_navigating_ = false;
    status_ = MilocStatus::kINACTIVE;
    return SLAM_ERROR;
  }
  LOG(INFO) << "load model success";
  estimator_.InitModel(lmodel_, gmodel_, mmodel_);
  LOG(INFO) << "start nav Done, Miloc status: RELOCALIZABLE";

  status_ = MilocStatus::kRELOCALIZABLE;
  return ret;
}

int MiLocServicer::StopNavigation()
{
  status_ = MilocStatus::kUNLOADING;
  if (is_navigating_ == false) {
    LOG(ERROR) << "navigation is not started";
    return SLAM_ERROR;
  }
  int ret = SLAM_OK;
  is_navigating_ = false;

  LOG(INFO) << "release model";

  ReleaseModel();
  estimator_.ReleaseModel();

  LOG(INFO) << "stop nav Done, Miloc status: INACTIVE";

  status_ = MilocStatus::kINACTIVE;

  return ret;
}

int MiLocServicer::CheckResult(
  Eigen::Vector3d & position_reloc,
  const Eigen::Vector3d & position_vio,
  const Eigen::Quaterniond & quat_vio, float & confidence)
{
  int result = estimator_.CheckResult(position_reloc, position_vio, quat_vio, confidence);
  return result;
}

void MiLocServicer::ReleaseModel()
{
  if (!is_model_loaded_){
    LOG(INFO) << "model is already release";
    return;
  }

  LOG(INFO) << "release model";
  
  if (mmodel_ != nullptr) {
    delete mmodel_;
    mmodel_ = nullptr;
  }
  if (lmodel_ != nullptr) {
    delete lmodel_;
    lmodel_ = nullptr;
  }
  if (gmodel_ != nullptr) {
    delete gmodel_;
    gmodel_ = nullptr;
  }

  is_model_loaded_ = false;
  
  int dev_count = 0;
  cudaSetDevice(dev_count);
  cudaDeviceReset();
  LOG(INFO) << "Cuda device reset complated. ";
  malloc_trim(0);
  LOG(INFO) << "Malloc trim complated. ";

}

MiLocServicer::~MiLocServicer()
{
  if (cyberdb_ != nullptr) {
    delete cyberdb_;
  }
  ReleaseModel();
}

}  // namespace miloc
}  // namespace cyberdog
