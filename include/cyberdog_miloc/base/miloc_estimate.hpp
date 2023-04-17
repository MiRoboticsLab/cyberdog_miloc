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

#ifndef CYBERDOG_MILOC__BASE__MILOC_ESTIMATE_HPP_
#define CYBERDOG_MILOC__BASE__MILOC_ESTIMATE_HPP_

#include <stdio.h>
#include <string>
#include <vector>
#include <unordered_map>

#include "runtime/miloc_trt_runtime.hpp"
#include "base/miloc_types.hpp"
#include "utils/miloc_database.hpp"
#include "utils/miloc_kdtree.hpp"
#include "utils/miloc_convertor.hpp"

namespace cyberdog
{
namespace miloc
{

class Estimator
{
public:
  Estimator()
  : status_(-1), m_image_num(0), m_success_num(0), tree_(nullptr)
  {
  }

  int Init(std::vector<RealCamera> * cameras, MilocConfig * config_all);

  int InitModel(LocalModel * lmodel, GlobalModel * gmodel, MatchModel * mmodel)
  {
    local_model_ = lmodel;
    global_model_ = gmodel;
    match_model_ = mmodel;
    return SLAM_OK;
  }

  void ReleaseModel()
  {
    local_model_ = nullptr;
    global_model_ = nullptr;
    match_model_ = nullptr;
  }

  int EstimatePose(
    const std::vector<RawImage> & images, const std::vector<int> & camera_idxs, Quat & quat,
    Trans & trans);
  //  set config
  void SetConfig(const SceneParam & scene_param)
  {
    config_ = scene_param;
  }

  int CheckResult(
    Eigen::Vector3d & position_reloc, const Eigen::Vector3d & position_vio,
    const Eigen::Quaterniond & quat_vio, float & confidence);

  int LoadMap(const std::string & map_url);
  int LoadPreMap();
  int ReleaseMap();

private:
  int status_;

  int m_image_num;
  int m_success_num;

  LocalModel * local_model_;
  GlobalModel * global_model_;
  MatchModel * match_model_;

  MilocConfig * config_all_;
  KDTree * tree_;

  std::vector<RealCamera> * cameras_;

  GlobalFeatureMap map_global_features_;
  std::vector<DbImage> images_;
  std::vector<DbPoints3D> points_3d_;

  std::unordered_map<std::string, int> name_map_;    // image_name to index
  std::unordered_map<int, int> id_map_;    // point3d_id to index
  std::unordered_map<int, int> image_id_map_;    // image_id to index

  std::string map_url_;

  SceneParam config_;

  int CameraSelection(
    const std::vector<RawImage> & images,
    std::vector<std::vector<int>> & clusters,
    std::vector<int> & sort_idx);

  int GetFeatures(
    const std::vector<RawImage> & images, std::vector<int> & best_cluster,
    std::vector<MatchData> & match_datas, LocalFeatureData & local_feature,
    int & image_idx);

  int LoadReconstruction();
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__BASE__MILOC_ESTIMATE_HPP_
