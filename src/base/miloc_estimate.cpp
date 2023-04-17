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

#include <cmath>

#include <vector>
#include <utility>
#include <string>
#include <unordered_set>
#include <algorithm>
#include <queue>
#include <unordered_map>

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <colmap/base/reconstruction.h>

#include "base/miloc_estimate.hpp"
#include "base/miloc_colmap_helper.hpp"
#include "base/miloc_io.hpp"

namespace cyberdog
{
namespace miloc
{

static int GetPosition(
  point_t & position, const Eigen::Vector3d & tvec,
  const Eigen::Matrix3d & rotation, const std::array<double, 16> & exintrincs)
{
  Eigen::Matrix3d rotate = rotation.transpose();
  Eigen::Vector3d trans = -rotate * tvec;

  Eigen::Matrix4d tmp_mat = Eigen::Matrix4d::Identity();
  tmp_mat.topLeftCorner(3, 3) = rotate;
  tmp_mat.topRightCorner(3, 1) = trans;

  Eigen::Matrix4d ex_mat;
  ex_mat(0, 0) = exintrincs[0];
  ex_mat(0, 1) = exintrincs[1];
  ex_mat(0, 2) = exintrincs[2];
  ex_mat(0, 3) = exintrincs[3];
  ex_mat(1, 0) = exintrincs[4];
  ex_mat(1, 1) = exintrincs[5];
  ex_mat(1, 2) = exintrincs[6];
  ex_mat(1, 3) = exintrincs[7];
  ex_mat(2, 0) = exintrincs[8];
  ex_mat(2, 1) = exintrincs[9];
  ex_mat(2, 2) = exintrincs[10];
  ex_mat(2, 3) = exintrincs[11];
  ex_mat(3, 0) = exintrincs[12];
  ex_mat(3, 1) = exintrincs[13];
  ex_mat(3, 2) = exintrincs[14];
  ex_mat(3, 3) = exintrincs[15];

  tmp_mat = tmp_mat * ex_mat;
  trans = tmp_mat.topRightCorner(3, 1);
  position[0] = trans[0];
  position[1] = trans[1];
  position[2] = trans[2];

  return SLAM_OK;
}

int Estimator::LoadReconstruction()
{
  // db_positions for kdtree
  if (!boost::filesystem::exists(map_url_)) {
    return SLAM_ERROR;
  }

  std::vector<point_t> db_positions;

  colmap::Reconstruction * map_model = new colmap::Reconstruction();
  map_model->ReadBinary(map_url_);
  int index = 0;
  for (auto & iter : map_model->Images()) {
    name_map_.insert(std::make_pair(iter.second.Name(), index));
    DbImage db_image;

    std::vector<float> tvec(iter.second.Tvec().data(), (iter.second.Tvec().data() + 3));
    std::vector<float> qvec(iter.second.Qvec().data(), (iter.second.Qvec().data() + 4));

    for (auto & point2d : iter.second.Points2D()) {
      if (point2d.HasPoint3D()) {
        db_image.point3d_ids.emplace_back(point2d.Point3DId());
      } else {
        db_image.point3d_ids.emplace_back(-1);
      }
      Point2f p;
      p[0] = point2d.X();
      p[1] = point2d.Y();
      db_image.xys.emplace_back(p);
    }

    db_image.image_id = iter.first;
    db_image.name = iter.second.Name();
    db_image.qvec.swap(qvec);
    db_image.tvec.swap(tvec);
    images_.emplace_back(db_image);
    index++;

    point_t position{0., 0., 0.};
    int result = GetPosition(
      position, iter.second.Tvec(),
      iter.second.RotationMatrix(), cameras_->at(
        iter.second.CameraId() - 1).camera_extrinsic);
    if (result != SLAM_OK) {
      LOG(ERROR) << "GetPosition fail";
      delete map_model;
      map_model = nullptr;
      return SLAM_ERROR;
    }
    db_positions.emplace_back(position);
  }

  // create kdtree
  tree_ = new KDTree(db_positions);

  index = 0;
  for (auto & iter : map_model->Points3D()) {
    DbPoints3D db_point;

    for (auto & track_elem : iter.second.Track().Elements()) {
      db_point.image_ids.emplace_back(track_elem.image_id);
      db_point.point2d_idxs.emplace_back(track_elem.point2D_idx);
    }

    db_point.xyz[0] = iter.second.X();
    db_point.xyz[1] = iter.second.Y();
    db_point.xyz[2] = iter.second.Z();

    db_point.point_id = iter.first;

    points_3d_.emplace_back(db_point);

    id_map_.insert(std::make_pair(iter.first, index));
    ++index;
  }

  delete map_model;
  map_model = nullptr;

  return SLAM_OK;
}

static void GernerateIdMap(
  const std::vector<DbImage> & images_, std::unordered_map<int,
  int> & image_id_map_)
{
  image_id_map_.clear();
  int image_num = images_.size();
  for (int i = 0; i < image_num; i++) {
    image_id_map_[images_[i].image_id] = i;
  }
}

int Estimator::Init(std::vector<RealCamera> * cameras, MilocConfig * config_all)
{
  if (nullptr == cameras) {
    LOG(ERROR) << "nullptr";
    return SLAM_NULL_PTR;
  }
  cameras_ = cameras;
  config_all_ = config_all;

  status_ = 1;

  return SLAM_OK;
}

int Estimator::ReleaseMap()
{
  map_global_features_.clear();
  images_.clear();
  points_3d_.clear();
  name_map_.clear();
  id_map_.clear();
  image_id_map_.clear();
  map_url_.clear();

  delete tree_;
  tree_ = nullptr;

  return SLAM_OK;
}

int Estimator::LoadMap(const std::string & map_url)
{
  if (0 == map_url.length()) {
    LOG(ERROR) << "map url is empty load map error";
    return SLAM_ERROR;
  }

  ReleaseMap();
  map_url_ = map_url;
  int ret = SLAM_OK;
  ret |= RawLoad(map_url_, map_global_features_);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "global Feature files not exist";
    return ret;
  }
  ret |= LoadReconstruction();
  if (ret != SLAM_OK) {
    LOG(ERROR) << "map files not exist";
    return ret;
  }
  GernerateIdMap(images_, image_id_map_);

  return ret;
}

static int SortByIndex(
  const std::vector<float> & v_data, const std::vector<int> & image_idxs,
  std::vector<float> & sort_data,
  std::vector<int> & sort_idxs, int k)
{
  int data_num = v_data.size();
  if (data_num < k) {
    LOG(ERROR) << "Bad data size";
    return SLAM_ERROR;
  }

  std::vector<int> v_idx(data_num);
  for (int i = 0; i < data_num; i++) {
    v_idx[i] = i;
  }
  std::sort(
    v_idx.begin(), v_idx.end(), [&v_data](int i1, int i2)
    {return v_data[i1] > v_data[i2];});

  sort_data.resize(k);
  sort_idxs.resize(k);
  for (int i = 0; i < k; i++) {
    sort_data[i] = v_data[v_idx[i]];
    sort_idxs[i] = image_idxs[v_idx[i]];
  }

  return SLAM_OK;
}

static int CovisibilityCluster(
  std::vector<int> & image_idxs, std::vector<float> & sims, std::vector<DbImage> & images,
  std::vector<DbPoints3D> & points_3d, std::unordered_map<int, int> & id_map,
  std::unordered_map<int, int> & image_id_map,
  std::vector<int> & best_cluster, float & best_score)
{
  int total_idxs_num = image_idxs.size();
  std::unordered_map<int, int> visit_flag;
  std::unordered_map<int, float> image_sims;
  for (int i = 0; i < total_idxs_num; i++) {
    visit_flag[image_idxs[i]] = 0;
    image_sims[image_idxs[i]] = sims[i];
  }

  unsigned int max_cluster_size = 0;
  for (int i = 0; i < total_idxs_num; i++) {
    int image_idx = image_idxs[i];
    if (visit_flag[image_idx] == 1) {
      continue;
    }

    std::vector<int> cur_idxs;
    std::vector<int> cluster;
    cur_idxs.push_back(image_idx);
    while (!cur_idxs.empty()) {
      image_idx = cur_idxs.back();
      cur_idxs.pop_back();
      if (visit_flag[image_idx] == 1) {
        continue;
      }

      cluster.push_back(image_idx);
      visit_flag[image_idx] = 1;

      std::vector<int> & point3d_ids = images[image_idx].point3d_ids;
      for (auto point_id : point3d_ids) {
        if (point_id == -1) {
          continue;
        }

        int cur_point_id = id_map[point_id];         // id_map[point_id] should always be valid
        for (auto idx : points_3d[cur_point_id].image_ids) {
          int image_id = image_id_map[idx];
          if (visit_flag.find(image_id) != visit_flag.end() && visit_flag[image_id] == 0) {
            cur_idxs.push_back(image_id);
          }
        }
      }
    }

    if (cluster.size() > max_cluster_size) {
      best_cluster = cluster;
      max_cluster_size = cluster.size();
    }
  }

  best_score = 0.f;
  for (auto image_idx : best_cluster) {
    int valid_point_num = 0;
    std::vector<int> & point3d_ids = images[image_idx].point3d_ids;
    for (auto point_id : point3d_ids) {
      valid_point_num += (point_id != -1);
    }

    best_score += valid_point_num * image_sims[image_idx];
  }

  return SLAM_OK;
}

static int GetIndexByName(const std::vector<MatchData> & match_datas, const std::string & name)
{
  int idx = 0;
  int data_count = match_datas.size();
  for (int i = 0; i < data_count; i++) {
    if (match_datas[i].match_pair_name == name) {
      idx = i;
      break;
    }
  }

  return idx;
}

static int GetIdxsPairs(
  std::vector<int> & cluster, std::vector<DbImage> & images,
  std::vector<MatchData> & match_datas, std::unordered_map<int,
  std::unordered_set<int>> & idxs_pair, std::unordered_map<int, int> & id_map)
{
  int cluster_size = cluster.size();
  for (int i = 0; i < cluster_size; i++) {
    std::vector<int> & points3d_ids = images[cluster[i]].point3d_ids;
    if (points3d_ids.size() <= 0) {
      continue;
    }
    std::string & name = images[cluster[i]].name;
    int idx = GetIndexByName(match_datas, name);
    std::vector<int> & matches = match_datas[idx].match_idxs;

    int match_size = matches.size();
    for (int j = 0; j < match_size; j++) {
      if (matches[j] != -1 && points3d_ids[matches[j]] != -1) {
        idxs_pair[j].insert(id_map[points3d_ids[matches[j]]]);
      }
    }
  }

  return SLAM_OK;
}

static int PoseFromCluster(
  std::vector<int> & cluster, LocalFeatureData & local_feature,
  std::vector<MatchData> & match_datas,
  std::vector<DbImage> & images, std::vector<DbPoints3D> & points_3d, std::unordered_map<int,
  int> & id_map,
  RealCamera & camera, float thresh, Quat & quat, Trans & trans, int & inline_num)
{
  std::unordered_map<int, std::unordered_set<int>> idxs_pair;
  int ret = SLAM_OK;
  LOG(INFO) << "start to get ID pairs";
  ret |= GetIdxsPairs(cluster, images, match_datas, idxs_pair, id_map);
  if (ret != SLAM_OK) {
    LOG(ERROR) << "GetIdxsPairs failed";
    return SLAM_ERROR;
  }
  LOG(INFO) << "end to get ID pairs";
  std::vector<Keypoint> & keypoints = local_feature.keypoints;
  ret = EstimateAbsolutePose(
    idxs_pair, keypoints, points_3d, camera, thresh, quat, trans,
    inline_num);
  if (ret != SLAM_OK) {
    LOG(INFO) << "EstimateAbsolutePose for current cluster fail!\n";
    return SLAM_ERROR;
  }

  return SLAM_OK;
}

int Estimator::CameraSelection(
  const std::vector<RawImage> & images,
  std::vector<std::vector<int>> & clusters,
  std::vector<int> & sort_idx)
{
  int ret = SLAM_OK;
  int image_num = images.size();

  std::vector<GlobalFeatureData> global_features;
  global_features.reserve(image_num);
  for (int i = 0; i < image_num; i++) {
    GlobalFeatureData global_feature;
    ret |= global_model_->Process(images[i], global_feature);
    global_features.emplace_back(global_feature);
  }
  if (ret != SLAM_OK) {
    LOG(ERROR) << "Global model process error";
    return ret;
  }

  int map_image_num = map_global_features_.size();
  std::vector<float> sims(map_image_num);
  std::vector<int> image_ids(map_image_num);
  // float best_score = -1.;

  std::vector<float> scores(image_num);

  for (int i = 0; i < image_num; i++) {
    GlobalFeature & cur_feature = global_features[i].feature;
    int j = 0;
    for (auto it = map_global_features_.begin(); it != map_global_features_.end(); ++it) {
      sims[j] = cur_feature.dot(it->second);
      image_ids[j] = name_map_[it->first];
      ++j;
    }

    std::vector<int> sort_idxs;
    std::vector<float> sort_sims;
    SortByIndex(sims, image_ids, sort_sims, sort_idxs, config_.retrieval_num);

    std::vector<int> cur_cluster;
    float cur_score = 0.;
    CovisibilityCluster(
      sort_idxs, sort_sims, images_, points_3d_, id_map_, image_id_map_,
      cur_cluster, cur_score);

    clusters.emplace_back(cur_cluster);
    scores[i] = cur_score;
    sort_idx.emplace_back(i);
  }

  std::sort(
    sort_idx.begin(), sort_idx.end(), [&scores](int i1, int i2)
    {return scores[i1] > scores[i2];});

  LOG(INFO) << "select camera done : ";

  return SLAM_OK;
}

int Estimator::GetFeatures(
  const std::vector<RawImage> & images, std::vector<int> & best_cluster,
  std::vector<MatchData> & match_datas, LocalFeatureData & local_feature,
  int & image_idx)
{
  LocalFeatureData local_feature_raw;
  m_image_num += 1;
  int ret = local_model_->Process(images[image_idx], local_feature_raw);

  KeypointRecover(
    local_feature_raw, local_feature, cameras_->at(image_idx).height,
    cameras_->at(image_idx).width);

  if (ret != SLAM_OK) {
    LOG(ERROR) << "local model process fail";
    return ret;
  }

  int cluster_size = best_cluster.size();
  match_datas.resize(cluster_size);
  LocalFeatureData local_feature_cmp;

  SparseMapDB * sparse_map_db = new SparseMapDB();
  sparse_map_db->Open(map_url_ + "database.db");

  for (int i = 0; i < cluster_size; i++) {
    const std::string & image_name = images_[best_cluster[i]].name;
    match_datas[i].match_pair_name = image_name;

    ret |= sparse_map_db->ReadLocalFeature(images_[best_cluster[i]].image_id, local_feature_cmp);
    ret |= match_model_->Process(local_feature_raw, local_feature_cmp, match_datas[i]);

    if (ret != SLAM_OK) {
      LOG(ERROR) << "get match feature fail";
      return ret;
    }
  }
  delete sparse_map_db;
  return SLAM_OK;
}

int Estimator::EstimatePose(
  const std::vector<RawImage> & images, const std::vector<int> & camera_idxs, Quat & quat,
  Trans & trans)
{
  if (status_ != 1) {
    LOG(ERROR) << "bad estimator status";
    return SLAM_ERROR;
  }

  std::vector<std::vector<int>> clusters;
  std::vector<int> sort_idx;

  int result = CameraSelection(images, clusters, sort_idx);
  if (result != SLAM_OK) {
    LOG(ERROR) << "CameraSelection error";
    return SLAM_ERROR;
  }

  int camera_idx = camera_idxs[sort_idx[0]];
  int image_idx = sort_idx[0];
  LOG(INFO) << "select camera: " << camera_idx;

  std::vector<MatchData> match_datas;
  LocalFeatureData local_feature;

  result |= GetFeatures(images, clusters[sort_idx[0]], match_datas, local_feature, image_idx);

  if (result != SLAM_OK) {
    LOG(ERROR) << "get features fail";
    return result;
  }

  int inline_num = 0;

  if (PoseFromCluster(
      clusters[sort_idx[0]], local_feature, match_datas, images_, points_3d_, id_map_,
      (*cameras_)[camera_idx - 1], config_.ransac_thresh, quat, trans, inline_num) != SLAM_OK)
  {
    LOG(ERROR) << "PoseFromCluster fail";
    return SLAM_ERROR;
  }

  if (inline_num < config_.inlier_num) {
    LOG(ERROR) << "get inline_num too small";
    return SLAM_ERROR;
  }

  m_success_num++;
  ChangePose(quat, trans, (*cameras_)[camera_idx - 1].camera_extrinsic);

  return SLAM_OK;
}

static int GetConfidence(const double & dis)
{
  int index = 0;
  if (dis <= 0.25) {
    index = 0;
  } else if (dis <= 0.5) {
    index = 1;
  } else if (dis <= 1) {
    index = 2;
  } else if (dis <= 2) {
    index = 3;
  } else if (dis < 4) {
    index = 4;
  } else {
    index = 5;
  }
  return index;
}

int Estimator::CheckResult(
  Eigen::Vector3d & position_reloc, const Eigen::Vector3d & position_vio,
  const Eigen::Quaterniond & quat_vio, float & confidence)
{
  if (tree_ != nullptr) {
    point_t p_est{position_reloc(0), position_reloc(1), position_reloc(2)};
    auto p_db = tree_->nearest_point(p_est);
    double dis = sqrt(pow(p_db[0] - p_est[0], 2) + pow(p_db[1] - p_est[1], 2));
    confidence = config_all_->confidence[GetConfidence(dis)];
  }

  // no vio pose
  if (0 == quat_vio.norm()) {
    return SLAM_OK;
  }

  auto distance =
    sqrt(pow(position_reloc(0) - position_vio(0), 2) + pow(position_reloc(1) - position_vio(1), 2));
  if (distance <= config_all_->position_threshold) {
    return SLAM_OK;
  } else {
    return SLAM_ERROR;
  }
}

}  // namespace miloc
}  // namespace cyberdog
