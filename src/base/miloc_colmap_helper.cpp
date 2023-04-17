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
#include <map>
#include <utility>

#include <Eigen/Dense>
#include <unordered_map>
#include <glog/logging.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <colmap/base/reconstruction.h>
#include <colmap/exe/sfm.h>
#include <colmap/util/types.h>
#include <colmap/estimators/pose.h>
#include <colmap/util/random.h>
#include <colmap/base/point2d.h>
#include <colmap/feature/matching.h>

#include "base/miloc_colmap_helper.hpp"
#include "base/miloc_io.hpp"
#include "utils/miloc_database.hpp"
#include "utils/miloc_interpolation.hpp"

namespace cyberdog
{
namespace miloc
{

int GetColmapCameras(
  const std::vector<RealCamera> & real_cameras,
  std::vector<colmap::Camera> & colmap_cameras)
{
  int ret = SLAM_OK;
  int camera_num = real_cameras.size();

  for (int i = 0; i < camera_num; i++) {
    const RealCamera & real_camera = real_cameras[i];
    colmap::Camera camera;
    camera.SetCameraId(real_camera.camera_id);
    camera.SetModelIdFromName(real_camera.model_name);
    camera.SetWidth(real_camera.width);
    camera.SetHeight(real_camera.height);
    const int param_num = camera.NumParams();
    double * param_data = camera.ParamsData();
    for (int i = 0; i < param_num; i++) {
      param_data[i] = real_camera.camera_intrinsic[i];
    }
    colmap_cameras.push_back(camera);
  }

  return ret;
}

static std::string GetTag(const char * str)
{
  double time_val = atof(str);
  char str_val[64];
  snprintf(str_val, sizeof(str_val), "%.3f.jpg", time_val);

  return std::string(str_val);
}

int ReadColmapImages(
  const std::string & tra_file,
  const std::unordered_set<std::string> & time_stamps,
  const std::vector<RealCamera> & real_cameras,
  std::vector<colmap::Image> & colmap_images, std::unordered_map<std::string,
  int> & name_map)
{
  if (real_cameras.size() != 3) {
    LOG(ERROR) << "Bad camera num";
    return SLAM_ERROR;
  }

  FILE * fp = fopen(tra_file.c_str(), "r");
  if (nullptr == fp) {
    LOG(ERROR) << "File open error";
    return SLAM_NULL_PTR;
  }

  char stamp[32];
  char q0[32], q1[32], q2[32], q3[32];
  char t0[32], t1[32], t2[32];

  Eigen::Vector3d trans;
  Eigen::Quaterniond quat;
  Eigen::Matrix3d camera_rotate[3];
  Eigen::Vector3d camera_trans[3];

  for (int i = 0; i < 3; i++) {
    const RealCamera & camera = real_cameras[i];
    camera_rotate[i](0, 0) = camera.camera_extrinsic[0];
    camera_rotate[i](0, 1) = camera.camera_extrinsic[1];
    camera_rotate[i](0, 2) = camera.camera_extrinsic[2];
    camera_trans[i](0) = camera.camera_extrinsic[3];
    camera_rotate[i](1, 0) = camera.camera_extrinsic[4];
    camera_rotate[i](1, 1) = camera.camera_extrinsic[5];
    camera_rotate[i](1, 2) = camera.camera_extrinsic[6];
    camera_trans[i](1) = camera.camera_extrinsic[7];
    camera_rotate[i](2, 0) = camera.camera_extrinsic[8];
    camera_rotate[i](2, 1) = camera.camera_extrinsic[9];
    camera_rotate[i](2, 2) = camera.camera_extrinsic[10];
    camera_trans[i](2) = camera.camera_extrinsic[11];
  }

  int image_id = 1;
  while (!feof(fp)) {
    fscanf(fp, "%s %s %s %s %s %s %s %s", stamp, t0, t1, t2, q0, q1, q2, q3);
    std::string tag = GetTag(stamp);
    trans.x() = atof(t0);
    trans.y() = atof(t1);
    trans.z() = atof(t2);
    quat.w() = atof(q3);
    quat.x() = atof(q0);
    quat.y() = atof(q1);
    quat.z() = atof(q2);

    if (time_stamps.find(tag) != time_stamps.end()) {
      for (int i = 0; i < 3; i++) {
        Eigen::Matrix3d tmp_rot = quat.matrix() * camera_rotate[i].transpose();
        Eigen::Vector3d tmp_trans = -tmp_rot * camera_trans[i] + trans;

        tmp_rot.transposeInPlace();
        tmp_trans = -tmp_rot * tmp_trans;
        Eigen::Quaterniond tmp_quat(tmp_rot);

        std::string image_name = real_cameras[i].prefix + "/" + tag;
        name_map[image_name] = image_id;

        colmap::Image colmap_image;
        colmap_image.SetImageId(image_id++);
        colmap_image.SetCameraId(real_cameras[i].camera_id);
        colmap_image.SetName(image_name);
        colmap_image.Qvec(0) = tmp_quat.w();
        colmap_image.Qvec(1) = tmp_quat.x();
        colmap_image.Qvec(2) = tmp_quat.y();
        colmap_image.Qvec(3) = tmp_quat.z();
        colmap_image.SetTvec(tmp_trans);
        colmap_image.NormalizeQvec();

        colmap_images.emplace_back(colmap_image);
      }
    }
  }

  fclose(fp);
  return SLAM_OK;
}

int ReadColmapImages(
  std::map<std::string, std::pair<Trans, Quat>> & traj,
  const std::unordered_set<std::string> & time_stamps,
  const std::vector<RealCamera> & real_cameras,
  std::vector<colmap::Image> & colmap_images, std::unordered_map<std::string,
  int> & name_map)
{
  if (real_cameras.size() != 3) {
    LOG(ERROR) << "Bad camera num";
    return SLAM_ERROR;
  }

  Eigen::Matrix3d camera_rotate[3];
  Eigen::Vector3d camera_trans[3];

  for (int i = 0; i < 3; i++) {
    const RealCamera & camera = real_cameras[i];
    camera_rotate[i](0, 0) = camera.camera_extrinsic[0];
    camera_rotate[i](0, 1) = camera.camera_extrinsic[1];
    camera_rotate[i](0, 2) = camera.camera_extrinsic[2];
    camera_trans[i](0) = camera.camera_extrinsic[3];
    camera_rotate[i](1, 0) = camera.camera_extrinsic[4];
    camera_rotate[i](1, 1) = camera.camera_extrinsic[5];
    camera_rotate[i](1, 2) = camera.camera_extrinsic[6];
    camera_trans[i](1) = camera.camera_extrinsic[7];
    camera_rotate[i](2, 0) = camera.camera_extrinsic[8];
    camera_rotate[i](2, 1) = camera.camera_extrinsic[9];
    camera_rotate[i](2, 2) = camera.camera_extrinsic[10];
    camera_trans[i](2) = camera.camera_extrinsic[11];
  }

  int image_id = 1;
  for (auto iter = traj.begin(); iter != traj.end(); ++iter) {
    std::string tag = GetTag(iter->first.c_str());

    if (time_stamps.find(tag) != time_stamps.end()) {
      for (int i = 0; i < 3; i++) {
        // For Tcb
        Eigen::Matrix3d tmp_rot = iter->second.second.matrix() * camera_rotate[i].transpose();
        Eigen::Vector3d tmp_trans = -tmp_rot * camera_trans[i] + iter->second.first;

        tmp_rot.transposeInPlace();
        tmp_trans = -tmp_rot * tmp_trans;
        Eigen::Quaterniond tmp_quat(tmp_rot);

        std::string image_name = real_cameras[i].prefix + "/" + tag;
        name_map[image_name] = image_id;

        colmap::Image colmap_image;
        colmap_image.SetImageId(image_id++);
        colmap_image.SetCameraId(real_cameras[i].camera_id);
        colmap_image.SetName(image_name);
        colmap_image.Qvec(0) = tmp_quat.w();
        colmap_image.Qvec(1) = tmp_quat.x();
        colmap_image.Qvec(2) = tmp_quat.y();
        colmap_image.Qvec(3) = tmp_quat.z();
        colmap_image.SetTvec(tmp_trans);
        colmap_image.NormalizeQvec();

        colmap_images.emplace_back(colmap_image);
      }
    }
  }

  return SLAM_OK;
}

int SaveNamePairs(const std::string & database_dir, const std::vector<std::string> & name_pair_list)
{
  std::string pair_file = database_dir + "/name_pair.txt";
  FILE * fp = fopen(pair_file.c_str(), "a");
  if (nullptr == fp) {
    LOG(ERROR) << "File open error";
    return SLAM_NULL_PTR;
  }

  int name_pair_num = name_pair_list.size();
  for (int i = 0; i < name_pair_num; i++) {
    fprintf(fp, "%s\n", name_pair_list[i].c_str());
  }

  fclose(fp);
  return SLAM_OK;
}

void VerifyMatch(const std::string & database_dir)
{
  std::string pair_file = database_dir + "/name_pair.txt";
  std::string database = database_dir + "/database.db";

  int max_num_trials = 20000;
  float min_inlier_ratio = 0.1;

  colmap::SiftMatchingOptions options;
  options.use_gpu = false;
  options.max_num_trials = max_num_trials;
  options.min_inlier_ratio = min_inlier_ratio;
  options.confidence = 0.999;
  options.num_threads = -1;

  colmap::ImagePairsMatchingOptions matcher_options;
  matcher_options.match_list_path = pair_file;

  colmap::ImagePairsFeatureMatcher feature_matcher(matcher_options, options, database);
  feature_matcher.Start();
  feature_matcher.Wait();
  feature_matcher.Stop();
}

int RunTriangulation(
  const std::string & database_dir, const std::vector<colmap::Camera> & cameras,
  const std::vector<colmap::Image> & images,
  const std::string & output_path, const MapperParams & map_params)
{
  colmap::Reconstruction reconstruction;

  int camera_num = cameras.size();
  for (int i = 0; i < camera_num; i++) {
    reconstruction.AddCamera(cameras[i]);
  }

  int image_num = images.size();
  for (int i = 0; i < image_num; i++) {
    reconstruction.AddImage(images[i]);
    if (!reconstruction.IsImageRegistered(images[i].ImageId())) {
      reconstruction.RegisterImage(images[i].ImageId());
    }
  }

  // reconstruction.Write(output_path);

  std::string database_path = database_dir + "/database.db";
  std::string image_path = "";
  colmap::IncrementalMapperOptions mapper_options;
  if (colmap::RunPointTriangulatorImpl(
      reconstruction, database_path, image_path, output_path,
      mapper_options, true, false) != 0)
  {
    LOG(ERROR) << "RunPointTriangulatorImpl error";
    return SLAM_ERROR;
  }

  reconstruction.FilterAllPoints3D(map_params.max_reproj_error, map_params.min_tri_angle);
  std::unordered_set<colmap::point3D_t> point_idxs = reconstruction.Point3DIds();
  std::unordered_set<colmap::point3D_t>::iterator it;
  for (it = point_idxs.begin(); it != point_idxs.end(); it++) {
    if (reconstruction.Point3D(*it).Track().Length() < map_params.min_track_len) {
      reconstruction.DeletePoint3D(*it);
    }
  }

  std::vector<uint32_t> filtered_image_ids =
    reconstruction.FilterImages(
    mapper_options.min_focal_length_ratio,
    mapper_options.max_focal_length_ratio,
    mapper_options.max_extra_param);
  GlobalFeatureMap global_data;
  const std::string golbal_data_dir = database_dir + "/";
  int ret = RawLoad(golbal_data_dir, global_data);
  for (uint32_t image_id : filtered_image_ids) {
    if (global_data.find(reconstruction.Image(image_id).Name()) != global_data.end()) {
      global_data.erase(reconstruction.Image(image_id).Name());
    }
  }
  ret |= RawSave(global_data, golbal_data_dir);

  // reconstruction.TearDown();

  reconstruction.Write(output_path);

  reconstruction.DeleteAllPoints2DAndPoints3D();
  for (const colmap::image_t image_id : reconstruction.RegImageIds()) {
    reconstruction.DeRegisterImage(image_id);
  }

  reconstruction.TearDown();

  return SLAM_OK;
}

static std::unordered_set<int> GetAbsolutePose(
  std::unordered_map<int,
  std::unordered_set<int>> & idxs_pair,
  std::vector<Keypoint> & keypoints,
  std::vector<DbPoints3D> & points_3d,
  RealCamera & camera, float thresh, Quat & quat,
  Trans & trans)
{
  LOG(INFO) << "start to get GetAbsolutePose";
  colmap::AbsolutePoseEstimationOptions abs_pose_options;
  abs_pose_options.estimate_focal_length = false;
  abs_pose_options.ransac_options.max_error = thresh;
  abs_pose_options.ransac_options.min_inlier_ratio = 0.1;  //0.01;
  abs_pose_options.ransac_options.min_num_trials = 15;  //1000;
  abs_pose_options.ransac_options.max_num_trials = 10000;
  abs_pose_options.ransac_options.confidence = 0.999;  //0.9999;

  colmap::Camera colmap_camera;
  colmap_camera.SetCameraId(camera.camera_id);
  colmap_camera.SetModelIdFromName(camera.model_name);
  colmap_camera.SetWidth(camera.width);
  colmap_camera.SetHeight(camera.height);
  const int param_num = colmap_camera.NumParams();
  double * param_data = colmap_camera.ParamsData();
  for (int i = 0; i < param_num; i++) {
    param_data[i] = camera.camera_intrinsic[i];
  }

  // get correspond 2d points and 3d points
  std::vector<Eigen::Vector2d> tri_points2d;
  std::vector<Eigen::Vector3d> tri_points3d;
  std::vector<int> point_3d_idx;

  std::unordered_set<int> inline_3d_idx;

  LOG(INFO) << "start to get 2d-3d pairs";
  for (auto idx_pair : idxs_pair) {
    Keypoint point_2d = keypoints[idx_pair.first];

    for (auto idx : idx_pair.second) {
      Point3f & point_3d = points_3d[idx].xyz;
      point_3d_idx.push_back(idx);
      tri_points2d.push_back(Eigen::Vector2d(point_2d[0], point_2d[1]));
      tri_points3d.push_back(Eigen::Vector3d(point_3d[0], point_3d[1], point_3d[2]));
    }
  }
  LOG(INFO) << "end to get 2d-3d pairs";

  if (tri_points3d.size() <= 5) {
    LOG(INFO) << "colmap EstimateAbsolutePose fail! point too small";
    inline_3d_idx.clear();

    return inline_3d_idx;
  }
  LOG(INFO) << "Triangulated 2D point size: " << tri_points2d.size();
  colmap::SetPRNGSeed(0);   // set ransac seed
  Eigen::Vector4d qvec;
  // Eigen::Vector3d tvec;
  size_t num_inliers;
  std::vector<char> inlier_mask;
  if (!colmap::EstimateAbsolutePose(
      abs_pose_options, tri_points2d, tri_points3d, &qvec, &trans,
      &colmap_camera, &num_inliers, &inlier_mask))
  {
    // LOG_INFO(BASE_LOG_TAG, "colmap EstimateAbsolutePose fail!\n");
    LOG(INFO) << "colmap EstimateAbsolutePose fail!";
    inline_3d_idx.clear();
    return inline_3d_idx;
  }
  LOG(INFO) << "Inlier point size: " << num_inliers;
  // init refine config
  colmap::AbsolutePoseRefinementOptions abs_pose_refinement_options;
  abs_pose_refinement_options.refine_focal_length = false;
  abs_pose_refinement_options.refine_extra_params = false;
  abs_pose_refinement_options.print_summary = false;
  if (!colmap::RefineAbsolutePose(
      abs_pose_refinement_options, inlier_mask, tri_points2d,
      tri_points3d, &qvec, &trans, &colmap_camera))
  {
    // LOG_INFO(BASE_LOG_TAG, "colmap RefineAbsolutePose fail!\n");
    LOG(INFO) << "colmap RefineAbsolutePose fail!";
    inline_3d_idx.clear();
    return inline_3d_idx;
  }
  LOG(INFO) << "Pose refinement finished.";
  // set the output

  int mask_size = inlier_mask.size();
  for (int i = 0; i < mask_size; i++) {
    if (inlier_mask[i]) {
      inline_3d_idx.insert(point_3d_idx[i]);
    }
  }

  quat.w() = qvec[0];
  quat.x() = qvec[1];
  quat.y() = qvec[2];
  quat.z() = qvec[3];

  return inline_3d_idx;
}

int EstimateAbsolutePose(
  std::unordered_map<int, std::unordered_set<int>> & idxs_pair, std::vector<Keypoint> & keypoints,
  std::vector<DbPoints3D> & points_3d, RealCamera & camera, float thresh,
  Quat & quat, Trans & trans, int & inline_num)
{
  // init config for estimate pose

  auto inline_3d_idx =
    GetAbsolutePose(idxs_pair, keypoints, points_3d, camera, thresh, quat, trans);
  if (0 == inline_3d_idx.size()) {
    LOG(ERROR) << "GetAbsolutePose error ";
    return SLAM_ERROR;
  }

  inline_num = inline_3d_idx.size();

  return SLAM_OK;
}

int ChangePose(Quat & quat, Trans & trans, std::array<double, 16> & exintrincs)
{
  Eigen::Matrix3d rotate = quat.matrix().transpose();

  trans = -rotate * trans;
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
  rotate = tmp_mat.topLeftCorner(3, 3);
  trans = tmp_mat.topRightCorner(3, 1);
  quat = rotate;

  return SLAM_OK;
}

}  // namespace miloc
}  // namespace cyberdog
