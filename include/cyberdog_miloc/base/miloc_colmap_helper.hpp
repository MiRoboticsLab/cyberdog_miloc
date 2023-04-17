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

#ifndef CYBERDOG_MILOC__BASE__MILOC_COLMAP_HELPER_HPP_
#define CYBERDOG_MILOC__BASE__MILOC_COLMAP_HELPER_HPP_

#include <vector>
#include <string>
#include <map>
#include <utility>
#include <unordered_set>
#include <unordered_map>

#include <colmap/base/camera.h>
#include <colmap/base/image.h>

#include "base/miloc_types.hpp"
#include "runtime/miloc_trt_runtime.hpp"

namespace cyberdog
{
namespace miloc
{

int GetColmapCameras(
  const std::vector<RealCamera> & real_cameras,
  std::vector<colmap::Camera> & colmap_cameras);
int ReadColmapImages(
  std::map<std::string, std::pair<Trans, Quat>> & traj,
  const std::unordered_set<std::string> & time_stamps,
  const std::vector<RealCamera> & real_cameras,
  std::vector<colmap::Image> & colmap_images, std::unordered_map<std::string,
  int> & name_map);
int SaveNamePairs(
  const std::string & database_dir,
  const std::vector<std::string> & name_pair_list);
void VerifyMatch(const std::string & database_dir);
int RunTriangulation(
  const std::string & database_dir, const std::vector<colmap::Camera> & cameras,
  const std::vector<colmap::Image> & images,
  const std::string & output_path, const MapperParams & map_params);

int EstimateAbsolutePose(
  std::unordered_map<int, std::unordered_set<int>> & idxs_pair,
  std::vector<Keypoint> & keypoints,
  std::vector<DbPoints3D> & points_3d, RealCamera & camera, float thresh,
  Quat & quat, Trans & trans, int & inline_num);
int ChangePose(Quat & quat, Trans & trans, std::array<double, 16> & exintrincs);

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__BASE__MILOC_COLMAP_HELPER_HPP_
