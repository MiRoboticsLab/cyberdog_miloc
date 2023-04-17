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

#ifndef CYBERDOG_MILOC__UTILS__MILOC_INTERPOLATION_HPP_
#define CYBERDOG_MILOC__UTILS__MILOC_INTERPOLATION_HPP_

#include <unordered_map>
#include <vector>
#include <string>
#include <map>
#include <utility>

#include <Eigen/Dense>

#include "base/miloc_types.hpp"

namespace cyberdog
{
namespace miloc
{

void InterpolateTraj(
  std::vector<std::string> & time_stamps,
  std::map<std::string, std::pair<Trans, Quat>> & src_traj,
  std::map<std::string, std::pair<Trans, Quat>> & dest_traj);

void InterpolateTranslation(
  double t1, const Eigen::Vector3d & trans1,
  double t2, const Eigen::Vector3d & trans2,
  double t_inter, Eigen::Vector3d & tran_inter);

void InterpolateRotation(
  double t1, const Eigen::Quaterniond & q1,
  double t2, const Eigen::Quaterniond & q2,
  double t_inter, Eigen::Quaterniond & q_inter);

Eigen::Quaterniond Exp(const Eigen::Vector3d & dx);

Eigen::Vector3d Log(const Eigen::Quaterniond & q);

bool IsLessThanEpsilons4thRoot(double theta);

double ArcSinXOverX(double x);

void FilterImages(
  std::vector<std::string> & time_stamps,
  std::map<std::string, std::pair<Trans, Quat>> & interpolated_traj,
  double translation_interval, double orientation_interval);

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__UTILS__MILOC_INTERPOLATION_HPP_
