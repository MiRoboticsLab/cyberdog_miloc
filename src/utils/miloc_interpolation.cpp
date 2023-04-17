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

#include <string>
#include <vector>
#include <map>
#include <utility>
#include <limits>

#include "utils/miloc_interpolation.hpp"

namespace cyberdog
{
namespace miloc
{

void InterpolateTraj(
  std::vector<std::string> & time_stamps,
  std::map<std::string, std::pair<Trans, Quat>> & src_traj,
  std::map<std::string, std::pair<Trans, Quat>> & dest_traj)
{
  bool is_initial = true;
  if (time_stamps.empty() || src_traj.empty()) {
    return;
  }
  size_t time_stamps_num = time_stamps.size();
  double time1, time2, time_inter;
  Quat q1, q2, q_inter;
  Trans trans1, trans2, trans_inter;
  time1 = time2 = time_inter = 0.0;
  q1.setIdentity();
  q2.setIdentity();
  q_inter.setIdentity();
  trans1.setZero();
  trans2.setZero();
  trans_inter.setZero();
  size_t i = 0;
  for (auto iter = src_traj.begin(); iter != src_traj.end(); ++iter) {
    if (is_initial) {
      time_inter = std::stod(time_stamps[i]);
      time1 = std::stod(iter->first);
      while (time_inter <= time1) {
        if (++i < time_stamps_num) {
          time_inter = std::stod(time_stamps[i]);
        } else {
          return;
        }
      }

      q1 = iter->second.second;
      trans1 = iter->second.first;
      is_initial = false;
      continue;
    }

    time2 = std::stod(iter->first);
    q2 = iter->second.second;
    trans2 = iter->second.first;

    while (time1 <= time_inter && time2 > time_inter) {
      InterpolateRotation(time1, q1, time2, q2, time_inter, q_inter);
      InterpolateTranslation(time1, trans1, time2, trans2, time_inter, trans_inter);

      dest_traj.insert(std::make_pair(time_stamps[i], std::make_pair(trans_inter, q_inter)));
      if (++i < time_stamps_num) {
        time_inter = std::stod(time_stamps[i]);
      } else {
        return;
      }
    }

    time1 = time2;
    q1 = q2;
    trans1 = trans2;
  }
}

void InterpolateTranslation(
  double t1, const Eigen::Vector3d & trans1,
  double t2, const Eigen::Vector3d & trans2,
  double t_inter, Eigen::Vector3d & tran_inter)
{
  tran_inter.setZero();
  tran_inter = trans1 + (trans2 - trans1) / (t2 - t1) * (t_inter - t1);
}

void InterpolateRotation(
  double t1, const Eigen::Quaterniond & q1,
  double t2, const Eigen::Quaterniond & q2,
  double t_inter, Eigen::Quaterniond & q_inter)
{
  q_inter.setIdentity();
  Eigen::Quaterniond q_1_2 = q1.inverse() * q2;
  double theta = (t_inter - t1) / (t2 - t1);
  q_inter = q1 * Exp(theta * Log(q_1_2));
}

Eigen::Quaterniond Exp(const Eigen::Vector3d & dx)
{
  double theta = dx.norm();
  double na;
  if (IsLessThanEpsilons4thRoot(theta)) {
    static const double one_over_48 = 1.0 / 48.0;
    na = 0.5 + (theta * theta) * one_over_48;
  } else {
    na = std::sin(theta * 0.5) / theta;
  }
  double ct = std::cos(theta * 0.5);
  return Eigen::Quaterniond(ct, dx[0] * na, dx[1] * na, dx[2] * na);
}

Eigen::Vector3d Log(const Eigen::Quaterniond & q)
{
  Eigen::Vector3d a(q.x(), q.y(), q.z());
  double na = a.norm();
  double eta = q.w();
  double scale;
  if (std::fabs(eta) < na) {
    if (eta >= 0) {
      scale = std::acos(eta) / na;
    } else {
      scale = -std::acos(eta) / na;
    }
  } else {
    if (eta > 0) {
      scale = ArcSinXOverX(na);
    } else {
      scale = -ArcSinXOverX(na);
    }
  }
  return a * (2.0 * scale);
}

bool IsLessThanEpsilons4thRoot(double x)
{
  const double epsilon_4th_root = std::pow(std::numeric_limits<double>::epsilon(), 1.0 / 4.0);
  return x < epsilon_4th_root;
}

double ArcSinXOverX(double x)
{
  if (IsLessThanEpsilons4thRoot(std::fabs(x))) {
    return 1.0 + x * x * (1.0 / 6.0);
  }
  return std::asin(x) / x;
}

void FilterImages(
  std::vector<std::string> & time_stamps,
  std::map<std::string, std::pair<Trans, Quat>> & interpolated_traj,
  double translation_interval, double orientation_interval)
{
  auto old_position = Trans(0, 0, 0);
  auto old_orientation = Quat(1, 0, 0, 0);
  for (auto iter = time_stamps.begin(); iter != time_stamps.end(); ++iter) {
    if (interpolated_traj.find(*iter) != interpolated_traj.end()) {
      Trans interval_position = interpolated_traj.at(*iter).first - old_position;
      Quat interval_orientation = interpolated_traj.at(*iter).second.inverse() * old_orientation;
      if (interval_position.norm() < translation_interval &&
        interval_orientation.vec().norm() < orientation_interval)
      {
        interpolated_traj.erase(*iter);
      } else {
        old_position = interpolated_traj.at(*iter).first;
        old_orientation = interpolated_traj.at(*iter).second;
      }
    }
  }
}

}  // namespace miloc
}  // namespace cyberdog
