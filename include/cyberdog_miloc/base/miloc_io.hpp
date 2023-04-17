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

#ifndef CYBERDOG_MILOC__BASE__MILOC_IO_HPP_
#define CYBERDOG_MILOC__BASE__MILOC_IO_HPP_

#include <map>
#include <string>
#include <vector>
#include <unordered_set>
#include <utility>

#include "base/miloc_types.hpp"
#include "runtime/miloc_trt_runtime.hpp"
#include "utils/miloc_half.hpp"

namespace cyberdog
{
namespace miloc
{

int GetImageList(
  const std::string & dir_name, std::vector<std::string> & image_list,
  std::unordered_set<std::string> & time_stamps);
int ReadImageFolder(
  const std::string & folder_name, std::vector<std::string> & image_list,
  std::unordered_set<std::string> & time_stamps);
int LoadJpg(const std::string & file_name, RawImage & image);
void ReleaseImg(RawImage & image);
bool IsValidDir(const std::string & dir_name);
bool IsValidFile(const std::string & file_name);
std::string TimeStampFormat(const std::string & str, TimeStamp type);
int RawSave(GlobalFeatureMap & DATA, const std::string & path_name);
int RawLoad(const std::string & path_name, GlobalFeatureMap & DATA);
int GetTrajectory(
  std::string traj_path, std::map<std::string, std::pair<Trans,
  Quat>> & trajectory);

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__BASE__MILOC_IO_HPP_
