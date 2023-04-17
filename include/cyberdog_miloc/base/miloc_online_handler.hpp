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

#ifndef CYBERDOG_MILOC__BASE__MILOC_ONLINE_HANDLER_HPP_
#define CYBERDOG_MILOC__BASE__MILOC_ONLINE_HANDLER_HPP_

#include <string>
#include <vector>

#include <colmap/lib/SQLite/sqlite3.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "base/miloc_types.hpp"
#include "utils/miloc_database.hpp"
#include "utils/miloc_convertor.hpp"
#include "base/miloc_io.hpp"

namespace cyberdog
{
namespace miloc
{

class OnlineHandler
{
public:
  OnlineHandler()
  : is_creating_map_(false) {}
  OnlineHandler(const OnlineHandler &) = default;
  ~OnlineHandler();

  int Init(
    std::string & table_name, CyberDB * cyberdb, std::string & map_path,
    std::vector<RealCamera> * cameras);

  int CreateMap(bool slam_flag, int & current_map_id);

  int FinishMap(bool slam_flag);

  int ProcessImage(
    uint64_t timestamp, const std::vector<cv::Mat> & images,
    const std::vector<int> & camera_idxs);

  int SaveImage(ImageBundle & image_bundle);

  std::string & GetImagePath()
  {
    return image_path_;
  }

private:
  bool is_creating_map_;
  std::string table_name_;
  std::string map_path_;
  std::string image_path_;
  std::string visual_path_;
  std::vector<RealCamera> * cameras_;
  SparseMapDB * map_db_;
  CyberDB * cyberdb_;
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__BASE__MILOC_ONLINE_HANDLER_HPP_
