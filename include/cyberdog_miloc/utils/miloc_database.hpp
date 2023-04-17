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

#ifndef CYBERDOG_MILOC__UTILS__MILOC_DATABASE_HPP_
#define CYBERDOG_MILOC__UTILS__MILOC_DATABASE_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <array>

#include <colmap/lib/SQLite/sqlite3.h>
#include <colmap/base/reconstruction.h>

#include "base/miloc_types.hpp"
#include "runtime/miloc_trt_runtime.hpp"
#include "utils/miloc_half.hpp"

namespace cyberdog
{
namespace miloc
{

class MiLocDatabase
{
public:
  MiLocDatabase() = default;
  explicit MiLocDatabase(std::string & path)
  {
    Open(path);
  }
  ~MiLocDatabase();

  int Open(const std::string & path);
  void Close();
  int Begin();
  int Commit();

protected:
  sqlite3 * m_database = nullptr;
  sqlite3_stmt * m_stmt = nullptr;
  mutable bool m_database_cleared = false;
  bool m_transaction_flag = false;
};

class CyberDB : public MiLocDatabase
{
public:
  CyberDB() = default;
  explicit CyberDB(const std::string & path)
  {
    Open(path);
  }
  ~CyberDB() = default;

  int UpdateMapInfo(
    const int & map_id, const std::string & map_name, const std::string & map_url,
    const std::string & table_name);
  int GetMaxMapId(int & max_map_id, const std::string table_name);
  int WriteNewMap(
    const int & map_id, const std::string & map_name, const std::string & map_url,
    const std::string & table_name, bool slam_flag);
  int ReadMapInfo(const int & map_id, MapInfo & map_info);
  int DeleteMap(const int map_id);
  int CreateMilocTable();
  int GetMapName(const int map_id, std::string & map_name);

  int GetMapNums(int & map_num);
  int GetAllMapIds(std::vector<int> & map_ids);

  int UpdateMap(const int map_id, const std::string & map_name);
  int QueryMaps(
    std::vector<std::string> & map_name_list, std::vector<int> & map_id_list,
    std::vector<int> & status_list, std::vector<std::string> & map_url_list);
  int SetMapStatus(const int & map_id, const int & map_status);
};

class SparseMapDB : public MiLocDatabase
{
public:
  SparseMapDB() = default;
  explicit SparseMapDB(const std::string & path)
  {
    Open(path);
  }
  ~SparseMapDB();
  int Commit();

  int CreateDbTables();

  int WriteCameras(const std::vector<colmap::Camera> & cameras);
  int WriteImages(const std::vector<colmap::Image> & images, const bool use_image_id);
  int GetImageIds(std::unordered_map<std::string, int> & name_map);
  int ReadImages(std::vector<colmap::Image> & images);

  int ReadLocalFeature(const int & image_id, LocalFeatureData & local_feature);
  int WriteLocalFeature_nocommit(const int & image_id, LocalFeatureData & local_feature);
  int WriteMatchFeature_nocommit(
    const int64_t & pair_id, std::vector<std::array<int,
    2>> & match_pair);

private:
  sqlite3_stmt * keypoint_stmt_ {nullptr};
  sqlite3_stmt * des_stmt_ {nullptr};
  sqlite3_stmt * score_stmt_ {nullptr};
};

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__UTILS__MILOC_DATABASE_HPP_
