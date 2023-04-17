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

#include <glog/logging.h>

#include <chrono>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "utils/miloc_basic.hpp"
#include "utils/miloc_database.hpp"

namespace cyberdog
{
namespace miloc
{

#define FEATURE_DIM 96
#define TABLE_MACRO
#ifdef TABLE_MACRO
#define CREATE_CAMERAS_TABLE \
  R"(CREATE TABLE IF NOT EXISTS cameras (\
camera_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, \
model INTEGER NOT NULL, \
width INTEGER NOT NULL, \
height INTEGER NOT NULL, \
params BLOB, \
prior_focal_length INTEGER NOT NULL);)"

#define CREATE_DESCRIPTORS_TABLE \
  R"(CREATE TABLE IF NOT EXISTS descriptors (\
image_id INTEGER PRIMARY KEY NOT NULL, \
rows INTEGER NOT NULL, \
cols INTEGER NOT NULL, \
data BLOB, \
FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);)"

#define CREATE_SCORES_TABLE \
  R"(CREATE TABLE IF NOT EXISTS scores (\
image_id INTEGER PRIMARY KEY NOT NULL, \
rows INTEGER NOT NULL, \
cols INTEGER NOT NULL, \
data BLOB, \
FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);)"

#define CREATE_IMAGES_TABLE \
  R"(CREATE TABLE IF NOT EXISTS images (\
image_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, \
name TEXT NOT NULL UNIQUE, \
camera_id INTEGER NOT NULL, \
prior_qw REAL, \
prior_qx REAL, \
prior_qy REAL, \
prior_qz REAL, \
prior_tx REAL, \
prior_ty REAL, \
prior_tz REAL, \
light INTERGER, \
FOREIGN KEY(camera_id) REFERENCES cameras(camera_id));)"

#define CREATE_TWO_VIEW_GEOMETRIES_TABLE \
  R"(CREATE TABLE IF NOT EXISTS two_view_geometries (\
pair_id INTEGER PRIMARY KEY NOT NULL, \
rows INTEGER NOT NULL, \
cols INTEGER NOT NULL, \
data BLOB, \
config INTEGER NOT NULL, \
F BLOB, \
E BLOB, \
H BLOB, \
qvec BLOB, \
tvec BLOB);)"

#define CREATE_KEYPOINTS_TABLE \
  R"(CREATE TABLE IF NOT EXISTS keypoints (\
image_id INTEGER PRIMARY KEY NOT NULL, \
rows INTEGER NOT NULL, \
cols INTEGER NOT NULL, \
data BLOB, \
FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE);)"

#define CREATE_MATCHES_TABLE \
  R"(CREATE TABLE IF NOT EXISTS matches (\
pair_id INTEGER PRIMARY KEY NOT NULL, \
rows INTEGER NOT NULL, \
cols INTEGER NOT NULL, \
data BLOB);)"

#define CREATE_NAME_INDEX "CREATE UNIQUE INDEX IF NOT EXISTS index_name ON images(name);"

#define CREATE_RELOCMAP_TABLE \
  R"(CREATE TABLE IF NOT EXISTS reloc_map (\
id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, \
name STRING NOT NULL, \
height FLOAT NOT NULL, \
width FLOAT NOT NULL, \
map_url STRING NOT NULL, \
version FLOAT NOT NULL, \
create_time INTERGER NOT NULL, \
update_time INTERGER NOT NULL, \
status INTERGER NOT NULL, \
scene INTERGER NOT NULL, \
slam  INTERGER NOT NULL, \
introduction STRING NOT NULL);)"

#endif

MiLocDatabase::~MiLocDatabase()
{
  if (m_database != nullptr) {
    Close();
    m_database = nullptr;
  }
}

int MiLocDatabase::Open(const std::string & path)
{
  // boost::filesystem::path db_dir{path};
  // std::string db_parent_path = db_root.parent_path().string();

  std::string db_error_file_shm = path + "-shm";
  std::string db_error_file_wal = path + "-wal";
  std::string db_error_file_j = path + "-journal";


  if (boost::filesystem::exists(db_error_file_shm)) {
    boost::filesystem::remove(db_error_file_shm);
  }
  if (boost::filesystem::exists(db_error_file_wal)) {
    boost::filesystem::remove(db_error_file_wal);
  }
  if (boost::filesystem::exists(db_error_file_j)) {
    boost::filesystem::remove(db_error_file_j);
  }

  int ret = sqlite3_open_v2(
    path.c_str(), &m_database, SQLITE_OPEN_CREATE | SQLITE_OPEN_READWRITE, NULL);
  if (ret != SQLITE_OK) {
    LOG(ERROR) << "open  database failed ";
    return SLAM_ERROR;
  }
  LOG(INFO) << "open database success";
  return SLAM_OK;
}

void MiLocDatabase::Close()
{
  if (m_database != nullptr) {
    if (m_database_cleared) {
      const int result_code = sqlite3_exec(m_database, "VACUUM", nullptr, nullptr, nullptr);
      if (result_code != SQLITE_OK) {
        LOG(ERROR) << "VACUUM database failed ";
      }
      m_database_cleared = false;
    }
    if (m_stmt != nullptr) {
      sqlite3_finalize(m_stmt);
      m_stmt = nullptr;
    }
    sqlite3_close_v2(m_database);
    LOG(INFO) << "close database success";
    m_database = nullptr;
  }
}

int MiLocDatabase::Begin()
{
  int result = sqlite3_exec(m_database, "BEGIN;", 0, 0, 0);
  if (result != SQLITE_OK) {
    LOG(ERROR) << "begin transaction failed ";
    return SLAM_ERROR;
  }
  m_transaction_flag = true;
  LOG(INFO) << "begin transaction";
  return SLAM_OK;
}

int MiLocDatabase::Commit()
{
  if (m_transaction_flag == false) {
    LOG(ERROR) << "db transaction is not begin";
    return SLAM_ERROR;
  }

  int result = sqlite3_exec(m_database, "COMMIT;", 0, 0, 0);
  if (m_stmt != nullptr) {
    sqlite3_finalize(m_stmt);
    m_stmt = nullptr;
  }

  m_transaction_flag = false;

  if (result != SQLITE_OK) {
    LOG(ERROR) << "commit transaction failed ";
    return SLAM_ERROR;
  }
  LOG(INFO) << "Commit transaction";
  return SLAM_OK;
}

int CyberDB::CreateMilocTable()
{
  int result = sqlite3_exec(m_database, CREATE_RELOCMAP_TABLE, nullptr, nullptr, nullptr);
  if (result != SQLITE_OK) {
    LOG(ERROR) << "CreateMilocTable failed";
    return SLAM_ERROR;
  }
  LOG(INFO) << "CreateMilocTable success";
  return SLAM_OK;
}

int CyberDB::UpdateMapInfo(
  const int & map_id, const std::string & map_name,
  const std::string & map_url, const std::string & table_name)
{
  char sql_buffer[256];
  sqlite3_stmt * stmt{nullptr};

  std::chrono::microseconds update_time = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch());

  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "UPDATE %s SET name = '%s', map_url = '%s', update_time = %ld, status = 2 WHERE id = %d;",
    table_name.c_str(), map_name.c_str(), map_url.c_str(), update_time.count(), map_id);
  int ret = 0;
  ret |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, nullptr);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  if (ret != SQLITE_OK) {
    LOG(ERROR) << "cyberdog db UpdateMapInfo failed";
    return SLAM_ERROR;
  }

  stmt = nullptr;
  LOG(INFO) << "UpdateMapInfo success";

  return SLAM_OK;
}

int CyberDB::SetMapStatus(
  const int & map_id, const int & map_status)
{
  char sql_buffer[256];
  sqlite3_stmt * stmt{nullptr};

  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "UPDATE reloc_map SET status = %d WHERE id = %d;", map_status, map_id);
  int ret = 0;
  ret |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, nullptr);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  if (ret != SQLITE_OK) {
    LOG(ERROR) << "SetMapStatus failed";
    return SLAM_ERROR;
  }

  stmt = nullptr;
  // LOG(INFO) << "UpdateMapInfo success";

  return SLAM_OK;
}

int CyberDB::GetMapName(const int map_id, std::string & map_name)
{
  char sql_buffer[128];
  sqlite3_stmt * stmt{nullptr};
  snprintf(sql_buffer, sizeof(sql_buffer), "SELECT name FROM reloc_map WHERE id = %d;", map_id);
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  if (result != SQLITE_OK || stmt == nullptr) {
    LOG(ERROR) << "GetMapName fail";
    return SLAM_NULL_PTR;
  }
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    map_name = std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
  }
  sqlite3_finalize(stmt);
  return SLAM_OK;
}

int CyberDB::GetMaxMapId(int & max_map_id, const std::string table_name)
{
  int map_num = -1;
  GetMapNums(map_num);
  if (map_num == 0) {
    max_map_id = 0;
    return SLAM_OK;
  }

  char sql_buffer[128];
  snprintf(sql_buffer, sizeof(sql_buffer), "SELECT max(id) FROM %s ;", table_name.c_str());

  sqlite3_stmt * stmt = nullptr;
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  if (result != SQLITE_OK || stmt == nullptr) {
    LOG(ERROR) << "sqlite3_prepare_v2 fail";
    return SLAM_NULL_PTR;
  }

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    max_map_id = sqlite3_column_int(stmt, 0);
  }

  sqlite3_finalize(stmt);

  return SLAM_OK;
}

int CyberDB::WriteNewMap(
  const int & map_id, const std::string & map_name,
  const std::string & map_url, const std::string & table_name, bool slam_flag)
{
  int result = SLAM_OK;
  char sql_buffer[512];
  sqlite3_stmt * stmt{nullptr};
  float height(0.0), width(0.0);
  float version = 0.0;
  std::chrono::microseconds create_time = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  std::chrono::microseconds update_time = create_time;
  int status = 1;
  int scene = 1;
  std::string msg = "map " + std::to_string(map_id);
  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "INSERT OR IGNORE INTO %s VALUES (%d, '%s', %f, %f, '%s', %lf, %ld, %ld, %d, %d, %d, '%s');",
    table_name.c_str(),
    map_id,
    map_name.c_str(),
    height,
    width,
    map_url.c_str(),
    version,
    create_time.count(),
    update_time.count(),
    status,
    scene,
    static_cast<int>(slam_flag),
    msg.c_str());
  result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "cyberdog db WriteNewMap failed";
    return SLAM_ERROR;
  }

  LOG(ERROR) << "WriteNewMap success";

  return SLAM_OK;
}

int CyberDB::ReadMapInfo(const int & map_id, MapInfo & map_info)
{
  char sql_buffer[128];
  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "SELECT name, map_url, version, create_time, update_time, status, scene, \
slam FROM reloc_map WHERE id = %d;",
    map_id);
  sqlite3_stmt * stmt{nullptr};
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "cyberdog db ReadMapInfo prepare failed";
    return SLAM_ERROR;
  }

  if (sqlite3_step(stmt) != SQLITE_ROW) {
    LOG(ERROR) << "database query data no exsit";
    sqlite3_finalize(stmt);
    return SLAM_NULL_PTR;
  }
  map_info.map_name = std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
  map_info.map_url = std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)));
  map_info.version = static_cast<float>(sqlite3_column_double(stmt, 2));
  map_info.create_time = sqlite3_column_int64(stmt, 3);
  map_info.update_time = sqlite3_column_int64(stmt, 4);
  map_info.status = sqlite3_column_int(stmt, 5);
  map_info.scene = sqlite3_column_int(stmt, 6);
  map_info.slam_flag = sqlite3_column_int(stmt, 7);

  LOG(INFO) << "query map info:\n map_id: " << map_id << "\n map_name: " << map_info.map_name <<
    "\n map_url:" << map_info.map_url << "\n map_status : " << map_info.status;

  sqlite3_finalize(stmt);

  return SLAM_OK;
}

int CyberDB::GetMapNums(int & map_num)
{
  char sql_buffer[128];
  snprintf(sql_buffer, sizeof(sql_buffer), "SELECT count(id) FROM reloc_map;");

  sqlite3_stmt * stmt = nullptr;
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  if (result != SQLITE_OK || stmt == nullptr) {
    LOG(ERROR) << "GetMapNums sqlite3_prepare_v2 fail";
    return SLAM_NULL_PTR;
  }

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    map_num = sqlite3_column_int(stmt, 0);
  }

  sqlite3_finalize(stmt);

  return SLAM_OK;
}

int CyberDB::GetAllMapIds(std::vector<int> & map_ids)
{
  char sql_buffer[128];
  snprintf(sql_buffer, sizeof(sql_buffer), "SELECT id FROM reloc_map;");

  sqlite3_stmt * stmt = nullptr;
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  if (result != SQLITE_OK || stmt == nullptr) {
    LOG(ERROR) << "GetAllMapIds sqlite3_prepare_v2 fail";
    return SLAM_NULL_PTR;
  }

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    map_ids.emplace_back(sqlite3_column_int(stmt, 0));
  }
  sqlite3_finalize(stmt);
  return SLAM_OK;
}

int CyberDB::DeleteMap(const int map_id)
{
  char sql_buffer[128];
  snprintf(sql_buffer, sizeof(sql_buffer), "DELETE FROM reloc_map WHERE id = %d;", map_id);
  // std::cout << std::string(sql_buffer) << std::endl;
  sqlite3_stmt * stmt{nullptr};
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  if (result != SQLITE_OK) {
    return SLAM_ERROR;
  }

  return SLAM_OK;
}

int CyberDB::UpdateMap(const int map_id, const std::string & map_name)
{
  char sql_buffer[128];
  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "UPDATE reloc_map SET name = '%s' WHERE id = %d;", map_name.c_str(), map_id);
  sqlite3_stmt * stmt{nullptr};
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "UpdateMap ERROR";
    return SLAM_ERROR;
  }

  return SLAM_OK;
}

int CyberDB::QueryMaps(
  std::vector<std::string> & map_name_list, std::vector<int> & map_id_list,
  std::vector<int> & status_list, std::vector<std::string> & map_url_list)
{
  char sql_buffer[128];
  snprintf(sql_buffer, sizeof(sql_buffer), "SELECT id, name, status, map_url FROM reloc_map;");

  sqlite3_stmt * stmt = nullptr;
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  if (result != SQLITE_OK || stmt == nullptr) {
    LOG(ERROR) << "QueryMaps sqlite3_prepare_v2 fail";
    return SLAM_NULL_PTR;
  }

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    map_id_list.emplace_back(sqlite3_column_int(stmt, 0));
    std::string map_name = std::string(
      reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)));
    map_name_list.emplace_back(map_name);
    status_list.emplace_back(sqlite3_column_int(stmt, 2));
    std::string map_url = std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3)));
    map_url_list.emplace_back(map_url);
  }
  sqlite3_finalize(stmt);
  return SLAM_OK;
}

int SparseMapDB::CreateDbTables()
{
  int result = sqlite3_exec(m_database, CREATE_CAMERAS_TABLE, nullptr, nullptr, nullptr);
  result |= sqlite3_exec(m_database, CREATE_DESCRIPTORS_TABLE, nullptr, nullptr, nullptr);
  result |= sqlite3_exec(m_database, CREATE_SCORES_TABLE, nullptr, nullptr, nullptr);
  result |= sqlite3_exec(m_database, CREATE_IMAGES_TABLE, nullptr, nullptr, nullptr);
  result |= sqlite3_exec(m_database, CREATE_TWO_VIEW_GEOMETRIES_TABLE, nullptr, nullptr, nullptr);
  result |= sqlite3_exec(m_database, CREATE_KEYPOINTS_TABLE, nullptr, nullptr, nullptr);
  result |= sqlite3_exec(m_database, CREATE_MATCHES_TABLE, nullptr, nullptr, nullptr);
  result |= sqlite3_exec(m_database, CREATE_NAME_INDEX, nullptr, nullptr, nullptr);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "CreateDbTables failed";
    return SLAM_ERROR;
  }
  LOG(INFO) << "CreateDbTables success";

  return SLAM_OK;
}

int SparseMapDB::WriteCameras(const std::vector<colmap::Camera> & cameras)
{
  char sql_buffer[128];
  sqlite3_stmt * stmt{nullptr};

  int camera_num = cameras.size();
  int camera_param_size = cameras[0].NumParams() * 8;

  int result = sqlite3_exec(m_database, "BEGIN;", 0, 0, 0);
  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "INSERT OR IGNORE INTO cameras VALUES (?, ?, ?, ?, ?, 1);");
  result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);

  for (int i = 0; i < camera_num; i++) {
    camera_param_size = cameras[i].NumParams() * 8;
    const colmap::Camera & camera = cameras[i];
    sqlite3_reset(stmt);
    result |= sqlite3_bind_int(stmt, 1, static_cast<int>(camera.CameraId()));
    result |= sqlite3_bind_int(stmt, 2, camera.ModelId());
    result |= sqlite3_bind_int(stmt, 3, static_cast<int>(camera.Width()));
    result |= sqlite3_bind_int(stmt, 4, static_cast<int>(camera.Height()));
    result |= sqlite3_bind_blob(stmt, 5, camera.ParamsData(), camera_param_size, nullptr);
    sqlite3_step(stmt);
  }
  sqlite3_finalize(stmt);
  result |= sqlite3_exec(m_database, "COMMIT;", 0, 0, 0);
  if (result != SQLITE_OK) {
    LOG(ERROR) << "database db write cameras fail";
    return SLAM_ERROR;
  }

  LOG(INFO) << "write cameras success";

  return SLAM_OK;
}

int SparseMapDB::WriteImages(const std::vector<colmap::Image> & images, const bool use_image_id)
{
  int result = 0;
  char sql_buffer[256];
  sqlite3_stmt * stmt{nullptr};
  int image_num = images.size();

  if (use_image_id) {
    snprintf(
      sql_buffer, sizeof(sql_buffer),
      "INSERT OR IGNORE INTO images VALUES (?, ?, ?,  1., 0., 0., 0., 0., 0., 0., 1);");
  } else {
    snprintf(
      sql_buffer, sizeof(sql_buffer),
      "INSERT OR IGNORE INTO images VALUES (NULL, ?, ?,  1., 0., 0., 0., 0., 0., 0., 1);");
  }
  result |= sqlite3_exec(m_database, "BEGIN;", 0, 0, 0);
  result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  for (int i = 0; i < image_num; i++) {
    int bind_num = 1;
    const colmap::Image & image = images[i];
    sqlite3_reset(stmt);
    if (use_image_id) {
      result |= sqlite3_bind_int(stmt, bind_num, static_cast<int>(image.ImageId()));
      bind_num++;
    }
    result |= sqlite3_bind_text(stmt, bind_num, image.Name().c_str(), -1, nullptr);
    result |= sqlite3_bind_int(stmt, bind_num + 1, static_cast<int>(image.CameraId()));
    sqlite3_step(stmt);
  }
  sqlite3_finalize(stmt);
  result |= sqlite3_exec(m_database, "COMMIT;", 0, 0, 0);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "database db write images fail";
    return SLAM_ERROR;
  }

  LOG(INFO) << "write images success";

  return SLAM_OK;
}

int SparseMapDB::GetImageIds(std::unordered_map<std::string, int> & name_map)
{
  char sql_buffer[128];
  snprintf(sql_buffer, sizeof(sql_buffer), "SELECT image_id, name FROM images;");
  sqlite3_stmt * stmt = nullptr;
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  if (result != SQLITE_OK || nullptr == stmt) {
    LOG(ERROR) << "sqlite3_prepare_v2 fail.";
    return SLAM_NULL_PTR;
  }
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    uint32_t image_id = sqlite3_column_int(stmt, 0);
    std::string name = std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)));
    name_map.insert(std::make_pair(name, image_id));
  }
  sqlite3_finalize(stmt);
  stmt = nullptr;

  return SLAM_OK;
}

int SparseMapDB::ReadImages(std::vector<colmap::Image> & images)
{
  char sql_buffer[128];
  snprintf(sql_buffer, sizeof(sql_buffer), "SELECT * FROM images;");
  sqlite3_stmt * stmt = nullptr;
  int result = sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  if (result != SQLITE_OK || nullptr == stmt) {
    LOG(ERROR) << "sqlite3_prepare_v2 fail.";
    return SLAM_NULL_PTR;
  }
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    colmap::Image image;
    image.SetImageId(sqlite3_column_int(stmt, 0));
    image.SetName(std::string(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1))));
    image.SetCameraId(sqlite3_column_int(stmt, 2));
    images.push_back(image);
  }
  sqlite3_finalize(stmt);
  stmt = nullptr;

  LOG(INFO) << "read images success";

  return SLAM_OK;
}

int SparseMapDB::WriteMatchFeature_nocommit(
  const int64_t & pair_id, std::vector<std::array<int,
  2>> & match_pair)
{
  if (m_transaction_flag == false) {
    LOG(ERROR) << "m_transaction_flag error";
    return SLAM_ERROR;
  }

  int result = 0;
  char sql_buffer[128];

  if (m_stmt == nullptr) {
    snprintf(sql_buffer, sizeof(sql_buffer), "INSERT OR IGNORE INTO matches VALUES (?, ?, 2, ?);");
    result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &m_stmt, NULL);
  }

  int match_pair_num = match_pair.size();
  sqlite3_reset(m_stmt);
  sqlite3_clear_bindings(m_stmt);
  result |= sqlite3_bind_int64(m_stmt, 1, pair_id);
  result |= sqlite3_bind_int(m_stmt, 2, match_pair_num);

  if (match_pair_num > 0) {
    result |= sqlite3_bind_blob(m_stmt, 3, match_pair.data(), match_pair_num * 8, nullptr);
  }

  sqlite3_step(m_stmt);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "database db write match feature fail";
    return SLAM_ERROR;
  }
  return SLAM_OK;
}

int SparseMapDB::ReadLocalFeature(const int & image_id, LocalFeatureData & local_feature)
{
  local_feature.features.clear();
  local_feature.keypoints.clear();
  local_feature.scores.clear();
  local_feature.image_name.clear();

  LocalFeatureDataHalf local_feature_half;

  int result = 0;
  char sql_buffer[128];
  sqlite3_stmt * stmt{nullptr};
  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "SELECT * FROM keypoints where image_id = %d;", image_id);
  result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  sqlite3_step(stmt);
  size_t rows = sqlite3_column_int(stmt, 1);
  local_feature_half.keypoints.resize(rows);
  memcpy(
    local_feature_half.keypoints.data(), sqlite3_column_blob(stmt, 3),
    sqlite3_column_bytes(stmt, 3));
  sqlite3_finalize(stmt);

  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "SELECT * FROM descriptors where image_id = %d;", image_id);
  result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  sqlite3_step(stmt);
  rows = sqlite3_column_int(stmt, 1);
  size_t cols = sqlite3_column_int(stmt, 2);
  local_feature_half.features.resize(rows * cols);
  memcpy(
    local_feature_half.features.data(), sqlite3_column_blob(stmt, 3),
    sqlite3_column_bytes(stmt, 3));
  sqlite3_finalize(stmt);

  snprintf(
    sql_buffer, sizeof(sql_buffer),
    "SELECT * FROM scores where image_id = %d;", image_id);
  result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &stmt, NULL);
  sqlite3_step(stmt);
  rows = sqlite3_column_int(stmt, 1);
  local_feature_half.scores.resize(rows);
  memcpy(
    local_feature_half.scores.data(), sqlite3_column_blob(stmt, 3),
    sqlite3_column_bytes(stmt, 3));
  sqlite3_finalize(stmt);
  stmt = nullptr;

  result |= miloc_half_impl::LocalFeaturesHalfToFloat(local_feature, local_feature_half);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "ReadLocalFeature fail.";
    return SLAM_ERROR;
  }

  return SLAM_OK;
}

int SparseMapDB::WriteLocalFeature_nocommit(const int & image_id, LocalFeatureData & local_feature)
{
  if (m_transaction_flag == false) {
    LOG(ERROR) << "m_transaction_flag error";
    return SLAM_ERROR;
  }

  LocalFeatureDataHalf local_feature_half;
  miloc_half_impl::LocalFeaturesFloatToHalf(local_feature, local_feature_half);

  int result = 0;
  char sql_buffer[128];

  if (keypoint_stmt_ == nullptr) {
    snprintf(
      sql_buffer, sizeof(sql_buffer),
      "INSERT OR IGNORE INTO keypoints VALUES (?, ?, 2, ?);");
    result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &keypoint_stmt_, NULL);
  }
  if (des_stmt_ == nullptr) {
    snprintf(
      sql_buffer, sizeof(sql_buffer),
      "INSERT OR IGNORE INTO descriptors VALUES (?, ?, ?, ?);");
    result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &des_stmt_, NULL);
  }
  if (score_stmt_ == nullptr) {
    snprintf(
      sql_buffer, sizeof(sql_buffer),
      "INSERT OR IGNORE INTO scores VALUES (?, ?, 1, ?);");
    result |= sqlite3_prepare_v2(m_database, sql_buffer, -1, &score_stmt_, NULL);
  }

  sqlite3_reset(keypoint_stmt_);
  result |= sqlite3_bind_int(keypoint_stmt_, 1, image_id);
  result |= sqlite3_bind_int(keypoint_stmt_, 2, local_feature_half.keypoints.size());
  result |= sqlite3_bind_blob(
    keypoint_stmt_, 3,
    local_feature_half.keypoints.data(), local_feature_half.keypoints.size() * 8, nullptr);
  sqlite3_step(keypoint_stmt_);

  sqlite3_reset(des_stmt_);
  result |= sqlite3_bind_int(des_stmt_, 1, image_id);
  result |= sqlite3_bind_int(des_stmt_, 2, FEATURE_DIM);
  result |= sqlite3_bind_int(des_stmt_, 3, local_feature_half.features.size() / FEATURE_DIM);
  result |= sqlite3_bind_blob(
    des_stmt_, 4,
    local_feature_half.features.data(), local_feature_half.features.size() * 2, nullptr);
  sqlite3_step(des_stmt_);

  sqlite3_reset(score_stmt_);
  result |= sqlite3_bind_int(score_stmt_, 1, image_id);
  result |= sqlite3_bind_int(score_stmt_, 2, local_feature_half.scores.size());
  result |= sqlite3_bind_blob(
    score_stmt_, 3,
    local_feature_half.scores.data(), local_feature_half.scores.size() * 2, nullptr);
  sqlite3_step(score_stmt_);

  if (result != SQLITE_OK) {
    LOG(ERROR) << "WriteLocalFeature_nocommit fail.";
    return SLAM_ERROR;
  }

  return SLAM_OK;
}

int SparseMapDB::Commit()
{
  if (m_transaction_flag == false) {
    return SLAM_ERROR;
  }
  int result = sqlite3_exec(m_database, "COMMIT;", NULL, NULL, NULL);
  m_transaction_flag = false;
  if (m_stmt != nullptr) {
    sqlite3_finalize(m_stmt);
    m_stmt = nullptr;
  }
  if (keypoint_stmt_ != nullptr) {
    sqlite3_finalize(keypoint_stmt_);
    keypoint_stmt_ = nullptr;
  }
  if (des_stmt_ != nullptr) {
    sqlite3_finalize(des_stmt_);
    des_stmt_ = nullptr;
  }
  if (score_stmt_ != nullptr) {
    sqlite3_finalize(score_stmt_);
    score_stmt_ = nullptr;
  }

  if (result != SQLITE_OK) {
    LOG(ERROR) << "commit transaction failed ";
    return SLAM_ERROR;
  }

  LOG(INFO) << "commit transaction success";

  return SLAM_OK;
}
SparseMapDB::~SparseMapDB()
{
  if (keypoint_stmt_ != nullptr) {
    sqlite3_finalize(keypoint_stmt_);
    keypoint_stmt_ = nullptr;
  }
  if (des_stmt_ != nullptr) {
    sqlite3_finalize(des_stmt_);
    des_stmt_ = nullptr;
  }
  if (score_stmt_ != nullptr) {
    sqlite3_finalize(score_stmt_);
    score_stmt_ = nullptr;
  }
}

}  // namespace miloc
}  // namespace cyberdog
